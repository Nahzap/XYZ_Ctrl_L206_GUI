"""
Autofocus Service - Servicio de Autoenfoque Asíncrono
=====================================================

Worker para ejecutar Z-scanning en background sin bloquear la UI.
Emite señales de progreso para visualización en tiempo real.

Autor: Sistema de Control L206
Fecha: 2025-12-12
"""

import math
import time
import logging
import numpy as np
import cv2

from typing import List, Tuple, Optional, Callable

from PyQt5.QtCore import QThread, pyqtSignal

from core.models.detected_object import DetectedObject
from core.models.focus_result import AutofocusResult
from core.autofocus.focus_metric import (
    bbox_to_contour,
    calculate_focus_score_detailed,
)
from core.autofocus.af_kpi import AfCycleKpi, AfSessionKpi
from core.autofocus.bpof_candidates import (
    BpofCandidateTable,
    build_fine_z_planes,
)
from core.autofocus.fine_scan_plan import (
    RingDeclineStop,
    center_out_sequence,
    fine_span_um,
    ring_counts,
)
from core.autofocus.roi_tracker import RoiTracker
from core.autofocus.stack_plan import (
    rebalance_symmetric,
    stack_asymmetry_ratio,
)
from core.autofocus.z_prior import BpofPrior, bootstrap_window

logger = logging.getLogger('MotorControl_L206')

# Alias para compatibilidad
FocusResult = AutofocusResult


class AutofocusService(QThread):
    """
    Servicio de autofoco asíncrono.
    
    Realiza Z-scanning para cada objeto detectado y emite progreso
    en tiempo real para actualización de UI.
    
    Signals:
        scan_started: Emitido al iniciar escaneo de un objeto
        z_changed: Emitido en cada posición Z evaluada (z, score, roi_frame)
        object_focused: Emitido cuando se encuentra el foco óptimo de un objeto
        scan_complete: Emitido cuando termina todo el proceso
        error_occurred: Emitido si hay un error
    """
    
    # Señales
    scan_started = pyqtSignal(int, int)  # object_index, total_objects
    z_changed = pyqtSignal(float, float, np.ndarray)  # z_position, score, roi_frame
    object_focused = pyqtSignal(int, float, float)  # object_index, z_optimal, score
    scan_complete = pyqtSignal(list)  # List[FocusResult]
    error_occurred = pyqtSignal(str)
    masks_detected = pyqtSignal(list)  # Máscaras/ROIs detectados para visualización
    # ROIs vigentes del tracker durante el Z-scan: (lista de dicts, (w, h) frame)
    roi_tracked = pyqtSignal(list, tuple)
    status_message = pyqtSignal(str)  # Mensajes de estado para UI y terminal
    score_updated = pyqtSignal(float, float)  # (z_position, score) para overlay en cámara
    progress_updated = pyqtSignal(int, int, str)  # (current_step, total_steps, phase_name)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Referencias a hardware (se configuran externamente)
        self.cfocus_controller = None
        self.get_frame_callback: Callable[[], np.ndarray] = None
        self.get_frame_count_callback: Optional[Callable[[], int]] = None
        self.get_exposure_s_callback: Optional[Callable[[], float]] = None
        # Única vía CMOS: CameraService/worker.acquire_scientific_frame
        self.acquire_scientific_frame_callback: Optional[Callable] = None
        # Legacy (ignorados si existe acquire_scientific_frame_callback)
        self.request_raw_frame_callback: Optional[Callable[[], None]] = None
        self.get_raw_frame_callback: Optional[Callable[[], np.ndarray]] = None
        self.get_raw_frame_count_callback: Optional[Callable[[], int]] = None
        self.get_raw_pixel_format_callback: Optional[Callable[[], str]] = None
        self._native_frame_hw: Optional[Tuple[int, int]] = None
        self._scientific_frame_id: int = 0
        
        # Objetos a enfocar
        self.objects_to_focus: List[DetectedObject] = []
        
        # Parámetros de búsqueda (configurables desde UI)
        # NOTA: Estos parámetros son para BÚSQUEDA de BPoF, NO para captura de volumen
        #
        # FINE es un refinamiento del plano que ya eligió el COARSE, no un
        # segundo barrido. Con Δ=6µm, paso 0.5µm y 9 capas la ventana real es
        # ±2µm: suficiente para decidir entre planos contiguos del COARSE.
        # (39 capas × 1µm daban ±19µm, casi el recorrido completo del piezo, y
        # 26 de esos planos medían fondo plano.)
        self.z_scan_range = 6.0        # µm - límite ±Δ fine alrededor del max S coarse
        self.use_full_range = True     # Si True, escanea todo el rango calibrado; si False, usa z_scan_range
        self.z_step_coarse = 3.0       # µm - paso grueso; localiza el pico sin saltárselo
        self.z_step_fine = 0.5         # µm - paso real FINE entre candidatos
        self.n_fine_planes = 9         # capas FINE impares, centradas en Z_c*
        self.refine_window = 2.0       # legacy; la zona fine usa z_scan_range
        # Condición de llegada a Z (NO sleep fijo de settle). Debe quedar por
        # debajo de la mitad del paso fino: con tol ≥ paso, dos candidatos FINE
        # distintos pueden medirse en la misma Z real y la curva miente.
        self.z_arrive_tol_um = 0.25    # µm - |Z_read−Z_cmd| ≤ tol
        self.z_arrive_stable_reads = 3 # lecturas consecutivas en banda
        self.z_arrive_timeout_s = 3.0  # s - timeout de seguridad (no es settle)
        # Compat: UI antigua / config; ya no se usa como sleep de asentamiento
        self.settle_time = 0.0
        self.capture_settle_time = 0.0
        self.roi_margin = 20    # px - margen ROI (JSON/UI)
        # Condición óptica tras Z en banda (exposición + cola; no sleep fijo)
        self.score_flush_frames = 3
        self.score_min_fresh_s = 0.12
        self.score_fps_est = 14.0
        self.score_samples_per_plane = 1  # métrica robusta v4: una toma por Z
        self.score_stable_rel = 0.08      # spread/mediana ≤ 8%
        self.score_stable_abs = 2.0       # o spread ≤ 2 puntos S
        self.score_stable_max_tries = 2   # segundo intento solo si frame inválido
        # Si al revalidar BPoF la S cae más que esto vs FINE, no fiarse del pico
        self.bpof_confirm_max_drop_rel = 0.08
        # Ventaja mínima que debe sacar un vecino ±paso fino para desbancar al
        # pico FINE en la confirmación (una toma suelta no vence a N mediciones)
        self.bpof_confirm_neighbor_margin_rel = 0.03
        self.coarse_near_max_rel = 0.005
        self.coarse_early_stop_patience = 4
        self.coarse_early_stop_drop_rel = 0.03
        self.bpof_min_relative_span = 0.005
        self.bpof_min_prominence_rel = 0.003
        # Corte temprano FINE, simétrico al del COARSE. Se decide por anillos
        # (pares de planos equidistantes) para que no dependa de qué lado se
        # visitó primero.
        self.fine_early_stop_patience_rings = 3
        self.fine_early_stop_drop_rel = 0.05
        # Un plano cuya S se hunde respecto a sus dos vecinos no es óptica: se
        # re-mide, y si insiste no vota (ver find_isolated_dips).
        self.fine_dip_reject_factor = 0.85
        # Mediana de N tomas SOLO en los planos que deciden Z (ancla FINE y
        # BPoF candidato). Pagar 2 tomas en los 9 planos no compra nada.
        self.score_samples_at_decision = 2
        # Sondear vecinos del BPoF en la confirmación sólo si el pico FINE es
        # sospechoso. Con un pico interior y prominente, 3 mediciones extra no
        # pueden desmentir a las N del barrido (ver bpof_confirm_neighbor_margin_rel).
        self.bpof_confirm_probe_neighbors_when_healthy = False
        # COARSE acotado por el historial de BPoF de la sesión. El primer punto
        # de una muestra barre lo que pida la interfaz; a partir de
        # coarse_prior_min_samples la ventana se centra en la mediana medida.
        self.coarse_prior_enabled = True
        self.coarse_prior_min_samples = 5
        self.coarse_prior_min_half_span_um = 4.0
        self.coarse_prior_mad_k = 2.0
        # Ventana de arranque cuando no hay historial y el usuario NO pidió
        # escaneo completo: asimétrica porque el pico aparece pasado el origen.
        self.coarse_bootstrap_below_um = 20.0
        self.coarse_bootstrap_above_um = 25.0
        # Asimetría tolerada del stack antes de reflejar la distancia menor.
        self.stack_asymmetry_max = 3.0
        
        # Límites de iteraciones para evitar bucles infinitos
        self.max_coarse_iterations = 50  # Máximo de iteraciones en fase gruesa
        self.max_fine_iterations = 101   # Máximo impar; coincide con límite GUI
        
        # Multi-focal: valores placeholder; la fuente de verdad es camera_tab JSON → UI → sync
        self.n_captures = 3
        self.capture_step = 2.0
        self.z_step_capture = 2.0
        self.capture_s_drop_rel = 0.10
        self.optical_search_refine_iterations = 4
        self._last_focus_curve: list = []

        # Historial de BPoF de la sesión: convierte cada ciclo en información
        # para el siguiente en vez de re-descubrir el plano focal 5292 veces.
        self._bpof_prior = BpofPrior(
            min_samples=self.coarse_prior_min_samples,
            min_half_span_um=self.coarse_prior_min_half_span_um,
            mad_k=self.coarse_prior_mad_k,
        )
        # KPI: una línea por ciclo + medianas de sesión para el ETA.
        self.session_kpi = AfSessionKpi()
        self._cycle_kpi: Optional[AfCycleKpi] = None
        self.kpi_session_log_every = 25

        # Al terminar el ciclo, dejar el piezo en el BPoF en vez de volver al
        # punto medio calibrado.
        self.park_at_bpof = True
        
        # ROI Tracking adaptativo durante Z-scan
        self.roi_tracking_enabled = True       # habilitar/deshabilitar tracking
        self.roi_max_drift_px = 30             # drift máximo acumulado (px)
        self.roi_update_threshold_px = 2.0     # drift mínimo para actualizar
        # Inferencias U2-Net por barrido con ventana estática: la máscara de
        # medida no se mueve, así que detectar en cada plano sólo repetía el
        # chequeo de contención a coste de una inferencia de 5 Mpx.
        self.roi_detect_interval = 8
        # S se mide sobre un único ROI cuadrado estático por objeto, holgado
        # (roi_margin por lado) para que la segmentación quepa dentro en todos
        # los planos. La segmentación es la variable del problema: si la
        # ventana la persigue o la copia, cada plano mide píxeles distintos y
        # las S dejan de ser comparables. El contorno vivo se sigue viendo en
        # el overlay vía tracker.display_rois.
        self.roi_static_window = True
        self.roi_track_adopt_contour = False
        self._roi_tracker: Optional[RoiTracker] = None  # instancia activa
        self._last_roi_emit_t = 0.0
        self._roi_emit_interval_s = 0.2        # overlay de ROI a ~5 Hz
        
        # Control
        self.running = False
        self.cancel_requested = False
        # True solo durante autofoco disparado por MicroscopyService (captura multi-focal)
        self.microscopy_mode = False
        
        logger.info("[AutofocusService] Inicializado")
    
    def configure(
        self,
        cfocus_controller,
        get_frame_callback: Callable = None,
        get_frame_count_callback: Optional[Callable[[], int]] = None,
        get_exposure_s_callback: Optional[Callable[[], float]] = None,
        acquire_scientific_frame_callback: Optional[Callable] = None,
        request_raw_frame_callback: Optional[Callable[[], None]] = None,
        get_raw_frame_callback: Optional[Callable[[], np.ndarray]] = None,
        get_raw_frame_count_callback: Optional[Callable[[], int]] = None,
        get_raw_pixel_format_callback: Optional[Callable[[], str]] = None,
    ):
        """
        Configura el servicio con referencias a hardware.

        ``acquire_scientific_frame_callback`` es la única vía CMOS admitida.
        """
        self.cfocus_controller = cfocus_controller
        self.get_frame_callback = get_frame_callback
        self.get_frame_count_callback = get_frame_count_callback
        self.get_exposure_s_callback = get_exposure_s_callback
        self.acquire_scientific_frame_callback = acquire_scientific_frame_callback
        self.request_raw_frame_callback = request_raw_frame_callback
        self.get_raw_frame_callback = get_raw_frame_callback
        self.get_raw_frame_count_callback = get_raw_frame_count_callback
        self.get_raw_pixel_format_callback = get_raw_pixel_format_callback
        self._native_frame_hw = None
        self._scientific_frame_id = 0
        if acquire_scientific_frame_callback is None:
            raise RuntimeError(
                "[AutofocusService] require acquire_scientific_frame_callback "
                "(única vía CMOS; no se admite preview/current_frame)"
            )
        try:
            sci = acquire_scientific_frame_callback(timeout_s=2.0)
            img = getattr(sci, "image16", None)
            if img is not None and getattr(img, "size", 0) > 0:
                self._native_frame_hw = (int(img.shape[0]), int(img.shape[1]))
                self._scientific_frame_id = int(getattr(sci, "frame_id", 0) or 0)
        except Exception as exc:
            logger.warning(
                "[AutofocusService] Probe inicial acquire_scientific_frame: %s",
                exc,
            )
            self._native_frame_hw = None
        logger.info(
            "[AutofocusService] Configurado con C-Focus | CMOS única vía "
            "acquire_scientific_frame | nativo=%s | pipeline=scientific_bgr16_v1",
            (
                f"{self._native_frame_hw[1]}x{self._native_frame_hw[0]}"
                if self._native_frame_hw is not None
                else "pendiente"
            ),
        )
    
    def get_search_info(self) -> dict:
        """Retorna información sobre los parámetros de búsqueda de autofoco.
        
        NOTA: Autofoco NO captura volúmenes. Usa algoritmo de búsqueda (hill climbing)
        con pasos heterogéneos (coarse → fine) para encontrar 1 posición óptima (BPoF).
        El número de evaluaciones depende del algoritmo, no es predecible.
        
        Para captura de volúmenes con pasos homogéneos, usar VolumetryService (Z-Stack).
        
        Returns:
            dict con parámetros de búsqueda
        """
        return {
            'scan_range_um': self.z_scan_range,
            'z_step_coarse': self.z_step_coarse,
            'z_step_fine': self.z_step_fine,
            'search_distance_um': 2 * self.z_scan_range,
            'algorithm': 'coarse_fine_bpof',
            # Coste real por medición S (MOVE + Z_STATIC + flush óptico + RAW
            # 2590×1942 + métrica). Medido, no estimado: el ciclo de referencia
            # hizo 68 mediciones en 62.4 s.
            's_per_plane_s': self.__dict__.get(
                'measured_s_per_plane_s',
                self.session_kpi.median_of('s_per_plane')
                if 'session_kpi' in self.__dict__
                else None,
            ),
        }
    
    def validate_scan_range(self) -> Tuple[bool, str]:
        """
        Valida que el C-Focus esté calibrado para escaneo completo.
        El autofoco ahora escanea TODO el rango calibrado (z_min a z_max).
        
        Returns:
            (is_valid, message)
        """
        if not self.cfocus_controller:
            return False, "C-Focus no conectado"
        
        calib_info = self.cfocus_controller.get_calibration_info()
        if not calib_info['is_calibrated']:
            return False, "C-Focus no calibrado. Ejecutar calibración primero."
        
        z_min_hw = calib_info['z_min']
        z_max_hw = calib_info['z_max']
        z_range = z_max_hw - z_min_hw
        
        # Validar que el rango calibrado sea razonable
        if z_range < 10.0:
            return False, f"Rango calibrado muy pequeño ({z_range:.2f}µm). Re-calibrar C-Focus."
        
        return True, f"Rango de escaneo: {z_min_hw:.2f} - {z_max_hw:.2f}µm ({z_range:.2f}µm total)"

    def _has_live_frame(self) -> bool:
        """Comprueba adquisición CMOS por la única vía científica."""
        acquire = self.__dict__.get("acquire_scientific_frame_callback")
        if not callable(acquire):
            return False
        try:
            sci = acquire(timeout_s=1.0)
        except Exception:
            return False
        img = getattr(sci, "image16", None)
        return img is not None and getattr(img, "size", 0) > 0

    def validate_can_run(self) -> Tuple[bool, str]:
        """Valida que el autofoco puede ejecutarse (manual o por algoritmo)."""
        if self.isRunning():
            return False, "Ya hay un escaneo de autofoco en progreso"
        if not self.cfocus_controller:
            return False, "C-Focus no configurado"
        if not callable(self.__dict__.get("acquire_scientific_frame_callback")):
            return False, (
                "Cámara no configurada (falta acquire_scientific_frame; "
                "conecta y calibra C-Focus)"
            )
        if not self._has_live_frame():
            return False, "No hay transmisión de cámara activa (inicia vista en vivo)"
        is_valid, msg = self.validate_scan_range()
        if not is_valid:
            return False, msg
        return True, "OK"

    def start_autofocus(self, objects: List[DetectedObject]) -> bool:
        """
        Inicia el proceso de autofoco para una lista de objetos (hilo propio).

        Debe invocarse solo desde la UI (manual) o desde MicroscopyService
        cuando el algoritmo detecta un objeto y el autofoco está habilitado.

        Args:
            objects: Lista de objetos detectados a enfocar

        Returns:
            True si el escaneo se inició correctamente.
        """
        can_run, msg = self.validate_can_run()
        if not can_run:
            logger.warning("[AutofocusService] No se puede iniciar autofoco: %s", msg)
            self.error_occurred.emit(msg)
            return False

        self.objects_to_focus = objects
        self.cancel_requested = False
        self.running = True
        self.start()
        return True
    
    def cancel(self):
        """Cancela el escaneo en progreso (lo antes posible entre pasos Z)."""
        self.cancel_requested = True
        self.running = False
        self.microscopy_mode = False
        logger.info("[AutofocusService] Cancelación solicitada (hard)")

    def prepare_new_session(self) -> None:
        """Limpia el latch de cancelación dejado por un hard-stop anterior.

        También descarta el prior de Z y las medianas: una sesión nueva es otra
        muestra, con otro plano focal y otra altura de portaobjetos. Arrastrar
        el historial anterior acotaría el COARSE alrededor de un plano que ya
        no existe.
        """
        self.cancel_requested = False
        self.running = False
        prior = self.__dict__.get("_bpof_prior")
        if prior is not None:
            prior.clear()
        session = self.__dict__.get("session_kpi")
        if session is not None:
            session.clear()

    def _sleep_interruptible(self, seconds: float) -> bool:
        """Sleep que respeta cancel_requested. True=completó, False=cancelado."""
        if seconds <= 0:
            return not self.cancel_requested
        end = time.perf_counter() + float(seconds)
        while time.perf_counter() < end:
            if self.cancel_requested:
                return False
            time.sleep(min(0.01, max(0.0, end - time.perf_counter())))
        return not self.cancel_requested

    def _command_z(self, z: float) -> bool:
        """FASE MOVE: consigna Z. No mide S ni lee frames."""
        if self.cancel_requested or self.cfocus_controller is None:
            return False
        prev_settle = getattr(self.cfocus_controller, "settle_time", 0.0)
        try:
            # Nunca sleep oculto en el driver: la quietud la impone este servicio
            self.cfocus_controller.settle_time = 0.0
            return bool(self.cfocus_controller.move_z(float(z)))
        finally:
            self.cfocus_controller.settle_time = prev_settle

    def _z_is_static(
        self,
        z_cmd: float,
        *,
        tol_um: Optional[float] = None,
    ) -> Tuple[bool, Optional[float]]:
        """True si |Z_read−Z_cmd|≤tol en ESTA lectura (piezo quieto ahora)."""
        tol = float(self.z_arrive_tol_um if tol_um is None else tol_um)
        last = self._read_z_um()
        if last is None:
            return False, None
        return abs(float(last) - float(z_cmd)) <= tol, float(last)

    def _wait_z_reached(
        self,
        z_cmd: float,
        *,
        tol_um: Optional[float] = None,
        stable_reads: Optional[int] = None,
        timeout_s: Optional[float] = None,
    ) -> Tuple[bool, Optional[float]]:
        """
        FASE Z_STATIC: |Z_read − Z_cmd| ≤ tol durante N lecturas consecutivas.

        Hasta cumplir esto está PROHIBIDO medir S (habría movimiento).
        """
        tol = float(
            self.z_arrive_tol_um if tol_um is None else tol_um
        )
        n_ok = max(1, int(
            self.z_arrive_stable_reads if stable_reads is None else stable_reads
        ))
        timeout = float(
            self.z_arrive_timeout_s if timeout_s is None else timeout_s
        )
        z_cmd = float(z_cmd)
        deadline = time.perf_counter() + max(0.05, timeout)
        streak = 0
        last: Optional[float] = None
        while time.perf_counter() < deadline:
            if self.cancel_requested:
                return False, last
            ok_now, last = self._z_is_static(z_cmd, tol_um=tol)
            if ok_now:
                streak += 1
                if streak >= n_ok:
                    return True, float(last) if last is not None else None
            else:
                streak = 0
            if not self._sleep_interruptible(0.005):
                return False, last
        logger.warning(
            "[Autofocus] Timeout Z_STATIC: cmd=%.2f last=%s tol=±%.2fµm",
            z_cmd,
            f"{last:.2f}" if last is not None else "?",
            tol,
        )
        return False, last

    def _move_z_interruptible(self, z: float, settle_s: Optional[float] = None) -> bool:
        """Mueve Z y espera Z_STATIC (settle_s ignorado; compat)."""
        del settle_s
        if not self._command_z(z):
            return False
        ok, _ = self._wait_z_reached(float(z))
        return ok

    def _goto_z_static(self, z_cmd: float) -> Tuple[bool, Optional[float]]:
        """MOVE + Z_STATIC (con un reintento). Sin frames / sin S."""
        z_cmd = float(z_cmd)
        if not self._command_z(z_cmd):
            return False, None
        ok, z_read = self._wait_z_reached(z_cmd)
        if ok:
            return True, z_read
        logger.warning(
            "[Autofocus] Z_STATIC falló cmd=%.2f read=%s — reintento MOVE",
            z_cmd,
            f"{z_read:.2f}" if z_read is not None else "?",
        )
        if not self._command_z(z_cmd):
            return False, z_read
        return self._wait_z_reached(z_cmd)

    def _normalize_rois(
        self,
        bbox=None,
        contour=None,
        rois=None,
    ) -> List[Tuple[Tuple[int, int, int, int], np.ndarray]]:
        """Lista [(bbox, contour), ...] — 1 ROI o superficie multi-ROI."""
        if rois:
            out = []
            for item in rois:
                if hasattr(item, "bounding_box"):
                    b = item.bounding_box
                    c = getattr(item, "contour", None)
                else:
                    b, c = item[0], item[1] if len(item) > 1 else None
                if c is None:
                    c = bbox_to_contour(b)
                out.append((b, c))
            return out
        if bbox is None:
            raise ValueError("Se requiere bbox o rois")
        if contour is None:
            contour = bbox_to_contour(bbox)
        return [(bbox, contour)]

    def _score_rois_on_frame(
        self,
        frame: np.ndarray,
        rois: List[Tuple[Tuple[int, int, int, int], np.ndarray]],
    ) -> float:
        """S superficie = Σ S_i (un frame estático, varios ROI).

        Con RoiTracker activo, los ROI que devuelve el tracker son la fuente de
        verdad: mantiene su propio estado entre planos y ``rois`` sólo actúa
        como semilla en la primera medición. Antes se reasignaba una variable
        local y el seguimiento se perdía en cada plano.

        S se mide sobre la máscara de área constante y el overlay recibe la
        silueta detectada: son dos vistas del mismo ROI y confundirlas hacía
        que S dependiera del tamaño que el detector le diera al objeto.
        """
        tracker = self._roi_tracker
        if tracker is not None and tracker.enabled:
            rois = tracker.update(frame, rois)
            self._emit_tracked_rois(frame, tracker.display_rois or rois)

        total = 0.0
        for bbox, contour in rois:
            total += float(self._calculate_sharpness(frame, bbox, contour))
        return total

    def _emit_tracked_rois(
        self,
        frame: np.ndarray,
        rois: List[Tuple[Tuple[int, int, int, int], np.ndarray]],
    ) -> None:
        """Publica los ROI vigentes para el overlay de cámara (throttled)."""
        if not rois or frame is None:
            return
        now = time.perf_counter()
        if now - self._last_roi_emit_t < self._roi_emit_interval_s:
            return
        self._last_roi_emit_t = now

        payload = [
            {
                "bbox": tuple(int(v) for v in bbox),
                "contour": (
                    None if contour is None
                    else np.asarray(contour, dtype=np.int32).copy()
                ),
            }
            for bbox, contour in rois
        ]
        try:
            self.roi_tracked.emit(
                payload, (int(frame.shape[1]), int(frame.shape[0]))
            )
        except (AttributeError, RuntimeError):
            pass

    def evaluate_s_at_z(
        self,
        z: float,
        bbox: Tuple[int, int, int, int] = None,
        contour=None,
        *,
        rois=None,
        settle_s: Optional[float] = None,
        return_frame: bool = False,
    ):
        """Única puerta de medición S. Orquestación síncrona anti-carrera.

        Pipeline: MOVE → Z_STATIC → OPTICAL → MEASURE (frame estático).
        Con varios ROI: S = Σ S_i sobre el mismo frame (1 superficie).
        ``settle_s`` se ignora (compat API).
        """
        del settle_s
        # Embudo único de medición: contar aquí es lo que hace comparable el
        # coste de dos algoritmos distintos (N_S del KPI).
        kpi = self.__dict__.get("_cycle_kpi")
        if kpi is not None:
            kpi.n_s_total += 1
        rois_n = self._normalize_rois(bbox, contour, rois)
        return self._measure_s_static_at_z(
            float(z), rois_n, return_frame=return_frame
        )

    def _measure_s_static_at_z(
        self,
        z_cmd: float,
        rois: List[Tuple[Tuple[int, int, int, int], np.ndarray]],
        *,
        return_frame: bool = False,
    ):
        """Implementación del contrato MOVE→Z_STATIC→OPTICAL→MEASURE."""
        z_cmd = float(z_cmd)

        ok, z_read = self._goto_z_static(z_cmd)
        if not ok:
            return (0.0, None) if return_frame else 0.0

        if not self._wait_optical_static(z_cmd):
            # No abortar el AF: si el piezo sigue en banda, medir de todos modos.
            ok_z, z_now = self._z_is_static(z_cmd)
            if not ok_z:
                logger.warning(
                    "[Autofocus] OPTICAL/Z_STATIC falló en Z=%.2fµm (read=%s)",
                    z_cmd,
                    f"{z_read:.2f}" if z_read is not None else "?",
                )
                return (0.0, None) if return_frame else 0.0
            logger.warning(
                "[Autofocus] OPTICAL timeout en Z=%.2fµm pero Z estático "
                "(read=%s) — se mide igualmente (AF no se interrumpe)",
                z_cmd,
                f"{z_now:.2f}" if z_now is not None else "?",
            )

        score, frame = self._measure_s_while_z_static(z_cmd, rois)
        if return_frame:
            return score, frame
        return score
    
    def run(self):
        """Un solo barrido Z: todos los ROI como una superficie (S=ΣS_i)."""
        objects = list(self.objects_to_focus or [])
        n_obj = len(objects)
        results: List[FocusResult] = []

        logger.info(
            "[AutofocusService] Autofoco superficie: %d ROI → 1 barrido Z",
            n_obj,
        )
        # Un job de escaneo (no N focos independientes)
        self.scan_started.emit(0, 1)

        try:
            if self.cancel_requested:
                raise RuntimeError("Cancelado")
            if self.microscopy_mode and n_obj == 1:
                results = [self._scan_single_object(objects[0], 0)]
            else:
                results = self._focus_surface_sync(objects)
            for i, result in enumerate(results):
                self.object_focused.emit(i, result.z_optimal, result.focus_score)
        except Exception as e:
            logger.error("[AutofocusService] Error: %s", e)
            self.error_occurred.emit(f"Error autofoco: {e}")
            try:
                ok_home, z_home, z_cmd = self.goto_calibration_origin(
                    log_prefix="[AutofocusService]", emit_status=True
                )
                if not ok_home:
                    logger.error(
                        "[AutofocusService] Retorno de seguridad falló: "
                        "Z_cmd=%.3f Z_read=%s",
                        z_cmd,
                        f"{z_home:.3f}" if z_home is not None else "?",
                    )
            except Exception as home_exc:
                logger.error(
                    "[AutofocusService] Error en retorno de seguridad: %s",
                    home_exc,
                )

        # El tracker no debe sobrevivir al barrido: un scan abortado dejaría
        # ROIs de la sesión anterior en la siguiente medición de S.
        if self._roi_tracker is not None:
            self._roi_tracker.reset()
            self._roi_tracker = None

        self.running = False
        self.scan_complete.emit(results)
        logger.info(
            "[AutofocusService] Completado: %d resultado(s), %d ROI",
            len(results),
            n_obj,
        )
    
    def _emit_candidate_table(
        self,
        table: BpofCandidateTable,
        title: str,
        log_prefix: str,
    ) -> None:
        """Vuelca UNA tabla de candidatos (coarse o fine) a logger/stdout/UI."""
        dump = table.format_dump(title=f"{log_prefix} {title}")
        logger.info("\n%s", dump)
        print(dump, flush=True)
        self.status_message.emit(dump)

    def _exposure_s(self) -> float:
        """Exposición actual (s) para invalidar frames mid-move."""
        cb = getattr(self, "get_exposure_s_callback", None)
        if cb is not None:
            try:
                exp = float(cb())
                if exp > 0:
                    return exp
            except Exception:
                pass
        return max(0.05, float(getattr(self, "score_min_fresh_s", 0.12)))

    def _score_flush_frames_needed(self) -> int:
        """Frames a descartar tras Z en banda (exposición + cola LatestImageOnly)."""
        base = max(1, int(getattr(self, "score_flush_frames", 3)))
        exp_s = self._exposure_s()
        fps = max(1.0, float(getattr(self, "score_fps_est", 14.0)))
        # +3: cola LatestImageOnly + frame a medio exponer tras MOVE (50ms→~4).
        from_exp = int(math.ceil(exp_s * fps)) + 3
        return max(base, from_exp)

    def _wait_optical_static(self, z_cmd: float) -> bool:
        """
        FASE OPTICAL: descartar frames expuestos durante el MOVE.

        Síncrono: entre frames se re-verifica Z_STATIC. Si el piezo se mueve,
        se aborta (no se mide en movimiento).
        """
        n_discard = self._score_flush_frames_needed()
        if n_discard <= 0:
            ok, _ = self._z_is_static(z_cmd)
            return ok

        if not self.get_frame_count_callback:
            raise RuntimeError(
                "AutofocusService sin get_frame_count_callback: "
                "inicializar autofoco con cámara en vivo"
            )

        start = int(self.get_frame_count_callback())
        target = start + int(n_discard)
        deadline = time.perf_counter() + max(2.0, self._exposure_s() * 8.0)

        while True:
            if self.cancel_requested:
                return False
            ok_z, _ = self._z_is_static(z_cmd)
            if not ok_z:
                logger.warning(
                    "[Autofocus] OPTICAL abortado: Z salió de banda durante "
                    "descarte de frames (cmd=%.2fµm) — no medir en movimiento",
                    float(z_cmd),
                )
                return False
            current = int(self.get_frame_count_callback())
            if current >= target:
                # Último chequeo: sigue quieto justo al salir de OPTICAL
                ok_z, _ = self._z_is_static(z_cmd)
                return ok_z
            if time.perf_counter() >= deadline:
                logger.warning(
                    "[Autofocus] Timeout OPTICAL (start=%d now=%d target=%d)",
                    start,
                    current,
                    target,
                )
                return False
            if not self._sleep_interruptible(0.01):
                return False

    def _measure_s_while_z_static(
        self,
        z_cmd: float,
        rois: List[Tuple[Tuple[int, int, int, int], np.ndarray]],
    ) -> Tuple[float, Optional[np.ndarray]]:
        """
        FASE MEASURE: S = Σ S_i sobre frame RAW nuevo y Z estático.

        CLAHE-HF-v4 ya rechaza ruido mediante gradientes robustos; por defecto
        usa una toma por plano. ``score_samples_per_plane`` permite mediana N
        para auditorías lentas.
        """
        min_samples = max(1, int(getattr(self, "score_samples_per_plane", 1)))
        max_tries = max(
            min_samples, int(getattr(self, "score_stable_max_tries", 2))
        )
        samples: List[Tuple[float, np.ndarray]] = []

        for attempt in range(max_tries):
            if self.cancel_requested:
                break
            ok_z, _ = self._z_is_static(z_cmd)
            if not ok_z:
                if attempt == 0 and self._goto_z_static(z_cmd)[0]:
                    if not self._wait_optical_static(z_cmd):
                        break
                    continue
                logger.warning(
                    "[Autofocus] MEASURE: Z no estático en Z=%.2fµm",
                    float(z_cmd),
                )
                break

            frame = self._get_fresh_frame(n_new=1)
            if frame is None or getattr(frame, "size", 0) == 0:
                continue

            ok_z, _ = self._z_is_static(z_cmd)
            if not ok_z:
                logger.warning(
                    "[Autofocus] MEASURE: frame descartado — Z se movió "
                    "(cmd=%.2fµm)",
                    float(z_cmd),
                )
                continue

            s = self._score_rois_on_frame(frame, rois)
            if not np.isfinite(s) or s <= 0.0:
                continue
            samples.append((float(s), frame))

            if len(samples) < min_samples:
                continue

            recent = np.asarray(
                [item[0] for item in samples[-min_samples:]], dtype=np.float64
            )
            med = float(np.median(recent))
            spread = float(np.max(recent) - np.min(recent))
            stable_limit = max(
                float(getattr(self, "score_stable_abs", 2.0)),
                abs(med) * float(getattr(self, "score_stable_rel", 0.08)),
            )
            if spread <= stable_limit:
                break

        if samples:
            values = np.asarray([item[0] for item in samples], dtype=np.float64)
            median_s = float(np.median(values))
            representative = min(samples, key=lambda item: abs(item[0] - median_s))
            logger.debug(
                "[Autofocus] S estable Z=%.2f: mediana=%.3f n=%d "
                "min=%.3f max=%.3f",
                float(z_cmd),
                median_s,
                len(samples),
                float(np.min(values)),
                float(np.max(values)),
            )
            return median_s, representative[1]

        return 0.0, None

    def _park_at_bpof(
        self,
        z_bpof: float,
        bbox,
        contour,
        log_prefix: str,
        *,
        score: Optional[float] = None,
    ) -> Tuple[float, float]:
        """Solo GOTO Z* (sin remarcar S). score=tabla si se pasa."""
        del bbox, contour  # park no remide
        ok, z_read = self._goto_z_static(float(z_bpof))
        s = float(score) if score is not None else 0.0
        if ok and score is not None:
            self.score_updated.emit(float(z_bpof), s)
        logger.info(
            "%s Park BPoF: Z_cmd=%.2fµm Z_read=%s S_tabla=%.1f ok=%s",
            log_prefix,
            float(z_bpof),
            f"{z_read:.2f}" if z_read is not None else "?",
            s,
            ok,
        )
        return float(z_bpof), s

    def goto_calibration_origin(
        self,
        *,
        log_prefix: str = "[Autofocus]",
        emit_status: bool = True,
    ) -> Tuple[bool, Optional[float], float]:
        """Vuelve al Z centro de calibración (origen de referencia).

        Returns:
            (ok, z_read, z_center_cmd)
        """
        if self.cfocus_controller is None:
            return False, None, 0.0
        info = self.cfocus_controller.get_calibration_info()
        if not info.get("is_calibrated") or info.get("z_center") is None:
            logger.warning("%s Origen calibrado no disponible", log_prefix)
            return False, None, 0.0
        z_cmd = float(info["z_center"])
        ok, z_read = self._goto_z_static(z_cmd)
        msg = (
            f"{log_prefix} Origen calibrado: Z_cmd={z_cmd:.2f}µm "
            f"Z_read={f'{z_read:.2f}' if z_read is not None else '?'}µm "
            f"ok={ok}"
        )
        logger.info(msg)
        if emit_status:
            try:
                self.status_message.emit(msg)
            except Exception:
                pass
        return ok, z_read, z_cmd

    def _optimize_focus_simple(
        self,
        rois: List[Tuple[Tuple[int, int, int, int], np.ndarray]],
        z_min: float,
        z_max: float,
        z_center: float,
        log_prefix: str = "[Autofocus]",
        microscopy_format: bool = False,
    ) -> tuple:
        """
        1) TABLA COARSE (Z, S_surf) → Z_c* = argmax
        2) Z_coarse* → FINE: Paso_fine × N capas (limitado por ±Δ)
        3) TABLA FINE → BPoF = argmax
        S_surf = Σ S_i (todos los ROI, un frame por plano).
        """
        focus_scan_t0 = time.perf_counter()
        self._last_focus_curve = []
        kpi = self.__dict__.get("_cycle_kpi")
        if kpi is None:
            kpi = AfCycleKpi()
            self._cycle_kpi = kpi
        kpi.coarse_window_source = str(
            self.__dict__.get("_coarse_window_source", "full")
        )
        kpi.coarse_span_um = float(z_max) - float(z_min)
        coarse_label = "SCAN" if microscopy_format else "COARSE"
        tol = float(self.z_arrive_tol_um)
        n_roi = len(rois)
        native_hw = self.__dict__.get("_native_frame_hw")
        native_label = (
            f"{native_hw[1]}×{native_hw[0]} 1:1"
            if native_hw is not None
            else "nativa 1:1"
        )

        msg = (
            f"{log_prefix} Autofoco superficie: {n_roi} ROI | "
            f"COARSE→Z_c*→FINE(paso×N)→BPoF→STACK | "
            f"Z {z_min:.2f}→{z_max:.2f}µm paso_c={self.z_step_coarse}µm | "
            f"llegada Z: |err|≤{tol:.2f}µm | "
            f"S=ΣS_i CLAHE-HF-v4 RAW12/uint16→float64 {native_label} "
            f"sin resize mediana×"
            f"{int(getattr(self, 'score_samples_per_plane', 3))}"
        )
        logger.info(msg)
        print(msg)
        self.status_message.emit(msg)

        z_range = z_max - z_min
        n_steps = int(z_range / self.z_step_coarse) + 1
        tabla_coarse = BpofCandidateTable(
            n_planned_planes=n_steps, phase="coarse"
        )
        min_req = tabla_coarse.min_required
        coarse_best_s = -float("inf")
        coarse_best_i = -1
        kpi.n_coarse_planned = n_steps
        coarse_t0 = time.perf_counter()

        # --- 1) TABLA COARSE ---
        for i in range(n_steps):
            if self.cancel_requested:
                break

            z_current = min(z_max, z_min + (i * self.z_step_coarse))
            self.progress_updated.emit(i + 1, n_steps, "Escaneo completo")

            score = self.evaluate_s_at_z(z_current, rois=rois)
            if self.cancel_requested:
                break

            tabla_coarse.add(z_current, score)
            self.score_updated.emit(z_current, score)
            if float(score) > coarse_best_s:
                coarse_best_s = float(score)
                coarse_best_i = i

            run_best = tabla_coarse.summary_top(1)
            best_s_run = run_best[0].s if run_best else 0.0
            best_z_run = run_best[0].z_um if run_best else z_current
            progress_pct = ((i + 1) / n_steps) * 100
            msg = (
                f"{log_prefix} {coarse_label}: {progress_pct:.0f}% | "
                f"Z={z_current:.2f}µm S={score:.1f} | "
                f"maxS_coarse={best_s_run:.1f}@{best_z_run:.2f}µm "
                f"(n={len(tabla_coarse)})"
            )
            if microscopy_format:
                print(msg, end='\r', flush=True)
            logger.debug(msg)
            self.status_message.emit(
                f"{log_prefix} S_ROI COARSE {i + 1}/{n_steps}: "
                f"Z={z_current:.2f}µm S={float(score):.2f}"
            )

            patience = max(
                2, int(self.__dict__.get("coarse_early_stop_patience", 4))
            )
            drop_rel = max(
                0.0,
                float(self.__dict__.get("coarse_early_stop_drop_rel", 0.03)),
            )
            if (
                coarse_best_s > 1e-9
                and len(tabla_coarse) >= min_req
                and coarse_best_i >= 0
                and (i - coarse_best_i) >= patience
                and float(z_current) >= float(z_center)
                and float(score) <= coarse_best_s * (1.0 - drop_rel)
            ):
                drop_pct = (
                    100.0
                    * (coarse_best_s - float(score))
                    / coarse_best_s
                )
                stop_msg = (
                    f"{log_prefix} COARSE optimizado: pico superado por "
                    f"{i - coarse_best_i} planos y S cayó "
                    f"{drop_pct:.1f}% "
                    f"→ fin temprano en {i + 1}/{n_steps}"
                )
                logger.info(stop_msg)
                self.status_message.emit(stop_msg)
                kpi.coarse_early_stop = True
                break

        print()
        kpi.n_coarse_measured = len(tabla_coarse)
        kpi.t_coarse = time.perf_counter() - coarse_t0
        z_coarse_star, s_coarse_star, info_c = tabla_coarse.select_near_max(
            float(z_center),
            relative_tie=float(
                self.__dict__.get("coarse_near_max_rel", 0.005)
            ),
        )
        if float(s_coarse_star) <= 0.0:
            raise RuntimeError(
                f"{log_prefix} COARSE inválido: todos los S=0 "
                f"(n={len(tabla_coarse)}). Revisar flush óptico "
                f"(frame_count live) y acquire_scientific_frame"
            )
        if not tabla_coarse.meets_minimum():
            logger.warning(
                "%s Tabla COARSE: válidos < mínimo %d (planificados=%d)",
                log_prefix,
                min_req,
                n_steps,
            )
        self._emit_candidate_table(
            tabla_coarse, "TABLA COARSE (candidatos)", log_prefix
        )
        logger.info(
            "%s COARSE robusto: argmax_raw=%.2fµm@%.3f, "
            "banda_empate=%.3f (n=%d) → Z_c*=%.2fµm@%.3f "
            "por cercanía al origen %.2fµm",
            log_prefix,
            float(info_c.get("raw_argmax_z", z_coarse_star)),
            float(info_c.get("raw_argmax_s", s_coarse_star)),
            float(info_c.get("tie_band", 0.0)),
            int(info_c.get("n_tied", 1)),
            z_coarse_star,
            s_coarse_star,
            float(z_center),
        )

        if s_coarse_star <= 0.0 or info_c.get("method") == "empty":
            tabla_coarse.clear()
            raise RuntimeError(
                f"{log_prefix} Tabla COARSE sin mediciones S válidas"
            )

        # --- 2) Zona fine centrada en max S de tabla coarse ---
        delta_um = float(self.z_scan_range)
        step_fine_ui = float(self.z_step_fine)
        n_fine_cfg = int(getattr(self, "n_fine_planes", 15))
        hw_info = self.cfocus_controller.get_calibration_info()
        z_fine_min = float(hw_info.get("z_min", z_min))
        z_fine_max = float(hw_info.get("z_max", z_max))
        fine_planes, delta_eff = build_fine_z_planes(
            z_coarse_star,
            delta_um,
            n_fine_cfg,
            z_fine_min,
            z_fine_max,
            z_step_um=step_fine_ui,
        )
        max_fine = max(3, int(self.max_fine_iterations))
        if max_fine % 2 == 0:
            max_fine -= 1
        if len(fine_planes) > max_fine:
            tabla_coarse.clear()
            raise RuntimeError(
                f"{log_prefix} Plan FINE inválido: N={len(fine_planes)} "
                f"excede max_fine_iterations={max_fine}; NO CAPTURA"
            )
        total_refine_steps = len(fine_planes)
        tabla_fine = BpofCandidateTable(
            n_planned_planes=total_refine_steps, phase="fine"
        )
        step_eff = (
            (
                (float(fine_planes[-1]) - float(fine_planes[0]))
                / (total_refine_steps - 1)
            )
            if total_refine_steps > 1
            else 0.0
        )

        msg = (
            f"{log_prefix} ENLACE COARSE→FINE: "
            f"Z_c*={z_coarse_star:.2f}µm S={s_coarse_star:.1f} "
            f"→ rango=[{fine_planes[0]:.2f},{fine_planes[-1]:.2f}]µm "
            f"({total_refine_steps} capas, "
            f"paso_UI={step_fine_ui:.3f}µm, paso_eff={step_eff:.3f}µm, "
            f"Δ_max_UI={delta_um:.2f}µm)"
        )
        logger.info(msg)
        print(msg)
        self.status_message.emit(msg)

        # Re-anclar en Z_c* antes del fine (coarse termina en z_max → salto grande)
        ok_c, z_c_read = self._goto_z_static(float(z_coarse_star))
        logger.info(
            "%s Re-ancla pre-FINE: Z_c*=%.2fµm Z_read=%s ok=%s",
            log_prefix,
            float(z_coarse_star),
            f"{z_c_read:.2f}" if z_c_read is not None else "?",
            ok_c,
        )
        if not ok_c:
            tabla_coarse.clear()
            tabla_fine.clear()
            raise RuntimeError(
                f"{log_prefix} FINE abortado: no se pudo re-anclar en "
                f"Z_coarse*={z_coarse_star:.2f}µm"
            )

        # El primer candidato FINE es exactamente la salida COARSE y desde ahí
        # se avanza por anillos simétricos. Recorrer la ventana en orden
        # creciente de Z obligaba a saltar del extremo del COARSE al extremo
        # inferior del FINE y a medir todo el fondo antes de llegar al pico.
        fine_order = center_out_sequence(fine_planes, z_coarse_star)
        early_stop = RingDeclineStop(
            ring_counts(fine_order),
            patience_rings=int(
                self.__dict__.get("fine_early_stop_patience_rings", 3)
            ),
            drop_rel=float(
                self.__dict__.get("fine_early_stop_drop_rel", 0.05)
            ),
        )
        n_decision_samples = max(
            1, int(self.__dict__.get("score_samples_at_decision", 2))
        )
        fine_t0 = time.perf_counter()

        # --- 3) TABLA FINE → BPoF = argmax ---
        for refine_iteration, (z_refine, ring) in enumerate(
            fine_order, start=1
        ):
            if self.cancel_requested:
                break

            self.progress_updated.emit(
                refine_iteration, total_refine_steps, "Refinamiento fino"
            )

            is_anchor = (
                refine_iteration == 1
                and abs(float(z_refine) - float(z_coarse_star)) <= 1e-6
            )
            if is_anchor:
                # El ancla decide dónde se centra todo el refinamiento: es el
                # único plano donde vale la pena pagar la mediana de N tomas.
                score = self._measure_s_decision(
                    z_refine,
                    rois=rois,
                    n_samples=n_decision_samples,
                    log_prefix=log_prefix,
                    role="ancla FINE",
                )
            else:
                score = self.evaluate_s_at_z(z_refine, rois=rois)
            if self.cancel_requested:
                break

            # El ancla repite exactamente Z_coarse*: si S discrepa mucho de la
            # medida en COARSE, la escala no es comparable entre fases y el
            # dato queda registrado como ε_ancla (KPI de repetibilidad).
            if is_anchor and float(s_coarse_star) > 1e-9:
                eps_anchor = abs(
                    float(score) - float(s_coarse_star)
                ) / float(s_coarse_star)
                kpi.eps_anchor = float(eps_anchor)
                if eps_anchor > 0.12:
                    retry_msg = (
                        f"{log_prefix} FINE ancla discrepante: "
                        f"Z={z_refine:.2f}µm S_coarse={s_coarse_star:.2f} "
                        f"S_fine={float(score):.2f} "
                        f"(ε_ancla={eps_anchor:.1%}); manda la mediana FINE"
                    )
                    logger.warning(retry_msg)
                    self.status_message.emit(retry_msg)

            tabla_fine.add(z_refine, score)
            self.score_updated.emit(z_refine, score)
            early_stop.observe(ring, float(score))

            run_best = tabla_fine.summary_top(1)
            best_s_run = run_best[0].s if run_best else 0.0
            best_z_run = run_best[0].z_um if run_best else z_refine
            progress_pct = (refine_iteration / max(1, total_refine_steps)) * 100
            msg = (
                f"{log_prefix} FINE: {progress_pct:.0f}% | "
                f"Z={z_refine:.2f}µm S={score:.1f} | "
                f"BPoF_tmp={best_s_run:.1f}@{best_z_run:.2f}µm "
                f"(n={len(tabla_fine)})"
            )
            if microscopy_format:
                print(msg, end='\r', flush=True)
            logger.debug(msg)
            self.status_message.emit(
                f"{log_prefix} S_ROI FINE {refine_iteration}/"
                f"{total_refine_steps}: Z={z_refine:.2f}µm "
                f"S={float(score):.2f}"
            )

            if early_stop.should_stop():
                kpi.fine_early_stop = True
                stop_msg = (
                    f"{log_prefix} FINE optimizado: {early_stop.reason} "
                    f"→ fin temprano en {refine_iteration}/"
                    f"{total_refine_steps}"
                )
                logger.info(stop_msg)
                self.status_message.emit(stop_msg)
                break

        print()
        kpi.t_fine = time.perf_counter() - fine_t0
        kpi.n_fine_measured = len(tabla_fine)
        self._reject_isolated_dips(tabla_fine, rois=rois, log_prefix=log_prefix)

        best_z, best_score, info_f = tabla_fine.select_argmax()
        self._emit_candidate_table(
            tabla_fine, "TABLA FINE (candidatos) → BPoF", log_prefix
        )

        if best_score <= 0.0 or info_f.get("method") == "empty":
            tabla_coarse.clear()
            tabla_fine.clear()
            raise RuntimeError(
                f"{log_prefix} Tabla FINE sin mediciones S válidas "
                f"(centro coarse Z={z_coarse_star:.2f}µm)"
            )

        peak_quality = tabla_fine.assess_peak(
            min_relative_span=float(
                self.__dict__.get("bpof_min_relative_span", 0.005)
            ),
            min_prominence_rel=float(
                self.__dict__.get("bpof_min_prominence_rel", 0.003)
            ),
        )
        # La confirmación posterior necesita saber si el pico es creíble para
        # decidir si vale la pena sondear vecinos (3 mediciones extra).
        self._last_peak_quality = peak_quality
        kpi.z_coarse_star = float(z_coarse_star)
        kpi.fine_span_um = fine_span_um(
            [row.z_um for row in tabla_fine.latest_per_z()]
        )
        kpi.peak_at_edge = bool(peak_quality.get("at_edge", False))
        kpi.span_rel = float(peak_quality.get("relative_span", 0.0))
        kpi.prominence_rel = float(peak_quality.get("prominence_rel", 0.0))

        quality_warning = not peak_quality["valid"]
        if quality_warning:
            warning = (
                f"{log_prefix} ⚠ BPoF de baja confianza: "
                f"{peak_quality['reason']}; "
                f"span_rel={peak_quality.get('relative_span', 0.0):.4f}, "
                f"prom_rel={peak_quality.get('prominence_rel', 0.0):.4f}. "
                "Se conserva el argmax medido y se construye el plan fotográfico."
            )
            logger.warning(warning)
            self.status_message.emit(warning)

        n_fine = len(tabla_fine)
        self._last_focus_curve = [
            {
                "z_um": float(row.z_um),
                "score": float(row.s),
                "phase": str(row.phase),
            }
            for row in (
                tabla_coarse.sorted_by_z() + tabla_fine.sorted_by_z()
            )
        ]
        scan_elapsed_s = time.perf_counter() - focus_scan_t0
        self.status_message.emit(
            f"{log_prefix} COARSE+FINE completado en {scan_elapsed_s:.2f}s "
            f"({len(tabla_coarse)}+{len(tabla_fine)} mediciones S)"
        )

        # --- ROI Tracking: estado de la ventana de medida ---
        tracker = self.__dict__.get("_roi_tracker")
        if tracker is not None and tracker.is_active:
            logger.info(tracker.get_summary())
            kpi.n_detections = int(tracker.n_detections)
            if tracker.static_window:
                # Un desborde significa que ese plano midió el grano recortado
                # y su S no es comparable con la del resto del barrido.
                overflows, worst = tracker.get_overflow_stats()
                if overflows:
                    roi_msg = (
                        f"{log_prefix} ROI estático: la segmentación se salió "
                        f"en {overflows}/{tracker.n_updates} planos "
                        f"(máx {worst}px) — subir 'margin' a "
                        f"≥{tracker.static_pad_px + worst}px"
                    )
                else:
                    roi_msg = (
                        f"{log_prefix} ROI estático: segmentación contenida en "
                        f"los {tracker.n_detections} planos verificados de "
                        f"{tracker.n_updates}; S comparable"
                    )
                logger.info(roi_msg)
                self.status_message.emit(roi_msg)
            else:
                drift_total = tracker.get_total_drift()
                if drift_total > 0.5:
                    drift_msg = (
                        f"{log_prefix} ROI Tracking: drift "
                        f"máximo={drift_total:.1f}px en "
                        f"{tracker.n_updates} pasos Z"
                    )
                    logger.info(drift_msg)
                    self.status_message.emit(drift_msg)

        tabla_coarse.clear()
        tabla_fine.clear()

        # Sin park aquí: focus_object_sync captura/aparca una sola vez al final
        msg = (
            f"{log_prefix} ✓ BPoF TABLA: Z={best_z:.2f}µm S={best_score:.1f} "
            f"(confianza={'BAJA' if quality_warning else 'OK'} "
            f"span_rel={peak_quality.get('relative_span', 0.0):.4f}, "
            f"prom_rel={peak_quality.get('prominence_rel', 0.0):.4f}; "
            f"centro COARSE "
            f"Z={z_coarse_star:.2f}µm S={s_coarse_star:.1f}; n_fine={n_fine})"
        )
        logger.info(msg)
        print(msg)
        self.status_message.emit(msg)
        self.score_updated.emit(float(best_z), float(best_score))

        return best_z, best_score

    def _measure_s_decision(
        self,
        z: float,
        *,
        rois,
        n_samples: int = 2,
        log_prefix: str = "[Autofocus]",
        role: str = "decisión",
    ) -> float:
        """Mediana de N mediciones S en un plano que decide Z.

        Una sola toma por plano es suficiente para dibujar la curva, pero no
        para elegir el plano que se va a fotografiar: en el log de referencia el
        mismo Z dio 362.4 y 280.0. El coste de la segunda toma sólo se paga en
        el ancla y en el BPoF candidato, no en los N planos del barrido.
        """
        n = max(1, int(n_samples))
        samples = []
        for _ in range(n):
            if self.cancel_requested:
                break
            value = float(self.evaluate_s_at_z(float(z), rois=rois))
            if value > 0.0 and np.isfinite(value):
                samples.append(value)

        extra = max(0, len(samples) - 1)
        if extra:
            kpi = self.__dict__.get("_cycle_kpi")
            if kpi is not None:
                kpi.n_extra_measurements += extra

        if not samples:
            return 0.0
        median_s = float(np.median(samples))
        if len(samples) > 1:
            spread = (max(samples) - min(samples)) / median_s
            logger.info(
                "%s S mediana (%s) Z=%.2fµm: %s → %.2f (dispersión=%.1f%%)",
                log_prefix,
                role,
                float(z),
                ", ".join(f"{value:.2f}" for value in samples),
                median_s,
                100.0 * spread,
            )
        return median_s

    def _reject_isolated_dips(
        self,
        table: BpofCandidateTable,
        *,
        rois,
        log_prefix: str,
    ) -> None:
        """Re-mide los planos hundidos y descarta los que insisten.

        Un pozo de S que se abre y se cierra en un paso fino no es óptica: es un
        frame que no representa el plano. Dejarlo en la tabla no sólo pierde ese
        plano, también infla a sus vecinos por comparación y puede entregarles
        el BPoF.
        """
        factor = float(self.__dict__.get("fine_dip_reject_factor", 0.85))
        if factor <= 0.0 or self.cancel_requested:
            return

        dips = table.isolated_dip_planes(factor=factor)
        if not dips:
            return

        kpi = self.__dict__.get("_cycle_kpi")
        if kpi is not None:
            kpi.n_holes += len(dips)

        for z_dip in dips:
            if self.cancel_requested:
                return
            s_new = float(self.evaluate_s_at_z(float(z_dip), rois=rois))
            if kpi is not None:
                kpi.n_extra_measurements += 1
            if s_new > 0.0 and np.isfinite(s_new):
                table.add(float(z_dip), s_new)
            msg = (
                f"{log_prefix} Plano hundido Z={float(z_dip):.2f}µm re-medido: "
                f"S={s_new:.2f}"
            )
            logger.info(msg)
            self.status_message.emit(msg)

        for z_dip in table.isolated_dip_planes(factor=factor):
            removed = table.invalidate_z(z_dip)
            if kpi is not None:
                kpi.n_holes_invalidated += 1
            msg = (
                f"{log_prefix} Plano Z={float(z_dip):.2f}µm descartado: S se "
                f"hunde bajo {factor:.0%} de sus vecinos en dos mediciones "
                f"({removed} fila(s) fuera del argmax)"
            )
            logger.warning(msg)
            self.status_message.emit(msg)

    def _confirm_bpof_before_stack(
        self,
        best_z: float,
        best_score: float,
        *,
        rois,
        log_prefix: str,
    ):
        """
        Re-mide S en el BPoF con flush óptico completo.

        Si la S cae demasiado vs el pico FINE, prueba vecinos ±paso fino y
        adopta el mejor (evita fotografiar un plano 'perdido' tras mid-move).
        """
        z0 = float(best_z)
        s_fine = float(best_score)
        if s_fine <= 0.0 or self.cancel_requested:
            return z0, s_fine

        s_confirm = float(self.evaluate_s_at_z(z0, rois=rois))
        kpi = self.__dict__.get("_cycle_kpi")
        if kpi is not None:
            kpi.n_extra_measurements += 1
            if s_fine > 0.0:
                kpi.eps_confirm = abs(s_confirm - s_fine) / s_fine
        max_drop = float(getattr(self, "bpof_confirm_max_drop_rel", 0.08))
        if s_confirm > 0.0 and s_confirm + 1e-9 >= s_fine * (1.0 - max_drop):
            if abs(s_confirm - s_fine) > 1e-3:
                logger.info(
                    "%s BPoF confirmado: Z=%.2fµm S_fine=%.1f → S_conf=%.1f",
                    log_prefix,
                    z0,
                    s_fine,
                    s_confirm,
                )
            return z0, float(s_confirm)

        # Una toma baja no basta para sondear vecinos. Si la curva FINE dejó un
        # pico interior y prominente, esas 3 mediciones extra no pueden desmentir
        # a las N del barrido: sólo añaden ruido y ~3s por semilla. Se registra
        # ε_confirm igualmente, que es el indicador de que S no se repite.
        peak = self.__dict__.get("_last_peak_quality") or {}
        peak_healthy = bool(peak.get("valid")) and not bool(
            peak.get("at_edge", True)
        )
        probe_neighbors = bool(
            self.__dict__.get(
                "bpof_confirm_probe_neighbors_when_healthy", False
            )
        )
        if peak_healthy and not probe_neighbors:
            drop = (s_fine - s_confirm) / s_fine if s_fine > 0 else 0.0
            msg = (
                f"{log_prefix} BPoF mantenido sin sondear vecinos: "
                f"Z={z0:.2f}µm S_fine={s_fine:.1f} S_conf={s_confirm:.1f} "
                f"(ε_confirm={drop:.1%}; pico FINE interior y prominente, "
                f"span_rel={float(peak.get('relative_span', 0.0)):.4f})"
            )
            logger.info(msg)
            try:
                self.status_message.emit(msg)
            except (AttributeError, RuntimeError):
                pass
            return z0, s_fine

        step = max(0.5, float(getattr(self, "z_step_fine", 1.0) or 1.0))
        s_at_z0 = max(0.0, s_confirm)
        neighbors = []
        for z_try in (z0, z0 - step, z0 + step):
            if self.cancel_requested:
                break
            s_try = float(self.evaluate_s_at_z(float(z_try), rois=rois))
            if abs(float(z_try) - z0) <= 1e-9:
                s_at_z0 = max(s_at_z0, s_try)
            else:
                neighbors.append((float(z_try), s_try))

        # El barrido FINE ya votó por z0 con N mediciones; un vecino sólo lo
        # desbanca si gana por encima del ruido de una toma suelta. Sin este
        # margen, cualquier confirmación baja desplazaba el BPoF un paso fino.
        margin = max(
            0.0, float(getattr(self, "bpof_confirm_neighbor_margin_rel", 0.03))
        )
        best_zc, best_sc = z0, s_at_z0
        if neighbors:
            cand_z, cand_s = max(neighbors, key=lambda item: item[1])
            if cand_s > s_at_z0 * (1.0 + margin):
                best_zc, best_sc = cand_z, cand_s

        drop = (s_fine - s_confirm) / s_fine if s_fine > 0 else 0.0
        if abs(best_zc - z0) <= 1e-9:
            msg = (
                f"{log_prefix} BPoF mantenido tras confirmación: "
                f"Z={z0:.2f}µm S_fine={s_fine:.1f} S_conf={best_sc:.1f} "
                f"(caída confirm={drop:.1%}; ningún vecino supera "
                f"+{margin:.0%})"
            )
        else:
            msg = (
                f"{log_prefix} BPoF reanclado tras confirmación: "
                f"Z_fine={z0:.2f}µm S={s_fine:.1f} → "
                f"Z={best_zc:.2f}µm S={best_sc:.1f} "
                f"(caída confirm={drop:.1%})"
            )
        logger.info(msg)
        try:
            self.status_message.emit(msg)
        except (AttributeError, RuntimeError):
            pass
        if best_sc > 0.0:
            self.score_updated.emit(float(best_zc), float(best_sc))
            return float(best_zc), float(best_sc)
        return z0, s_fine

    def _seek_optical_drop_plane(
        self,
        *,
        best_z: float,
        baseline_s: float,
        direction: int,
        target_drop_rel: float,
        z_min: float,
        z_max: float,
        rois,
        log_prefix: str,
    ) -> dict:
        """Busca Z por respuesta óptica S, con expansión y refinamiento."""
        sign = -1 if int(direction) < 0 else 1
        limit = float(z_min if sign < 0 else z_max)
        max_distance = abs(limit - float(best_z))
        if max_distance <= 1e-9:
            raise RuntimeError(
                f"{log_prefix} Sin recorrido Z disponible hacia "
                f"{'abajo' if sign < 0 else 'arriba'} desde BPoF"
            )
        if baseline_s <= 0.0:
            raise RuntimeError(f"{log_prefix} S_BPoF inválido: {baseline_s}")

        target = max(0.001, min(0.95, float(target_drop_rel)))
        base_step = max(
            0.25,
            float(getattr(self, "z_step_fine", 0.1)),
            float(getattr(self, "z_arrive_tol_um", 0.5)),
        )
        distance = min(base_step, max_distance)
        candidates = []
        previous = None
        bracket = None

        while True:
            z_probe = float(best_z) + sign * distance
            score = float(self.evaluate_s_at_z(z_probe, rois=rois))
            if score > 0.0 and np.isfinite(score):
                drop = max(0.0, (float(baseline_s) - score) / float(baseline_s))
                current = {
                    "z_um": z_probe,
                    "score": score,
                    "drop_rel": drop,
                    "distance_um": distance,
                }
                candidates.append(current)
                logger.info(
                    "%s PROBE ÓPTICO %s: ΔZ=%+.3fµm S=%.3f "
                    "caída=%.2f%% objetivo=%.2f%%",
                    log_prefix,
                    "abajo" if sign < 0 else "arriba",
                    sign * distance,
                    score,
                    100.0 * drop,
                    100.0 * target,
                )
                if drop >= target:
                    bracket = (previous, current) if previous is not None else None
                    break
                previous = current

            if distance >= max_distance - 1e-9:
                break
            distance = min(
                max_distance,
                max(distance + base_step, distance * 1.75),
            )

        if bracket is not None and bracket[0]["drop_rel"] < target:
            low = bracket[0]
            high = bracket[1]
            for _ in range(
                max(0, int(getattr(self, "optical_search_refine_iterations", 4)))
            ):
                mid_distance = 0.5 * (
                    float(low["distance_um"]) + float(high["distance_um"])
                )
                z_probe = float(best_z) + sign * mid_distance
                score = float(self.evaluate_s_at_z(z_probe, rois=rois))
                if score <= 0.0 or not np.isfinite(score):
                    continue
                drop = max(
                    0.0, (float(baseline_s) - score) / float(baseline_s)
                )
                current = {
                    "z_um": z_probe,
                    "score": score,
                    "drop_rel": drop,
                    "distance_um": mid_distance,
                }
                candidates.append(current)
                if drop >= target:
                    high = current
                else:
                    low = current

        if not candidates:
            raise RuntimeError(
                f"{log_prefix} No se obtuvo ninguna medición S válida hacia "
                f"{'abajo' if sign < 0 else 'arriba'}"
            )

        reached = [c for c in candidates if c["drop_rel"] >= target]
        if reached:
            selected = min(
                reached,
                key=lambda c: (abs(c["drop_rel"] - target), c["distance_um"]),
            )
        else:
            selected = max(candidates, key=lambda c: c["drop_rel"])
        selected = dict(selected)
        selected["target_drop_rel"] = target
        selected["target_met"] = bool(selected["drop_rel"] >= target)
        if not selected["target_met"]:
            logger.warning(
                "%s Límite óptico/hardware %s: objetivo ΔS=%.2f%% no "
                "alcanzable; se usa máxima caída medida %.2f%% en ΔZ=%+.3fµm",
                log_prefix,
                "abajo" if sign < 0 else "arriba",
                100.0 * target,
                100.0 * selected["drop_rel"],
                sign * selected["distance_um"],
            )
        return selected

    def _build_plan_from_measured_focus_curve(
        self,
        *,
        best_z: float,
        best_score: float,
        n_captures: int,
        target_drop_rel: float,
        log_prefix: str,
    ) -> Optional[Tuple[list, float, list]]:
        """Selecciona el stack desde COARSE+FINE ya medidos; cero probes extra."""
        rows = list(self.__dict__.get("_last_focus_curve", []) or [])
        if not rows:
            return None

        # Sólo FINE. COARSE es otra pasada: entre ambas el ROI se recoloca y la
        # misma Z puede medir muy distinto (visto en log: Z=3.04µm dio S=325.8
        # en COARSE y S=505.6 al fotografiar). Mezclar las dos escalas hacía
        # que un plano COARSE lejano "cumpliera" el ΔS y se fotografiara a 9µm
        # del BPoF. Dentro de FINE todas las medidas comparten estado y paso.
        fine_rows = [row for row in rows if str(row.get("phase")) == "fine"]
        if len(fine_rows) >= int(n_captures):
            rows = fine_rows

        by_z = {}
        for row in rows:
            z = float(row["z_um"])
            score = float(row["score"])
            if score <= 0.0 or not np.isfinite(score):
                continue
            key = round(z, 6)
            previous = by_z.get(key)
            if previous is None or str(row.get("phase")) == "fine":
                by_z[key] = {
                    "z_um": z,
                    "score": score,
                    "phase": str(row.get("phase", "measured")),
                }

        # El baseline tiene que salir de la MISMA curva contra la que se miden
        # las caídas. Si se usa la S re-medida en la confirmación, basta con
        # que esa toma salga baja para que todo un lado de la curva aparente
        # ΔS≤0, el stack se vuelva unilateral y el BPoF quede en el borde en
        # vez de en el centro.
        anchor_row = by_z.get(round(float(best_z), 6))
        baseline = (
            float(anchor_row["score"]) if anchor_row else float(best_score)
        )
        if baseline <= 0.0 or len(by_z) < int(n_captures):
            return None

        candidates = []
        for row in by_z.values():
            if abs(float(row["z_um"]) - float(best_z)) <= 1e-6:
                continue
            drop = max(0.0, (baseline - float(row["score"])) / baseline)
            candidates.append(
                {
                    **row,
                    "drop_rel": drop,
                    "distance_um": abs(float(row["z_um"]) - float(best_z)),
                    "curve_reused": True,
                }
            )

        # Solo planos cercanos al BPoF (FINE o entorno). Reutilizar COARSE
        # lejano (p.ej. −37µm) “cumple %” pero pierde el foco del objeto.
        fine_step = float(self.__dict__.get("z_step_fine", 1.0) or 1.0)
        n_fine = int(self.__dict__.get("n_fine_planes", 9) or 9)
        max_span = max(
            12.0,
            fine_step * max(4.0, (n_fine // 2) * 2.0),
            float(self.__dict__.get("z_scan_range", 7.0) or 7.0) * 2.0,
        )
        near = [
            item
            for item in candidates
            if float(item["distance_um"]) <= max_span + 1e-9
        ]
        if len(near) >= int(n_captures) - 1:
            candidates = near

        lower = [item for item in candidates if item["z_um"] < float(best_z)]
        upper = [item for item in candidates if item["z_um"] > float(best_z)]
        half = int(n_captures) // 2
        outer = float(target_drop_rel)

        def select_levels(
            pool: list, count: int, target_outer: float, *, require_met: bool
        ):
            """Elige planos que CUMPLEN el ΔS pedido; el más cercano al BPoF."""
            available = list(pool)
            selected = []
            for level in range(1, count + 1):
                if not available:
                    return None
                target = target_outer * level / count
                meeting = [
                    candidate
                    for candidate in available
                    if float(candidate["drop_rel"]) + 1e-12 >= target
                ]
                if meeting:
                    # Cumple ΔS: preferir el más cercano en Z al BPoF.
                    # (Antes se prefería |ΔZ| máximo → planos lejanos sin foco.)
                    item = min(
                        meeting,
                        key=lambda candidate: (
                            abs(float(candidate["drop_rel"]) - target),
                            float(candidate["distance_um"]),
                        ),
                    )
                elif require_met:
                    # La curva COARSE/FINE no alcanza el % GUI → forzar seek óptico.
                    return None
                else:
                    item = max(
                        available,
                        key=lambda candidate: (
                            float(candidate["drop_rel"]),
                            float(candidate["distance_um"]),
                        ),
                    )
                available.remove(item)
                chosen = dict(item)
                chosen["target_drop_rel"] = target
                chosen["target_met"] = bool(
                    float(chosen["drop_rel"]) + 1e-12 >= target
                )
                chosen["requested_target_drop_rel"] = target
                chosen["requested_target_met"] = chosen["target_met"]
                selected.append(chosen)
            return selected

        lower_max = max((item["drop_rel"] for item in lower), default=0.0)
        upper_max = max((item["drop_rel"] for item in upper), default=0.0)
        # Reutilizar curva solo si ambos lados alcanzan el ΔS de la GUI.
        centered = (
            len(lower) >= half
            and len(upper) >= half
            and lower_max + 1e-12 >= outer
            and upper_max + 1e-12 >= outer
        )

        if centered:
            lower_sel = select_levels(lower, half, outer, require_met=True)
            upper_sel = select_levels(upper, half, outer, require_met=True)
            if lower_sel is None or upper_sel is None:
                return None
            selected = lower_sel + upper_sel
        else:
            useful_pool = upper if upper_max >= lower_max else lower
            if len(useful_pool) < int(n_captures) - 1:
                return None
            available_outer = max(
                (float(item["drop_rel"]) for item in useful_pool), default=0.0
            )
            if available_outer <= 1e-6:
                return None
            # Si la curva no llega al % GUI, no fingir planos: dejar que el seek busque.
            if available_outer + 1e-12 < outer:
                return None
            selected = select_levels(
                useful_pool,
                int(n_captures) - 1,
                outer,
                require_met=True,
            )
            if selected is None:
                return None
            for level, item in enumerate(selected, start=1):
                requested = outer * level / (int(n_captures) - 1)
                item["requested_target_drop_rel"] = requested
                item["requested_target_met"] = bool(
                    float(item["drop_rel"]) + 1e-12 >= requested
                )

        lower_selected = sorted(
            [item for item in selected if item["z_um"] < float(best_z)],
            key=lambda item: item["z_um"],
        )
        upper_selected = sorted(
            [item for item in selected if item["z_um"] > float(best_z)],
            key=lambda item: item["z_um"],
        )
        z_positions = (
            [float(item["z_um"]) for item in lower_selected]
            + [float(best_z)]
            + [float(item["z_um"]) for item in upper_selected]
        )
        if len(z_positions) != int(n_captures) or len(
            {round(z, 6) for z in z_positions}
        ) != int(n_captures):
            return None

        # Si un lado exigió mucho más recorrido que el otro para la misma caída
        # de S, la curva no es una escala fiable: se refleja la distancia menor
        # (la del lado que sí responde) sobre planos ya medidos. Cero sondeos
        # nuevos y las tres tomas quedan a la misma distancia del foco.
        balanced = rebalance_symmetric(
            lower_selected,
            upper_selected,
            pool=candidates,
            z_bpof=float(best_z),
            baseline=baseline,
            max_ratio=float(self.__dict__.get("stack_asymmetry_max", 3.0)),
            tol_um=max(
                0.05, 0.25 * float(self.__dict__.get("z_step_fine", 0.5) or 0.5)
            ),
        )
        if balanced is not None:
            z_balanced, items_balanced = balanced
            ratio_before = stack_asymmetry_ratio(
                [float(z) - float(best_z) for z in z_positions]
            )
            ratio_after = stack_asymmetry_ratio(
                [float(z) - float(best_z) for z in z_balanced]
            )
            planes_before = ", ".join(f"{z:.2f}" for z in z_positions)
            planes_after = ", ".join(f"{z:.2f}" for z in z_balanced)
            drops_after = ", ".join(
                f"{100.0 * float(item['drop_rel']):.1f}%"
                for item in items_balanced
            )
            msg = (
                f"{log_prefix} STACK SIMETRIZADO: asimetría "
                f"{ratio_before:.1f}× → "
                f"{ratio_after if ratio_after is not None else 1.0:.1f}× | "
                f"planos {planes_before} → {planes_after}µm "
                f"(ΔS por lado: {drops_after})"
            )
            logger.warning(msg)
            try:
                self.status_message.emit(msg)
            except (AttributeError, RuntimeError):
                pass
            z_positions = z_balanced
            lower_selected = [
                item
                for item in items_balanced
                if float(item["z_um"]) < float(best_z)
            ]
            upper_selected = [
                item
                for item in items_balanced
                if float(item["z_um"]) > float(best_z)
            ]

        logger.info(
            "%s STACK DESDE CURVA MEDIDA: se reutilizan %d candidatos "
            "COARSE/FINE; probes Z adicionales=0; planos=%s",
            log_prefix,
            len(by_z),
            ", ".join(f"{z:.3f}" for z in z_positions),
        )
        try:
            self.status_message.emit(
                f"{log_prefix} STACK desde COARSE+FINE: "
                f"{len(by_z)} mediciones reutilizadas, 0 probes Z adicionales"
            )
        except (AttributeError, RuntimeError):
            pass
        return z_positions, baseline, lower_selected + upper_selected

    def _build_optical_multifocal_plan(
        self,
        *,
        best_z: float,
        best_score: float,
        n_captures: int,
        z_min: float,
        z_max: float,
        rois,
        log_prefix: str,
    ) -> Tuple[list, float, list]:
        """Construye N planos por caída relativa de S, no por µm arbitrarios."""
        n = int(n_captures)
        if n < 1 or n % 2 == 0:
            raise ValueError(f"n_captures debe ser impar ≥1 (recibido: {n})")
        if n == 1:
            return [float(best_z)], float(best_score), []

        outer_drop = max(
            0.001,
            min(0.95, float(getattr(self, "capture_s_drop_rel", 0.10))),
        )
        measured_plan = self._build_plan_from_measured_focus_curve(
            best_z=float(best_z),
            best_score=float(best_score),
            n_captures=n,
            target_drop_rel=outer_drop,
            log_prefix=log_prefix,
        )
        if measured_plan is not None:
            return measured_plan

        baseline_s = float(self.evaluate_s_at_z(float(best_z), rois=rois))
        if baseline_s <= 0.0 or not np.isfinite(baseline_s):
            raise RuntimeError(
                f"{log_prefix} No se pudo verificar S_BPoF antes del stack"
            )

        half = n // 2
        selections = []
        for direction in (-1, 1):
            direction_items = []
            for level in range(1, half + 1):
                direction_items.append(
                    self._seek_optical_drop_plane(
                        best_z=float(best_z),
                        baseline_s=baseline_s,
                        direction=direction,
                        target_drop_rel=outer_drop * level / half,
                        z_min=float(z_min),
                        z_max=float(z_max),
                        rois=rois,
                        log_prefix=log_prefix,
                    )
                )
            if len(
                {round(float(item["z_um"]), 6) for item in direction_items}
            ) != len(direction_items):
                outer = max(
                    direction_items, key=lambda item: item["distance_um"]
                )
                rebuilt = []
                for level in range(1, half + 1):
                    distance = float(outer["distance_um"]) * level / half
                    z_probe = float(best_z) + direction * distance
                    score = float(self.evaluate_s_at_z(z_probe, rois=rois))
                    if score <= 0.0 or not np.isfinite(score):
                        raise RuntimeError(
                            f"{log_prefix} S inválido al separar planos "
                            f"ópticos en Z={z_probe:.3f}µm"
                        )
                    target = outer_drop * level / half
                    drop = max(0.0, (baseline_s - score) / baseline_s)
                    rebuilt.append(
                        {
                            "z_um": z_probe,
                            "score": score,
                            "drop_rel": drop,
                            "distance_um": distance,
                            "target_drop_rel": target,
                            "target_met": bool(drop >= target),
                        }
                    )
                direction_items = rebuilt
            selections.extend(direction_items)

        lower = sorted(
            [item for item in selections if item["z_um"] < float(best_z)],
            key=lambda item: item["z_um"],
        )
        upper = sorted(
            [item for item in selections if item["z_um"] > float(best_z)],
            key=lambda item: item["z_um"],
        )
        lower_ok = len(lower) == half and all(
            bool(item.get("target_met")) for item in lower
        )
        upper_ok = len(upper) == half and all(
            bool(item.get("target_met")) for item in upper
        )

        # Si BPoF cae junto a un límite (caso real: solo 1 µm hacia abajo),
        # no guardar un plano lateral con ΔS=0. Se distribuyen los N−1 planos
        # sobre el lado que sí entrega información, a niveles crecientes de S.
        if not (lower_ok and upper_ok):
            lower_max = max(
                (float(item["drop_rel"]) for item in lower), default=0.0
            )
            upper_max = max(
                (float(item["drop_rel"]) for item in upper), default=0.0
            )
            useful_direction = 1 if upper_max >= lower_max else -1
            available_drop = max(lower_max, upper_max)
            if available_drop <= 1e-6:
                raise RuntimeError(
                    f"{log_prefix} No existe variación óptica medible de S "
                    "en el rango Z calibrado; no se guardará un stack duplicado"
                )

            effective_outer_drop = min(outer_drop, available_drop)
            one_sided = []
            for level in range(1, n):
                effective_target = effective_outer_drop * level / (n - 1)
                item = self._seek_optical_drop_plane(
                    best_z=float(best_z),
                    baseline_s=baseline_s,
                    direction=useful_direction,
                    target_drop_rel=effective_target,
                    z_min=float(z_min),
                    z_max=float(z_max),
                    rois=rois,
                    log_prefix=log_prefix,
                )
                requested_target = outer_drop * level / (n - 1)
                item["requested_target_drop_rel"] = requested_target
                item["requested_target_met"] = bool(
                    float(item["drop_rel"]) >= requested_target
                )
                one_sided.append(item)

            if len(
                {round(float(item["z_um"]), 6) for item in one_sided}
            ) != len(one_sided):
                outer = max(one_sided, key=lambda item: item["distance_um"])
                rebuilt = []
                for level in range(1, n):
                    distance = float(outer["distance_um"]) * level / (n - 1)
                    z_probe = float(best_z) + useful_direction * distance
                    score = float(self.evaluate_s_at_z(z_probe, rois=rois))
                    if score <= 0.0 or not np.isfinite(score):
                        raise RuntimeError(
                            f"{log_prefix} S inválido al reconstruir stack "
                            f"unilateral en Z={z_probe:.3f}µm"
                        )
                    drop = max(0.0, (baseline_s - score) / baseline_s)
                    requested_target = outer_drop * level / (n - 1)
                    rebuilt.append(
                        {
                            "z_um": z_probe,
                            "score": score,
                            "drop_rel": drop,
                            "distance_um": distance,
                            "target_drop_rel": (
                                effective_outer_drop * level / (n - 1)
                            ),
                            "target_met": bool(
                                drop
                                >= effective_outer_drop * level / (n - 1)
                            ),
                            "requested_target_drop_rel": requested_target,
                            "requested_target_met": bool(
                                drop >= requested_target
                            ),
                        }
                    )
                one_sided = rebuilt

            lower = sorted(
                [item for item in one_sided if item["z_um"] < float(best_z)],
                key=lambda item: item["z_um"],
            )
            upper = sorted(
                [item for item in one_sided if item["z_um"] > float(best_z)],
                key=lambda item: item["z_um"],
            )
            logger.warning(
                "%s STACK ÓPTICO UNILATERAL: BPoF próximo a límite o lado "
                "sin ΔS; dirección=%s, ΔS disponible=%.2f%%, solicitado=%.2f%%",
                log_prefix,
                "arriba" if useful_direction > 0 else "abajo",
                100.0 * available_drop,
                100.0 * outer_drop,
            )

        z_positions = (
            [float(item["z_um"]) for item in lower]
            + [float(best_z)]
            + [float(item["z_um"]) for item in upper]
        )
        if len(z_positions) != n or len(
            {round(float(z), 6) for z in z_positions}
        ) != n:
            raise RuntimeError(
                f"{log_prefix} La respuesta óptica no produjo {n} planos "
                f"distintos: {z_positions}"
            )
        return z_positions, baseline_s, lower + upper

    def _resolve_scan_range(self) -> Tuple[float, float, float, float]:
        """Retorna (z_min, z_max, z_center, z_range_total) según configuración."""
        z_current = self.cfocus_controller.read_z()
        if z_current is None or z_current < 0:
            z_current = 0.0
            logger.warning("[Autofocus] No se pudo leer posición Z, usando 0.0µm")

        calib_info = self.cfocus_controller.get_calibration_info()
        if not calib_info['is_calibrated']:
            raise ValueError("C-Focus no calibrado. Ejecutar calibración antes de usar autofoco.")

        z_min_hw = calib_info['z_min']
        z_max_hw = calib_info['z_max']
        z_center_hw = calib_info['z_center']

        # El historial de BPoF de la sesión manda sobre el modo de la interfaz:
        # una vez que la muestra reveló dónde está su plano focal, barrer los
        # 86µm calibrados sólo añade planos de meseta. Los primeros ciclos sí
        # respetan lo pedido en la GUI, porque ahí todavía no hay información.
        prior = self.__dict__.get("_bpof_prior")
        prior_window = None
        if (
            prior is not None
            and bool(self.__dict__.get("coarse_prior_enabled", True))
            and prior.ready
        ):
            prior_window = prior.window(z_min_hw, z_max_hw)

        if prior_window is not None:
            z_min, z_max = prior_window
            z_range_total = z_max - z_min
            self._coarse_window_source = "prior"
            logger.info(
                "[Autofocus] COARSE acotado por prior de sesión: "
                "%.2f→%.2fµm (%.2fµm) | mediana BPoF=%.2fµm "
                "semiancho=%.2fµm (MAD=%.2fµm, n=%d)",
                z_min,
                z_max,
                z_range_total,
                float(prior.center),
                float(prior.half_span_um),
                float(prior.mad_um or 0.0),
                len(prior),
            )
        elif self.use_full_range:
            z_min = z_min_hw
            z_max = z_max_hw
            z_range_total = z_max_hw - z_min_hw
            self._coarse_window_source = "full"
            logger.info(
                f"[Autofocus] ESCANEO COMPLETO: {z_min:.2f} -> {z_max:.2f}um "
                f"(rango total: {z_range_total:.2f}µm)"
            )
        else:
            # Sin historial y sin escaneo completo: ventana asimétrica alrededor
            # del origen calibrado. El pico aparece pasado el origen (43µm en el
            # log de referencia), así que abrir más hacia arriba cuesta lo mismo
            # y falla menos que centrar en la Z actual.
            z_min, z_max = bootstrap_window(
                z_current,
                below_um=float(
                    self.__dict__.get("coarse_bootstrap_below_um", 20.0)
                ),
                above_um=float(
                    self.__dict__.get("coarse_bootstrap_above_um", 25.0)
                ),
                z_min_hw=z_min_hw,
                z_max_hw=z_max_hw,
            )
            z_range_total = z_max - z_min
            self._coarse_window_source = "bootstrap"
            logger.info(
                f"[Autofocus] Escaneo local (arranque): {z_min:.2f}-{z_max:.2f}µm "
                f"({z_range_total:.2f}µm desde Z={z_current:.2f}µm)"
            )

        return z_min, z_max, z_current, z_range_total

    def _focus_surface_sync(
        self,
        objects: List,
        *,
        return_to_z_center: bool = False,
        log_prefix: str = "[Autofocus]",
        microscopy_format: bool = False,
    ) -> List[FocusResult]:
        """
        Un barrido Z para N ROI como superficie (S=ΣS_i).
        Todos los resultados comparten el mismo BPoF.
        """
        del return_to_z_center  # El retorno al origen ahora es obligatorio.
        if not objects:
            raise ValueError("Sin objetos para autofoco superficie")
        focus_cycle_t0 = time.perf_counter()
        kpi = AfCycleKpi()
        self._cycle_kpi = kpi

        rois = self._normalize_rois(rois=objects)
        is_valid, validation_msg = self.validate_scan_range()
        if not is_valid:
            raise ValueError(validation_msg)

        # --- ROI Tracking: crear tracker para esta sesión ---
        self._last_roi_emit_t = 0.0
        if self.roi_tracking_enabled:
            self._roi_tracker = RoiTracker(
                max_drift_px=self.roi_max_drift_px,
                update_threshold_px=self.roi_update_threshold_px,
                enabled=True,
                adopt_contour=bool(self.roi_track_adopt_contour),
                static_window=bool(self.roi_static_window),
                static_pad_px=int(self.roi_margin),
                detect_interval=int(
                    self.__dict__.get("roi_detect_interval", 1)
                ),
            )
            logger.info(
                "%s ROI Tracking HABILITADO: medida=%s, overlay=silueta "
                "detectada en vivo",
                log_prefix,
                (
                    f"1 ROI cuadrado estático por objeto, holgura "
                    f"{int(self.roi_margin)}px/lado"
                    if self.roi_static_window
                    else f"máscara trasladable (max_drift="
                         f"{self.roi_max_drift_px}px)"
                ),
            )
        else:
            self._roi_tracker = None

        z_center_hw = self.cfocus_controller.get_calibration_info()["z_center"]

        # Primero anclar; recién después resolver el rango local. Antes se
        # calculaba alrededor del BPoF previo y luego se volvía al origen,
        # desacoplando físicamente el COARSE de la referencia calibrada.
        ok_origin, z_origin_read, z_origin_cmd = self.goto_calibration_origin(
            log_prefix=log_prefix, emit_status=True
        )
        if not ok_origin:
            raise RuntimeError(
                f"{log_prefix} Autofoco abortado: no se alcanzó el origen "
                f"calibrado Z={z_origin_cmd:.2f}µm "
                f"(read={f'{z_origin_read:.2f}' if z_origin_read is not None else '?'})"
            )

        z_min, z_max, z_center, _z_range = self._resolve_scan_range()
        best_z, best_score = self._optimize_focus_simple(
            rois,
            z_min,
            z_max,
            float(z_center_hw) if z_center_hw is not None else z_center,
            log_prefix=log_prefix,
            microscopy_format=microscopy_format,
        )

        # Revalidar BPoF con flush completo antes del plan fotográfico.
        # Evita stacks anclados a un pico FINE contaminado por frame mid-move.
        confirm_t0 = time.perf_counter()
        s_fine_bpof = float(best_score)
        best_z, best_score = self._confirm_bpof_before_stack(
            float(best_z),
            float(best_score),
            rois=rois,
            log_prefix=log_prefix,
        )
        kpi.t_confirm = time.perf_counter() - confirm_t0
        kpi.z_bpof = float(best_z)

        n_captures = max(1, int(self.n_captures))
        target_s_drop_rel = max(
            0.001,
            min(0.95, float(getattr(self, "capture_s_drop_rel", 0.10))),
        )
        capture_step = 0.0
        hw = self.cfocus_controller.get_calibration_info()
        z_hw_min = float(hw.get("z_min", z_min))
        z_hw_max = float(hw.get("z_max", z_max))

        do_multifocal = n_captures >= 3 and not self.cancel_requested
        frames: list = []
        scores: list = []
        z_reads: List[Optional[float]] = []
        z_positions: list = [best_z]
        center_idx = 0
        optical_plan: list = []
        stack_layout = "centered"

        if do_multifocal:
            z_positions, best_score, optical_plan = (
                self._build_optical_multifocal_plan(
                    best_z=float(best_z),
                    best_score=float(best_score),
                    n_captures=n_captures,
                    z_min=z_hw_min,
                    z_max=z_hw_max,
                    rois=rois,
                    log_prefix=log_prefix,
                )
            )
            center_idx = min(
                range(len(z_positions)),
                key=lambda idx: abs(
                    float(z_positions[idx]) - float(best_z)
                ),
            )
            stack_layout = (
                "centered"
                if center_idx == n_captures // 2
                else (
                    "one_sided_up"
                    if center_idx == 0
                    else "one_sided_down"
                )
            )
            capture_step = round(
                float(
                    np.mean(
                        [
                            abs(float(z) - float(best_z))
                            for z in z_positions
                            if abs(float(z) - float(best_z)) > 1e-9
                        ]
                    )
                ),
                3,
            )
            if len(z_positions) != n_captures or len(
                {round(float(z), 6) for z in z_positions}
            ) != n_captures:
                raise RuntimeError(
                    f"{log_prefix} Plan fotográfico inválido: se pidieron "
                    f"{n_captures} planos Z distintos y se obtuvo "
                    f"{z_positions}. NO SE TOMAN FOTOGRAFÍAS"
                )
            logger.info(
                "%s PLAN FOTOGRÁFICO FINAL (mediciones terminadas): "
                "BPoF=%.2fµm → "
                "%d planos por caída S objetivo=%.2f%% "
                "(distancia media efectiva=%.3fµm, superficie %d ROI): %s",
                log_prefix,
                best_z,
                n_captures,
                100.0 * target_s_drop_rel,
                capture_step,
                len(rois),
                ", ".join(f"{z:.2f}" for z in z_positions),
            )
            self.status_message.emit(
                f"{log_prefix} PLAN FINAL: mediciones completas → BPoF "
                f"{best_z:.2f}µm → {n_captures} fotografías distintas "
                f"(ΔS objetivo {100.0 * target_s_drop_rel:.1f}%; "
                f"distancia Z resultante media {capture_step:.3f}µm)"
            )
            photos_t0 = time.perf_counter()
            for i, z_capture in enumerate(z_positions):
                if self.cancel_requested:
                    break
                photo_msg = (
                    f"{log_prefix} FOTO {i + 1}/{n_captures}: "
                    f"distancia objetivo Z={z_capture:.3f}µm "
                    f"(offset BPoF={z_capture - best_z:+.3f}µm)"
                )
                logger.info(photo_msg)
                self.status_message.emit(photo_msg)
                frame_i, score_i, z_read_i = self._capture_plane_at_z(
                    z_capture,
                    rois=rois,
                    log_prefix=log_prefix,
                    max_attempts=2,
                )
                reached = (
                    z_read_i is not None
                    and abs(float(z_read_i) - float(z_capture))
                    <= float(self.z_arrive_tol_um)
                )
                if (
                    frame_i is None
                    or getattr(frame_i, "size", 0) == 0
                    or not reached
                ):
                    raise RuntimeError(
                        f"{log_prefix} FOTO {i + 1}/{n_captures} cancelada: "
                        f"Z_cmd={z_capture:.3f}µm, "
                        f"Z_read={f'{z_read_i:.3f}' if z_read_i is not None else '?'}. "
                        "Stack incompleto; no se guarda ninguna serie"
                    )
                frames.append(frame_i)
                scores.append(score_i)
                z_reads.append(z_read_i)
                self.score_updated.emit(float(z_read_i), float(score_i))
                self.status_message.emit(
                    f"{log_prefix} S_ROI FOTO {i + 1}/{n_captures}: "
                    f"Z={float(z_read_i):.2f}µm S={float(score_i):.2f}"
                )
            kpi.t_photos = time.perf_counter() - photos_t0
            if len(frames) != n_captures:
                raise RuntimeError(
                    f"{log_prefix} Stack incompleto: {len(frames)}/"
                    f"{n_captures} fotografías; no se guarda ninguna serie"
                )
            self._record_stack_kpi(
                kpi,
                best_z=float(best_z),
                s_reference=s_fine_bpof,
                z_positions=z_positions,
                scores=scores,
                center_idx=center_idx,
                log_prefix=log_prefix,
            )
        else:
            frame_i, score_i = self._capture_at_z(best_z, rois=rois)
            frames = [frame_i]
            scores = [float(score_i) if score_i else float(best_score)]
            z_reads = [self._read_z_um()]

        # Las imágenes ya están en memoria; guardar archivos no requiere
        # permanecer en Z. El ciclo termina aparcado en el BPoF para dejar la
        # vista en el plano enfocado; park_at_bpof=False vuelve al punto medio
        # calibrado. El siguiente barrido reancla en el origen de todos modos.
        if self.park_at_bpof:
            z_park_cmd = float(best_z)
            ok_return, z_final_read = self._goto_z_static(z_park_cmd)
            destino = "BPoF"
            if ok_return:
                self.score_updated.emit(z_park_cmd, float(best_score))
        else:
            ok_return, z_final_read, z_park_cmd = self.goto_calibration_origin(
                log_prefix=log_prefix, emit_status=True
            )
            destino = "origen calibrado"
        if not ok_return:
            raise RuntimeError(
                f"{log_prefix} Fotografías tomadas, pero falló el aparcado en "
                f"{destino} Z={z_park_cmd:.3f}µm "
                f"(read={f'{z_final_read:.3f}' if z_final_read is not None else '?'})"
            )
        msg_final = (
            f"{log_prefix} ✓ Ciclo completo: BPoF={best_z:.2f}µm "
            f"(SΣ={best_score:.1f}, n_ROI={len(rois)}) → "
            f"aparcado en {destino} Z={z_final_read:.2f}µm | "
            f"tiempo={time.perf_counter() - focus_cycle_t0:.2f}s"
        )
        logger.info(msg_final)
        print(msg_final, flush=True)
        self.status_message.emit(msg_final)

        final_frame = (
            frames[center_idx]
            if frames and len(frames) > center_idx
            else (frames[0] if frames else None)
        )
        alt_idx = (
            max(
                range(len(z_positions)),
                key=lambda idx: abs(
                    float(z_positions[idx]) - float(best_z)
                ),
            )
            if z_positions else 0
        )
        frame_alt = frames[alt_idx] if len(frames) > alt_idx else None
        z_alt = z_positions[alt_idx] if z_positions else best_z
        score_alt = scores[alt_idx] if len(scores) > alt_idx else 0.0

        # El bbox reportado debe ser el que se midió en el último plano, no el
        # de la detección inicial: si el objeto se desplazó, el ROI original ya
        # no lo contiene.
        final_rois = rois
        tracker = self.__dict__.get("_roi_tracker")
        if tracker is not None and tracker.is_active:
            tracked = tracker.current_rois
            if len(tracked) == len(rois):
                final_rois = tracked

        kpi.t_total = time.perf_counter() - focus_cycle_t0
        self._publish_cycle_kpi(kpi, best_z=float(best_z))

        results: List[FocusResult] = []
        for i, obj in enumerate(objects):
            bbox_i, contour_i = final_rois[i]
            s_i = best_score
            if final_frame is not None:
                s_i = float(
                    self._calculate_sharpness(final_frame, bbox_i, contour_i)
                )
            results.append(
                FocusResult(
                    object_index=i,
                    z_optimal=best_z,
                    focus_score=s_i,
                    bbox=bbox_i,
                    frame=final_frame,
                    frames=frames,
                    z_positions=z_positions,
                    focus_scores=scores,
                    z_reads=z_reads,
                    capture_step_um=capture_step,
                    capture_mode=(
                        "optical_s_drop"
                        if stack_layout == "centered"
                        else "optical_s_drop_one_sided"
                    ),
                    target_s_drop_rel=target_s_drop_rel,
                    optical_plan=optical_plan,
                    bpof_index=center_idx,
                    stack_layout=stack_layout,
                    frame_alt=frame_alt,
                    z_alt=z_alt,
                    score_alt=score_alt,
                )
            )
        
        # --- Limpiar ROI Tracker ---
        if tracker is not None:
            tracker.reset()
        self._roi_tracker = None

        return results

    def _record_stack_kpi(
        self,
        kpi: AfCycleKpi,
        *,
        best_z: float,
        s_reference: float,
        z_positions: list,
        scores: list,
        center_idx: int,
        log_prefix: str,
    ) -> None:
        """Mide el stack contra lo que se pidió, no contra lo que se planificó.

        Las tres cifras que importan salen de las fotografías, no de la curva:
        ε_foto (¿la S del BPoF se repite al fotografiarlo?), ΔS_stack real
        (¿los planos laterales son distinguibles?) y la asimetría (¿el bracket
        rodea el foco o cuelga de un lado?). En el log de referencia el plan
        pedía 7.5% y las fotos salieron con 5.3%, 0% y 0.4%: indistinguibles.
        """
        if not scores or not z_positions:
            return

        s_center = (
            float(scores[center_idx]) if center_idx < len(scores) else 0.0
        )
        if s_reference > 0.0 and s_center > 0.0:
            kpi.eps_photo = abs(s_center - float(s_reference)) / float(
                s_reference
            )
        if s_center > 0.0:
            laterals = [
                float(score)
                for idx, score in enumerate(scores)
                if idx != center_idx
            ]
            if laterals:
                kpi.delta_s_stack = max(
                    0.0, (s_center - max(laterals)) / s_center
                )
        kpi.stack_asymmetry = stack_asymmetry_ratio(
            [float(z) - float(best_z) for z in z_positions]
        )

        logger.info(
            "%s STACK MEDIDO: S_fotos=%s | ε_foto=%s ΔS_real=%s asimetría=%s",
            log_prefix,
            ", ".join(f"{float(score):.1f}" for score in scores),
            f"{kpi.eps_photo:.1%}" if kpi.eps_photo is not None else "na",
            (
                f"{kpi.delta_s_stack:.1%}"
                if kpi.delta_s_stack is not None
                else "na"
            ),
            (
                f"{kpi.stack_asymmetry:.1f}×"
                if kpi.stack_asymmetry is not None
                else "na"
            ),
        )

    def _publish_cycle_kpi(self, kpi: AfCycleKpi, *, best_z: float) -> None:
        """Publica la línea AF_KPI y alimenta el prior y las medianas."""
        kpi.z_bpof = float(best_z)
        prior = self.__dict__.get("_bpof_prior")
        if prior is not None:
            prior.add(float(best_z))

        line = kpi.format_line()
        logger.info("[Autofocus] %s", line)
        print(line, flush=True)
        try:
            self.status_message.emit(line)
        except (AttributeError, RuntimeError):
            pass

        session = self.__dict__.get("session_kpi")
        if session is None:
            return
        session.add_cycle(kpi)
        every = max(1, int(self.__dict__.get("kpi_session_log_every", 25)))
        if session.n_cycles % every == 0:
            summary = session.summary_line()
            logger.info("[Autofocus] %s", summary)
            print(summary, flush=True)
            try:
                self.status_message.emit(summary)
            except (AttributeError, RuntimeError):
                pass

    def focus_object_sync(
        self,
        obj,
        obj_index: int = 0,
        return_to_z_center: bool = False,
        log_prefix: str = "[Autofocus]",
        microscopy_format: bool = False,
    ) -> AutofocusResult:
        """Compat: un objeto → superficie de 1 ROI."""
        del obj_index
        results = self._focus_surface_sync(
            [obj],
            return_to_z_center=return_to_z_center,
            log_prefix=log_prefix,
            microscopy_format=microscopy_format,
        )
        return results[0]

    def _scan_single_object(self, obj, obj_index: int) -> FocusResult:
        """Microscopía: un objeto (superficie 1 ROI)."""
        del obj_index
        if self.microscopy_mode:
            return self.focus_object_sync(
                obj,
                return_to_z_center=True,
                log_prefix="[MicroscopyService]",
                microscopy_format=True,
            )
        return self.focus_object_sync(
            obj,
            return_to_z_center=True,
            log_prefix="[Autofocus]",
            microscopy_format=False,
        )
    
    def _get_score_at_z(self, z: float, bbox: Tuple[int, int, int, int]) -> float:
        """Compat: mide S en Z con el método único."""
        return float(self.evaluate_s_at_z(z, bbox, None))
    
    def _verify_and_refine_bpof(
        self,
        best_z: float,
        best_score: float,
        bbox,
        contour,
        z_min: float,
        z_max: float,
        log_prefix: str,
    ) -> Tuple[float, float]:
        """Re-evalúa BPoF con settle de captura (mismo evaluate_s_at_z)."""
        step = self.z_step_fine
        candidates = [best_z]
        for dz in (-2 * step, -step, step, 2 * step):
            z_try = max(z_min, min(z_max, best_z + dz))
            if z_try not in candidates:
                candidates.append(z_try)

        best_z_cap = best_z
        best_score_cap = 0.0
        for z_try in candidates:
            score_try = self.evaluate_s_at_z(
                z_try, bbox, contour, settle_s=self.capture_settle_time
            )
            logger.info(
                f"{log_prefix} Refine captura Z={z_try:.2f}µm -> S={score_try:.1f}"
            )
            if score_try > best_score_cap:
                best_score_cap = score_try
                best_z_cap = z_try

        if best_z_cap != best_z:
            logger.warning(
                f"{log_prefix} BPoF ajustado tras verificación: "
                f"Z {best_z:.2f} -> {best_z_cap:.2f}µm, "
                f"S_scan={best_score:.1f} -> S_capture={best_score_cap:.1f}"
            )
        return best_z_cap, best_score_cap if best_score_cap > 0 else best_score

    def _ensure_bpof_at_center(
        self,
        best_z: float,
        best_score: float,
        frames: list,
        scores: list,
        z_positions: list,
        center_idx: int,
        bbox,
        contour,
        log_prefix: str,
        z_min: float = None,
        z_max: float = None,
        n_captures: int = None,
        capture_step: float = None,
    ) -> Tuple[float, float, list, list, list]:
        """El BPoF lo fija la tabla de scan; el stack multi-focal NO lo redefine.

        Si un offset tiene S mayor, solo se registra (asociación óptica distinta
        o settle); Z* permanece el argmax de la tabla de candidatos.
        """
        if not scores or not frames or not z_positions:
            return best_z, best_score, frames, scores, z_positions

        center_score = float(scores[center_idx]) if center_idx < len(scores) else 0.0
        best_cap_idx = int(np.argmax(scores))
        max_s = float(scores[best_cap_idx])

        if best_cap_idx != center_idx and max_s > center_score:
            logger.warning(
                f"{log_prefix} Multi-focal: f{best_cap_idx} S={max_s:.1f} > "
                f"f{center_idx} S={center_score:.1f}; "
                f"se MANTIENE BPoF de tabla Z={best_z:.2f}µm (no se cambia)"
            )
        else:
            logger.info(
                f"{log_prefix} Multi-focal OK en f{center_idx}: "
                f"Z={best_z:.2f}µm, S_cap={center_score:.1f}"
            )
        # Actualizar score reportado con medición de captura en el centro (mismo Z)
        if center_score > 0:
            best_score = center_score
        return best_z, best_score, frames, scores, z_positions

    def _capture_at_z(
        self,
        z: float,
        bbox=None,
        contour=None,
        settle_s: Optional[float] = None,
        *,
        rois=None,
    ) -> Tuple[Optional[np.ndarray], float]:
        """Captura frame + S en Z (superficie si rois)."""
        del settle_s
        score, frame = self.evaluate_s_at_z(
            z, bbox, contour, rois=rois, return_frame=True
        )
        return frame, float(score)

    def _capture_plane_at_z(
        self,
        z: float,
        bbox=None,
        contour=None,
        *,
        rois=None,
        settle_s: Optional[float] = None,
        log_prefix: str = "[Autofocus]",
        z_tol_um: Optional[float] = None,
        max_attempts: int = 1,
    ) -> Tuple[Optional[np.ndarray], float, Optional[float]]:
        """Captura plano: una sola pasada evaluate_s_at_z (S superficie si rois)."""
        del settle_s
        z_cmd = float(z)
        tol = float(self.z_arrive_tol_um if z_tol_um is None else z_tol_um)
        last_frame: Optional[np.ndarray] = None
        last_score = 0.0
        last_read: Optional[float] = None

        for attempt in range(1, max_attempts + 1):
            if self.cancel_requested:
                break
            score, frame = self.evaluate_s_at_z(
                z_cmd, bbox, contour, rois=rois, return_frame=True
            )
            z_read = self._read_z_um()
            last_frame, last_score, last_read = frame, float(score), z_read

            ok_frame = frame is not None and getattr(frame, "size", 0) > 0
            ok_z = z_read is not None and abs(float(z_read) - z_cmd) <= tol
            if ok_frame and ok_z:
                return frame, float(score), z_read

            logger.warning(
                "%s Plano Z=%.2fµm intento %d/%d: frame=%s Z_read=%s "
                "(tol=±%.2fµm)",
                log_prefix,
                z_cmd,
                attempt,
                max_attempts,
                "ok" if ok_frame else "None",
                f"{z_read:.2f}" if z_read is not None else "?",
                tol,
            )

        return last_frame, last_score, last_read

    def _read_z_um(self) -> Optional[float]:
        """Lee Z actual del piezo; None si no hay controlador/lectura."""
        cfocus = getattr(self, "cfocus_controller", None)
        if cfocus is None:
            return None
        try:
            z = cfocus.read_z()
            return float(z) if z is not None else None
        except Exception:
            return None

    def _wait_for_new_frames(self, n_new: int = 2, timeout_s: float = 2.0) -> bool:
        """Espera N frames nuevos reales (por frame_count). False si cancela/timeout."""
        if n_new <= 0:
            return not self.cancel_requested
        if not self.get_frame_count_callback:
            raise RuntimeError(
                "AutofocusService sin get_frame_count_callback: "
                "inicializar autofoco con cámara en vivo"
            )

        start = int(self.get_frame_count_callback())
        target = start + int(n_new)
        deadline = time.perf_counter() + float(timeout_s)
        while True:
            if self.cancel_requested:
                return False
            current = int(self.get_frame_count_callback())
            if current >= target:
                return True
            if time.perf_counter() >= deadline:
                logger.warning(
                    "[Autofocus] Timeout esperando frames nuevos "
                    f"(start={start}, now={current}, target={target}) — NO se usa frame stale"
                )
                return False
            if not self._sleep_interruptible(0.01):
                return False

    def _get_fresh_raw16_frame(self, timeout_s: float = 2.0) -> Optional[np.ndarray]:
        """Adquiere ScientificFrame por la única vía CMOS."""
        acquire = self.__dict__.get("acquire_scientific_frame_callback")
        if not callable(acquire):
            raise RuntimeError(
                "[AutofocusService] Sin acquire_scientific_frame_callback"
            )
        if self.cancel_requested:
            return None
        sci = acquire(timeout_s=float(timeout_s))
        if sci is None:
            return None
        img = getattr(sci, "image16", None)
        if img is None or getattr(img, "size", 0) == 0:
            return None
        self._scientific_frame_id = int(getattr(sci, "frame_id", 0) or 0)
        return np.asarray(img, dtype=np.uint16).copy()

    def _get_fresh_frame(self, n_new: int = 3) -> Optional[np.ndarray]:
        """Única vía: acquire_scientific_frame (nunca preview/current_frame)."""
        _ = n_new  # flush real lo hace el acquire fresco del worker
        frame = self._get_fresh_raw16_frame()
        if frame is None or getattr(frame, "size", 0) == 0:
            return None
        arr = np.asarray(frame)
        frame_hw = (int(arr.shape[0]), int(arr.shape[1]))
        expected_hw = self.__dict__.get("_native_frame_hw")
        if expected_hw is None:
            self._native_frame_hw = frame_hw
            logger.info(
                "[AutofocusService] Resolución AF nativa fijada: %dx%d "
                "(sin resize; vía acquire_scientific_frame)",
                frame_hw[1],
                frame_hw[0],
            )
        elif frame_hw != expected_hw:
            raise RuntimeError(
                "Resolución AF alterada: "
                f"esperada={expected_hw[1]}x{expected_hw[0]}, "
                f"recibida={frame_hw[1]}x{frame_hw[0]}. "
                "Se rechaza preview/downsampling para no perder detalle."
            )
        return arr.copy()

    def _get_stable_score(
        self,
        bbox: Tuple[int, int, int, int],
        contour: np.ndarray = None,
        n_samples: int = 1,
        flush_buffer: bool = True,
        n_flush_frames: int = 3,
    ) -> float:
        """Compat legado: sin Z nuevo, solo flush + S (preferir evaluate_s_at_z)."""
        _ = flush_buffer
        frame = self._get_fresh_frame(n_new=max(1, int(n_flush_frames)))
        if frame is None or getattr(frame, "size", 0) == 0:
            logger.warning("[Autofocus] Frame inválido para bbox=%s", bbox)
            return 0.0
        return self._calculate_sharpness(frame, bbox, contour)

    def _calculate_sharpness(
        self,
        frame: np.ndarray,
        bbox: Tuple[int, int, int, int],
        contour: np.ndarray = None,
    ) -> float:
        """Delega en focus_metric (única fórmula del índice S)."""
        if contour is None:
            contour = bbox_to_contour(bbox)

        score, details = calculate_focus_score_detailed(
            frame, bbox, contour, self.roi_margin
        )
        x, y, w, h = details["bbox"]
        logger.debug(
            "[Autofocus] S=%.3f %s input=%dbit señal=%dbit cálculo=%s/%dbit | "
            "Ten=%.1f Lap=%.1f Brenner=%.1f HP=%.1f | "
            "bbox=(%d,%d,%d,%d) ROI=%dx%d mask=%d/%dpx",
            score,
            details.get("metric_version", "CLAHE-HF"),
            details.get("input_storage_bits", 0),
            details.get("input_signal_bits", 0),
            details.get("compute_dtype", "?"),
            details.get("compute_mantissa_bits", 0),
            details.get("tenengrad_sqrt", 0.0),
            details.get("lap_sqrt", 0.0),
            details.get("brenner_sqrt", 0.0),
            details.get("highpass_sqrt", 0.0),
            x, y, w, h,
            details["roi_w"], details["roi_h"], details.get("mask_pixels", 0),
            details.get("inner_mask_pixels", 0),
        )
        return score

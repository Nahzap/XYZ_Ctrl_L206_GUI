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
from core.autofocus.bpof_candidates import (
    BpofCandidateTable,
    build_fine_z_planes,
)

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
        self.z_scan_range = 70.0       # µm - límite ±Δ fine alrededor del max S coarse
        self.use_full_range = True     # Si True, escanea todo el rango calibrado; si False, usa z_scan_range
        self.z_step_coarse = 1.0       # µm - paso grueso OPTIMIZADO para velocidad (reducir pasos totales)
        self.z_step_fine = 0.05        # µm - paso real FINE entre candidatos
        self.n_fine_planes = 15        # capas FINE impares, centradas en Z_c*
        self.refine_window = 2.0       # legacy; la zona fine usa z_scan_range
        # Condición de llegada a Z (NO sleep fijo de settle)
        self.z_arrive_tol_um = 0.5     # µm - |Z_read−Z_cmd| ≤ tol
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
        self.coarse_near_max_rel = 0.005
        self.coarse_early_stop_patience = 4
        self.coarse_early_stop_drop_rel = 0.03
        self.bpof_min_relative_span = 0.005
        self.bpof_min_prominence_rel = 0.003
        
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
        
        # Registro del máximo Z encontrado (para optimizar futuros escaneos)
        self.z_max_recorded = None  # Se actualiza tras primer escaneo completo
        
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
            'algorithm': 'hill_climbing'
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
        """Limpia el latch de cancelación dejado por un hard-stop anterior."""
        self.cancel_requested = False
        self.running = False

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
        """S superficie = Σ S_i (un frame estático, varios ROI)."""
        total = 0.0
        for bbox, contour in rois:
            total += float(self._calculate_sharpness(frame, bbox, contour))
        return total

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
                break

        print()
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

        # El primer candidato FINE es exactamente la salida COARSE; después se
        # recorren los demás planos del vecindario. Esto hace explícito el
        # encadenamiento y permite comparar S_coarse vs S_fine en el mismo Z.
        center_i = min(
            range(len(fine_planes)),
            key=lambda i: abs(float(fine_planes[i]) - float(z_coarse_star)),
        )
        fine_sequence = [fine_planes[center_i]] + [
            z for i, z in enumerate(fine_planes) if i != center_i
        ]

        # --- 3) TABLA FINE → BPoF = argmax ---
        for refine_iteration, z_refine in enumerate(fine_sequence, start=1):
            if self.cancel_requested:
                break

            self.progress_updated.emit(
                refine_iteration, total_refine_steps, "Refinamiento fino"
            )

            score = self.evaluate_s_at_z(z_refine, rois=rois)
            if self.cancel_requested:
                break

            # El primer FINE repite exactamente Z_coarse*. Una discrepancia
            # grande delata frame transitorio del barrido grueso; una segunda
            # toma nueva tiene autoridad y evita centrar FINE en un falso pico.
            if (
                refine_iteration == 1
                and abs(float(z_refine) - float(z_coarse_star)) <= 1e-6
                and float(s_coarse_star) > 1e-9
                and abs(float(score) - float(s_coarse_star))
                / float(s_coarse_star) > 0.12
            ):
                first_score = float(score)
                retry_score = float(
                    self.evaluate_s_at_z(z_refine, rois=rois)
                )
                if retry_score > 0.0 and np.isfinite(retry_score):
                    score = retry_score
                    retry_msg = (
                        f"{log_prefix} FINE ancla revalidada: "
                        f"Z={z_refine:.2f}µm S1={first_score:.2f} "
                        f"S2={retry_score:.2f}; S2 tiene autoridad"
                    )
                    logger.warning(retry_msg)
                    self.status_message.emit(retry_msg)

            tabla_fine.add(z_refine, score)
            self.score_updated.emit(z_refine, score)

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

        print()

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

        step = max(0.5, float(getattr(self, "z_step_fine", 1.0) or 1.0))
        candidates = [z0, z0 - step, z0 + step]
        best_zc, best_sc = z0, max(0.0, s_confirm)
        for z_try in candidates:
            if self.cancel_requested:
                break
            s_try = float(self.evaluate_s_at_z(float(z_try), rois=rois))
            if s_try > best_sc:
                best_zc, best_sc = float(z_try), s_try

        msg = (
            f"{log_prefix} BPoF reanclado tras confirmación: "
            f"Z_fine={z0:.2f}µm S={s_fine:.1f} → "
            f"Z={best_zc:.2f}µm S={best_sc:.1f} "
            f"(caída confirm={(s_fine - s_confirm) / s_fine if s_fine > 0 else 0:.1%})"
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

        # En Z repetido, FINE tiene autoridad sobre COARSE.
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

        baseline = float(best_score)
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

        if self.use_full_range:
            z_min = z_min_hw
            z_max = z_max_hw
            z_range_total = z_max_hw - z_min_hw
            logger.info(
                f"[Autofocus] ESCANEO COMPLETO: {z_min:.2f} -> {z_max:.2f}um "
                f"(rango total: {z_range_total:.2f}µm)"
            )
        else:
            z_min = max(z_min_hw, z_current - self.z_scan_range)
            z_max = min(z_max_hw, z_current + self.z_scan_range)
            z_range_total = z_max - z_min
            logger.info(
                f"[Autofocus] Escaneo local: {z_min:.2f}-{z_max:.2f}µm "
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

        rois = self._normalize_rois(rois=objects)
        is_valid, validation_msg = self.validate_scan_range()
        if not is_valid:
            raise ValueError(validation_msg)

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
        best_z, best_score = self._confirm_bpof_before_stack(
            float(best_z),
            float(best_score),
            rois=rois,
            log_prefix=log_prefix,
        )

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
            if len(frames) != n_captures:
                raise RuntimeError(
                    f"{log_prefix} Stack incompleto: {len(frames)}/"
                    f"{n_captures} fotografías; no se guarda ninguna serie"
                )
        else:
            frame_i, score_i = self._capture_at_z(best_z, rois=rois)
            frames = [frame_i]
            scores = [float(score_i) if score_i else float(best_score)]
            z_reads = [self._read_z_um()]

        # Las imágenes ya están en memoria. El ciclo óptico termina siempre en
        # el punto medio calibrado; guardar archivos no requiere permanecer en Z.
        ok_return, z_final_read, z_origin_cmd = self.goto_calibration_origin(
            log_prefix=log_prefix, emit_status=True
        )
        if not ok_return:
            raise RuntimeError(
                f"{log_prefix} Fotografías tomadas, pero falló retorno al "
                f"origen calibrado Z={z_origin_cmd:.3f}µm "
                f"(read={f'{z_final_read:.3f}' if z_final_read is not None else '?'})"
            )
        msg_final = (
            f"{log_prefix} ✓ Ciclo completo: BPoF={best_z:.2f}µm "
            f"(SΣ={best_score:.1f}, n_ROI={len(rois)}) → "
            f"origen calibrado Z={z_final_read:.2f}µm | "
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

        results: List[FocusResult] = []
        for i, obj in enumerate(objects):
            bbox_i, contour_i = rois[i]
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
        return results

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

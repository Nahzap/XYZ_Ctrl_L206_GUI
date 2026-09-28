"""
Camera Orchestrator - Orquestador de Cámara
============================================

Orquesta las operaciones de cámara, detección, autofoco y captura.
Extrae la lógica de negocio de CameraTab para mejorar testabilidad.

Autor: Sistema de Control L206
Fecha: 2025-12-29
"""

import logging
import time
import numpy as np
import cv2
from typing import Optional, List

from PyQt5.QtCore import QObject, pyqtSignal

from core.models import DetectedObject, AutofocusConfig
from core.autofocus.center_then_af import (
    CenterContext,
    CenterResult,
    CenterThenAf,
)
from core.autofocus.center_candidate import FORCE_CENTER_PX, can_start_zscan, pick_tracked_primary

logger = logging.getLogger('MotorControl_L206')


class CameraOrchestrator(QObject):
    """
    Orquestador de operaciones de cámara.
    
    Coordina:
    - CameraService: Adquisición de frames
    - DetectionService: Detección de objetos (U2-Net)
    - AutofocusService: Z-scanning para BPoF
    - SmartFocusScorer: Evaluación de nitidez
    
    Signals:
        autofocus_started: Emitido al iniciar autofoco
        autofocus_complete: Emitido al completar autofoco (results)
        detection_complete: Emitido al completar detección (objects)
        validation_error: Emitido si hay error de validación (message)
        status_message: Mensajes de estado para UI (message)
    """
    
    # Señales
    autofocus_started = pyqtSignal()
    autofocus_complete = pyqtSignal(list)  # List[AutofocusResult]
    detection_complete = pyqtSignal(list)  # List[DetectedObject]
    validation_error = pyqtSignal(str)
    status_message = pyqtSignal(str)
    center_sign_changed = pyqtSignal(int, int)
    
    def __init__(self, camera_service, detection_service, 
                 autofocus_service, smart_focus_scorer):
        """
        Inicializa el orquestador.
        
        Args:
            camera_service: Instancia de CameraService
            detection_service: Instancia de DetectionService
            autofocus_service: Instancia de AutofocusService
            smart_focus_scorer: Instancia de SmartFocusScorer
        """
        super().__init__()
        self.camera = camera_service
        self.detection = detection_service
        self.autofocus = autofocus_service
        self.scorer = smart_focus_scorer
        
        # Estado interno
        self._pending_capture = False
        self._current_frame = None
        self._af_min_area = 0.0
        self._af_max_area = float("inf")
        self._center_runner = CenterThenAf(
            parent=self,
            grab_bgr_frame=self._grab_bgr_frame,
            filter_objects=self._filter_manual_objects,
            log_prefix="[CameraOrchestrator]",
        )
        self._center_runner.set_scorer(lambda: self.scorer)
        self._center_runner.completed.connect(self._on_center_completed)
        self._center_runner.status_message.connect(self.status_message)
        self._center_runner.sign_changed.connect(self.center_sign_changed)
        self._center_runner.overlay_update.connect(self._on_center_overlay)
        if autofocus_service is not None and hasattr(autofocus_service, "scan_complete"):
            autofocus_service.scan_complete.connect(self._on_af_scan_complete)
    
    def is_centering(self) -> bool:
        runner = getattr(self, "_center_runner", None)
        return bool(runner is not None and runner.is_pending())

    def set_current_frame(self, frame: np.ndarray):
        """Actualiza el frame actual."""
        self._current_frame = frame

    @staticmethod
    def _frame_to_bgr(frame) -> Optional[np.ndarray]:
        if frame is None:
            return None
        src = frame.copy()
        if len(src.shape) == 2:
            if src.dtype == np.uint16:
                frame_max = src.max()
                if frame_max > 0:
                    gray_uint8 = (src / frame_max * 255).astype(np.uint8)
                else:
                    gray_uint8 = np.zeros(src.shape, dtype=np.uint8)
                return cv2.cvtColor(gray_uint8, cv2.COLOR_GRAY2BGR)
            return cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)
        if src.dtype == np.uint16:
            frame_max = src.max()
            if frame_max > 0:
                return (src / frame_max * 255).astype(np.uint8)
            return np.zeros(src.shape[:2] + (3,), dtype=np.uint8)
        return src.astype(np.uint8)

    def _grab_bgr_frame(self):
        if self.camera is not None and hasattr(self.camera, "acquire_scientific_frame"):
            try:
                sci = self.camera.acquire_scientific_frame(timeout_s=1.5)
                return self._frame_to_bgr(sci.image16)
            except Exception as exc:
                logger.warning(
                    "[CameraOrchestrator] acquire_scientific_frame: %s", exc
                )
        return self._frame_to_bgr(self._current_frame)

    def _filter_manual_objects(self, objects) -> list:
        lo, hi = float(self._af_min_area), float(self._af_max_area)
        return [
            obj for obj in (objects or [])
            if lo <= float(getattr(obj, "area", 0) or 0) <= hi
        ]

    def _finish_standalone_jog(self, reason: str = "") -> None:
        ts = self._center_runner._test_service
        if ts is not None and hasattr(ts, "finish_standalone_jog"):
            ts.finish_standalone_jog(reason)

    def _start_zscan(self, objects: List) -> None:
        if self.autofocus is None:
            self.validation_error.emit("AutofocusService no disponible")
            self._finish_standalone_jog("no_autofocus")
            if self._pending_capture:
                self.autofocus_complete.emit([])
            return
        ts = self._center_runner._test_service
        if ts is not None and hasattr(ts, "pause_xy_for_capture"):
            ts.pause_xy_for_capture("manual_af_zscan")
        if hasattr(self.autofocus, "_pending_point_index"):
            self.autofocus._pending_point_index = None
        self.autofocus._pending_center_kpi = dict(getattr(self, "_last_center_kpi", {}) or {})
        self.status_message.emit("🎯 Iniciando Z-scan autofoco...")
        self.autofocus_started.emit()
        started = self.autofocus.start_autofocus(objects)
        if not started:
            self._finish_standalone_jog("af_start_fail")
            if self._pending_capture:
                self.autofocus_complete.emit([])

    def _on_af_scan_complete(self, _results) -> None:
        self._finish_standalone_jog("af_complete")

    def _on_center_overlay(self, objects) -> None:
        """Actualiza cruces Δpx del overlay durante el lazo de centrado."""
        objs = list(objects or [])
        if objs:
            self.detection_complete.emit(objs)

    def _on_center_completed(self, result: CenterResult) -> None:
        self._last_center_kpi = dict(result.kpi or {})
        objects = list(result.objects or [])
        if not objects and result.primary is not None:
            objects = [result.primary]
        residual = (result.kpi or {}).get("xy_offset_post_px")
        tau = float((result.kpi or {}).get("tau_px") or FORCE_CENTER_PX)
        if not can_start_zscan(
            action=result.action,
            reason=result.reason,
            center_attempted=bool((result.kpi or {}).get("center_attempted")),
            residual_px=residual if residual is not None else None,
            tau_px=tau,
            objects_found=bool(objects),
        ):
            logger.warning(
                "[CameraOrchestrator] NO Z-scan: reason=%s objects=%d residual=%s",
                result.reason,
                len(objects),
                residual,
            )
            self.status_message.emit(
                f"❌ NO Z-scan ({result.reason}): sin objeto o lost_lock"
            )
            self._finish_standalone_jog(result.reason or "no_object")
            self.autofocus_complete.emit([])
            return
        attempted = bool((result.kpi or {}).get("center_attempted"))
        use_lock = attempted or str(result.reason) in ("already_centered", "centered")
        if use_lock and result.primary is not None:
            z_objects = [result.primary]
        else:
            z_objects = objects
        if len(z_objects) > 1:
            self.status_message.emit(
                f"🎯 Autofoco superficie: {len(z_objects)} ROI en 1 solo barrido Z "
                f"(S = Σ S_i por plano)"
            )
        self.detection_complete.emit(z_objects)
        self._start_zscan(z_objects)
    
    def run_autofocus(
        self,
        capture_after: bool = False,
        min_area: float = 0,
        max_area: float = float("inf"),
        center_ctx: Optional[CenterContext] = None,
        test_service=None,
    ) -> None:
        """
        Ejecuta detección de objetos + (opcional) jog XY + autofoco.

        Flujo:
        1. Obtiene frame actual de cámara
        2. Detecta objetos con SmartFocusScorer
        3. Filtra por rango de área
        4. Si el centrado está ON y hay offset/clip: jog XY → settle → re-detectar
        5. Inicia Z-scan asíncrono
        6. Opcionalmente captura después
        """
        if self._current_frame is None:
            self.validation_error.emit("No hay frame disponible")
            return
        
        current_frame = self._current_frame
        
        if self.scorer is None:
            self.validation_error.emit("SmartFocusScorer no disponible")
            return

        self._af_min_area = float(min_area)
        self._af_max_area = float(max_area)
        self._pending_capture = capture_after
        self._last_center_kpi = {}
        
        self.status_message.emit("🔍 Detectando objetos...")
        
        frame_bgr = self._frame_to_bgr(current_frame)
        if frame_bgr is None:
            self.validation_error.emit("No hay frame disponible")
            return
        
        h_frame, w_frame = frame_bgr.shape[:2]
        logger.info(
            "[CameraOrchestrator] Frame para detección: %sx%s (mantiene dimensiones RAW)",
            w_frame,
            h_frame,
        )
        
        detect_t0 = time.perf_counter()
        result = self.scorer.assess_image(frame_bgr)
        t_detect = time.perf_counter() - detect_t0
        all_objects = result.objects if result.objects else []
        
        logger.info(
            "[CameraOrchestrator] ✅ Bounding boxes en escala correcta (%sx%s)",
            w_frame,
            h_frame,
        )
        
        objects = self._filter_manual_objects(all_objects)
        
        if not objects:
            msg = f"⚠️ No hay objetos en rango [{min_area}-{max_area}] px"
            self.status_message.emit(msg)
            self.status_message.emit(f"   (Detectados {len(all_objects)} objetos totales)")
            
            if capture_after:
                self.status_message.emit("   Capturando sin autofoco...")
                self.autofocus_complete.emit([])
            return
        
        self.status_message.emit(
            f"✅ {len(objects)} objeto(s) en rango (de {len(all_objects)} detectados)"
        )
        for i, obj in enumerate(objects):
            self.status_message.emit(
                f"   #{i+1}: área={obj.area:.0f}px, score={obj.focus_score:.1f}"
            )

        primary = pick_tracked_primary(objects)
        if primary is not None:
            try:
                setattr(primary, "_center_locked", True)
            except Exception:
                pass
        self.detection_complete.emit(objects)

        if self.autofocus is None:
            self.validation_error.emit("AutofocusService no disponible")
            if capture_after:
                self.autofocus_complete.emit([])
            return

        ctx = center_ctx
        if ctx is None:
            ctx = CenterContext(
                enabled=False,
                t_detect_s=t_detect,
                log_prefix="[CameraOrchestrator]",
            )
        else:
            ctx.t_detect_s = t_detect
            ctx.log_prefix = ctx.log_prefix or "[CameraOrchestrator]"

        if test_service is not None:
            self._center_runner.bind_test_service(test_service)
            ctx.has_stage = bool(
                (
                    hasattr(test_service, "goto_xy_um")
                    or hasattr(test_service, "start_center_jog")
                )
                and (
                    getattr(test_service, "_controller_a", None) is not None
                    or getattr(test_service, "_controller_b", None) is not None
                )
            )
            if ctx.has_stage:
                sx, sy, _ex, _ey = test_service.read_current_position_um(
                    ctx.stage_x_um, ctx.stage_y_um
                )
                if sx is not None:
                    ctx.stage_x_um = float(sx)
                if sy is not None:
                    ctx.stage_y_um = float(sy)
            else:
                logger.error(
                    "[CameraOrchestrator] AF_CENTER: stage no listo "
                    "(controladores / TestService)"
                )

        self._center_runner.begin(objects, frame_bgr, ctx)
    
    def validate_autofocus_params(self, config: AutofocusConfig, 
                                  cfocus_limits: Optional[dict] = None) -> tuple:
        """
        Valida parámetros de autofoco.
        
        Args:
            config: Configuración de autofoco
            cfocus_limits: Límites del C-Focus {'z_min': float, 'z_max': float, 'current_z': float}
        
        Returns:
            (is_valid, error_message)
        """
        # Validación básica de parámetros
        is_valid, error = config.validate()
        if not is_valid:
            return False, error
        
        # Validación contra límites del C-Focus
        if cfocus_limits:
            z_min = cfocus_limits.get('z_min', 0)
            z_max = cfocus_limits.get('z_max', 1000)
            current_z = cfocus_limits.get('current_z', 500)
            
            is_valid, error = config.validate_against_cfocus_limits(z_min, z_max, current_z)
            if not is_valid:
                return False, error
        
        return True, None
    
    def update_autofocus_params(self, config: AutofocusConfig) -> bool:
        """
        Actualiza parámetros de autofoco en el servicio.
        
        Args:
            config: Nueva configuración
        
        Returns:
            True si se actualizó correctamente
        """
        if self.autofocus is None:
            self.validation_error.emit("AutofocusService no disponible")
            return False
        
        # Validar y aplicar exactamente lo que viene del JSON/UI (sin clamps ni reescritura)
        is_valid, error = config.validate()
        if not is_valid:
            self.validation_error.emit(f"Configuración inválida: {error}")
            return False

        coarse = float(config.z_step_coarse)
        fine = float(config.z_step_fine)
        roi_margin = int(config.roi_margin)
        capture_s_drop_percent = float(config.z_step_capture)
        n_captures = int(config.n_captures)
        n_fine = int(getattr(config, "n_fine_planes", 15))
        z_tol = float(getattr(config, "z_arrive_tol_um", 0.5))

        self.autofocus.use_full_range = bool(config.use_full_range)
        self.autofocus.z_scan_range = float(config.z_scan_range)
        self.autofocus.z_step_coarse = coarse
        self.autofocus.z_step_fine = fine
        self.autofocus.n_fine_planes = n_fine
        self.autofocus.z_arrive_tol_um = z_tol
        self.autofocus.z_arrive_timeout_s = float(
            getattr(config, "z_arrive_timeout_s", 3.0)
        )
        # Ya no hay sleep de settle; valores a 0 por compat
        self.autofocus.settle_time = 0.0
        self.autofocus.capture_settle_time = 0.0
        self.autofocus.roi_margin = roi_margin
        self.autofocus.max_coarse_iterations = int(config.max_coarse_iterations)
        self.autofocus.max_fine_iterations = int(config.max_fine_iterations)
        self.autofocus.n_captures = n_captures
        # ``z_step_capture`` conserva el nombre por compatibilidad JSON/UI,
        # pero desde v3 representa porcentaje óptico, no micrómetros.
        self.autofocus.z_step_capture = capture_s_drop_percent
        self.autofocus.capture_step = capture_s_drop_percent
        self.autofocus.capture_s_drop_rel = capture_s_drop_percent / 100.0
        self.autofocus.z_range_capture = float(config.z_range_capture)

        self.status_message.emit(
            f"✅ Autofoco encadenado: coarse={coarse:.2f}µm → "
            f"fine paso={fine:.3f}µm, N={n_fine}, "
            f"Δmáx=±{float(config.z_scan_range):.1f}µm → BPoF → "
            f"{n_captures} capturas por ΔS={capture_s_drop_percent:.1f}% | "
            f"tolZ=±{z_tol:.2f}µm margin={roi_margin}px"
        )
        return True
    
    def get_autofocus_search_info(self) -> Optional[dict]:
        """
        Obtiene información estimada de búsqueda de autofoco.
        
        Returns:
            dict con estimaciones o None si no hay servicio
        """
        if self.autofocus is None:
            return None
        
        return self.autofocus.get_search_info()
    
    def update_scorer_morphology_params(self, min_circularity: float = 0.0, 
                                       min_aspect_ratio: float = 0.0) -> bool:
        """
        Actualiza parámetros morfológicos del scorer.
        
        Args:
            min_circularity: Circularidad mínima [0-1]
            min_aspect_ratio: Aspect ratio mínimo [0-1]
        
        Returns:
            True si se actualizó correctamente
        """
        if self.scorer is None:
            self.validation_error.emit("SmartFocusScorer no disponible")
            return False
        
        if not hasattr(self.scorer, 'set_morphology_params'):
            self.validation_error.emit("SmartFocusScorer no soporta parámetros morfológicos")
            return False
        
        self.scorer.set_morphology_params(
            min_circularity=min_circularity,
            min_aspect_ratio=min_aspect_ratio
        )
        
        self.status_message.emit(f"✅ Filtros morfológicos: circ≥{min_circularity:.2f}, aspect≥{min_aspect_ratio:.2f}")
        return True
    
    def is_pending_capture(self) -> bool:
        """Indica si hay captura pendiente después de autofoco."""
        return self._pending_capture
    
    def clear_pending_capture(self):
        """Limpia flag de captura pendiente."""
        self._pending_capture = False

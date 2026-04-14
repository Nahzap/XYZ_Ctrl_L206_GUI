"""
Camera Orchestrator - Orquestador de Cámara
============================================

Orquesta las operaciones de cámara, detección, autofoco y captura.
Extrae la lógica de negocio de CameraTab para mejorar testabilidad.

Autor: Sistema de Control L206
Fecha: 2025-12-29
"""

import logging
import numpy as np
import cv2
from typing import Optional, List

from PyQt5.QtCore import QObject, pyqtSignal

from core.models import DetectedObject, AutofocusConfig

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
    
    def set_current_frame(self, frame: np.ndarray):
        """Actualiza el frame actual."""
        self._current_frame = frame
    
    def run_autofocus(self, capture_after: bool = False, 
                     min_area: float = 0, max_area: float = float('inf')) -> None:
        """
        Ejecuta detección de objetos + autofoco.
        
        Flujo:
        1. Obtiene frame actual de cámara
        2. Detecta objetos con SmartFocusScorer
        3. Filtra por rango de área
        4. Inicia autofoco asíncrono
        5. Opcionalmente captura después
        
        Args:
            capture_after: Si debe capturar imagen después del autofoco
            min_area: Área mínima de objetos (px²)
            max_area: Área máxima de objetos (px²)
        """
        # Validar que hay frame disponible
        if self._current_frame is None:
            self.validation_error.emit("No hay frame disponible")
            return
        
        current_frame = self._current_frame
        
        # Validar que scorer está disponible
        if self.scorer is None:
            self.validation_error.emit("SmartFocusScorer no disponible")
            return
        
        self.status_message.emit("🔍 Detectando objetos...")
        
        # CRÍTICO: Usar el MISMO frame para detección Y autofoco
        # NO convertir ni normalizar - usar frame RAW directamente
        # SmartFocusScorer debe manejar internamente la conversión uint16->uint8
        
        frame = current_frame.copy()
        
        # Convertir a BGR si es necesario (pero mantener dimensiones originales)
        if len(frame.shape) == 2:
            # Grayscale -> BGR (para SmartFocusScorer)
            if frame.dtype == np.uint16:
                # Normalizar uint16 -> uint8 MANTENIENDO dimensiones
                frame_max = frame.max()
                if frame_max > 0:
                    gray_uint8 = (frame / frame_max * 255).astype(np.uint8)
                else:
                    gray_uint8 = np.zeros(frame.shape, dtype=np.uint8)
                frame_bgr = cv2.cvtColor(gray_uint8, cv2.COLOR_GRAY2BGR)
            else:
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            # Ya es color
            if frame.dtype == np.uint16:
                frame_max = frame.max()
                if frame_max > 0:
                    frame_bgr = (frame / frame_max * 255).astype(np.uint8)
                else:
                    frame_bgr = np.zeros(frame.shape[:2] + (3,), dtype=np.uint8)
            else:
                frame_bgr = frame.astype(np.uint8)
        
        # Detectar objetos en frame con dimensiones RAW
        h_frame, w_frame = frame_bgr.shape[:2]
        logger.info(f"[CameraOrchestrator] Frame para detección: {w_frame}x{h_frame} (mantiene dimensiones RAW)")
        
        result = self.scorer.assess_image(frame_bgr)
        all_objects = result.objects if result.objects else []
        
        # Las coordenadas de bbox ahora están en las dimensiones correctas del frame RAW
        logger.info(f"[CameraOrchestrator] ✅ Bounding boxes en escala correcta ({w_frame}x{h_frame})")
        
        # Filtrar por rango de área
        objects = [obj for obj in all_objects if min_area <= obj.area <= max_area]
        
        if not objects:
            msg = f"⚠️ No hay objetos en rango [{min_area}-{max_area}] px"
            self.status_message.emit(msg)
            self.status_message.emit(f"   (Detectados {len(all_objects)} objetos totales)")
            
            if capture_after:
                self.status_message.emit("   Capturando sin autofoco...")
                # Emitir señal para que UI maneje captura
                self.autofocus_complete.emit([])
            return
        
        # Objetos detectados
        self.status_message.emit(f"✅ {len(objects)} objeto(s) en rango (de {len(all_objects)} detectados)")
        for i, obj in enumerate(objects):
            self.status_message.emit(f"   #{i+1}: área={obj.area:.0f}px, score={obj.focus_score:.1f}")
        
        self.detection_complete.emit(objects)
        
        # Iniciar autofoco asíncrono
        if self.autofocus is not None:
            self.status_message.emit("🎯 Iniciando Z-scan autofoco...")
            self._pending_capture = capture_after
            self.autofocus_started.emit()
            self.autofocus.start_autofocus(objects)
        else:
            self.validation_error.emit("AutofocusService no disponible")
            if capture_after:
                self.autofocus_complete.emit([])
    
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
        
        # Validar configuración
        is_valid, error = config.validate()
        if not is_valid:
            self.validation_error.emit(f"Configuración inválida: {error}")
            return False
        
        # Actualizar parámetros en servicio
        self.autofocus.z_scan_range = config.z_scan_range
        self.autofocus.z_step_coarse = config.z_step_coarse
        self.autofocus.z_step_fine = config.z_step_fine
        self.autofocus.settle_time = config.settle_time
        self.autofocus.capture_settle_time = config.capture_settle_time
        self.autofocus.roi_margin = config.roi_margin
        self.autofocus.max_coarse_iterations = config.max_coarse_iterations
        self.autofocus.max_fine_iterations = config.max_fine_iterations
        self.autofocus.n_captures = config.n_captures
        self.autofocus.z_step_capture = config.z_step_capture
        self.autofocus.z_range_capture = config.z_range_capture
        
        self.status_message.emit("✅ Parámetros de autofoco actualizados")
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

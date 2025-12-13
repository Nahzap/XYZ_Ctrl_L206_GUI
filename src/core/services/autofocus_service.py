"""
Autofocus Service - Servicio de Autoenfoque Asíncrono
=====================================================

Worker para ejecutar Z-scanning en background sin bloquear la UI.
Emite señales de progreso para visualización en tiempo real.

Autor: Sistema de Control L206
Fecha: 2025-12-12
"""

import time
import logging
import numpy as np
import cv2

from typing import List, Tuple, Optional, Callable
from dataclasses import dataclass

from PyQt5.QtCore import QThread, pyqtSignal

from core.detection.u2net_detector import DetectedObject

logger = logging.getLogger('MotorControl_L206')


@dataclass
class FocusResult:
    """Resultado de autofoco para un objeto."""
    object_index: int
    z_optimal: float
    focus_score: float
    bbox: Tuple[int, int, int, int]
    frame: Optional[np.ndarray] = None


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
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Referencias a hardware (se configuran externamente)
        self.cfocus_controller = None
        self.get_frame_callback: Callable[[], np.ndarray] = None
        
        # Objetos a enfocar
        self.objects_to_focus: List[DetectedObject] = []
        
        # Parámetros de escaneo
        self.z_step_coarse = 5.0  # µm - paso grueso
        self.z_step_fine = 1.0   # µm - paso fino
        self.settle_time = 0.05  # segundos
        
        # Control
        self.running = False
        self.cancel_requested = False
        
        logger.info("[AutofocusService] Inicializado")
    
    def configure(self, cfocus_controller, get_frame_callback: Callable):
        """
        Configura el servicio con referencias a hardware.
        
        Args:
            cfocus_controller: Controlador del piezo C-Focus
            get_frame_callback: Función que retorna el frame actual de la cámara
        """
        self.cfocus_controller = cfocus_controller
        self.get_frame_callback = get_frame_callback
        logger.info("[AutofocusService] Configurado con C-Focus y cámara")
    
    def start_autofocus(self, objects: List[DetectedObject]):
        """
        Inicia el proceso de autofoco para una lista de objetos.
        
        Args:
            objects: Lista de objetos detectados a enfocar
        """
        if self.isRunning():
            logger.warning("[AutofocusService] Ya hay un escaneo en progreso")
            return
        
        if not self.cfocus_controller:
            self.error_occurred.emit("C-Focus no configurado")
            return
        
        if not self.get_frame_callback:
            self.error_occurred.emit("Callback de cámara no configurado")
            return
        
        self.objects_to_focus = objects
        self.cancel_requested = False
        self.running = True
        self.start()
    
    def cancel(self):
        """Cancela el escaneo en progreso."""
        self.cancel_requested = True
        logger.info("[AutofocusService] Cancelación solicitada")
    
    def run(self):
        """Loop principal de autofoco."""
        results: List[FocusResult] = []
        total_objects = len(self.objects_to_focus)
        
        logger.info(f"[AutofocusService] Iniciando autofoco para {total_objects} objetos")
        
        for i, obj in enumerate(self.objects_to_focus):
            if self.cancel_requested:
                logger.info("[AutofocusService] Cancelado por usuario")
                break
            
            # Emitir inicio de escaneo
            self.scan_started.emit(i, total_objects)
            
            try:
                # Escanear objeto (pasar índice ya que ObjectInfo no tiene .index)
                result = self._scan_single_object(obj, i)
                results.append(result)
                
                # Emitir resultado
                self.object_focused.emit(i, result.z_optimal, result.focus_score)
                
            except Exception as e:
                logger.error(f"[AutofocusService] Error en objeto {i}: {e}")
                self.error_occurred.emit(f"Error en objeto {i}: {e}")
        
        self.running = False
        self.scan_complete.emit(results)
        logger.info(f"[AutofocusService] Completado: {len(results)}/{total_objects} objetos")
    
    def _scan_single_object(self, obj, obj_index: int) -> FocusResult:
        """
        Algoritmo de Hill-Climbing (Ascenso de Gradiente) para autofoco rápido.
        
        1. Empezar desde posición actual
        2. Medir score inicial
        3. Moverse en una dirección mientras el score mejore
        4. Si empeora, cambiar dirección
        5. Reducir paso y refinar hasta convergencia
        
        Mucho más rápido que escaneo completo (~5-10 pasos vs ~40+ pasos)
        """
        z_range = self.cfocus_controller.get_z_range()
        bbox = obj.bounding_box
        
        # Obtener posición actual como punto de partida
        current_z = self.cfocus_controller.read_z()
        if current_z is None:
            current_z = z_range * 0.5  # Empezar en el medio si no hay lectura
        
        logger.info(f"[Autofocus] Hill-Climbing desde Z={current_z:.1f}µm")
        
        # Medir score inicial
        best_z = current_z
        best_score = self._get_score_at_z(current_z, bbox)
        logger.debug(f"[Autofocus] Inicio: Z={best_z:.1f}µm, S={best_score:.2f}")
        
        # Parámetros de hill-climbing
        step = self.z_step_coarse  # Empezar con paso grueso
        min_step = self.z_step_fine  # Paso mínimo para convergencia
        direction = 1  # 1 = subir, -1 = bajar
        max_iterations = 30  # Límite de seguridad
        no_improvement_count = 0
        
        for iteration in range(max_iterations):
            if self.cancel_requested:
                break
            
            # Calcular nueva posición
            new_z = best_z + (direction * step)
            
            # Verificar límites
            if new_z < 0 or new_z > z_range:
                # Cambiar dirección si llegamos al límite
                direction *= -1
                new_z = best_z + (direction * step)
                if new_z < 0 or new_z > z_range:
                    # Ambas direcciones fuera de rango, reducir paso
                    step = step / 2
                    if step < min_step:
                        break
                    continue
            
            # Medir score en nueva posición
            new_score = self._get_score_at_z(new_z, bbox)
            logger.debug(f"[Autofocus] #{iteration}: Z={new_z:.1f}µm, S={new_score:.2f} (best={best_score:.2f})")
            
            # Emitir progreso
            frame = self.get_frame_callback()
            if frame is not None:
                x, y, w, h = bbox
                if y+h <= frame.shape[0] and x+w <= frame.shape[1]:
                    roi_frame = frame[y:y+h, x:x+w].copy()
                    self.z_changed.emit(new_z, new_score, roi_frame)
            
            if new_score > best_score:
                # Mejora! Continuar en esta dirección
                best_z = new_z
                best_score = new_score
                no_improvement_count = 0
            else:
                # No mejora
                no_improvement_count += 1
                
                if no_improvement_count >= 2:
                    # Dos fallos seguidos: reducir paso y cambiar dirección
                    step = step / 2
                    direction *= -1
                    no_improvement_count = 0
                    
                    if step < min_step:
                        # Convergencia alcanzada
                        logger.info(f"[Autofocus] Convergencia en Z={best_z:.1f}µm, S={best_score:.2f}")
                        break
                else:
                    # Primer fallo: cambiar dirección
                    direction *= -1
        
        # ========== MOVER A BPoF FINAL ==========
        logger.info(f"[Autofocus] BPoF encontrado: Z={best_z:.1f}µm, S={best_score:.2f}")
        self.cfocus_controller.move_z(best_z)
        time.sleep(self.settle_time * 2)
        
        # Verificar posición final
        final_z = self.cfocus_controller.read_z()
        if final_z is not None:
            logger.info(f"[Autofocus] FINAL: Z={final_z:.1f}µm")
        
        final_frame = self.get_frame_callback()
        
        return FocusResult(
            object_index=obj_index,
            z_optimal=best_z,
            focus_score=best_score,
            bbox=bbox,
            frame=final_frame
        )
    
    def _get_score_at_z(self, z: float, bbox: Tuple[int, int, int, int]) -> float:
        """Mueve a posición Z y obtiene score estable."""
        self.cfocus_controller.move_z(z)
        time.sleep(self.settle_time)
        return self._get_stable_score(bbox, n_samples=2)  # Solo 2 muestras para velocidad
    
    def _get_stable_score(self, bbox: Tuple[int, int, int, int], n_samples: int = 3) -> float:
        """
        Obtiene un score estable promediando múltiples lecturas.
        Esto reduce el ruido y mejora la reproducibilidad.
        """
        scores = []
        for _ in range(n_samples):
            frame = self.get_frame_callback()
            if frame is not None:
                score = self._calculate_sharpness(frame, bbox)
                scores.append(score)
            time.sleep(0.02)  # Pequeña pausa entre lecturas
        
        if scores:
            return float(np.median(scores))  # Mediana es más robusta que promedio
        return 0.0
    
    def _calculate_sharpness(self, frame: np.ndarray, bbox: Tuple[int, int, int, int]) -> float:
        """
        Calcula el índice de nitidez combinando múltiples métricas:
        - Laplacian Variance (bordes)
        - Tenengrad (gradiente Sobel)
        - Normalized Variance
        """
        import cv2
        
        x, y, w, h = bbox
        h_frame, w_frame = frame.shape[:2]
        
        # Validar bbox
        x = max(0, min(x, w_frame - 1))
        y = max(0, min(y, h_frame - 1))
        w = min(w, w_frame - x)
        h = min(h, h_frame - y)
        
        if w <= 0 or h <= 0:
            return 0.0
        
        # Extraer ROI
        roi = frame[y:y+h, x:x+w]
        
        # Convertir a grayscale
        if len(roi.shape) == 3:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        else:
            gray = roi.copy()
        
        # Normalizar si es uint16
        if gray.dtype == np.uint16:
            gray = (gray / 256).astype(np.uint8)
        
        # Aplicar filtro gaussiano para reducir ruido
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        
        # 1. Laplacian Variance (sensible a bordes finos)
        laplacian = cv2.Laplacian(gray, cv2.CV_64F, ksize=5)
        lap_var = laplacian.var()
        
        # 2. Tenengrad (gradiente Sobel - robusto)
        gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        tenengrad = np.mean(gx**2 + gy**2)
        
        # 3. Normalized Variance (contraste local)
        mean_val = gray.mean()
        if mean_val > 0:
            norm_var = gray.var() / mean_val
        else:
            norm_var = 0
        
        # Combinar métricas (ponderadas)
        # Tenengrad es más robusto para microscopía
        combined = (lap_var * 0.3) + (tenengrad * 0.5) + (norm_var * 0.2)
        
        return float(combined)
    
    def set_parameters(self, z_step_coarse: float = None, z_step_fine: float = None,
                       settle_time: float = None):
        """Actualiza parámetros de escaneo."""
        if z_step_coarse is not None:
            self.z_step_coarse = z_step_coarse
        if z_step_fine is not None:
            self.z_step_fine = z_step_fine
        if settle_time is not None:
            self.settle_time = settle_time

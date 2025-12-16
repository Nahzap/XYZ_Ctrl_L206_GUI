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

from PyQt5.QtCore import QThread, pyqtSignal

from core.models.detected_object import DetectedObject
from core.models.focus_result import AutofocusResult
from core.autofocus.smart_focus_scorer import SmartFocusScorer

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
        self.z_search_range = 40.0  # µm - rango de búsqueda
        self.z_tolerance = 0.5  # µm - tolerancia
        self.settle_time = 0.05  # segundos
        
        # Control
        self.running = False
        self.cancel_requested = False
        
        # Scorer morfológico para Smart Autofocus (usa máscara de morfología)
        self._focus_scorer = SmartFocusScorer(
            min_circularity=0.45,
            min_aspect_ratio=0.4
        )
        try:
            self._focus_scorer.load_model()
        except Exception:
            # Si falla la carga del modelo, el scorer cae a fallback interno
            pass
        
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
        
        # MEJORA: Retornar a la mitad del rango Z después de completar autofoco
        # Esto permite que el siguiente punto comience desde la posición central
        if self.cfocus_controller and len(results) > 0:
            try:
                z_current = self.cfocus_controller.read_z()
                z_mid = 50.0  # Mitad del rango 0-100µm
                logger.info(f"[AutofocusService] Retornando de Z={z_current:.1f}µm a posición central Z={z_mid:.1f}µm")
                self.cfocus_controller.move_z(z_mid)
                time.sleep(self.settle_time * 2)
                logger.info(f"[AutofocusService] ✓ Eje Z retornado a posición central para próximo punto")
            except Exception as e:
                logger.warning(f"[AutofocusService] No se pudo retornar a posición central: {e}")
        
        self.running = False
        self.scan_complete.emit(results)
        logger.info(f"[AutofocusService] Completado: {len(results)}/{total_objects} objetos")
    
    def _scan_single_object(self, obj, obj_index: int) -> FocusResult:
        """
        Algoritmo de autofoco bidireccional desde punto medio.
        
        1. Guarda posición Z inicial (referencia)
        2. Mueve al punto medio del rango de búsqueda
        3. Busca el Best Plane of Focus hacia arriba y abajo
        4. Captura en el BPoF
        5. Vuelve a la posición de referencia inicial
        """
        bbox = obj.bounding_box
        
        # PASO 1: Guardar posición Z inicial (referencia)
        reference_z = self.cfocus_controller.read_z()
        if reference_z is None:
            reference_z = 50.0  # Fallback
        
        logger.info(f"[Autofocus] Posición de referencia guardada: Z={reference_z:.1f}µm")
        
        # Obtener rango calibrado del C-Focus
        z_max_hardware = self.cfocus_controller.get_z_range()
        if z_max_hardware is None or z_max_hardware <= 0:
            z_max_hardware = 100.0  # Fallback
        
        # PASO 2: Calcular punto medio del rango de búsqueda
        # Usar z_search_range (ej: 50µm) centrado en la referencia
        z_min = max(0, reference_z - self.z_search_range / 2)
        z_max = min(z_max_hardware, reference_z + self.z_search_range / 2)
        z_mid = (z_min + z_max) / 2
        
        logger.info(f"[Autofocus] Rango de búsqueda: [{z_min:.1f}, {z_max:.1f}]µm")
        logger.info(f"[Autofocus] Moviendo a punto medio: Z={z_mid:.1f}µm")
        
        # Mover al punto medio
        self.cfocus_controller.move_z(z_mid)
        time.sleep(self.settle_time * 2)  # Esperar estabilización
        
        # PASO 3: Buscar el Best Plane of Focus con Hill-Climbing bidireccional
        best_z = z_mid
        best_score = self._get_score_at_z(z_mid, bbox)
        logger.debug(f"[Autofocus] Inicio desde medio: Z={best_z:.1f}µm, S={best_score:.2f}")
        
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
            
            # Verificar límites del rango de búsqueda
            if new_z < z_min or new_z > z_max:
                # Cambiar dirección si llegamos al límite
                direction *= -1
                new_z = best_z + (direction * step)
                if new_z < z_min or new_z > z_max:
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
        
        # PASO 4: Mover al BPoF para captura
        logger.info(f"[Autofocus] BPoF encontrado: Z={best_z:.1f}µm, S={best_score:.2f}")
        self.cfocus_controller.move_z(best_z)
        time.sleep(self.settle_time * 2)
        
        # Capturar frame en el BPoF
        final_frame = self.get_frame_callback()
        logger.info(f"[Autofocus] Frame capturado en BPoF: Z={best_z:.1f}µm")
        
        # PASO 5: QUEDARSE EN EL BPoF (NO volver a referencia)
        # La imagen debe capturarse AQUÍ, en el plano de mejor enfoque
        final_z = self.cfocus_controller.read_z()
        if final_z is not None:
            logger.info(f"[Autofocus] ✓ Posición final en BPoF: Z={final_z:.1f}µm (objetivo={best_z:.1f}µm)")
        
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
        x, y, w, h = bbox
        h_frame, w_frame = frame.shape[:2]
        
        # Validar bbox
        x = max(0, min(x, w_frame - 1))
        y = max(0, min(y, h_frame - 1))
        w = min(w, w_frame - x)
        h = min(h, h_frame - y)
        
        if w <= 0 or h <= 0:
            return 0.0
        
        # Extraer ROI (región del objeto) y aplicar SmartFocusScorer sobre ella
        roi = frame[y:y+h, x:x+w]
        try:
            if hasattr(self, "_focus_scorer") and self._focus_scorer is not None:
                smart_score, _ = self._focus_scorer.get_smart_score(roi)
                return float(smart_score)
        except Exception as e:
            logger.error(f"[AutofocusService] Error en SmartFocusScorer.get_smart_score: {e}")
            # En caso de error, continuar con el cálculo clásico como fallback
        
        # --- Fallback: métrica clásica combinada (código original) ---
        if len(roi.shape) == 3:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        else:
            gray = roi.copy()
        
        if gray.dtype == np.uint16:
            gray = (gray / 256).astype(np.uint8)
        
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        
        laplacian = cv2.Laplacian(gray, cv2.CV_64F, ksize=5)
        lap_var = laplacian.var()
        
        gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        tenengrad = np.mean(gx**2 + gy**2)
        
        mean_val = gray.mean()
        if mean_val > 0:
            norm_var = gray.var() / mean_val
        else:
            norm_var = 0
        
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

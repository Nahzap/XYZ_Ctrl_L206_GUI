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
        self.z_step_coarse = 5.0  # µm - paso grueso inicial
        self.z_step_fine = 1.0   # µm - paso fino para refinamiento
        self.z_search_range = 40.0  # µm - rango de búsqueda (ignorado en full scan)
        self.z_tolerance = 0.5  # µm - tolerancia
        self.settle_time = 0.10  # segundos - tiempo de estabilización base
        self.capture_settle_time = 0.50  # segundos - tiempo para captura final (500ms)
        
        # Registro del máximo Z encontrado (para optimizar futuros escaneos)
        self.z_max_recorded = None  # Se actualiza tras primer escaneo completo
        
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
        
        # NOTA: NO mover Z después del autofoco - el frame ya fue capturado en BPoF
        # El sistema debe permanecer en BPoF hasta que se guarde la imagen
        # El movimiento a posición central se hace DESPUÉS de guardar la imagen
        
        self.running = False
        self.scan_complete.emit(results)
        logger.info(f"[AutofocusService] Completado: {len(results)}/{total_objects} objetos")
    
    def _scan_single_object(self, obj, obj_index: int) -> FocusResult:
        """
        Algoritmo de autofoco con ESCANEO COMPLETO desde Z=0 hasta Z_max.
        
        MEJORAS IMPLEMENTADAS:
        1. Siempre empieza desde Z=0 (punto más bajo)
        2. Escaneo completo 0→max registrando todos los scores
        3. Refinamiento alrededor del pico encontrado
        4. Captura con 500ms de estabilización cuando S está magnificado
        5. Registra Z_max para futuros escaneos
        6. Calcula sharpness SOLO sobre la máscara del objeto (U2-Net)
        """
        bbox = obj.bounding_box
        # Obtener contorno del objeto para calcular sharpness solo sobre la máscara
        contour = getattr(obj, 'contour', None)
        
        # Obtener rango calibrado del C-Focus
        z_max_hardware = self.cfocus_controller.get_z_range()
        if z_max_hardware is None or z_max_hardware <= 0:
            z_max_hardware = 80.0  # Fallback
        
        # PASO 1: Mover a Z=0 (punto más bajo)
        logger.info(f"[Autofocus] Iniciando escaneo completo: Z=0 → {z_max_hardware:.1f}µm")
        self.cfocus_controller.move_z(0.0)
        time.sleep(self.settle_time * 2)  # Estabilización inicial
        
        # PASO 2: ESCANEO COMPLETO 0→max con paso grueso
        z_positions = []
        sharpness_scores = []
        z_current = 0.0
        step = self.z_step_coarse  # 5µm por defecto
        
        logger.info(f"[Autofocus] Paso grueso: {step}µm, evaluaciones estimadas: {int(z_max_hardware/step)+1}")
        
        while z_current <= z_max_hardware:
            if self.cancel_requested:
                break
            
            # Mover y esperar estabilización
            self.cfocus_controller.move_z(z_current)
            time.sleep(self.settle_time)
            
            # Obtener score estable (usando máscara del contorno U2-Net)
            score = self._get_stable_score(bbox, contour, n_samples=2)
            z_positions.append(z_current)
            sharpness_scores.append(score)
            
            # Emitir progreso para visualización
            frame = self.get_frame_callback()
            if frame is not None:
                x, y, w, h = bbox
                if y+h <= frame.shape[0] and x+w <= frame.shape[1]:
                    roi_frame = frame[y:y+h, x:x+w].copy()
                    self.z_changed.emit(z_current, score, roi_frame)
            
            logger.debug(f"[Autofocus] Z={z_current:.1f}µm, S={score:.2f}")
            z_current += step
        
        if not z_positions:
            logger.error("[Autofocus] No se pudo evaluar ninguna posición Z")
            return FocusResult(
                object_index=obj_index,
                z_optimal=0.0,
                focus_score=0.0,
                bbox=bbox,
                frame=None
            )
        
        # PASO 3: Encontrar el pico (máximo S)
        max_idx = int(np.argmax(sharpness_scores))
        z_peak = z_positions[max_idx]
        peak_score = sharpness_scores[max_idx]
        
        logger.info(f"[Autofocus] Pico encontrado: Z={z_peak:.1f}µm, S={peak_score:.2f} ({len(z_positions)} evaluaciones)")
        
        # Registrar Z_max para futuros escaneos (optimización)
        self.z_max_recorded = z_peak
        
        # PASO 4: REFINAMIENTO alrededor del pico con paso fino
        logger.info(f"[Autofocus] Refinando ±{self.z_step_coarse}µm con paso {self.z_step_fine}µm")
        
        z_refine_min = max(0.0, z_peak - self.z_step_coarse)
        z_refine_max = min(z_max_hardware, z_peak + self.z_step_coarse)
        
        best_z = z_peak
        best_score = peak_score
        
        z_refine = z_refine_min
        while z_refine <= z_refine_max:
            if self.cancel_requested:
                break
            
            self.cfocus_controller.move_z(z_refine)
            time.sleep(self.settle_time)
            
            score = self._get_stable_score(bbox, contour, n_samples=2)
            
            # Emitir progreso
            frame = self.get_frame_callback()
            if frame is not None:
                x, y, w, h = bbox
                if y+h <= frame.shape[0] and x+w <= frame.shape[1]:
                    roi_frame = frame[y:y+h, x:x+w].copy()
                    self.z_changed.emit(z_refine, score, roi_frame)
            
            if score > best_score:
                best_z = z_refine
                best_score = score
                logger.debug(f"[Autofocus] Refinamiento: Z={z_refine:.1f}µm, S={score:.2f} ★ NUEVO MEJOR")
            else:
                logger.debug(f"[Autofocus] Refinamiento: Z={z_refine:.1f}µm, S={score:.2f}")
            
            z_refine += self.z_step_fine
        
        # PASO 5: CAPTURA PRINCIPAL en BPoF con estabilización extendida (500ms)
        logger.info(f"[Autofocus] BPoF final: Z={best_z:.1f}µm, S={best_score:.2f}")
        logger.info(f"[Autofocus] Moviendo a BPoF y esperando {self.capture_settle_time*1000:.0f}ms para captura...")
        
        self.cfocus_controller.move_z(best_z)
        time.sleep(self.capture_settle_time)  # 500ms de estabilización para captura nítida
        
        # Capturar frame en el BPoF con S magnificado
        final_frame = self.get_frame_callback()
        
        # Verificar score final (debe estar cerca del máximo)
        final_score = self._get_stable_score(bbox, contour, n_samples=3)
        logger.info(f"[Autofocus] ✓ Frame 1 (BPoF) capturado: Z={best_z:.1f}µm, S={final_score:.2f}")
        
        # PASO 6: CAPTURA ALTERNATIVA (segundo mejor foco o offset)
        # Buscar el segundo mejor pico en los datos del escaneo
        z_alt = None
        score_alt = 0.0
        frame_alt = None
        
        # Estrategia: usar un offset de ±10µm desde el BPoF para captura alternativa
        alt_offset = 10.0  # µm de diferencia
        
        # Elegir dirección del offset (hacia donde hay más rango disponible)
        if best_z + alt_offset <= z_max_hardware:
            z_alt = best_z + alt_offset
        elif best_z - alt_offset >= 0:
            z_alt = best_z - alt_offset
        else:
            z_alt = best_z + (alt_offset / 2)  # Fallback: medio offset
        
        logger.info(f"[Autofocus] Capturando imagen alternativa en Z={z_alt:.1f}µm (offset={z_alt-best_z:+.1f}µm)")
        
        self.cfocus_controller.move_z(z_alt)
        time.sleep(self.capture_settle_time)  # 500ms de estabilización
        
        frame_alt = self.get_frame_callback()
        score_alt = self._get_stable_score(bbox, contour, n_samples=2)
        
        logger.info(f"[Autofocus] ✓ Frame 2 (alternativo) capturado: Z={z_alt:.1f}µm, S={score_alt:.2f}")
        
        # IMPORTANTE: Volver a BPoF para dejar el sistema enfocado
        logger.info(f"[Autofocus] Regresando a BPoF: Z={best_z:.1f}µm")
        self.cfocus_controller.move_z(best_z)
        time.sleep(self.capture_settle_time)  # Estabilizar en posición óptima
        
        # Verificar posición final (debe ser BPoF)
        final_z = self.cfocus_controller.read_z()
        if final_z is not None:
            logger.info(f"[Autofocus] ✓ Posición final verificada: Z={final_z:.1f}µm (BPoF={best_z:.1f}µm)")
        
        return FocusResult(
            object_index=obj_index,
            z_optimal=best_z,
            focus_score=best_score,
            bbox=bbox,
            frame=final_frame,
            frame_alt=frame_alt,
            z_alt=z_alt,
            score_alt=score_alt
        )
    
    def _get_score_at_z(self, z: float, bbox: Tuple[int, int, int, int]) -> float:
        """Mueve a posición Z y obtiene score estable."""
        self.cfocus_controller.move_z(z)
        time.sleep(self.settle_time)
        return self._get_stable_score(bbox, n_samples=2)  # Solo 2 muestras para velocidad
    
    def _get_stable_score(self, bbox: Tuple[int, int, int, int], contour: np.ndarray = None, n_samples: int = 3) -> float:
        """
        Obtiene un score estable promediando múltiples lecturas.
        Calcula sharpness SOLO sobre los píxeles de la máscara (contorno).
        """
        scores = []
        for i in range(n_samples):
            frame = self.get_frame_callback()
            if frame is not None:
                # Verificar que el frame tiene contenido
                if frame.size == 0:
                    logger.warning(f"[Autofocus] Frame {i} vacío")
                    continue
                    
                score = self._calculate_sharpness(frame, bbox, contour)
                scores.append(score)
            else:
                logger.warning(f"[Autofocus] Frame {i} es None")
            time.sleep(0.02)  # Pequeña pausa entre lecturas
        
        if scores:
            median_score = float(np.median(scores))
            return median_score
        
        logger.warning(f"[Autofocus] No se obtuvieron scores válidos para bbox={bbox}")
        return 0.0
    
    def _calculate_sharpness(self, frame: np.ndarray, bbox: Tuple[int, int, int, int], 
                              contour: np.ndarray = None) -> float:
        """
        Calcula el índice de nitidez SOLO sobre los píxeles de la máscara del objeto.
        
        Si se proporciona un contorno (de U2-Net), crea una máscara y calcula
        el sharpness solo sobre esos píxeles. Si no, usa todo el bbox.
        """
        x, y, w, h = bbox
        h_frame, w_frame = frame.shape[:2]
        
        # Validar bbox
        x = max(0, min(x, w_frame - 1))
        y = max(0, min(y, h_frame - 1))
        w = min(w, w_frame - x)
        h = min(h, h_frame - y)
        
        if w <= 0 or h <= 0:
            logger.warning(f"[Autofocus] bbox inválido: w={w}, h={h}")
            return 0.0
        
        # Extraer ROI (región del objeto)
        roi = frame[y:y+h, x:x+w]
        
        # Convertir a grayscale
        if len(roi.shape) == 3:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        else:
            gray = roi.copy()
        
        # Normalizar uint16 → uint8 si es necesario
        if gray.dtype == np.uint16:
            gray = (gray / 256).astype(np.uint8)
        
        # Crear máscara del objeto si hay contorno disponible
        mask = None
        if contour is not None and len(contour) > 0:
            # Crear máscara del tamaño del ROI
            mask = np.zeros((h, w), dtype=np.uint8)
            # Ajustar contorno a coordenadas del ROI
            contour_shifted = contour.copy()
            contour_shifted[:, :, 0] -= x
            contour_shifted[:, :, 1] -= y
            cv2.drawContours(mask, [contour_shifted], -1, 255, -1)
        
        # Suavizado ligero para reducir ruido
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        
        # Calcular Laplacian
        laplacian = cv2.Laplacian(gray, cv2.CV_64F, ksize=5)
        
        # Calcular gradientes Sobel
        gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        gradient_mag = gx**2 + gy**2
        
        # Si hay máscara, aplicarla para calcular solo sobre el objeto
        if mask is not None and np.count_nonzero(mask) > 0:
            # Extraer valores solo donde la máscara es > 0
            lap_values = laplacian[mask > 0]
            grad_values = gradient_mag[mask > 0]
            gray_values = gray[mask > 0]
            
            lap_var = lap_values.var() if len(lap_values) > 0 else 0
            tenengrad = grad_values.mean() if len(grad_values) > 0 else 0
            mean_val = gray_values.mean() if len(gray_values) > 0 else 0
            norm_var = gray_values.var() / mean_val if mean_val > 0 else 0
            n_pixels = len(lap_values)
        else:
            # Sin máscara: usar todo el ROI
            lap_var = laplacian.var()
            tenengrad = gradient_mag.mean()
            mean_val = gray.mean()
            norm_var = gray.var() / mean_val if mean_val > 0 else 0
            n_pixels = gray.size
        
        # Combinar métricas
        combined = (lap_var * 0.25) + (tenengrad * 0.50) + (norm_var * 0.25)
        
        logger.debug(f"[Autofocus] S={combined:.1f} (lap={lap_var:.1f}, ten={tenengrad:.1f}, nv={norm_var:.1f}, px={n_pixels})")
        
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

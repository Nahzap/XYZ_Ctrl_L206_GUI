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
    status_message = pyqtSignal(str)  # Mensajes de estado para UI y terminal
    score_updated = pyqtSignal(float, float)  # (z_position, score) para overlay en cámara
    progress_updated = pyqtSignal(int, int, str)  # (current_step, total_steps, phase_name)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Referencias a hardware (se configuran externamente)
        self.cfocus_controller = None
        self.get_frame_callback: Callable[[], np.ndarray] = None
        
        # Objetos a enfocar
        self.objects_to_focus: List[DetectedObject] = []
        
        # Parámetros de búsqueda (configurables desde UI)
        # NOTA: Estos parámetros son para BÚSQUEDA de BPoF, NO para captura de volumen
        self.z_scan_range = 20.0  # µm - distancia máxima de búsqueda desde posición actual (±20µm)
        self.z_step_coarse = 0.5  # µm - paso grueso para fase de búsqueda inicial (hill climbing)
        self.z_step_fine = 0.1    # µm - paso fino para refinamiento alrededor del pico
        self.settle_time = 0.10   # segundos - tiempo de estabilización
        self.capture_settle_time = 0.50  # segundos - tiempo para captura final (500ms)
        self.roi_margin = 20      # px - margen adicional alrededor del bbox para sharpness
        
        # Límites de iteraciones para evitar bucles infinitos
        self.max_coarse_iterations = 50  # Máximo de iteraciones en fase gruesa
        self.max_fine_iterations = 100   # Máximo de iteraciones en fase fina
        
        # Parámetros de captura multi-focal (para trayectoria XY)
        # NOTA: Estas capturas son para obtener imágenes con diferentes niveles de enfoque
        self.n_captures = 3       # Número de capturas (siempre impar: 3, 5, 7, etc.)
        self.capture_step = None  # µm - paso entre capturas (None = usar z_step_coarse)
        
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
    
    def _optimize_focus_simple(self, bbox, contour, z_min: float, z_max: float, z_center: float) -> tuple:
        """
        ESCANEO COMPLETO de autofocus - recorre TODO el rango calibrado.
        
        Algoritmo:
        1. Escanea desde z_min hasta z_max con paso coarse
        2. Registra el mejor Z encontrado
        3. Refina alrededor del mejor Z con paso fine
        4. Retorna (best_z, best_score)
        
        Garantiza cubrir TODO el rango disponible.
        """
        msg = f"[Autofocus] ESCANEO COMPLETO: {z_min:.2f} → {z_max:.2f}µm (paso={self.z_step_coarse}µm)"
        logger.info(msg)
        print(msg)
        
        # FASE 1: Escaneo completo con paso grueso
        z_range = z_max - z_min
        n_steps = int(z_range / self.z_step_coarse) + 1
        
        msg = f"[Autofocus] Escaneando {n_steps} posiciones en rango completo..."
        logger.info(msg)
        print(msg)
        
        best_z = z_min
        best_score = 0.0
        
        for i in range(n_steps):
            if self.cancel_requested:
                break
            
            z_current = z_min + (i * self.z_step_coarse)
            if z_current > z_max:
                z_current = z_max
            
            # Emitir progreso
            self.progress_updated.emit(i + 1, n_steps, "Escaneo completo")
            
            # Mover y evaluar
            move_success = self.cfocus_controller.move_z(z_current)
            if not move_success:
                logger.warning(f"[Autofocus] Fallo al mover a Z={z_current:.2f}µm")
                continue
            
            time.sleep(self.settle_time)
            score = self._get_stable_score(bbox, contour, n_samples=2)
            
            # Actualizar mejor posición
            if score > best_score:
                best_z = z_current
                best_score = score
                msg = f"[Autofocus] [{i+1}/{n_steps}] Z={z_current:.2f}µm, S={score:.1f} ★ MEJOR"
                logger.info(msg)
                print(msg)
            else:
                msg = f"[Autofocus] [{i+1}/{n_steps}] Z={z_current:.2f}µm, S={score:.1f}"
                logger.debug(msg)
                if i % 10 == 0:  # Mostrar cada 10 pasos
                    print(msg)
        
        msg = f"[Autofocus] Escaneo completo finalizado: mejor Z={best_z:.2f}µm, S={best_score:.1f}"
        logger.info(msg)
        print(msg)
        
        # FASE 2: Refinamiento con paso fino alrededor del mejor Z
        step = self.z_step_fine
        
        msg = f"[Autofocus] Refinamiento fino (paso={step}µm) alrededor de Z={best_z:.2f}µm..."
        logger.info(msg)
        print(msg)
        
        # Explorar ±z_step_coarse alrededor del pico con paso fino
        z_refine_min = max(z_min, best_z - self.z_step_coarse)
        z_refine_max = min(z_max, best_z + self.z_step_coarse)
        
        # Calcular número total de pasos de refinamiento
        total_refine_steps = int((z_refine_max - z_refine_min) / step) + 1
        total_refine_steps = min(total_refine_steps, self.max_fine_iterations)
        
        z_refine = z_refine_min
        refine_iteration = 0
        
        # Guardar mejor resultado del escaneo grueso
        best_z_coarse = best_z
        best_score_coarse = best_score
        
        while z_refine <= z_refine_max and refine_iteration < self.max_fine_iterations:
            if self.cancel_requested:
                break
            
            refine_iteration += 1
            
            # Emitir progreso (fase fina)
            self.progress_updated.emit(refine_iteration, total_refine_steps, "Refinamiento fino")
            
            move_success = self.cfocus_controller.move_z(z_refine)
            if not move_success:
                logger.warning(f"[Autofocus] Fallo al mover a Z={z_refine:.2f}µm, abortando refinamiento")
                break
            time.sleep(self.settle_time)
            score = self._get_stable_score(bbox, contour, n_samples=2)
            
            if score > best_score:
                best_z = z_refine
                best_score = score
                msg = f"[Autofocus] Refinamiento [{refine_iteration}/{total_refine_steps}]: Z={z_refine:.2f}µm, S={score:.1f} ★ MEJOR"
                logger.info(msg)
                print(msg)
            else:
                msg = f"[Autofocus] Refinamiento [{refine_iteration}/{total_refine_steps}]: Z={z_refine:.2f}µm, S={score:.1f}"
                logger.debug(msg)
            
            z_refine += step
        
        msg = f"[Autofocus] ✓ ÓPTIMO FINAL: Z={best_z:.2f}µm, S={best_score:.1f}"
        logger.info(msg)
        print(msg)
        
        return best_z, best_score
    
    def _scan_single_object(self, obj, obj_index: int) -> FocusResult:
        """
        Algoritmo de autofoco con ESCANEO desde la MITAD del recorrido.
        
        MEJORAS IMPLEMENTADAS:
        1. Siempre empieza desde Z_center (mitad del recorrido)
        2. Escaneo bidireccional usando z_scan_range configurado por usuario
        3. Refinamiento alrededor del pico encontrado
        4. Captura con 500ms de estabilización cuando S está magnificado
        5. Registra Z_max para futuros escaneos
        6. Calcula sharpness SOLO sobre la máscara del objeto (U2-Net)
        7. Emite mensajes de estado para monitoreo en terminal y UI
        """
        bbox = obj.bounding_box
        # Obtener contorno del objeto para calcular sharpness solo sobre la máscara
        contour = getattr(obj, 'contour', None)
        
        # PASO 4: DETERMINAR RANGO DE ESCANEO
        # Obtener posición actual del C-Focus
        z_current = self.cfocus_controller.read_z()
        if z_current is None or z_current < 0:
            z_current = 0.0
            logger.warning(f"[Autofocus] No se pudo leer posición Z, usando 0.0µm")
        
        # Obtener límites calibrados del hardware
        calib_info = self.cfocus_controller.get_calibration_info()
        
        if calib_info['is_calibrated']:
            z_min_hw = calib_info['z_min']
            z_max_hw = calib_info['z_max']
            z_center_hw = calib_info['z_center']
            
            # ESCANEO COMPLETO: usar TODO el rango calibrado
            # Si z_scan_range es el rango total, escanear desde límites calibrados
            z_min = z_min_hw
            z_max = z_max_hw
            
            msg = f"[Autofocus] Rango de escaneo COMPLETO: {z_min:.2f}-{z_max:.2f}µm (rango total calibrado)"
            logger.info(msg)
            print(msg)
            
            # Usar posición actual como punto de inicio
            z_center = z_current
            msg = f"[Autofocus] Inicio desde posición actual: Z={z_current:.2f}µm"
            logger.info(msg)
            print(msg)
        else:
            # Fallback: usar rango de hardware
            z_range_hw = self.cfocus_controller.get_z_range()
            if z_range_hw is None or z_range_hw <= 0:
                z_range_hw = 80.0
            
            z_min = 0.0
            z_max = z_range_hw
            z_center = z_current
            msg = f"[Autofocus] ⚠️ Sin calibración, rango completo: {z_min:.2f}-{z_max:.2f}µm"
            logger.warning(msg)
            print(msg)
        
        # Validar que el rango solicitado es válido
        is_valid, validation_msg = self.validate_scan_range()
        if not is_valid:
            logger.error(f"[Autofocus] {validation_msg}")
            raise ValueError(validation_msg)
        
        # Mostrar parámetros de búsqueda
        search_info = self.get_search_info()
        msg = f"[Autofocus] Búsqueda: ±{search_info['scan_range_um']}µm, pasos: {search_info['z_step_coarse']}µm→{search_info['z_step_fine']}µm"
        logger.info(msg)
        print(msg)
        
        # USAR OPTIMIZADOR SIMPLE en lugar de escaneo completo
        best_z, best_score = self._optimize_focus_simple(bbox, contour, z_min, z_max, z_center)
        
        # PASO 5: CAPTURA PRINCIPAL en BPoF con estabilización extendida (500ms)
        msg = f"[Autofocus] ✓ BPoF FINAL: Z={best_z:.1f}µm, Score={best_score:.1f}"
        logger.info(msg)
        self.status_message.emit(msg)
        print(msg)  # Terminal
        
        msg = f"[Autofocus] Moviendo a BPoF y esperando {self.capture_settle_time*1000:.0f}ms para captura..."
        logger.info(msg)
        self.status_message.emit(msg)
        
        self.cfocus_controller.move_z(best_z)
        time.sleep(self.capture_settle_time)  # 500ms de estabilización para captura nítida
        
        # Capturar frame en el BPoF con S magnificado
        final_frame = self.get_frame_callback()
        
        # Verificar score final (debe estar cerca del máximo)
        final_score = self._get_stable_score(bbox, contour, n_samples=3)
        logger.info(f"[Autofocus] ✓ Frame 1 (BPoF) capturado: Z={best_z:.1f}µm, S={final_score:.2f}")
        
        # PASO 6: CAPTURA MULTI-FOCAL (N imágenes con diferentes niveles de enfoque)
        # Capturar N imágenes (siempre impar) centradas en BPoF
        # Ejemplo con n_captures=3: [BPoF-coarse, BPoF, BPoF+coarse]
        # Ejemplo con n_captures=5: [BPoF-2*coarse, BPoF-coarse, BPoF, BPoF+coarse, BPoF+2*coarse]
        
        n_captures = self.n_captures if self.n_captures % 2 == 1 else 3  # Asegurar impar
        capture_step = self.capture_step if self.capture_step else self.z_step_coarse
        
        # Calcular posiciones Z para captura (BPoF en el centro)
        z_positions = []
        frames = []
        scores = []
        
        logger.info(f"[Autofocus] Capturando {n_captures} imágenes multi-focales (paso={capture_step}µm)")
        
        for i in range(n_captures):
            offset = (i - n_captures // 2) * capture_step  # -coarse, 0, +coarse para n=3
            z_capture = best_z + offset
            
            # Validar que no exceda límites
            if z_capture < z_min or z_capture > z_max:
                logger.warning(f"[Autofocus] Z={z_capture:.1f}µm fuera de rango, ajustando...")
                z_capture = max(z_min, min(z_max, z_capture))
            
            # Mover y capturar
            self.cfocus_controller.move_z(z_capture)
            time.sleep(self.capture_settle_time)
            
            frame_i = self.get_frame_callback()
            score_i = self._get_stable_score(bbox, contour, n_samples=2)
            
            z_positions.append(z_capture)
            frames.append(frame_i)
            scores.append(score_i)
            
            focus_label = "BPoF" if i == n_captures // 2 else f"offset={offset:+.1f}µm"
            logger.info(f"[Autofocus] ✓ Frame {i+1}/{n_captures} ({focus_label}): Z={z_capture:.1f}µm, S={score_i:.2f}")
        
        # Para compatibilidad: frame_alt es la última imagen (más desenfocada arriba)
        frame_alt = frames[-1] if len(frames) > 1 else None
        z_alt = z_positions[-1] if len(z_positions) > 1 else best_z
        score_alt = scores[-1] if len(scores) > 1 else 0.0
        
        # CRÍTICO: REGRESAR AL BPoF (mejor foco) y QUEDARSE AHÍ
        msg = f"[Autofocus] Regresando al BPoF: Z={best_z:.2f}µm (mejor foco encontrado)"
        logger.info(msg)
        self.status_message.emit(msg)
        print(msg)  # Terminal
        
        self.cfocus_controller.move_z(best_z)
        time.sleep(self.settle_time)
        
        # Verificar posición final
        z_final_read = self.cfocus_controller.read_z()
        logger.info(f"[Autofocus] ✓ Posición final verificada: Z={z_final_read:.2f}µm (BPoF={best_z:.2f}µm)")
        
        return FocusResult(
            object_index=obj_index,
            z_optimal=best_z,
            focus_score=best_score,
            bbox=bbox,
            frame=final_frame,
            frames=frames,
            z_positions=z_positions,
            focus_scores=scores,
            # Campos legacy para compatibilidad
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
        Calcula el índice de nitidez sobre un ROI expandido alrededor del objeto.
        
        El ROI se expande con self.roi_margin píxeles para capturar mejor el contexto
        y calcular sharpness de forma más robusta.
        """
        x, y, w, h = bbox
        h_frame, w_frame = frame.shape[:2]
        
        # EXPANDIR bbox con margen para mejor cálculo de sharpness
        margin = self.roi_margin
        x_expanded = max(0, x - margin)
        y_expanded = max(0, y - margin)
        w_expanded = min(w + 2*margin, w_frame - x_expanded)
        h_expanded = min(h + 2*margin, h_frame - y_expanded)
        
        if w_expanded <= 0 or h_expanded <= 0:
            logger.warning(f"[Autofocus] ROI expandido inválido: w={w_expanded}, h={h_expanded}")
            return 0.0
        
        # Extraer ROI EXPANDIDO (región del objeto + margen)
        roi = frame[y_expanded:y_expanded+h_expanded, x_expanded:x_expanded+w_expanded]
        
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
            # Crear máscara del tamaño del ROI EXPANDIDO
            mask = np.zeros((h_expanded, w_expanded), dtype=np.uint8)
            # Ajustar contorno a coordenadas del ROI EXPANDIDO
            contour_shifted = contour.copy()
            contour_shifted[:, :, 0] -= x_expanded
            contour_shifted[:, :, 1] -= y_expanded
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

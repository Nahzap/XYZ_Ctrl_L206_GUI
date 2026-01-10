"""
Worker Thread para Autofoco No Bloqueante
==========================================

Thread worker para ejecutar Z-scan y captura multi-focal sin bloquear
el event loop de Qt, manteniendo la UI siempre receptiva.

Autor: Sistema de Control L206
Fecha: 2026-01-09
"""

import time
import logging
import numpy as np
from typing import Callable, Optional, List, Tuple
from PyQt5.QtCore import QThread, pyqtSignal

logger = logging.getLogger('MotorControl_L206')


class AutofocusWorker(QThread):
    """
    Thread worker para Z-scan y captura sin bloquear UI.
    
    Ejecuta operaciones bloqueantes (Z-scan, captura) en thread separado,
    permitiendo que el event loop de Qt y el timer de TestService continúen
    ejecutándose sin interrupciones.
    
    Signals:
        progress: (current_step, total_steps, z_position) - Progreso del Z-scan
        scan_complete: (best_z, best_score) - Z-scan completado
        capture_complete: (frames, z_positions, scores) - Captura completada
        error_occurred: (error_msg) - Error durante ejecución
    """
    
    # Señales
    progress = pyqtSignal(int, int, float)  # current, total, z_position
    scan_complete = pyqtSignal(float, float)  # best_z, best_score
    capture_complete = pyqtSignal(list, list, list)  # frames, z_positions, scores
    error_occurred = pyqtSignal(str)
    
    def __init__(
        self,
        cfocus,
        get_frame_func: Callable,
        scorer,
        config: dict,
        parent=None
    ):
        """
        Inicializa el worker thread.
        
        Args:
            cfocus: Controlador de C-Focus
            get_frame_func: Función para obtener frame actual
            scorer: AutofocusService o SmartFocusScorer
            config: Configuración con z_min, z_max, z_step, bbox, offset
            parent: Parent Qt object
        """
        super().__init__(parent)
        self.cfocus = cfocus
        self.get_frame = get_frame_func
        self.scorer = scorer
        self.config = config
        self._running = True
    
    def run(self):
        """Ejecuta Z-scan y captura en thread separado."""
        try:
            logger.info("[AutofocusWorker] Iniciando Z-scan en thread separado")
            
            # PASO 1: Z-scan
            best_z, best_score = self._perform_zscan()
            self.scan_complete.emit(best_z, best_score)
            
            # PASO 2: Captura multi-focal
            frames, z_positions, scores = self._capture_multifocal(best_z)
            self.capture_complete.emit(frames, z_positions, scores)
            
            logger.info("[AutofocusWorker] Trabajo completado exitosamente")
            
        except Exception as e:
            logger.error(f"[AutofocusWorker] Error: {e}", exc_info=True)
            self.error_occurred.emit(str(e))
    
    def _perform_zscan(self) -> Tuple[float, float]:
        """
        Realiza Z-scan optimizado.
        
        Returns:
            (best_z, best_score): Posición Z óptima y su score
        """
        z_min = self.config['z_min']
        z_max = self.config['z_max']
        z_step = self.config['z_step']
        bbox = self.config['bbox']
        contour = self.config.get('contour', None)
        
        best_z = z_min
        best_score = 0.0
        
        z = z_min
        n_steps = int((z_max - z_min) / z_step) + 1
        step_count = 0
        
        logger.info(f"[AutofocusWorker] Z-scan: {z_min:.2f} → {z_max:.2f}µm, paso={z_step:.2f}µm ({n_steps} pasos)")
        
        while z <= z_max and self._running:
            step_count += 1
            
            # Mover Z
            self.cfocus.move_z(z)
            time.sleep(0.02)  # 20ms estabilización (optimizado)
            
            # Obtener frame y calcular score
            frame = self.get_frame()
            if frame is not None:
                # Usar método del scorer para calcular score
                if hasattr(self.scorer, '_get_stable_score'):
                    score = self.scorer._get_stable_score(bbox, contour, n_samples=1)
                else:
                    # Fallback: calcular sharpness directamente
                    score = self._calculate_sharpness(frame, bbox)
                
                if score > best_score:
                    best_z = z
                    best_score = score
                
                # Emitir progreso
                self.progress.emit(step_count, n_steps, z)
            
            z += z_step
        
        logger.info(f"[AutofocusWorker] BPoF encontrado: Z={best_z:.2f}µm, Score={best_score:.1f}")
        return best_z, best_score
    
    def _capture_multifocal(self, best_z: float) -> Tuple[List, List, List]:
        """
        Captura N imágenes multi-focales.
        
        Args:
            best_z: Posición Z del BPoF
            
        Returns:
            (frames, z_positions, scores): Listas de frames, posiciones Z y scores
        """
        offset = self.config['offset']
        z_min = self.config['z_min']
        z_max = self.config['z_max']
        bbox = self.config['bbox']
        contour = self.config.get('contour', None)
        
        # Configurar posiciones Z (BPoF, +offset, -offset)
        z_positions = [
            best_z,
            min(z_max, best_z + offset),
            max(z_min, best_z - offset)
        ]
        
        frames = []
        scores = []
        
        logger.info(f"[AutofocusWorker] Capturando {len(z_positions)} imágenes multi-focales")
        
        for i, z_pos in enumerate(z_positions):
            if not self._running:
                break
            
            # Mover Z
            self.cfocus.move_z(z_pos)
            time.sleep(0.02)  # 20ms estabilización (optimizado)
            
            # Capturar frame
            frame = self.get_frame()
            if frame is not None:
                frames.append(frame.copy())
                
                # Calcular score
                if hasattr(self.scorer, '_get_stable_score'):
                    score = self.scorer._get_stable_score(bbox, contour, n_samples=1)
                else:
                    score = self._calculate_sharpness(frame, bbox)
                scores.append(score)
                
                label = "BPoF" if i == 0 else f"{'+' if i == 1 else '-'}{offset}µm"
                logger.info(f"[AutofocusWorker] Captura {i+1}/{len(z_positions)} ({label}): Z={z_pos:.2f}µm, S={score:.1f}")
        
        return frames, z_positions, scores
    
    def _calculate_sharpness(self, frame: np.ndarray, bbox: Tuple[int, int, int, int]) -> float:
        """
        Calcula sharpness de una región usando Laplacian variance.
        
        Args:
            frame: Frame completo
            bbox: (x, y, w, h) región de interés
            
        Returns:
            Sharpness score
        """
        import cv2
        
        x, y, w, h = bbox
        
        # Asegurar que bbox está dentro de la imagen
        h_img, w_img = frame.shape[:2]
        x = max(0, min(x, w_img - 1))
        y = max(0, min(y, h_img - 1))
        w = max(1, min(w, w_img - x))
        h = max(1, min(h, h_img - y))
        
        # Extraer ROI
        roi = frame[y:y+h, x:x+w]
        
        # Convertir a escala de grises si es necesario
        if len(roi.shape) == 3:
            roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        else:
            roi_gray = roi
        
        # Normalizar a uint8 si es uint16
        if roi_gray.dtype == np.uint16:
            roi_max = roi_gray.max()
            if roi_max > 0:
                roi_gray = (roi_gray / roi_max * 255).astype(np.uint8)
            else:
                roi_gray = roi_gray.astype(np.uint8)
        
        # Calcular Laplacian variance
        laplacian = cv2.Laplacian(roi_gray, cv2.CV_64F)
        variance = laplacian.var()
        
        return float(variance)
    
    def stop(self):
        """Detiene el worker de manera segura."""
        logger.info("[AutofocusWorker] Deteniendo worker...")
        self._running = False
        self.wait(2000)  # Esperar hasta 2 segundos
        if self.isRunning():
            logger.warning("[AutofocusWorker] Worker no se detuvo a tiempo, terminando forzadamente")
            self.terminate()

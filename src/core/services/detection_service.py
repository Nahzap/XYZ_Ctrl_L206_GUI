"""
Detection Service - Servicio de Detección Asíncrono
====================================================

Worker para ejecutar detección U2-Net en background sin bloquear la UI.
Emite señales con resultados para actualización en tiempo real.

Autor: Sistema de Control L206
Fecha: 2025-12-12
"""

import logging
import numpy as np
from typing import Optional, List
from queue import Queue, Empty, Full

from PyQt5.QtCore import QThread, pyqtSignal, QMutex

from core.detection.u2net_detector import U2NetDetector, DetectedObject

logger = logging.getLogger('MotorControl_L206')


class DetectionService(QThread):
    """
    Servicio de detección asíncrono.
    
    Procesa frames en background y emite resultados via señales PyQt.
    Usa el detector U2-Net singleton (carga única del modelo).
    
    Signals:
        detection_ready: Emitido cuando hay nuevos resultados (saliency_map, objects)
        status_changed: Emitido cuando cambia el estado del servicio
    """
    
    # Señales
    detection_ready = pyqtSignal(np.ndarray, list)  # saliency_map, List[DetectedObject]
    status_changed = pyqtSignal(str)  # mensaje de estado
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Detector singleton
        self.detector = U2NetDetector.get_instance()
        
        # Cola de frames (solo 1 frame pendiente)
        self.frame_queue = Queue(maxsize=1)
        
        # Control
        self.running = False
        self.paused = False
        self._mutex = QMutex()
        
        # Estadísticas
        self.frames_processed = 0
        self.last_detection_time_ms = 0
        
        logger.info("[DetectionService] Inicializado")
    
    def submit_frame(self, frame: np.ndarray) -> bool:
        """
        Envía un frame para detección (no bloqueante).
        
        Si hay un frame pendiente, se descarta el anterior.
        
        Args:
            frame: Imagen BGR o grayscale
            
        Returns:
            True si el frame fue aceptado
        """
        if not self.running or self.paused:
            return False
        
        try:
            # Limpiar cola si está llena
            try:
                self.frame_queue.get_nowait()
            except Empty:
                pass
            
            # Agregar nuevo frame
            self.frame_queue.put_nowait(frame.copy())
            return True
            
        except Full:
            return False
    
    def start_detection(self):
        """Inicia el servicio de detección."""
        if self.isRunning():
            return
        
        self.running = True
        self.paused = False
        self.start()
        
        device = self.detector.get_device()
        model_status = "U2-Net" if self.detector.is_model_loaded() else "Contornos"
        self.status_changed.emit(f"Detección activa ({model_status} en {device})")
        logger.info(f"[DetectionService] Iniciado - {model_status} en {device}")
    
    def stop_detection(self):
        """Detiene el servicio de detección."""
        self.running = False
        self.wait(1000)  # Esperar hasta 1 segundo
        self.status_changed.emit("Detección detenida")
        logger.info("[DetectionService] Detenido")
    
    def pause(self):
        """Pausa la detección (no procesa frames)."""
        self.paused = True
        self.status_changed.emit("Detección pausada")
    
    def resume(self):
        """Reanuda la detección."""
        self.paused = False
        self.status_changed.emit("Detección activa")
    
    def run(self):
        """Loop principal del worker."""
        import time
        
        while self.running:
            if self.paused:
                time.sleep(0.05)
                continue
            
            try:
                # Obtener frame (timeout 100ms)
                frame = self.frame_queue.get(timeout=0.1)
                
                # Medir tiempo de detección
                t_start = time.perf_counter()
                
                # Ejecutar detección
                saliency_map, objects = self.detector.detect(frame)
                
                t_end = time.perf_counter()
                self.last_detection_time_ms = (t_end - t_start) * 1000
                self.frames_processed += 1
                
                # Emitir resultados
                logger.info(f"[DetectionService] ✅ EMITIENDO detection_ready: {len(objects)} objetos detectados")
                print(f"[DetectionService] ✅ EMITIENDO detection_ready: {len(objects)} objetos")
                self.detection_ready.emit(saliency_map, objects)
                logger.info(f"[DetectionService] Señal detection_ready emitida correctamente")
                
            except Empty:
                continue
            except Exception as e:
                logger.error(f"[DetectionService] Error en detección: {e}")
                continue
    
    def set_parameters(self, min_area: int = None, max_area: int = None,
                       saliency_threshold: float = None):
        """Actualiza parámetros de detección."""
        self.detector.set_parameters(min_area, max_area, saliency_threshold)
    
    def get_stats(self) -> dict:
        """Retorna estadísticas del servicio."""
        return {
            'frames_processed': self.frames_processed,
            'last_detection_ms': self.last_detection_time_ms,
            'model_loaded': self.detector.is_model_loaded(),
            'device': self.detector.get_device()
        }

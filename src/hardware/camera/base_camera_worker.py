"""
Base Camera Worker - Interfaz Abstracta para Workers de Cámara
================================================================

Define el contrato común que todos los workers de cámara deben implementar.
Permite soporte multi-cámara (Thorlabs, Basler, futuras) con interfaz unificada.

Autor: Sistema de Control L206
Fecha: 2026-03-05
"""

from abc import ABCMeta, abstractmethod
from typing import Optional
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal


# Metaclase combinada para resolver conflicto QThread + ABC
class QThreadABCMeta(type(QThread), ABCMeta):
    """Metaclase que combina QThread y ABCMeta."""
    pass


class BaseCameraWorker(QThread, metaclass=QThreadABCMeta):
    """
    Clase base abstracta para workers de cámara.
    
    Todas las implementaciones de cámara (Thorlabs, Basler, etc.) deben heredar
    de esta clase e implementar los métodos abstractos.
    
    Señales comunes (todas las cámaras deben emitirlas):
        status_update(str): Mensajes de estado para logging en UI
        connection_success(bool, str): Resultado de conexión (success, camera_info)
        new_frame_ready(QImage, np.ndarray): Frame listo (display, raw)
    
    Atributos comunes:
        running (bool): Flag de ejecución del thread
        current_frame (np.ndarray): Frame actual en buffer
        exposure (float): Exposición en segundos
        fps (int): Frame rate objetivo
        buffer_size (int): Tamaño de buffer de frames
    """
    
    # Señales comunes (todas las cámaras deben emitirlas)
    status_update = pyqtSignal(str)
    connection_success = pyqtSignal(bool, str)  # success, camera_info
    new_frame_ready = pyqtSignal(object, object)  # QImage, raw_frame (uint16/uint8)
    
    def __init__(self):
        """Inicializa worker base con valores por defecto."""
        super().__init__()
        self.running = False
        self.current_frame: Optional[np.ndarray] = None
        self.exposure = 0.02  # segundos
        self.fps = 30
        self.buffer_size = 2
        self.frame_count = 0
    
    @abstractmethod
    def connect_camera(self) -> None:
        """
        Conecta con la cámara.
        
        Debe emitir:
            connection_success(True, camera_info) si exitoso
            connection_success(False, "") si falla
            status_update(mensaje) con progreso
        """
        pass
    
    @abstractmethod
    def disconnect_camera(self) -> None:
        """
        Desconecta la cámara y libera recursos.
        
        Debe:
            - Detener adquisición
            - Cerrar conexión
            - Liberar memoria
            - Emitir status_update con confirmación
        """
        pass
    
    @abstractmethod
    def start_live_view(self) -> None:
        """
        Inicia adquisición de video en vivo.
        
        Debe:
            - Configurar cámara
            - Iniciar adquisición continua
            - Loop de captura emitiendo new_frame_ready
            - Manejar errores y timeouts
        """
        pass
    
    @abstractmethod
    def stop_live_view(self) -> None:
        """
        Detiene adquisición de video.
        
        Debe:
            - Establecer flag running = False
            - Detener loop de adquisición
        """
        pass
    
    @abstractmethod
    def change_exposure(self, exposure_value: float) -> None:
        """
        Cambia exposición en tiempo real.
        
        Args:
            exposure_value: Exposición en segundos
        
        Debe:
            - Aplicar a hardware si está conectado
            - Actualizar self.exposure
            - Emitir status_update con confirmación
        """
        pass
    
    @abstractmethod
    def change_fps(self, fps_value: int) -> None:
        """
        Cambia frame rate en tiempo real.
        
        Args:
            fps_value: Frame rate en fps
        
        Debe:
            - Aplicar a hardware si está conectado
            - Actualizar self.fps
            - Emitir status_update con confirmación
        """
        pass
    
    @abstractmethod
    def change_buffer_size(self, buffer_value: int) -> None:
        """
        Cambia tamaño de buffer.
        
        Args:
            buffer_value: Número de frames en buffer
        
        Debe:
            - Actualizar self.buffer_size
            - Aplicar en próxima adquisición
            - Emitir status_update con confirmación
        """
        pass
    
    def run(self):
        """
        Método run del thread - inicia vista en vivo.
        Implementación común para todos los workers.
        """
        self.start_live_view()
    
    def get_camera_type(self) -> str:
        """
        Retorna el tipo de cámara.
        
        Returns:
            Tipo de cámara ("thorlabs", "basler", etc.)
        """
        return self.__class__.__name__.lower().replace("worker", "")

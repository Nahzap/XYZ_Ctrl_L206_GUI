"""
Camera Worker Factory - Factory para Creación de Workers de Cámara
===================================================================

Implementa el patrón Factory para crear workers de cámara según hardware disponible.
Soporta detección automática y selección manual.

Autor: Sistema de Control L206
Fecha: 2026-03-05
"""

import logging
from typing import Optional, Literal, List
from .base_camera_worker import BaseCameraWorker
from .thorlabs_worker import ThorlabsWorker
from .basler_worker import BaslerWorker
from config.hardware_availability import THORLABS_AVAILABLE, BASLER_AVAILABLE

logger = logging.getLogger('MotorControl_L206')

# Type hint para tipos de cámara
CameraType = Literal["thorlabs", "basler", "auto"]


class CameraWorkerFactory:
    """
    Factory para crear workers de cámara.
    
    Soporta:
    - Detección automática de hardware disponible
    - Selección manual de tipo de cámara
    - Extensibilidad para futuras cámaras
    
    Ejemplo:
        # Auto-detección
        worker = CameraWorkerFactory.create_worker("auto")
        
        # Selección manual
        worker = CameraWorkerFactory.create_worker("basler")
        
        # Detectar disponibles
        available = CameraWorkerFactory.detect_available_cameras()
    """
    
    @staticmethod
    def detect_available_cameras() -> List[str]:
        """
        Detecta cámaras disponibles en el sistema.
        
        Verifica qué SDKs están instalados y funcionales.
        
        Returns:
            Lista de tipos de cámara disponibles ["thorlabs", "basler"]
        
        Example:
            >>> available = CameraWorkerFactory.detect_available_cameras()
            >>> print(available)
            ['thorlabs', 'basler']
        """
        available = []
        
        if THORLABS_AVAILABLE:
            available.append("thorlabs")
            logger.info("[CameraFactory] Thorlabs SDK disponible (pylablib)")
        else:
            logger.info("[CameraFactory] Thorlabs SDK no disponible")
        
        if BASLER_AVAILABLE:
            available.append("basler")
            logger.info("[CameraFactory] Basler SDK disponible (pypylon)")
        else:
            logger.info("[CameraFactory] Basler SDK no disponible")
        
        if not available:
            logger.warning("[CameraFactory] No hay cámaras disponibles - instalar pylablib y/o pypylon")
        
        return available
    
    @staticmethod
    def create_worker(camera_type: CameraType = "auto") -> Optional[BaseCameraWorker]:
        """
        Crea worker apropiado según tipo de cámara.
        
        Args:
            camera_type: Tipo de cámara a crear
                - "auto": Detecta automáticamente (prioridad: Thorlabs → Basler)
                - "thorlabs": Crea ThorlabsWorker
                - "basler": Crea BaslerWorker
        
        Returns:
            Worker apropiado o None si no hay hardware disponible
        
        Raises:
            No lanza excepciones, retorna None si falla
        
        Example:
            >>> worker = CameraWorkerFactory.create_worker("auto")
            >>> if worker:
            ...     worker.connect_camera()
        """
        logger.info(f"[CameraFactory] create_worker(camera_type='{camera_type}')")
        
        # Auto-detección
        if camera_type == "auto":
            available = CameraWorkerFactory.detect_available_cameras()
            
            if not available:
                logger.error("[CameraFactory] No hay cámaras disponibles para auto-detección")
                return None
            
            # Prioridad: Thorlabs → Basler (mantener compatibilidad con código existente)
            camera_type = available[0]
            logger.info(f"[CameraFactory] Auto-detección: usando '{camera_type}'")
        
        # Crear worker específico
        if camera_type == "thorlabs":
            if not THORLABS_AVAILABLE:
                logger.error("[CameraFactory] Thorlabs solicitado pero SDK no disponible")
                return None
            
            logger.info("[CameraFactory] Creando ThorlabsWorker")
            return ThorlabsWorker()
        
        elif camera_type == "basler":
            if not BASLER_AVAILABLE:
                logger.error("[CameraFactory] Basler solicitado pero SDK no disponible")
                return None
            
            logger.info("[CameraFactory] Creando BaslerWorker")
            return BaslerWorker()
        
        else:
            logger.error(f"[CameraFactory] Tipo de cámara desconocido: '{camera_type}'")
            logger.error(f"[CameraFactory] Tipos válidos: 'auto', 'thorlabs', 'basler'")
            return None
    
    @staticmethod
    def get_available_camera_info() -> dict:
        """
        Obtiene información detallada de cámaras disponibles.
        
        Returns:
            Diccionario con información de cada SDK disponible
        
        Example:
            >>> info = CameraWorkerFactory.get_available_camera_info()
            >>> print(info)
            {
                'thorlabs': {'available': True, 'sdk': 'pylablib'},
                'basler': {'available': True, 'sdk': 'pypylon'}
            }
        """
        return {
            'thorlabs': {
                'available': THORLABS_AVAILABLE,
                'sdk': 'pylablib',
                'worker_class': 'ThorlabsWorker'
            },
            'basler': {
                'available': BASLER_AVAILABLE,
                'sdk': 'pypylon',
                'worker_class': 'BaslerWorker'
            }
        }
    
    @staticmethod
    def is_camera_available(camera_type: str) -> bool:
        """
        Verifica si un tipo de cámara específico está disponible.
        
        Args:
            camera_type: Tipo de cámara a verificar
        
        Returns:
            True si el SDK está disponible
        
        Example:
            >>> if CameraWorkerFactory.is_camera_available("basler"):
            ...     print("Basler disponible")
        """
        if camera_type == "thorlabs":
            return THORLABS_AVAILABLE
        elif camera_type == "basler":
            return BASLER_AVAILABLE
        else:
            return False

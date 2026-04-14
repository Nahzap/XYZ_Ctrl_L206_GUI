"""
Módulo de integración con cámaras (Thorlabs, Basler).

Contiene workers para manejar cámaras en threads separados y factory
para creación automática según hardware disponible.

Refactorizado 2026-03-05: Soporte multi-cámara con patrón Adapter + Factory
"""

from .base_camera_worker import BaseCameraWorker
from .thorlabs_worker import ThorlabsWorker, CameraWorker  # CameraWorker es alias
from .basler_worker import BaslerWorker
from .camera_factory import CameraWorkerFactory

__all__ = [
    'BaseCameraWorker',
    'ThorlabsWorker',
    'BaslerWorker',
    'CameraWorkerFactory',
    'CameraWorker',  # Compatibilidad retroactiva
]

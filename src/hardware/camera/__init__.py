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
from .scientific_config import (
    ACA2500_14UC,
    ScientificCameraSettings,
    audit_project_camera_config,
    build_scientific_settings,
    resolve_save_frame,
)
from .scientific_image import (
    PIPELINE_ID,
    ScientificFrame,
    prepare_scientific_bgr16,
    save_scientific_image,
)

__all__ = [
    'BaseCameraWorker',
    'ThorlabsWorker',
    'BaslerWorker',
    'CameraWorkerFactory',
    'CameraWorker',  # Compatibilidad retroactiva
    'ACA2500_14UC',
    'ScientificCameraSettings',
    'audit_project_camera_config',
    'build_scientific_settings',
    'resolve_save_frame',
    'PIPELINE_ID',
    'ScientificFrame',
    'prepare_scientific_bgr16',
    'save_scientific_image',
]

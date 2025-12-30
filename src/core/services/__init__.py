"""
Servicios de Procesamiento Asíncrono.

Contiene workers para detección, autofoco y cámara en background.
Todos los servicios ejecutan en threads separados (QThread) para no bloquear la UI.

- DetectionService: Detección U2-Net asíncrona
- AutofocusService: Z-scan para autofoco (50%→0%→50%→100%)
- CameraService: Orquestación de CameraWorker (conexión y live view)
- TestService: Control dual y ejecución de trayectorias
"""

from .detection_service import DetectionService
from .autofocus_service import AutofocusService
from .camera_service import CameraService
from .camera_orchestrator import CameraOrchestrator
from .test_service import TestService, ControllerConfig, TrajectoryConfig

__all__ = ['DetectionService', 'AutofocusService', 'CameraService', 
           'TestService', 'CameraOrchestrator', 'ControllerConfig', 'TrajectoryConfig']

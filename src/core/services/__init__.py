"""
Servicios de Procesamiento Asíncrono.

Contiene workers para detección y autofoco en background.
Todos los servicios ejecutan en threads separados (QThread) para no bloquear la UI.

- DetectionService: Detección U2-Net asíncrona
- AutofocusService: Z-scan para autofoco (50%→0%→50%→100%)
"""

from .detection_service import DetectionService
from .autofocus_service import AutofocusService

__all__ = ['DetectionService', 'AutofocusService']

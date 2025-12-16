"""
Módulo de Detección de Objetos Salientes.

Contiene el detector U2-Net singleton para detección eficiente.
"""

from .u2net_detector import U2NetDetector
# Re-exportar DetectedObject desde core.models para compatibilidad
from core.models.detected_object import DetectedObject

__all__ = ['U2NetDetector', 'DetectedObject']

"""
Modelos de datos unificados para el sistema de control.

Este módulo centraliza las dataclasses usadas en todo el proyecto
para evitar duplicación y conflictos de nombres.
"""

from .detected_object import DetectedObject
from .focus_result import AutofocusResult, ImageAssessmentResult, ObjectInfo
from .autofocus_config import AutofocusConfig

__all__ = [
    'DetectedObject',
    'AutofocusResult',
    'ImageAssessmentResult',
    'ObjectInfo',
    'AutofocusConfig',
]

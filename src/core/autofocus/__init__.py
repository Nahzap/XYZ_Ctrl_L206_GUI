"""
Autofocus module for multi-object detection and focusing.
Módulo de autofoco multi-objeto.
"""

from .multi_object_autofocus import (
    MultiObjectAutofocusController,
    DetectedObject,
    FocusedCapture
)
from .smart_focus_scorer import SmartFocusScorer

__all__ = [
    'MultiObjectAutofocusController',
    'DetectedObject',
    'FocusedCapture',
    'SmartFocusScorer'
]

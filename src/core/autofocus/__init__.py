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
from .focus_metric import calculate_focus_score, build_multifocal_z_positions

__all__ = [
    'MultiObjectAutofocusController',
    'DetectedObject',
    'FocusedCapture',
    'SmartFocusScorer',
    'calculate_focus_score',
    'build_multifocal_z_positions',
]

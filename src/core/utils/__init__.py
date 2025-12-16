"""
Utilidades compartidas del core.

Contiene funciones de procesamiento de imagen y métricas reutilizables.
"""

from .image_metrics import (
    calculate_laplacian_variance,
    calculate_brenner_gradient,
    preprocess_for_detection,
    normalize_image,
)

__all__ = [
    'calculate_laplacian_variance',
    'calculate_brenner_gradient',
    'preprocess_for_detection',
    'normalize_image',
]

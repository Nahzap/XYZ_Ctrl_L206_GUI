"""
Módulo de Análisis de Imagen - Índice de Nitidez (Sharpness).

Submódulos:
- background_model: Calibración y modelo estadístico del fondo
- sharpness_detector: Detector robusto de nitidez con Z-Score

Basado en Teoría de Control Robusto (Zhou/Doyle).
"""

from .background_model import (
    train_background_model,
    load_background_model,
    validate_background_model,
    BackgroundModelCache,
    get_supported_image_extensions,
    list_images_in_folder,
    generate_contrast_variations
)

from .sharpness_detector import (
    SharpnessDetector,
    SharpnessResult,
    create_debug_composite
)

from .smart_focus_scorer import (
    SmartFocusScorer,
    FocusResult,
    ObjectInfo,
    compare_focus_methods
)

__all__ = [
    # Background Model (Legacy)
    'train_background_model',
    'load_background_model',
    'validate_background_model',
    'BackgroundModelCache',
    'get_supported_image_extensions',
    'list_images_in_folder',
    'generate_contrast_variations',
    # Sharpness Detector (Legacy)
    'SharpnessDetector',
    'SharpnessResult',
    'create_debug_composite',
    # Smart Focus Scorer (NEW - Blind Assessment)
    'SmartFocusScorer',
    'FocusResult',
    'ObjectInfo',
    'compare_focus_methods'
]

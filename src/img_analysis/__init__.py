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

# SmartFocusScorer ahora viene de core.autofocus (versión unificada)
from core.autofocus.smart_focus_scorer import SmartFocusScorer, FocusResult
from core.models.focus_result import ObjectInfo, ImageAssessmentResult

# Función de comparación se mantiene local por ahora
def compare_focus_methods(img_gray):
    """Wrapper para compatibilidad - usa SmartFocusScorer unificado."""
    import cv2
    import numpy as np
    EPSILON = 1e-10
    
    laplacian = cv2.Laplacian(img_gray, cv2.CV_64F)
    laplacian_var = float(laplacian.var())
    
    img_float = img_gray.astype(np.float32)
    diff_h = img_float[:, 2:] - img_float[:, :-2]
    diff_v = img_float[2:, :] - img_float[:-2, :]
    brenner = (np.sum(diff_h ** 2) + np.sum(diff_v ** 2)) / (2 * img_gray.size + EPSILON)
    
    scorer = SmartFocusScorer()
    result = scorer.assess_image(img_gray)
    
    return {
        'laplacian_var_global': laplacian_var,
        'brenner_global': brenner,
        'roi_focus_score': result.focus_score,
        'roi_status': result.status,
    }

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

"""
FocusResult - Modelos unificados para resultados de enfoque.

Centraliza las definiciones de resultados de autofoco y evaluación de imagen.
"""

from dataclasses import dataclass, field
from typing import Tuple, Optional, List, Dict
import numpy as np


@dataclass
class ObjectInfo:
    """
    Información detallada de un objeto detectado con métricas de enfoque.
    
    Usado por SmartFocusScorer para almacenar información completa
    de cada objeto detectado incluyendo su score de enfoque.
    """
    contour: np.ndarray
    bounding_box: Tuple[int, int, int, int]  # (x, y, w, h)
    centroid: Tuple[int, int]  # (cx, cy)
    area: float
    mean_probability: float
    focus_score: float
    raw_score: float
    is_focused: bool
    
    @property
    def bbox(self) -> Tuple[int, int, int, int]:
        """Alias para compatibilidad."""
        return self.bounding_box


@dataclass
class AutofocusResult:
    """
    Resultado de autofoco Z-scan para un objeto.
    
    Usado por AutofocusService para almacenar el resultado
    del escaneo Z de un objeto específico.
    
    Incluye 2 capturas:
    - frame: Imagen en BPoF (máximo enfoque)
    - frame_alt: Imagen en foco alternativo (ligeramente desenfocada)
    """
    object_index: int
    z_optimal: float
    focus_score: float
    bbox: Tuple[int, int, int, int]
    frame: Optional[np.ndarray] = None
    # Captura alternativa (foco diferente)
    frame_alt: Optional[np.ndarray] = None
    z_alt: float = 0.0  # Posición Z de la captura alternativa
    score_alt: float = 0.0  # Score de la captura alternativa
    
    @property
    def bounding_box(self) -> Tuple[int, int, int, int]:
        """Alias para compatibilidad."""
        return self.bbox


@dataclass
class ImageAssessmentResult:
    """
    Resultado de evaluación de imagen completa.
    
    Usado por SmartFocusScorer.assess_image() para retornar
    información completa sobre todos los objetos detectados.
    """
    status: str  # "FOCUSED_OBJECT", "BLURRY_OBJECT", "EMPTY", "MULTIPLE_OBJECTS", "ERROR"
    focus_score: float
    centroid: Optional[Tuple[int, int]] = None
    bounding_box: Optional[Tuple[int, int, int, int]] = None
    contour_area: float = 0.0
    raw_score: float = 0.0
    is_valid: bool = False
    num_objects: int = 0
    mean_probability: float = 0.0
    objects: List[ObjectInfo] = field(default_factory=list)
    debug_mask: Optional[np.ndarray] = field(default=None, repr=False)
    probability_map: Optional[np.ndarray] = field(default=None, repr=False)
    binary_mask: Optional[np.ndarray] = field(default=None, repr=False)
    
    # Campos legacy para compatibilidad
    entropy: float = 0.0
    raw_brenner: float = 0.0
    
    @property
    def bbox(self) -> Optional[Tuple[int, int, int, int]]:
        """Alias para compatibilidad."""
        return self.bounding_box
    
    def to_dict(self) -> Dict:
        """Convierte a diccionario para serialización."""
        return {
            'status': self.status,
            'focus_score': self.focus_score,
            'centroid': self.centroid,
            'bounding_box': self.bounding_box,
            'contour_area': self.contour_area,
            'raw_score': self.raw_score,
            'is_valid': self.is_valid,
            'num_objects': self.num_objects,
            'mean_probability': self.mean_probability,
            'objects': [
                {
                    'bbox': o.bounding_box, 
                    'score': o.focus_score, 
                    'focused': o.is_focused
                } 
                for o in self.objects
            ]
        }


# Alias para compatibilidad con código existente
FocusResult = ImageAssessmentResult

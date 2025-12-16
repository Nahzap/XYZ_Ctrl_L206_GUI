"""
DetectedObject - Modelo unificado para objetos detectados.

Centraliza la definición de objetos detectados para evitar
duplicación entre u2net_detector.py y multi_object_autofocus.py.
"""

from dataclasses import dataclass
from typing import Tuple, Optional
import numpy as np


@dataclass
class DetectedObject:
    """
    Objeto detectado unificado.
    
    Combina los campos de ambas versiones anteriores:
    - core/detection/u2net_detector.py
    - core/autofocus/multi_object_autofocus.py
    
    Attributes:
        index: Índice del objeto en la lista de detecciones
        bbox: Bounding box (x, y, w, h) - NOMBRE ESTÁNDAR
        area: Área del objeto en píxeles
        probability: Probabilidad de detección (0-1)
        centroid: Centro del objeto (cx, cy)
        contour: Contorno OpenCV del objeto (opcional)
        circularity: Circularidad del objeto (0-1, 1=círculo perfecto)
        focus_score: Score de enfoque del objeto
        is_focused: Si el objeto está enfocado
    """
    index: int
    bbox: Tuple[int, int, int, int]  # (x, y, w, h)
    area: float
    probability: float
    centroid: Tuple[int, int]
    contour: Optional[np.ndarray] = None
    circularity: float = 0.0
    focus_score: float = 0.0
    is_focused: bool = False
    
    @property
    def bounding_box(self) -> Tuple[int, int, int, int]:
        """Alias para compatibilidad con código que usa bounding_box."""
        return self.bbox
    
    @property
    def x(self) -> int:
        """Coordenada X del bounding box."""
        return self.bbox[0]
    
    @property
    def y(self) -> int:
        """Coordenada Y del bounding box."""
        return self.bbox[1]
    
    @property
    def w(self) -> int:
        """Ancho del bounding box."""
        return self.bbox[2]
    
    @property
    def h(self) -> int:
        """Alto del bounding box."""
        return self.bbox[3]

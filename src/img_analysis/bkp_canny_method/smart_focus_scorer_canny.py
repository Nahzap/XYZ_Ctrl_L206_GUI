"""
Smart Focus Scorer - ROI-Based Focus Assessment.

Estrategia robusta para microscopía automatizada:
1. Detección de candidatos mediante Canny edge detection
2. Filtrado de ruido por área mínima de contornos
3. Selección del objeto principal (mayor área)
4. Puntuación de enfoque SOLO en el ROI del objeto

Ventajas:
- No requiere calibración previa
- Inmune a ruido de fondo (solo evalúa el objeto)
- Proporciona localización del objeto (centroide, bounding box)
- Discrimina entre objeto enfocado, desenfocado y vacío

Referencias:
- Brenner et al. (1976): "An automated microscope for cytologic research"
- Canny (1986): "A Computational Approach to Edge Detection"
"""

import numpy as np
import cv2
import logging
from typing import Union, Dict, Optional, Tuple
from dataclasses import dataclass, field
from pathlib import Path

logger = logging.getLogger('MotorControl_L206')

# Constantes
EPSILON = 1e-10


@dataclass
class FocusResult:
    """Resultado de evaluación de enfoque con información de localización."""
    status: str  # "FOCUSED_OBJECT", "BLURRY_OBJECT", "EMPTY"
    focus_score: float
    centroid: Optional[Tuple[int, int]] = None  # (cx, cy)
    bounding_box: Optional[Tuple[int, int, int, int]] = None  # (x, y, w, h)
    contour_area: float = 0.0
    raw_score: float = 0.0
    is_valid: bool = False
    debug_mask: Optional[np.ndarray] = field(default=None, repr=False)
    
    # Compatibilidad con versión anterior
    entropy: float = 0.0
    raw_brenner: float = 0.0
    
    def to_dict(self) -> Dict:
        return {
            'status': self.status,
            'focus_score': self.focus_score,
            'centroid': self.centroid,
            'bounding_box': self.bounding_box,
            'contour_area': self.contour_area,
            'raw_score': self.raw_score,
            'is_valid': self.is_valid
        }


class SmartFocusScorer:
    """
    Evaluador de Enfoque basado en ROI (Region of Interest).
    
    Pipeline:
    1. Detección de bordes (Canny) para encontrar candidatos
    2. Filtrado de contornos por área mínima (elimina ruido)
    3. Selección del contorno principal (mayor área)
    4. Cálculo de enfoque SOLO dentro del bounding box del objeto
    
    Esto evita falsos positivos donde el ruido acumulado del fondo
    genera scores similares a objetos reales.
    """
    
    def __init__(
        self,
        min_area: int = 100,
        canny_low: int = 50,
        canny_high: int = 150,
        blur_kernel: int = 3,
        focus_threshold: float = 50.0,
        use_laplacian: bool = True,
        roi_margin: int = 5
    ):
        """
        Inicializa el evaluador ROI-Based.
        
        Args:
            min_area: Área mínima de contorno para considerar objeto válido (px²)
            canny_low: Umbral bajo para Canny edge detection
            canny_high: Umbral alto para Canny edge detection
            blur_kernel: Tamaño del kernel de GaussianBlur (debe ser impar)
            focus_threshold: Umbral para clasificar como enfocado vs desenfocado
            use_laplacian: Si True usa varianza de Laplaciano, si False usa Brenner
            roi_margin: Margen extra alrededor del bounding box (px)
        """
        self.min_area = min_area
        self.canny_low = canny_low
        self.canny_high = canny_high
        self.blur_kernel = blur_kernel if blur_kernel % 2 == 1 else blur_kernel + 1
        self.focus_threshold = focus_threshold
        self.use_laplacian = use_laplacian
        self.roi_margin = roi_margin
        
        # Compatibilidad con versión anterior
        self.entropy_threshold = 4.0
        
        logger.info(f"[SmartFocusScorer] ROI-Based: min_area={min_area}, "
                   f"canny=({canny_low},{canny_high}), focus_thresh={focus_threshold}")
    
    def _detect_edges(self, img_gray: np.ndarray) -> np.ndarray:
        """
        Detecta bordes usando Canny con pre-procesamiento.
        
        Args:
            img_gray: Imagen en escala de grises
            
        Returns:
            Mapa binario de bordes
        """
        # Suavizado para reducir ruido
        blurred = cv2.GaussianBlur(img_gray, (self.blur_kernel, self.blur_kernel), 0)
        
        # Detección de bordes Canny
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        
        return edges
    
    def _find_main_contour(self, edges: np.ndarray) -> Optional[np.ndarray]:
        """
        Encuentra el contorno principal (mayor área) que supera el umbral mínimo.
        
        Args:
            edges: Mapa binario de bordes
            
        Returns:
            Contorno principal o None si no hay candidatos válidos
        """
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Filtrar por área mínima y encontrar el mayor
        valid_contours = [(c, cv2.contourArea(c)) for c in contours if cv2.contourArea(c) >= self.min_area]
        
        if not valid_contours:
            return None
        
        # Seleccionar el de mayor área
        main_contour = max(valid_contours, key=lambda x: x[1])[0]
        return main_contour
    
    def _get_contour_info(self, contour: np.ndarray, img_shape: Tuple[int, int]) -> Tuple[Tuple[int, int], Tuple[int, int, int, int], float]:
        """
        Extrae información del contorno: centroide, bounding box y área.
        
        Args:
            contour: Contorno de OpenCV
            img_shape: (height, width) de la imagen
            
        Returns:
            (centroid, bounding_box, area)
        """
        # Área
        area = cv2.contourArea(contour)
        
        # Momentos para centroide
        M = cv2.moments(contour)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            # Fallback: centro del bounding box
            x, y, w, h = cv2.boundingRect(contour)
            cx, cy = x + w // 2, y + h // 2
        
        # Bounding box con margen
        x, y, w, h = cv2.boundingRect(contour)
        
        # Aplicar margen respetando límites de imagen
        h_img, w_img = img_shape
        x = max(0, x - self.roi_margin)
        y = max(0, y - self.roi_margin)
        w = min(w_img - x, w + 2 * self.roi_margin)
        h = min(h_img - y, h + 2 * self.roi_margin)
        
        return (cx, cy), (x, y, w, h), area
    
    def _calculate_roi_focus(self, img_gray: np.ndarray, bbox: Tuple[int, int, int, int]) -> Tuple[float, float]:
        """
        Calcula el score de enfoque SOLO dentro del ROI.
        
        Args:
            img_gray: Imagen completa en escala de grises
            bbox: (x, y, w, h) del bounding box
            
        Returns:
            (focus_score_normalized, raw_score)
        """
        x, y, w, h = bbox
        roi = img_gray[y:y+h, x:x+w]
        
        if roi.size == 0:
            return 0.0, 0.0
        
        if self.use_laplacian:
            # Varianza del Laplaciano (más sensible a enfoque)
            laplacian = cv2.Laplacian(roi, cv2.CV_64F)
            raw_score = float(laplacian.var())
        else:
            # Gradiente de Brenner
            roi_float = roi.astype(np.float32)
            if roi.shape[1] > 2:
                diff_h = roi_float[:, 2:] - roi_float[:, :-2]
                energy_h = np.sum(diff_h ** 2)
            else:
                energy_h = 0
            if roi.shape[0] > 2:
                diff_v = roi_float[2:, :] - roi_float[:-2, :]
                energy_v = np.sum(diff_v ** 2)
            else:
                energy_v = 0
            raw_score = (energy_h + energy_v) / (2 * roi.size + EPSILON)
        
        # Normalización: sqrt para linealizar respuesta
        focus_score = np.sqrt(raw_score)
        
        return float(focus_score), float(raw_score)
    
    def assess_image(
        self, 
        image: Union[str, Path, np.ndarray],
        return_debug_mask: bool = True
    ) -> FocusResult:
        """
        Evalúa el enfoque de una imagen usando detección ROI-Based.
        
        Pipeline:
        1. Detectar bordes (Canny)
        2. Encontrar contornos y filtrar por área mínima
        3. Seleccionar objeto principal (mayor área)
        4. Calcular enfoque SOLO en el ROI del objeto
        5. Clasificar: FOCUSED_OBJECT, BLURRY_OBJECT, o EMPTY
        
        Args:
            image: Ruta a imagen o array numpy
            return_debug_mask: Si True, incluye máscara de bordes en resultado
            
        Returns:
            FocusResult con status, focus_score, centroid, bounding_box
        """
        # Cargar imagen si es ruta
        if isinstance(image, (str, Path)):
            img_gray = cv2.imread(str(image), cv2.IMREAD_GRAYSCALE)
            if img_gray is None:
                logger.error(f"[SmartFocusScorer] No se pudo cargar: {image}")
                return FocusResult(
                    status="ERROR",
                    focus_score=0.0,
                    is_valid=False
                )
        else:
            img_gray = image
            if len(img_gray.shape) == 3:
                img_gray = cv2.cvtColor(img_gray, cv2.COLOR_BGR2GRAY)
        
        # PASO 1: Detectar bordes
        edges = self._detect_edges(img_gray)
        
        # Crear máscara de visualización (siempre)
        debug_mask = self._create_debug_visualization(img_gray, edges, None, None, None)
        
        # PASO 2: Encontrar contorno principal
        main_contour = self._find_main_contour(edges)
        
        if main_contour is None:
            # No hay objeto válido
            logger.debug(f"[SmartFocusScorer] EMPTY: No se encontró objeto con área >= {self.min_area}")
            return FocusResult(
                status="EMPTY",
                focus_score=0.0,
                centroid=None,
                bounding_box=None,
                contour_area=0.0,
                raw_score=0.0,
                is_valid=False,
                debug_mask=debug_mask
            )
        
        # PASO 3: Extraer información del contorno
        centroid, bbox, area = self._get_contour_info(main_contour, img_gray.shape)
        
        # PASO 4: Calcular enfoque en ROI
        focus_score, raw_score = self._calculate_roi_focus(img_gray, bbox)
        
        # PASO 5: Clasificar
        if focus_score >= self.focus_threshold:
            status = "FOCUSED_OBJECT"
            is_valid = True
        else:
            status = "BLURRY_OBJECT"
            is_valid = True  # Hay objeto, pero desenfocado
        
        # Crear visualización de debug con toda la info
        debug_mask = self._create_debug_visualization(img_gray, edges, main_contour, centroid, bbox)
        
        logger.debug(f"[SmartFocusScorer] {status}: score={focus_score:.2f}, "
                    f"area={area:.0f}, centroid={centroid}, bbox={bbox}")
        
        return FocusResult(
            status=status,
            focus_score=focus_score,
            centroid=centroid,
            bounding_box=bbox,
            contour_area=area,
            raw_score=raw_score,
            is_valid=is_valid,
            debug_mask=debug_mask,
            # Compatibilidad
            entropy=0.0,
            raw_brenner=raw_score
        )
    
    def _create_debug_visualization(
        self, 
        img_gray: np.ndarray, 
        edges: np.ndarray,
        contour: Optional[np.ndarray],
        centroid: Optional[Tuple[int, int]],
        bbox: Optional[Tuple[int, int, int, int]]
    ) -> np.ndarray:
        """
        Crea una visualización de debug mostrando bordes, contorno y ROI.
        
        Returns:
            Imagen BGR con visualización de debug
        """
        h, w = img_gray.shape
        
        # Crear imagen de 3 canales
        debug = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
        
        # Superponer bordes en rojo
        edges_colored = np.zeros((h, w, 3), dtype=np.uint8)
        edges_colored[:, :, 2] = edges  # Canal rojo
        debug = cv2.addWeighted(debug, 0.7, edges_colored, 0.5, 0)
        
        if contour is not None:
            # Dibujar contorno en verde
            cv2.drawContours(debug, [contour], -1, (0, 255, 0), 2)
        
        if bbox is not None:
            # Dibujar bounding box en amarillo
            x, y, bw, bh = bbox
            cv2.rectangle(debug, (x, y), (x + bw, y + bh), (0, 255, 255), 2)
        
        if centroid is not None:
            # Dibujar cruz en el centroide (cian)
            cx, cy = centroid
            cv2.drawMarker(debug, (cx, cy), (255, 255, 0), cv2.MARKER_CROSS, 20, 2)
        
        return debug
    
    def assess_batch(
        self, 
        images: list,
        progress_callback: Optional[callable] = None
    ) -> list:
        """
        Evalúa un lote de imágenes.
        
        Args:
            images: Lista de rutas o arrays
            progress_callback: Función callback(current, total, filename)
            
        Returns:
            Lista de FocusResult
        """
        results = []
        total = len(images)
        
        for i, img in enumerate(images):
            result = self.assess_image(img)
            results.append(result)
            
            if progress_callback:
                filename = str(img) if isinstance(img, (str, Path)) else f"image_{i}"
                progress_callback(i + 1, total, filename)
        
        return results
    
    def get_parameters(self) -> Dict:
        """Retorna parámetros actuales."""
        return {
            'min_area': self.min_area,
            'canny_low': self.canny_low,
            'canny_high': self.canny_high,
            'blur_kernel': self.blur_kernel,
            'focus_threshold': self.focus_threshold,
            'use_laplacian': self.use_laplacian,
            'roi_margin': self.roi_margin
        }
    
    def set_parameters(
        self,
        min_area: Optional[int] = None,
        canny_low: Optional[int] = None,
        canny_high: Optional[int] = None,
        focus_threshold: Optional[float] = None,
        **kwargs
    ):
        """Actualiza parámetros."""
        if min_area is not None:
            self.min_area = min_area
        if canny_low is not None:
            self.canny_low = canny_low
        if canny_high is not None:
            self.canny_high = canny_high
        if focus_threshold is not None:
            self.focus_threshold = focus_threshold
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)


def compare_focus_methods(img_gray: np.ndarray) -> Dict:
    """
    Compara diferentes métodos de evaluación de enfoque en toda la imagen.
    
    Útil para debugging y comparación.
    
    Args:
        img_gray: Imagen en escala de grises
        
    Returns:
        Dict con scores de cada método
    """
    # Laplacian variance (global)
    laplacian = cv2.Laplacian(img_gray, cv2.CV_64F)
    laplacian_var = float(laplacian.var())
    
    # Brenner (global)
    img_float = img_gray.astype(np.float32)
    diff_h = img_float[:, 2:] - img_float[:, :-2]
    diff_v = img_float[2:, :] - img_float[:-2, :]
    brenner = (np.sum(diff_h ** 2) + np.sum(diff_v ** 2)) / (2 * img_gray.size + EPSILON)
    
    # Tenengrad (gradiente Sobel)
    sobel_x = cv2.Sobel(img_gray, cv2.CV_64F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(img_gray, cv2.CV_64F, 0, 1, ksize=3)
    tenengrad = np.mean(sobel_x**2 + sobel_y**2)
    
    # Entropía
    hist, _ = np.histogram(img_gray.ravel(), bins=256, range=(0, 256))
    hist = hist.astype(np.float64)
    hist = hist / (hist.sum() + EPSILON)
    hist = hist[hist > EPSILON]
    entropy = -np.sum(hist * np.log2(hist))
    
    # ROI-Based (nuevo método)
    scorer = SmartFocusScorer()
    result = scorer.assess_image(img_gray)
    
    return {
        'laplacian_var_global': laplacian_var,
        'brenner_global': brenner,
        'tenengrad_global': tenengrad,
        'entropy_global': entropy,
        'roi_focus_score': result.focus_score,
        'roi_status': result.status,
        'roi_area': result.contour_area,
        'roi_centroid': result.centroid,
        'roi_bbox': result.bounding_box
    }

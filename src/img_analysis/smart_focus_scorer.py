"""
Smart Focus Scorer - U2-Net Salient Object Detection.

Estrategia avanzada para microscopía automatizada usando Deep Learning:
1. Detección de objetos salientes mediante U2-Net (Salient Object Detection)
2. Segmentación precisa del objeto (polen/célula) del fondo ruidoso
3. Cálculo de enfoque SOLO en los píxeles del objeto detectado

Ventajas:
- No requiere calibración previa
- Inmune a ruido de fondo (deep learning)
- Segmentación precisa de objetos complejos
- Funciona con fondos sucios/ruidosos

Referencias:
- Qin et al. (2020): "U2-Net: Going Deeper with Nested U-Structure for Salient Object Detection"
- Brenner et al. (1976): "An automated microscope for cytologic research"
"""

import numpy as np
import cv2
import logging
from typing import Union, Dict, Optional, Tuple, List
from dataclasses import dataclass, field
from pathlib import Path

logger = logging.getLogger('MotorControl_L206')

# Constantes
EPSILON = 1e-10


@dataclass
class ObjectInfo:
    """Información de un objeto detectado."""
    contour: np.ndarray
    bounding_box: Tuple[int, int, int, int]  # (x, y, w, h)
    centroid: Tuple[int, int]  # (cx, cy)
    area: float
    mean_probability: float
    focus_score: float
    raw_score: float
    is_focused: bool


@dataclass
class FocusResult:
    """Resultado de evaluación de enfoque con información de localización."""
    status: str  # "FOCUSED_OBJECT", "BLURRY_OBJECT", "EMPTY", "MULTIPLE_OBJECTS"
    focus_score: float  # Score del objeto principal (mayor área)
    centroid: Optional[Tuple[int, int]] = None  # (cx, cy) del principal
    bounding_box: Optional[Tuple[int, int, int, int]] = None  # (x, y, w, h) del principal
    contour_area: float = 0.0
    raw_score: float = 0.0
    is_valid: bool = False
    num_objects: int = 0  # Número de objetos detectados
    mean_probability: float = 0.0  # Probabilidad promedio del objeto principal
    objects: List[ObjectInfo] = field(default_factory=list)  # Lista de TODOS los objetos
    debug_mask: Optional[np.ndarray] = field(default=None, repr=False)
    probability_map: Optional[np.ndarray] = field(default=None, repr=False)
    binary_mask: Optional[np.ndarray] = field(default=None, repr=False)  # Máscara binaria
    
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
            'is_valid': self.is_valid,
            'num_objects': self.num_objects,
            'mean_probability': self.mean_probability,
            'objects': [{'bbox': o.bounding_box, 'score': o.focus_score, 'focused': o.is_focused} for o in self.objects]
        }


class SmartFocusScorer:
    """
    Evaluador de Enfoque usando U2-Net para Salient Object Detection.
    
    Pipeline:
    1. Segmentación del objeto saliente usando U2-Net (deep learning)
    2. Binarización de la máscara de probabilidad
    3. Extracción de bounding box y centroide
    4. Cálculo de enfoque SOLO en los píxeles del objeto
    
    Esto elimina completamente el ruido de fondo usando inteligencia artificial.
    """
    
    def __init__(
        self,
        model_type: str = 'u2netp',
        threshold: float = 0.5,
        min_area: int = 28000,
        max_area: int = 35000,
        min_prob: float = 0.3,
        focus_threshold: float = 50.0,
        use_laplacian: bool = True,
        device: Optional[str] = None
    ):
        """
        Inicializa el evaluador con U2-Net.
        
        Args:
            model_type: 'u2netp' (rápido, ~4MB) o 'u2net' (preciso, ~176MB)
            threshold: Umbral de binarización para la máscara de saliencia
            min_area: Área mínima para considerar objeto válido (px²)
            max_area: Área máxima para considerar objeto válido (px²)
            min_prob: Probabilidad mínima promedio para considerar objeto real
            focus_threshold: Umbral para clasificar como enfocado vs desenfocado
            use_laplacian: Si True usa varianza de Laplaciano, si False usa Brenner
            device: 'cuda', 'cpu' o None (auto-detecta)
        """
        self.model_type = model_type
        self.threshold = threshold
        self.min_area = min_area
        self.max_area = max_area
        self.min_prob = min_prob
        self.focus_threshold = focus_threshold
        self.use_laplacian = use_laplacian
        self.device = device
        
        # Detector de objetos salientes (lazy loading)
        self._detector = None
        self._model_loaded = False
        self._load_error = None
        
        # Compatibilidad con versión anterior
        self.entropy_threshold = 4.0
        
        logger.info(f"[SmartFocusScorer] U2-Net: model={model_type}, "
                   f"threshold={threshold}, min_area={min_area}, max_area={max_area}, min_prob={min_prob}")
    
    def _ensure_model_loaded(self) -> bool:
        """
        Carga el modelo U2-Net si no está cargado (lazy loading).
        
        Returns:
            True si el modelo está listo, False si hay error
        """
        if self._model_loaded:
            return True
        
        if self._load_error:
            return False
        
        try:
            from ai_segmentation import SalientObjectDetector
            self._detector = SalientObjectDetector(
                model_type=self.model_type,
                device=self.device,
                auto_download=True
            )
            self._model_loaded = True
            logger.info("[SmartFocusScorer] Modelo U2-Net cargado exitosamente")
            return True
        except Exception as e:
            self._load_error = str(e)
            logger.error(f"[SmartFocusScorer] Error cargando U2-Net: {e}")
            return False
    
    def _get_saliency_mask(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Obtiene la máscara de saliencia usando U2-Net.
        
        Args:
            image: Imagen BGR o grayscale
            
        Returns:
            (binary_mask, probability_map)
        """
        if not self._ensure_model_loaded():
            # Fallback: máscara vacía
            h, w = image.shape[:2]
            return np.zeros((h, w), dtype=np.uint8), np.zeros((h, w), dtype=np.float32)
        
        # Obtener máscara de probabilidad
        prob_map = self._detector.get_mask(image, return_probability=True)
        
        # Binarizar
        binary_mask = (prob_map > self.threshold).astype(np.uint8) * 255
        
        return binary_mask, prob_map
    
    def _find_all_objects(self, binary_mask: np.ndarray, prob_map: np.ndarray, img_gray: np.ndarray) -> List[ObjectInfo]:
        """
        Encuentra TODOS los objetos válidos en la máscara, con sus scores individuales.
        
        Args:
            binary_mask: Máscara binaria de saliencia
            prob_map: Mapa de probabilidad para validar objetos reales
            img_gray: Imagen en escala de grises para calcular focus
            
        Returns:
            Lista de ObjectInfo ordenada por área (mayor primero)
        """
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return []
        
        objects = []
        for c in contours:
            area = cv2.contourArea(c)
            # Filtrar por área mínima Y máxima
            if area < self.min_area or area > self.max_area:
                continue
            
            # Crear máscara para este contorno
            contour_mask = np.zeros(binary_mask.shape, dtype=np.uint8)
            cv2.drawContours(contour_mask, [c], -1, 255, -1)
            
            # Probabilidad promedio dentro del contorno
            mean_prob = float(cv2.mean(prob_map, mask=contour_mask)[0])
            
            if mean_prob < self.min_prob:
                continue
            
            # Bounding box
            x, y, w, h = cv2.boundingRect(c)
            
            # Centroide
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = x + w // 2, y + h // 2
            
            # Calcular focus score para ESTE objeto
            focus_score, raw_score = self._calculate_masked_focus(img_gray, contour_mask)
            is_focused = focus_score >= self.focus_threshold
            
            objects.append(ObjectInfo(
                contour=c,
                bounding_box=(x, y, w, h),
                centroid=(cx, cy),
                area=area,
                mean_probability=mean_prob,
                focus_score=focus_score,
                raw_score=raw_score,
                is_focused=is_focused
            ))
        
        # Ordenar por área (mayor primero)
        objects.sort(key=lambda o: o.area, reverse=True)
        
        return objects
    
    def _calculate_masked_focus(self, img_gray: np.ndarray, mask: np.ndarray) -> Tuple[float, float]:
        """
        Calcula el score de enfoque SOLO en los píxeles de la máscara.
        
        Args:
            img_gray: Imagen en escala de grises
            mask: Máscara binaria (255 = objeto)
            
        Returns:
            (focus_score_normalized, raw_score)
        """
        # Contar píxeles válidos
        mask_bool = mask > 127
        n_pixels = np.sum(mask_bool)
        
        if n_pixels < 10:
            return 0.0, 0.0
        
        if self.use_laplacian:
            # Varianza del Laplaciano solo en píxeles de la máscara
            laplacian = cv2.Laplacian(img_gray, cv2.CV_64F)
            masked_laplacian = laplacian[mask_bool]
            raw_score = float(np.var(masked_laplacian))
        else:
            # Brenner solo en píxeles de la máscara
            img_float = img_gray.astype(np.float32)
            
            # Gradientes
            diff_h = np.zeros_like(img_float)
            diff_v = np.zeros_like(img_float)
            diff_h[:, 2:] = img_float[:, 2:] - img_float[:, :-2]
            diff_v[2:, :] = img_float[2:, :] - img_float[:-2, :]
            
            # Energía solo en máscara
            energy = (diff_h ** 2 + diff_v ** 2)[mask_bool]
            raw_score = float(np.mean(energy))
        
        # Normalización
        focus_score = np.sqrt(raw_score)
        
        return float(focus_score), float(raw_score)
    
    def assess_image(
        self, 
        image: Union[str, Path, np.ndarray],
        return_debug_mask: bool = True
    ) -> FocusResult:
        """
        Evalúa el enfoque de una imagen usando U2-Net Salient Object Detection.
        
        Pipeline:
        1. Obtener máscara de saliencia con U2-Net
        2. Binarizar y encontrar objeto principal
        3. Calcular enfoque SOLO en los píxeles del objeto
        4. Clasificar: FOCUSED_OBJECT, BLURRY_OBJECT, o EMPTY
        
        Args:
            image: Ruta a imagen o array numpy
            return_debug_mask: Si True, incluye visualización de debug
            
        Returns:
            FocusResult con status, focus_score, centroid, bounding_box
        """
        # Cargar imagen
        if isinstance(image, (str, Path)):
            img_bgr = cv2.imread(str(image))
            if img_bgr is None:
                logger.error(f"[SmartFocusScorer] No se pudo cargar: {image}")
                return FocusResult(
                    status="ERROR",
                    focus_score=0.0,
                    is_valid=False
                )
            img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        else:
            if len(image.shape) == 3:
                img_bgr = image
                img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                img_gray = image
                img_bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        
        # PASO 1: Obtener máscara de saliencia con U2-Net
        binary_mask, prob_map = self._get_saliency_mask(img_bgr)
        
        # PASO 2: Encontrar TODOS los objetos válidos con sus scores
        objects = self._find_all_objects(binary_mask, prob_map, img_gray)
        num_objects = len(objects)
        
        # Calcular probabilidad promedio global
        global_mean_prob = float(np.mean(prob_map))
        
        if not objects:
            # No hay objeto válido (ni por área ni por probabilidad)
            logger.debug(f"[SmartFocusScorer] EMPTY: No se encontró objeto (min_area={self.min_area}, min_prob={self.min_prob})")
            debug_mask = self._create_debug_visualization(img_gray, binary_mask, prob_map, objects)
            return FocusResult(
                status="EMPTY",
                focus_score=0.0,
                centroid=None,
                bounding_box=None,
                contour_area=0.0,
                raw_score=0.0,
                is_valid=False,
                num_objects=0,
                mean_probability=global_mean_prob,
                objects=[],
                debug_mask=debug_mask,
                probability_map=prob_map,
                binary_mask=binary_mask
            )
        
        # El objeto principal es el de mayor área (primero en la lista)
        main_obj = objects[0]
        
        # Contar objetos enfocados
        focused_count = sum(1 for o in objects if o.is_focused)
        
        # PASO 3: Clasificar según cantidad de objetos
        if num_objects > 1:
            status = "MULTIPLE_OBJECTS"
        elif main_obj.is_focused:
            status = "FOCUSED_OBJECT"
        else:
            status = "BLURRY_OBJECT"
        
        is_valid = True
        
        # Crear visualización de debug con TODOS los objetos
        debug_mask = self._create_debug_visualization(img_gray, binary_mask, prob_map, objects)
        
        logger.debug(f"[SmartFocusScorer] {status}: {num_objects} obj, {focused_count} focused, "
                    f"main_score={main_obj.focus_score:.2f}")
        
        return FocusResult(
            status=status,
            focus_score=main_obj.focus_score,
            centroid=main_obj.centroid,
            bounding_box=main_obj.bounding_box,
            contour_area=main_obj.area,
            raw_score=main_obj.raw_score,
            is_valid=is_valid,
            num_objects=num_objects,
            mean_probability=main_obj.mean_probability,
            objects=objects,
            debug_mask=debug_mask,
            probability_map=prob_map,
            binary_mask=binary_mask,
            entropy=0.0,
            raw_brenner=main_obj.raw_score
        )
    
    def _create_debug_visualization(
        self, 
        img_gray: np.ndarray, 
        binary_mask: np.ndarray,
        prob_map: Optional[np.ndarray],
        objects: List[ObjectInfo]
    ) -> np.ndarray:
        """
        Crea una visualización de debug mostrando TODOS los objetos con sus scores.
        
        Args:
            img_gray: Imagen en escala de grises
            binary_mask: Máscara binaria
            prob_map: Mapa de probabilidad
            objects: Lista de objetos detectados
            
        Returns:
            Imagen BGR con visualización de debug
        """
        h, w = img_gray.shape
        
        # Crear imagen base
        debug = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
        
        # Superponer máscara de probabilidad como heatmap
        if prob_map is not None:
            prob_uint8 = (prob_map * 255).astype(np.uint8)
            heatmap = cv2.applyColorMap(prob_uint8, cv2.COLORMAP_JET)
            debug = cv2.addWeighted(debug, 0.6, heatmap, 0.4, 0)
        
        # Dibujar CADA objeto con su score
        for i, obj in enumerate(objects):
            # Color según si está enfocado: verde = enfocado, rojo = desenfocado
            color = (0, 255, 0) if obj.is_focused else (0, 0, 255)
            
            # Contorno
            cv2.drawContours(debug, [obj.contour], -1, color, 2)
            
            # Bounding box
            x, y, bw, bh = obj.bounding_box
            cv2.rectangle(debug, (x, y), (x + bw, y + bh), (0, 255, 255), 1)
            
            # Score arriba a la derecha del bbox
            score_text = f"S:{obj.focus_score:.1f}"
            text_x = x + bw - 5  # Alineado a la derecha
            text_y = y - 5 if y > 20 else y + 15  # Arriba o abajo si no hay espacio
            
            # Fondo para el texto
            (tw, th), _ = cv2.getTextSize(score_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(debug, (text_x - tw - 2, text_y - th - 2), 
                         (text_x + 2, text_y + 2), (0, 0, 0), -1)
            
            # Texto del score
            cv2.putText(debug, score_text, (text_x - tw, text_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
            
            # Centroide (solo para el principal)
            if i == 0:
                cx, cy = obj.centroid
                cv2.drawMarker(debug, (cx, cy), (255, 255, 0), cv2.MARKER_CROSS, 15, 2)
        
        # Info general en esquina superior izquierda
        info_text = f"Obj: {len(objects)} | Thresh: {self.focus_threshold:.0f}"
        cv2.putText(debug, info_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(debug, info_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        
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
            'model_type': self.model_type,
            'threshold': self.threshold,
            'min_area': self.min_area,
            'min_prob': self.min_prob,
            'focus_threshold': self.focus_threshold,
            'use_laplacian': self.use_laplacian,
            'device': self.device,
            'model_loaded': self._model_loaded
        }
    
    def set_parameters(
        self,
        threshold: Optional[float] = None,
        min_area: Optional[int] = None,
        max_area: Optional[int] = None,
        min_prob: Optional[float] = None,
        focus_threshold: Optional[float] = None,
        use_laplacian: Optional[bool] = None,
        **kwargs
    ):
        """Actualiza parámetros."""
        if threshold is not None:
            self.threshold = threshold
        if min_area is not None:
            self.min_area = min_area
        if max_area is not None:
            self.max_area = max_area
        if min_prob is not None:
            self.min_prob = min_prob
        if focus_threshold is not None:
            self.focus_threshold = focus_threshold
        if use_laplacian is not None:
            self.use_laplacian = use_laplacian
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
        logger.debug(f"[SmartFocusScorer] Params: min_area={self.min_area}, max_area={self.max_area}")
    
    def is_model_loaded(self) -> bool:
        """Verifica si el modelo U2-Net está cargado."""
        return self._model_loaded
    
    def get_load_error(self) -> Optional[str]:
        """Retorna el error de carga si hubo alguno."""
        return self._load_error


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

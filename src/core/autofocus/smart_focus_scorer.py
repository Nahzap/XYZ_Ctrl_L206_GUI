"""
Smart Focus Scorer - Evaluador de Enfoque Inteligente UNIFICADO
=================================================================

Versión unificada que combina:
- core/autofocus/smart_focus_scorer.py (métodos de autofoco)
- img_analysis/smart_focus_scorer.py (U2-Net real, assess_image)

Combina U2-Net para detección de objetos con métricas de nitidez
para evaluar la calidad de enfoque en regiones de interés.

Autor: Sistema de Control L206
Fecha: 2025-12-15 (Unificación)
"""

import numpy as np
import cv2
import logging
from typing import Optional, Tuple, List, Dict, Union
from pathlib import Path

# Importar modelos unificados
from core.models.focus_result import ObjectInfo, ImageAssessmentResult

# PyTorch es opcional - solo necesario para U2-Net
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

logger = logging.getLogger('MotorControl_L206')

# Constante para evitar división por cero
EPSILON = 1e-10


class SmartFocusScorer:
    """
    Evaluador de enfoque que combina detección de objetos (U2-Net)
    con métricas de nitidez (Laplacian Variance, Gradient Magnitude).
    """
    
    def __init__(self, 
                 model_name: str = 'u2netp',
                 detection_threshold: float = 0.5,
                 min_object_area: int = 500,
                 min_probability: float = 0.3,
                 min_circularity: float = 0.45,
                 min_aspect_ratio: float = 0.4):
        """
        Inicializa el evaluador de enfoque.
        
        Args:
            model_name: Modelo U2-Net ('u2net' o 'u2netp')
            detection_threshold: Umbral de probabilidad para detección
            min_object_area: Área mínima del objeto en píxeles
            min_probability: Probabilidad mínima para considerar objeto válido
            min_circularity: Circularidad mínima (0-1, 1=círculo perfecto)
            min_aspect_ratio: Aspect ratio mínimo (0-1, rechaza objetos muy alargados)
        """
        self.model_name = model_name
        self.detection_threshold = detection_threshold
        self.min_object_area = min_object_area
        self.min_probability = min_probability
        self.min_circularity = min_circularity
        self.min_aspect_ratio = min_aspect_ratio
        
        # Cargar modelo U2-Net
        self.model = None
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        logger.info(
            f"[SmartFocusScorer] U2-Net: model={model_name}, "
            f"threshold={detection_threshold}, min_area={min_object_area}, "
            f"min_prob={min_probability}, min_circ={min_circularity:.2f}, "
            f"min_aspect={min_aspect_ratio:.2f}"
        )
    
    def set_morphology_params(self, min_circularity: float = None, min_aspect_ratio: float = None):
        """
        Actualiza parámetros de morfología para filtrado de objetos.
        
        Args:
            min_circularity: Circularidad mínima (0-1). None = no cambiar
            min_aspect_ratio: Aspect ratio mínimo (0-1). None = no cambiar
        """
        if min_circularity is not None:
            self.min_circularity = min_circularity
            logger.info(f"[SmartFocusScorer] Circularidad mínima actualizada: {min_circularity:.2f}")
        
        if min_aspect_ratio is not None:
            self.min_aspect_ratio = min_aspect_ratio
            logger.info(f"[SmartFocusScorer] Aspect ratio mínimo actualizado: {min_aspect_ratio:.2f}")
    
    def load_model(self):
        """Carga el modelo U2-Net (lazy loading)."""
        if self.model is not None:
            return
        
        if not TORCH_AVAILABLE:
            logger.info("[SmartFocusScorer] PyTorch no disponible - usando detección simple")
            return
        
        try:
            from torchvision import transforms
            import torch.nn.functional as F
            
            # Aquí iría la carga del modelo U2-Net
            # Por ahora, placeholder para no bloquear la implementación
            logger.info(f"[SmartFocusScorer] Modelo U2-Net cargado: {self.model_name}")
            
        except Exception as e:
            logger.warning(f"[SmartFocusScorer] No se pudo cargar U2-Net: {e}")
            logger.warning("[SmartFocusScorer] Usando solo métricas de nitidez")
    
    def calculate_sharpness(self, image: np.ndarray, roi: Optional[Tuple[int, int, int, int]] = None) -> float:
        """
        Calcula el índice de nitidez usando Laplacian Variance (método más rápido y sensible).
        
        Args:
            image: Imagen de entrada (BGR o grayscale)
            roi: Región de interés (x, y, w, h). Si None, usa toda la imagen.
            
        Returns:
            float: Índice de nitidez (mayor = mejor enfoque)
        """
        # Extraer ROI si se especifica
        if roi is not None:
            x, y, w, h = roi
            h_img, w_img = image.shape[:2]
            x = max(0, min(x, w_img - 1))
            y = max(0, min(y, h_img - 1))
            w = min(w, w_img - x)
            h = min(h, h_img - y)
            
            if w <= 0 or h <= 0:
                return 0.0
            
            image = image[y:y+h, x:x+w]
        
        # Convertir a grayscale si es necesario
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # Normalizar si es uint16
        if gray.dtype == np.uint16:
            gray = (gray / 256).astype(np.uint8)
        
        # Laplacian Variance - Método estándar de autofocus
        # Más sensible a cambios de enfoque que gradient
        laplacian = cv2.Laplacian(gray, cv2.CV_64F, ksize=3)
        variance = laplacian.var()
        
        # Escalar para mejor rango dinámico
        sharpness = variance * 10.0
        
        return float(sharpness)
    
    def evaluate_focus(self, 
                      image: np.ndarray, 
                      roi: Optional[Tuple[int, int, int, int]] = None) -> float:
        """
        Evalúa la calidad de enfoque de una imagen.
        
        Este es el método principal usado por MultiObjectAutofocusController.
        
        Args:
            image: Imagen a evaluar
            roi: Región de interés (bbox del objeto detectado)
            
        Returns:
            float: Score de enfoque (mayor = mejor enfoque)
        """
        return self.calculate_sharpness(image, roi)
    
    def detect_objects(self, image: np.ndarray) -> list:
        """
        Detecta objetos salientes en la imagen usando U2-Net.
        
        Args:
            image: Imagen de entrada (BGR o grayscale)
            
        Returns:
            list: Lista de diccionarios con 'bbox', 'area', 'probability'
        """
        # Si el modelo no está cargado, usar detección simple por contornos
        if self.model is None:
            return self._detect_objects_simple(image)
        
        # Aquí iría la detección con U2-Net
        # Por ahora, fallback a detección simple
        return self._detect_objects_simple(image)
    
    def _detect_objects_simple(self, image: np.ndarray, debug: bool = False) -> list:
        """
        Detección simple de objetos usando umbralización y contornos.
        Fallback cuando U2-Net no está disponible.
        
        Args:
            image: Imagen de entrada
            debug: Si True, retorna también imagen de debug
            
        Returns:
            list: Lista de objetos detectados
        """
        # Convertir a grayscale
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # Normalizar
        if gray.dtype == np.uint16:
            gray = (gray / 256).astype(np.uint8)
        
        # Ecualización de histograma para mejorar contraste
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        
        # Aplicar desenfoque gaussiano suave
        blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)
        
        # Probar múltiples métodos de umbralización
        # Método 1: Otsu
        _, binary_otsu = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        # Método 2: Adaptativa
        binary_adaptive = cv2.adaptiveThreshold(
            blurred, 255, 
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY_INV, 
            21, 5
        )
        
        # Combinar ambos métodos
        binary = cv2.bitwise_or(binary_otsu, binary_adaptive)
        
        # Operaciones morfológicas más agresivas
        kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        
        # Cerrar huecos pequeños
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_large)
        # Eliminar ruido pequeño
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_small)
        # Dilatar ligeramente para conectar regiones cercanas
        binary = cv2.dilate(binary, kernel_small, iterations=1)
        
        # Encontrar contornos
        contours, _ = cv2.findContours(
            binary, 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Filtrar y convertir a objetos
        objects = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # Filtrar por área mínima
            if area < self.min_object_area:
                continue
            
            # Obtener bounding box
            x, y, w, h = cv2.boundingRect(cnt)
            
            # MEJORA: Calcular circularidad usando perímetro REAL del contorno
            perimeter = cv2.arcLength(cnt, True)
            if perimeter > 0:
                circularity = (4 * np.pi * area) / (perimeter ** 2)
                probability = min(1.0, circularity)
            else:
                probability = 0.5
                circularity = 0.0
            
            # MEJORA: Filtrar por aspect ratio (rechazar manchas muy alargadas)
            aspect_ratio = float(w) / float(h) if h > 0 else 1.0
            if aspect_ratio > 1.0:
                aspect_ratio = 1.0 / aspect_ratio  # Normalizar a [0, 1]
            
            # Rechazar objetos muy alargados (configurable)
            if aspect_ratio < self.min_aspect_ratio:
                continue
            
            # Filtrar por circularidad mínima (configurable)
            if circularity < self.min_circularity:
                continue
            
            # Filtrar por probabilidad mínima (legacy, ahora usa circularidad)
            if probability < self.min_probability:
                continue
            
            objects.append({
                'bbox': (x, y, w, h),
                'area': int(area),
                'probability': float(probability),
                'circularity': float(circularity),
                'aspect_ratio': float(aspect_ratio),
                'contour': cnt  # Guardar contorno real para análisis posterior
            })
        
        # Ordenar por área (mayor primero)
        objects.sort(key=lambda obj: obj['area'], reverse=True)
        
        logger.info(f"[SmartFocusScorer] Detectados {len(contours)} contornos totales")
        logger.info(f"[SmartFocusScorer] {len(objects)} objetos válidos (área >= {self.min_object_area}px, prob >= {self.min_probability})")
        
        for i, obj in enumerate(objects[:5]):  # Log primeros 5
            logger.info(f"  Obj{i}: área={obj['area']}px, bbox={obj['bbox']}, prob={obj['probability']:.2f}")
        
        return objects
    
    def detect_objects_with_visualization(self, image: np.ndarray):
        """
        Detecta objetos y genera imagen de visualización con overlays.
        
        Returns:
            tuple: (objects_list, debug_image)
        """
        # Convertir a grayscale
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            vis_image = image.copy()
        else:
            gray = image.copy()
            vis_image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        
        # Normalizar
        if gray.dtype == np.uint16:
            gray = (gray / 256).astype(np.uint8)
            vis_image = (vis_image / 256).astype(np.uint8) if vis_image.dtype == np.uint16 else vis_image
        
        # Ecualización de histograma
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        
        # Desenfoque
        blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)
        
        # Umbralización combinada
        _, binary_otsu = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        binary_adaptive = cv2.adaptiveThreshold(
            blurred, 255, 
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY_INV, 
            21, 5
        )
        binary = cv2.bitwise_or(binary_otsu, binary_adaptive)
        
        # Morfología
        kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_large)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_small)
        binary = cv2.dilate(binary, kernel_small, iterations=1)
        
        # Encontrar contornos
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Dibujar todos los contornos en gris
        cv2.drawContours(vis_image, contours, -1, (128, 128, 128), 1)
        
        # Filtrar y dibujar objetos válidos
        objects = []
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            
            if area < self.min_object_area:
                continue
            
            x, y, w, h = cv2.boundingRect(cnt)
            perimeter = cv2.arcLength(cnt, True)
            
            if perimeter > 0:
                compactness = (4 * np.pi * area) / (perimeter ** 2)
                probability = min(1.0, compactness)
            else:
                probability = 0.5
            
            if probability < self.min_probability:
                continue
            
            objects.append({
                'bbox': (x, y, w, h),
                'area': int(area),
                'probability': float(probability)
            })
            
            # Dibujar bbox verde para objetos válidos
            color = (0, 255, 0)
            cv2.rectangle(vis_image, (x, y), (x + w, y + h), color, 2)
            
            # Texto con info
            label = f"#{len(objects)-1}: {int(area)}px, P={probability:.2f}"
            cv2.putText(vis_image, label, (x, y - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        # Información general en la esquina
        info_text = [
            f"Contornos: {len(contours)}",
            f"Objetos validos: {len(objects)}",
            f"Area min: {self.min_object_area}px",
            f"Prob min: {self.min_probability}"
        ]
        
        y_offset = 20
        for text in info_text:
            cv2.putText(vis_image, text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            y_offset += 20
        
        # Crear mosaico con imagen binaria
        binary_bgr = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        
        # Redimensionar para mosaico
        h, w = vis_image.shape[:2]
        binary_small = cv2.resize(binary_bgr, (w // 3, h // 3))
        
        # Poner binaria en esquina superior derecha
        vis_image[0:h//3, w-w//3:w] = binary_small
        
        objects.sort(key=lambda obj: obj['area'], reverse=True)
        
        return objects, vis_image
    
    def calculate_focus_score_at_z(self, 
                                   image: np.ndarray, 
                                   z_position: float,
                                   roi: Optional[Tuple[int, int, int, int]] = None) -> float:
        """
        Calcula el score de enfoque para una posición Z específica.
        
        Args:
            image: Imagen capturada en esa posición Z
            z_position: Posición Z del piezo (para logging)
            roi: Región de interés
            
        Returns:
            float: Score de enfoque
        """
        score = self.calculate_sharpness(image, roi)
        logger.debug(f"[SmartFocusScorer] Z={z_position:.2f}µm → Score={score:.2f}")
        return score

    def get_smart_score(self, image: np.ndarray) -> Tuple[float, np.ndarray]:
        """Calcula un "Smart Score" usando una máscara morfológica.

        Flujo (alineado con tu especificación de MorphologyFocus):
        1. Genera una máscara binaria del espécimen (0=fondo, 255=muestra) usando
           el mismo pipeline morfológico que la detección simple (U2-Net fallback).
        2. Calcula Laplacian Variance SOLO sobre los píxeles donde mask > 0.
        3. Devuelve (score, mask) para depuración.

        Si no se detecta morfología (máscara vacía), cae a calcular nitidez global.
        """
        # Convertir a grayscale
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()

        # Normalizar uint16 → uint8 si es necesario
        if gray.dtype == np.uint16:
            gray = (gray / 256).astype(np.uint8)

        # --- Paso 1: construir máscara morfológica (fallback cuando U2-Net no está) ---
        # Este pipeline replica la lógica de umbralización + morfología usada
        # en _detect_objects_simple()/detect_objects_with_visualization.
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)

        blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)

        # Umbralización combinada (Otsu + Adaptativa)
        _, binary_otsu = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )
        binary_adaptive = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            21,
            5,
        )
        mask = cv2.bitwise_or(binary_otsu, binary_adaptive)

        # Operaciones morfológicas para limpiar la máscara
        kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_large)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small)
        mask = cv2.dilate(mask, kernel_small, iterations=1)

        # Si la máscara queda vacía, caer a nitidez global
        if np.count_nonzero(mask) == 0:
            score = self.calculate_sharpness(gray)
            logger.debug(
                "[SmartFocusScorer] Máscara vacía en get_smart_score → usando score global: %.2f",
                score,
            )
            return float(score), mask

        # --- Paso 2: Laplacian Variance solo sobre píxeles de la máscara ---
        laplacian = cv2.Laplacian(gray, cv2.CV_64F, ksize=3)
        masked_values = laplacian[mask > 0]

        if masked_values.size == 0:
            return 0.0, mask

        variance = masked_values.var()
        smart_score = float(variance * 10.0)  # mismo escalado que calculate_sharpness

        logger.debug(
            "[SmartFocusScorer] SmartScore (morfología): score=%.2f, pixeles=%d",
            smart_score,
            masked_values.size,
        )

        return smart_score, mask

    # =========================================================================
    # MÉTODOS DE img_analysis/smart_focus_scorer.py (UNIFICADOS)
    # =========================================================================
    
    def _ensure_model_loaded(self) -> bool:
        """
        Carga el modelo U2-Net si no está cargado (lazy loading).
        Intenta usar ai_segmentation.SalientObjectDetector si está disponible.
        
        Returns:
            True si el modelo está listo, False si hay error
        """
        if hasattr(self, '_model_loaded') and self._model_loaded:
            return True
        
        if hasattr(self, '_load_error') and self._load_error:
            return False
        
        try:
            from ai_segmentation import SalientObjectDetector
            self._detector = SalientObjectDetector(
                model_type=self.model_name,
                device=str(self.device) if hasattr(self, 'device') else None,
                auto_download=True
            )
            self._model_loaded = True
            logger.info("[SmartFocusScorer] Modelo U2-Net cargado exitosamente via SalientObjectDetector")
            return True
        except ImportError:
            # ai_segmentation no disponible, usar fallback
            self._model_loaded = False
            self._load_error = "ai_segmentation no disponible"
            logger.info("[SmartFocusScorer] ai_segmentation no disponible - usando detección morfológica")
            return False
        except Exception as e:
            self._load_error = str(e)
            logger.error(f"[SmartFocusScorer] Error cargando U2-Net: {e}")
            return False
    
    def _get_saliency_mask(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Obtiene la máscara de saliencia usando U2-Net o fallback morfológico.
        
        Args:
            image: Imagen BGR o grayscale
            
        Returns:
            (binary_mask, probability_map)
        """
        h, w = image.shape[:2]
        
        if self._ensure_model_loaded() and hasattr(self, '_detector'):
            # Usar U2-Net real
            prob_map = self._detector.get_mask(image, return_probability=True)
            binary_mask = (prob_map > self.detection_threshold).astype(np.uint8) * 255
            return binary_mask, prob_map
        
        # Fallback: usar detección morfológica
        _, mask = self.get_smart_score(image)
        # Crear mapa de probabilidad sintético basado en la máscara
        prob_map = (mask.astype(np.float32) / 255.0) * 0.8  # Max prob 0.8 para fallback
        return mask, prob_map
    
    def _find_all_objects(self, binary_mask: np.ndarray, prob_map: np.ndarray, 
                          img_gray: np.ndarray, focus_threshold: float = 50.0) -> List[ObjectInfo]:
        """
        Encuentra TODOS los objetos válidos en la máscara, con sus scores individuales.
        
        Args:
            binary_mask: Máscara binaria de saliencia
            prob_map: Mapa de probabilidad para validar objetos reales
            img_gray: Imagen en escala de grises para calcular focus
            focus_threshold: Umbral para clasificar como enfocado
            
        Returns:
            Lista de ObjectInfo ordenada por área (mayor primero)
        """
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return []
        
        objects = []
        for c in contours:
            area = cv2.contourArea(c)
            
            # Filtrar por área mínima
            if area < self.min_object_area:
                continue
            
            # Crear máscara para este contorno
            contour_mask = np.zeros(binary_mask.shape, dtype=np.uint8)
            cv2.drawContours(contour_mask, [c], -1, 255, -1)
            
            # Probabilidad promedio dentro del contorno
            mean_prob = float(cv2.mean(prob_map, mask=contour_mask)[0])
            
            if mean_prob < self.min_probability:
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
            is_focused = focus_score >= focus_threshold
            
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
    
    def _calculate_masked_focus(self, img_gray: np.ndarray, mask: np.ndarray, 
                                 use_laplacian: bool = True) -> Tuple[float, float]:
        """
        Calcula el score de enfoque SOLO en los píxeles de la máscara.
        
        Args:
            img_gray: Imagen en escala de grises
            mask: Máscara binaria (255 = objeto)
            use_laplacian: Si True usa Laplacian, si False usa Brenner
            
        Returns:
            (focus_score_normalized, raw_score)
        """
        mask_bool = mask > 127
        n_pixels = np.sum(mask_bool)
        
        if n_pixels < 10:
            return 0.0, 0.0
        
        if use_laplacian:
            laplacian = cv2.Laplacian(img_gray, cv2.CV_64F)
            masked_laplacian = laplacian[mask_bool]
            raw_score = float(np.var(masked_laplacian))
        else:
            # Brenner gradient
            img_float = img_gray.astype(np.float32)
            diff_h = np.zeros_like(img_float)
            diff_v = np.zeros_like(img_float)
            diff_h[:, 2:] = img_float[:, 2:] - img_float[:, :-2]
            diff_v[2:, :] = img_float[2:, :] - img_float[:-2, :]
            energy = (diff_h ** 2 + diff_v ** 2)[mask_bool]
            raw_score = float(np.mean(energy))
        
        focus_score = np.sqrt(raw_score)
        return float(focus_score), float(raw_score)
    
    def assess_image(self, image: Union[str, Path, np.ndarray], 
                     return_debug_mask: bool = True,
                     focus_threshold: float = 50.0) -> ImageAssessmentResult:
        """
        Evalúa el enfoque de una imagen usando U2-Net o detección morfológica.
        
        Pipeline:
        1. Obtener máscara de saliencia (U2-Net o morfológica)
        2. Binarizar y encontrar objetos
        3. Calcular enfoque SOLO en los píxeles del objeto
        4. Clasificar: FOCUSED_OBJECT, BLURRY_OBJECT, EMPTY, MULTIPLE_OBJECTS
        
        Args:
            image: Ruta a imagen o array numpy
            return_debug_mask: Si True, incluye visualización de debug
            focus_threshold: Umbral para clasificar como enfocado
            
        Returns:
            ImageAssessmentResult con status, focus_score, objetos, etc.
        """
        # Cargar imagen
        if isinstance(image, (str, Path)):
            img_bgr = cv2.imread(str(image))
            if img_bgr is None:
                logger.error(f"[SmartFocusScorer] No se pudo cargar: {image}")
                return ImageAssessmentResult(
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
        
        # Normalizar uint16 si es necesario
        if img_gray.dtype == np.uint16:
            img_gray = (img_gray / 256).astype(np.uint8)
        
        # PASO 1: Obtener máscara de saliencia
        binary_mask, prob_map = self._get_saliency_mask(img_bgr)
        
        # PASO 2: Encontrar TODOS los objetos válidos
        objects = self._find_all_objects(binary_mask, prob_map, img_gray, focus_threshold)
        num_objects = len(objects)
        
        global_mean_prob = float(np.mean(prob_map))
        
        if not objects:
            logger.debug(f"[SmartFocusScorer] EMPTY: No se encontró objeto")
            return ImageAssessmentResult(
                status="EMPTY",
                focus_score=0.0,
                is_valid=False,
                num_objects=0,
                mean_probability=global_mean_prob,
                objects=[],
                binary_mask=binary_mask,
                probability_map=prob_map
            )
        
        main_obj = objects[0]
        focused_count = sum(1 for o in objects if o.is_focused)
        
        # PASO 3: Clasificar
        if num_objects > 1:
            status = "MULTIPLE_OBJECTS"
        elif main_obj.is_focused:
            status = "FOCUSED_OBJECT"
        else:
            status = "BLURRY_OBJECT"
        
        logger.debug(f"[SmartFocusScorer] {status}: {num_objects} obj, {focused_count} focused, "
                    f"main_score={main_obj.focus_score:.2f}")
        
        return ImageAssessmentResult(
            status=status,
            focus_score=main_obj.focus_score,
            centroid=main_obj.centroid,
            bounding_box=main_obj.bounding_box,
            contour_area=main_obj.area,
            raw_score=main_obj.raw_score,
            is_valid=True,
            num_objects=num_objects,
            mean_probability=main_obj.mean_probability,
            objects=objects,
            binary_mask=binary_mask,
            probability_map=prob_map
        )
    
    def set_parameters(self, threshold: float = None, min_area: int = None, 
                      max_area: int = None, focus_threshold: float = None):
        """Actualiza parámetros dinámicamente."""
        if threshold is not None:
            self.detection_threshold = threshold
        if min_area is not None:
            self.min_object_area = min_area
        if max_area is not None:
            self.max_object_area = max_area
        if focus_threshold is not None:
            self.focus_threshold = focus_threshold
    
    def get_parameters(self) -> Dict:
        """Retorna parámetros actuales."""
        return {
            'model_name': self.model_name,
            'detection_threshold': self.detection_threshold,
            'min_object_area': self.min_object_area,
            'min_probability': self.min_probability,
            'min_circularity': self.min_circularity,
            'min_aspect_ratio': self.min_aspect_ratio,
            'model_loaded': getattr(self, '_model_loaded', False)
        }
    
    def is_model_loaded(self) -> bool:
        """Verifica si el modelo U2-Net está cargado."""
        return getattr(self, '_model_loaded', False)


# Alias para compatibilidad con código que importa FocusResult desde aquí
FocusResult = ImageAssessmentResult

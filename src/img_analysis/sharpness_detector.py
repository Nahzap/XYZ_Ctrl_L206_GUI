"""
Detector de Nitidez (Sharpness) Robusto con Observador de Histéresis.

Implementación basada en Teoría de Control Robusto (Zhou & Doyle):
1. Rechazo de perturbaciones de iluminación (Bias correction)
2. Cálculo automático de umbral óptimo (H2 minimization / Otsu)
3. Reconstrucción de objeto por histéresis (conectividad espacial)

Teoría:
- Cap. 8 (Incertidumbre): Bias correction rechaza perturbaciones DC
- Cap. 3 (Observadores): Histéresis reconstruye estado por conectividad
- Optimización H2: Otsu minimiza varianza intra-clase
"""

import numpy as np
import cv2
import logging
from typing import Tuple, Optional, Dict, List
from dataclasses import dataclass

from .background_model import load_background_model, validate_background_model

logger = logging.getLogger('MotorControl_L206')

EPSILON = 1e-6


def threshold_multiotsu_manual(image: np.ndarray, classes: int = 3, nbins: int = 64) -> np.ndarray:
    """
    Implementación manual de Multi-Otsu (sin dependencia de skimage).
    
    Encuentra N-1 umbrales que dividen la imagen en N clases minimizando
    la varianza intra-clase total (equivalente a maximizar varianza inter-clase).
    
    Optimizado con nbins=64 para velocidad (suficiente precisión para Z-Score).
    
    Args:
        image: Imagen normalizada [0, 1]
        classes: Número de clases (default 3 → 2 umbrales)
        nbins: Número de bins para histograma (64 es buen balance)
        
    Returns:
        Array de umbrales (classes - 1 valores)
    """
    # Aplanar y filtrar valores válidos
    flat = image.ravel()
    flat = flat[np.isfinite(flat)]
    
    if len(flat) == 0:
        return np.array([0.3, 0.6])  # Fallback
    
    # Histograma normalizado
    hist, bin_edges = np.histogram(flat, bins=nbins, range=(0, 1))
    bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2
    
    # Normalizar histograma a probabilidades
    hist = hist.astype(np.float64)
    hist_sum = hist.sum()
    if hist_sum == 0:
        return np.array([0.3, 0.6])
    prob = hist / hist_sum
    
    # Precálculos para eficiencia
    omega = np.cumsum(prob)
    mu = np.cumsum(prob * bin_centers)
    mu_total = mu[-1] if mu[-1] > 0 else 1.0
    
    # Para 3 clases, buscar 2 umbrales óptimos
    if classes == 3:
        best_variance = -1
        best_t1, best_t2 = nbins // 3, 2 * nbins // 3
        
        # Búsqueda con paso de 1 (nbins=64 es manejable)
        for t1 in range(1, nbins - 2):
            w0 = omega[t1]
            if w0 < EPSILON:
                continue
            mu0 = mu[t1] / w0
            
            for t2 in range(t1 + 1, nbins - 1):
                w1 = omega[t2] - omega[t1]
                w2 = 1 - omega[t2]
                
                if w1 < EPSILON or w2 < EPSILON:
                    continue
                
                mu1 = (mu[t2] - mu[t1]) / w1
                mu2 = (mu_total - mu[t2]) / w2
                
                # Varianza inter-clase
                variance = (w0 * (mu0 - mu_total)**2 + 
                           w1 * (mu1 - mu_total)**2 + 
                           w2 * (mu2 - mu_total)**2)
                
                if variance > best_variance:
                    best_variance = variance
                    best_t1, best_t2 = t1, t2
        
        return np.array([bin_centers[best_t1], bin_centers[best_t2]])
    
    # Para 2 clases, usar Otsu simple
    else:
        best_variance = -1
        best_t = nbins // 2
        
        for t in range(1, nbins - 1):
            w0 = omega[t]
            w1 = 1 - w0
            
            if w0 < EPSILON or w1 < EPSILON:
                continue
            
            mu0 = mu[t] / w0
            mu1 = (mu_total - mu[t]) / w1
            
            variance = w0 * w1 * (mu0 - mu1)**2
            
            if variance > best_variance:
                best_variance = variance
                best_t = t
        
        return np.array([bin_centers[best_t]])


@dataclass
class SharpnessResult:
    """Resultado de análisis de sharpness de una imagen."""
    sharpness: float
    is_focused: bool
    focus_state: str
    object_pixels: int
    object_ratio: float
    z_score_max: float
    z_score_mean_object: float
    bias_correction: float = 0.0
    optimal_z_threshold: float = 0.0
    hysteresis_low: float = 0.0
    z_score_map: Optional[np.ndarray] = None
    mask: Optional[np.ndarray] = None
    laplacian_map: Optional[np.ndarray] = None


class SharpnessDetector:
    """
    Detector de Nitidez Robusto con Observador de Histéresis Automatizado.
    
    Basado en Zhou & Doyle "Essentials of Robust Control":
    - Robustez (Cap. 8): Bias correction para rechazo de perturbaciones DC
    - Observabilidad (Cap. 3): Histéresis como observador de estado espacial
    - Optimización H2: Otsu minimiza varianza intra-clase
    """
    
    def __init__(
        self,
        mu_matrix: Optional[np.ndarray] = None,
        sigma_matrix: Optional[np.ndarray] = None,
        z_threshold: float = 3.0,
        sharpness_threshold: float = 50.0,
        morph_kernel_size: int = 5,
        min_object_ratio: float = 0.001
    ):
        self.mu_matrix = mu_matrix
        self.sigma_matrix = sigma_matrix
        self.z_threshold = z_threshold
        self.sharpness_threshold = sharpness_threshold
        self.morph_kernel_size = morph_kernel_size
        self.min_object_ratio = min_object_ratio
        
        # Modo de operación
        self.use_automatic_threshold = True
        
        # Parámetros de robustez
        self.hysteresis_factor = 0.4
        self.z_floor = 3.0
        self.z_ceiling = 10.0
        
        # Parámetros de sharpness
        self.dilate_mask_for_sharpness = True
        self.dilation_kernel_size = 15
        self.use_mse_sharpness = True
        self.very_focused_multiplier = 5.0
        self.closing_kernel_size = 5
        
        # Legacy (compatibilidad GUI)
        self.morph_open = True
        self.morph_close = True
        
        # Estado interno
        self._model_loaded = False
        self._sigma_safe = None
        
        if mu_matrix is not None and sigma_matrix is not None:
            self._validate_and_prepare_model()
    
    @property
    def is_ready(self) -> bool:
        return self._model_loaded
    
    def _validate_and_prepare_model(self) -> bool:
        is_valid, msg = validate_background_model(self.mu_matrix, self.sigma_matrix)
        if not is_valid:
            logger.error(f"[SharpnessDetector] Modelo inválido: {msg}")
            self._model_loaded = False
            return False
        self._sigma_safe = self.sigma_matrix.astype(np.float64) + EPSILON
        self._model_loaded = True
        logger.info(f"[SharpnessDetector] Modelo robusto preparado: {self.mu_matrix.shape}")
        return True
    
    def load_model(self, models_dir: Optional[str] = None) -> Tuple[bool, str]:
        success, msg, mu, sigma = load_background_model(models_dir)
        if success:
            self.mu_matrix = mu
            self.sigma_matrix = sigma
            self._validate_and_prepare_model()
        return success, msg
    
    def set_model(self, mu_matrix: np.ndarray, sigma_matrix: np.ndarray) -> Tuple[bool, str]:
        self.mu_matrix = mu_matrix
        self.sigma_matrix = sigma_matrix
        if self._validate_and_prepare_model():
            return True, "Modelo establecido correctamente"
        return False, "Error validando modelo"
    
    def set_parameters(
        self,
        z_threshold: Optional[float] = None,
        sharpness_threshold: Optional[float] = None,
        morph_kernel_size: Optional[int] = None,
        min_object_ratio: Optional[float] = None,
        **kwargs
    ):
        if z_threshold is not None:
            self.z_threshold = z_threshold
        if sharpness_threshold is not None:
            self.sharpness_threshold = sharpness_threshold
        if morph_kernel_size is not None:
            self.morph_kernel_size = morph_kernel_size
        if min_object_ratio is not None:
            self.min_object_ratio = min_object_ratio
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
    
    def get_parameters(self) -> Dict:
        return {
            'z_threshold': self.z_threshold,
            'sharpness_threshold': self.sharpness_threshold,
            'morph_kernel_size': self.morph_kernel_size,
            'min_object_ratio': self.min_object_ratio,
            'use_automatic_threshold': self.use_automatic_threshold,
            'hysteresis_factor': self.hysteresis_factor,
            'model_loaded': self._model_loaded,
            'model_shape': self.mu_matrix.shape if self.mu_matrix is not None else None
        }
    
    def compute_robust_z_score(self, img_gray: np.ndarray) -> Tuple[np.ndarray, float]:
        """Z-Score con corrección de BIAS (Rechazo de perturbación DC)."""
        img_float = img_gray.astype(np.float64)
        mu_float = self.mu_matrix.astype(np.float64)
        diff_raw = img_float - mu_float
        bias = np.median(diff_raw)
        diff_corrected = np.abs(diff_raw - bias)
        z_score = diff_corrected / self._sigma_safe
        return z_score, float(bias)
    
    def compute_z_score_map(self, img_gray: np.ndarray) -> np.ndarray:
        """Z-Score simple (compatibilidad)."""
        diff = np.abs(img_gray - self.mu_matrix.astype(np.float64))
        return diff / self._sigma_safe
    
    def compute_automated_mask(self, z_score_map: np.ndarray) -> Tuple[np.ndarray, float, float]:
        """
        Máscara usando detección adaptativa enfocada en OBJETOS REALES.
        
        Principio: Un objeto es una región CONECTADA con Z-Score significativamente
        mayor que el fondo. Usa umbrales RELATIVOS al rango observado.
        
        Estrategia:
        1. Analizar distribución de Z-Score (max, median, std)
        2. Determinar si hay señal significativa
        3. Calcular umbrales adaptativos (Multi-Otsu o percentiles)
        4. Aplicar histéresis + filtro de área
        """
        z_flat = z_score_map.ravel()
        z_max = float(np.max(z_score_map))
        z_min = float(np.min(z_score_map))
        z_median = float(np.median(z_flat))
        z_std = float(np.std(z_flat))
        z_range = z_max - z_min
        
        # === PASO 1: ¿Hay objeto? ===
        # Criterio adaptativo: z_max debe ser significativamente mayor que z_median
        signal_ratio = z_max / max(z_median, 0.01)
        has_significant_signal = (signal_ratio > 2.0) or (z_max > z_median + 1.5 * z_std)
        
        # Si no hay variación significativa, no hay objeto
        if z_range < 0.5 or z_max < 0.5:
            logger.debug(f"[AutoMask] Sin señal: z_range={z_range:.2f}, z_max={z_max:.2f}")
            return np.zeros(z_score_map.shape, dtype=np.uint8), 0.0, 0.0
        
        # === PASO 2: Calcular umbrales adaptativos ===
        # Normalizar al rango observado para Multi-Otsu
        z_normalized = (z_score_map - z_min) / max(z_range, 0.01)
        z_normalized = np.clip(z_normalized, 0, 1)
        
        try:
            thresholds = threshold_multiotsu_manual(z_normalized, classes=3)
            # Convertir de vuelta a escala original
            z_low = z_min + thresholds[0] * z_range
            z_high = z_min + thresholds[1] * z_range
        except:
            # Fallback: usar percentiles
            z_high = np.percentile(z_flat, 90)
            z_low = np.percentile(z_flat, 70)
        
        # Asegurar separación mínima entre umbrales
        if z_high - z_low < 0.1 * z_range:
            z_high = z_median + z_std
            z_low = z_median + 0.5 * z_std
        
        # === PASO 3: Histéresis con conectividad ===
        mask_high = (z_score_map > z_high).astype(np.uint8)
        mask_low = (z_score_map > z_low).astype(np.uint8)
        
        # Encontrar componentes conectados en mask_low
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask_low, connectivity=8)
        
        # Solo mantener componentes que tienen píxeles en mask_high (semillas fuertes)
        high_labels = set(np.unique(labels[mask_high == 1]))
        high_labels.discard(0)  # Quitar fondo
        
        final_mask = np.zeros_like(mask_low)
        min_area = max(int(z_score_map.size * self.min_object_ratio), 10)  # Mínimo 10 píxeles
        
        for label in high_labels:
            area = stats[label, cv2.CC_STAT_AREA]
            if area >= min_area:
                final_mask[labels == label] = 255
        
        # === PASO 4: Fallback si no hay detección pero hay señal ===
        if np.sum(final_mask) == 0 and has_significant_signal:
            # Usar percentil 85 como umbral
            z_adaptive = np.percentile(z_flat, 85)
            mask_adaptive = (z_score_map > z_adaptive).astype(np.uint8)
            
            # Filtrar componentes pequeños
            num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask_adaptive, connectivity=8)
            for label in range(1, num_labels):
                area = stats[label, cv2.CC_STAT_AREA]
                if area >= min_area:
                    final_mask[labels == label] = 255
            
            if np.sum(final_mask) > 0:
                z_high = z_adaptive
                z_low = np.percentile(z_flat, 70)
        
        # === PASO 5: Limpieza morfológica ===
        if np.sum(final_mask) > 0:
            # Opening para quitar ruido
            kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_OPEN, kernel_small)
            
            # Closing para consolidar
            if self.closing_kernel_size > 0:
                kernel = cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE,
                    (self.closing_kernel_size, self.closing_kernel_size)
                )
                final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_CLOSE, kernel)
        
        logger.debug(f"[AutoMask] z_range={z_range:.2f}, z_high={z_high:.2f}, z_low={z_low:.2f}, obj_pixels={np.sum(final_mask > 0)}")
        return final_mask, z_high, z_low
    
    def compute_object_mask(self, z_score_map: np.ndarray, apply_morphology: bool = True) -> np.ndarray:
        """Máscara binaria (automática o legacy)."""
        if self.use_automatic_threshold:
            mask, _, _ = self.compute_automated_mask(z_score_map)
            return mask
        
        mask = (z_score_map > self.z_threshold).astype(np.uint8) * 255
        if apply_morphology and self.morph_kernel_size > 0:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (self.morph_kernel_size, self.morph_kernel_size)
            )
            if self.morph_open:
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            if self.morph_close:
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask
    
    def compute_sharpness(self, img_gray: np.ndarray, mask: np.ndarray) -> Tuple[float, np.ndarray]:
        """
        Energía de alta frecuencia (Sharpness) SOLO en la región del objeto.
        
        Optimización: Calcula Laplaciano solo en el bounding box del objeto
        para mayor velocidad en imágenes grandes.
        """
        if np.sum(mask) == 0:
            return 0.0, np.zeros_like(img_gray, dtype=np.float64)
        
        # Encontrar bounding box del objeto para optimizar cálculo
        coords = np.where(mask > 0)
        if len(coords[0]) == 0:
            return 0.0, np.zeros_like(img_gray, dtype=np.float64)
        
        y_min, y_max = coords[0].min(), coords[0].max()
        x_min, x_max = coords[1].min(), coords[1].max()
        
        # Expandir bounding box con margen para dilatación
        margin = self.dilation_kernel_size if self.dilate_mask_for_sharpness else 5
        y_min = max(0, y_min - margin)
        y_max = min(img_gray.shape[0], y_max + margin)
        x_min = max(0, x_min - margin)
        x_max = min(img_gray.shape[1], x_max + margin)
        
        # Extraer ROI
        img_roi = img_gray[y_min:y_max, x_min:x_max].astype(np.float64)
        mask_roi = mask[y_min:y_max, x_min:x_max]
        
        # Dilatar máscara si está habilitado
        if self.dilate_mask_for_sharpness and self.dilation_kernel_size > 0:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (self.dilation_kernel_size, self.dilation_kernel_size)
            )
            mask_roi = cv2.dilate(mask_roi, kernel, iterations=1)
        
        # Calcular Laplaciano solo en ROI (más rápido)
        laplacian_roi = cv2.Laplacian(img_roi, cv2.CV_64F)
        
        # Aplicar máscara
        mask_bool = mask_roi > 0
        laplacian_masked = laplacian_roi[mask_bool]
        
        if laplacian_masked.size == 0:
            return 0.0, np.zeros_like(img_gray, dtype=np.float64)
        
        # Calcular sharpness
        if self.use_mse_sharpness:
            mse = np.mean(laplacian_masked ** 2)
            sharpness = np.sqrt(mse)
        else:
            sharpness = np.mean(np.abs(laplacian_masked))
        
        # Crear mapa completo de Laplaciano para visualización (opcional)
        laplacian_full = np.zeros_like(img_gray, dtype=np.float64)
        laplacian_full[y_min:y_max, x_min:x_max] = np.abs(laplacian_roi)
        
        return float(sharpness), laplacian_full
    
    def classify_sharpness(self, sharpness: float, has_object: bool) -> Tuple[bool, str]:
        if not has_object:
            return False, "Sin Objeto"
        if sharpness > self.sharpness_threshold * self.very_focused_multiplier:
            return True, "Muy Enfocada"
        elif sharpness > self.sharpness_threshold:
            return True, "Enfocada"
        else:
            return False, "Desenfocada"
    
    def analyze_image(self, image_path: str, return_maps: bool = False) -> Optional[SharpnessResult]:
        if not self._model_loaded:
            logger.error("[SharpnessDetector] Modelo no cargado")
            return None
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            logger.error(f"[SharpnessDetector] No se pudo cargar: {image_path}")
            return None
        return self.analyze_array(img, return_maps)
    
    def analyze_array(self, img_gray: np.ndarray, return_maps: bool = False) -> Optional[SharpnessResult]:
        if not self._model_loaded:
            logger.error("[SharpnessDetector] Modelo no cargado")
            return None
        
        if img_gray.shape != self.mu_matrix.shape:
            logger.error(f"[SharpnessDetector] Dimensiones no coinciden: {img_gray.shape} vs {self.mu_matrix.shape}")
            return None
        
        z_score_map, bias = self.compute_robust_z_score(img_gray)
        
        if self.use_automatic_threshold:
            mask, z_high, z_low = self.compute_automated_mask(z_score_map)
        else:
            mask = self.compute_object_mask(z_score_map)
            z_high = self.z_threshold
            z_low = 0.0
        
        total_pixels = mask.size
        object_pixels = int(np.sum(mask > 0))
        object_ratio = object_pixels / total_pixels
        has_object = object_ratio > self.min_object_ratio
        
        sharpness = 0.0
        laplacian_map = None
        z_mean_object = 0.0
        
        if has_object:
            sharpness, laplacian_map = self.compute_sharpness(img_gray, mask)
            z_mean_object = float(np.mean(z_score_map[mask > 0]))
        
        is_focused, focus_state = self.classify_sharpness(sharpness, has_object)
        
        return SharpnessResult(
            sharpness=sharpness,
            is_focused=is_focused,
            focus_state=focus_state,
            object_pixels=object_pixels,
            object_ratio=object_ratio,
            z_score_max=float(np.max(z_score_map)),
            z_score_mean_object=z_mean_object,
            bias_correction=bias,
            optimal_z_threshold=z_high,
            hysteresis_low=z_low,
            z_score_map=z_score_map.astype(np.float32) if return_maps else None,
            mask=mask if return_maps else None,
            laplacian_map=laplacian_map.astype(np.float32) if return_maps and laplacian_map is not None else None
        )


def create_debug_composite(
    original: np.ndarray,
    z_score_map: np.ndarray,
    mask: np.ndarray,
    sharpness: float,
    optimal_threshold: float = 0.0,
    bias: float = 0.0,
    hysteresis_low: float = 0.0
) -> np.ndarray:
    """
    Imagen compuesta para debug: [Original | Z-Score | Máscara]
    
    Muestra información de Multi-Otsu:
    - Z_hi: Umbral alto (clase objeto)
    - Z_lo: Umbral bajo (histéresis)
    """
    h, w = original.shape[:2]
    
    if len(original.shape) == 2:
        panel1 = cv2.cvtColor(original, cv2.COLOR_GRAY2BGR)
    else:
        panel1 = original.copy()
    
    # Panel 1: Original con S y Bias
    cv2.putText(panel1, f"S={sharpness:.1f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(panel1, f"Bias={bias:.1f}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
    
    # Panel 2: Z-Score heatmap con umbrales Multi-Otsu
    z_norm = np.clip(z_score_map / 10.0, 0, 1)
    z_uint8 = (z_norm * 255).astype(np.uint8)
    panel2 = cv2.applyColorMap(z_uint8, cv2.COLORMAP_JET)
    cv2.putText(panel2, f"Z_hi={optimal_threshold:.2f}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(panel2, f"Z_lo={hysteresis_low:.2f}", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    
    # Panel 3: Máscara con porcentaje
    panel3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    obj_pct = np.sum(mask > 0) / mask.size * 100
    cv2.putText(panel3, f"Obj={obj_pct:.2f}%", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    return np.hstack([panel1, panel2, panel3])

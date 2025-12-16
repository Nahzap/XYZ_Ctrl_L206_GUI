"""
Métricas de Imagen - Funciones Compartidas
==========================================

Funciones de procesamiento de imagen y cálculo de métricas
reutilizables en todo el proyecto.

Centraliza algoritmos que antes estaban duplicados en:
- core/autofocus/smart_focus_scorer.py
- img_analysis/smart_focus_scorer.py
- img_analysis/sharpness_detector.py
- core/services/autofocus_service.py
"""

import numpy as np
import cv2
from typing import Optional, Tuple

# Constante para evitar división por cero
EPSILON = 1e-10


def normalize_image(image: np.ndarray) -> np.ndarray:
    """
    Normaliza una imagen a uint8.
    
    Maneja imágenes uint16 (cámaras científicas) y las convierte a uint8.
    
    Args:
        image: Imagen de entrada (uint8, uint16, o float)
        
    Returns:
        Imagen normalizada como uint8
    """
    if image.dtype == np.uint16:
        # Normalizar uint16 a uint8
        return (image / 256).astype(np.uint8)
    elif image.dtype == np.float32 or image.dtype == np.float64:
        # Asumir rango [0, 1] para float
        return (image * 255).astype(np.uint8)
    else:
        return image.astype(np.uint8)


def to_grayscale(image: np.ndarray) -> np.ndarray:
    """
    Convierte imagen a escala de grises si es necesario.
    
    Args:
        image: Imagen BGR o grayscale
        
    Returns:
        Imagen en escala de grises
    """
    if len(image.shape) == 3:
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return image.copy()


def calculate_laplacian_variance(image: np.ndarray, 
                                  mask: Optional[np.ndarray] = None,
                                  ksize: int = 3,
                                  scale: float = 1.0) -> float:
    """
    Calcula la varianza del Laplaciano como métrica de nitidez.
    
    El Laplaciano detecta bordes/cambios de intensidad. Una imagen
    enfocada tiene alta varianza (muchos bordes definidos), mientras
    que una desenfocada tiene baja varianza.
    
    Args:
        image: Imagen en escala de grises (uint8)
        mask: Máscara opcional (255 = incluir, 0 = excluir)
        ksize: Tamaño del kernel Laplaciano (1, 3, 5, 7)
        scale: Factor de escala para el resultado
        
    Returns:
        Varianza del Laplaciano (mayor = más enfocado)
    """
    # Asegurar grayscale
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    
    # Normalizar si es necesario
    if gray.dtype == np.uint16:
        gray = (gray / 256).astype(np.uint8)
    
    # Calcular Laplaciano
    laplacian = cv2.Laplacian(gray, cv2.CV_64F, ksize=ksize)
    
    # Aplicar máscara si existe
    if mask is not None:
        mask_bool = mask > 127
        if np.sum(mask_bool) < 10:
            return 0.0
        masked_values = laplacian[mask_bool]
        variance = float(np.var(masked_values))
    else:
        variance = float(laplacian.var())
    
    return variance * scale


def calculate_brenner_gradient(image: np.ndarray,
                                mask: Optional[np.ndarray] = None) -> float:
    """
    Calcula el gradiente de Brenner como métrica de nitidez alternativa.
    
    El método de Brenner usa diferencias de segundo orden para
    detectar cambios de intensidad. Es más robusto al ruido que
    el Laplaciano simple.
    
    Args:
        image: Imagen en escala de grises
        mask: Máscara opcional
        
    Returns:
        Energía del gradiente de Brenner (mayor = más enfocado)
    """
    # Asegurar grayscale
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    
    # Normalizar
    if gray.dtype == np.uint16:
        gray = (gray / 256).astype(np.uint8)
    
    img_float = gray.astype(np.float32)
    
    # Diferencias de segundo orden
    diff_h = np.zeros_like(img_float)
    diff_v = np.zeros_like(img_float)
    diff_h[:, 2:] = img_float[:, 2:] - img_float[:, :-2]
    diff_v[2:, :] = img_float[2:, :] - img_float[:-2, :]
    
    # Energía
    energy = diff_h ** 2 + diff_v ** 2
    
    if mask is not None:
        mask_bool = mask > 127
        if np.sum(mask_bool) < 10:
            return 0.0
        return float(np.mean(energy[mask_bool]))
    
    return float(np.mean(energy))


def preprocess_for_detection(image: np.ndarray,
                              clip_limit: float = 2.0,
                              tile_size: int = 8,
                              blur_size: int = 5) -> np.ndarray:
    """
    Preprocesamiento estándar para detección de objetos.
    
    Pipeline:
    1. Convertir a grayscale
    2. Normalizar a uint8
    3. CLAHE (Contrast Limited Adaptive Histogram Equalization)
    4. Gaussian blur suave
    
    Args:
        image: Imagen de entrada
        clip_limit: Límite de contraste para CLAHE
        tile_size: Tamaño de tiles para CLAHE
        blur_size: Tamaño del kernel Gaussian (debe ser impar)
        
    Returns:
        Imagen preprocesada (grayscale, uint8)
    """
    # Convertir a grayscale
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image.copy()
    
    # Normalizar
    if gray.dtype == np.uint16:
        gray = (gray / 256).astype(np.uint8)
    
    # CLAHE
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(tile_size, tile_size))
    enhanced = clahe.apply(gray)
    
    # Gaussian blur
    blurred = cv2.GaussianBlur(enhanced, (blur_size, blur_size), 0)
    
    return blurred


def create_binary_mask(image: np.ndarray,
                       method: str = 'combined') -> np.ndarray:
    """
    Crea una máscara binaria para segmentación de objetos.
    
    Args:
        image: Imagen preprocesada (grayscale, uint8)
        method: 'otsu', 'adaptive', o 'combined'
        
    Returns:
        Máscara binaria (255 = objeto, 0 = fondo)
    """
    if method == 'otsu':
        _, binary = cv2.threshold(image, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        return binary
    
    elif method == 'adaptive':
        return cv2.adaptiveThreshold(
            image, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            21, 5
        )
    
    else:  # combined
        _, binary_otsu = cv2.threshold(image, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        binary_adaptive = cv2.adaptiveThreshold(
            image, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            21, 5
        )
        return cv2.bitwise_or(binary_otsu, binary_adaptive)


def clean_binary_mask(mask: np.ndarray,
                      close_size: int = 7,
                      open_size: int = 3,
                      dilate_iterations: int = 1) -> np.ndarray:
    """
    Limpia una máscara binaria con operaciones morfológicas.
    
    Args:
        mask: Máscara binaria de entrada
        close_size: Tamaño del kernel para cerrar huecos
        open_size: Tamaño del kernel para eliminar ruido
        dilate_iterations: Iteraciones de dilatación
        
    Returns:
        Máscara limpia
    """
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_size, close_size))
    kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_size, open_size))
    
    # Cerrar huecos
    result = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
    # Eliminar ruido
    result = cv2.morphologyEx(result, cv2.MORPH_OPEN, kernel_open)
    # Dilatar ligeramente
    if dilate_iterations > 0:
        result = cv2.dilate(result, kernel_open, iterations=dilate_iterations)
    
    return result


def calculate_circularity(contour: np.ndarray) -> float:
    """
    Calcula la circularidad de un contorno.
    
    Circularidad = 4π × Area / Perímetro²
    - 1.0 = círculo perfecto
    - < 1.0 = formas más irregulares
    
    Args:
        contour: Contorno OpenCV
        
    Returns:
        Circularidad (0-1)
    """
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    
    if perimeter < EPSILON:
        return 0.0
    
    circularity = (4 * np.pi * area) / (perimeter ** 2)
    return min(1.0, circularity)


def calculate_aspect_ratio(contour: np.ndarray) -> float:
    """
    Calcula el aspect ratio normalizado de un contorno.
    
    Args:
        contour: Contorno OpenCV
        
    Returns:
        Aspect ratio normalizado (0-1, donde 1 = cuadrado)
    """
    x, y, w, h = cv2.boundingRect(contour)
    
    if h < 1 or w < 1:
        return 0.0
    
    ratio = float(w) / float(h)
    if ratio > 1.0:
        ratio = 1.0 / ratio
    
    return ratio

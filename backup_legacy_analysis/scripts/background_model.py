"""
Modelo de Fondo para Análisis de Imagen.

Calibración estadística del fondo (μ, σ) para separar objeto de ruido.
Basado en Teoría de Control Robusto - Identificación del Sistema.

Uso:
    1. Recolectar 100-300 imágenes de fondo (sin objeto)
    2. Llamar train_background_model(folder) para generar μ y σ
    3. Los modelos se guardan en models/sharpness_mu.npy y sharpness_sigma.npy
"""

import os
import json
import numpy as np
import cv2
import logging
from typing import Tuple, Optional, List

logger = logging.getLogger('MotorControl_L206')

# Directorio por defecto para modelos de sharpness
DEFAULT_SHARPNESS_MODELS_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..', 'models', 'sharpness'
)


def get_supported_image_extensions() -> Tuple[str, ...]:
    """Retorna extensiones de imagen soportadas."""
    return ('.png', '.jpg', '.jpeg', '.bmp', '.tiff', '.tif')


def list_images_in_folder(folder_path: str) -> List[str]:
    """
    Lista archivos de imagen en una carpeta.
    
    Args:
        folder_path: Ruta a la carpeta
        
    Returns:
        Lista de rutas completas a imágenes (ordenadas)
    """
    extensions = get_supported_image_extensions()
    image_files = []
    
    try:
        for filename in sorted(os.listdir(folder_path)):
            if filename.lower().endswith(extensions):
                filepath = os.path.join(folder_path, filename)
                image_files.append(filepath)
    except Exception as e:
        logger.error(f"Error listando imágenes de {folder_path}: {e}")
    
    return image_files


def generate_contrast_variations(img: np.ndarray, mode: str = "light") -> List[np.ndarray]:
    """
    Genera variaciones de contraste/brillo de una imagen para calibración robusta.
    
    IMPORTANTE: Las variaciones son SUTILES para no inflar σ excesivamente.
    El objetivo es capturar variabilidad natural del fondo, no simular
    cambios extremos de iluminación (eso se maneja con bias correction).
    
    Modos:
    - "light": Variaciones sutiles (±5 brillo, ±5% contraste) - Recomendado
    - "medium": Variaciones moderadas (±12 brillo, ±10% contraste)
    - "heavy": Variaciones fuertes (±25 brillo, ±20% contraste) - No recomendado
    
    Args:
        img: Imagen en escala de grises (float64)
        mode: Intensidad de variaciones ("light", "medium", "heavy")
        
    Returns:
        Lista de imágenes con variaciones de contraste
    """
    variations = [img]  # Original siempre incluido
    
    img_mean = img.mean()
    
    # Configuración según modo
    if mode == "light":
        brightness_offsets = [-5, 5]
        contrast_factors = [0.95, 1.05]
    elif mode == "medium":
        brightness_offsets = [-12, -6, 6, 12]
        contrast_factors = [0.9, 0.95, 1.05, 1.1]
    else:  # heavy
        brightness_offsets = [-25, -12, 12, 25]
        contrast_factors = [0.8, 0.9, 1.1, 1.2]
    
    # Variaciones de brillo (offset aditivo)
    for offset in brightness_offsets:
        varied = np.clip(img + offset, 0, 255)
        variations.append(varied)
    
    # Variaciones de contraste (escalado desde la media)
    for factor in contrast_factors:
        varied = img_mean + (img - img_mean) * factor
        varied = np.clip(varied, 0, 255)
        variations.append(varied)
    
    return variations


def train_background_model(
    background_folder: str,
    output_dir: Optional[str] = None,
    progress_callback: Optional[callable] = None,
    simulate_contrast: bool = False  # Deshabilitado por defecto - infla σ demasiado
) -> Tuple[bool, str, Optional[np.ndarray], Optional[np.ndarray]]:
    """
    Entrena el modelo estadístico del fondo para análisis de sharpness.
    
    Calcula μ (media) y σ (desviación estándar) píxel a píxel del conjunto
    de imágenes de fondo. Esto representa la "incertidumbre" del sistema.
    
    Con simulate_contrast=True, genera variaciones de brillo/contraste de cada
    imagen para hacer el modelo robusto a diferentes condiciones de iluminación.
    
    Args:
        background_folder: Carpeta con imágenes de fondo (100-300 recomendadas)
        output_dir: Directorio para guardar modelos (default: models/sharpness/)
        progress_callback: Función callback(current, total, message) para progreso
        simulate_contrast: Si True, simula variaciones de contraste (recomendado)
        
    Returns:
        Tuple: (success, message, mu_matrix, sigma_matrix)
    """
    if output_dir is None:
        output_dir = DEFAULT_SHARPNESS_MODELS_DIR
    
    # Crear directorio de modelos si no existe
    os.makedirs(output_dir, exist_ok=True)
    
    # Obtener lista de imágenes
    image_files = list_images_in_folder(background_folder)
    
    if len(image_files) < 10:
        return False, f"Se requieren al menos 10 imágenes de fondo. Encontradas: {len(image_files)}", None, None
    
    # Calcular total de muestras (con variaciones)
    # Modo "light": 1 original + 2 brillo + 2 contraste = 5 variaciones
    n_variations = 5 if simulate_contrast else 1
    total_samples = len(image_files) * n_variations
    
    logger.info(f"[SharpnessModel] Iniciando calibración con {len(image_files)} imágenes de fondo")
    if simulate_contrast:
        logger.info(f"[SharpnessModel] Simulando {n_variations} variaciones sutiles por imagen ({total_samples} muestras totales)")
    
    if progress_callback:
        progress_callback(0, len(image_files), "Cargando imágenes...")
    
    # Cargar primera imagen para obtener dimensiones
    first_img = cv2.imread(image_files[0], cv2.IMREAD_GRAYSCALE)
    if first_img is None:
        return False, f"Error cargando imagen: {image_files[0]}", None, None
    
    height, width = first_img.shape
    
    # Acumuladores para cálculo incremental de media y varianza
    # Usando algoritmo de Welford para estabilidad numérica
    n = 0
    mean = np.zeros((height, width), dtype=np.float64)
    M2 = np.zeros((height, width), dtype=np.float64)  # Suma de cuadrados de diferencias
    
    # Procesar cada imagen
    for i, filepath in enumerate(image_files):
        img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
        
        if img is None:
            logger.warning(f"[SharpnessModel] No se pudo cargar: {filepath}")
            continue
        
        # Verificar dimensiones
        if img.shape != (height, width):
            logger.warning(f"[SharpnessModel] Dimensiones incorrectas en {filepath}")
            continue
        
        # Convertir a float64 para precisión
        img_float = img.astype(np.float64)
        
        # Generar variaciones de contraste si está habilitado
        if simulate_contrast:
            variations = generate_contrast_variations(img_float)
        else:
            variations = [img_float]
        
        # Procesar cada variación con Welford
        for img_var in variations:
            n += 1
            delta = img_var - mean
            mean += delta / n
            delta2 = img_var - mean
            M2 += delta * delta2
        
        if progress_callback:
            progress_callback(i + 1, len(image_files), f"Procesando {os.path.basename(filepath)}")
    
    if n < 10:
        return False, f"Solo se procesaron {n} muestras válidas. Se requieren al menos 10.", None, None
    
    # Calcular desviación estándar
    variance = M2 / n
    sigma = np.sqrt(variance)
    
    # Aplicar un piso mínimo a sigma para evitar divisiones por cero
    # Valor bajo para mantener sensibilidad a objetos de bajo contraste
    sigma_floor = 1.0  # Mínimo 1 nivel de gris
    sigma = np.maximum(sigma, sigma_floor)
    
    # Convertir a float32 para almacenamiento eficiente
    mu_matrix = mean.astype(np.float32)
    sigma_matrix = sigma.astype(np.float32)
    
    # Guardar modelos con nombres descriptivos
    mu_path = os.path.join(output_dir, 'sharpness_mu.npy')
    sigma_path = os.path.join(output_dir, 'sharpness_sigma.npy')
    
    try:
        np.save(mu_path, mu_matrix)
        np.save(sigma_path, sigma_matrix)
        logger.info(f"[SharpnessModel] Modelos guardados en {output_dir}")
    except Exception as e:
        return False, f"Error guardando modelos: {e}", mu_matrix, sigma_matrix
    
    # Estadísticas del modelo
    n_original = len(image_files)
    stats = {
        'n_images_original': n_original,
        'n_samples_total': n,
        'contrast_simulation': simulate_contrast,
        'shape': (height, width),
        'mu_range': (float(mu_matrix.min()), float(mu_matrix.max())),
        'sigma_range': (float(sigma_matrix.min()), float(sigma_matrix.max())),
        'sigma_mean': float(sigma_matrix.mean())
    }
    
    logger.info(f"[SharpnessModel] Calibración completada: {stats}")
    
    if simulate_contrast:
        message = (f"Modelo robusto entrenado:\n"
                   f"• {n_original} imágenes × {n_variations} variaciones = {n} muestras\n"
                   f"• Dimensiones: {width}×{height}\n"
                   f"• σ promedio: {stats['sigma_mean']:.2f}\n"
                   f"• Simulación de contraste: ✓ Activada")
    else:
        message = (f"Modelo entrenado con {n} imágenes.\n"
                   f"Dimensiones: {width}×{height}\n"
                   f"σ promedio: {stats['sigma_mean']:.2f}")
    
    return True, message, mu_matrix, sigma_matrix


def load_background_model(
    models_dir: Optional[str] = None
) -> Tuple[bool, str, Optional[np.ndarray], Optional[np.ndarray]]:
    """
    Carga el modelo de fondo previamente entrenado.
    
    Args:
        models_dir: Directorio con los modelos (default: models/sharpness/)
        
    Returns:
        Tuple: (success, message, mu_matrix, sigma_matrix)
    """
    if models_dir is None:
        models_dir = DEFAULT_SHARPNESS_MODELS_DIR
    
    mu_path = os.path.join(models_dir, 'sharpness_mu.npy')
    sigma_path = os.path.join(models_dir, 'sharpness_sigma.npy')
    
    if not os.path.exists(mu_path) or not os.path.exists(sigma_path):
        return False, "Modelo de sharpness no encontrado. Ejecute calibración primero.", None, None
    
    try:
        mu_matrix = np.load(mu_path)
        sigma_matrix = np.load(sigma_path)
        
        logger.info(f"[SharpnessModel] Modelo cargado: shape={mu_matrix.shape}, σ_mean={sigma_matrix.mean():.2f}")
        
        return True, f"Modelo cargado: {mu_matrix.shape[1]}x{mu_matrix.shape[0]}", mu_matrix, sigma_matrix
        
    except Exception as e:
        logger.error(f"[SharpnessModel] Error cargando modelo: {e}")
        return False, f"Error cargando modelo: {e}", None, None


def validate_background_model(
    mu_matrix: np.ndarray, 
    sigma_matrix: np.ndarray
) -> Tuple[bool, str]:
    """
    Valida que el modelo de fondo sea consistente.
    
    Args:
        mu_matrix: Matriz de medias
        sigma_matrix: Matriz de desviaciones estándar
        
    Returns:
        Tuple: (is_valid, message)
    """
    if mu_matrix is None or sigma_matrix is None:
        return False, "Modelo no inicializado"
    
    if mu_matrix.shape != sigma_matrix.shape:
        return False, f"Dimensiones inconsistentes: μ={mu_matrix.shape}, σ={sigma_matrix.shape}"
    
    if len(mu_matrix.shape) != 2:
        return False, f"Se esperan matrices 2D, recibido: {len(mu_matrix.shape)}D"
    
    # Verificar valores razonables
    if sigma_matrix.min() < 0:
        return False, "σ contiene valores negativos (inválido)"
    
    # Advertencia si sigma es muy bajo
    zero_sigma_count = np.sum(sigma_matrix < 1e-6)
    if zero_sigma_count > 0:
        logger.warning(f"[SharpnessModel] {zero_sigma_count} píxeles con σ≈0")
    
    return True, f"Modelo válido: {mu_matrix.shape[1]}x{mu_matrix.shape[0]}"


class BackgroundModelCache:
    """Cache persistente para resultados de sharpness por carpeta."""
    
    def __init__(self, folder: str):
        """
        Inicializa el cache para una carpeta de imágenes.
        
        Args:
            folder: Ruta a la carpeta de imágenes
        """
        self.folder = folder
        self.cache_file = os.path.join(folder, ".sharpness_cache.json")
        self.results = self._load()
    
    def _load(self) -> dict:
        """Carga resultados desde archivo JSON."""
        if os.path.exists(self.cache_file):
            try:
                with open(self.cache_file, 'r') as f:
                    return json.load(f)
            except Exception as e:
                logger.warning(f"[SharpnessCache] Error cargando cache: {e}")
                return {}
        return {}
    
    def save(self):
        """Guarda resultados a archivo JSON."""
        try:
            with open(self.cache_file, 'w') as f:
                json.dump(self.results, f, indent=2)
        except Exception as e:
            logger.error(f"[SharpnessCache] Error guardando cache: {e}")
    
    def get(self, filename: str) -> Optional[dict]:
        """Obtiene resultado del cache."""
        return self.results.get(filename)
    
    def set(self, filename: str, result: dict):
        """Guarda resultado en cache."""
        self.results[filename] = result
        self.save()
    
    def has(self, filename: str) -> bool:
        """Verifica si existe resultado en cache."""
        return filename in self.results
    
    def clear(self):
        """Limpia el cache."""
        self.results = {}
        self.save()

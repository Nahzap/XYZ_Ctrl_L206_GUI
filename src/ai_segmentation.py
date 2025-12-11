"""
AI Segmentation - Salient Object Detection usando U2-Net.

Wrapper para usar U2-Net en detección de objetos salientes (polen, células, etc.)
sin necesidad de calibración previa.
"""

import os
import logging
import numpy as np
import cv2
from pathlib import Path
from typing import Optional, Tuple, Union

import torch
import torch.nn.functional as F

from models.u2net import U2NET, U2NETP

logger = logging.getLogger('MotorControl_L206')

# Rutas de pesos
WEIGHTS_DIR = Path(__file__).parent.parent / "models" / "weights"
U2NET_WEIGHTS = WEIGHTS_DIR / "u2net.pth"
U2NETP_WEIGHTS = WEIGHTS_DIR / "u2netp.pth"

# URLs oficiales de descarga (Google Drive)
WEIGHTS_URLS = {
    'u2net': 'https://drive.google.com/uc?id=1ao1ovG1Qtx4b7EoskHXmi2E9rp5CHLcZ',
    'u2netp': 'https://drive.google.com/uc?id=1rbSTGKAE-MTxBYHd-51l2hMOQPT_7EPy'
}


class SalientObjectDetector:
    """
    Detector de Objetos Salientes usando U2-Net.
    
    Permite aislar objetos (polen, células) del fondo sucio usando
    deep learning, sin necesidad de calibración previa.
    
    Attributes:
        model_type: 'u2netp' (rápido, ~4MB) o 'u2net' (preciso, ~176MB)
        device: 'cuda' o 'cpu'
        input_size: Tamaño de entrada del modelo (default 320x320)
    """
    
    def __init__(
        self,
        model_type: str = 'u2netp',
        device: Optional[str] = None,
        input_size: int = 320,
        auto_download: bool = True
    ):
        """
        Inicializa el detector.
        
        Args:
            model_type: 'u2netp' (pequeño/rápido) o 'u2net' (full/preciso)
            device: 'cuda', 'cpu' o None (auto-detecta)
            input_size: Tamaño de entrada (320 recomendado)
            auto_download: Si True, descarga pesos automáticamente si no existen
        """
        self.model_type = model_type
        self.input_size = input_size
        self.auto_download = auto_download
        
        # Detectar dispositivo
        if device is None:
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        else:
            self.device = torch.device(device)
        
        logger.info(f"[SalientObjectDetector] Usando dispositivo: {self.device}")
        
        # Cargar modelo
        self.model = None
        self._load_model()
    
    def _get_weights_path(self) -> Path:
        """Retorna la ruta del archivo de pesos según el tipo de modelo."""
        if self.model_type == 'u2net':
            return U2NET_WEIGHTS
        return U2NETP_WEIGHTS
    
    def _download_weights(self) -> bool:
        """
        Descarga los pesos del modelo si no existen.
        
        Returns:
            True si descarga exitosa o ya existen, False si falla
        """
        weights_path = self._get_weights_path()
        
        if weights_path.exists():
            return True
        
        # Crear directorio si no existe
        WEIGHTS_DIR.mkdir(parents=True, exist_ok=True)
        
        url = WEIGHTS_URLS.get(self.model_type)
        if not url:
            logger.error(f"[SalientObjectDetector] URL no encontrada para {self.model_type}")
            return False
        
        logger.info(f"[SalientObjectDetector] Descargando pesos {self.model_type}...")
        
        try:
            import gdown
            gdown.download(url, str(weights_path), quiet=False)
            logger.info(f"[SalientObjectDetector] Pesos descargados: {weights_path}")
            return True
        except ImportError:
            logger.error("[SalientObjectDetector] gdown no instalado. Ejecuta: pip install gdown")
            logger.error(f"[SalientObjectDetector] O descarga manualmente desde: {url}")
            return False
        except Exception as e:
            logger.error(f"[SalientObjectDetector] Error descargando pesos: {e}")
            return False
    
    def _load_model(self):
        """Carga el modelo U2-Net."""
        weights_path = self._get_weights_path()
        
        # Verificar/descargar pesos
        if not weights_path.exists():
            if self.auto_download:
                if not self._download_weights():
                    raise FileNotFoundError(
                        f"No se encontraron pesos en {weights_path}. "
                        f"Ejecuta setup_ai.py o descarga manualmente."
                    )
            else:
                raise FileNotFoundError(
                    f"Pesos no encontrados: {weights_path}. "
                    f"Ejecuta setup_ai.py para descargarlos."
                )
        
        # Crear modelo
        if self.model_type == 'u2net':
            self.model = U2NET(3, 1)
        else:
            self.model = U2NETP(3, 1)
        
        # Cargar pesos
        logger.info(f"[SalientObjectDetector] Cargando pesos desde {weights_path}")
        state_dict = torch.load(str(weights_path), map_location=self.device, weights_only=True)
        self.model.load_state_dict(state_dict)
        
        # Mover a dispositivo y modo evaluación
        self.model.to(self.device)
        self.model.eval()
        
        logger.info(f"[SalientObjectDetector] Modelo {self.model_type} cargado exitosamente")
    
    def _preprocess(self, image: np.ndarray) -> Tuple[torch.Tensor, Tuple[int, int]]:
        """
        Preprocesa imagen para el modelo.
        
        Args:
            image: Imagen BGR o grayscale (numpy array)
            
        Returns:
            (tensor, original_size)
        """
        original_size = image.shape[:2]  # (H, W)
        
        # Convertir a RGB si es necesario
        if len(image.shape) == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        elif image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
        else:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Resize a tamaño de entrada
        image = cv2.resize(image, (self.input_size, self.input_size))
        
        # Normalizar (similar a ImageNet)
        image = image.astype(np.float32) / 255.0
        image = (image - np.array([0.485, 0.456, 0.406])) / np.array([0.229, 0.224, 0.225])
        
        # Convertir a tensor: (H, W, C) -> (C, H, W) -> (1, C, H, W)
        tensor = torch.from_numpy(image.transpose(2, 0, 1)).float().unsqueeze(0)
        
        return tensor.to(self.device), original_size
    
    def _postprocess(self, output: torch.Tensor, original_size: Tuple[int, int]) -> np.ndarray:
        """
        Postprocesa la salida del modelo.
        
        Args:
            output: Tensor de salida del modelo
            original_size: (H, W) tamaño original
            
        Returns:
            Máscara de probabilidad [0-1] en tamaño original
        """
        # Tomar primera salida (d0 - máscara fusionada)
        mask = output[0].squeeze().cpu().numpy()
        
        # Normalizar a [0, 1]
        mask = (mask - mask.min()) / (mask.max() - mask.min() + 1e-8)
        
        # Resize al tamaño original
        mask = cv2.resize(mask, (original_size[1], original_size[0]))
        
        return mask
    
    @torch.no_grad()
    def get_mask(
        self, 
        image: np.ndarray,
        threshold: Optional[float] = None,
        return_probability: bool = False
    ) -> np.ndarray:
        """
        Obtiene la máscara de saliencia para una imagen.
        
        Args:
            image: Imagen BGR o grayscale (numpy array)
            threshold: Umbral para binarizar (None = retorna probabilidad)
            return_probability: Si True, retorna mapa de probabilidad [0-1]
            
        Returns:
            Máscara binaria (uint8, 0-255) o mapa de probabilidad (float32, 0-1)
        """
        if self.model is None:
            raise RuntimeError("Modelo no cargado")
        
        # Preprocesar
        tensor, original_size = self._preprocess(image)
        
        # Inferencia
        outputs = self.model(tensor)
        
        # Postprocesar
        mask = self._postprocess(outputs, original_size)
        
        if return_probability or threshold is None:
            return mask.astype(np.float32)
        
        # Binarizar
        binary_mask = (mask > threshold).astype(np.uint8) * 255
        return binary_mask
    
    def get_mask_with_bbox(
        self,
        image: np.ndarray,
        threshold: float = 0.5,
        min_area: int = 100
    ) -> Tuple[np.ndarray, Optional[Tuple[int, int, int, int]], Optional[Tuple[int, int]]]:
        """
        Obtiene máscara, bounding box y centroide del objeto saliente.
        
        Args:
            image: Imagen BGR o grayscale
            threshold: Umbral de binarización
            min_area: Área mínima para considerar objeto válido
            
        Returns:
            (mask, bounding_box, centroid)
            - mask: Máscara binaria uint8
            - bounding_box: (x, y, w, h) o None si no hay objeto
            - centroid: (cx, cy) o None si no hay objeto
        """
        # Obtener máscara de probabilidad
        prob_mask = self.get_mask(image, return_probability=True)
        
        # Binarizar
        binary_mask = (prob_mask > threshold).astype(np.uint8) * 255
        
        # Encontrar contornos
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return binary_mask, None, None
        
        # Filtrar por área y encontrar el mayor
        valid_contours = [(c, cv2.contourArea(c)) for c in contours if cv2.contourArea(c) >= min_area]
        
        if not valid_contours:
            return binary_mask, None, None
        
        # Seleccionar el mayor
        main_contour = max(valid_contours, key=lambda x: x[1])[0]
        
        # Bounding box
        x, y, w, h = cv2.boundingRect(main_contour)
        
        # Centroide
        M = cv2.moments(main_contour)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = x + w // 2, y + h // 2
        
        return binary_mask, (x, y, w, h), (cx, cy)
    
    def is_ready(self) -> bool:
        """Verifica si el modelo está listo para usar."""
        return self.model is not None
    
    def get_device(self) -> str:
        """Retorna el dispositivo actual."""
        return str(self.device)


def download_weights(model_type: str = 'u2netp') -> bool:
    """
    Función utilitaria para descargar pesos.
    
    Args:
        model_type: 'u2netp' o 'u2net'
        
    Returns:
        True si exitoso
    """
    detector = SalientObjectDetector.__new__(SalientObjectDetector)
    detector.model_type = model_type
    detector.auto_download = True
    return detector._download_weights()

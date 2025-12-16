"""
U2-Net Detector - Singleton para Detección de Objetos Salientes
================================================================

Carga el modelo U2-Net UNA SOLA VEZ al inicio de la aplicación.
Proporciona detección eficiente de objetos salientes con mapas de saliencia.

Autor: Sistema de Control L206
Fecha: 2025-12-12
"""

import os
import logging
import numpy as np
import cv2
from typing import Tuple, List, Dict, Optional

logger = logging.getLogger('MotorControl_L206')

# Intentar importar PyTorch
try:
    import torch
    import torch.nn.functional as F
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    logger.warning("[U2NetDetector] PyTorch no disponible - usando detección por contornos")


# Importar modelo unificado
from core.models.detected_object import DetectedObject


class U2NetDetector:
    """
    Singleton para detección de objetos salientes con U2-Net.
    
    Características:
    - Carga el modelo UNA SOLA VEZ (patrón Singleton)
    - Soporta GPU (CUDA) si está disponible
    - Fallback a detección por contornos si PyTorch no está disponible
    - Genera mapas de saliencia y lista de objetos detectados
    
    Uso:
        detector = U2NetDetector.get_instance()
        saliency_map, objects = detector.detect(frame)
    """
    
    _instance = None
    _initialized = False
    
    # Configuración del modelo
    MODEL_INPUT_SIZE = 320  # Tamaño de entrada del modelo
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    @classmethod
    def get_instance(cls) -> 'U2NetDetector':
        """Obtiene la instancia singleton del detector."""
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance
    
    def __init__(self):
        """Inicializa el detector (solo se ejecuta una vez)."""
        if U2NetDetector._initialized:
            return
        
        self.model = None
        self.device = None
        self.model_loaded = False
        
        # Parámetros de detección
        self.min_area = 500  # Área mínima en píxeles
        self.max_area = 500000  # Área máxima en píxeles
        self.saliency_threshold = 0.3  # Umbral de probabilidad (más bajo = más sensible)
        
        # Cargar modelo
        self._load_model()
        
        U2NetDetector._initialized = True
    
    def _load_model(self):
        """Carga el modelo U2-Net (u2netp para velocidad)."""
        if not TORCH_AVAILABLE:
            logger.info("[U2NetDetector] Usando detección por contornos (PyTorch no disponible)")
            return
        
        try:
            # Importar definición del modelo
            import sys
            src_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            if src_path not in sys.path:
                sys.path.insert(0, src_path)
            
            from models.u2net.model_def import U2NETP
            
            # Configurar dispositivo
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            
            # Crear modelo
            self.model = U2NETP(in_ch=3, out_ch=1)
            
            # Buscar archivo de pesos
            weights_paths = [
                os.path.join(src_path, '..', 'models', 'weights', 'u2netp.pth'),
                os.path.join(src_path, 'models', 'u2net', 'u2netp.pth'),
                'models/weights/u2netp.pth',
                'u2netp.pth'
            ]
            
            weights_path = None
            for path in weights_paths:
                if os.path.exists(path):
                    weights_path = path
                    break
            
            if weights_path is None:
                logger.warning("[U2NetDetector] Pesos u2netp.pth no encontrados - usando modelo sin entrenar")
                logger.warning(f"[U2NetDetector] Buscado en: {weights_paths}")
            else:
                # Cargar pesos
                state_dict = torch.load(weights_path, map_location=self.device)
                self.model.load_state_dict(state_dict)
                logger.info(f"[U2NetDetector] Pesos cargados desde: {weights_path}")
            
            # Mover a dispositivo y modo evaluación
            self.model.to(self.device)
            self.model.eval()
            
            # Warmup: primera inferencia siempre es lenta (compilación CUDA JIT)
            self._warmup()
            
            self.model_loaded = True
            logger.info(f"[U2NetDetector] ✅ Modelo U2-NETP cargado en {self.device}")
            
        except Exception as e:
            logger.error(f"[U2NetDetector] Error cargando modelo: {e}")
            self.model = None
            self.model_loaded = False
    
    def _warmup(self):
        """Ejecuta inferencias de warmup para compilar kernels CUDA."""
        if self.model is None:
            return
        
        logger.info("[U2NetDetector] Ejecutando warmup CUDA...")
        import time
        
        # Crear tensor dummy del tamaño de entrada
        dummy = torch.randn(1, 3, self.MODEL_INPUT_SIZE, self.MODEL_INPUT_SIZE).to(self.device)
        
        # Ejecutar 3 inferencias de warmup
        warmup_times = []
        with torch.no_grad():
            for i in range(3):
                t0 = time.perf_counter()
                _ = self.model(dummy)
                torch.cuda.synchronize() if self.device.type == 'cuda' else None
                t_ms = (time.perf_counter() - t0) * 1000
                warmup_times.append(t_ms)
                logger.info(f"[U2NetDetector] Warmup {i+1}/3: {t_ms:.0f}ms")
        
        # Limpiar memoria GPU
        del dummy
        if self.device.type == 'cuda':
            torch.cuda.empty_cache()
        
        logger.info(f"[U2NetDetector] Warmup completado. Tiempos: {[f'{t:.0f}ms' for t in warmup_times]}")
    
    def detect(self, image: np.ndarray) -> Tuple[np.ndarray, List[DetectedObject]]:
        """
        Detecta objetos salientes en la imagen.
        
        Args:
            image: Imagen BGR o grayscale (numpy array)
            
        Returns:
            saliency_map: Mapa de probabilidades [0-1] del mismo tamaño que la imagen
            objects: Lista de DetectedObject con bbox, área, probabilidad, etc.
        """
        if image is None or image.size == 0:
            return np.zeros((100, 100), dtype=np.float32), []
        
        # Si el modelo está cargado, usar U2-Net
        if self.model_loaded and self.model is not None:
            return self._detect_with_u2net(image)
        else:
            return self._detect_with_contours(image)
    
    def _detect_with_u2net(self, image: np.ndarray) -> Tuple[np.ndarray, List[DetectedObject]]:
        """Detección usando U2-Net con pipeline optimizado para GPU."""
        import time
        t_total = time.perf_counter()
        
        h_orig, w_orig = image.shape[:2]
        
        # PASO 1: Preprocesar (CPU → GPU)
        t0 = time.perf_counter()
        input_tensor = self._preprocess_gpu(image)
        t_preprocess = (time.perf_counter() - t0) * 1000
        
        # PASO 2: Inferencia GPU
        t0 = time.perf_counter()
        with torch.no_grad():
            outputs = self.model(input_tensor)
            d0 = outputs[0]
            # Mantener en GPU para resize
            saliency_gpu = d0.squeeze()
        t_inference = (time.perf_counter() - t0) * 1000
        
        # PASO 3: Resize en GPU y transferir a CPU
        t0 = time.perf_counter()
        saliency_gpu = saliency_gpu.unsqueeze(0).unsqueeze(0)  # [1, 1, H, W]
        saliency_gpu = F.interpolate(saliency_gpu, size=(h_orig, w_orig), mode='bilinear', align_corners=False)
        saliency = saliency_gpu.squeeze().cpu().numpy()
        
        # NO normalizar - valores bajos significan "no hay objeto saliente"
        # La normalización amplifica ruido como si fuera detección real
        
        t_resize = (time.perf_counter() - t0) * 1000
        
        # PASO 4: Extraer objetos (CPU - operaciones morfológicas)
        t0 = time.perf_counter()
        objects = self._extract_objects(saliency, image)
        t_extract = (time.perf_counter() - t0) * 1000
        
        t_total_ms = (time.perf_counter() - t_total) * 1000
        
        # Log detallado de tiempos (INFO para ver en log)
        logger.info(
            f"[U2Net] Total={t_total_ms:.0f}ms | "
            f"Preproc={t_preprocess:.0f}ms | Infer={t_inference:.0f}ms | "
            f"Resize={t_resize:.0f}ms | Extract={t_extract:.0f}ms | "
            f"Objetos={len(objects)}"
        )
        
        return saliency, objects
    
    def _preprocess_gpu(self, image: np.ndarray) -> 'torch.Tensor':
        """Preprocesa imagen usando GPU para operaciones pesadas."""
        # Convertir uint16 a uint8 si es necesario
        if image.dtype == np.uint16:
            image = (image / 256).astype(np.uint8)
        
        # Convertir a RGB
        if len(image.shape) == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        elif image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
        elif image.shape[2] == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Convertir a tensor y mover a GPU ANTES de resize
        tensor = torch.from_numpy(image).float().to(self.device)
        tensor = tensor.permute(2, 0, 1).unsqueeze(0)  # [1, 3, H, W]
        
        # Resize en GPU (mucho más rápido que cv2.resize)
        tensor = F.interpolate(tensor, size=(self.MODEL_INPUT_SIZE, self.MODEL_INPUT_SIZE), 
                               mode='bilinear', align_corners=False)
        
        # Normalizar en GPU
        tensor = tensor / 255.0
        mean = torch.tensor([0.485, 0.456, 0.406], device=self.device).view(1, 3, 1, 1)
        std = torch.tensor([0.229, 0.224, 0.225], device=self.device).view(1, 3, 1, 1)
        tensor = (tensor - mean) / std
        
        return tensor
    
    def _preprocess(self, image: np.ndarray) -> 'torch.Tensor':
        """Preprocesa imagen para U2-Net (versión CPU - fallback)."""
        # Convertir a RGB si es necesario
        if len(image.shape) == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        elif image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
        else:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Redimensionar
        image = cv2.resize(image, (self.MODEL_INPUT_SIZE, self.MODEL_INPUT_SIZE))
        
        # Normalizar [0, 1]
        image = image.astype(np.float32) / 255.0
        
        # Normalizar con media y std de ImageNet
        mean = np.array([0.485, 0.456, 0.406])
        std = np.array([0.229, 0.224, 0.225])
        image = (image - mean) / std
        
        # Convertir a tensor [1, 3, H, W]
        image = image.transpose(2, 0, 1)
        tensor = torch.from_numpy(image).unsqueeze(0).float()
        
        return tensor.to(self.device)
    
    def _extract_objects(self, saliency: np.ndarray, original_image: np.ndarray) -> List[DetectedObject]:
        """Extrae objetos del mapa de saliencia."""
        # DEBUG: Estadísticas del mapa de saliencia
        sal_min, sal_max, sal_mean = float(saliency.min()), float(saliency.max()), float(saliency.mean())
        logger.debug(f"[Extract] Saliency stats: min={sal_min:.3f}, max={sal_max:.3f}, mean={sal_mean:.3f}, threshold={self.saliency_threshold}")
        
        # Binarizar
        binary = (saliency > self.saliency_threshold).astype(np.uint8) * 255
        pixels_above = np.sum(binary > 0)
        logger.debug(f"[Extract] Pixels above threshold: {pixels_above} ({100*pixels_above/binary.size:.1f}%)")
        
        # Operaciones morfológicas para limpiar
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        # Encontrar contornos
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        logger.debug(f"[Extract] Contours found: {len(contours)}, min_area={self.min_area}, max_area={self.max_area}")
        
        objects = []
        rejected_small = 0
        rejected_large = 0
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            
            # Filtrar por área
            if area < self.min_area:
                rejected_small += 1
                continue
            if area > self.max_area:
                rejected_large += 1
                continue
            
            # Bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Centroide
            M = cv2.moments(contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
            else:
                cx, cy = x + w // 2, y + h // 2
            
            # Probabilidad promedio en la región
            mask = np.zeros(saliency.shape, dtype=np.uint8)
            cv2.drawContours(mask, [contour], -1, 255, -1)
            probability = float(np.mean(saliency[mask > 0]))
            
            obj = DetectedObject(
                index=len(objects),
                bbox=(x, y, w, h),
                area=int(area),
                probability=probability,
                centroid=(cx, cy),
                contour=contour
            )
            objects.append(obj)
        
        # Log de rechazos
        if rejected_small > 0 or rejected_large > 0:
            logger.debug(f"[Extract] Rejected: {rejected_small} too small, {rejected_large} too large")
        
        # Ordenar por área (mayor primero)
        objects.sort(key=lambda o: o.area, reverse=True)
        
        # Reasignar índices
        for i, obj in enumerate(objects):
            obj.index = i
        
        logger.debug(f"[Extract] Final objects: {len(objects)}")
        return objects
    
    def _detect_with_contours(self, image: np.ndarray) -> Tuple[np.ndarray, List[DetectedObject]]:
        """Detección fallback usando contornos (sin U2-Net)."""
        # Convertir a grayscale
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # Normalizar si es uint16
        if gray.dtype == np.uint16:
            gray = (gray / 256).astype(np.uint8)
        
        # CLAHE para mejorar contraste
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        
        # Desenfoque
        blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)
        
        # Umbralización Otsu
        _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        # Morfología
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        # Crear mapa de saliencia pseudo
        saliency = binary.astype(np.float32) / 255.0
        
        # Encontrar contornos
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        objects = []
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            
            if area < self.min_area or area > self.max_area:
                continue
            
            x, y, w, h = cv2.boundingRect(contour)
            
            M = cv2.moments(contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
            else:
                cx, cy = x + w // 2, y + h // 2
            
            # Compacidad como proxy de probabilidad
            perimeter = cv2.arcLength(contour, True)
            if perimeter > 0:
                compactness = (4 * np.pi * area) / (perimeter ** 2)
                probability = min(1.0, compactness)
            else:
                probability = 0.5
            
            obj = DetectedObject(
                index=len(objects),
                bbox=(x, y, w, h),
                area=int(area),
                probability=probability,
                centroid=(cx, cy),
                contour=contour
            )
            objects.append(obj)
        
        objects.sort(key=lambda o: o.area, reverse=True)
        for i, obj in enumerate(objects):
            obj.index = i
        
        return saliency, objects
    
    def set_parameters(self, min_area: int = None, max_area: int = None, 
                       saliency_threshold: float = None):
        """Actualiza parámetros de detección."""
        if min_area is not None:
            self.min_area = min_area
        if max_area is not None:
            self.max_area = max_area
        if saliency_threshold is not None:
            self.saliency_threshold = saliency_threshold
    
    def is_model_loaded(self) -> bool:
        """Retorna True si el modelo U2-Net está cargado."""
        return self.model_loaded
    
    def get_device(self) -> str:
        """Retorna el dispositivo usado (cuda/cpu)."""
        return str(self.device) if self.device else "cpu (fallback)"

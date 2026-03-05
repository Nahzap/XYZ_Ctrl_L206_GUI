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
from enum import Enum

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


class DetectionMode(Enum):
    """Modos de detección preconfigurados."""
    NORMAL = "normal"
    SENSITIVE = "sensitive"  # Para polen/objetos pequeños
    ROBUST = "robust"        # Para objetos grandes con ruido
    HYBRID = "hybrid"        # U2NET + Gradiente para polen desenfocado


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
        
        # Parámetros de detección (valores por defecto)
        self.min_area = 500  # Área mínima en píxeles
        self.max_area = 500000  # Área máxima en píxeles
        self.saliency_threshold = 0.3  # Umbral de probabilidad
        self.adaptive_k = 0.5  # Factor adaptativo para umbral
        self.morph_kernel_size = 5  # Tamaño kernel morfológico
        self.clahe_clip_limit = 2.0  # CLAHE clip limit
        self.clahe_tile_size = (8, 8)  # CLAHE tile size
        
        # Modo de detección
        self.detection_mode = DetectionMode.NORMAL
        
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
        
        # Si modo HYBRID, usar detección híbrida
        if self.detection_mode == DetectionMode.HYBRID:
            return self.detect_hybrid(image)
        
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
        # R3: MEJORA DE CONTRASTE OPTIMIZADA (GPU)
        # Normalizar uint16 preservando rango dinámico
        if image.dtype == np.uint16:
            # Normalización min-max para preservar detalles
            image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        
        # Convertir a grayscale para CLAHE
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # CLAHE para mejorar contraste local (parámetros configurables)
        clahe = cv2.createCLAHE(clipLimit=self.clahe_clip_limit, tileGridSize=self.clahe_tile_size)
        enhanced = clahe.apply(gray)
        
        # Sharpening suave para objetos borrosos (kernel optimizado)
        # Usar unsharp masking en lugar de kernel directo (más suave)
        blurred = cv2.GaussianBlur(enhanced, (3, 3), 1.0)
        sharpened = cv2.addWeighted(enhanced, 1.5, blurred, -0.5, 0)
        
        # Convertir a RGB para U2-Net
        image = cv2.cvtColor(sharpened, cv2.COLOR_GRAY2RGB)
        
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
        # R2: UMBRAL ADAPTATIVO BASADO EN ESTADÍSTICAS
        sal_min = float(np.min(saliency))
        sal_max = float(np.max(saliency))
        sal_mean = float(np.mean(saliency))
        sal_std = float(np.std(saliency))
        
        # Calcular umbral adaptativo: media + k*std, con límites
        # k configurable según modo de detección
        adaptive_threshold = sal_mean + self.adaptive_k * sal_std
        
        # Límites: mínimo 0.15 (muy sensible), máximo 0.5 (conservador)
        adaptive_threshold = max(0.15, min(0.5, adaptive_threshold))
        
        # Si la saliencia máxima es muy baja, usar umbral más bajo
        if sal_max < 0.3:
            adaptive_threshold = min(adaptive_threshold, sal_max * 0.7)
        
        logger.debug(
            f"[Extract] Saliency: min={sal_min:.3f}, max={sal_max:.3f}, "
            f"mean={sal_mean:.3f}, std={sal_std:.3f} → threshold={adaptive_threshold:.3f} "
            f"(fixed={self.saliency_threshold:.3f})"
        )
        
        # Binarizar con umbral adaptativo
        binary = (saliency > adaptive_threshold).astype(np.uint8) * 255
        pixels_above = np.sum(binary > 0)
        logger.debug(f"[Extract] Pixels above threshold: {pixels_above} ({100*pixels_above/binary.size:.1f}%)")
        
        # Operaciones morfológicas para limpiar (kernel configurable)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, 
                                          (self.morph_kernel_size, self.morph_kernel_size))
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
    
    def _detect_blurred_objects(self, image: np.ndarray) -> List[DetectedObject]:
        """
        Detección especializada para objetos muy desenfocados usando gradientes.
        
        Método:
        1. CLAHE agresivo para realzar gradientes suaves
        2. Calcular gradiente con Sobel (magnitud)
        3. Umbralización adaptativa sobre gradiente
        4. Morfología MUY SUAVE (kernel 2×2)
        5. Filtrar por circularidad (muy permisivo: >0.25)
        
        Args:
            image: Imagen de entrada (BGR o grayscale)
            
        Returns:
            Lista de DetectedObject detectados por gradiente
        """
        import time
        t0 = time.perf_counter()
        
        # Convertir a grayscale
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # Normalizar uint16 → uint8
        if gray.dtype == np.uint16:
            gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        
        # CLAHE agresivo para realzar gradientes suaves
        clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(4, 4))
        enhanced = clahe.apply(gray)
        
        # Calcular gradiente con Sobel (kernel 5×5 para suavidad)
        grad_x = cv2.Sobel(enhanced, cv2.CV_64F, 1, 0, ksize=5)
        grad_y = cv2.Sobel(enhanced, cv2.CV_64F, 0, 1, ksize=5)
        gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)
        
        # Normalizar gradiente a [0, 255]
        gradient_magnitude = cv2.normalize(gradient_magnitude, None, 0, 255, 
                                          cv2.NORM_MINMAX).astype(np.uint8)
        
        # Umbralización adaptativa sobre gradiente
        # blockSize=21 (vecindario grande para gradientes suaves)
        # C=-5 (negativo para invertir, capturar regiones oscuras)
        binary = cv2.adaptiveThreshold(
            gradient_magnitude, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            21, -5
        )
        
        # Morfología MUY SUAVE (kernel 2×2, preserva geometría)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
        # NO aplicar MORPH_OPEN (destruiría objetos pequeños)
        
        # Encontrar contornos
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        objects = []
        rejected_small = 0
        rejected_circularity = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filtro de área (más permisivo: min_area/2)
            min_area_gradient = self.min_area // 2
            if area < min_area_gradient or area > self.max_area:
                rejected_small += 1
                continue
            
            # Bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Calcular circularidad
            perimeter = cv2.arcLength(contour, True)
            if perimeter > 0:
                circularity = (4 * np.pi * area) / (perimeter ** 2)
            else:
                circularity = 0.0
            
            # Filtro de circularidad MÁS PERMISIVO (0.25 vs 0.45 en U2NET)
            if circularity < 0.25:
                rejected_circularity += 1
                continue
            
            # Centroide
            M = cv2.moments(contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
            else:
                cx, cy = x + w // 2, y + h // 2
            
            # Probabilidad basada en circularidad (más circular = más probable)
            probability = min(1.0, circularity * 1.5)
            
            obj = DetectedObject(
                index=len(objects),
                bbox=(x, y, w, h),
                area=int(area),
                probability=probability,
                centroid=(cx, cy),
                contour=contour,
                circularity=circularity
            )
            objects.append(obj)
        
        t_ms = (time.perf_counter() - t0) * 1000
        
        logger.info(
            f"[BlurredDetection] {len(objects)} objetos detectados | "
            f"Rechazados: {rejected_small} (área), {rejected_circularity} (circ) | "
            f"Tiempo: {t_ms:.0f}ms"
        )
        
        return objects
    
    def _merge_detections(self, 
                         objects_a: List[DetectedObject],
                         objects_b: List[DetectedObject],
                         iou_threshold: float = 0.5) -> List[DetectedObject]:
        """
        Fusiona dos listas de objetos eliminando duplicados con IoU.
        
        Algoritmo:
        1. Concatenar todas las detecciones
        2. Ordenar por probabilidad (mayor primero)
        3. NMS: suprimir objetos con IoU > threshold
        4. Reindexar objetos finales
        
        Args:
            objects_a: Lista A (típicamente U2NET)
            objects_b: Lista B (típicamente Gradiente)
            iou_threshold: Umbral IoU para considerar duplicado (default: 0.5)
            
        Returns:
            Lista fusionada sin duplicados
        """
        # Concatenar todas las detecciones
        all_objects = list(objects_a) + list(objects_b)
        
        if len(all_objects) == 0:
            return []
        
        # Ordenar por probabilidad (mayor primero)
        all_objects.sort(key=lambda o: o.probability, reverse=True)
        
        # NMS: Non-Maximum Suppression
        keep = []
        suppressed = set()
        
        for i, obj_i in enumerate(all_objects):
            if i in suppressed:
                continue
            
            keep.append(obj_i)
            
            # Suprimir objetos con alto overlap
            for j in range(i + 1, len(all_objects)):
                if j in suppressed:
                    continue
                
                obj_j = all_objects[j]
                iou = self._calculate_iou(obj_i.bbox, obj_j.bbox)
                
                if iou > iou_threshold:
                    suppressed.add(j)
                    logger.debug(f"[NMS] Suprimido obj {j} (IoU={iou:.2f} con obj {i})")
        
        # Reindexar
        for i, obj in enumerate(keep):
            obj.index = i
        
        logger.info(f"[NMS] Fusión: {len(objects_a)} + {len(objects_b)} → {len(keep)} objetos")
        return keep
    
    def _calculate_iou(self, 
                      bbox1: Tuple[int, int, int, int],
                      bbox2: Tuple[int, int, int, int]) -> float:
        """
        Calcula Intersection over Union entre dos bounding boxes.
        
        IoU = Area(intersection) / Area(union)
        
        Args:
            bbox1: (x, y, w, h)
            bbox2: (x, y, w, h)
            
        Returns:
            IoU en [0, 1]
        """
        x1, y1, w1, h1 = bbox1
        x2, y2, w2, h2 = bbox2
        
        # Calcular intersección
        x_left = max(x1, x2)
        y_top = max(y1, y2)
        x_right = min(x1 + w1, x2 + w2)
        y_bottom = min(y1 + h1, y2 + h2)
        
        if x_right < x_left or y_bottom < y_top:
            return 0.0
        
        intersection = (x_right - x_left) * (y_bottom - y_top)
        
        # Calcular unión
        area1 = w1 * h1
        area2 = w2 * h2
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0
    
    def detect_hybrid(self, image: np.ndarray) -> Tuple[np.ndarray, List[DetectedObject]]:
        """
        Detección híbrida: U2NET + Gradiente con fusión inteligente.
        
        Pipeline:
        1. RAMA A: U2NET → objetos con textura/contraste (polen enfocado)
        2. RAMA B: Gradiente → blobs circulares (polen desenfocado)
        3. Fusión NMS → eliminar duplicados (IoU > 0.5)
        4. Mapa de saliencia combinado
        
        Args:
            image: Imagen de entrada (BGR o grayscale)
            
        Returns:
            saliency_combined: Mapa de saliencia fusionado
            objects_fused: Lista de objetos sin duplicados
        """
        import time
        t_total = time.perf_counter()
        
        # RAMA A: U2NET (objetos bien definidos)
        if self.model_loaded and self.model is not None:
            saliency_u2net, objects_u2net = self._detect_with_u2net(image)
        else:
            saliency_u2net, objects_u2net = self._detect_with_contours(image)
        
        logger.info(f"[Hybrid] U2NET: {len(objects_u2net)} objetos")
        
        # RAMA B: Gradiente (objetos desenfocados)
        objects_gradient = self._detect_blurred_objects(image)
        logger.info(f"[Hybrid] Gradiente: {len(objects_gradient)} objetos")
        
        # FUSIÓN: NMS con IoU
        objects_fused = self._merge_detections(
            objects_u2net, 
            objects_gradient,
            iou_threshold=0.5
        )
        
        # Mapa de saliencia combinado (para visualización)
        if len(objects_gradient) > 0:
            saliency_gradient = self._create_gradient_saliency(objects_gradient, image.shape[:2])
            # Combinar con peso 0.7 para gradiente (menos confiable que U2NET)
            saliency_combined = np.maximum(saliency_u2net, saliency_gradient * 0.7)
        else:
            saliency_combined = saliency_u2net
        
        t_total_ms = (time.perf_counter() - t_total) * 1000
        logger.info(f"[Hybrid] Total: {len(objects_fused)} objetos | Tiempo: {t_total_ms:.0f}ms")
        
        return saliency_combined, objects_fused
    
    def _create_gradient_saliency(self, 
                                  objects: List[DetectedObject],
                                  shape: Tuple[int, int]) -> np.ndarray:
        """
        Crea mapa de saliencia pseudo para visualización de detecciones por gradiente.
        
        Dibuja Gaussianos centrados en cada objeto detectado.
        
        Args:
            objects: Lista de objetos detectados por gradiente
            shape: (height, width) de la imagen
            
        Returns:
            Mapa de saliencia [0, 1]
        """
        h, w = shape
        saliency = np.zeros((h, w), dtype=np.float32)
        
        for obj in objects:
            cx, cy = obj.centroid
            
            # Radio del objeto (máximo de w, h del bbox)
            r = max(obj.w, obj.h) // 2
            
            # Dibujar Gaussiano centrado en objeto
            y1, y2 = max(0, cy - r), min(h, cy + r)
            x1, x2 = max(0, cx - r), min(w, cx + r)
            
            for yy in range(y1, y2):
                for xx in range(x1, x2):
                    dist = np.sqrt((xx - cx)**2 + (yy - cy)**2)
                    if dist < r:
                        # Gaussiano: exp(-dist²/2σ²)
                        sigma = r / 2.0
                        val = np.exp(-(dist**2) / (2 * sigma**2))
                        # Actualizar saliencia (máximo entre actual y nuevo)
                        saliency[yy, xx] = max(saliency[yy, xx], val * obj.probability)
        
        return saliency
    
    def set_detection_mode(self, mode: DetectionMode):
        """Cambia el modo de detección y aplica parámetros preconfigurados."""
        self.detection_mode = mode
        self._apply_mode_parameters()
        logger.info(f"[U2NetDetector] Modo cambiado a: {mode.value}")
    
    def _apply_mode_parameters(self):
        """Aplica parámetros según el modo activo."""
        if self.detection_mode == DetectionMode.HYBRID:
            # HÍBRIDO: U2NET + Gradiente para polen desenfocado
            self.min_area = 50
            self.saliency_threshold = 0.15
            self.adaptive_k = 0.2
            self.morph_kernel_size = 2
            self.clahe_clip_limit = 4.0
            self.clahe_tile_size = (4, 4)
            logger.info("[U2NetDetector] Parámetros HÍBRIDO aplicados (U2NET + Gradiente)")
            
        elif self.detection_mode == DetectionMode.SENSITIVE:
            # POLEN / OBJETOS PEQUEÑOS
            self.min_area = 100
            self.saliency_threshold = 0.15
            self.adaptive_k = 0.3
            self.morph_kernel_size = 3
            self.clahe_clip_limit = 3.5
            self.clahe_tile_size = (4, 4)
            logger.info("[U2NetDetector] Parámetros SENSIBLE aplicados (polen/objetos pequeños)")
            
        elif self.detection_mode == DetectionMode.ROBUST:
            # OBJETOS GRANDES CON RUIDO
            self.min_area = 1000
            self.saliency_threshold = 0.35
            self.adaptive_k = 0.7
            self.morph_kernel_size = 7
            self.clahe_clip_limit = 2.0
            self.clahe_tile_size = (8, 8)
            logger.info("[U2NetDetector] Parámetros ROBUSTO aplicados (objetos grandes)")
            
        else:  # NORMAL
            self.min_area = 500
            self.saliency_threshold = 0.3
            self.adaptive_k = 0.5
            self.morph_kernel_size = 5
            self.clahe_clip_limit = 2.0
            self.clahe_tile_size = (8, 8)
            logger.info("[U2NetDetector] Parámetros NORMAL aplicados")
    
    def set_advanced_parameters(self, 
                               saliency_threshold: float = None,
                               adaptive_k: float = None,
                               morph_kernel_size: int = None,
                               clahe_clip_limit: float = None,
                               clahe_tile_size: tuple = None):
        """Actualiza parámetros avanzados de detección."""
        if saliency_threshold is not None:
            self.saliency_threshold = np.clip(saliency_threshold, 0.1, 0.5)
        if adaptive_k is not None:
            self.adaptive_k = np.clip(adaptive_k, 0.1, 1.0)
        if morph_kernel_size is not None:
            self.morph_kernel_size = morph_kernel_size
        if clahe_clip_limit is not None:
            self.clahe_clip_limit = np.clip(clahe_clip_limit, 1.0, 5.0)
        if clahe_tile_size is not None:
            self.clahe_tile_size = clahe_tile_size
        
        logger.info(f"[U2NetDetector] Parámetros actualizados: "
                   f"sal_thr={self.saliency_threshold:.2f}, k={self.adaptive_k:.2f}, "
                   f"kernel={self.morph_kernel_size}, clahe_clip={self.clahe_clip_limit:.1f}, "
                   f"clahe_tiles={self.clahe_tile_size}")
    
    def set_parameters(self, min_area: int = None, max_area: int = None, 
                       saliency_threshold: float = None):
        """Actualiza parámetros de detección (legacy compatibility)."""
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
    
    def get_parameters(self) -> Dict:
        """Retorna parámetros actuales del detector."""
        return {
            'mode': self.detection_mode.value,
            'min_area': self.min_area,
            'max_area': self.max_area,
            'saliency_threshold': self.saliency_threshold,
            'adaptive_k': self.adaptive_k,
            'morph_kernel_size': self.morph_kernel_size,
            'clahe_clip_limit': self.clahe_clip_limit,
            'clahe_tile_size': self.clahe_tile_size,
            'model_loaded': self.model_loaded,
            'device': self.get_device()
        }

"""
Controlador de autofoco multi-objeto con C-Focus piezo.
Integra detección U2-Net con búsqueda de foco por objeto.
"""

import os
import time
import logging
from dataclasses import dataclass
from typing import List, Tuple, Optional, Callable
import numpy as np
import cv2

logger = logging.getLogger('MotorControl_L206')


@dataclass
class DetectedObject:
    """Objeto detectado en pre-escaneo."""
    index: int
    bounding_box: Tuple[int, int, int, int]
    centroid: Tuple[int, int]
    area: float
    initial_score: float
    circularity: float = 0.0  # Métrica de forma: 1.0 = círculo perfecto


@dataclass
class FocusedCapture:
    """Resultado de captura enfocada de un objeto."""
    object_index: int
    z_optimal: float
    focus_score: float
    image_path: str
    bounding_box: Tuple[int, int, int, int]


class MultiObjectAutofocusController:
    """
    Controlador de autofoco multi-objeto con C-Focus.
    
    Flujo:
    1. Pre-detectar objetos con U2-Net (filtrar por área en píxeles)
    2. Para cada objeto, buscar su Z óptimo usando C-Focus
    3. Capturar imagen cuando cada objeto esté enfocado
    """
    
    def __init__(
        self,
        scorer,
        cfocus_controller,
        get_frame_callback: Callable[[], np.ndarray]
    ):
        """
        Inicializa el controlador de autofoco.
        
        Args:
            scorer: SmartFocusScorer para detección y scoring
            cfocus_controller: Controlador del piezo C-Focus
            get_frame_callback: Función para obtener frame de cámara
        """
        self.scorer = scorer
        self.cfocus = cfocus_controller
        self.get_frame = get_frame_callback
        
        self.z_search_range = 50.0
        self.z_tolerance = 0.5
        self.max_iterations = 20
        
        self.min_area_pixels = 100
        self.max_area_pixels = 50000
        self.min_probability = 0.3
        
        # Usar parámetros del SmartFocusScorer (configurables desde UI)
        self.min_circularity = scorer.min_circularity if hasattr(scorer, 'min_circularity') else 0.45
        self.min_aspect_ratio = scorer.min_aspect_ratio if hasattr(scorer, 'min_aspect_ratio') else 0.4
        
        logger.info(f"MultiObjectAutofocus inicializado: "
                   f"rango={self.z_search_range}µm, "
                   f"área=[{self.min_area_pixels}, {self.max_area_pixels}] px, "
                   f"circularidad_min={self.min_circularity:.2f}, "
                   f"aspect_min={self.min_aspect_ratio:.2f}")
    
    def set_pixel_threshold(self, min_pixels: int, max_pixels: int):
        """
        Configura el umbral de píxeles para filtrar objetos.
        
        Args:
            min_pixels: Área mínima en píxeles
            max_pixels: Área máxima en píxeles
        """
        self.min_area_pixels = min_pixels
        self.max_area_pixels = max_pixels
        logger.info(f"Umbral de píxeles actualizado: [{min_pixels}, {max_pixels}]")
    
    def predetect_objects(self) -> List[DetectedObject]:
        """
        FASE 2: Pre-detecta todos los objetos en el frame actual.
        Filtra por umbral de píxeles (min_area ≤ área ≤ max_area).
        
        Returns:
            Lista de objetos detectados válidos
        """
        frame = self.get_frame()
        
        if frame is None:
            logger.error("No hay frame disponible para pre-detección")
            return []
        
        # Actualizar parámetros de morfología desde SmartFocusScorer (configurables desde UI)
        if hasattr(self.scorer, 'min_circularity'):
            self.min_circularity = self.scorer.min_circularity
        if hasattr(self.scorer, 'min_aspect_ratio'):
            self.min_aspect_ratio = self.scorer.min_aspect_ratio
        
        # Usar SmartFocusScorer.detect_objects()
        raw_objects = self.scorer.detect_objects(frame)
        
        if not raw_objects:
            logger.info("Pre-detección: sin objetos detectados")
            return []
        
        detected = []
        for i, obj_dict in enumerate(raw_objects):
            area_px = obj_dict['area']
            bbox = obj_dict['bbox']
            
            # Filtrar por umbral de píxeles
            if self.min_area_pixels <= area_px <= self.max_area_pixels:
                x, y, w, h = bbox
                centroid = (x + w // 2, y + h // 2)
                
                # MEJORA: Usar circularidad del objeto si está disponible (calculada con contorno real)
                if 'circularity' in obj_dict and obj_dict['circularity'] > 0:
                    circularity = obj_dict['circularity']
                else:
                    # Fallback: calcular con bbox
                    perimeter = 2 * (w + h)
                    circularity = (4 * np.pi * area_px) / (perimeter ** 2) if perimeter > 0 else 0
                
                # Filtrar por circularidad mínima (rechazar manchas irregulares)
                if circularity < self.min_circularity:
                    logger.info(f"  Objeto {i} RECHAZADO: circularidad={circularity:.2f} < {self.min_circularity:.2f} (mancha irregular)")
                    continue
                
                # MEJORA: Filtrar por aspect ratio (rechazar manchas muy alargadas)
                aspect_ratio = float(w) / float(h) if h > 0 else 1.0
                if aspect_ratio > 1.0:
                    aspect_ratio = 1.0 / aspect_ratio
                
                if aspect_ratio < self.min_aspect_ratio:
                    logger.info(f"  Objeto {i} RECHAZADO: aspect_ratio={aspect_ratio:.2f} < {self.min_aspect_ratio:.2f} (muy alargado)")
                    continue
                
                # Calcular score inicial en el ROI
                initial_score = self.scorer.calculate_sharpness(frame, roi=bbox)
                
                detected.append(DetectedObject(
                    index=i,
                    bounding_box=bbox,
                    centroid=centroid,
                    area=area_px,
                    initial_score=initial_score,
                    circularity=circularity
                ))
                logger.debug(f"  Objeto {i}: área={area_px:.0f}px, circ={circularity:.2f}, aspect={aspect_ratio:.2f}, score={initial_score:.1f}")
            else:
                logger.debug(f"  Objeto {i} RECHAZADO: área={area_px:.0f}px fuera de rango")
        
        logger.info(f"Pre-detección: {len(detected)}/{len(raw_objects)} objetos válidos")
        return detected
    
    def focus_single_object(
        self, 
        obj: DetectedObject,
        z_center: Optional[float] = None,
        use_full_scan: bool = True
    ) -> Tuple[float, float]:
        """
        FASE 3: Busca el Z óptimo para UN objeto específico usando C-Focus.
        
        Implementa Z-scanning completo (0-100µm) evaluando índice S en cada paso
        para encontrar el Best Plane of Focus (BPoF).
        
        Args:
            obj: Objeto a enfocar
            z_center: Centro del rango de búsqueda (ignorado si use_full_scan=True)
            use_full_scan: Si True, escanea todo el rango Z disponible
            
        Returns:
            (z_optimal, max_score)
        """
        z_range_max = self.cfocus.get_z_range()
        
        if use_full_scan:
            # MODO 1: Z-SCANNING RÁPIDO Y OPTIMIZADO
            z_min = 0.0
            z_max = z_range_max
            z_step = 5.0  # Paso más grande: 5µm (20 evaluaciones en 100µm)
            
            logger.info(f"Autofoco Obj{obj.index}: Z-SCAN RÁPIDO [0 → {z_max:.0f}µm], paso={z_step}µm")
            
            # Escaneo rápido con settle time mínimo
            z_positions = []
            sharpness_scores = []
            
            z_current = z_min
            while z_current <= z_max:
                # Mover sin esperar settle completo (piezo es rápido)
                self.cfocus.move_z(z_current)
                time.sleep(0.05)  # 50ms settle mínimo
                
                # Capturar y evaluar
                frame = self.get_frame()
                if frame is not None:
                    score = self.scorer.calculate_sharpness(frame, roi=obj.bounding_box)
                    z_positions.append(z_current)
                    sharpness_scores.append(score)
                
                z_current += z_step
            
            if not z_positions:
                logger.error("No se pudo evaluar ninguna posición Z")
                return 0.0, 0.0
            
            # Encontrar máximo
            max_idx = np.argmax(sharpness_scores)
            z_peak = z_positions[max_idx]
            max_score = sharpness_scores[max_idx]
            
            logger.info(f"  Escaneo: BPoF en Z={z_peak:.1f}µm, S={max_score:.1f} ({len(z_positions)} eval)")
            
            # Refinamiento rápido ±2 pasos alrededor del pico
            z_refine_positions = []
            z_refine_scores = []
            
            for offset in [-2, -1, 0, 1, 2]:
                z_test = z_peak + offset * 1.0  # Pasos de 1µm
                if 0 <= z_test <= z_range_max:
                    self.cfocus.move_z(z_test)
                    time.sleep(0.05)
                    frame = self.get_frame()
                    if frame is not None:
                        score = self.scorer.calculate_sharpness(frame, roi=obj.bounding_box)
                        z_refine_positions.append(z_test)
                        z_refine_scores.append(score)
            
            if z_refine_scores:
                max_refine_idx = np.argmax(z_refine_scores)
                z_optimal = z_refine_positions[max_refine_idx]
                final_score = z_refine_scores[max_refine_idx]
            else:
                z_optimal = z_peak
                final_score = max_score
            
            logger.info(f"Autofoco Obj{obj.index}: BPoF final Z={z_optimal:.2f}µm, S={final_score:.1f}")
            
        else:
            # MODO 2: Golden Section Search (más rápido, menos exhaustivo)
            if z_center is None:
                z_center = self.cfocus.read_z()
                if z_center is None:
                    logger.error("No se pudo leer posición Z actual")
                    return 0.0, 0.0
            
            z_min = max(0, z_center - self.z_search_range / 2)
            z_max = min(z_range_max, z_center + self.z_search_range / 2)
            
            logger.info(f"Autofoco Obj{obj.index}: Golden Search [{z_min:.1f}, {z_max:.1f}]µm")
            
            phi = 0.618
            iteration = 0
            best_score = 0.0
            
            while (z_max - z_min) > self.z_tolerance and iteration < self.max_iterations:
                z1 = z_max - phi * (z_max - z_min)
                z2 = z_min + phi * (z_max - z_min)
                
                # Evaluar z1
                self.cfocus.move_z(z1)
                time.sleep(0.05)
                frame1 = self.get_frame()
                s1 = self.scorer.calculate_sharpness(frame1, roi=obj.bounding_box) if frame1 is not None else 0.0
                
                # Evaluar z2
                self.cfocus.move_z(z2)
                time.sleep(0.05)
                frame2 = self.get_frame()
                s2 = self.scorer.calculate_sharpness(frame2, roi=obj.bounding_box) if frame2 is not None else 0.0
                
                if s1 > s2:
                    z_max = z2
                    best_score = s1
                else:
                    z_min = z1
                    best_score = s2
                
                iteration += 1
            
            z_optimal = (z_min + z_max) / 2
            final_score = best_score
            
            logger.info(f"Autofoco Obj{obj.index} completado: Z={z_optimal:.2f}µm, S={final_score:.1f}")
        
        # Mover a posición óptima final
        self.cfocus.move_z(z_optimal)
        time.sleep(0.05)  # Settle time mínimo
        
        return z_optimal, final_score
    
    def capture_all_objects(
        self,
        objects: List[DetectedObject],
        save_folder: str,
        class_name: str,
        point_index: int,
        config: dict,
        use_full_scan: bool = True
    ) -> List[FocusedCapture]:
        """
        FASE 3+4: Enfoca y captura cada objeto individualmente.
        
        Args:
            objects: Lista de objetos pre-detectados
            save_folder: Carpeta de destino
            class_name: Nombre de la clase para el archivo
            point_index: Índice del punto de trayectoria
            config: Configuración de microscopía (canales, etc.)
            use_full_scan: Si True, usa Z-scanning completo; si False, Golden Section
            
        Returns:
            Lista de capturas realizadas
        """
        captures = []
        z_start = self.cfocus.read_z()
        
        if z_start is None:
            logger.error("No se pudo leer Z inicial")
            return captures
        
        scan_mode = "Z-SCAN COMPLETO" if use_full_scan else "Golden Section"
        logger.info(f"Capturando {len(objects)} objetos en punto {point_index} (Modo: {scan_mode})")
        
        for obj in objects:
            z_opt, score = self.focus_single_object(obj, z_center=z_start, use_full_scan=use_full_scan)
            
            if score < 5.0:
                logger.warning(f"Obj{obj.index}: score bajo ({score:.1f}), saltando captura")
                continue
            
            frame = self.get_frame()
            if frame is None:
                logger.error(f"No hay frame para capturar Obj{obj.index}")
                continue
            
            processed_frame = self._process_frame(frame, config)
            
            filename = f"{class_name}_{point_index:05d}_obj{obj.index:02d}.png"
            filepath = os.path.join(save_folder, filename)
            
            success = cv2.imwrite(filepath, processed_frame, [cv2.IMWRITE_PNG_COMPRESSION, 6])
            
            if success:
                file_size_kb = os.path.getsize(filepath) / 1024
                logger.info(f"  {filename}: Z={z_opt:.2f}µm, S={score:.1f}, {file_size_kb:.0f}KB")
                
                captures.append(FocusedCapture(
                    object_index=obj.index,
                    z_optimal=z_opt,
                    focus_score=score,
                    image_path=filepath,
                    bounding_box=obj.bounding_box
                ))
            else:
                logger.error(f"Error guardando {filename}")
        
        logger.info(f"Punto {point_index}: {len(captures)} imágenes capturadas")
        return captures
    
    def _process_frame(self, frame: np.ndarray, config: dict) -> np.ndarray:
        """
        Procesa el frame según configuración (canales, resize, etc.).
        Replica lógica de camera_tab.capture_microscopy_image().
        """
        if frame.dtype == np.uint16:
            if frame.max() > 0:
                frame = (frame / frame.max() * 255).astype(np.uint8)
            else:
                frame = frame.astype(np.uint8)
        
        target_width = config.get('img_width', 1920)
        target_height = config.get('img_height', 1080)
        h, w = frame.shape[:2]
        
        if w != target_width or h != target_height:
            frame = cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_LINEAR)
        
        channels = config.get('channels', {'R': False, 'G': True, 'B': False})
        selected_channels = [c for c in ['R', 'G', 'B'] if channels.get(c, False)]
        n_selected = len(selected_channels)
        
        if len(frame.shape) == 2:
            if n_selected >= 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                if n_selected < 3:
                    new_frame = np.zeros_like(frame)
                    if channels.get('B', False):
                        new_frame[:, :, 0] = frame[:, :, 0]
                    if channels.get('G', False):
                        new_frame[:, :, 1] = frame[:, :, 1]
                    if channels.get('R', False):
                        new_frame[:, :, 2] = frame[:, :, 2]
                    frame = new_frame
        
        elif len(frame.shape) == 3:
            if n_selected == 1:
                channel_map = {'B': 0, 'G': 1, 'R': 2}
                channel_idx = channel_map[selected_channels[0]]
                frame = frame[:, :, channel_idx]
            elif n_selected < 3:
                new_frame = np.zeros_like(frame)
                if channels.get('B', False):
                    new_frame[:, :, 0] = frame[:, :, 0]
                if channels.get('G', False):
                    new_frame[:, :, 1] = frame[:, :, 1]
                if channels.get('R', False):
                    new_frame[:, :, 2] = frame[:, :, 2]
                frame = new_frame
        
        return frame

"""
Servicio de Volumetría para captura de múltiples imágenes en diferentes planos Z.

Este servicio coordina:
1. Detección de objeto
2. Z-scan para encontrar BPoF y límites Z_min/Z_max
3. Captura de X imágenes distribuidas en el rango Z
4. Generación de JSON con metadatos
"""

import logging
import os
import json
import numpy as np
from datetime import datetime
from typing import Optional, List, Dict, Tuple, Callable
from dataclasses import dataclass, asdict
from PyQt5.QtCore import QObject, pyqtSignal

logger = logging.getLogger('MotorControl_L206')


@dataclass
class VolumetryImage:
    """Información de una imagen de volumetría."""
    filename: str
    z_position: float
    score: float
    is_bpof: bool
    index: int


@dataclass
class VolumetryResult:
    """Resultado completo de una captura de volumetría."""
    success: bool
    object_id: int
    timestamp: str
    folder_path: str
    
    # Detección
    centroid_x: int
    centroid_y: int
    area_pixels: int
    
    # Z-scan
    z_start: float
    z_end: float
    z_step: float
    n_steps: int
    
    # Focus analysis
    z_min_detected: float
    z_max_detected: float
    z_bpof: float
    score_bpof: float
    score_threshold: float
    
    # Imágenes capturadas
    images: List[VolumetryImage]
    
    # Configuración de cámara
    exposure_ms: float
    bit_depth: int
    img_format: str
    
    # Scores del Z-scan completo (opcional)
    z_scan_data: Optional[List[Dict]] = None


class VolumetryService(QObject):
    """
    Servicio para captura volumétrica de objetos.
    
    Signals:
        volumetry_started: Emitido al iniciar volumetría
        volumetry_progress: (current_image, total_images, z_position)
        volumetry_image_captured: (z_position, score, filepath)
        volumetry_complete: (VolumetryResult)
        volumetry_error: (error_message)
    """
    
    volumetry_started = pyqtSignal(int)  # n_images
    volumetry_progress = pyqtSignal(int, int, float)  # current, total, z
    volumetry_image_captured = pyqtSignal(float, float, str)  # z, score, path
    volumetry_complete = pyqtSignal(object)  # VolumetryResult
    volumetry_error = pyqtSignal(str)
    
    def __init__(self,
                 get_current_frame: Optional[Callable[[], Optional[np.ndarray]]] = None,
                 smart_focus_scorer=None,
                 move_z: Optional[Callable[[float], None]] = None,
                 get_z_position: Optional[Callable[[], float]] = None,
                 capture_image: Optional[Callable[[str, dict], bool]] = None,
                 parent=None):
        """
        Args:
            get_current_frame: Función para obtener frame actual de la cámara
            smart_focus_scorer: Instancia de SmartFocusScorer para detección
            move_z: Función para mover el eje Z (relativo o absoluto)
            get_z_position: Función para obtener posición Z actual
            capture_image: Función para capturar y guardar imagen
        """
        super().__init__(parent)
        
        self._get_current_frame = get_current_frame
        self._smart_focus_scorer = smart_focus_scorer
        self._move_z = move_z
        self._get_z_position = get_z_position
        self._capture_image = capture_image
        
        self._running = False
        self._abort_requested = False
    
    def is_running(self) -> bool:
        return self._running
    
    def abort(self):
        """Solicita abortar la volumetría en curso."""
        self._abort_requested = True
        logger.info("[VolumetryService] Abort solicitado")
    
    def start_volumetry(self, config: dict) -> bool:
        """
        Inicia el proceso de volumetría.
        
        Args:
            config: Diccionario con configuración:
                - n_images: Número de imágenes a capturar
                - distribution: 'uniform' o 'centered'
                - include_bpof: Incluir imagen exacta en BPoF
                - save_json: Guardar JSON con metadatos
                - save_folder: Carpeta de destino
                - img_format: 'png', 'tiff', 'jpg'
                - use_16bit: True/False
                - z_range: Rango de búsqueda Z (µm)
                - z_step: Paso del Z-scan (µm)
                - min_area: Área mínima de objeto (pixels)
                - max_area: Área máxima de objeto (pixels)
                - score_threshold: Umbral de score para determinar límites
                - class_name: Nombre base para archivos
                - exposure_ms: Exposición de cámara
                
        Returns:
            True si se inició correctamente
        """
        if self._running:
            self.volumetry_error.emit("Volumetría ya en curso")
            return False
        
        # Validar dependencias
        if not all([self._get_current_frame, self._smart_focus_scorer, 
                    self._move_z, self._get_z_position]):
            self.volumetry_error.emit("Dependencias no configuradas")
            return False
        
        self._running = True
        self._abort_requested = False
        
        try:
            result = self._execute_volumetry(config)
            if result.success:
                self.volumetry_complete.emit(result)
            else:
                self.volumetry_error.emit("Volumetría falló")
        except Exception as e:
            logger.error(f"[VolumetryService] Error: {e}")
            self.volumetry_error.emit(str(e))
        finally:
            self._running = False
        
        return True
    
    def _execute_volumetry(self, config: dict) -> VolumetryResult:
        """Ejecuta el proceso completo de volumetría."""
        import cv2
        
        n_images = config.get('n_images', 10)
        distribution = config.get('distribution', 'uniform')
        include_bpof = config.get('include_bpof', True)
        save_json = config.get('save_json', True)
        save_folder = config.get('save_folder', '.')
        img_format = config.get('img_format', 'png')
        use_16bit = config.get('use_16bit', True)
        z_range = config.get('z_range', 100.0)
        z_step = config.get('z_step', 5.0)
        min_area = config.get('min_area', 5000)
        max_area = config.get('max_area', 50000)
        score_threshold = config.get('score_threshold', 0.3)
        class_name = config.get('class_name', 'volumetry')
        exposure_ms = config.get('exposure_ms', 50.0)
        
        self.volumetry_started.emit(n_images)
        logger.info(f"[VolumetryService] Iniciando volumetría: {n_images} imágenes")
        
        # 1. Obtener frame y detectar objeto
        frame = self._get_current_frame()
        if frame is None:
            raise ValueError("No hay frame disponible")
        
        # Convertir a uint8 para detección si es necesario
        if frame.dtype == np.uint16:
            frame_8bit = (frame / frame.max() * 255).astype(np.uint8) if frame.max() > 0 else frame.astype(np.uint8)
        else:
            frame_8bit = frame
        
        # Detectar objetos usando assess_image (método correcto de SmartFocusScorer)
        import cv2
        if len(frame_8bit.shape) == 2:
            frame_bgr = cv2.cvtColor(frame_8bit, cv2.COLOR_GRAY2BGR)
        else:
            frame_bgr = frame_8bit
        
        result = self._smart_focus_scorer.assess_image(frame_bgr)
        all_objects = result.objects if result.objects else []
        
        # Filtrar por área
        filtered_objects = [obj for obj in all_objects 
                          if min_area <= obj.area <= max_area]
        
        if not filtered_objects:
            raise ValueError(f"No se detectaron objetos en rango de área [{min_area}, {max_area}] "
                           f"(detectados: {len(all_objects)} totales)")
        
        # Seleccionar objeto más grande
        target_object = max(filtered_objects, key=lambda o: o.area)
        logger.info(f"[VolumetryService] Objeto seleccionado: área={target_object.area}, "
                   f"centroid=({target_object.centroid[0]}, {target_object.centroid[1]})")
        
        # 2. Obtener posición Z actual (debe ser el BPoF del autofoco previo)
        # La volumetría ASUME que el usuario ya hizo autofoco antes
        z_bpof = self._get_z_position()
        
        if self._abort_requested:
            raise ValueError("Volumetría abortada por usuario")
        
        # Verificar que estamos en una posición razonable (no en los extremos)
        if z_bpof < 5.0 or z_bpof > 97.0:
            logger.warning(f"[VolumetryService] Posición Z={z_bpof:.1f}µm parece estar en un extremo. "
                          "¿Ejecutaste autofoco primero?")
        
        logger.info(f"[VolumetryService] ✅ Usando posición actual como BPoF: Z={z_bpof:.1f}µm")
        
        # 3. Calcular rango de captura CENTRADO en el BPoF
        # Las imágenes se capturan en BPoF ± z_range (parámetro del usuario)
        z_min_capture = max(0.0, z_bpof - z_range)
        z_max_capture = min(102.0, z_bpof + z_range)
        
        logger.info(f"[VolumetryService] Rango de captura: [{z_min_capture:.1f}, {z_max_capture:.1f}]µm "
                   f"(BPoF ± {z_range}µm)")
        
        # 4. Calcular posiciones Z para captura
        z_positions = self._calculate_z_positions(
            z_min_capture, z_max_capture, z_bpof, n_images, distribution, include_bpof
        )
        
        # 4. Crear carpeta de salida
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        folder_name = f"{class_name}_{timestamp}"
        output_folder = os.path.join(save_folder, folder_name)
        os.makedirs(output_folder, exist_ok=True)
        
        # 5. Capturar imágenes en cada posición Z (OPTIMIZADO)
        captured_images = []
        
        # Calcular score solo 1 vez al inicio (no en cada Z)
        frame_initial = self._get_current_frame()
        if frame_initial is None:
            raise ValueError("No hay frame disponible para captura")
        
        if frame_initial.dtype == np.uint16:
            frame_8bit_initial = (frame_initial / frame_initial.max() * 255).astype(np.uint8) if frame_initial.max() > 0 else frame_initial.astype(np.uint8)
        else:
            frame_8bit_initial = frame_initial
        
        # Score inicial del objeto
        initial_score = self._get_roi_score(frame_8bit_initial, target_object)
        
        logger.info(f"[VolumetryService] Iniciando captura de {len(z_positions)} imágenes (score inicial: {initial_score:.2f})")
        
        for i, z_pos in enumerate(z_positions):
            if self._abort_requested:
                raise ValueError("Volumetría abortada por usuario")
            
            # Mover a posición Z (sin sleep adicional, el move_z ya tiene su propio settle)
            self._move_z(z_pos)
            
            # Determinar si es BPoF
            is_bpof = abs(z_pos - z_bpof) < z_step / 2
            
            # Generar nombre de archivo ÚNICO usando índice secuencial
            z_sign = "+" if z_pos >= 0 else "-"
            bpof_suffix = "_BPoF" if is_bpof else ""
            # Usar índice para garantizar unicidad + 3 decimales de Z
            filename = f"{class_name}_{i:04d}_z{z_sign}{abs(z_pos):06.3f}um{bpof_suffix}.{img_format}"
            filepath = os.path.join(output_folder, filename)
            
            # Guardar imagen usando el callback proporcionado
            if self._capture_image is not None:
                # Usar método de captura del programa
                capture_config = {
                    'img_format': img_format,
                    'use_16bit': use_16bit
                }
                success = self._capture_image(filepath, capture_config)
            else:
                # Fallback: guardar directamente (no recomendado)
                logger.warning("[VolumetryService] capture_image callback no configurado, usando fallback")
                if use_16bit and img_format in ['png', 'tiff']:
                    success = cv2.imwrite(filepath, frame)
                else:
                    if frame.dtype == np.uint16:
                        frame_save = (frame / frame.max() * 255).astype(np.uint8) if frame.max() > 0 else frame.astype(np.uint8)
                    else:
                        frame_save = frame
                    
                    if img_format == 'jpg':
                        success = cv2.imwrite(filepath, frame_save, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    else:
                        success = cv2.imwrite(filepath, frame_save, [cv2.IMWRITE_PNG_COMPRESSION, 6])
            
            if success:
                img_info = VolumetryImage(
                    filename=filename,
                    z_position=z_pos,
                    score=initial_score,  # Usar score inicial (no recalcular)
                    is_bpof=is_bpof,
                    index=i
                )
                captured_images.append(img_info)
                
                self.volumetry_progress.emit(i + 1, n_images, z_pos)
                self.volumetry_image_captured.emit(z_pos, initial_score, filepath)
                
                # Log cada 10 imágenes para no saturar
                if (i + 1) % 10 == 0 or (i + 1) == n_images:
                    logger.info(f"[VolumetryService] Progreso: {i + 1}/{n_images} imágenes capturadas")
        
        # 6. Volver a posición BPoF
        self._move_z(z_bpof)
        
        # 7. Crear resultado
        result = VolumetryResult(
            success=len(captured_images) > 0,
            object_id=1,
            timestamp=timestamp,
            folder_path=output_folder,
            centroid_x=int(target_object.centroid[0]),
            centroid_y=int(target_object.centroid[1]),
            area_pixels=int(target_object.area),
            z_start=z_min_capture,
            z_end=z_max_capture,
            z_step=z_step,
            n_steps=n_images,
            z_min_detected=z_min_capture,
            z_max_detected=z_max_capture,
            z_bpof=z_bpof,
            score_bpof=0.0,  # No hacemos Z-scan, usamos BPoF del autofoco
            score_threshold=score_threshold,
            images=captured_images,
            exposure_ms=exposure_ms,
            bit_depth=16 if use_16bit else 8,
            img_format=img_format,
            z_scan_data=None  # No guardamos datos de Z-scan
        )
        
        # 8. Guardar JSON
        if save_json:
            json_path = os.path.join(output_folder, "metadata.json")
            self._save_json(result, json_path)
            logger.info(f"[VolumetryService] JSON guardado: {json_path}")
        
        return result
    
    def _perform_z_scan(self, target_object, z_range: float, z_step: float, 
                        score_threshold: float) -> dict:
        """
        Realiza Z-scan COMPLETO para encontrar BPoF.
        
        IMPORTANTE: Escanea TODO el rango del C-Focus (0-102µm) para encontrar
        el mejor punto de enfoque, NO desde la posición actual.
        
        Returns:
            Dict con z_bpof, z_min, z_max, score_bpof, scan_data
        """
        import time
        
        z_start = self._get_z_position()
        logger.info(f"[VolumetryService] Posición inicial: Z={z_start:.2f}µm")
        logger.info(f"[VolumetryService] Iniciando Z-scan COMPLETO para encontrar BPoF...")
        scan_data = []
        
        # ESCANEO COMPLETO del rango del C-Focus (0 a 102µm)
        # Usar paso grueso para encontrar BPoF rápidamente
        z_step_coarse = max(z_step, 2.0)  # Mínimo 2µm para escaneo inicial
        z_min_scan = 0.0
        z_max_scan = 102.0
        z_positions = np.arange(z_min_scan, z_max_scan + z_step_coarse, z_step_coarse)
        
        logger.info(f"[VolumetryService] Z-scan: {len(z_positions)} posiciones, "
                   f"rango [{z_min_scan:.1f}, {z_max_scan:.1f}]µm, paso {z_step_coarse}µm")
        
        best_z = z_start
        best_score = 0.0
        
        for i, z_pos in enumerate(z_positions):
            if self._abort_requested:
                logger.info("[VolumetryService] Z-scan abortado")
                break
            
            # Log cada 10 posiciones o al inicio
            if i % 10 == 0:
                logger.debug(f"[VolumetryService] Z-scan progreso: {i+1}/{len(z_positions)}, Z={z_pos:.2f}µm")
            
            self._move_z(z_pos)
            time.sleep(0.05)  # Estabilización
            
            frame = self._get_current_frame()
            if frame is None:
                logger.warning(f"[VolumetryService] Frame None en Z={z_pos:.2f}µm")
                continue
            
            # Convertir para scoring
            if frame.dtype == np.uint16:
                frame_8bit = (frame / frame.max() * 255).astype(np.uint8) if frame.max() > 0 else frame.astype(np.uint8)
            else:
                frame_8bit = frame
            
            score = self._get_roi_score(frame_8bit, target_object)
            
            scan_data.append({
                'z': float(z_pos),
                'score': float(score)
            })
            
            if score > best_score:
                best_score = score
                best_z = z_pos
        
        logger.info(f"[VolumetryService] Z-scan completado: {len(scan_data)} muestras, "
                   f"mejor Z={best_z:.2f}µm, score={best_score:.3f}")
        
        # Determinar límites donde score > threshold * best_score
        threshold_value = score_threshold * best_score
        valid_positions = [d['z'] for d in scan_data if d['score'] >= threshold_value]
        
        if valid_positions:
            z_min = min(valid_positions)
            z_max = max(valid_positions)
        else:
            z_min = best_z - z_range / 2
            z_max = best_z + z_range / 2
        
        return {
            'z_bpof': best_z,
            'score_bpof': best_score,
            'z_min': z_min,
            'z_max': z_max,
            'scan_data': scan_data
        }
    
    def _get_roi_score(self, frame: np.ndarray, obj) -> float:
        """Calcula el score de enfoque en el ROI del objeto."""
        try:
            # Obtener bounding box del objeto
            x, y = int(obj.centroid[0]), int(obj.centroid[1])
            
            # Estimar tamaño del ROI basado en área
            roi_size = int(np.sqrt(obj.area) * 1.5)
            half_size = roi_size // 2
            
            # Extraer ROI
            h, w = frame.shape[:2]
            x1 = max(0, x - half_size)
            y1 = max(0, y - half_size)
            x2 = min(w, x + half_size)
            y2 = min(h, y + half_size)
            
            roi = frame[y1:y2, x1:x2]
            
            if roi.size == 0:
                return 0.0
            
            # Calcular score usando varianza del Laplaciano
            import cv2
            if len(roi.shape) == 3:
                roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            
            laplacian = cv2.Laplacian(roi, cv2.CV_64F)
            score = laplacian.var()
            
            # Normalizar score (típicamente entre 0 y 1)
            normalized_score = min(1.0, score / 1000.0)
            
            return normalized_score
            
        except Exception as e:
            logger.warning(f"[VolumetryService] Error calculando score: {e}")
            return 0.0
    
    def _calculate_z_positions(self, z_min: float, z_max: float, z_bpof: float,
                               n_images: int, distribution: str, 
                               include_bpof: bool) -> List[float]:
        """
        Calcula las posiciones Z para captura.
        
        Args:
            z_min: Límite inferior Z
            z_max: Límite superior Z
            z_bpof: Posición del BPoF
            n_images: Número de imágenes
            distribution: 'uniform' o 'centered'
            include_bpof: Si incluir BPoF exacto
            
        Returns:
            Lista de posiciones Z ordenadas
        """
        if distribution == 'uniform' or distribution.startswith('Uniforme'):
            # Distribución uniforme
            positions = np.linspace(z_min, z_max, n_images).tolist()
        else:
            # Distribución centrada (más densidad cerca del BPoF)
            # Usar distribución no lineal
            t = np.linspace(-1, 1, n_images)
            # Función que concentra valores cerca de 0
            t_transformed = np.sign(t) * np.abs(t) ** 0.5
            # Mapear a rango Z
            z_center = (z_min + z_max) / 2
            z_half_range = (z_max - z_min) / 2
            positions = (z_center + t_transformed * z_half_range).tolist()
        
        # Asegurar que BPoF esté incluido
        if include_bpof:
            # Encontrar posición más cercana y reemplazarla con BPoF exacto
            closest_idx = min(range(len(positions)), 
                            key=lambda i: abs(positions[i] - z_bpof))
            positions[closest_idx] = z_bpof
        
        return sorted(positions)
    
    def _save_json(self, result: VolumetryResult, filepath: str):
        """Guarda el resultado como JSON simplificado (solo imágenes capturadas)."""
        data = {
            'timestamp': result.timestamp,
            'z_bpof_um': round(float(result.z_bpof), 2),
            'n_images': len(result.images),
            'images': [
                {
                    'filename': img.filename,
                    'z_um': round(float(img.z_position), 2),
                    'score': round(float(img.score), 4),
                    'is_bpof': bool(img.is_bpof)
                }
                for img in result.images
            ]
        }
        
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

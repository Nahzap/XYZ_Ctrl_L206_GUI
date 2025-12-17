"""
Camera Service - Servicio de Cámara Thorlabs
===========================================

Orquesta CameraWorker en un QThread separado y expone señales
para que la UI (CameraTab) no tenga lógica de hardware.

REFACTORIZACIÓN 2025-12-17:
- Expandido con lógica de captura, detección y configuración
- Toda la lógica de cámara movida desde CameraTab

Autor: Sistema de Control L206
"""

import os
import logging
from datetime import datetime
from typing import Optional, List, Dict, Any

import numpy as np
import cv2

from PyQt5.QtCore import QObject, pyqtSignal

from hardware.camera.camera_worker import CameraWorker
from config.hardware_availability import THORLABS_AVAILABLE, Thorlabs


logger = logging.getLogger('MotorControl_L206')


class CameraService(QObject):
    """Servicio de cámara que encapsula CameraWorker.

    Expone señales de alto nivel para que la UI se mantenga liviana.
    Contiene toda la lógica de cámara: conexión, captura, configuración.

    Signals:
        status_changed: Mensajes de estado para logging en UI.
        connected: Resultado de conexión (success, info).
        disconnected: Emite cuando la cámara se desconecta.
        frame_ready: Nuevo frame disponible (QImage, raw_frame).
        capture_completed: Captura exitosa (filepath).
        cameras_detected: Lista de cámaras detectadas.
        error_occurred: Error durante operación.
    """

    # Señales
    status_changed = pyqtSignal(str)
    connected = pyqtSignal(bool, str)  # success, info
    disconnected = pyqtSignal()
    frame_ready = pyqtSignal(object, object)  # QImage, raw_frame
    capture_completed = pyqtSignal(str)  # filepath
    cameras_detected = pyqtSignal(list)  # lista de cámaras
    error_occurred = pyqtSignal(str)  # mensaje de error

    def __init__(self, parent=None, thorlabs_available: bool = False):
        super().__init__(parent)
        self.worker: Optional[CameraWorker] = None
        self._thorlabs_available = thorlabs_available
        self._pending_capture = False  # Flag para captura después de autofoco

    def set_thorlabs_available(self, available: bool) -> None:
        """Configura si el SDK de Thorlabs está disponible."""
        self._thorlabs_available = available

    def connect_camera(self, thorlabs_available: Optional[bool] = None, buffer_size: int = 2) -> None:
        """Conecta con la cámara Thorlabs usando CameraWorker.

        Args:
            thorlabs_available: Si pylablib/Thorlabs está disponible.
            buffer_size: Tamaño de buffer para adquisición.
        """
        if thorlabs_available is not None:
            self._thorlabs_available = thorlabs_available

        if not self._thorlabs_available:
            msg = "❌ Error: pylablib no está disponible"
            self.status_changed.emit(msg)
            logger.warning(f"[CameraService] {msg}")
            self.connected.emit(False, "pylablib no disponible")
            return

        if self.worker is None:
            logger.info("[CameraService] Creando CameraWorker...")
            self.worker = CameraWorker()
            self.worker.connection_success.connect(self._on_worker_connected)
            self.worker.new_frame_ready.connect(self._on_new_frame)
            self.worker.status_update.connect(self.status_changed.emit)

        # Configurar buffer inicial
        try:
            self.worker.buffer_size = int(buffer_size)
        except Exception:
            self.worker.buffer_size = 2
        logger.info(f"[CameraService] Buffer inicial: {self.worker.buffer_size} frames")

        self.status_changed.emit("🔌 Conectando cámara Thorlabs...")
        logger.info("[CameraService] Conectando cámara Thorlabs...")
        self.worker.connect_camera()

    def disconnect_camera(self) -> None:
        """Desconecta la cámara y libera el worker."""
        if self.worker is None:
            return

        logger.info("[CameraService] Desconectando cámara...")
        try:
            self.worker.disconnect_camera()
        except Exception as e:
            logger.error(f"[CameraService] Error al desconectar cámara: {e}")
        finally:
            self.worker = None
            self.disconnected.emit()
            self.status_changed.emit("🔌 Cámara desconectada")
            logger.info("[CameraService] Cámara desconectada")

    def start_live(self, exposure_s: float, fps: int, buffer_size: int) -> None:
        """Inicia vista en vivo configurando el worker."""
        if self.worker is None:
            self.status_changed.emit("❌ Error: Cámara no conectada")
            logger.warning("[CameraService] start_live llamado sin cámara conectada")
            return

        # Configurar parámetros en el worker
        self.worker.exposure = exposure_s
        self.worker.fps = fps
        self.worker.buffer_size = buffer_size

        self.status_changed.emit(
            f"▶️ Iniciando vista en vivo (exp={exposure_s}s, fps={fps}, buffer={buffer_size})"
        )
        logger.info(
            f"[CameraService] Iniciando live view: exp={exposure_s}s, fps={fps}, buffer={buffer_size}"
        )

        # Ejecutar run() de CameraWorker en su propio thread
        self.worker.start()

    def stop_live(self) -> None:
        """Detiene la vista en vivo si el worker está activo."""
        if self.worker is None:
            return

        try:
            self.worker.stop_live_view()
            self.status_changed.emit("⏹️ Vista en vivo detenida")
            logger.info("[CameraService] Vista en vivo detenida")
        except Exception as e:
            logger.error(f"[CameraService] Error al detener live view: {e}")

    # ------------------------------------------------------------------
    # Slots internos
    # ------------------------------------------------------------------
    def _on_worker_connected(self, success: bool, info: str) -> None:
        """Reemite el resultado de conexión a la UI."""
        if success:
            logger.info(f"[CameraService] Cámara conectada: {info}")
        else:
            logger.error(f"[CameraService] Fallo de conexión: {info}")
        self.connected.emit(success, info)

    def _on_new_frame(self, q_image, raw_frame) -> None:
        """Reemite el frame nuevo para que la UI lo consuma."""
        self.frame_ready.emit(q_image, raw_frame)

    # ==================================================================
    # DETECCIÓN DE CÁMARAS
    # ==================================================================

    def detect_cameras(self) -> List[str]:
        """Detecta cámaras Thorlabs conectadas.
        
        Returns:
            Lista de identificadores de cámaras encontradas.
        """
        if not self._thorlabs_available:
            self.status_changed.emit("❌ Error: pylablib no está instalado")
            self.error_occurred.emit("pylablib no está instalado")
            logger.warning("[CameraService] Intento de detectar cámara sin pylablib")
            return []
        
        self.status_changed.emit("🔍 Buscando cámaras Thorlabs...")
        logger.info("[CameraService] Detectando cámaras Thorlabs...")
        
        try:
            cameras = Thorlabs.list_cameras_tlcam()
            
            if not cameras:
                self.status_changed.emit("⚠️ No se encontraron cámaras")
                logger.warning("[CameraService] No se encontraron cámaras")
            else:
                self.status_changed.emit(f"✅ Encontradas {len(cameras)} cámara(s)")
                for i, cam in enumerate(cameras, 1):
                    self.status_changed.emit(f"   Cámara {i}: {cam}")
                logger.info(f"[CameraService] Detectadas {len(cameras)} cámaras Thorlabs")
            
            self.cameras_detected.emit(list(cameras) if cameras else [])
            return list(cameras) if cameras else []
            
        except Exception as e:
            self.status_changed.emit(f"❌ Error detectando: {e}")
            self.error_occurred.emit(f"Error detectando cámaras: {e}")
            logger.error(f"[CameraService] Error en detección: {e}")
            return []

    # ==================================================================
    # CONFIGURACIÓN DE PARÁMETROS
    # ==================================================================

    def apply_exposure(self, exposure_s: float) -> bool:
        """Aplica valor de exposición a la cámara.
        
        Args:
            exposure_s: Exposición en segundos.
            
        Returns:
            True si se aplicó correctamente.
        """
        if self.worker is None:
            self.error_occurred.emit("Cámara no conectada")
            return False
        
        try:
            self.worker.change_exposure(exposure_s)
            exposure_ms = exposure_s * 1000
            self.status_changed.emit(f"✅ Exposición configurada: {exposure_s}s ({exposure_ms:.1f}ms)")
            logger.info(f"[CameraService] Exposición: {exposure_s}s")
            return True
        except Exception as e:
            self.status_changed.emit(f"❌ Error aplicando exposición: {e}")
            self.error_occurred.emit(f"Error aplicando exposición: {e}")
            logger.error(f"[CameraService] Error aplicando exposición: {e}")
            return False

    def apply_fps(self, fps: int) -> bool:
        """Aplica valor de FPS a la cámara.
        
        Args:
            fps: Frames por segundo.
            
        Returns:
            True si se aplicó correctamente.
        """
        if self.worker is None:
            self.error_occurred.emit("Cámara no conectada")
            return False
        
        try:
            self.worker.change_fps(fps)
            self.status_changed.emit(f"✅ FPS configurado: {fps}")
            logger.info(f"[CameraService] FPS: {fps}")
            return True
        except Exception as e:
            self.status_changed.emit(f"❌ Error aplicando FPS: {e}")
            self.error_occurred.emit(f"Error aplicando FPS: {e}")
            logger.error(f"[CameraService] Error aplicando FPS: {e}")
            return False

    def apply_buffer(self, buffer_size: int) -> bool:
        """Aplica tamaño de buffer a la cámara.
        
        Args:
            buffer_size: Número de frames en buffer (1-10).
            
        Returns:
            True si se aplicó correctamente.
        """
        if self.worker is None:
            self.error_occurred.emit("Cámara no conectada")
            return False
        
        if buffer_size < 1 or buffer_size > 10:
            self.error_occurred.emit("Buffer debe estar entre 1 y 10")
            return False
        
        try:
            self.worker.change_buffer_size(buffer_size)
            self.status_changed.emit(f"✅ Buffer configurado: {buffer_size} frames")
            logger.info(f"[CameraService] Buffer: {buffer_size}")
            return True
        except Exception as e:
            self.status_changed.emit(f"❌ Error aplicando buffer: {e}")
            self.error_occurred.emit(f"Error aplicando buffer: {e}")
            logger.error(f"[CameraService] Error aplicando buffer: {e}")
            return False

    # ==================================================================
    # CAPTURA DE IMÁGENES
    # ==================================================================

    def capture_image(self, folder: str, img_format: str = 'png') -> Optional[str]:
        """Captura una imagen única y la guarda.
        
        Args:
            folder: Carpeta de destino.
            img_format: Formato de imagen ('png', 'tiff', 'jpg').
            
        Returns:
            Ruta del archivo guardado o None si falló.
        """
        if self.worker is None or self.worker.current_frame is None:
            self.status_changed.emit("❌ Error: No hay frame disponible")
            self.error_occurred.emit("No hay frame disponible para capturar")
            logger.warning("[CameraService] No hay frame en buffer para capturar")
            return None
        
        try:
            # Crear carpeta si no existe
            os.makedirs(folder, exist_ok=True)
            
            # Generar nombre de archivo
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(folder, f"captura_{timestamp}.{img_format}")
            
            frame = self.worker.current_frame.copy()
            frame_info = f"Original: {frame.shape}, dtype={frame.dtype}"
            
            # Normalizar frame uint16 para visualización correcta
            if frame.dtype == np.uint16:
                frame_min, frame_max = frame.min(), frame.max()
                
                if img_format == 'tiff':
                    # TIFF: mantener 16 bits original
                    cv2.imwrite(filename, frame)
                    self.status_changed.emit(f"   16-bit TIFF: rango [{frame_min}, {frame_max}]")
                else:
                    # PNG/JPG: normalizar a 8 bits
                    if frame_max > 0:
                        frame_norm = (frame / frame_max * 255).astype(np.uint8)
                    else:
                        frame_norm = np.zeros_like(frame, dtype=np.uint8)

                    if img_format == 'jpg':
                        cv2.imwrite(filename, frame_norm, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    else:  # png
                        cv2.imwrite(filename, frame_norm, [cv2.IMWRITE_PNG_COMPRESSION, 6])

                    self.status_changed.emit(f"   Normalizado: [{frame_min}, {frame_max}] → 8-bit")
            else:
                # Frame ya es uint8
                if img_format == 'jpg':
                    cv2.imwrite(filename, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                elif img_format == 'png':
                    cv2.imwrite(filename, frame, [cv2.IMWRITE_PNG_COMPRESSION, 6])
                else:
                    cv2.imwrite(filename, frame)
            
            self.status_changed.emit(f"📸 Imagen guardada: {filename}")
            self.status_changed.emit(f"   {frame_info}")
            self.capture_completed.emit(filename)
            logger.info(f"[CameraService] Captura guardada: {filename}")
            return filename
            
        except Exception as e:
            self.status_changed.emit(f"❌ Error capturando imagen: {e}")
            self.error_occurred.emit(f"Error capturando imagen: {e}")
            logger.error(f"[CameraService] Error en captura: {e}")
            return None

    def capture_microscopy_image(self, config: Dict[str, Any], image_index: int) -> bool:
        """Captura una imagen para microscopía automatizada.
        
        Lógica de canales:
        - 1 canal seleccionado: Guarda como GRAYSCALE puro (1 canal)
        - 2-3 canales seleccionados: Guarda como BGR (3 canales)
        
        Args:
            config: Configuración de microscopía.
            image_index: Índice de la imagen (0 a n_points-1).
            
        Returns:
            True si la captura fue exitosa.
        """
        if self.worker is None or self.worker.current_frame is None:
            self.status_changed.emit(f"❌ Error: No hay frame disponible para imagen {image_index}")
            return False
        
        try:
            # Obtener frame actual
            frame = self.worker.current_frame.copy()
            h_orig, w_orig = frame.shape[:2]
            original_dtype = frame.dtype
            
            # Redimensionar si es necesario (mantener dtype original)
            target_width = config.get('img_width', 1920)
            target_height = config.get('img_height', 1080)
            
            if w_orig != target_width or h_orig != target_height:
                frame = cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_LINEAR)
            
            # Procesar canales según selección del usuario
            channels = config.get('channels', {'R': False, 'G': True, 'B': False})
            selected_channels = [c for c in ['R', 'G', 'B'] if channels.get(c, False)]
            n_selected = len(selected_channels)
            
            # Lógica de canales
            if len(frame.shape) == 2:  # Frame grayscale original
                if n_selected == 1:
                    pass  # frame ya está en grayscale
                elif n_selected >= 2:
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
            
            elif len(frame.shape) == 3:  # Frame ya es color (BGR)
                if n_selected == 1:
                    channel_map = {'B': 0, 'G': 1, 'R': 2}
                    channel_idx = channel_map[selected_channels[0]]
                    frame = frame[:, :, channel_idx]
                else:
                    if n_selected < 3:
                        new_frame = np.zeros_like(frame)
                        if channels.get('B', False):
                            new_frame[:, :, 0] = frame[:, :, 0]
                        if channels.get('G', False):
                            new_frame[:, :, 1] = frame[:, :, 1]
                        if channels.get('R', False):
                            new_frame[:, :, 2] = frame[:, :, 2]
                        frame = new_frame
            
            # Generar nombre de archivo
            class_name = config.get('class_name', 'Imagen')
            save_folder = config.get('save_folder', '.')
            img_format = config.get('img_format', 'png').lower()
            use_16bit = config.get('use_16bit', True)  # Por defecto 16-bit
            
            # Determinar extensión y guardar según formato y profundidad de bits
            if img_format == 'tiff':
                filename = f"{class_name}_{image_index:05d}.tiff"
                filepath = os.path.join(save_folder, filename)
                
                if use_16bit:
                    # TIFF 16-bit: mantener uint16
                    success = cv2.imwrite(filepath, frame)
                    bits_str = "16-bit"
                else:
                    # TIFF 8-bit: convertir a uint8
                    if original_dtype == np.uint16:
                        if frame.max() > 0:
                            frame = (frame / frame.max() * 255).astype(np.uint8)
                        else:
                            frame = frame.astype(np.uint8)
                    success = cv2.imwrite(filepath, frame)
                    bits_str = "8-bit"
                    
            elif img_format == 'png':
                filename = f"{class_name}_{image_index:05d}.png"
                filepath = os.path.join(save_folder, filename)
                
                if use_16bit:
                    # PNG 16-bit: OpenCV soporta PNG 16-bit nativamente con uint16
                    # El frame ya está en uint16, cv2.imwrite lo guarda correctamente
                    success = cv2.imwrite(filepath, frame, [cv2.IMWRITE_PNG_COMPRESSION, 6])
                    bits_str = "16-bit"
                else:
                    # PNG 8-bit: convertir a uint8
                    if original_dtype == np.uint16:
                        if frame.max() > 0:
                            frame = (frame / frame.max() * 255).astype(np.uint8)
                        else:
                            frame = frame.astype(np.uint8)
                    success = cv2.imwrite(filepath, frame, [cv2.IMWRITE_PNG_COMPRESSION, 6])
                    bits_str = "8-bit"
                    
            else:  # jpg
                # JPG solo soporta 8-bit
                filename = f"{class_name}_{image_index:05d}.jpg"
                filepath = os.path.join(save_folder, filename)
                
                if original_dtype == np.uint16:
                    if frame.max() > 0:
                        frame = (frame / frame.max() * 255).astype(np.uint8)
                    else:
                        frame = frame.astype(np.uint8)
                success = cv2.imwrite(filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                bits_str = "8-bit (JPG)"
            
            if not success:
                self.status_changed.emit(f"❌ Error: cv2.imwrite falló para {filename}")
                return False
            
            # Calcular tamaño del archivo
            file_size_kb = os.path.getsize(filepath) / 1024
            channels_str = ''.join(selected_channels)
            self.status_changed.emit(f"[{image_index+1}] {filename} ({bits_str}, {channels_str}, {file_size_kb:.0f} KB)")
            logger.info(f"[CameraService] Microscopía: {filepath} ({bits_str})")
            
            return True
            
        except Exception as e:
            self.status_changed.emit(f"❌ Error capturando imagen {image_index}: {e}")
            logger.error(f"[CameraService] Error en capture_microscopy_image: {e}")
            return False

    # ==================================================================
    # PROPIEDADES Y UTILIDADES
    # ==================================================================

    @property
    def is_connected(self) -> bool:
        """Retorna True si la cámara está conectada."""
        return self.worker is not None

    @property
    def current_frame(self) -> Optional[np.ndarray]:
        """Retorna el frame actual del buffer."""
        if self.worker is not None:
            return self.worker.current_frame
        return None

    def get_frame_info(self) -> Dict[str, Any]:
        """Retorna información del frame actual."""
        if self.worker is None or self.worker.current_frame is None:
            return {'available': False}
        
        frame = self.worker.current_frame
        return {
            'available': True,
            'shape': frame.shape,
            'dtype': str(frame.dtype),
            'min': int(frame.min()),
            'max': int(frame.max())
        }

"""
Camera Service - Servicio de Cámara (Multi-Cámara)
===================================================

Orquesta CameraWorker en un QThread separado y expone señales
para que la UI (CameraTab) no tenga lógica de hardware.

REFACTORIZACIÓN 2025-12-17:
- Expandido con lógica de captura, detección y configuración
- Toda la lógica de cámara movida desde CameraTab

REFACTORIZACIÓN 2026-03-05:
- Soporte multi-cámara (Thorlabs, Basler)
- Usa CameraWorkerFactory para creación automática
- Detección automática de hardware disponible

Autor: Sistema de Control L206
"""

import os
import logging
from datetime import datetime
from typing import Optional, List, Dict, Any, Tuple

import numpy as np
import cv2

from utils.microscopy_filename import build_single_capture_filename
from core.utils.image_io import safe_imwrite

import time

from PyQt5.QtCore import QObject, pyqtSignal, Qt, QTimer

from hardware.camera import CameraWorkerFactory, BaseCameraWorker
from hardware.camera.scientific_config import (
    ACA2500_14UC,
    read_camera_roi_wh,
    resolve_camera_resolution,
)
from config.hardware_availability import THORLABS_AVAILABLE, BASLER_AVAILABLE
import traceback


logger = logging.getLogger('MotorControl_L206')


class CameraService(QObject):
    """Servicio de cámara multi-hardware que encapsula CameraWorker.

    Expone señales de alto nivel para que la UI se mantenga liviana.
    Contiene toda la lógica de cámara: conexión, captura, configuración.
    
    Soporta múltiples tipos de cámara mediante factory pattern:
    - Thorlabs (vía pylablib)
    - Basler (vía pypylon)

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

    def __init__(self, parent=None, camera_type: str = "auto"):
        """
        Inicializa el servicio de cámara.
        
        Args:
            parent: Parent QObject
            camera_type: Tipo de cámara ("auto", "thorlabs", "basler")
                - "auto": Detecta automáticamente (prioridad: Thorlabs → Basler)
                - "thorlabs": Fuerza uso de Thorlabs
                - "basler": Fuerza uso de Basler
        """
        super().__init__(parent)
        self.worker: Optional[BaseCameraWorker] = None
        self._camera_type = camera_type
        self._pending_capture = False  # Flag para captura después de autofoco
        self._frames_received = 0
        self._frames_emitted = 0
        self._frames_dropped = 0
        self._last_frame_log_time = 0.0
        self._resolution_logged = False
        # Coalesce: solo el último frame llega a la UI (sin cola de latencia)
        self._pending_q_image = None
        self._pending_raw_frame = None
        self._flush_scheduled = False
        
        logger.info(f"[CameraService] Inicializado con camera_type='{camera_type}'")

    def set_camera_type(self, camera_type: str) -> None:
        """
        Configura el tipo de cámara a usar.
        
        Args:
            camera_type: "auto", "thorlabs", o "basler"
        """
        self._camera_type = camera_type
        logger.info(f"[CameraService] Tipo de cámara configurado: '{camera_type}'")

    def connect_camera(self, buffer_size: int = 2, camera_type: Optional[str] = None) -> None:
        """Conecta con la cámara usando CameraWorkerFactory.

        Args:
            buffer_size: Tamaño de buffer para adquisición.
            camera_type: Tipo de cámara (opcional, usa self._camera_type si None)
        """
        # Permitir override de tipo
        if camera_type is not None:
            self._camera_type = camera_type
        
        # NUEVA LÓGICA: Detectar cámaras físicas primero
        physical_cameras = self.detect_cameras()
        if not physical_cameras:
            msg = "❌ Error: No se detectaron cámaras físicas conectadas"
            self.status_changed.emit(msg)
            logger.error(f"[CameraService] {msg}")
            self.connected.emit(False, "No hay cámaras físicas")
            return
        
        # Si es auto, usar el tipo de la primera cámara detectada
        if self._camera_type == "auto":
            self._camera_type = physical_cameras[0]['type']
            logger.info(f"[CameraService] Auto-detección: usando primera cámara '{self._camera_type}'")
            self.status_changed.emit(f"Auto-detección: conectando a {physical_cameras[0]['name']}")

        if self.worker is None:
            logger.info(f"[CameraService] Creando worker para '{self._camera_type}'...")
            
            # Factory crea el worker apropiado
            self.worker = CameraWorkerFactory.create_worker(self._camera_type)
            
            if self.worker is None:
                msg = f"❌ Error: No se pudo crear worker para '{self._camera_type}'"
                self.status_changed.emit(msg)
                logger.error(f"[CameraService] {msg}")
                self.connected.emit(False, f"Worker creation failed: {self._camera_type}")
                return
            
            # Conectar señales (igual para todos los workers gracias a BaseCameraWorker)
            self.worker.connection_success.connect(self._on_worker_connected)
            self.worker.new_frame_ready.connect(
                self._on_new_frame, Qt.QueuedConnection
            )
            self.worker.status_update.connect(self.status_changed.emit)
            
            logger.info(f"[CameraService] Worker creado: {self.worker.__class__.__name__}")

        # Configurar buffer inicial
        try:
            self.worker.buffer_size = int(buffer_size)
        except Exception:
            self.worker.buffer_size = 2
        logger.info(f"[CameraService] Buffer inicial: {self.worker.buffer_size} frames")

        # Worker emite status_update que está conectado a status_changed
        logger.info(f"[CameraService] Conectando cámara ({self.worker.get_camera_type()})...")
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
            # CameraWorker ya emite "Camara cerrada." via status_update
            logger.info("[CameraService] Cámara desconectada")

    def start_live(self, exposure_s: float, fps: int, buffer_size: int) -> None:
        """Inicia vista en vivo configurando el worker."""
        if self.worker is None:
            self.status_changed.emit("❌ Error: Cámara no conectada")
            logger.warning("[CameraService] start_live llamado sin cámara conectada")
            return

        if self.worker.isRunning():
            logger.warning("[CameraService] start_live: worker ya en ejecución, ignorando")
            return

        # Configurar parámetros en el worker
        self.worker.exposure = exposure_s
        self.worker.fps = fps
        self.worker.buffer_size = buffer_size
        self._frames_received = 0
        self._frames_emitted = 0
        self._frames_dropped = 0
        self._pending_q_image = None
        self._pending_raw_frame = None
        self._flush_scheduled = False

        logger.info(
            "[CameraService] Iniciando live view: exp=%ss fps=%d buffer=%d worker=%s",
            exposure_s,
            fps,
            buffer_size,
            self.worker.__class__.__name__,
        )
        self.worker.start()

    def stop_live(self) -> None:
        """Detiene la vista en vivo si el worker está activo."""
        if self.worker is None:
            return

        try:
            self.worker.stop_live_view()
            # CameraWorker emite "Vista en vivo detenida." via status_update
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
        """Guarda solo el frame más reciente; evita cola Qt de frames grandes."""
        self._frames_received += 1
        if self._pending_q_image is not None:
            self._frames_dropped += 1
        self._pending_q_image = q_image
        self._pending_raw_frame = raw_frame

        if self._frames_received == 1:
            shape = getattr(raw_frame, "shape", None)
            qw = q_image.width() if q_image is not None else 0
            qh = q_image.height() if q_image is not None else 0
            logger.info(
                "[CameraService] Primer frame del worker: qimage=%dx%d raw_shape=%s",
                qw,
                qh,
                shape,
            )

        if not self._flush_scheduled:
            self._flush_scheduled = True
            QTimer.singleShot(0, self._flush_pending_frame)

    def _flush_pending_frame(self) -> None:
        """Emite el último frame pendiente hacia la UI."""
        self._flush_scheduled = False
        q_image = self._pending_q_image
        raw_frame = self._pending_raw_frame
        self._pending_q_image = None
        self._pending_raw_frame = None
        if q_image is None:
            return

        now = time.perf_counter()
        if now - self._last_frame_log_time >= 5.0:
            logger.info(
                "[CameraService] Frames worker->UI: recibidos=%d emitidos=%d dropped=%d",
                self._frames_received,
                self._frames_emitted,
                self._frames_dropped,
            )
            self._last_frame_log_time = now

        self._frames_emitted += 1
        # Sync drop count into worker metrics si existe
        metrics = getattr(self.worker, "live_metrics", None) if self.worker else None
        if metrics is not None:
            metrics.frames_dropped_coalesce = int(self._frames_dropped)
            metrics.frames_emitted_ui = int(self._frames_emitted)
        self.frame_ready.emit(q_image, raw_frame)

    def set_preview_enabled(self, enabled: bool) -> None:
        """Activa/desactiva construcción de QImage preview en el worker."""
        if self.worker is None:
            return
        setter = getattr(self.worker, "set_preview_enabled", None)
        if callable(setter):
            setter(bool(enabled))
            logger.info("[CameraService] preview_enabled=%s", bool(enabled))

    def get_live_metrics(self) -> Optional[dict]:
        """Snapshot de métricas live (latencia/memoria) o None."""
        if self.worker is None:
            return None
        metrics = getattr(self.worker, "live_metrics", None)
        if metrics is None or not hasattr(metrics, "snapshot"):
            return {
                "frames_received": self._frames_received,
                "frames_emitted": self._frames_emitted,
                "frames_dropped": self._frames_dropped,
            }
        snap = metrics.snapshot()
        snap["frames_received_service"] = self._frames_received
        snap["frames_dropped_coalesce"] = self._frames_dropped
        snap["frames_emitted_ui"] = self._frames_emitted
        return snap

    def acquire_scientific_frame(self, timeout_s: float = 2.0):
        """
        ÚNICA vía de adquisición de imagen del CMOS para ciencia/guardado.

        Delega a ``worker.acquire_scientific_frame`` (ScientificFrame).
        """
        if self.worker is None:
            raise RuntimeError("[CameraService] Sin worker de cámara")
        acquire = getattr(self.worker, "acquire_scientific_frame", None)
        if not callable(acquire):
            raise RuntimeError(
                "[CameraService] El worker no implementa acquire_scientific_frame"
            )
        return acquire(timeout_s=float(timeout_s))

    # ==================================================================
    # DETECCIÓN DE CÁMARAS
    # ==================================================================

    def detect_cameras(self) -> List[Dict[str, str]]:
        """Detecta cámaras FÍSICAS disponibles (no solo SDKs).
        
        Returns:
            Lista de diccionarios con info de cámaras: [{'type': 'basler', 'name': '...', 'serial': '...'}, ...]
        """
        self.status_changed.emit("🔍 Buscando cámaras físicas conectadas...")
        logger.info("[CameraService] Detectando cámaras físicas...")
        
        cameras_found = []
        
        try:
            # 1. Detectar cámaras Thorlabs físicas
            if THORLABS_AVAILABLE:
                try:
                    from config.hardware_availability import Thorlabs
                    thorlabs_cameras = Thorlabs.list_cameras_tlcam()
                    
                    if thorlabs_cameras:
                        for serial in thorlabs_cameras:
                            cameras_found.append({
                                'type': 'thorlabs',
                                'serial': serial,
                                'name': f'Thorlabs S/N:{serial}'
                            })
                            self.status_changed.emit(f"   ✅ Thorlabs encontrada: S/N {serial}")
                            logger.info(f"[CameraService] Thorlabs detectada: {serial}")
                except Exception as e:
                    logger.debug(f"[CameraService] Error detectando Thorlabs: {e}")
            
            # 2. Detectar cámaras Basler físicas
            if BASLER_AVAILABLE:
                try:
                    from config.hardware_availability import pylon
                    tlFactory = pylon.TlFactory.GetInstance()
                    devices = tlFactory.EnumerateDevices()
                    
                    if devices:
                        for device in devices:
                            model = device.GetModelName()
                            serial = device.GetSerialNumber()
                            cameras_found.append({
                                'type': 'basler',
                                'serial': serial,
                                'name': f'{model} S/N:{serial}'
                            })
                            self.status_changed.emit(f"   ✅ Basler encontrada: {model} S/N {serial}")
                            logger.info(f"[CameraService] Basler detectada: {model} {serial}")
                except Exception as e:
                    logger.debug(f"[CameraService] Error detectando Basler: {e}")
            
            # Resultado final
            if not cameras_found:
                self.status_changed.emit("⚠️ No se encontraron cámaras físicas conectadas")
                logger.warning("[CameraService] No se detectaron cámaras físicas")
                self.cameras_detected.emit([])
                return []
            else:
                self.status_changed.emit(f"✅ Encontradas {len(cameras_found)} cámara(s)")
                logger.info(f"[CameraService] Total detectadas: {len(cameras_found)} cámaras")
                self.cameras_detected.emit(cameras_found)
                return cameras_found
            
        except Exception as e:
            self.status_changed.emit(f"❌ Error detectando: {e}")
            self.error_occurred.emit(f"Error detectando cámaras: {e}")
            logger.error(f"[CameraService] Error en detección: {e}\n{traceback.format_exc()}")
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
            # CameraWorker emite mensaje via status_update
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
            # CameraWorker emite mensaje via status_update
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
            # CameraWorker emite mensaje via status_update
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
            
            from hardware.camera.scientific_image import (
                image16_to_u8_preview,
                save_scientific_image,
            )

            sci = self.acquire_scientific_frame(timeout_s=2.0)
            use_16bit = bool(img_format.lower() in ("tiff", "tif", "png"))
            if img_format.lower() in ("jpg", "jpeg"):
                use_16bit = False

            if use_16bit:
                ok = save_scientific_image(
                    filename,
                    sci.image16,
                    already_prepared=True,
                )
                frame_info = (
                    f"shape={sci.image16.shape}, dtype={sci.image16.dtype}, "
                    f"source={sci.pipeline_id}, packed_as=16"
                )
            else:
                frame8 = image16_to_u8_preview(sci.image16)
                if img_format.lower() in ("jpg", "jpeg"):
                    ok = safe_imwrite(
                        filename, frame8, [cv2.IMWRITE_JPEG_QUALITY, 95]
                    )
                else:
                    ok = safe_imwrite(filename, frame8)
                frame_info = (
                    f"shape={frame8.shape}, dtype={frame8.dtype}, "
                    f"source={sci.pipeline_id}_u8, packed_as=8"
                )

            if not ok:
                self.status_changed.emit(f"❌ Error guardando: {filename}")
                return
            
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
        if self.worker is None:
            self.status_changed.emit(
                f"❌ Error: No hay cámara para imagen {image_index}"
            )
            return False

        try:
            from hardware.camera.scientific_image import (
                image16_to_u8_preview,
                save_scientific_image,
            )

            # Única adquisición CMOS (nunca preview current_frame)
            sci = self.acquire_scientific_frame(timeout_s=2.0)
            frame = np.asarray(sci.image16)
            h_orig, w_orig = frame.shape[:2]
            logger.debug(
                "[CameraService] Captura nativa 1:1: %dx%d via %s",
                w_orig,
                h_orig,
                sci.pipeline_id,
            )

            channels = config.get('channels', {'R': False, 'G': True, 'B': False})
            selected_channels = [c for c in ['R', 'G', 'B'] if channels.get(c, False)]
            n_selected = len(selected_channels)

            if frame.ndim == 3 and n_selected == 1:
                channel_map = {'B': 0, 'G': 1, 'R': 2}
                frame = frame[:, :, channel_map[selected_channels[0]]]
            elif frame.ndim == 3 and 0 < n_selected < 3:
                new_frame = np.zeros_like(frame)
                if channels.get('B', False):
                    new_frame[:, :, 0] = frame[:, :, 0]
                if channels.get('G', False):
                    new_frame[:, :, 1] = frame[:, :, 1]
                if channels.get('R', False):
                    new_frame[:, :, 2] = frame[:, :, 2]
                frame = new_frame
            elif frame.ndim == 2 and n_selected >= 2:
                frame = cv2.cvtColor(
                    image16_to_u8_preview(frame), cv2.COLOR_GRAY2BGR
                )
                frame = (frame.astype(np.uint16) << 8)

            class_name = config.get('class_name', 'Imagen')
            save_folder = config.get('save_folder', '.')
            img_format = config.get('img_format', 'png').lower()
            use_16bit = bool(config.get('use_16bit', True))
            x_um = config.get('x_um')
            y_um = config.get('y_um')
            has_xy = x_um is not None and y_um is not None

            def _microscopy_filename(ext: str) -> str:
                if has_xy:
                    return build_single_capture_filename(
                        class_name, image_index, float(x_um), float(y_um), ext
                    )
                return f"{class_name}_{image_index:05d}.{ext}"

            if img_format == 'jpg':
                use_16bit = False

            if img_format == 'tiff':
                filename = _microscopy_filename('tiff')
            elif img_format == 'png':
                filename = _microscopy_filename('png')
            else:
                filename = _microscopy_filename('jpg')
            filepath = os.path.join(save_folder, filename)

            if use_16bit and img_format in ('tiff', 'png'):
                success = save_scientific_image(
                    filepath, frame, already_prepared=True
                )
                bits_str = "16-bit"
            else:
                frame8 = image16_to_u8_preview(frame)
                if img_format == 'jpg':
                    success = safe_imwrite(
                        filepath, frame8, [cv2.IMWRITE_JPEG_QUALITY, 95]
                    )
                    bits_str = "8-bit (JPG)"
                else:
                    success = safe_imwrite(
                        filepath, frame8, [cv2.IMWRITE_PNG_COMPRESSION, 6]
                    )
                    bits_str = "8-bit"

            if not success:
                self.status_changed.emit(
                    f"❌ Error: guardado científico falló para {filename}"
                )
                return False

            capture_position = config.get("capture_position")
            point_base = config.get("point_base")
            if capture_position and point_base:
                try:
                    from core.canvas.capture_position import (
                        CapturePositionMetadata,
                        save_position_sidecar,
                    )

                    position = CapturePositionMetadata.from_dict(capture_position)
                    save_position_sidecar(save_folder, point_base, position)
                except Exception as sidecar_exc:
                    logger.warning(
                        "[CameraService] No se pudo guardar sidecar de posición: %s",
                        sidecar_exc,
                    )
            
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

    def is_streaming(self) -> bool:
        """True si la cámara transmite frames (vista en vivo activa)."""
        if self.worker is None:
            return False
        frame = self.worker.current_frame
        return (
            self.worker.isRunning()
            and getattr(self.worker, "running", False)
            and frame is not None
            and getattr(frame, "size", 0) > 0
        )

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
    
    def get_resolution(self) -> Optional[Tuple[int, int]]:
        """Resolución real (width, height). Nunca inventa 1920×1080.

        Prioridad: frame actual → ROI del worker → nodos GenICam → datasheet Basler.
        """
        if self.worker is None:
            logger.warning("[CameraService] Sin worker; resolución desconocida")
            return None

        frame = self.worker.current_frame
        frame_hw = frame.shape[:2] if frame is not None else None

        configured_wh = None
        cw = getattr(self.worker, "configured_width", None)
        ch = getattr(self.worker, "configured_height", None)
        if cw is not None and ch is not None:
            configured_wh = (int(cw), int(ch))

        camera = getattr(self.worker, "camera", None)
        node_wh = read_camera_roi_wh(camera) if camera is not None else None

        datasheet_wh = None
        worker_type = ""
        try:
            worker_type = str(self.worker.get_camera_type()).lower()
        except Exception:
            worker_type = type(self.worker).__name__.lower()
        if "basler" in worker_type:
            datasheet_wh = (ACA2500_14UC.width, ACA2500_14UC.height)

        resolved = resolve_camera_resolution(
            frame_hw=frame_hw,
            configured_wh=configured_wh,
            node_wh=node_wh,
            datasheet_wh=datasheet_wh,
        )
        if resolved is None:
            logger.warning("[CameraService] No se pudo resolver resolución de cámara")
            return None

        w, h = resolved
        if not self._resolution_logged:
            logger.info("[CameraService] Resolución de cámara: %dx%d", w, h)
            self._resolution_logged = True
        else:
            logger.debug("[CameraService] Resolución de cámara: %dx%d", w, h)
        return (w, h)

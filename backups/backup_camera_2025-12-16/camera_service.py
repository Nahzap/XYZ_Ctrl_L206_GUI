"""
Camera Service - Servicio de Cámara Thorlabs
===========================================

Orquesta CameraWorker en un QThread separado y expone señales
para que la UI (CameraTab) no tenga lógica de hardware.

Autor: Sistema de Control L206
"""

import logging
from typing import Optional

from PyQt5.QtCore import QObject, pyqtSignal

from hardware.camera.camera_worker import CameraWorker


logger = logging.getLogger('MotorControl_L206')


class CameraService(QObject):
    """Servicio de cámara que encapsula CameraWorker.

    Expone señales de alto nivel para que la UI se mantenga liviana.

    Signals:
        status_changed: Mensajes de estado para logging en UI.
        connected: Resultado de conexión (success, info).
        disconnected: Emite cuando la cámara se desconecta.
        frame_ready: Nuevo frame disponible (QImage, raw_frame).
    """

    # Señales
    status_changed = pyqtSignal(str)
    connected = pyqtSignal(bool, str)  # success, info
    disconnected = pyqtSignal()
    frame_ready = pyqtSignal(object, object)  # QImage, raw_frame

    def __init__(self, parent=None, thorlabs_available: bool = False):
        super().__init__(parent)
        self.worker: Optional[CameraWorker] = None
        self._thorlabs_available = thorlabs_available

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

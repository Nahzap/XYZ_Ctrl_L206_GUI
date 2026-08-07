"""
Basler Camera Worker - Worker para Cámara Basler acA2500-14uc
==============================================================

Implementa BaseCameraWorker usando pypylon SDK.
Basado en la experiencia del proyecto BaslerCAM.

Características:
- Resolución: 2590x1942 (5 MP)
- Profundidad: 12-bit nativo
- Frame rate: 14 fps @ 5MP
- Interfaz: USB 3.0 Vision
- Optimizaciones de latencia aplicadas

Autor: Sistema de Control L206
Fecha: 2026-03-05
"""

import gc
import logging
import time
import traceback
import numpy as np
from .base_camera_worker import BaseCameraWorker
from typing import Optional

from .scientific_config import (
    ACA2500_14UC,
    DEFAULT_SCIENTIFIC_BUFFER,
    DEFAULT_SCIENTIFIC_EXPOSURE_S,
    ScientificCameraSettings,
    apply_channel_gains,
    apply_scientific_settings,
    build_scientific_settings,
    clamp_fps,
    estimate_brightfield_wb_gains,
)
from .live_preview import (
    LivePipelineMetrics,
    bgr8_to_qimage_copy,
    make_preview_bgr,
)

logger = logging.getLogger('MotorControl_L206')

# Importar pypylon desde módulo centralizado
from config.hardware_availability import BASLER_AVAILABLE, pylon


class BaslerWorker(BaseCameraWorker):
    """Worker para manejar cámara Basler usando pypylon en thread separado."""
    
    def __init__(self):
        super().__init__()
        self.camera = None
        self.converter = None
        self.fps = int(ACA2500_14UC.max_fps_full_frame)
        self.exposure = DEFAULT_SCIENTIFIC_EXPOSURE_S
        self.buffer_size = DEFAULT_SCIENTIFIC_BUFFER
        # Preview BGR8 (UI); científico = prepare_scientific_bgr16 único
        self.current_raw_frame = None
        self.current_scientific_bgr = None
        self.current_raw_frame_count = 0
        self.scientific_pixel_format: Optional[str] = None
        # Live: captura científica pide 1 Bayer → pipeline único
        self._copy_raw_next = False
        # Si False: no construir QImage (ventana oculta) — ahorra ~15 MB/frame
        self.preview_enabled = True
        self.live_metrics = LivePipelineMetrics()
        self.last_scientific_apply_log = None
        # ROI efectivo tras perfil científico (Width×Height GenICam)
        self.configured_width: Optional[int] = None
        self.configured_height: Optional[int] = None
        # Balance campo claro compartido preview ↔ pipeline científico
        self._wb_gains = (1.0, 1.0, 1.0)
        self._wb_last_frame = -10_000
        self._last_scientific_frame = None

    def request_scientific_raw(self) -> None:
        """Pide copiar el próximo Bayer/raw en el loop live (captura 16-bit)."""
        self._copy_raw_next = True

    def set_preview_enabled(self, enabled: bool) -> None:
        """Activa/desactiva build de QImage preview (GUI visible)."""
        self.preview_enabled = bool(enabled)

    def get_wb_gains(self):
        """Ganancias BGR de campo claro usadas en preview y pipeline único."""
        return tuple(self._wb_gains)

    def _publish_scientific_from_grab(self, grab_result, preview_bgr8=None) -> None:
        """Publica ScientificFrame WYSIWYG = mismos píxeles que el preview."""
        from hardware.camera.scientific_image import (
            scientific_frame_from_preview_bgr8,
            scientific_frame_from_raw,
        )

        try:
            raw = grab_result.Array
            self.current_raw_frame = np.asarray(raw).copy()
        except Exception:
            try:
                self.current_raw_frame = np.asarray(
                    grab_result.GetArray()
                ).copy()
            except Exception:
                self.current_raw_frame = None

        self.current_scientific_bgr = None
        self._last_scientific_frame = None
        frame_id = int(self.frame_count) + 1
        try:
            if preview_bgr8 is not None:
                # Color idéntico a la vista en vivo (demosaic pylon + WB).
                sci = scientific_frame_from_preview_bgr8(
                    preview_bgr8,
                    frame_id=frame_id,
                    wb_gains=tuple(self._wb_gains),
                    raw=self.current_raw_frame,
                    pixel_format=str(
                        self.scientific_pixel_format or "BayerGB12"
                    ),
                )
            elif self.current_raw_frame is not None:
                sci = scientific_frame_from_raw(
                    self.current_raw_frame,
                    pixel_format=str(
                        self.scientific_pixel_format or "BayerGB12"
                    ),
                    wb_gains=None,
                    frame_id=frame_id,
                )
                self._wb_gains = tuple(sci.wb_gains)
                self._wb_last_frame = int(self.frame_count)
            else:
                return
            self._last_scientific_frame = sci
            self.current_scientific_bgr = sci.image16
            self.current_raw_frame_count = frame_id
        except Exception as exc:
            logger.error(
                "[BaslerWorker] publish scientific falló: %s", exc
            )

    def acquire_scientific_frame(self, timeout_s: float = 2.0):
        """Única vía pública: siguiente grab → ScientificFrame preparado."""
        import time

        if not self.running:
            raise RuntimeError(
                "[BaslerWorker] acquire_scientific_frame requiere live view"
            )
        start_id = int(self.current_raw_frame_count)
        self.request_scientific_raw()
        deadline = time.perf_counter() + float(timeout_s)
        while time.perf_counter() < deadline:
            last = getattr(self, "_last_scientific_frame", None)
            if (
                last is not None
                and int(self.current_raw_frame_count) > start_id
            ):
                return last
            time.sleep(0.005)
        raise TimeoutError(
            f"[BaslerWorker] acquire_scientific_frame timeout ({timeout_s}s)"
        )

    def connect_camera(self):
        """Conecta con la primera cámara Basler disponible."""
        if not BASLER_AVAILABLE:
            self.status_update.emit("❌ Error: pypylon no está disponible")
            logger.warning("[BaslerWorker] pypylon no disponible")
            self.connection_success.emit(False, "")
            return
        
        try:
            self.status_update.emit("Conectando cámara Basler...")
            logger.info("[BaslerWorker] Intentando conectar cámara Basler")
            
            # Crear y abrir cámara
            self.camera = pylon.InstantCamera(
                pylon.TlFactory.GetInstance().CreateFirstDevice()
            )
            self.camera.Open()
            
            # Obtener información
            device_info = self.camera.GetDeviceInfo()
            model = device_info.GetModelName()
            serial = device_info.GetSerialNumber()
            vendor = device_info.GetVendorName()
            camera_info = f"{model} S/N:{serial}"
            
            logger.info(f"[BaslerWorker] Conectada: {vendor} {camera_info}")
            
            # Configurar cámara para máxima calidad
            self._configure_camera()
            
            # Test de captura
            logger.info("[BaslerWorker] Ejecutando test de captura...")
            test_result = self._test_single_capture()
            if test_result:
                self.status_update.emit("Test de captura: OK - Cámara funcional")
            else:
                self.status_update.emit("Test de captura: FALLO - Revisar logs")
            
            self.status_update.emit(f"Conexión exitosa: {camera_info}")
            self.connection_success.emit(True, camera_info)
            
        except Exception as e:
            error_msg = f"Error al conectar Basler: {str(e)}"
            self.status_update.emit(error_msg)
            logger.error(f"[BaslerWorker] Error conexión: {e}\n{traceback.format_exc()}")
            self.connection_success.emit(False, "")
    
    def _configure_camera(self):
        """Configura parámetros científicos (datasheet + autos Off + profundidad)."""
        try:
            logger.info("[BaslerWorker] Aplicando perfil científico acA2500-14uc...")

            available_formats = []
            try:
                available_formats = list(self.camera.PixelFormat.Symbolics)
            except Exception as exc:
                logger.debug("[BaslerWorker] No se listaron PixelFormat: %s", exc)

            settings = build_scientific_settings(
                exposure_s=self.exposure,
                fps=self.fps,
                buffer_frames=self.buffer_size,
                available_pixel_formats=available_formats,
            )
            apply_log = apply_scientific_settings(self.camera, settings)
            self.last_scientific_apply_log = apply_log
            for item in apply_log.get("applied", []):
                logger.info("[BaslerWorker] %s", item)
            for item in apply_log.get("skipped", []):
                logger.debug("[BaslerWorker] omitido: %s", item)

            res = apply_log.get("resolution")
            if isinstance(res, (tuple, list)) and len(res) >= 2:
                self.configured_width = int(res[0])
                self.configured_height = int(res[1])
            else:
                try:
                    self.configured_width = int(self.camera.Width.GetValue())
                    self.configured_height = int(self.camera.Height.GetValue())
                except Exception:
                    self.configured_width = int(ACA2500_14UC.width)
                    self.configured_height = int(ACA2500_14UC.height)
            logger.info(
                "[BaslerWorker] ROI configurado: %dx%d",
                self.configured_width,
                self.configured_height,
            )

            # Sincronizar atributos locales con lo aplicado
            self.exposure = settings.exposure_s
            self.fps = int(settings.fps)
            self.buffer_size = int(settings.buffer_frames)
            self.scientific_pixel_format = settings.pixel_format
            if not self.scientific_pixel_format:
                try:
                    self.scientific_pixel_format = str(
                        self.camera.PixelFormat.GetValue()
                    )
                except Exception:
                    self.scientific_pixel_format = None

            # Preview UI solamente. Persistencia/AF usan scientific_image (SRP).
            self.converter = pylon.ImageFormatConverter()
            self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
            self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
            logger.info("[BaslerWorker] Conversor preview: BGR8packed MsbAligned")
            logger.info(
                "[BaslerWorker] Pipeline científico único: prepare_scientific_bgr16"
            )

            logger.info("[BaslerWorker] Configuración científica completada")

        except Exception as e:
            logger.error(f"[BaslerWorker] Error configurando cámara: {e}\n{traceback.format_exc()}")

    def get_scientific_settings(self) -> ScientificCameraSettings:
        """Perfil científico actual derivado de atributos del worker."""
        return build_scientific_settings(
            exposure_s=self.exposure,
            fps=self.fps,
            buffer_frames=self.buffer_size,
        )
    
    def _test_single_capture(self) -> bool:
        """Test de captura simple para verificar funcionalidad."""
        if not self.camera or not self.camera.IsOpen():
            logger.error("[BaslerWorker] Test: cámara no conectada")
            return False
        
        try:
            logger.info("[BaslerWorker] === TEST DE CAPTURA ===")
            
            # Iniciar adquisición temporal
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            
            # Intentar capturar frame
            grabResult = self.camera.RetrieveResult(
                2000, pylon.TimeoutHandling_ThrowException
            )
            
            if grabResult.GrabSucceeded():
                image = self.converter.Convert(grabResult)
                frame = image.GetArray()
                self.current_frame = frame.copy()
                h, w = frame.shape[:2]
                self.configured_width = int(w)
                self.configured_height = int(h)
                logger.info(
                    "[BaslerWorker] Test OK: Frame capturado shape=%s, dtype=%s",
                    frame.shape,
                    frame.dtype,
                )
                grabResult.Release()
                self.camera.StopGrabbing()
                return True
            else:
                logger.error("[BaslerWorker] Test FALLO: GrabSucceeded() = False")
                grabResult.Release()
                self.camera.StopGrabbing()
                return False
                
        except Exception as e:
            logger.error(f"[BaslerWorker] Error en test: {e}\n{traceback.format_exc()}")
            try:
                self.camera.StopGrabbing()
            except:
                pass
            return False
    
    def start_live_view(self):
        """Inicia adquisición de video en vivo (baja latencia)."""
        if not self.camera or not self.camera.IsOpen():
            self.status_update.emit("Error: Cámara Basler no conectada")
            logger.warning("[BaslerWorker] start_live_view: cámara no conectada")
            return
        
        try:
            self.status_update.emit("Iniciando vista en vivo Basler...")
            logger.info("[BaslerWorker] Iniciando adquisición continua")

            # Aplicar params de UI al hardware ANTES de grabbing
            try:
                self.change_exposure(float(self.exposure))
            except Exception as exc:
                logger.debug("[BaslerWorker] exposure pre-live: %s", exc)
            try:
                self.change_fps(int(self.fps))
            except Exception as exc:
                logger.debug("[BaslerWorker] fps pre-live: %s", exc)
            try:
                # 2 buffers: estable con LatestImageOnly y poca cola
                self.change_buffer_size(max(2, int(self.buffer_size or 2)))
            except Exception as exc:
                logger.debug("[BaslerWorker] buffer pre-live: %s", exc)
            
            # LatestImageOnly: descarta frames viejos en cámara (mínima latencia)
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            self.running = True
            self.frame_count = 0
            self.current_raw_frame_count = 0
            
            logger.info(
                "[BaslerWorker] Loop live: fps=%s exp=%ss buffer=%s",
                self.fps,
                self.exposure,
                self.buffer_size,
            )
            
            while self.running and self.camera.IsGrabbing():
                t_grab = time.perf_counter()
                grabResult = self.camera.RetrieveResult(
                    200, pylon.TimeoutHandling_Return
                )
                
                if grabResult and grabResult.GrabSucceeded():
                    image = self.converter.Convert(grabResult)
                    frame = image.GetArray()

                    # Preview = referencia de color. WB sobre demosaic pylon;
                    # el PNG científico se empaqueta desde este mismo BGR8.
                    if (
                        self.frame_count - int(self._wb_last_frame) >= 14
                        or self._wb_last_frame < 0
                    ):
                        self._wb_gains = estimate_brightfield_wb_gains(frame)
                        self._wb_last_frame = int(self.frame_count)
                    frame = apply_channel_gains(frame, self._wb_gains)

                    if self._copy_raw_next:
                        self._publish_scientific_from_grab(grabResult, frame)
                        self._copy_raw_next = False

                    # Publicar frame ANTES del contador. Autofocus espera
                    # ``frame_count`` y luego lee ``current_frame``; el orden
                    # inverso permitía consumir el frame anterior.
                    self.current_frame = frame.copy()
                    self.frame_count += 1
                    self.live_metrics.frames_grabbed += 1
                    h, w = frame.shape[:2]
                    self.live_metrics.last_full_w = int(w)
                    self.live_metrics.last_full_h = int(h)

                    q_image = None
                    t_prev = time.perf_counter()
                    if self.preview_enabled:
                        preview = make_preview_bgr(self.current_frame)
                        q_image = bgr8_to_qimage_copy(preview)
                        ph, pw = preview.shape[:2]
                        self.live_metrics.last_preview_w = int(pw)
                        self.live_metrics.last_preview_h = int(ph)
                        self.live_metrics.preview_builds += 1
                        self.live_metrics.last_preview_ms = (
                            time.perf_counter() - t_prev
                        ) * 1000.0
                    else:
                        self.live_metrics.preview_builds_skipped += 1
                        self.live_metrics.last_preview_w = 0
                        self.live_metrics.last_preview_h = 0
                        self.live_metrics.last_preview_ms = 0.0

                    self.new_frame_ready.emit(q_image, self.current_frame)
                    
                    grab_ms = (time.perf_counter() - t_grab) * 1000.0
                    self.live_metrics.last_grab_ms = grab_ms
                    if self.frame_count == 1:
                        logger.info(
                            "[BaslerWorker] Primer frame live: %dx%d dtype=%s "
                            "grab=%.1fms preview=%dx%d",
                            w,
                            h,
                            frame.dtype,
                            grab_ms,
                            self.live_metrics.last_preview_w,
                            self.live_metrics.last_preview_h,
                        )
                    elif self.frame_count % 60 == 0:
                        m = self.live_metrics.snapshot()
                        logger.info(
                            "[BaslerWorker] Live #%d grab=%.1fms preview_ms=%.1f "
                            "full=%dB preview=%dB skipped=%d",
                            self.frame_count,
                            grab_ms,
                            m["last_preview_ms"],
                            m["est_full_bytes"],
                            m["est_preview_bytes"],
                            m["preview_builds_skipped"],
                        )
                    elif grab_ms > 120.0:
                        logger.warning(
                            "[BaslerWorker] Grab lento frame #%d: %.1fms",
                            self.frame_count, grab_ms,
                        )
                    
                    grabResult.Release()
                    
                elif grabResult:
                    grabResult.Release()
            
            self.status_update.emit("Vista en vivo Basler detenida")
            logger.info("[BaslerWorker] Adquisición detenida")
            
        except Exception as e:
            error_msg = f"Error en vista en vivo: {str(e)}"
            self.status_update.emit(error_msg)
            logger.error(f"[BaslerWorker] Error en live view: {e}\n{traceback.format_exc()}")
        
        finally:
            try:
                if self.camera and self.camera.IsGrabbing():
                    self.camera.StopGrabbing()
                    logger.info("[BaslerWorker] StopGrabbing() ejecutado")
            except Exception as e:
                logger.error(f"[BaslerWorker] Error limpiando recursos: {e}")
            
            self.frame_count = 0
    
    def stop_live_view(self):
        """Detiene adquisición de video."""
        logger.info("[BaslerWorker] Deteniendo vista en vivo...")
        self.running = False
    
    def change_exposure(self, exposure_value: float):
        """Cambia exposición en tiempo real (en segundos)."""
        try:
            if self.camera and self.camera.IsOpen():
                self.exposure = exposure_value
                exposure_us = exposure_value * 1e6  # s → µs
                
                # Validar rango
                exposure_min = self.camera.ExposureTime.GetMin()
                exposure_max = self.camera.ExposureTime.GetMax()
                exposure_us = max(exposure_min, min(exposure_us, exposure_max))
                
                self.camera.ExposureTime.SetValue(exposure_us)
                self.status_update.emit(f"Exposición Basler: {exposure_us:.0f} µs")
                logger.info(f"[BaslerWorker] Exposición: {exposure_us} µs")
            else:
                self.status_update.emit("Error: Cámara Basler no conectada")
                logger.warning("[BaslerWorker] change_exposure: cámara no conectada")
        except Exception as e:
            error_msg = f"Error al cambiar exposición: {str(e)}"
            self.status_update.emit(error_msg)
            logger.error(f"[BaslerWorker] Error cambio exposición: {e}")
    
    def change_fps(self, fps_value: int):
        """Cambia frame rate en tiempo real (clamp a datasheet/hardware)."""
        try:
            if self.camera and self.camera.IsOpen():
                hw_max = float(self.camera.AcquisitionFrameRate.GetMax())
                fps_value = int(clamp_fps(float(fps_value), min(hw_max, ACA2500_14UC.max_fps_full_frame)))
                self.fps = fps_value
                self.camera.AcquisitionFrameRate.SetValue(float(fps_value))
                self.status_update.emit(f"Frame rate Basler: {fps_value} fps")
                logger.info(f"[BaslerWorker] Frame rate: {fps_value} fps")
            else:
                self.fps = int(clamp_fps(float(fps_value), ACA2500_14UC.max_fps_full_frame))
                self.status_update.emit("Error: Cámara Basler no conectada")
                logger.warning("[BaslerWorker] change_fps: cámara no conectada")
        except Exception as e:
            error_msg = f"Error al cambiar FPS: {str(e)}"
            self.status_update.emit(error_msg)
            logger.error(f"[BaslerWorker] Error cambio FPS: {e}")
    
    def change_buffer_size(self, buffer_value: int):
        """Cambia tamaño de buffer."""
        try:
            self.buffer_size = buffer_value
            if self.camera and self.camera.IsOpen():
                self.camera.MaxNumBuffer.SetValue(buffer_value)
                self.status_update.emit(f"Buffer Basler: {buffer_value} frames")
                logger.info(f"[BaslerWorker] Buffer: {buffer_value} frames")
            else:
                self.status_update.emit(f"Buffer guardado: {buffer_value} frames (aplicará en próxima conexión)")
                logger.info(f"[BaslerWorker] Buffer guardado: {buffer_value}")
        except Exception as e:
            error_msg = f"Error al cambiar buffer: {str(e)}"
            self.status_update.emit(error_msg)
            logger.error(f"[BaslerWorker] Error cambio buffer: {e}")
    
    def disconnect_camera(self):
        """Desconecta cámara y libera recursos."""
        if self.camera and self.camera.IsOpen():
            logger.info("[BaslerWorker] Desconectando cámara...")
            
            # Detener adquisición si está activa
            self.stop_live_view()
            
            # Esperar a que termine el loop
            self.wait(1000)  # Timeout de 1 segundo
            
            # Cerrar cámara
            try:
                self.camera.Close()
                logger.info("[BaslerWorker] Cámara cerrada")
            except Exception as e:
                logger.error(f"[BaslerWorker] Error cerrando cámara: {e}")
            
            # Liberar referencias
            self.camera = None
            self.converter = None
            self.current_frame = None
            self.current_raw_frame = None
            self.current_raw_frame_count = 0
            self.frame_count = 0
            
            # Forzar garbage collection
            gc.collect()
            
            self.status_update.emit("Cámara Basler cerrada")
            logger.info("[BaslerWorker] Recursos liberados")

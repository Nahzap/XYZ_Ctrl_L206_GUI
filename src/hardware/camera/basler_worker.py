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
import traceback
import numpy as np
from PyQt5.QtGui import QImage
from .base_camera_worker import BaseCameraWorker

logger = logging.getLogger('MotorControl_L206')

# Importar pypylon desde módulo centralizado
from config.hardware_availability import BASLER_AVAILABLE, pylon


class BaslerWorker(BaseCameraWorker):
    """Worker para manejar cámara Basler usando pypylon en thread separado."""
    
    def __init__(self):
        super().__init__()
        self.camera = None
        self.converter = None
        self.fps = 14  # Max fps para acA2500-14uc
    
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
        """Configura parámetros de cámara para máxima calidad y precisión."""
        try:
            logger.info("[BaslerWorker] Configurando parámetros para máxima calidad...")
            
            # Resolución máxima
            max_width = self.camera.Width.GetMax()
            max_height = self.camera.Height.GetMax()
            self.camera.Width.SetValue(max_width)
            self.camera.Height.SetValue(max_height)
            logger.info(f"[BaslerWorker] Resolución: {max_width}x{max_height}")
            
            # Formato de píxel (prioridad 12-bit)
            available_formats = list(self.camera.PixelFormat.Symbolics)
            logger.info(f"[BaslerWorker] Formatos disponibles: {available_formats}")
            
            format_priority = ["BayerGB12", "BayerGB12p", "BayerGB8", "Mono8", "RGB8"]
            format_set = False
            for fmt in format_priority:
                if fmt in available_formats:
                    self.camera.PixelFormat.SetValue(fmt)
                    logger.info(f"[BaslerWorker] Formato de píxel: {fmt}")
                    format_set = True
                    break
            
            if not format_set:
                current_fmt = self.camera.PixelFormat.GetValue()
                logger.warning(f"[BaslerWorker] Usando formato actual: {current_fmt}")
            
            # Exposición inicial
            exposure_us = self.exposure * 1e6  # s → µs
            exposure_min = self.camera.ExposureTime.GetMin()
            exposure_max = self.camera.ExposureTime.GetMax()
            exposure_us = max(exposure_min, min(exposure_us, exposure_max))
            self.camera.ExposureTime.SetValue(exposure_us)
            logger.info(f"[BaslerWorker] Exposición: {exposure_us} µs (rango: {exposure_min}-{exposure_max})")
            
            # Ganancia inicial (0 dB para mínimo ruido)
            try:
                self.camera.Gain.SetValue(0.0)
                logger.info("[BaslerWorker] Ganancia: 0 dB (mínimo ruido)")
            except Exception as e:
                logger.debug(f"[BaslerWorker] Ganancia no configurada: {e}")
            
            # Frame rate
            try:
                self.camera.AcquisitionFrameRateEnable.SetValue(True)
                max_fps = self.camera.AcquisitionFrameRate.GetMax()
                target_fps = min(self.fps, max_fps)
                self.camera.AcquisitionFrameRate.SetValue(target_fps)
                logger.info(f"[BaslerWorker] Frame rate: {target_fps} fps (max: {max_fps})")
            except Exception as e:
                logger.debug(f"[BaslerWorker] Frame rate no configurado: {e}")
            
            # Buffer optimizado (5 frames para mínima latencia)
            try:
                self.camera.MaxNumBuffer.SetValue(5)
                logger.info("[BaslerWorker] Buffer: 5 frames (optimizado)")
            except Exception as e:
                logger.debug(f"[BaslerWorker] Buffer no configurado: {e}")
            
            # Binning desactivado (1x1 para máxima resolución)
            try:
                self.camera.BinningHorizontal.SetValue(1)
                self.camera.BinningVertical.SetValue(1)
                logger.info("[BaslerWorker] Binning: 1x1 (máxima resolución)")
            except Exception as e:
                logger.debug(f"[BaslerWorker] Binning no configurado: {e}")
            
            # Conversor de formato (BGR8 para compatibilidad con Qt/OpenCV)
            self.converter = pylon.ImageFormatConverter()
            self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
            self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
            logger.info("[BaslerWorker] Conversor: BGR8packed configurado")
            
            logger.info("[BaslerWorker] Configuración completada exitosamente")
            
        except Exception as e:
            logger.error(f"[BaslerWorker] Error configurando cámara: {e}\n{traceback.format_exc()}")
    
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
                logger.info(f"[BaslerWorker] Test OK: Frame capturado shape={frame.shape}, dtype={frame.dtype}")
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
        """Inicia adquisición de video en vivo."""
        if not self.camera or not self.camera.IsOpen():
            self.status_update.emit("Error: Cámara Basler no conectada")
            logger.warning("[BaslerWorker] start_live_view: cámara no conectada")
            return
        
        try:
            self.status_update.emit("Iniciando vista en vivo Basler...")
            logger.info("[BaslerWorker] Iniciando adquisición continua")
            
            # Iniciar adquisición (LatestImageOnly para mínima latencia)
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            self.running = True
            self.frame_count = 0
            
            logger.info("[BaslerWorker] Loop de adquisición iniciado")
            
            while self.running and self.camera.IsGrabbing():
                # Timeout de 1000ms (optimizado para latencia)
                grabResult = self.camera.RetrieveResult(
                    1000, pylon.TimeoutHandling_Return
                )
                
                if grabResult and grabResult.GrabSucceeded():
                    # Convertir a BGR8
                    image = self.converter.Convert(grabResult)
                    frame = image.GetArray()
                    
                    # Incrementar contador
                    self.frame_count += 1
                    
                    # Guardar copia para captura
                    self.current_frame = frame.copy()
                    
                    # Convertir a QImage para display
                    h, w = frame.shape[:2]
                    bytes_per_line = 3 * w
                    q_image = QImage(
                        frame.data, w, h, bytes_per_line,
                        QImage.Format_RGB888
                    ).rgbSwapped().copy()
                    
                    # Emitir frame (QImage para display, raw para procesamiento)
                    self.new_frame_ready.emit(q_image, self.current_frame)
                    
                    # Liberar resultado
                    grabResult.Release()
                    
                    # Limpieza periódica cada 30 frames
                    if self.frame_count % 30 == 0:
                        gc.collect()
                    
                elif grabResult:
                    # GrabSucceeded() = False
                    grabResult.Release()
            
            self.status_update.emit("Vista en vivo Basler detenida")
            logger.info("[BaslerWorker] Adquisición detenida")
            
        except Exception as e:
            error_msg = f"Error en vista en vivo: {str(e)}"
            self.status_update.emit(error_msg)
            logger.error(f"[BaslerWorker] Error en live view: {e}\n{traceback.format_exc()}")
        
        finally:
            # Limpiar recursos
            try:
                if self.camera and self.camera.IsGrabbing():
                    self.camera.StopGrabbing()
                    logger.info("[BaslerWorker] StopGrabbing() ejecutado")
                gc.collect()
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
        """Cambia frame rate en tiempo real."""
        try:
            if self.camera and self.camera.IsOpen():
                self.fps = fps_value
                
                # Validar rango
                max_fps = self.camera.AcquisitionFrameRate.GetMax()
                fps_value = min(fps_value, max_fps)
                
                self.camera.AcquisitionFrameRate.SetValue(float(fps_value))
                self.status_update.emit(f"Frame rate Basler: {fps_value} fps")
                logger.info(f"[BaslerWorker] Frame rate: {fps_value} fps")
            else:
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
            self.frame_count = 0
            
            # Forzar garbage collection
            gc.collect()
            
            self.status_update.emit("Cámara Basler cerrada")
            logger.info("[BaslerWorker] Recursos liberados")

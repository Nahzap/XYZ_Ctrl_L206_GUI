"""
Thorlabs Camera Worker - Worker para Cámara Thorlabs
=====================================================

Implementa BaseCameraWorker usando pylablib SDK.
Refactorizado desde camera_worker.py para heredar de la interfaz común.

Autor: Sistema de Control L206
Fecha: 2026-03-05 (Refactorizado)
"""

import gc
import logging
import time
import traceback
import numpy as np
from PyQt5.QtGui import QImage
from .base_camera_worker import BaseCameraWorker

logger = logging.getLogger('MotorControl_L206')

# Importar Thorlabs desde módulo centralizado
from config.hardware_availability import THORLABS_AVAILABLE, Thorlabs


class ThorlabsWorker(BaseCameraWorker):
    """Worker para manejar cámara Thorlabs en thread separado."""
    
    def __init__(self):
        super().__init__()
        self.cam = None
        self.buffer_size = 1  # Buffer inicial para Thorlabs
    
    def connect_camera(self):
        """Conecta con la primera cámara Thorlabs disponible."""
        if not THORLABS_AVAILABLE:
            self.status_update.emit("❌ Error: pylablib no está disponible")
            logger.warning("[ThorlabsWorker] pylablib no disponible")
            self.connection_success.emit(False, "")
            return
        
        try:
            self.status_update.emit("Conectando con la cámara Thorlabs...")
            logger.info("[ThorlabsWorker] Intentando conectar con cámara Thorlabs")
            
            self.cam = Thorlabs.ThorlabsTLCamera()
            info = self.cam.get_device_info()
            
            # Construir info de cámara con atributos disponibles
            camera_info_parts = []
            if hasattr(info, 'model'):
                camera_info_parts.append(info.model)
            if hasattr(info, 'serial_number'):
                camera_info_parts.append(f"S/N: {info.serial_number}")
            elif hasattr(info, 'serial'):
                camera_info_parts.append(f"S/N: {info.serial}")
            
            camera_info = " - ".join(camera_info_parts) if camera_info_parts else "Camara Thorlabs"
            
            self.status_update.emit(f"Conexión exitosa: {camera_info}")
            logger.info(f"[ThorlabsWorker] Cámara conectada: {camera_info}")
            
            # Ejecutar test de captura simple para verificar funcionalidad
            logger.info("[ThorlabsWorker] Ejecutando test de captura simple...")
            test_result = self.test_single_capture()
            if test_result:
                self.status_update.emit("Test de captura: OK - Cámara funcional")
            else:
                self.status_update.emit("Test de captura: FALLO - Revisar logs")
            
            self.connection_success.emit(True, camera_info)
            
        except Exception as e:
            error_msg = f"Error al conectar: {str(e)}"
            self.status_update.emit(error_msg)
            logger.error(f"[ThorlabsWorker] Error conexión cámara: {e}\n{traceback.format_exc()}")
            self.connection_success.emit(False, "")
    
    def start_live_view(self):
        """Inicia la adquisición de video en vivo."""
        if not self.cam or not self.cam.is_opened():
            self.status_update.emit("Error: La cámara no está conectada.")
            return
        
        try:
            self.status_update.emit("Iniciando vista en vivo...")
            logger.info("[ThorlabsWorker] Iniciando adquisición de cámara")
            
            # Configurar cámara
            logger.info(f"[ThorlabsWorker] Configurando exposición: {self.exposure}s")
            self.cam.set_exposure(self.exposure)
            actual_exposure = self.cam.get_exposure()
            logger.info(f"[ThorlabsWorker] Exposición actual: {actual_exposure}s")
            
            # Configurar trigger mode
            logger.info("[ThorlabsWorker] Configurando trigger mode: 'int' (interno)")
            self.cam.set_trigger_mode("int")
            
            # Configurar frame rate usando frame period
            frame_period = 1.0 / self.fps  # Periodo en segundos
            logger.info(f"[ThorlabsWorker] Configurando frame period: {frame_period:.6f}s ({self.fps} FPS)")
            self.cam.set_frame_period(frame_period)
            
            # Verificar el periodo configurado
            actual_period = self.cam.get_frame_period()
            actual_fps = 1.0 / actual_period if actual_period > 0 else 0
            logger.info(f"[ThorlabsWorker] Frame period actual: {actual_period:.6f}s ({actual_fps:.2f} FPS)")
            
            # Setup acquisition con buffer configurable
            logger.info(f"[ThorlabsWorker] Configurando adquisición con buffer de {self.buffer_size} frames")
            self.cam.setup_acquisition(nframes=self.buffer_size)
            
            # Iniciar adquisición
            logger.info("[ThorlabsWorker] Llamando a start_acquisition()...")
            self.cam.start_acquisition()
            logger.info("[ThorlabsWorker] start_acquisition() completado")
            
            # Verificar estado
            is_setup = self.cam.is_acquisition_setup()
            logger.info(f"[ThorlabsWorker] is_acquisition_setup(): {is_setup}")
            
            self.running = True
            logger.info(f"[ThorlabsWorker] Loop running activado: {self.running}")
            
            # Esperar un poco más para el primer frame
            first_frame = True
            timeout_count = 0
            max_timeouts = 10  # Máximo 10 timeouts consecutivos antes de abortar
            
            logger.info(f"[ThorlabsWorker] Loop de adquisición iniciado")
            while self.running:
                # Usar timeout más largo para el primer frame
                timeout = 3.0 if first_frame else 0.5
                
                try:
                    frame_available = self.cam.wait_for_frame(timeout=timeout)
                except Exception as timeout_error:
                    # Capturar TimeoutError específico de Thorlabs
                    if "Timeout" in type(timeout_error).__name__:
                        timeout_count += 1
                        logger.warning(f"[ThorlabsWorker] Timeout #{timeout_count} de {max_timeouts}")
                        
                        # En el primer timeout, hacer diagnóstico adicional
                        if timeout_count == 1:
                            try:
                                logger.info("[ThorlabsWorker] === DIAGNOSTICO PRIMER TIMEOUT ===")
                                logger.info(f"Cámara abierta: {self.cam.is_opened()}")
                                logger.info(f"Adquisición configurada: {self.cam.is_acquisition_setup()}")
                                
                                # Intentar obtener info de frames disponibles
                                if hasattr(self.cam, 'get_frames_status'):
                                    status = self.cam.get_frames_status()
                                    logger.info(f"Estado de frames: {status}")
                                
                                if hasattr(self.cam, 'get_new_images_range'):
                                    img_range = self.cam.get_new_images_range()
                                    logger.info(f"Rango de imágenes nuevas: {img_range}")
                                
                            except Exception as diag_error:
                                logger.warning(f"[ThorlabsWorker] Error en diagnóstico: {diag_error}")
                        
                        if timeout_count >= max_timeouts:
                            self.status_update.emit(f"Demasiados timeouts ({max_timeouts}). Verificar cámara.")
                            logger.warning(f"[ThorlabsWorker] Máximo de timeouts alcanzado ({max_timeouts}), deteniendo live view")
                            break
                        continue
                    else:
                        # Otro tipo de error, re-lanzar
                        logger.error(f"[ThorlabsWorker] Error no-timeout en wait_for_frame: {timeout_error}")
                        raise
                
                if frame_available:
                    # Frame disponible
                    timeout_count = 0  # Resetear contador de timeouts
                    first_frame = False
                    
                    # Leer frame más antiguo para evitar acumulación en buffer
                    frame = self.cam.read_oldest_image()
                    
                    if frame is not None:
                        # Contrato AF: publicar la imagen antes de anunciar
                        # el serial/contador correspondiente.
                        raw_frame = frame.copy()
                        self.current_frame = raw_frame
                        self.frame_count += 1
                        
                        # GESTIÓN DE MEMORIA: Limpiar buffer cada 30 frames
                        if self.frame_count % 30 == 0:
                            try:
                                # Limpiar frames sin leer del buffer
                                status = self.cam.get_frames_status()
                                if status.unread > 5:
                                    # Leer y descartar frames antiguos
                                    for _ in range(min(status.unread - 1, 10)):
                                        self.cam.read_oldest_image()
                                
                                # Forzar garbage collection cada 30 frames
                                gc.collect()
                            except Exception as e:
                                pass  # Ignorar errores de limpieza
                        
                        # Normalizar a uint8 para visualización
                        if frame.dtype != np.uint8:
                            frame = (frame / frame.max() * 255).astype(np.uint8)
                        
                        h, w = frame.shape
                        bytes_per_line = w
                        
                        # Crear QImage
                        q_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_Grayscale8).copy()
                        
                        # Emitir AMBOS: q_image para display, raw_frame para detección
                        self.new_frame_ready.emit(q_image, raw_frame)
                        
                        # Liberar referencia al frame original
                        del frame
                        
                elif frame_available is False:
                    # Adquisición detenida
                    self.status_update.emit("La adquisición se detuvo inesperadamente.")
                    break
                elif frame_available is None:
                    # Timeout sin excepción - continuar esperando
                    timeout_count += 1
                    if timeout_count >= max_timeouts:
                        self.status_update.emit(f"Demasiados timeouts silenciosos ({max_timeouts}). Verificar cámara.")
                        break
                    
        except Exception as e:
            self.status_update.emit(f"Error en vista en vivo: {str(e)}")
            logger.error(f"[ThorlabsWorker] Error en live view: {e}\n{traceback.format_exc()}")
        finally:
            # Limpiar memoria al detener
            try:
                if self.cam and self.cam.is_opened() and self.cam.is_acquisition_setup():
                    logger.info("[ThorlabsWorker] Deteniendo adquisición y limpiando buffer...")
                    self.cam.stop_acquisition()
                    
                    # Limpiar buffer completamente
                    if hasattr(self.cam, 'clear_acquisition'):
                        self.cam.clear_acquisition()
                        logger.info("[ThorlabsWorker] Buffer de cámara limpiado")
                    
                    # Forzar garbage collection
                    gc.collect()
                    logger.info("[ThorlabsWorker] Garbage collection ejecutado")
            except Exception as e:
                logger.error(f"[ThorlabsWorker] Error al limpiar recursos: {e}")
            
            self.frame_count = 0
            self.status_update.emit("Vista en vivo detenida.")
            logger.info("[ThorlabsWorker] Vista en vivo detenida")
    
    def stop_live_view(self):
        """Detiene la adquisición de video."""
        self.running = False
    
    def test_single_capture(self):
        """Prueba de captura simplificada para diagnóstico."""
        if not self.cam or not self.cam.is_opened():
            logger.error("[ThorlabsWorker] test_single_capture: cámara no conectada")
            return False
        
        try:
            logger.info("[ThorlabsWorker] === TEST DE CAPTURA SIMPLE ===")
            
            # Configuración
            logger.info(f"[ThorlabsWorker] Configurando: exposure={self.exposure}s, fps={self.fps}")
            self.cam.set_exposure(self.exposure)
            self.cam.set_trigger_mode("int")
            
            # Configurar frame rate usando frame period
            frame_period = 1.0 / self.fps
            self.cam.set_frame_period(frame_period)
            logger.info(f"[ThorlabsWorker] Frame period configurado: {frame_period:.6f}s ({self.fps} FPS)")
            
            # Setup acquisition con buffer configurable
            logger.info(f"[ThorlabsWorker] Llamando setup_acquisition(nframes={self.buffer_size})")
            self.cam.setup_acquisition(nframes=self.buffer_size)
            
            # Iniciar
            logger.info("[ThorlabsWorker] Iniciando adquisición...")
            self.cam.start_acquisition()
            
            # Esperar
            wait_time = 0.5
            logger.info(f"[ThorlabsWorker] Esperando {wait_time} segundos...")
            time.sleep(wait_time)
            
            # Leer frame
            logger.info("[ThorlabsWorker] Intentando read_oldest_image()...")
            frame = self.cam.read_oldest_image()
            
            if frame is not None:
                logger.info(f"[ThorlabsWorker] EXITO: Frame capturado! Shape: {frame.shape}, dtype: {frame.dtype}")
                self.cam.stop_acquisition()
                return True
            else:
                logger.error("[ThorlabsWorker] read_oldest_image() retornó None")
                self.cam.stop_acquisition()
                return False
                
        except Exception as e:
            logger.error(f"[ThorlabsWorker] Error en test_single_capture: {e}\n{traceback.format_exc()}")
            try:
                self.cam.stop_acquisition()
            except:
                pass
            return False
    
    def acquire_scientific_frame(self, timeout_s: float = 2.0):
        """Única vía pública: mono nativo → ScientificFrame (prepare único)."""
        from hardware.camera.scientific_image import scientific_frame_from_raw

        if not self.running:
            raise RuntimeError(
                "[ThorlabsWorker] acquire_scientific_frame requiere live view"
            )
        start_id = int(self.frame_count)
        deadline = time.perf_counter() + float(timeout_s)
        while time.perf_counter() < deadline:
            if int(self.frame_count) > start_id and self.current_frame is not None:
                raw = np.asarray(self.current_frame)
                if raw.dtype != np.uint16:
                    # Contenedor uint16 LSB; prepare empaqueta a MSB
                    if raw.dtype == np.uint8:
                        raw = raw.astype(np.uint16)
                    else:
                        raw = raw.astype(np.uint16, copy=False)
                return scientific_frame_from_raw(
                    raw,
                    pixel_format="Mono12",
                    wb_gains=(1.0, 1.0, 1.0),
                    frame_id=int(self.frame_count),
                )
            time.sleep(0.005)
        raise TimeoutError(
            f"[ThorlabsWorker] acquire_scientific_frame timeout ({timeout_s}s)"
        )

    def change_exposure(self, exposure_value: float):
        """Cambia la exposición de la cámara en tiempo real."""
        try:
            if self.cam and self.cam.is_opened():
                self.exposure = exposure_value
                self.cam.set_exposure(exposure_value)
                self.status_update.emit(f"Exposición cambiada a {exposure_value} s")
                logger.info(f"[ThorlabsWorker] Exposición cambiada: {exposure_value}s")
            else:
                self.status_update.emit("Error: Cámara no conectada")
        except Exception as e:
            self.status_update.emit(f"Error al cambiar exposición: {str(e)}")
            logger.error(f"[ThorlabsWorker] Error cambio exposición: {e}")
    
    def change_fps(self, fps_value: int):
        """Cambia el frame rate de la cámara usando frame period."""
        try:
            if self.cam and self.cam.is_opened():
                self.fps = fps_value
                frame_period = 1.0 / fps_value
                self.cam.set_frame_period(frame_period)
                self.status_update.emit(f"Frame rate cambiado a {fps_value} FPS")
                logger.info(f"[ThorlabsWorker] Frame rate cambiado: {fps_value} FPS (period={frame_period:.6f}s)")
            else:
                self.status_update.emit("Error: Cámara no conectada")
        except Exception as e:
            self.status_update.emit(f"Error al cambiar FPS: {str(e)}")
            logger.error(f"[ThorlabsWorker] Error cambio FPS: {e}")
    
    def change_buffer_size(self, buffer_value: int):
        """Cambia el tamaño del buffer de frames."""
        try:
            if self.cam and self.cam.is_opened():
                self.buffer_size = buffer_value
                self.status_update.emit(f"Buffer configurado: {buffer_value} frames (aplicará en próxima adquisición)")
                logger.info(f"[ThorlabsWorker] Buffer size guardado: {buffer_value} frames")
            else:
                self.status_update.emit("Error: Cámara no conectada")
        except Exception as e:
            self.status_update.emit(f"Error al cambiar buffer: {str(e)}")
            logger.error(f"[ThorlabsWorker] Error cambio buffer: {e}")
    
    def disconnect_camera(self):
        """Desconecta la cámara y libera memoria."""
        if self.cam and self.cam.is_opened():
            self.stop_live_view()
            self.status_update.emit("Cerrando conexión y liberando memoria...")
            
            # Limpiar buffer antes de cerrar
            try:
                if self.cam.is_acquisition_setup():
                    self.cam.stop_acquisition()
                if hasattr(self.cam, 'clear_acquisition'):
                    self.cam.clear_acquisition()
                    logger.info("[ThorlabsWorker] Buffer limpiado antes de desconectar")
            except Exception as e:
                logger.debug(f"[ThorlabsWorker] Error al limpiar buffer: {e}")
            
            self.cam.close()
            self.current_frame = None
            self.frame_count = 0
            
            # Forzar garbage collection
            gc.collect()
            logger.info("[ThorlabsWorker] Memoria liberada")
            
            self.status_update.emit("Cámara cerrada.")
            logger.info("[ThorlabsWorker] Cámara desconectada")


# Mantener compatibilidad retroactiva - alias para código existente
CameraWorker = ThorlabsWorker

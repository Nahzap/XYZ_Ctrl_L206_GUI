"""
Worker para camara Thorlabs en thread separado.

Este modulo maneja la adquisicion de frames de la camara Thorlabs en un thread
independiente para no bloquear la interfaz grafica.
"""

import gc
import logging
import time
import traceback

import numpy as np

from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QImage

logger = logging.getLogger(__name__)

# Importar Thorlabs si esta disponible
try:
    from pylablib.devices import Thorlabs
    THORLABS_AVAILABLE = True
except ImportError:
    THORLABS_AVAILABLE = False
    logger.warning("pylablib no disponible para CameraWorker")


class CameraWorker(QThread):
    """Worker para manejar la camara Thorlabs en un thread separado."""
    status_update = pyqtSignal(str)
    connection_success = pyqtSignal(bool, str)  # success, camera_info
    new_frame_ready = pyqtSignal(object)  # QImage
    
    def __init__(self):
        super().__init__()
        self.cam = None
        self.running = False
        self.exposure = 0.01
        self.fps = 60
        self.buffer_size = 2  # Buffer de 2: visualiza actual, guarda anterior
        self.current_frame = None  # Para captura de imagen
        self.frame_count = 0  # Contador para limpieza periódica
    
    def run(self):
        """Metodo run del thread - inicia la vista en vivo."""
        self.start_live_view()
        
    def connect_camera(self):
        """Conecta con la primera camara Thorlabs disponible."""
        try:
            self.status_update.emit("Conectando con la camara Thorlabs...")
            logger.info("Intentando conectar con camara Thorlabs")
            
            self.cam = Thorlabs.ThorlabsTLCamera()
            info = self.cam.get_device_info()
            
            # Construir info de camara con atributos disponibles
            camera_info_parts = []
            if hasattr(info, 'model'):
                camera_info_parts.append(info.model)
            if hasattr(info, 'serial_number'):
                camera_info_parts.append(f"S/N: {info.serial_number}")
            elif hasattr(info, 'serial'):
                camera_info_parts.append(f"S/N: {info.serial}")
            
            camera_info = " - ".join(camera_info_parts) if camera_info_parts else "Camara Thorlabs"
            
            self.status_update.emit(f"Conexion exitosa: {camera_info}")
            logger.info(f"Camara conectada: {camera_info}")
            
            # Ejecutar test de captura simple para verificar funcionalidad
            logger.info("Ejecutando test de captura simple...")
            test_result = self.test_single_capture()
            if test_result:
                self.status_update.emit("Test de captura: OK - Camara funcional")
            else:
                self.status_update.emit("Test de captura: FALLO - Revisar logs")
            
            self.connection_success.emit(True, camera_info)
            
        except Exception as e:
            error_msg = f"Error al conectar: {str(e)}"
            self.status_update.emit(error_msg)
            logger.error(f"Error conexion camara: {e}\n{traceback.format_exc()}")
            self.connection_success.emit(False, "")
    
    def start_live_view(self):
        """Inicia la adquisicion de video en vivo."""
        if not self.cam or not self.cam.is_opened():
            self.status_update.emit("Error: La camara no esta conectada.")
            return
        
        try:
            self.status_update.emit("Iniciando vista en vivo...")
            logger.info("Iniciando adquisicion de camara")
            
            # Configurar camara
            logger.info(f"Configurando exposicion: {self.exposure}s")
            self.cam.set_exposure(self.exposure)
            actual_exposure = self.cam.get_exposure()
            logger.info(f"Exposicion actual: {actual_exposure}s")
            
            # Configurar trigger mode
            logger.info("Configurando trigger mode: 'int' (interno)")
            self.cam.set_trigger_mode("int")
            
            # Configurar frame rate usando frame period
            frame_period = 1.0 / self.fps  # Periodo en segundos
            logger.info(f"Configurando frame period: {frame_period:.6f}s ({self.fps} FPS)")
            self.cam.set_frame_period(frame_period)
            
            # Verificar el periodo configurado
            actual_period = self.cam.get_frame_period()
            actual_fps = 1.0 / actual_period if actual_period > 0 else 0
            logger.info(f"Frame period actual: {actual_period:.6f}s ({actual_fps:.2f} FPS)")
            
            # Setup acquisition con buffer configurable
            logger.info(f"Configurando adquisicion con buffer de {self.buffer_size} frames")
            self.cam.setup_acquisition(nframes=self.buffer_size)
            
            # Iniciar adquisicion
            logger.info("Llamando a start_acquisition()...")
            self.cam.start_acquisition()
            logger.info("start_acquisition() completado")
            
            # Verificar estado
            is_setup = self.cam.is_acquisition_setup()
            logger.info(f"is_acquisition_setup(): {is_setup}")
            
            self.running = True
            logger.info(f"Loop running activado: {self.running}")
            
            # Esperar un poco mas para el primer frame
            first_frame = True
            timeout_count = 0
            max_timeouts = 10  # Maximo 10 timeouts consecutivos antes de abortar
            
            logger.info(f"Loop de adquisicion iniciado")
            while self.running:
                # Usar timeout mas largo para el primer frame
                timeout = 3.0 if first_frame else 0.5
                
                try:
                    frame_available = self.cam.wait_for_frame(timeout=timeout)
                except Exception as timeout_error:
                    # Capturar TimeoutError especifico de Thorlabs
                    if "Timeout" in type(timeout_error).__name__:
                        timeout_count += 1
                        logger.warning(f"Timeout #{timeout_count} de {max_timeouts}")
                        
                        # En el primer timeout, hacer diagnostico adicional
                        if timeout_count == 1:
                            try:
                                logger.info("=== DIAGNOSTICO PRIMER TIMEOUT ===")
                                logger.info(f"Camara abierta: {self.cam.is_opened()}")
                                logger.info(f"Adquisicion configurada: {self.cam.is_acquisition_setup()}")
                                
                                # Intentar obtener info de frames disponibles
                                if hasattr(self.cam, 'get_frames_status'):
                                    status = self.cam.get_frames_status()
                                    logger.info(f"Estado de frames: {status}")
                                
                                if hasattr(self.cam, 'get_new_images_range'):
                                    img_range = self.cam.get_new_images_range()
                                    logger.info(f"Rango de imagenes nuevas: {img_range}")
                                
                                # Listar algunos metodos disponibles relacionados con frames
                                frame_methods = [m for m in dir(self.cam) if 'frame' in m.lower() or 'image' in m.lower() or 'buffer' in m.lower()]
                                logger.info(f"Metodos disponibles con 'frame/image/buffer': {frame_methods}")
                                
                            except Exception as diag_error:
                                logger.warning(f"Error en diagnostico: {diag_error}")
                        
                        if timeout_count >= max_timeouts:
                            self.status_update.emit(f"Demasiados timeouts ({max_timeouts}). Verificar camara.")
                            logger.warning(f"Maximo de timeouts alcanzado ({max_timeouts}), deteniendo live view")
                            break
                        continue
                    else:
                        # Otro tipo de error, re-lanzar
                        logger.error(f"Error no-timeout en wait_for_frame: {timeout_error}")
                        raise
                
                if frame_available:
                    # Frame disponible
                    timeout_count = 0  # Resetear contador de timeouts
                    first_frame = False
                    
                    # Leer frame mas antiguo para evitar acumulacion en buffer
                    frame = self.cam.read_oldest_image()
                    
                    if frame is not None:
                        self.frame_count += 1
                        
                        # GESTION DE MEMORIA: Limpiar buffer cada 30 frames
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
                        
                        # GUARDAR frame para captura (una sola copia)
                        self.current_frame = frame.copy()
                        
                        # Normalizar a uint8 para visualizacion (reutilizar el frame original)
                        if frame.dtype != np.uint8:
                            frame = (frame / frame.max() * 255).astype(np.uint8)
                        
                        h, w = frame.shape
                        bytes_per_line = w
                        
                        # Crear QImage (PyQt5 usa Format_Grayscale8 sin .Format)
                        q_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_Grayscale8).copy()
                        
                        self.new_frame_ready.emit(q_image)
                        
                        # Liberar referencia al frame original
                        del frame
                        
                elif frame_available is False:
                    # Adquisicion detenida
                    self.status_update.emit("La adquisicion se detuvo inesperadamente.")
                    break
                elif frame_available is None:
                    # Timeout sin excepcion - continuar esperando
                    timeout_count += 1
                    if timeout_count >= max_timeouts:
                        self.status_update.emit(f"Demasiados timeouts silenciosos ({max_timeouts}). Verificar camara.")
                        break
                    
        except Exception as e:
            self.status_update.emit(f"Error en vista en vivo: {str(e)}")
            logger.error(f"Error en live view: {e}\n{traceback.format_exc()}")
        finally:
            # Limpiar memoria al detener
            try:
                if self.cam and self.cam.is_opened() and self.cam.is_acquisition_setup():
                    logger.info("Deteniendo adquisicion y limpiando buffer...")
                    self.cam.stop_acquisition()
                    
                    # Limpiar buffer completamente
                    if hasattr(self.cam, 'clear_acquisition'):
                        self.cam.clear_acquisition()
                        logger.info("Buffer de camara limpiado")
                    
                    # Forzar garbage collection
                    gc.collect()
                    logger.info("Garbage collection ejecutado")
            except Exception as e:
                logger.error(f"Error al limpiar recursos: {e}")
            
            self.frame_count = 0
            self.status_update.emit("Vista en vivo detenida.")
            logger.info("Vista en vivo detenida")
    
    def stop_live_view(self):
        """Detiene la adquisicion de video."""
        self.running = False
    
    def test_single_capture(self):
        """Prueba de captura simplificada para diagnostico."""
        if not self.cam or not self.cam.is_opened():
            logger.error("test_single_capture: camara no conectada")
            return False
        
        try:
            logger.info("=== TEST DE CAPTURA SIMPLE ===")
            
            # Configuracion
            logger.info(f"Configurando: exposure={self.exposure}s, fps={self.fps}")
            self.cam.set_exposure(self.exposure)
            self.cam.set_trigger_mode("int")
            
            # Configurar frame rate usando frame period
            frame_period = 1.0 / self.fps
            self.cam.set_frame_period(frame_period)
            logger.info(f"Frame period configurado: {frame_period:.6f}s ({self.fps} FPS)")
            
            # Setup acquisition con buffer configurable
            logger.info(f"Llamando setup_acquisition(nframes={self.buffer_size})")
            self.cam.setup_acquisition(nframes=self.buffer_size)
            
            # Iniciar
            logger.info("Iniciando adquisicion...")
            self.cam.start_acquisition()
            
            # Esperar
            wait_time = 0.5
            logger.info(f"Esperando {wait_time} segundos...")
            time.sleep(wait_time)
            
            # Leer frame
            logger.info("Intentando read_oldest_image()...")
            frame = self.cam.read_oldest_image()
            
            if frame is not None:
                logger.info(f"EXITO: Frame capturado! Shape: {frame.shape}, dtype: {frame.dtype}")
                self.cam.stop_acquisition()
                return True
            else:
                logger.error("read_oldest_image() retorno None")
                self.cam.stop_acquisition()
                return False
                
        except Exception as e:
            logger.error(f"Error en test_single_capture: {e}\n{traceback.format_exc()}")
            try:
                self.cam.stop_acquisition()
            except:
                pass
            return False
    
    def change_exposure(self, exposure_value):
        """Cambia la exposicion de la camara en tiempo real."""
        try:
            if self.cam and self.cam.is_opened():
                self.exposure = exposure_value
                self.cam.set_exposure(exposure_value)
                self.status_update.emit(f"Exposicion cambiada a {exposure_value} s")
                logger.info(f"Exposicion cambiada: {exposure_value}s")
            else:
                self.status_update.emit("Error: Camara no conectada")
        except Exception as e:
            self.status_update.emit(f"Error al cambiar exposicion: {str(e)}")
            logger.error(f"Error cambio exposicion: {e}")
    
    def change_fps(self, fps_value):
        """Cambia el frame rate de la camara usando frame period."""
        try:
            if self.cam and self.cam.is_opened():
                self.fps = fps_value
                frame_period = 1.0 / fps_value
                self.cam.set_frame_period(frame_period)
                self.status_update.emit(f"Frame rate cambiado a {fps_value} FPS")
                logger.info(f"Frame rate cambiado: {fps_value} FPS (period={frame_period:.6f}s)")
            else:
                self.status_update.emit("Error: Camara no conectada")
        except Exception as e:
            self.status_update.emit(f"Error al cambiar FPS: {str(e)}")
            logger.error(f"Error cambio FPS: {e}")
    
    def change_buffer_size(self, buffer_value):
        """Cambia el tamano del buffer de frames."""
        try:
            if self.cam and self.cam.is_opened():
                self.buffer_size = buffer_value
                self.status_update.emit(f"Buffer configurado: {buffer_value} frames (aplicara en proxima adquisicion)")
                logger.info(f"Buffer size guardado: {buffer_value} frames")
            else:
                self.status_update.emit("Error: Camara no conectada")
        except Exception as e:
            self.status_update.emit(f"Error al cambiar buffer: {str(e)}")
            logger.error(f"Error cambio buffer: {e}")
    
    def disconnect_camera(self):
        """Desconecta la camara y libera memoria."""
        if self.cam and self.cam.is_opened():
            self.stop_live_view()
            self.status_update.emit("Cerrando conexion y liberando memoria...")
            
            # Limpiar buffer antes de cerrar
            try:
                if self.cam.is_acquisition_setup():
                    self.cam.stop_acquisition()
                if hasattr(self.cam, 'clear_acquisition'):
                    self.cam.clear_acquisition()
                    logger.info("Buffer limpiado antes de desconectar")
            except Exception as e:
                logger.debug(f"Error al limpiar buffer: {e}")
            
            self.cam.close()
            self.current_frame = None
            self.frame_count = 0
            
            # Forzar garbage collection
            gc.collect()
            logger.info("Memoria liberada")
            
            self.status_update.emit("Camara cerrada.")
            logger.info("Camara desconectada")

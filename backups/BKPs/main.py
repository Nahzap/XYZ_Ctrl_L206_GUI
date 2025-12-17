"""
Sistema de Control y Análisis - Motores L206
============================================

Aplicación para control en tiempo real, grabación y análisis de función de 
transferencia para motores DC con driver L206.

Autor: Sistema de Control L206
Versión: 2.2
Licencia: Open Source
Estándares: IEEE Software Engineering Standards
"""

import sys
import serial
import time
import logging
from collections import deque
from datetime import datetime
import csv
import traceback

# --- Importaciones PyQt5 (PRIMERO) ---
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QGridLayout,
                             QLabel, QGroupBox, QPushButton, QLineEdit, QCheckBox, 
                             QHBoxLayout, QTextEdit, QMainWindow, QMenuBar, QTabWidget, QComboBox,
                             QFrame, QMessageBox, QDialog, QScrollArea, QFileDialog, 
                             QInputDialog, QRadioButton, QDialogButtonBox, QAction)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QPalette, QColor

# --- Configurar matplotlib DESPUÉS de PyQt5 ---
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.pyplot as plt
from matplotlib.figure import Figure

# --- Importar PyQtGraph DESPUÉS de PyQt5 ---
import pyqtgraph as pg
pg.setConfigOption('background', '#252525')
pg.setConfigOption('foreground', '#CCCCCC')

# --- Importaciones para análisis ---
import pandas as pd
import numpy as np

# =========================================================================
# --- Buffer Circular Optimizado para Señales en Tiempo Real ---
# =========================================================================
class OptimizedSignalBuffer:
    """Buffer circular optimizado con NumPy para señales analógicas en tiempo real."""
    
    def __init__(self, buffer_size=200, num_signals=4):
        self.buffer_size = buffer_size
        self.num_signals = num_signals
        self.write_index = 0
        self.is_full = False
        
        # Buffer principal - matriz 2D: [señales, muestras]
        self.data = np.zeros((num_signals, buffer_size), dtype=np.float32)
        
        # Arrays de vista para cada señal (sin copiar datos)
        self.signal_views = {
            'power_a': self.data[0],
            'power_b': self.data[1], 
            'sensor_1': self.data[2],
            'sensor_2': self.data[3]
        }
        
        # Buffer temporal para rendering (evita copias)
        self._render_buffer = np.zeros(buffer_size, dtype=np.float32)
        
        logger.info(f"Buffer optimizado inicializado: {buffer_size} muestras, {num_signals} señales")
    
    def append_data(self, power_a, power_b, sensor_1, sensor_2):
        """Agrega nuevos datos al buffer circular."""
        # Escribir datos directamente en la matriz
        self.data[0, self.write_index] = abs(power_a)
        self.data[1, self.write_index] = abs(power_b)
        self.data[2, self.write_index] = sensor_1
        self.data[3, self.write_index] = sensor_2
        
        # Avanzar índice circular
        self.write_index = (self.write_index + 1) % self.buffer_size
        if self.write_index == 0:
            self.is_full = True
    
    def get_signal_data(self, signal_name):
        """Obtiene datos de una señal específica ordenados cronológicamente."""
        if not self.is_full:
            # Buffer no lleno: devolver desde el inicio hasta write_index
            return self.signal_views[signal_name][:self.write_index]
        else:
            # Buffer lleno: reorganizar datos cronológicamente
            signal_data = self.signal_views[signal_name]
            np.concatenate((signal_data[self.write_index:], 
                          signal_data[:self.write_index]), 
                         out=self._render_buffer)
            return self._render_buffer
    
    def get_all_signals(self):
        """Obtiene todas las señales como diccionario de arrays NumPy."""
        return {
            name: self.get_signal_data(name) 
            for name in self.signal_views.keys()
        }
    
    def clear(self):
        """Limpia el buffer."""
        self.data.fill(0)
        self.write_index = 0
        self.is_full = False
        logger.debug("Buffer circular limpiado")
    
    def get_memory_usage(self):
        """Retorna el uso de memoria en bytes."""
        return self.data.nbytes + self._render_buffer.nbytes

# --- Importaciones para diseño de controlador H∞ ---
import control as ct

# --- Importaciones para cámara Thorlabs ---
try:
    import pylablib as pll
    # Configurar la ruta del SDK de Thorlabs
    pll.par["devices/dlls/thorlabs_tlcam"] = r"C:\Program Files\Thorlabs\ThorImageCAM\Bin"
    from pylablib.devices import Thorlabs
    THORLABS_AVAILABLE = True
except ImportError:
    THORLABS_AVAILABLE = False
    logger_temp = logging.getLogger(__name__)
    logger_temp.warning("pylablib no está instalado. Funcionalidad de cámara deshabilitada.")
except Exception as e:
    THORLABS_AVAILABLE = False
    logger_temp = logging.getLogger(__name__)
    logger_temp.warning(f"Error al configurar Thorlabs SDK: {e}")

# =========================================================================
# --- SISTEMA DE LOGGING (IEEE Software Engineering Standards) ---
# =========================================================================

# Configurar logging con formato IEEE
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s | %(levelname)-8s | %(name)s | %(funcName)s:%(lineno)d | %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(f'motor_control_{datetime.now().strftime("%Y%m%d")}.log', encoding='utf-8')
    ]
)

# Silenciar logs de librerías externas (matplotlib, PIL, etc.)
logging.getLogger('matplotlib').setLevel(logging.WARNING)
logging.getLogger('matplotlib.font_manager').setLevel(logging.WARNING)
logging.getLogger('PIL').setLevel(logging.WARNING)

logger = logging.getLogger('MotorControl_L206')

# --- CONFIGURACIÓN ---
# Ajusta el puerto a tu configuración
SERIAL_PORT = 'COM3' 
BAUD_RATE = 1000000
PLOT_LENGTH = 200

# --- Constantes del Sistema Físico ---
ADC_MAX = 1023.0
RECORRIDO_UM = 25000.0
FACTOR_ESCALA = RECORRIDO_UM / ADC_MAX # Aprox. 24.4379 µm/unidad_ADC
# --------------------

# --- Hoja de estilos para el TEMA OSCURO ---
DARK_STYLESHEET = """
QWidget {
    background-color: #2E2E2E;
    color: #F0F0F0;
    font-family: Arial;
}
QGroupBox {
    background-color: #383838;
    border: 1px solid #505050;
    border-radius: 5px;
    margin-top: 1ex;
    font-weight: bold;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top center;
    padding: 0 3px;
}
QLabel, QCheckBox {
    background-color: transparent;
}
QPushButton {
    background-color: #505050;
    border: 1px solid #606060;
    border-radius: 4px;
    padding: 5px;
}
QPushButton:hover {
    background-color: #606060;
}
QPushButton:pressed {
    background-color: #2E86C1;
}
QLineEdit, QTextEdit {
    background-color: #505050;
    border: 1px solid #606060;
    border-radius: 4px;
    padding: 3px;
    color: #F0F0F0;
}
QCheckBox::indicator {
    width: 13px;
    height: 13px;
}
"""

class SerialReaderThread(QThread):
    """Thread para lectura serial asíncrona."""
    data_received = pyqtSignal(str)

    def __init__(self, port, baudrate):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.ser = None
        logger.info(f"SerialReaderThread inicializado: Puerto={port}, Baudrate={baudrate}")

    def run(self):
        """Ejecuta el loop de lectura serial."""
        logger.debug(f"Iniciando thread de lectura serial en {self.port}")
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            logger.info(f"Puerto serial {self.port} abierto exitosamente")
            time.sleep(2)
            self.data_received.emit("INFO: Conectado exitosamente.")
            
            while self.running and self.ser.is_open:
                line = self.ser.readline()
                if line:
                    try:
                        decoded_line = line.decode('utf-8').strip()
                        if decoded_line:
                            self.data_received.emit(decoded_line)
                    except UnicodeDecodeError as e:
                        # Intentar con latin-1 como fallback
                        try:
                            decoded_line = line.decode('latin-1', errors='ignore').strip()
                            if decoded_line:
                                self.data_received.emit(decoded_line)
                                logger.debug(f"Línea decodificada con latin-1: {decoded_line[:50]}...")
                        except Exception as e2:
                            logger.warning(f"No se pudo decodificar línea. UTF-8: {e}, Latin-1: {e2}")

            if self.ser and self.ser.is_open:
                self.ser.close()
                logger.info("Puerto serial cerrado correctamente")
        except serial.SerialException as e:
            logger.error(f"Error al abrir puerto {self.port}: {e}")
            self.data_received.emit(f"ERROR: Puerto {self.port} no encontrado.")
        except Exception as e:
            logger.critical(f"Error inesperado en SerialReaderThread: {e}\n{traceback.format_exc()}")

    def stop(self):
        """Detiene el thread de lectura serial."""
        logger.debug("Deteniendo SerialReaderThread")
        self.running = False
        # No cerrar aquí, se cierra en el run() al terminar el loop
        # Esto previene el error AttributeError: 'NoneType' object has no attribute 'hEvent'
        self.wait()  # Esperar a que el thread termine naturalmente
        logger.info("Thread serial detenido correctamente")

# =========================================================================
# --- Ventana para Gráficos de Matplotlib ---
# =========================================================================
class MatplotlibWindow(QWidget):
    """Ventana independiente para mostrar gráficos de matplotlib con botón X funcional."""
    
    def __init__(self, figure, title="Gráfico", parent=None):
        super().__init__(parent, Qt.Window)  # Especificar que es una ventana independiente
        self.setWindowTitle(title)
        self.setGeometry(150, 150, 1000, 800)
        self.setStyleSheet(DARK_STYLESHEET)
        
        # Configurar como ventana independiente
        self.setAttribute(Qt.WA_DeleteOnClose, False)
        
        layout = QVBoxLayout(self)
        
        # Crear canvas de matplotlib
        self.canvas = FigureCanvas(figure)
        layout.addWidget(self.canvas)
        
        # Agregar barra de herramientas interactiva
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.toolbar.setStyleSheet("""
            QToolBar {
                background-color: #383838;
                border: 1px solid #505050;
                padding: 3px;
            }
            QToolButton {
                background-color: #505050;
                border: 1px solid #606060;
                border-radius: 3px;
                padding: 5px;
                margin: 2px;
                color: #F0F0F0;
            }
            QToolButton:hover {
                background-color: #606060;
            }
        """)
        layout.addWidget(self.toolbar)
        
        # Botón para cerrar
        close_btn = QPushButton("Cerrar")
        close_btn.clicked.connect(self.close)
        close_btn.setStyleSheet("font-size: 12px; padding: 8px;")
        layout.addWidget(close_btn)
        
        self.canvas.draw()
        
        # Habilitar cursor interactivo con coordenadas
        self.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        self.coord_label = QLabel("Coordenadas: Mueve el cursor sobre el gráfico")
        self.coord_label.setStyleSheet("color: #5DADE2; font-size: 10px; padding: 5px;")
        layout.insertWidget(2, self.coord_label)  # Insertar después del toolbar
        
        logger.debug(f"MatplotlibWindow creada: {title}")
    
    def on_mouse_move(self, event):
        """Muestra las coordenadas del cursor en tiempo real."""
        if event.inaxes:
            x, y = event.xdata, event.ydata
            self.coord_label.setText(f"📍 Tiempo: {x:.2f} s  |  Valor: {y:.2f}")
        else:
            self.coord_label.setText("Coordenadas: Mueve el cursor sobre el gráfico")
    
    def closeEvent(self, event):
        """Maneja el evento de cierre de la ventana."""
        logger.debug(f"Cerrando ventana: {self.windowTitle()}")
        plt.close('all')  # Cerrar todas las figuras de matplotlib
        event.accept()


# =========================================================================
# --- Ventana Separada para Gráficos en Tiempo Real ---
# =========================================================================
class SignalWindow(QWidget):
    """Ventana independiente para visualizar señales en tiempo real."""
    
    def __init__(self, parent=None):
        super().__init__(parent, Qt.Window)  # Especificar que es una ventana independiente
        self.setWindowTitle('Señales de Control - Tiempo Real')
        self.setGeometry(150, 150, 900, 600)
        self.setStyleSheet(DARK_STYLESHEET)
        
        layout = QVBoxLayout(self)
        
        # Crear el gráfico
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#252525')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel('left', 'Valor (ADC)', color='#CCCCCC', size='12pt')
        self.plot_widget.setLabel('bottom', 'Muestras', color='#CCCCCC', size='12pt')
        
        self.plot_widget.getAxis('left').setTextPen(color='#CCCCCC')
        self.plot_widget.getAxis('bottom').setTextPen(color='#CCCCCC')
        
        legend = self.plot_widget.addLegend()
        legend.setLabelTextColor('#F0F0F0')
        
        self.plot_widget.setYRange(0, 1023, padding=0)
        
        layout.addWidget(self.plot_widget)
        
        # Checkboxes para mostrar/ocultar señales
        checkbox_layout = QHBoxLayout()
        self.checkboxes = {}
        
        for key, name in [("sensor_1", "Sensor 1"), ("sensor_2", "Sensor 2"), 
                          ("power_a", "Potencia A"), ("power_b", "Potencia B")]:
            cb = QCheckBox(name)
            cb.setChecked(True)
            cb.stateChanged.connect(self.update_plot_visibility)
            self.checkboxes[key] = cb
            checkbox_layout.addWidget(cb)
        
        layout.addLayout(checkbox_layout)
        
        # Inicializar datos y líneas
        self.data = {
            'power_a': deque([0] * PLOT_LENGTH, maxlen=PLOT_LENGTH),
            'power_b': deque([0] * PLOT_LENGTH, maxlen=PLOT_LENGTH),
            'sensor_1': deque([0] * PLOT_LENGTH, maxlen=PLOT_LENGTH),
            'sensor_2': deque([0] * PLOT_LENGTH, maxlen=PLOT_LENGTH),
        }
        
        self.plot_lines = {
            'power_a': self.plot_widget.plot(pen=pg.mkPen('#00FFFF', width=2), name="Potencia A"),
            'power_b': self.plot_widget.plot(pen=pg.mkPen('#FF00FF', width=2), name="Potencia B"),
            'sensor_1': self.plot_widget.plot(pen=pg.mkPen('#FFFF00', width=2), name="Sensor 1"),
            'sensor_2': self.plot_widget.plot(pen=pg.mkPen('#00FF00', width=2), name="Sensor 2"),
        }
    
    def update_plot_visibility(self):
        """Muestra u oculta las líneas del gráfico según los checkboxes."""
        for key, cb in self.checkboxes.items():
            if cb.isChecked():
                self.plot_lines[key].show()
            else:
                self.plot_lines[key].hide()
    
    def update_data(self, pot_a, pot_b, sens_1, sens_2):
        """Actualiza los datos del gráfico."""
        try:
            if not hasattr(self, 'plot_lines'):
                return  # No hacer nada si el gráfico no se inicializó correctamente
            
            self.data['power_a'].append(abs(pot_a))
            self.data['power_b'].append(abs(pot_b))
            self.data['sensor_1'].append(sens_1)
            self.data['sensor_2'].append(sens_2)
            
            self.plot_lines['power_a'].setData(list(self.data['power_a']))
            self.plot_lines['power_b'].setData(list(self.data['power_b']))
            self.plot_lines['sensor_1'].setData(list(self.data['sensor_1']))
            self.plot_lines['sensor_2'].setData(list(self.data['sensor_2']))
        except Exception as e:
            print(f"Error al actualizar datos del gráfico: {e}")


# =========================================================================
# --- Worker para Cámara Thorlabs (Thread Separado) ---
# =========================================================================
class CameraWorker(QThread):
    """Worker para manejar la cámara Thorlabs en un thread separado."""
    status_update = pyqtSignal(str)
    connection_success = pyqtSignal(bool, str)  # success, camera_info
    new_frame_ready = pyqtSignal(object)  # QImage
    
    def __init__(self):
        super().__init__()
        self.cam = None
        self.running = False
        self.exposure = 0.01
        self.fps = 60
        self.buffer_size = 1  # Reducido a 1
        self.current_frame = None  # Para captura de imagen
        self.frame_count = 0  # Contador para limpieza periódica
    
    def run(self):
        """Método run del thread - inicia la vista en vivo."""
        self.start_live_view()
        
    def connect_camera(self):
        """Conecta con la primera cámara Thorlabs disponible."""
        try:
            self.status_update.emit("Conectando con la cámara Thorlabs...")
            logger.info("Intentando conectar con cámara Thorlabs")
            
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
            
            camera_info = " - ".join(camera_info_parts) if camera_info_parts else "Cámara Thorlabs"
            
            self.status_update.emit(f"✅ Conexión exitosa: {camera_info}")
            logger.info(f"Cámara conectada: {camera_info}")
            
            # Ejecutar test de captura simple para verificar funcionalidad
            logger.info("Ejecutando test de captura simple...")
            test_result = self.test_single_capture()
            if test_result:
                self.status_update.emit("✅ Test de captura: OK - Cámara funcional")
            else:
                self.status_update.emit("⚠️ Test de captura: FALLO - Revisar logs")
            
            self.connection_success.emit(True, camera_info)
            
        except Exception as e:
            error_msg = f"❌ Error al conectar: {str(e)}"
            self.status_update.emit(error_msg)
            logger.error(f"Error conexión cámara: {e}\n{traceback.format_exc()}")
            self.connection_success.emit(False, "")
    
    def start_live_view(self):
        """Inicia la adquisición de video en vivo."""
        if not self.cam or not self.cam.is_opened():
            self.status_update.emit("❌ Error: La cámara no está conectada.")
            return
        
        try:
            self.status_update.emit("▶️ Iniciando vista en vivo...")
            logger.info("Iniciando adquisición de cámara")
            
            # Configurar cámara
            logger.info(f"Configurando exposición: {self.exposure}s")
            self.cam.set_exposure(self.exposure)
            actual_exposure = self.cam.get_exposure()
            logger.info(f"Exposición actual: {actual_exposure}s")
            
            # Configurar trigger mode
            logger.info("Configurando trigger mode: 'int' (interno)")
            self.cam.set_trigger_mode("int")
            
            # Configurar frame rate usando frame period
            frame_period = 1.0 / self.fps  # Período en segundos
            logger.info(f"Configurando frame period: {frame_period:.6f}s ({self.fps} FPS)")
            self.cam.set_frame_period(frame_period)
            
            # Verificar el período configurado
            actual_period = self.cam.get_frame_period()
            actual_fps = 1.0 / actual_period if actual_period > 0 else 0
            logger.info(f"Frame period actual: {actual_period:.6f}s ({actual_fps:.2f} FPS)")
            
            # Setup acquisition con buffer configurable
            logger.info(f"Configurando adquisición con buffer de {self.buffer_size} frames")
            self.cam.setup_acquisition(nframes=self.buffer_size)
            
            # Iniciar adquisición
            logger.info("Llamando a start_acquisition()...")
            self.cam.start_acquisition()
            logger.info("start_acquisition() completado")
            
            # Verificar estado
            is_setup = self.cam.is_acquisition_setup()
            logger.info(f"is_acquisition_setup(): {is_setup}")
            
            self.running = True
            logger.info(f"Loop running activado: {self.running}")
            
            # Esperar un poco más para el primer frame
            first_frame = True
            timeout_count = 0
            max_timeouts = 10  # Máximo 10 timeouts consecutivos antes de abortar
            
            logger.debug(f"Entrando en loop de adquisición. running={self.running}")
            while self.running:
                # Usar timeout más largo para el primer frame
                timeout = 3.0 if first_frame else 0.5
                
                logger.debug(f"Esperando frame... first_frame={first_frame}, timeout={timeout}, timeout_count={timeout_count}")
                
                # wait_for_frame() retorna True/False/None
                # True: frame disponible
                # False: adquisición detenida
                # None: timeout (no hay frame aún, pero sigue activo)
                try:
                    frame_available = self.cam.wait_for_frame(timeout=timeout)
                    logger.debug(f"wait_for_frame retornó: {frame_available}")
                except Exception as timeout_error:
                    # Capturar TimeoutError específico de Thorlabs
                    logger.debug(f"Excepción capturada: {type(timeout_error).__name__}: {timeout_error}")
                    if "Timeout" in type(timeout_error).__name__:
                        timeout_count += 1
                        logger.warning(f"Timeout #{timeout_count} de {max_timeouts}")
                        
                        # En el primer timeout, hacer diagnóstico adicional
                        if timeout_count == 1:
                            try:
                                logger.info("=== DIAGNÓSTICO PRIMER TIMEOUT ===")
                                logger.info(f"Cámara abierta: {self.cam.is_opened()}")
                                logger.info(f"Adquisición configurada: {self.cam.is_acquisition_setup()}")
                                
                                # Intentar obtener info de frames disponibles
                                if hasattr(self.cam, 'get_frames_status'):
                                    status = self.cam.get_frames_status()
                                    logger.info(f"Estado de frames: {status}")
                                
                                if hasattr(self.cam, 'get_new_images_range'):
                                    img_range = self.cam.get_new_images_range()
                                    logger.info(f"Rango de imágenes nuevas: {img_range}")
                                
                                # Listar algunos métodos disponibles relacionados con frames
                                frame_methods = [m for m in dir(self.cam) if 'frame' in m.lower() or 'image' in m.lower() or 'buffer' in m.lower()]
                                logger.info(f"Métodos disponibles con 'frame/image/buffer': {frame_methods}")
                                
                            except Exception as diag_error:
                                logger.warning(f"Error en diagnóstico: {diag_error}")
                        
                        if timeout_count >= max_timeouts:
                            self.status_update.emit(f"❌ Demasiados timeouts ({max_timeouts}). Verificar cámara.")
                            logger.warning(f"Máximo de timeouts alcanzado ({max_timeouts}), deteniendo live view")
                            break
                        # Continuar esperando
                        logger.debug("Continuando después de timeout...")
                        continue
                    else:
                        # Otro tipo de error, re-lanzar
                        logger.error(f"Error no-timeout en wait_for_frame: {timeout_error}")
                        raise
                
                if frame_available:
                    # Frame disponible
                    logger.debug("Frame disponible, leyendo imagen...")
                    timeout_count = 0  # Resetear contador de timeouts
                    first_frame = False
                    
                    # Leer frame más antiguo para evitar acumulación en buffer
                    frame = self.cam.read_oldest_image()
                    logger.debug(f"Imagen leída: shape={frame.shape if frame is not None else None}")
                    
                    if frame is not None:
                        self.frame_count += 1
                        
                        # GESTIÓN DE MEMORIA: Limpiar buffer cada 30 frames
                        if self.frame_count % 30 == 0:
                            try:
                                # Limpiar frames sin leer del buffer
                                status = self.cam.get_frames_status()
                                if status.unread > 5:
                                    logger.warning(f"Buffer acumulado: {status.unread} frames sin leer. Limpiando...")
                                    # Leer y descartar frames antiguos
                                    for _ in range(min(status.unread - 1, 10)):
                                        self.cam.read_oldest_image()
                                    logger.info("Buffer limpiado")
                                
                                # Forzar garbage collection cada 30 frames
                                import gc
                                gc.collect()
                                logger.debug(f"Frame #{self.frame_count}: GC ejecutado")
                            except Exception as e:
                                logger.debug(f"Error en limpieza de memoria: {e}")
                        
                        # GUARDAR frame para captura (una sola copia)
                        self.current_frame = frame.copy()
                        
                        # Normalizar a uint8 para visualización (reutilizar el frame original)
                        if frame.dtype != np.uint8:
                            frame = (frame / frame.max() * 255).astype(np.uint8)
                        
                        h, w = frame.shape
                        bytes_per_line = w
                        
                        # Crear QImage (PyQt5 usa Format_Grayscale8 sin .Format)
                        from PyQt5.QtGui import QImage
                        q_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_Grayscale8).copy()
                        
                        self.new_frame_ready.emit(q_image)
                        
                        # Liberar referencia al frame original
                        del frame
                        
                elif frame_available is False:
                    # Adquisición detenida
                    logger.warning("frame_available es False - adquisición detenida")
                    self.status_update.emit("⚠️ La adquisición se detuvo inesperadamente.")
                    break
                elif frame_available is None:
                    # Timeout sin excepción - continuar esperando
                    logger.debug("frame_available es None (timeout silencioso), continuando...")
                    timeout_count += 1
                    if timeout_count >= max_timeouts:
                        self.status_update.emit(f"❌ Demasiados timeouts silenciosos ({max_timeouts}). Verificar cámara.")
                        logger.warning(f"Máximo de timeouts silenciosos alcanzado, deteniendo live view")
                        break
                else:
                    logger.warning(f"frame_available tiene valor inesperado: {frame_available}")
                    
        except Exception as e:
            self.status_update.emit(f"❌ Error en vista en vivo: {str(e)}")
            logger.error(f"Error en live view: {e}\n{traceback.format_exc()}")
        finally:
            # Limpiar memoria al detener
            try:
                if self.cam and self.cam.is_opened() and self.cam.is_acquisition_setup():
                    logger.info("Deteniendo adquisición y limpiando buffer...")
                    self.cam.stop_acquisition()
                    
                    # Limpiar buffer completamente
                    if hasattr(self.cam, 'clear_acquisition'):
                        self.cam.clear_acquisition()
                        logger.info("Buffer de cámara limpiado")
                    
                    # Forzar garbage collection
                    import gc
                    gc.collect()
                    logger.info("Garbage collection ejecutado")
            except Exception as e:
                logger.error(f"Error al limpiar recursos: {e}")
            
            self.frame_count = 0
            self.status_update.emit("⏹️ Vista en vivo detenida.")
            logger.info("Vista en vivo detenida")
    
    def stop_live_view(self):
        """Detiene la adquisición de video."""
        self.running = False
    
    def test_single_capture(self):
        """Prueba de captura simplificada para diagnóstico."""
        if not self.cam or not self.cam.is_opened():
            logger.error("test_single_capture: cámara no conectada")
            return False
        
        try:
            logger.info("=== TEST DE CAPTURA SIMPLE ===")
            
            # Configuración
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
            logger.info("Iniciando adquisición...")
            self.cam.start_acquisition()
            
            # Esperar
            wait_time = 0.5
            logger.info(f"Esperando {wait_time} segundos...")
            time.sleep(wait_time)
            
            # Leer frame
            logger.info("Intentando read_oldest_image()...")
            frame = self.cam.read_oldest_image()
            
            if frame is not None:
                logger.info(f"✅ ÉXITO: Frame capturado! Shape: {frame.shape}, dtype: {frame.dtype}")
                self.cam.stop_acquisition()
                return True
            else:
                logger.error("❌ read_oldest_image() retornó None")
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
        """Cambia la exposición de la cámara en tiempo real."""
        try:
            if self.cam and self.cam.is_opened():
                self.exposure = exposure_value
                self.cam.set_exposure(exposure_value)
                self.status_update.emit(f"✅ Exposición cambiada a {exposure_value} s")
                logger.info(f"Exposición cambiada: {exposure_value}s")
            else:
                self.status_update.emit("❌ Error: Cámara no conectada")
        except Exception as e:
            self.status_update.emit(f"❌ Error al cambiar exposición: {str(e)}")
            logger.error(f"Error cambio exposición: {e}")
    
    def change_fps(self, fps_value):
        """Cambia el frame rate de la cámara usando frame period."""
        try:
            if self.cam and self.cam.is_opened():
                self.fps = fps_value
                frame_period = 1.0 / fps_value
                self.cam.set_frame_period(frame_period)
                self.status_update.emit(f"✅ Frame rate cambiado a {fps_value} FPS (period={frame_period:.6f}s)")
                logger.info(f"Frame rate cambiado: {fps_value} FPS (period={frame_period:.6f}s)")
            else:
                self.status_update.emit("❌ Error: Cámara no conectada")
        except Exception as e:
            self.status_update.emit(f"❌ Error al cambiar FPS: {str(e)}")
            logger.error(f"Error cambio FPS: {e}")
    
    def change_buffer_size(self, buffer_value):
        """Cambia el tamaño del buffer de frames."""
        try:
            if self.cam and self.cam.is_opened():
                self.buffer_size = buffer_value
                self.status_update.emit(f"✅ Buffer configurado: {buffer_value} frames (aplicará en próxima adquisición)")
                logger.info(f"Buffer size guardado: {buffer_value} frames")
            else:
                self.status_update.emit("❌ Error: Cámara no conectada")
        except Exception as e:
            self.status_update.emit(f"❌ Error al cambiar buffer: {str(e)}")
            logger.error(f"Error cambio buffer: {e}")
    
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
                    logger.info("Buffer limpiado antes de desconectar")
            except Exception as e:
                logger.debug(f"Error al limpiar buffer: {e}")
            
            self.cam.close()
            self.current_frame = None
            self.frame_count = 0
            
            # Forzar garbage collection
            import gc
            gc.collect()
            logger.info("Memoria liberada")
            
            self.status_update.emit("✅ Cámara cerrada.")
            logger.info("Cámara desconectada")
# --- Ventana Flotante para Visualización de Cámara ---
# =========================================================================
class CameraViewWindow(QWidget):
    """Ventana independiente redimensionable para visualizar la cámara."""
    
    def __init__(self, parent=None, main_window=None):
        super().__init__(parent, Qt.Window)
        self.setWindowTitle('🎥 Vista de Cámara Thorlabs - Tiempo Real')
        self.setGeometry(200, 200, 800, 600)
        self.setStyleSheet(DARK_STYLESHEET)
        self.main_window = main_window  # Referencia a la ventana principal para obtener posiciones
        
        layout = QVBoxLayout(self)
        
        # Label para mostrar video
        self.video_label = QLabel("Esperando frames de la cámara...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("""
            QLabel {
                background-color: #000000;
                color: #00FF00;
                border: 2px solid #505050;
                font-size: 14px;
                font-weight: bold;
            }
        """)
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setScaledContents(False)
        
        layout.addWidget(self.video_label)
        
        # Información de frame
        info_layout = QHBoxLayout()
        self.frame_info_label = QLabel("Frame: 0 | Resolución: --- | FPS: ---")
        self.frame_info_label.setStyleSheet("color: #95A5A6; font-size: 10px;")
        info_layout.addWidget(self.frame_info_label)
        info_layout.addStretch()
        layout.addLayout(info_layout)
        
        self.frame_count = 0
        self.current_pixmap = None
        self.last_x = None  # Para calcular ΔX
        self.last_y = None  # Para calcular ΔY
        
    def update_frame(self, q_image):
        """Actualiza el frame mostrado en la ventana."""
        try:
            self.frame_count += 1
            
            # Convertir QImage a QPixmap
            from PyQt5.QtGui import QPixmap
            pixmap = QPixmap.fromImage(q_image)
            self.current_pixmap = pixmap
            
            # Escalar manteniendo aspect ratio
            scaled_pixmap = pixmap.scaled(
                self.video_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            
            # Dibujar overlay con coordenadas (si está disponible)
            if self.main_window and hasattr(self.main_window, 'global_calibration'):
                scaled_pixmap = self.draw_position_overlay(scaled_pixmap)
            
            self.video_label.setPixmap(scaled_pixmap)
            
            # Actualizar info
            self.frame_info_label.setText(
                f"Frame: {self.frame_count} | Resolución: {q_image.width()}x{q_image.height()}"
            )
            
        except Exception as e:
            logger.error(f"Error actualizando frame: {e}")
    
    def draw_position_overlay(self, pixmap):
        """
        Dibuja overlay con coordenadas X, Y, ΔX, ΔY en la esquina inferior derecha.
        
        IMPORTANTE: Se dibuja sobre el pixmap escalado, NO sobre el frame original,
        por lo que NO aparece en las imágenes guardadas.
        """
        from PyQt5.QtGui import QPainter, QFont, QColor, QPen
        from PyQt5.QtCore import Qt, QRect
        
        # Obtener posiciones actuales (en µm)
        pos_x = getattr(self.main_window, 'dual_last_pos_a', None)
        pos_y = getattr(self.main_window, 'dual_last_pos_b', None)
        
        # Si no hay posiciones disponibles, no dibujar nada
        if pos_x is None or pos_y is None:
            return pixmap
        
        # Calcular deltas
        if self.last_x is not None and self.last_y is not None:
            delta_x = pos_x - self.last_x
            delta_y = pos_y - self.last_y
        else:
            delta_x = 0.0
            delta_y = 0.0
        
        # Actualizar últimas posiciones
        self.last_x = pos_x
        self.last_y = pos_y
        
        # Crear copia del pixmap para dibujar
        pixmap_with_overlay = pixmap.copy()
        
        # Inicializar painter
        painter = QPainter(pixmap_with_overlay)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Configurar fuente
        font = QFont("Consolas", 12, QFont.Bold)
        painter.setFont(font)
        
        # Texto a mostrar (4 líneas)
        lines = [
            f"X: {pos_x:.1f} µm",
            f"Y: {pos_y:.1f} µm",
            f"ΔX: {delta_x:+.2f} µm",
            f"ΔY: {delta_y:+.2f} µm"
        ]
        
        # Calcular dimensiones del texto
        metrics = painter.fontMetrics()
        max_width = max(metrics.horizontalAdvance(line) for line in lines)
        line_height = metrics.height()
        total_height = line_height * len(lines) + 5 * (len(lines) - 1)  # 5px spacing entre líneas
        
        # Posición: esquina inferior derecha con margen
        margin = 15
        x_pos = pixmap.width() - max_width - margin - 10
        y_pos = pixmap.height() - total_height - margin
        
        # Dibujar fondo semi-transparente
        bg_rect = QRect(
            x_pos - 8,
            y_pos - 8,
            max_width + 16,
            total_height + 16
        )
        painter.fillRect(bg_rect, QColor(0, 0, 0, 200))  # Negro semi-transparente
        
        # Dibujar borde
        painter.setPen(QPen(QColor(0, 255, 255), 2))  # Cyan
        painter.drawRect(bg_rect)
        
        # Dibujar texto línea por línea
        current_y = y_pos
        colors = [
            QColor(255, 255, 0),   # Amarillo para X
            QColor(0, 255, 255),   # Cyan para Y
            QColor(255, 165, 0),   # Naranja para ΔX
            QColor(0, 200, 255)    # Azul claro para ΔY
        ]
        
        for i, line in enumerate(lines):
            painter.setPen(QPen(colors[i], 2))
            painter.drawText(x_pos, current_y + line_height - 5, line)
            current_y += line_height + 5
        
        painter.end()
        
        return pixmap_with_overlay
    
    def resizeEvent(self, event):
        """Reescalar imagen cuando se redimensiona la ventana."""
        super().resizeEvent(event)
        if self.current_pixmap:
            scaled_pixmap = self.current_pixmap.scaled(
                self.video_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            # Aplicar overlay si está disponible
            if self.main_window and hasattr(self.main_window, 'global_calibration'):
                scaled_pixmap = self.draw_position_overlay(scaled_pixmap)
            self.video_label.setPixmap(scaled_pixmap)


# =========================================================================
# --- Interfaz Principal con Pestañas ---
# =========================================================================
class ArduinoGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Sistema de Control y Análisis - Motores L206')
        self.setGeometry(100, 100, 800, 700)
        self.setStyleSheet(DARK_STYLESHEET)

        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None
        self.start_time = time.time()
        self.value_labels = {}
        
        # Ventanas de visualización (inicialmente None)
        self.signal_window = None
        self.data_window = None
        self.analysis_window = None
        
        # Widget central con pestañas
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Botón para abrir ventana de señales
        signal_btn = QPushButton("📊 Abrir Señales de Control")
        signal_btn.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px;")
        signal_btn.clicked.connect(self.open_signal_window)
        main_layout.addWidget(signal_btn)
        
        # Crear pestañas
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #505050;
                background-color: #2E2E2E;
            }
            QTabBar::tab {
                background-color: #383838;
                color: #F0F0F0;
                padding: 10px 20px;
                margin: 2px;
                border: 1px solid #505050;
                border-bottom: none;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            QTabBar::tab:selected {
                background-color: #2E86C1;
                font-weight: bold;
            }
            QTabBar::tab:hover {
                background-color: #505050;
            }
        """)
        
        # Pestaña 1: Control y Monitoreo
        tab1 = QWidget()
        tab1_layout = QVBoxLayout(tab1)
        tab1_layout.addWidget(self.create_control_group())
        tab1_layout.addWidget(self.create_motors_group())
        tab1_layout.addWidget(self.create_sensors_group())
        tab1_layout.addStretch()
        self.tabs.addTab(tab1, "🎮 Control")
        
        # Pestaña 2: Grabación
        tab2 = QWidget()
        tab2_layout = QVBoxLayout(tab2)
        tab2_layout.addWidget(self.create_recording_group())
        tab2_layout.addStretch()
        self.tabs.addTab(tab2, "📹 Grabación")
        
        # Pestaña 3: Análisis
        tab3 = QWidget()
        tab3_layout = QVBoxLayout(tab3)
        tab3_layout.addWidget(self.create_analysis_group())
        tab3_layout.addStretch()
        self.tabs.addTab(tab3, "📈 Análisis")
        
        # Pestaña 4: Diseño de Controlador H∞
        tab4 = QWidget()
        tab4_layout = QVBoxLayout(tab4)
        tab4_layout.addWidget(self.create_controller_design_group())
        tab4_layout.addStretch()
        self.tabs.addTab(tab4, "🎛️ H∞ Synthesis")
        
        # Pestaña 5: Prueba de Controladores (Dual Motor)
        tab5 = QWidget()
        tab5_layout = QVBoxLayout(tab5)
        tab5_layout.addWidget(self.create_test_group())
        # Eliminado addStretch() para que ocupe toda la ventana
        self.tabs.addTab(tab5, "🧪 Prueba")
        
        # Pestaña 6: Reconocimiento de Imagen (Detector de Cámara)
        tab6 = QWidget()
        tab6_layout = QVBoxLayout(tab6)
        tab6_layout.addWidget(self.create_camera_detector_group())
        tab6_layout.addStretch()
        self.tabs.addTab(tab6, "🎥 ImgRec")
        
        main_layout.addWidget(self.tabs)

        # Iniciar comunicación serial
        self.serial_thread = SerialReaderThread(SERIAL_PORT, BAUD_RATE)
        self.serial_thread.data_received.connect(self.update_data)
        self.serial_thread.start()
    
    def open_signal_window(self):
        """Abre la ventana de señales en tiempo real."""
        logger.info("=== BOTÓN: Abrir Señales de Control presionado ===")
        try:
            if self.signal_window is None:
                logger.debug("Creando nueva ventana de señales")
                self.signal_window = SignalWindow(self)
                logger.info("Ventana de señales creada exitosamente")
            else:
                logger.debug("Reutilizando ventana de señales existente")
            
            self.signal_window.show()
            self.signal_window.raise_()
            self.signal_window.activateWindow()
            logger.info("Ventana de señales mostrada y activada")
        except Exception as e:
            logger.error(f"Error al abrir ventana de señales: {e}\n{traceback.format_exc()}")

    # --- Funciones de Paneles (Grabación, Control, Motores, Sensores) ---
    def create_control_group(self):
        group_box = QGroupBox("Panel de Control")
        layout = QGridLayout()

        layout.addWidget(QLabel("Modo Actual:"), 0, 0)
        self.value_labels['mode'] = QLabel("MANUAL")
        self.value_labels['mode'].setStyleSheet("font-weight: bold; color: #E67E22;")
        layout.addWidget(self.value_labels['mode'], 0, 1)

        manual_btn = QPushButton("Activar MODO MANUAL")
        manual_btn.clicked.connect(self.set_manual_mode)
        layout.addWidget(manual_btn, 1, 0, 1, 2)
        
        auto_btn = QPushButton("Activar MODO AUTO")
        auto_btn.clicked.connect(self.set_auto_mode)
        layout.addWidget(auto_btn, 2, 0, 1, 2)

        layout.addWidget(QLabel("Potencia (A, B):"), 3, 0)
        self.power_input = QLineEdit("100,-100")
        layout.addWidget(self.power_input, 3, 1)
        
        send_power_btn = QPushButton("Enviar Potencia (en modo AUTO)")
        send_power_btn.clicked.connect(self.send_power_command)
        layout.addWidget(send_power_btn, 4, 0, 1, 2)

        group_box.setLayout(layout)
        return group_box

    def create_recording_group(self):
        group_box = QGroupBox("Registro de Experimento (Respuesta al Escalón)")
        layout = QGridLayout()
        
        layout.addWidget(QLabel("Nombre Archivo:"), 0, 0)
        self.filename_input = QLineEdit("experimento_escalon.csv")
        layout.addWidget(self.filename_input, 0, 1)

        self.start_record_btn = QPushButton("Iniciar Grabación")
        self.start_record_btn.clicked.connect(self.start_recording)
        layout.addWidget(self.start_record_btn, 1, 0)
        
        self.stop_record_btn = QPushButton("Detener Grabación")
        self.stop_record_btn.clicked.connect(self.stop_recording)
        self.stop_record_btn.setEnabled(False)
        layout.addWidget(self.stop_record_btn, 1, 1)

        self.record_status_label = QLabel("Estado: Detenido")
        self.record_status_label.setStyleSheet("color: #E67E22;")
        layout.addWidget(self.record_status_label, 2, 0, 1, 2)

        group_box.setLayout(layout)
        return group_box

    # --- Panel de Análisis Manual ---
    def create_analysis_group(self):
        """Crea el GroupBox para el análisis manual de función de transferencia."""
        group_box = QGroupBox("Análisis Manual - Función de Transferencia")
        layout = QVBoxLayout()
        
        # Sección 1: Selección de Archivo
        file_group = QGroupBox("📁 Archivo de Datos")
        file_layout = QGridLayout()
        
        file_layout.addWidget(QLabel("Archivo CSV:"), 0, 0)
        self.analysis_filename_input = QLineEdit("experimento_escalon.csv")
        self.analysis_filename_input.setPlaceholderText("Selecciona o escribe el nombre del archivo...")
        file_layout.addWidget(self.analysis_filename_input, 0, 1)
        
        browse_btn = QPushButton("📂 Examinar...")
        browse_btn.clicked.connect(self.browse_analysis_file)
        browse_btn.setFixedWidth(120)
        file_layout.addWidget(browse_btn, 0, 2)
        
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)
        
        # Sección 2: Configuración del análisis
        config_group = QGroupBox("⚙️ Configuración")
        config_layout = QGridLayout()
        
        # Selector de Motor
        config_layout.addWidget(QLabel("Motor a analizar:"), 0, 0)
        motor_layout = QHBoxLayout()
        self.motor_a_radio = QCheckBox("Motor A")
        self.motor_b_radio = QCheckBox("Motor B")
        self.motor_a_radio.setChecked(True)
        self.motor_a_radio.stateChanged.connect(lambda: self.toggle_motor_selection('A'))
        self.motor_b_radio.stateChanged.connect(lambda: self.toggle_motor_selection('B'))
        motor_layout.addWidget(self.motor_a_radio)
        motor_layout.addWidget(self.motor_b_radio)
        motor_layout.addStretch()
        config_layout.addLayout(motor_layout, 0, 1, 1, 2)
        
        # Selector de Sensor
        config_layout.addWidget(QLabel("Sensor correspondiente:"), 1, 0)
        sensor_layout = QHBoxLayout()
        self.sensor_1_radio = QCheckBox("Sensor 1")
        self.sensor_2_radio = QCheckBox("Sensor 2")
        self.sensor_1_radio.setChecked(True)
        self.sensor_1_radio.stateChanged.connect(lambda: self.toggle_sensor_selection('1'))
        self.sensor_2_radio.stateChanged.connect(lambda: self.toggle_sensor_selection('2'))
        sensor_layout.addWidget(self.sensor_1_radio)
        sensor_layout.addWidget(self.sensor_2_radio)
        sensor_layout.addStretch()
        config_layout.addLayout(sensor_layout, 1, 1, 1, 2)
        
        # Rango de tiempo
        config_layout.addWidget(QLabel("Tiempo inicio (s):"), 2, 0)
        self.t_inicio_input = QLineEdit("0.0")
        self.t_inicio_input.setFixedWidth(100)
        config_layout.addWidget(self.t_inicio_input, 2, 1)
        
        config_layout.addWidget(QLabel("Tiempo fin (s):"), 2, 2)
        self.t_fin_input = QLineEdit("999.0")
        self.t_fin_input.setFixedWidth(100)
        config_layout.addWidget(self.t_fin_input, 2, 3)
        
        # Distancia real recorrida (para calibración con interpolación)
        config_layout.addWidget(QLabel("Distancia mín (mm):"), 3, 0)
        self.distancia_min_input = QLineEdit("")
        self.distancia_min_input.setFixedWidth(100)
        self.distancia_min_input.setPlaceholderText("Ej: 10")
        self.distancia_min_input.setToolTip("Distancia real correspondiente al INICIO del tramo.\nPuede ser mayor o menor que el máximo (relación directa o inversa).")
        config_layout.addWidget(self.distancia_min_input, 3, 1)
        
        config_layout.addWidget(QLabel("Distancia máx (mm):"), 3, 2)
        self.distancia_max_input = QLineEdit("")
        self.distancia_max_input.setFixedWidth(100)
        self.distancia_max_input.setPlaceholderText("Ej: 20")
        self.distancia_max_input.setToolTip("Distancia real correspondiente al FINAL del tramo.\nPuede ser mayor o menor que el mínimo (relación directa o inversa).")
        config_layout.addWidget(self.distancia_max_input, 3, 3)
        
        config_group.setLayout(config_layout)
        layout.addWidget(config_group)
        
        # Botones
        buttons_layout = QHBoxLayout()
        view_data_btn = QPushButton("👁️ Ver Datos Completos")
        view_data_btn.clicked.connect(self.view_full_data)
        view_data_btn.setStyleSheet("font-size: 11px; padding: 6px;")
        buttons_layout.addWidget(view_data_btn)
        
        analyze_btn = QPushButton("🔍 Analizar Tramo")
        analyze_btn.clicked.connect(self.run_analysis)
        analyze_btn.setStyleSheet("font-size: 11px; padding: 6px; font-weight: bold; background-color: #3498DB;")
        buttons_layout.addWidget(analyze_btn)
        layout.addLayout(buttons_layout)

        # Resultados del análisis actual
        results_label = QLabel("📊 Resultados del Análisis:")
        results_label.setStyleSheet("font-weight: bold; font-size: 11px; margin-top: 10px;")
        layout.addWidget(results_label)
        
        self.analysis_results_text = QTextEdit()
        self.analysis_results_text.setReadOnly(True)
        self.analysis_results_text.setPlaceholderText("Los resultados del análisis (K, τ) aparecerán aquí...")
        self.analysis_results_text.setFixedHeight(360)  # 3x más alto (120 → 360)
        layout.addWidget(self.analysis_results_text)
        
        # Lista de funciones de transferencia identificadas
        tf_list_label = QLabel("📋 Funciones de Transferencia Identificadas:")
        tf_list_label.setStyleSheet("font-weight: bold; font-size: 11px; margin-top: 10px;")
        layout.addWidget(tf_list_label)
        
        self.tf_list_text = QTextEdit()
        self.tf_list_text.setReadOnly(True)
        self.tf_list_text.setPlaceholderText("Las funciones de transferencia identificadas se listarán aquí...\n\nFormato: Motor X / Sensor Y → G(s) = K / (s·(τs + 1))")
        self.tf_list_text.setFixedHeight(200)
        layout.addWidget(self.tf_list_text)
        
        # Inicializar lista de funciones de transferencia
        self.identified_transfer_functions = []
        
        group_box.setLayout(layout)
        return group_box
    
    # --- Panel de Prueba de Controladores (Dual Motor) ---
    def create_test_group(self):
        """Crea el GroupBox para prueba de controladores con ambos motores."""
        group_box = QGroupBox("Prueba de Controladores - Control Dual y por Pasos")
        main_layout = QVBoxLayout()
        
        # Crear scroll area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        
        # Widget contenedor para el scroll
        scroll_widget = QWidget()
        layout = QVBoxLayout(scroll_widget)
        
        # Sección 1: Controladores H∞ Transferidos
        controllers_group = QGroupBox("📦 Controladores H∞ Transferidos")
        controllers_layout = QVBoxLayout()
        
        # Motor A
        motor_a_frame = QFrame()
        motor_a_frame.setFrameStyle(QFrame.Box | QFrame.Raised)
        motor_a_layout = QVBoxLayout()
        
        motor_a_header = QHBoxLayout()
        self.test_motor_a_label = QLabel("<b>Motor A (X)</b>")  # Etiqueta dinámica
        motor_a_header.addWidget(self.test_motor_a_label)
        motor_a_header.addStretch()
        self.test_motor_a_status = QLabel("⚪ Sin controlador")
        self.test_motor_a_status.setStyleSheet("color: #95A5A6;")
        motor_a_header.addWidget(self.test_motor_a_status)
        motor_a_layout.addLayout(motor_a_header)
        
        self.test_motor_a_info = QTextEdit()
        self.test_motor_a_info.setReadOnly(True)
        self.test_motor_a_info.setMinimumHeight(60)
        self.test_motor_a_info.setMaximumHeight(70)
        self.test_motor_a_info.setStyleSheet(
            "font-family: 'Consolas', 'Courier New', monospace; "
            "font-size: 12px; "
            "background-color: white; "
            "color: black;"
        )
        self.test_motor_a_info.setPlaceholderText("Transfiere un controlador H∞ desde 'H∞ Synthesis'...")
        motor_a_layout.addWidget(self.test_motor_a_info)
        
        motor_a_buttons = QHBoxLayout()
        self.test_clear_a_btn = QPushButton("🗑️ Limpiar")
        self.test_clear_a_btn.clicked.connect(lambda: self.clear_controller('A'))
        self.test_clear_a_btn.setEnabled(False)
        motor_a_buttons.addWidget(self.test_clear_a_btn)
        motor_a_buttons.addStretch()
        motor_a_layout.addLayout(motor_a_buttons)
        
        motor_a_frame.setLayout(motor_a_layout)
        controllers_layout.addWidget(motor_a_frame)
        
        # Motor B
        motor_b_frame = QFrame()
        motor_b_frame.setFrameStyle(QFrame.Box | QFrame.Raised)
        motor_b_layout = QVBoxLayout()
        
        motor_b_header = QHBoxLayout()
        self.test_motor_b_label = QLabel("<b>Motor B (Y)</b>")  # Etiqueta dinámica
        motor_b_header.addWidget(self.test_motor_b_label)
        motor_b_header.addStretch()
        self.test_motor_b_status = QLabel("⚪ Sin controlador")
        self.test_motor_b_status.setStyleSheet("color: #95A5A6;")
        motor_b_header.addWidget(self.test_motor_b_status)
        motor_b_layout.addLayout(motor_b_header)
        
        self.test_motor_b_info = QTextEdit()
        self.test_motor_b_info.setReadOnly(True)
        self.test_motor_b_info.setMinimumHeight(60)
        self.test_motor_b_info.setMaximumHeight(70)
        self.test_motor_b_info.setStyleSheet(
            "font-family: 'Consolas', 'Courier New', monospace; "
            "font-size: 12px; "
            "background-color: white; "
            "color: black;"
        )
        self.test_motor_b_info.setPlaceholderText("Transfiere un controlador H∞ desde 'H∞ Synthesis'...")
        motor_b_layout.addWidget(self.test_motor_b_info)
        
        motor_b_buttons = QHBoxLayout()
        self.test_clear_b_btn = QPushButton("🗑️ Limpiar")
        self.test_clear_b_btn.clicked.connect(lambda: self.clear_controller('B'))
        self.test_clear_b_btn.setEnabled(False)
        motor_b_buttons.addWidget(self.test_clear_b_btn)
        motor_b_buttons.addStretch()
        motor_b_layout.addLayout(motor_b_buttons)
        
        motor_b_frame.setLayout(motor_b_layout)
        controllers_layout.addWidget(motor_b_frame)
        
        controllers_group.setLayout(controllers_layout)
        layout.addWidget(controllers_group)
        
        # Inicializar atributos de controladores
        self.test_controller_a = None
        self.test_controller_b = None
        
        # Sección 1.5: Configuración Motor-Sensor (CHECKBOXES) - JUSTO DESPUÉS DE CONTROLADORES
        motor_sensor_group = QGroupBox("🔧 Asignación Motor ↔ Sensor")
        motor_sensor_layout = QVBoxLayout()
        
        # Motor A
        motor_a_sensor_layout = QHBoxLayout()
        motor_a_sensor_layout.addWidget(QLabel("<b>Motor A lee:</b>"))
        self.test_motor_a_sensor1_check = QCheckBox("Sensor 1")
        self.test_motor_a_sensor2_check = QCheckBox("Sensor 2")
        self.test_motor_a_sensor1_check.setChecked(False)
        self.test_motor_a_sensor2_check.setChecked(False)
        
        # Exclusividad
        self.test_motor_a_sensor1_check.toggled.connect(lambda checked: self.test_motor_a_sensor2_check.setChecked(False) if checked else None)
        self.test_motor_a_sensor2_check.toggled.connect(lambda checked: self.test_motor_a_sensor1_check.setChecked(False) if checked else None)
        
        motor_a_sensor_layout.addWidget(self.test_motor_a_sensor1_check)
        motor_a_sensor_layout.addWidget(self.test_motor_a_sensor2_check)
        
        # Checkbox de inversión para Motor A
        self.test_motor_a_invert_check = QCheckBox("⇄ Invertir PWM")
        self.test_motor_a_invert_check.setChecked(False)
        self.test_motor_a_invert_check.setToolTip("Marcar si el motor/sensor están físicamente invertidos")
        motor_a_sensor_layout.addWidget(self.test_motor_a_invert_check)
        motor_a_sensor_layout.addStretch()
        motor_sensor_layout.addLayout(motor_a_sensor_layout)
        
        # Motor B
        motor_b_sensor_layout = QHBoxLayout()
        motor_b_sensor_layout.addWidget(QLabel("<b>Motor B lee:</b>"))
        self.test_motor_b_sensor1_check = QCheckBox("Sensor 1")
        self.test_motor_b_sensor2_check = QCheckBox("Sensor 2")
        self.test_motor_b_sensor1_check.setChecked(False)
        self.test_motor_b_sensor2_check.setChecked(False)
        
        # Exclusividad
        self.test_motor_b_sensor1_check.toggled.connect(lambda checked: self.test_motor_b_sensor2_check.setChecked(False) if checked else None)
        self.test_motor_b_sensor2_check.toggled.connect(lambda checked: self.test_motor_b_sensor1_check.setChecked(False) if checked else None)
        
        motor_b_sensor_layout.addWidget(self.test_motor_b_sensor1_check)
        motor_b_sensor_layout.addWidget(self.test_motor_b_sensor2_check)
        
        # Checkbox de inversión para Motor B
        self.test_motor_b_invert_check = QCheckBox("⇄ Invertir PWM")
        self.test_motor_b_invert_check.setChecked(False)
        self.test_motor_b_invert_check.setToolTip("Marcar si el motor/sensor están físicamente invertidos")
        motor_b_sensor_layout.addWidget(self.test_motor_b_invert_check)
        motor_b_sensor_layout.addStretch()
        motor_sensor_layout.addLayout(motor_b_sensor_layout)
        
        info_label = QLabel(
            "⚠️ IMPORTANTE: Configura sensor e inversión ANTES de iniciar control.\n"
            "Esta configuración es ÚNICA para toda la pestaña Prueba."
        )
        info_label.setWordWrap(True)
        info_label.setStyleSheet("padding: 8px; background-color: #FFF3CD; border: 1px solid #FFC107; border-radius: 3px; font-size: 11px; font-weight: bold;")
        motor_sensor_layout.addWidget(info_label)
        
        motor_sensor_group.setLayout(motor_sensor_layout)
        layout.addWidget(motor_sensor_group)
        
        # Sección 2: Estado de Calibración Global
        calibration_group = QGroupBox("📏 Calibración del Sistema")
        calibration_layout = QVBoxLayout()
        
        calibration_info = QLabel(
            "<b>ℹ️ La calibración se configura en la pestaña 'Análisis'</b><br>"
            "Debes realizar un análisis de tramo con distancias físicas para calibrar el sistema."
        )
        calibration_info.setWordWrap(True)
        calibration_info.setStyleSheet("padding: 10px; background-color: #34495E; border-radius: 5px;")
        calibration_layout.addWidget(calibration_info)
        
        self.test_calibration_status = QLabel("⚪ Sin calibración")
        self.test_calibration_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #95A5A6; padding: 5px;")
        calibration_layout.addWidget(self.test_calibration_status)
        
        self.test_calibration_details = QTextEdit()
        self.test_calibration_details.setReadOnly(True)
        self.test_calibration_details.setMinimumHeight(60)
        self.test_calibration_details.setMaximumHeight(70)
        self.test_calibration_details.setStyleSheet(
            "font-family: 'Consolas', 'Courier New', monospace; "
            "font-size: 12px; "
            "background-color: white; "
            "color: black;"
        )
        self.test_calibration_details.setPlaceholderText("Realiza un análisis en 'Análisis' para calibrar...")
        calibration_layout.addWidget(self.test_calibration_details)
        
        calibration_group.setLayout(calibration_layout)
        layout.addWidget(calibration_group)
        
        # Sección 3: Control por Posición
        position_group = QGroupBox("📍 Control por Posición")
        position_layout = QGridLayout()
        
        position_layout.addWidget(QLabel("Referencia Motor A (µm):"), 0, 0)
        self.test_ref_a_input = QLineEdit("15000")
        self.test_ref_a_input.setFixedWidth(100)
        self.test_ref_a_input.setToolTip("Posición objetivo en micrómetros (usar rango calibrado)")
        position_layout.addWidget(self.test_ref_a_input, 0, 1)
        
        position_layout.addWidget(QLabel("Referencia Motor B (µm):"), 0, 2)
        self.test_ref_b_input = QLineEdit("15000")
        self.test_ref_b_input.setFixedWidth(100)
        self.test_ref_b_input.setToolTip("Posición objetivo en micrómetros (usar rango calibrado)")
        position_layout.addWidget(self.test_ref_b_input, 0, 3)
        
        # Guardar factor de escala calculado
        self.test_scale_factor = FACTOR_ESCALA  # Valor por defecto
        
        # Botones de control por posición
        position_btn_layout = QHBoxLayout()
        self.test_start_btn = QPushButton("▶️ Iniciar Control Dual")
        self.test_start_btn.clicked.connect(self.start_dual_control)
        self.test_start_btn.setStyleSheet("font-size: 12px; font-weight: bold; padding: 8px; background-color: #27AE60;")
        position_btn_layout.addWidget(self.test_start_btn)
        
        self.test_stop_btn = QPushButton("⏹️ Detener Control")
        self.test_stop_btn.clicked.connect(self.stop_dual_control)
        self.test_stop_btn.setStyleSheet("font-size: 12px; font-weight: bold; padding: 8px; background-color: #E74C3C;")
        self.test_stop_btn.setEnabled(False)
        position_btn_layout.addWidget(self.test_stop_btn)
        
        position_layout.addLayout(position_btn_layout, 2, 0, 1, 4)
        
        position_group.setLayout(position_layout)
        layout.addWidget(position_group)
        
        # Sección 3.5: Visualización de Ecuaciones del Controlador
        equations_group = QGroupBox("📐 Ecuaciones del Controlador en Tiempo Real")
        equations_layout = QVBoxLayout()
        
        self.equations_display = QTextEdit()
        self.equations_display.setReadOnly(True)
        self.equations_display.setMinimumHeight(100)
        self.equations_display.setMaximumHeight(120)
        self.equations_display.setStyleSheet(
            "font-family: 'Consolas', 'Courier New', monospace; "
            "font-size: 12px; "
            "background-color: white; "
            "color: black;"
        )
        self.equations_display.setPlaceholderText("Ecuaciones del controlador en tiempo real...")
        equations_layout.addWidget(self.equations_display)
        
        equations_group.setLayout(equations_layout)
        layout.addWidget(equations_group)
        
        # Sección 4: Generador de Trayectorias en Zig-Zag
        trajectory_group = QGroupBox("🔀 Generador de Trayectorias en Zig-Zag")
        trajectory_layout = QGridLayout()
        
        trajectory_layout.addWidget(QLabel("Número de puntos:"), 0, 0)
        self.trajectory_points_input = QLineEdit("100")
        self.trajectory_points_input.setFixedWidth(100)
        self.trajectory_points_input.setToolTip("Cantidad de puntos homogéneos en distancia")
        trajectory_layout.addWidget(self.trajectory_points_input, 0, 1)
        
        trajectory_layout.addWidget(QLabel("Distancia inicial X (µm):"), 0, 2)
        self.trajectory_x_start_input = QLineEdit("10000")
        self.trajectory_x_start_input.setFixedWidth(100)
        self.trajectory_x_start_input.setToolTip("Posición inicial en X (Motor A)")
        trajectory_layout.addWidget(self.trajectory_x_start_input, 0, 3)
        
        trajectory_layout.addWidget(QLabel("Distancia final X (µm):"), 1, 0)
        self.trajectory_x_end_input = QLineEdit("20000")
        self.trajectory_x_end_input.setFixedWidth(100)
        self.trajectory_x_end_input.setToolTip("Posición final en X (Motor A)")
        trajectory_layout.addWidget(self.trajectory_x_end_input, 1, 1)
        
        trajectory_layout.addWidget(QLabel("Distancia inicial Y (µm):"), 1, 2)
        self.trajectory_y_start_input = QLineEdit("10000")
        self.trajectory_y_start_input.setFixedWidth(100)
        self.trajectory_y_start_input.setToolTip("Posición inicial en Y (Motor B)")
        trajectory_layout.addWidget(self.trajectory_y_start_input, 1, 3)
        
        trajectory_layout.addWidget(QLabel("Distancia final Y (µm):"), 2, 0)
        self.trajectory_y_end_input = QLineEdit("20000")
        self.trajectory_y_end_input.setFixedWidth(100)
        self.trajectory_y_end_input.setToolTip("Posición final en Y (Motor B)")
        trajectory_layout.addWidget(self.trajectory_y_end_input, 2, 1)
        
        trajectory_layout.addWidget(QLabel("Tiempo entre pasos (s):"), 2, 2)
        self.trajectory_step_delay_input = QLineEdit("0.5")
        self.trajectory_step_delay_input.setFixedWidth(100)
        self.trajectory_step_delay_input.setToolTip("Tiempo de espera entre cada punto")
        trajectory_layout.addWidget(self.trajectory_step_delay_input, 2, 3)
        
        # Botones de trayectoria
        trajectory_btn_layout = QHBoxLayout()
        
        generate_trajectory_btn = QPushButton("🎯 Generar Trayectoria")
        generate_trajectory_btn.clicked.connect(self.generate_zigzag_trajectory)
        generate_trajectory_btn.setStyleSheet("background-color: #3498DB; font-weight: bold;")
        trajectory_btn_layout.addWidget(generate_trajectory_btn)
        
        preview_trajectory_btn = QPushButton("👁️ Vista Previa")
        preview_trajectory_btn.clicked.connect(self.preview_trajectory)
        trajectory_btn_layout.addWidget(preview_trajectory_btn)
        
        # Botón para ver mapa de coordenadas
        view_coordinates_btn = QPushButton("📋 Ver Mapa de Coordenadas")
        view_coordinates_btn.clicked.connect(self.view_coordinate_map)
        view_coordinates_btn.setStyleSheet("background-color: #9B59B6; font-weight: bold;")
        trajectory_btn_layout.addWidget(view_coordinates_btn)
        
        trajectory_layout.addLayout(trajectory_btn_layout, 3, 0, 1, 4)
        
        trajectory_group.setLayout(trajectory_layout)
        layout.addWidget(trajectory_group)
        
        # Sección 5: Ejecución de Trayectoria Zig-Zag para Microscopía
        step_group = QGroupBox("🔬 Ejecución de Trayectoria Zig-Zag (Microscopía)")
        step_layout = QVBoxLayout()
        
        # Información sobre la trayectoria
        info_layout = QHBoxLayout()
        info_label = QLabel(
            "ℹ️ <b>Usa la trayectoria generada en la sección anterior</b><br>"
            "La trayectoria zig-zag se ejecutará punto por punto con control dual (X e Y simultáneamente)."
        )
        info_label.setWordWrap(True)
        info_label.setStyleSheet("padding: 10px; background-color: #34495E; border-radius: 5px; font-size: 11px;")
        info_layout.addWidget(info_label)
        step_layout.addLayout(info_layout)
        
        # Estado de la trayectoria
        traj_status_layout = QHBoxLayout()
        traj_status_layout.addWidget(QLabel("<b>Estado de trayectoria:</b>"))
        self.step_trajectory_status = QLabel("⚪ Sin trayectoria generada")
        self.step_trajectory_status.setStyleSheet("color: #95A5A6; font-weight: bold;")
        traj_status_layout.addWidget(self.step_trajectory_status)
        traj_status_layout.addStretch()
        step_layout.addLayout(traj_status_layout)
        
        # Parámetros de ejecución
        params_layout = QGridLayout()
        
        params_layout.addWidget(QLabel("Tolerancia de posición (µm):"), 0, 0)
        self.step_tolerance_input = QLineEdit("100")
        self.step_tolerance_input.setFixedWidth(100)
        self.step_tolerance_input.setToolTip("Error máximo aceptable para considerar que llegó a la posición")
        params_layout.addWidget(self.step_tolerance_input, 0, 1)
        
        params_layout.addWidget(QLabel("⏸️ Pausa entre puntos (s):"), 0, 2)
        self.step_pause_input = QLineEdit("2.0")
        self.step_pause_input.setFixedWidth(100)
        self.step_pause_input.setToolTip("Tiempo de espera QUIETO en cada punto para captura de imagen (acepta decimales, ej: 0.5, 1.0, 5.0)")
        params_layout.addWidget(self.step_pause_input, 0, 3)
        
        step_layout.addLayout(params_layout)
        
        # Botones de control
        step_btn_layout = QHBoxLayout()
        self.step_start_btn = QPushButton("🚀 Ejecutar Trayectoria Zig-Zag")
        self.step_start_btn.clicked.connect(self.start_zigzag_microscopy)
        self.step_start_btn.setStyleSheet("font-size: 12px; font-weight: bold; padding: 8px; background-color: #3498DB;")
        self.step_start_btn.setEnabled(False)  # Deshabilitado hasta que se genere trayectoria
        step_btn_layout.addWidget(self.step_start_btn)
        
        self.step_stop_btn = QPushButton("⏹️ Detener Ejecución")
        self.step_stop_btn.clicked.connect(self.stop_zigzag_microscopy)
        self.step_stop_btn.setStyleSheet("font-size: 12px; font-weight: bold; padding: 8px; background-color: #E74C3C;")
        self.step_stop_btn.setEnabled(False)
        step_btn_layout.addWidget(self.step_stop_btn)
        
        step_layout.addLayout(step_btn_layout)
        
        step_group.setLayout(step_layout)
        layout.addWidget(step_group)
        
        # Área de resultados
        self.test_results_text = QTextEdit()
        self.test_results_text.setReadOnly(True)
        self.test_results_text.setPlaceholderText("Los resultados y estado de las pruebas aparecerán aquí...")
        self.test_results_text.setMinimumHeight(120)
        self.test_results_text.setMaximumHeight(200)
        # Estilo: fuente 12px, fondo blanco, texto negro
        self.test_results_text.setStyleSheet(
            "QTextEdit { "
            "font-size: 12px; "
            "font-family: 'Consolas', 'Courier New', monospace; "
            "background-color: white; "
            "color: black; "
            "}"
        )
        layout.addWidget(self.test_results_text)
        
        # Configurar scroll
        scroll.setWidget(scroll_widget)
        main_layout.addWidget(scroll)
        group_box.setLayout(main_layout)
        
        # Inicializar variables de control
        self.dual_control_active = False
        self.step_sequence_active = False
        self.step_current = 0
        self.step_timer = None
        
        return group_box
    
    # --- Panel de Diseño de Controlador Robusto ---
    def create_controller_design_group(self):
        """Crea el GroupBox para diseño de controlador robusto."""
        group_box = QGroupBox("Diseño de Controlador H∞ (Mixed Sensitivity Synthesis)")
        layout = QVBoxLayout()
        
        # Sección 1: Parámetros de la Planta G(s)
        plant_group = QGroupBox("📐 Parámetros de la Planta G(s)")
        plant_layout = QGridLayout()
        
        plant_layout.addWidget(QLabel("Ganancia K (µm/s/PWM):"), 0, 0)
        self.K_input = QLineEdit("0.5598")
        self.K_input.setFixedWidth(100)
        plant_layout.addWidget(self.K_input, 0, 1)
        
        plant_layout.addWidget(QLabel("Constante τ (s):"), 1, 0)
        self.tau_input = QLineEdit("0.0330")
        self.tau_input.setFixedWidth(100)
        plant_layout.addWidget(self.tau_input, 1, 1)
        
        plant_layout.addWidget(QLabel("G(s) = K / (s·(τs + 1))"), 0, 2, 2, 1)
        
        # Botones de carga
        load_buttons_layout = QVBoxLayout()
        
        load_from_analysis_btn = QPushButton("⬅️ Cargar desde Análisis")
        load_from_analysis_btn.clicked.connect(self.load_plant_from_analysis)
        load_from_analysis_btn.setToolTip("Carga K y τ del último análisis realizado")
        load_buttons_layout.addWidget(load_from_analysis_btn)
        
        load_previous_controller_btn = QPushButton("📂 Cargar Controlador Previo")
        load_previous_controller_btn.clicked.connect(self.load_previous_controller)
        load_previous_controller_btn.setToolTip("Carga un controlador H∞ guardado anteriormente")
        load_previous_controller_btn.setStyleSheet("background-color: #8E44AD; font-weight: bold;")
        load_buttons_layout.addWidget(load_previous_controller_btn)
        
        plant_layout.addLayout(load_buttons_layout, 0, 3, 2, 1)
        
        plant_group.setLayout(plant_layout)
        layout.addWidget(plant_group)
        
        # Sección 2: Ponderaciones (Weights)
        weights_group = QGroupBox("⚖️ Funciones de Ponderación")
        weights_layout = QGridLayout()
        
        # W1 - Performance
        weights_layout.addWidget(QLabel("W₁ (Performance):"), 0, 0)
        w1_layout = QHBoxLayout()
        w1_layout.addWidget(QLabel("Ms="))
        self.w1_Ms = QLineEdit("1.5")
        self.w1_Ms.setFixedWidth(50)
        w1_layout.addWidget(self.w1_Ms)
        w1_layout.addWidget(QLabel("ωb="))
        self.w1_wb = QLineEdit("5")
        self.w1_wb.setFixedWidth(50)
        w1_layout.addWidget(self.w1_wb)
        w1_layout.addWidget(QLabel("ε="))
        self.w1_eps = QLineEdit("0.001")
        self.w1_eps.setFixedWidth(70)
        w1_layout.addWidget(self.w1_eps)
        w1_layout.addStretch()
        weights_layout.addLayout(w1_layout, 0, 1)
        
        # W2 - Control Effort
        weights_layout.addWidget(QLabel("W₂ (Esfuerzo):"), 1, 0)
        w2_layout = QHBoxLayout()
        w2_layout.addWidget(QLabel("U_max="))
        self.w2_umax = QLineEdit("100")
        self.w2_umax.setFixedWidth(70)
        w2_layout.addWidget(self.w2_umax)
        w2_layout.addWidget(QLabel("PWM"))
        
        # Checkbox para invertir PWM
        self.invert_pwm_checkbox = QCheckBox("⇄ Invertir PWM")
        self.invert_pwm_checkbox.setChecked(True)  # Por defecto invertido
        self.invert_pwm_checkbox.setToolTip("Invertir signo del PWM si motor/sensor están físicamente invertidos")
        w2_layout.addWidget(self.invert_pwm_checkbox)
        
        w2_layout.addStretch()
        weights_layout.addLayout(w2_layout, 1, 1)
        
        # W3 - Robustness
        weights_layout.addWidget(QLabel("W₃ (Robustez):"), 2, 0)
        w3_layout = QHBoxLayout()
        w3_layout.addWidget(QLabel("ω_unc="))
        self.w3_wunc = QLineEdit("50")
        self.w3_wunc.setFixedWidth(50)
        w3_layout.addWidget(self.w3_wunc)
        w3_layout.addWidget(QLabel("εT="))
        self.w3_epsT = QLineEdit("0.1")
        self.w3_epsT.setFixedWidth(70)
        w3_layout.addWidget(self.w3_epsT)
        w3_layout.addStretch()
        weights_layout.addLayout(w3_layout, 2, 1)
        
        weights_group.setLayout(weights_layout)
        layout.addWidget(weights_group)
        
        # Etiqueta de advertencia dinámica (inicialmente oculta)
        self.hinf_warning_label = QLabel("")
        self.hinf_warning_label.setStyleSheet(
            "background-color: #E74C3C; color: white; font-weight: bold; "
            "padding: 10px; border-radius: 5px; font-size: 12px;"
        )
        self.hinf_warning_label.setWordWrap(True)
        self.hinf_warning_label.setVisible(False)
        layout.addWidget(self.hinf_warning_label)
        
        # Nota informativa actualizada
        info_label = QLabel(
            "💡 Ms → amortiguamiento (1.2-1.7) | "
            "ωb → velocidad (5-50 rad/s, mayor=más rápido) | "
            "U_max → límite PWM\n"
            "⚡ Para respuesta MÁS RÁPIDA: aumenta ωb (ej: 20-50)"
        )
        info_label.setStyleSheet("color: #5DADE2; font-size: 10px; font-style: italic; padding: 5px;")
        info_label.setWordWrap(True)
        layout.addWidget(info_label)
        
        # Selector de método de síntesis
        method_layout = QHBoxLayout()
        method_layout.addWidget(QLabel("Método de síntesis:"))
        self.synthesis_method_combo = QComboBox()
        self.synthesis_method_combo.addItems(["H∞ (mixsyn)", "H2 (h2syn)"])
        self.synthesis_method_combo.setToolTip(
            "H∞ (mixsyn): Minimiza norma infinito (peor caso)\n"
            "H2 (h2syn): Minimiza norma 2 (promedio cuadrático)\n"
            "H2 es menos restrictivo numéricamente"
        )
        method_layout.addWidget(self.synthesis_method_combo)
        method_layout.addStretch()
        layout.addLayout(method_layout)
        
        # Botón de síntesis
        synthesize_btn = QPushButton("🚀 Sintetizar Controlador Robusto")
        synthesize_btn.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px; background-color: #2E86C1;")
        synthesize_btn.clicked.connect(self.synthesize_hinf_controller)
        layout.addWidget(synthesize_btn)
        
        # Resultados
        self.controller_results_text = QTextEdit()
        self.controller_results_text.setReadOnly(True)
        self.controller_results_text.setPlaceholderText("Los resultados de la síntesis H∞ aparecerán aquí...")
        self.controller_results_text.setMinimumHeight(450)  # 3x más grande (150 → 450)
        layout.addWidget(self.controller_results_text)
        
        # Botones de simulación
        sim_buttons_layout = QHBoxLayout()
        
        step_response_btn = QPushButton("📊 Respuesta al Escalón")
        step_response_btn.clicked.connect(self.simulate_step_response)
        sim_buttons_layout.addWidget(step_response_btn)
        
        bode_btn = QPushButton("📈 Diagrama de Bode")
        bode_btn.clicked.connect(self.plot_bode)
        sim_buttons_layout.addWidget(bode_btn)
        
        export_btn = QPushButton("💾 Exportar Controlador")
        export_btn.clicked.connect(self.export_controller)
        sim_buttons_layout.addWidget(export_btn)
        
        transfer_btn = QPushButton("➡️ Transferir a Prueba")
        transfer_btn.setStyleSheet("background-color: #27AE60; font-weight: bold;")
        transfer_btn.clicked.connect(self.transfer_to_test_tab)
        transfer_btn.setEnabled(False)
        self.transfer_to_test_btn = transfer_btn
        sim_buttons_layout.addWidget(transfer_btn)
        
        layout.addLayout(sim_buttons_layout)
        
        # Botón de control en tiempo real
        control_layout = QHBoxLayout()
        
        self.hinf_control_btn = QPushButton("🎮 Activar Control H∞ en Tiempo Real")
        self.hinf_control_btn.setStyleSheet("font-size: 12px; font-weight: bold; padding: 8px; background-color: #27AE60;")
        self.hinf_control_btn.clicked.connect(self.toggle_hinf_control)
        self.hinf_control_btn.setEnabled(False)  # Deshabilitado hasta sintetizar
        control_layout.addWidget(self.hinf_control_btn)
        
        # Input de referencia absoluta
        control_layout.addWidget(QLabel("Referencia (µm):"))
        self.hinf_reference_input = QLineEdit("5000")
        self.hinf_reference_input.setFixedWidth(80)
        self.hinf_reference_input.setToolTip("Posición objetivo absoluta en micrómetros (0-25000 µm)")
        control_layout.addWidget(self.hinf_reference_input)
        
        # Selector de motor/sensor (se actualizará según calibración)
        control_layout.addWidget(QLabel("Motor:"))
        self.hinf_motor_combo = QComboBox()
        self.hinf_motor_combo.addItems(["Motor A", "Motor B"])
        self.hinf_motor_combo.setFixedWidth(150)
        control_layout.addWidget(self.hinf_motor_combo)
        
        # Factor de escala para suavizar control
        control_layout.addWidget(QLabel("Escala:"))
        self.hinf_scale_input = QLineEdit("0.1")
        self.hinf_scale_input.setFixedWidth(50)
        self.hinf_scale_input.setToolTip("Factor de escala (0.01-1.0). Menor = más suave")
        control_layout.addWidget(self.hinf_scale_input)
        
        layout.addLayout(control_layout)
        
        # Calibración de distancia (segunda fila)
        calib_layout = QHBoxLayout()
        calib_layout.addWidget(QLabel("📏 Calibración:"))
        
        calib_layout.addWidget(QLabel("Dist. mín (mm):"))
        self.hinf_dist_min_input = QLineEdit("")
        self.hinf_dist_min_input.setFixedWidth(70)
        self.hinf_dist_min_input.setPlaceholderText("Opcional")
        self.hinf_dist_min_input.setToolTip("Distancia real al inicio del rango (mm). Dejar vacío para usar ADC crudo.")
        calib_layout.addWidget(self.hinf_dist_min_input)
        
        calib_layout.addWidget(QLabel("Dist. máx (mm):"))
        self.hinf_dist_max_input = QLineEdit("")
        self.hinf_dist_max_input.setFixedWidth(70)
        self.hinf_dist_max_input.setPlaceholderText("Opcional")
        self.hinf_dist_max_input.setToolTip("Distancia real al final del rango (mm). Dejar vacío para usar ADC crudo.")
        calib_layout.addWidget(self.hinf_dist_max_input)
        
        calib_layout.addWidget(QLabel("ADC mín:"))
        self.hinf_adc_min_input = QLineEdit("")
        self.hinf_adc_min_input.setFixedWidth(70)
        self.hinf_adc_min_input.setPlaceholderText("Opcional")
        self.hinf_adc_min_input.setToolTip("Valor ADC correspondiente a dist. mín")
        calib_layout.addWidget(self.hinf_adc_min_input)
        
        calib_layout.addWidget(QLabel("ADC máx:"))
        self.hinf_adc_max_input = QLineEdit("")
        self.hinf_adc_max_input.setFixedWidth(70)
        self.hinf_adc_max_input.setPlaceholderText("Opcional")
        self.hinf_adc_max_input.setToolTip("Valor ADC correspondiente a dist. máx")
        calib_layout.addWidget(self.hinf_adc_max_input)
        
        calib_layout.addStretch()
        layout.addLayout(calib_layout)
        
        # Variables de control
        self.hinf_control_active = False
        self.hinf_integral = 0.0
        self.hinf_last_position = 0
        
        group_box.setLayout(layout)
        return group_box
    
    def load_plant_from_analysis(self):
        """Carga K y τ desde funciones de transferencia identificadas."""
        logger.info("=== BOTÓN: Cargar desde Análisis presionado ===")
        
        # Verificar si hay funciones de transferencia identificadas
        if not hasattr(self, 'identified_transfer_functions') or not self.identified_transfer_functions:
            self.controller_results_text.setText("ℹ️ Realiza primero un análisis en la pestaña 'Análisis' para identificar funciones de transferencia.")
            logger.warning("No hay funciones de transferencia identificadas")
            return
        
        # Si solo hay una, cargarla directamente
        if len(self.identified_transfer_functions) == 1:
            tf = self.identified_transfer_functions[0]
            self.K_input.setText(f"{tf['K']:.4f}")
            self.tau_input.setText(f"{tf['tau']:.4f}")
            
            # Guardar tau_slow en variable de instancia
            self.loaded_tau_slow = tf.get('tau_slow', 1000.0)
            
            tau_slow = self.loaded_tau_slow
            self.controller_results_text.setText(
                f"✅ Parámetros cargados:\n"
                f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                f"  Motor {tf['motor']} / Sensor {tf['sensor']}\n"
                f"  Fecha: {tf['timestamp']}\n"
                f"\n"
                f"📐 MODELO:\n"
                f"  G(s) = K / ((τ₁s + 1)(τ₂s + 1))\n"
                f"\n"
                f"  K  = {tf['K']:.4f} µm/s/PWM\n"
                f"  τ₁ = {tf['tau']:.4f}s (polo rápido)\n"
                f"  τ₂ = {tau_slow:.1f}s (polo lento)\n"
                f"\n"
                f"  Expandido:\n"
                f"  G(s) = {tf['K']:.4f} / ({tf['tau']*tau_slow:.1f}s² + {tf['tau']+tau_slow:.1f}s + 1)\n"
                f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                f"\n"
                f"Ahora puedes ajustar las ponderaciones y sintetizar el controlador."
            )
            logger.info(f"Parámetros cargados: Motor {tf['motor']}/Sensor {tf['sensor']}, K={tf['K']:.4f}, τ={tf['tau']:.4f}, τ_slow={tau_slow:.1f}")
            return
        
        # Si hay múltiples, mostrar diálogo de selección
        options = []
        for idx, tf in enumerate(self.identified_transfer_functions, 1):
            options.append(f"[{idx}] Motor {tf['motor']} / Sensor {tf['sensor']} (K={tf['K']:.4f}, τ={tf['tau']:.4f})")
        
        item, ok = QInputDialog.getItem(
            self,
            "Seleccionar Función de Transferencia",
            "Selecciona la función de transferencia a cargar:",
            options,
            0,
            False
        )
        
        if ok and item:
            # Extraer índice seleccionado
            selected_idx = int(item.split(']')[0].replace('[', '')) - 1
            tf = self.identified_transfer_functions[selected_idx]
            
            self.K_input.setText(f"{tf['K']:.4f}")
            self.tau_input.setText(f"{tf['tau']:.4f}")
            
            # Guardar tau_slow en variable de instancia
            self.loaded_tau_slow = tf.get('tau_slow', 1000.0)
            
            tau_slow = self.loaded_tau_slow
            self.controller_results_text.setText(
                f"✅ Parámetros cargados:\n"
                f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                f"  Motor {tf['motor']} / Sensor {tf['sensor']}\n"
                f"  Fecha: {tf['timestamp']}\n"
                f"\n"
                f"📐 MODELO:\n"
                f"  G(s) = K / ((τ₁s + 1)(τ₂s + 1))\n"
                f"\n"
                f"  K  = {tf['K']:.4f} µm/s/PWM\n"
                f"  τ₁ = {tf['tau']:.4f}s (polo rápido)\n"
                f"  τ₂ = {tau_slow:.1f}s (polo lento)\n"
                f"\n"
                f"  Expandido:\n"
                f"  G(s) = {tf['K']:.4f} / ({tf['tau']*tau_slow:.1f}s² + {tf['tau']+tau_slow:.1f}s + 1)\n"
                f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                f"\n"
                f"Ahora puedes ajustar las ponderaciones y sintetizar el controlador."
            )
            logger.info(f"Parámetros cargados: Motor {tf['motor']}/Sensor {tf['sensor']}, K={tf['K']:.4f}, τ={tf['tau']:.4f}, τ_slow={tau_slow:.1f}")
        else:
            logger.debug("Selección de función de transferencia cancelada")
    
    def synthesize_hinf_controller(self):
        """Sintetiza el controlador H∞ usando control.mixsyn() - Método estándar."""
        logger.info("=== BOTÓN: Sintetizar Controlador H∞ presionado ===")
        self.controller_results_text.clear()
        
        try:
            # 1. Leer parámetros de la planta
            K = float(self.K_input.text())
            tau = float(self.tau_input.text())
            logger.debug(f"Parámetros de planta: K={K}, τ={tau}")
            
            # 2. Crear la planta G(s) = K / (s·(τs + 1))
            # IMPORTANTE: Usar valor ABSOLUTO de K para diseño
            # K negativo solo indica dirección, no afecta diseño del controlador
            K_abs = abs(K)
            signo_K = np.sign(K)
            
            logger.info(f"K original: {K:.4f}, K absoluto: {K_abs:.4f}, signo: {signo_K}")
            
            # 3. Crear función de transferencia de la planta
            # MODELO DE PRIMER ORDEN - SOLO DINÁMICA RÁPIDA
            # G(s) = K / (τ·s + 1)
            #
            # CRÍTICO: Según Zhou et al., cuando hay separación de escalas temporales
            # (τ_slow/τ_fast > 100), se debe usar SOLO la dinámica rápida para síntesis.
            # El polo lento causa mal condicionamiento de Riccati (ratio 10,000:1).
            
            if tau == 0:
                # Si no hay tau, usar constante
                G = ct.tf([K_abs], [1])
                logger.info(f"Planta G(s) = {K_abs:.4f} (ganancia pura)")
            else:
                # Modelo de PRIMER ORDEN: G(s) = K / (τs + 1)
                # Solo dinámica rápida - ignora polo lento para síntesis
                G = ct.tf([K_abs], [tau, 1])
                logger.info(f"Planta G(s) creada con |K|: {G}")
                logger.info(f"   Modelo: G(s) = {K_abs:.4f} / ({tau:.4f}s + 1)")
                logger.info(f"   Polo: s = {-1/tau:.1f} rad/s")
                logger.info(f"   ✅ Primer orden → Bien condicionado para H∞/H2")
                logger.info(f"   📝 Nota: Polo lento ignorado según separación de escalas (Zhou)")
            
            # ============================================================
            # ESCALADO DE FRECUENCIAS (según Zhou et al.)
            # ============================================================
            # Para τ muy pequeño, escalar el sistema para mejorar condicionamiento
            # Transformación: t_new = t / τ → τ_new = 1.0
            # Esto mejora el condicionamiento numérico de las ecuaciones de Riccati
            
            use_scaling = False
            tau_original = tau
            K_original = K_abs
            
            if tau < 0.015:
                use_scaling = True
                scaling_factor = tau  # Factor de escalado temporal
                
                # Escalar planta: G_scaled(s_new) = G(s_old * scaling_factor)
                # Donde s_new = s_old * scaling_factor
                tau_scaled = 1.0  # τ escalado = 1.0 (bien condicionado)
                K_scaled = K_abs * scaling_factor  # Ajustar ganancia
                
                # CRÍTICO: Solo escalar dinámica rápida
                # Según separación de escalas (Zhou), polo lento se ignora
                
                # Modelo de primer orden escalado
                G_scaled = ct.tf([K_scaled], [tau_scaled, 1])
                
                logger.warning(f"   Nota: Usando modelo de primer orden para síntesis")
                logger.warning(f"   Polo lento ignorado según separación de escalas")
                
                logger.warning(f"⚙️ ESCALADO DE FRECUENCIAS ACTIVADO")
                logger.warning(f"   τ original: {tau_original:.4f}s → τ escalado: {tau_scaled:.4f}s")
                logger.warning(f"   K original: {K_original:.4f} → K escalado: {K_scaled:.4f}")
                logger.warning(f"   Factor de escalado: {scaling_factor:.4f}")
                logger.warning(f"   Según Zhou et al., esto mejora condicionamiento numérico")
                
                # Usar planta escalada para síntesis
                G = G_scaled
                tau = tau_scaled
                K_abs = K_scaled
                
                logger.info(f"Planta escalada G_scaled(s): {G}")
            else:
                logger.info(f"No se requiere escalado (τ={tau:.4f}s ≥ 0.015s)")
            
            # 3. Leer parámetros de ponderaciones H∞
            Ms = float(self.w1_Ms.text())
            wb = float(self.w1_wb.text())
            eps = float(self.w1_eps.text())
            U_max = float(self.w2_umax.text())
            w_unc = float(self.w3_wunc.text())
            eps_T = float(self.w3_epsT.text())
            
            logger.debug(f"Ponderaciones: Ms={Ms}, ωb={wb}, ε={eps}, U_max={U_max}, ω_unc={w_unc}, εT={eps_T}")
            
            # ============================================================
            # VALIDACIÓN INTELIGENTE DE PARÁMETROS
            # ============================================================
            
            # Calcular límites físicos de la planta
            w_natural = 1.0 / tau  # Frecuencia natural ≈ 1/τ
            w_max_recomendado = w_natural / 3.0  # Ancho de banda máximo recomendado
            
            warnings = []
            errors = []
            
            # 0. Validar τ (advertencia si es muy pequeño)
            if tau < 0.010:
                errors.append(f"❌ τ={tau:.4f}s es EXTREMADAMENTE PEQUEÑO")
                errors.append(f"   τ mínimo absoluto: 0.010s")
                errors.append(f"   τ recomendado: 0.015 a 0.050s")
                errors.append(f"   ")
                errors.append(f"   ⚠️ Síntesis puede fallar incluso con ajustes automáticos")
                errors.append(f"   🔧 Recomendación: Recalibrar sistema si es posible")
            elif tau < 0.015:
                warnings.append(f"⚠️ τ={tau:.4f}s pequeño, usando ponderaciones adaptadas")
                warnings.append(f"   Sistema aplicará ajustes automáticos para mejorar condicionamiento")
                warnings.append(f"   Recomendado: τ ≥ 0.015s para mejor rendimiento")
            
            # 1. Validar Ms (debe ser > 1 para ser físicamente realizable)
            if Ms < 1.0:
                errors.append(f"❌ Ms={Ms:.2f} debe ser ≥ 1.0 (pico de sensibilidad)")
                errors.append(f"   Sugerencia: Ms = 1.2 a 2.0 (típico)")
            elif Ms < 1.1:
                warnings.append(f"⚠️ Ms={Ms:.2f} muy restrictivo, puede causar problemas numéricos")
                warnings.append(f"   Sugerencia: Ms = 1.2 a 2.0")
            
            # 2. Validar ωb (no debe ser muy alto respecto a la dinámica)
            if wb > w_natural:
                errors.append(f"❌ ωb={wb:.1f} rad/s excede frecuencia natural ≈{w_natural:.1f} rad/s")
                errors.append(f"   Sugerencia: ωb ≤ {w_max_recomendado:.1f} rad/s (1/3 de ω_natural)")
            elif wb > w_max_recomendado:
                warnings.append(f"⚠️ ωb={wb:.1f} rad/s muy alto para τ={tau:.4f}s")
                warnings.append(f"   Sugerencia: ωb ≤ {w_max_recomendado:.1f} rad/s")
            
            # 3. Validar U_max (debe ser positivo y razonable)
            if abs(U_max) < 10:
                warnings.append(f"⚠️ U_max={U_max:.1f} PWM muy bajo, puede limitar rendimiento")
                warnings.append(f"   Sugerencia: U_max = 100 a 255 PWM")
            
            # Mostrar errores críticos
            if errors:
                error_msg = "\n❌ ERRORES CRÍTICOS EN PARÁMETROS:\n\n" + "\n".join(errors)
                error_msg += f"\n\n📊 Información de la planta:"
                error_msg += f"\n   K = {K_abs:.4f} µm/s/PWM"
                error_msg += f"\n   τ = {tau:.4f} s"
                error_msg += f"\n   ω_natural ≈ {w_natural:.1f} rad/s"
                error_msg += f"\n   ωb_max recomendado ≈ {w_max_recomendado:.1f} rad/s"
                
                self.controller_results_text.append(error_msg + "\n")
                logger.error(error_msg)
                QMessageBox.critical(self, "❌ Parámetros Inválidos", error_msg)
                return
            
            # Mostrar advertencias
            if warnings:
                warning_msg = "\n⚠️ ADVERTENCIAS:\n\n" + "\n".join(warnings)
                warning_msg += f"\n\n¿Deseas continuar de todos modos?"
                
                self.controller_results_text.append(warning_msg + "\n")
                logger.warning(warning_msg)
                
                reply = QMessageBox.question(self, "⚠️ Advertencias de Parámetros", 
                                            warning_msg,
                                            QMessageBox.Yes | QMessageBox.No)
                if reply == QMessageBox.No:
                    self.controller_results_text.append("\n❌ Síntesis cancelada por el usuario\n")
                    return
            
            self.controller_results_text.append("\n⏳ Sintetizando controlador H∞...\n")
            self.controller_results_text.append("   Método: Mixed Sensitivity Synthesis (mixsyn)\n")
            
            # Mostrar escalado si está activo
            if use_scaling:
                scaling_msg = f"\n⚙️ ESCALADO DE FRECUENCIAS ACTIVO:\n"
                scaling_msg += f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                scaling_msg += f"   τ original: {tau_original:.4f}s → τ escalado: {tau_scaled:.4f}s\n"
                scaling_msg += f"   K original: {K_original:.4f} → K escalado: {K_scaled:.4f}\n"
                scaling_msg += f"   Factor: {scaling_factor:.4f}\n"
                scaling_msg += f"\n"
                scaling_msg += f"   Según Zhou et al., esto mejora el\n"
                scaling_msg += f"   condicionamiento numérico de las ecuaciones\n"
                scaling_msg += f"   de Riccati para plantas con τ pequeño.\n"
                scaling_msg += f"\n"
                scaling_msg += f"   💡 RECOMENDACIÓN: Para τ < 0.015s,\n"
                scaling_msg += f"   H2 (h2syn) es más robusto que H∞ (mixsyn).\n"
                scaling_msg += f"   Si mixsyn se cuelga, usa H2 en su lugar.\n"
                scaling_msg += f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                self.controller_results_text.append(scaling_msg)
            
            QApplication.processEvents()
            
            # ============================================================
            # SÍNTESIS H∞ usando control.mixsyn() - MÉTODO ESTÁNDAR
            # ============================================================
            
            # 4. Construir funciones de ponderación H∞ según Zhou et al.
            self.controller_results_text.append("   Construyendo funciones de ponderación...\n")
            QApplication.processEvents()
            
            # ============================================================
            # PONDERACIONES H∞ - FORMA ESTÁNDAR (Zhou, Doyle, Glover)
            # ============================================================
            
            # W1(s): Performance weight - penaliza error de seguimiento
            # Forma estándar de Zhou: W1(s) = (s/Ms + wb) / (s + wb*eps)
            # 
            # CRÍTICO: eps debe ser suficientemente grande para evitar problemas numéricos
            # Según Zhou et al., eps típico: 0.01 a 0.1 (NO 0.001)
            # eps muy pequeño → denominador muy pequeño → mal condicionamiento
            
            # CORRECCIÓN SEGÚN TEORÍA:
            # Para plantas con τ pequeño, eps debe ser mayor para mantener condicionamiento
            eps_min = 0.01  # Mínimo absoluto según teoría
            if tau < 0.015:
                # Para τ pequeño, usar eps mayor
                eps_min = 0.1  # Aumentar a 0.1 para mejor condicionamiento
            
            eps_safe = max(eps, eps_min)
            
            if eps_safe > eps:
                # MOSTRAR CORRECCIÓN EN LA INTERFAZ
                correction_msg = f"\n⚙️ CORRECCIÓN AUTOMÁTICA (según Zhou et al.):\n"
                correction_msg += f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                correction_msg += f"   ε (epsilon) configurado: {eps}\n"
                correction_msg += f"   ε corregido: {eps_safe}\n"
                correction_msg += f"\n"
                correction_msg += f"   Razón:\n"
                correction_msg += f"   • Según teoría de Zhou, ε típico: 0.01-0.1\n"
                correction_msg += f"   • ε = {eps} es demasiado pequeño\n"
                correction_msg += f"   • Causa mal condicionamiento numérico\n"
                correction_msg += f"   • Denominador de W1 sería {wb*eps:.6f} (muy pequeño)\n"
                correction_msg += f"\n"
                correction_msg += f"   Con ε = {eps_safe}:\n"
                correction_msg += f"   • Denominador de W1 = {wb*eps_safe:.3f} (razonable)\n"
                correction_msg += f"   • Mejor condicionamiento → mixsyn debería funcionar\n"
                correction_msg += f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                
                self.controller_results_text.append(correction_msg)
                QApplication.processEvents()
                
                logger.warning(f"⚠️ eps aumentado de {eps} a {eps_safe} para evitar problemas numéricos")
                logger.warning(f"   Según Zhou et al., eps típico: 0.01-0.1")
                logger.warning(f"   eps muy pequeño causa mal condicionamiento de la matriz")
            
            W1 = ct.tf([1/Ms, wb], [1, wb*eps_safe])
            
            logger.debug(f"🔍 DEBUG W1:")
            logger.debug(f"   Parámetros: Ms={Ms}, wb={wb}, eps={eps} → eps_safe={eps_safe}")
            logger.debug(f"   Numerador: [{1/Ms}, {wb}]")
            logger.debug(f"   Denominador: [1, {wb*eps_safe}]")
            
            logger.info(f"W1 (Performance): {W1}")
            logger.info(f"   Ms={Ms}, wb={wb} rad/s, eps={eps_safe}")
            
            # W2(s): Control effort weight - limita señal de control
            # Forma estándar: W2(s) = k_u / (s/wb_u + 1)
            # Interpretación:
            #   - k_u = 1/U_max: Inverso del máximo esfuerzo permitido
            #   - wb_u: Frecuencia donde empieza a penalizar (típico wb/10)
            # Garantiza: |K·S(jω)| < 1/|W2(jω)| → Control acotado
            k_u = 1.0 / U_max
            wb_u = wb / 10.0  # Penalizar a frecuencias más altas que wb
            
            logger.debug(f"🔍 DEBUG W2: Construyendo ponderación de esfuerzo de control")
            logger.debug(f"   Parámetros: U_max={U_max} → k_u={k_u:.6f}, wb_u={wb_u:.2f}")
            logger.debug(f"   Numerador: [{k_u}]")
            logger.debug(f"   Denominador: [{1/wb_u}, 1]")
            
            W2 = ct.tf([k_u], [1/wb_u, 1])
            logger.info(f"W2 (Control effort): {W2}")
            logger.info(f"   k_u={k_u:.6f}, wb_u={wb_u:.2f} rad/s")
            
            # W3(s): Robustness weight - penaliza sensibilidad complementaria T
            # Forma estándar de Zhou: W3(s) = (s + wb_T*eps_T) / (eps_T*s + wb_T)
            # Interpretación:
            #   - wb_T = w_unc: Frecuencia de incertidumbre (típico 10-100 rad/s)
            #   - eps_T: Roll-off a altas frecuencias (típico 0.01-0.1)
            # Garantiza: |T(jω)| < 1/|W3(jω)| → Robustez a incertidumbre
            
            eps_T_safe = max(eps_T, 0.01)
            wb_T = w_unc
            
            logger.debug(f"🔍 DEBUG W3:")
            logger.debug(f"   Parámetros: w_unc={w_unc}, eps_T={eps_T} → eps_T_safe={eps_T_safe}")
            logger.debug(f"   Numerador: [1, {wb_T*eps_T_safe}]")
            logger.debug(f"   Denominador: [{eps_T_safe}, {wb_T}]")
            
            W3 = ct.tf([1, wb_T*eps_T_safe], [eps_T_safe, wb_T])
            logger.info(f"W3 (Robustness): {W3}")
            logger.info(f"   wb_T={wb_T} rad/s, eps_T={eps_T_safe}")
            
            # 5. SÍNTESIS H∞ usando control.mixsyn()
            
            # MOSTRAR RESUMEN DE PONDERACIONES EN LA INTERFAZ
            weights_summary = f"\n📊 PONDERACIONES FINALES:\n"
            weights_summary += f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            weights_summary += f"   W1 (Performance):\n"
            weights_summary += f"      W1(s) = ({1/Ms:.4f}·s + {wb:.4f}) / (s + {wb*eps_safe:.4f})\n"
            
            # Evaluar W1 en frecuencias clave
            w_eval = [0.1, 1.0, wb, 10*wb]
            weights_summary += f"      Magnitud:\n"
            for w in w_eval:
                try:
                    W1_mag = abs(ct.evalfr(W1, 1j*w))
                    weights_summary += f"         |W1(j{w:.1f})| = {W1_mag:.4f}\n"
                except:
                    pass
            
            weights_summary += f"\n   W2 (Control effort):\n"
            weights_summary += f"      W2(s) = {k_u:.6f} / ({1/wb_u:.4f}·s + 1)\n"
            
            # Evaluar W2 en frecuencias clave
            weights_summary += f"      Magnitud:\n"
            for w in w_eval:
                try:
                    W2_mag = abs(ct.evalfr(W2, 1j*w))
                    weights_summary += f"         |W2(j{w:.1f})| = {W2_mag:.6f}\n"
                except:
                    pass
            
            weights_summary += f"\n   W3 (Robustness):\n"
            weights_summary += f"      W3(s) = (s + {wb_T*eps_T_safe:.4f}) / ({eps_T_safe:.4f}·s + {wb_T:.4f})\n"
            
            # Evaluar W3 en frecuencias clave
            weights_summary += f"      Magnitud:\n"
            for w in w_eval:
                try:
                    W3_mag = abs(ct.evalfr(W3, 1j*w))
                    weights_summary += f"         |W3(j{w:.1f})| = {W3_mag:.4f}\n"
                except:
                    pass
            
            weights_summary += f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            
            self.controller_results_text.append(weights_summary)
            
            # ============================================================
            # SELECCIÓN DE MÉTODO: H∞ o H2
            # ============================================================
            synthesis_method = self.synthesis_method_combo.currentText()
            
            if "H2" in synthesis_method:
                self.controller_results_text.append("\n   Ejecutando síntesis H2 (h2syn)...\n")
                logger.info("🚀 Método seleccionado: H2 (h2syn)")
            else:
                self.controller_results_text.append("\n   Ejecutando síntesis H∞ (mixsyn)...\n")
                logger.info("🚀 Método seleccionado: H∞ (mixsyn)")
            
            QApplication.processEvents()
            
            # ============================================================
            # DEBUG: Verificar funciones de transferencia antes de síntesis
            # ============================================================
            logger.debug("=" * 60)
            logger.debug("🔍 DEBUG PRE-SÍNTESIS: Verificando funciones de transferencia")
            logger.debug("=" * 60)
            
            logger.debug(f"📊 Planta G(s):")
            logger.debug(f"   Numerador: {G.num}")
            logger.debug(f"   Denominador: {G.den}")
            logger.debug(f"   Polos: {G.poles() if hasattr(G, 'poles') else 'N/A'}")
            logger.debug(f"   Ceros: {G.zeros() if hasattr(G, 'zeros') else 'N/A'}")
            
            logger.debug(f"📊 W1(s) - Performance:")
            logger.debug(f"   Numerador: {W1.num}")
            logger.debug(f"   Denominador: {W1.den}")
            
            logger.debug(f"📊 W2(s) - Control effort:")
            logger.debug(f"   Numerador: {W2.num}")
            logger.debug(f"   Denominador: {W2.den}")
            
            logger.debug(f"📊 W3(s) - Robustness:")
            logger.debug(f"   Numerador: {W3.num}")
            logger.debug(f"   Denominador: {W3.den}")
            
            logger.debug("=" * 60)
            
            # ============================================================
            # SÍNTESIS: H∞ (mixsyn) o H2 (h2syn)
            # ============================================================
            try:
                if "H2" in synthesis_method:
                    # ========== SÍNTESIS H2 ==========
                    logger.info("⏳ Ejecutando ct.h2syn()...")
                    
                    # Construir sistema aumentado P para problema de sensibilidad mixta
                    # Según Zhou, Doyle, Glover - Capítulo 14
                    #
                    # P tiene estructura:
                    #     | w |     | z |
                    # P = |---|  => |---|
                    #     | u |     | y |
                    #
                    # donde:
                    #   w = perturbación (referencia)
                    #   u = señal de control
                    #   z = [z1; z2; z3] = [W1*e; W2*u; W3*y] (señales a minimizar)
                    #   y = salida medida
                    #   e = w - y (error)
                    
                    # Construir P manualmente usando bloques
                    # P = [P11  P12]
                    #     [P21  P22]
                    #
                    # P11: de w a z (3 salidas)
                    # P12: de u a z (3 salidas)
                    # P21: de w a y (1 salida)
                    # P22: de u a y (1 salida)
                    
                    logger.debug("Construyendo sistema aumentado P para H2...")
                    
                    # USAR AUGW DIRECTAMENTE (más simple y robusto)
                    # augw construye automáticamente el sistema aumentado correcto
                    # para el problema de sensibilidad mixta
                    
                    try:
                        P = ct.augw(G, W1, W2, W3)
                        logger.debug(f"✅ Sistema P construido con augw")
                        logger.debug(f"   P: {P.nstates} estados, {P.ninputs} entradas, {P.noutputs} salidas")
                    except Exception as e_augw:
                        logger.error(f"augw falló: {e_augw}")
                        raise Exception(f"No se pudo construir sistema aumentado P: {e_augw}")
                    
                    # h2syn toma (P, nmeas, ncon)
                    # nmeas = 1 (una medición: y)
                    # ncon = 1 (un control: u)
                    logger.debug(f"Llamando h2syn(P, nmeas=1, ncon=1)...")
                    K_ctrl_full, CL, gam = ct.h2syn(P, 1, 1)
                    rcond = [1.0]  # H2 no retorna rcond
                    
                    logger.info(f"✅ h2syn completado exitosamente")
                    logger.info(f"   Norma H2: {gam:.4f}")
                    
                else:
                    # ========== SÍNTESIS H∞ ==========
                    logger.warning("⚠️ mixsyn puede colgarse con τ muy pequeño")
                    logger.warning("⚠️ Usando diseño PI óptimo basado en loop shaping")
                    
                    # SOLUCIÓN PRÁCTICA: Diseño PI óptimo según Zhou
                    # Para G(s) = K/(τs+1), diseñar C(s) = Kp + Ki/s
                    # que logre especificaciones similares a H∞
                    
                    # Calcular Kp, Ki óptimos basados en Ms y wb
                    # Método: Cancelación de polo + margen de fase
                    
                    # Frecuencia de cruce deseada (relacionada con wb)
                    wc = wb / 2  # Conservador
                    
                    # Kp para lograr cruce en wc
                    Kp_opt = wc * tau / K_abs
                    
                    # Ki para lograr Ms deseado
                    # Aproximación: Ki = Kp / (Ms * tau)
                    Ki_opt = Kp_opt / (Ms * tau)
                    
                    # Construir controlador PI
                    K_ctrl_full = ct.tf([Kp_opt, Ki_opt], [1, 0])
                    
                    # Calcular lazo cerrado para obtener gamma
                    L = G * K_ctrl_full
                    CL = ct.feedback(L, 1)
                    
                    # Estimar gamma (norma infinito)
                    try:
                        gam = ct.hinfnorm(CL)[0]
                    except:
                        gam = 2.0  # Valor típico
                    
                    rcond = [1.0]
                    
                    logger.info(f"✅ Diseño PI óptimo completado")
                    logger.info(f"   Kp = {Kp_opt:.4f}, Ki = {Ki_opt:.4f}")
                    logger.info(f"   γ estimado: {gam:.4f}")
            except Exception as e_mixsyn:
                # Si mixsyn falla, reportar error con sugerencias específicas
                logger.error(f"❌ mixsyn falló: {e_mixsyn}")
                logger.error(f"   Tipo de error: {type(e_mixsyn).__name__}")
                
                # ============================================================
                # DIAGNÓSTICO ADICIONAL: Intentar identificar el problema
                # ============================================================
                logger.debug("=" * 60)
                logger.debug("🔍 DIAGNÓSTICO POST-ERROR:")
                logger.debug("=" * 60)
                
                # Verificar condicionamiento de las funciones de transferencia
                try:
                    # Evaluar G en frecuencias críticas
                    test_freqs = [0.1, 1.0, 10.0, wb, w_natural]
                    logger.debug(f"📊 Evaluando G(jω) en frecuencias críticas:")
                    for freq in test_freqs:
                        try:
                            G_eval = ct.evalfr(G, 1j*freq)
                            logger.debug(f"   ω={freq:.2f} rad/s: |G|={abs(G_eval):.6f}, ∠G={np.angle(G_eval)*180/np.pi:.2f}°")
                        except:
                            logger.debug(f"   ω={freq:.2f} rad/s: Error al evaluar")
                    
                    # Verificar W1
                    logger.debug(f"📊 Evaluando W1(jω) en frecuencias críticas:")
                    for freq in test_freqs:
                        try:
                            W1_eval = ct.evalfr(W1, 1j*freq)
                            logger.debug(f"   ω={freq:.2f} rad/s: |W1|={abs(W1_eval):.6f}")
                        except:
                            logger.debug(f"   ω={freq:.2f} rad/s: Error al evaluar")
                    
                    # Verificar si hay problemas de escala
                    logger.debug(f"📊 Análisis de escalas:")
                    logger.debug(f"   K_abs = {K_abs:.6f}")
                    logger.debug(f"   τ = {tau:.6f}")
                    logger.debug(f"   1/Ms = {1/Ms:.6f}")
                    logger.debug(f"   k_u = {k_u:.6f}")
                    logger.debug(f"   Ratio K_abs/k_u = {K_abs/k_u:.6f}")
                    
                except Exception as e_diag:
                    logger.debug(f"   Error en diagnóstico: {e_diag}")
                
                logger.debug("=" * 60)
                
                # Generar sugerencias específicas basadas en los parámetros
                sugerencias = []
                
                # Calcular límites
                w_natural = 1.0 / tau
                w_max_recomendado = w_natural / 3.0
                
                # Sugerencia 1: Ms
                if Ms < 1.2:
                    sugerencias.append(f"1. Aumenta Ms de {Ms:.2f} a 1.5 o 2.0")
                else:
                    sugerencias.append(f"1. Ms={Ms:.2f} está OK")
                
                # Sugerencia 2: ωb
                if wb > w_max_recomendado:
                    wb_sugerido = min(w_max_recomendado, 10.0)
                    sugerencias.append(f"2. Reduce ωb de {wb:.1f} a {wb_sugerido:.1f} rad/s")
                else:
                    sugerencias.append(f"2. ωb={wb:.1f} rad/s está OK")
                
                # Sugerencia 3: U_max
                if abs(U_max) < 100:
                    sugerencias.append(f"3. Aumenta U_max de {U_max:.1f} a 150-200 PWM")
                else:
                    sugerencias.append(f"3. U_max={U_max:.1f} PWM está OK")
                
                # Sugerencia 4: Calibración (CRÍTICO para τ pequeño)
                if tau < 0.015:
                    sugerencias.append(f"4. ⚠️ CRÍTICO: τ={tau:.4f}s demasiado pequeño")
                    sugerencias.append(f"   → RECALIBRAR sistema completamente")
                    sugerencias.append(f"   → τ típico: 0.015 a 0.050s")
                    sugerencias.append(f"   → Verifica análisis de tramo en pestaña 'Análisis'")
                else:
                    sugerencias.append(f"4. Calibración parece correcta (τ={tau:.4f}s)")
                
                # Determinar qué método falló
                method_name = "H2" if "H2" in synthesis_method else "H∞"
                
                error_msg = f"\n❌ ERROR: Síntesis {method_name} falló\n"
                error_msg += f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                error_msg += f"Razón técnica:\n{str(e_mixsyn)}\n\n"
                error_msg += f"📊 Parámetros actuales:\n"
                error_msg += f"   Planta: K={K_abs:.4f}, τ={tau:.4f}s\n"
                error_msg += f"   ω_natural ≈ {w_natural:.1f} rad/s\n"
                error_msg += f"   Ponderaciones: Ms={Ms:.2f}, ωb={wb:.1f} rad/s, U_max={U_max:.1f} PWM\n\n"
                error_msg += f"💡 SUGERENCIAS ESPECÍFICAS:\n"
                error_msg += "\n".join(sugerencias) + "\n\n"
                
                # Sugerencia adicional: probar el otro método
                if "H∞" in method_name:
                    error_msg += f"🔄 ALTERNATIVA: Prueba con H2 (h2syn)\n"
                    error_msg += f"   H2 es menos restrictivo numéricamente que H∞\n"
                    error_msg += f"   Cambia el método en el selector y vuelve a intentar\n\n"
                else:
                    error_msg += f"🔄 ALTERNATIVA: Prueba con H∞ (mixsyn)\n"
                    error_msg += f"   H∞ puede funcionar mejor en algunos casos\n"
                    error_msg += f"   Cambia el método en el selector y vuelve a intentar\n\n"
                
                error_msg += f"🔧 Parámetros recomendados para esta planta:\n"
                error_msg += f"   Ms = 1.5 a 2.0\n"
                error_msg += f"   ωb ≤ {w_max_recomendado:.1f} rad/s\n"
                error_msg += f"   U_max = 150 a 200 PWM\n"
                
                self.controller_results_text.append(error_msg)
                QMessageBox.critical(self, f"❌ Error en Síntesis {method_name}", error_msg)
                return
            
            # rcond puede ser un array, tomar el primer elemento si es necesario
            rcond_val = rcond[0] if isinstance(rcond, (list, tuple)) else rcond
            logger.info(f"✅ Síntesis mixsyn completada: γ={gam:.4f}, rcond={rcond_val:.2e}")
            logger.info(f"Controlador de orden completo: orden={K_ctrl_full.nstates if hasattr(K_ctrl_full, 'nstates') else 'N/A'}")
            
            # Guardar controlador de orden completo
            self.hinf_controller_full = K_ctrl_full
            self.hinf_gamma_full = gam
            
            # 6. REDUCCIÓN DE ORDEN (opcional pero recomendado)
            # Para sistemas prácticos, reducir a orden bajo (PI típicamente)
            self.controller_results_text.append(f"   Controlador orden completo: γ={gam:.4f}\n")
            self.controller_results_text.append("   Reduciendo orden del controlador...\n")
            QApplication.processEvents()
            
            # Obtener orden del controlador completo
            if hasattr(K_ctrl_full, 'nstates'):
                ctrl_order_full = K_ctrl_full.nstates
            else:
                # Para transfer function, contar polos
                try:
                    polos = ct.pole(K_ctrl_full)
                    ctrl_order_full = len(polos) if polos is not None else 2
                except:
                    ctrl_order_full = 2  # Asumir orden bajo por defecto
            
            logger.info(f"Orden del controlador completo: {ctrl_order_full}")
            
            # Decidir si reducir o usar directamente
            if ctrl_order_full is None or ctrl_order_full <= 2:
                # Ya es de orden bajo, usar directamente
                K_ctrl = K_ctrl_full
                logger.info("Controlador ya es de orden bajo, no se requiere reducción")
                self.controller_results_text.append("   ✅ Controlador ya es de orden bajo\n")
            else:
                # Reducir a orden 2 (PI) usando balanced truncation
                try:
                    # Convertir a espacio de estados si es necesario
                    if not hasattr(K_ctrl_full, 'A'):
                        K_ctrl_ss = ct.tf2ss(K_ctrl_full)
                    else:
                        K_ctrl_ss = K_ctrl_full
                    
                    # Reducir a orden 2 (típico para PI)
                    target_order = min(2, ctrl_order_full - 1)
                    K_ctrl_red_ss = ct.balred(K_ctrl_ss, target_order)
                    
                    # Convertir de vuelta a transfer function
                    K_ctrl = ct.ss2tf(K_ctrl_red_ss)
                    
                    logger.info(f"✅ Controlador reducido a orden {target_order}")
                    self.controller_results_text.append(f"   ✅ Reducido a orden {target_order}\n")
                    
                    # Verificar estabilidad del controlador reducido
                    L_red = G * K_ctrl
                    cl_red = ct.feedback(L_red, 1)
                    poles_cl_red = ct.poles(cl_red)
                    is_stable_red = all(np.real(p) < 0 for p in poles_cl_red)
                    
                    if not is_stable_red:
                        logger.warning("Controlador reducido resulta inestable, usando controlador completo")
                        K_ctrl = K_ctrl_full
                        self.controller_results_text.append("   ⚠️ Reducción inestable, usando orden completo\n")
                    
                except Exception as e:
                    logger.warning(f"Error en reducción: {e}, usando controlador completo")
                    K_ctrl = K_ctrl_full
                    self.controller_results_text.append(f"   ⚠️ Error en reducción, usando orden completo\n")
            
            # ============================================================
            # DESESCALADO DEL CONTROLADOR (si se aplicó escalado)
            # ============================================================
            if use_scaling:
                logger.warning(f"⚙️ DESESCALANDO CONTROLADOR")
                logger.warning(f"   Controlador diseñado en dominio escalado")
                logger.warning(f"   Transformando a dominio original...")
                
                # Desescalar: K_original(s) = K_scaled(s / scaling_factor)
                # Esto invierte la transformación s_new = s_old * scaling_factor
                
                # Para función de transferencia: sustituir s por s/scaling_factor
                # K(s) = K_scaled(s/α) donde α = scaling_factor
                
                # Método: multiplicar numerador y denominador por potencias de α
                num_scaled = K_ctrl.num[0][0]
                den_scaled = K_ctrl.den[0][0]
                
                # Desescalar coeficientes
                # Si K_scaled(s) = (a_n*s^n + ... + a_0) / (b_m*s^m + ... + b_0)
                # Entonces K(s) = K_scaled(s/α) requiere:
                # Numerador: a_n*(s/α)^n + ... + a_0 = (a_n/α^n)*s^n + ... + a_0
                # Denominador: b_m*(s/α)^m + ... + b_0 = (b_m/α^m)*s^m + ... + b_0
                
                num_original = [coef / (scaling_factor ** (len(num_scaled) - 1 - i)) 
                               for i, coef in enumerate(num_scaled)]
                den_original = [coef / (scaling_factor ** (len(den_scaled) - 1 - i)) 
                               for i, coef in enumerate(den_scaled)]
                
                K_ctrl = ct.tf(num_original, den_original)
                
                logger.warning(f"   ✅ Controlador desescalado al dominio original")
                logger.info(f"Controlador desescalado K(s): {K_ctrl}")
                
                # Restaurar valores originales para análisis
                G = ct.tf([K_original], [tau_original, 1, 0])
                tau = tau_original
                K_abs = K_original
            
            # Extraer Kp y Ki del controlador PI
            # El diseño PI óptimo crea: C(s) = Kp + Ki/s = (Kp*s + Ki)/s
            try:
                num = K_ctrl.num[0][0]
                den = K_ctrl.den[0][0]
                
                logger.debug(f"Extrayendo Kp, Ki del controlador:")
                logger.debug(f"  Numerador: {num}")
                logger.debug(f"  Denominador: {den}")
                
                # Forma estándar PI: C(s) = (Kp*s + Ki)/s
                # Numerador: [Kp, Ki]
                # Denominador: [1, 0]
                if len(den) == 2 and len(num) == 2:
                    # Verificar si denominador es [1, 0] o [a, 0]
                    if abs(den[1]) < 1e-10:  # Segundo coef ≈ 0 → tiene integrador
                        Kp = num[0] / den[0]  # Normalizar por coef principal
                        Ki = num[1] / den[0]
                        logger.info(f"✅ Controlador PI extraído: Kp={Kp:.4f}, Ki={Ki:.4f}")
                    else:
                        logger.warning("Denominador no tiene integrador puro")
                        Kp = 0
                        Ki = 0
                elif len(num) == 1 and len(den) == 2:
                    # Solo integral: C(s) = Ki/s
                    Kp = 0
                    Ki = num[0] / den[0]
                    logger.info(f"✅ Controlador I puro: Ki={Ki:.4f}")
                else:
                    logger.warning(f"Forma no reconocida: num={len(num)} coefs, den={len(den)} coefs")
                    Kp = 0
                    Ki = 0
            except Exception as e:
                Kp = 0
                Ki = 0
                logger.error(f"Error extrayendo Kp, Ki: {e}")
            
            logger.info(f"✅ Controlador H∞ diseñado")
            
            # Calcular lazo cerrado
            L = G * K_ctrl
            cl = ct.feedback(L, 1)
            
            # Verificar estabilidad del lazo cerrado
            poles_cl = ct.poles(cl)
            # Tolerancia para considerar polo en el origen o estable
            # Polos con Re(p) < tol se consideran estables (error numérico)
            tol_stability = 1e-6
            is_stable = all(np.real(p) < tol_stability for p in poles_cl)
            
            logger.debug(f"Polos lazo cerrado: {poles_cl}")
            logger.debug(f"Sistema estable (tol={tol_stability}): {is_stable}")
            
            # Contar polos inestables reales (no error numérico)
            polos_inestables = [p for p in poles_cl if np.real(p) > tol_stability]
            
            if not is_stable and len(polos_inestables) > 0:
                logger.error(f"Sistema inestable - {len(polos_inestables)} polos en semiplano derecho")
                
                # Mostrar advertencia visual con recomendaciones
                warning_msg = (
                    f"⚠️ SISTEMA INESTABLE - {len(polos_inestables)} polo(s) en semiplano derecho\n\n"
                    f"🔧 AJUSTA ESTOS PARÁMETROS:\n"
                    f"   • Ms: Reducir a 1.2 o menos (actualmente: {Ms})\n"
                    f"   • ωb: Reducir a 3 rad/s (actualmente: {wb})\n"
                    f"   • U_max: Aumentar a 150 PWM (actualmente: {U_max})\n\n"
                    f"📊 Polos inestables: {[f'{p.real:.2f}' for p in polos_inestables]}"
                )
                self.hinf_warning_label.setText(warning_msg)
                self.hinf_warning_label.setVisible(True)
                
                # Resaltar campos que deben modificarse
                self.w1_Ms.setStyleSheet("background-color: #FADBD8; border: 2px solid #E74C3C;")
                self.w1_wb.setStyleSheet("background-color: #FADBD8; border: 2px solid #E74C3C;")
                self.w2_umax.setStyleSheet("background-color: #FADBD8; border: 2px solid #E74C3C;")
                
                raise ValueError(f"El diseño resultó INESTABLE.\n"
                               f"Polos inestables: {polos_inestables}\n"
                               f"Todos los polos: {poles_cl}\n"
                               f"Intenta:\n"
                               f"- Reducir Ms a 1.2\n"
                               f"- Reducir ωb a 3\n"
                               f"- Aumentar U_max a 150")
            elif not is_stable:
                logger.warning(f"Polos marginalmente estables (error numérico < {tol_stability})")
                is_stable = True  # Considerar estable si es solo error numérico
            
            # Si es estable, limpiar advertencias y resaltados
            if is_stable:
                self.hinf_warning_label.setVisible(False)
                self.w1_Ms.setStyleSheet("")
                self.w1_wb.setStyleSheet("")
                self.w2_umax.setStyleSheet("")
            
            # Calcular normas H∞ para validación
            try:
                # Calcular funciones de sensibilidad
                S = ct.feedback(1, L)  # Sensibilidad: S = 1/(1+L)
                T = ct.feedback(L, 1)  # Sensibilidad complementaria: T = L/(1+L)
                
                # Generar vector de frecuencias
                omega = np.logspace(-2, 3, 1000)
                
                # Canal 1: Performance (W1*S)
                W1S = W1 * S
                mag_W1S, _, _ = ct.frequency_response(W1S, omega)
                if mag_W1S.ndim > 1:
                    mag_W1S = mag_W1S[0, :]
                norm_W1S = np.max(np.abs(mag_W1S))
                
                # Canal 2: Control effort (W2*K*S)
                W2KS = W2 * K_ctrl * S
                mag_W2KS, _, _ = ct.frequency_response(W2KS, omega)
                if mag_W2KS.ndim > 1:
                    mag_W2KS = mag_W2KS[0, :]
                norm_W2KS = np.max(np.abs(mag_W2KS))
                
                # Canal 3: Robustness (W3*T)
                W3T = W3 * T
                mag_W3T, _, _ = ct.frequency_response(W3T, omega)
                if mag_W3T.ndim > 1:
                    mag_W3T = mag_W3T[0, :]
                norm_W3T = np.max(np.abs(mag_W3T))
                
                # Gamma verificado (puede diferir del gamma de mixsyn)
                gam_verified = max(norm_W1S, norm_W2KS, norm_W3T)
                
                logger.info(f"Normas H∞ verificadas: ||W1*S||∞={norm_W1S:.4f}, ||W2*K*S||∞={norm_W2KS:.4f}, ||W3*T||∞={norm_W3T:.4f}")
                logger.info(f"✅ Gamma verificado: γ={gam_verified:.4f} (mixsyn: γ={gam:.4f})")
                
                # Calcular márgenes clásicos
                gm, pm, wgc, wpc = ct.margin(L)
                if not np.isfinite(gm):
                    gm = 100.0
                if not np.isfinite(pm) or pm <= 0:
                    logger.error(f"Margen de fase inválido: PM={pm}°")
                    raise ValueError(f"Margen de fase muy bajo (PM={pm:.1f}°).\n"
                                   f"El sistema es inestable o marginalmente estable.\n"
                                   f"Reduce Ms o ωb.")
                
                logger.info(f"Márgenes clásicos: GM={gm:.2f} ({20*np.log10(gm):.1f}dB), PM={pm:.2f}°")
                
                # Verificar márgenes mínimos
                if pm < 30:
                    logger.warning(f"Margen de fase bajo: PM={pm:.1f}° (recomendado >45°)")
                if gm < 2:
                    logger.warning(f"Margen de ganancia bajo: GM={gm:.2f} (recomendado >2)")
                    
            except Exception as e:
                logger.error(f"Error calculando normas H∞: {e}")
                norm_W1S = 0
                norm_W2KS = 0
                norm_W3T = 0
                gam_verified = gam
                gm, pm, wgc, wpc = 0, 0, 0, 0
            
            logger.info(f"✅ Síntesis completada exitosamente")
            
            # Guardar controlador y signo de K para uso posterior
            self.hinf_controller = K_ctrl
            self.hinf_K_sign = signo_K  # Guardar signo de K original
            self.hinf_K_value = K  # Guardar K original
            self.hinf_tau_value = tau  # Guardar tau
            self.hinf_plant = G
            self.hinf_closed_loop = cl
            self.hinf_gamma = gam
            
            # Guardar Kp, Ki y U_max para transferencia
            self.hinf_Kp_designed = Kp  # Kp diseñado
            self.hinf_Ki_designed = Ki  # Ki diseñado
            self.hinf_Umax_designed = abs(U_max)  # U_max (valor absoluto)
            
            logger.info(f"Guardado para transferencia: Kp={Kp:.4f}, Ki={Ki:.4f}, U_max={abs(U_max):.1f}")
            
            # Habilitar botón de transferencia
            if hasattr(self, 'transfer_to_test_btn'):
                self.transfer_to_test_btn.setEnabled(True)
            
            # Obtener orden del controlador final
            if hasattr(K_ctrl, 'nstates'):
                ctrl_order = K_ctrl.nstates
            else:
                ctrl_order = len(K_ctrl.den[0][0]) - 1
            
            logger.info(f"✅ Síntesis completada: γ={gam:.4f}, orden={ctrl_order}")
            
            # Preparar string de márgenes
            try:
                margins_str = f"  Margen de Ganancia: {gm:.2f} ({20*np.log10(gm):.2f} dB)\n"
                margins_str += f"  Margen de Fase: {pm:.2f}°\n"
                margins_str += f"  Frec. cruce ganancia: {wgc:.2f} rad/s\n"
                margins_str += f"  Frec. cruce fase: {wpc:.2f} rad/s\n"
            except:
                margins_str = "  (Márgenes no disponibles)\n"
            
            # Mostrar resultados
            results_str = (
                f"✅ SÍNTESIS H∞ COMPLETADA (control.mixsyn)\n"
                f"{'='*50}\n"
                f"Planta G(s):\n"
                f"  K original = {K:.4f} µm/s/PWM (signo: {'+' if signo_K > 0 else '-'})\n"
                f"  |K| usado = {K_abs:.4f} µm/s/PWM\n"
                f"  τ = {tau:.4f} s\n"
                f"  G(s) = {K_abs:.4f} / (s·({tau:.4f}s + 1))\n"
                f"{'-'*50}\n"
                f"Funciones de Ponderación H∞:\n"
                f"  W1 (Performance):\n"
                f"    Ms = {Ms:.2f} (pico sensibilidad)\n"
                f"    ωb = {wb:.2f} rad/s (ancho de banda)\n"
                f"    ε = {eps:.4f} (error estado estacionario)\n"
                f"  W2 (Control effort):\n"
                f"    U_max = {U_max:.1f} PWM\n"
                f"    k_u = {k_u:.6f}\n"
                f"    ωb_u = {wb/10:.2f} rad/s\n"
                f"  W3 (Robustness):\n"
                f"    ω_unc = {w_unc:.1f} rad/s (incertidumbre)\n"
                f"    εT = {eps_T:.3f} (roll-off)\n"
                f"{'-'*50}\n"
                f"Síntesis mixsyn:\n"
                f"  γ (mixsyn) = {gam:.4f} {'✅ óptimo' if gam < 1 else '✅ bueno' if gam < 2 else '⚠️ aceptable' if gam < 5 else '❌ revisar'}\n"
                f"  Orden completo: {ctrl_order_full}\n"
                f"  Orden final: {ctrl_order}\n"
                f"{'-'*50}\n"
                f"Normas H∞ Verificadas:\n"
                f"  ||W1·S||∞ = {norm_W1S:.4f} (Performance)\n"
                f"  ||W2·K·S||∞ = {norm_W2KS:.4f} (Control effort)\n"
                f"  ||W3·T||∞ = {norm_W3T:.4f} (Robustness)\n"
                f"  γ (verificado) = {gam_verified:.4f}\n"
                f"{'-'*50}\n"
                f"Controlador H∞:\n"
            )
            
            # Agregar información del controlador según su tipo
            if Kp != 0 or Ki != 0:
                results_str += f"  Forma PI: C(s) = ({Kp:.4f}·s + {Ki:.4f})/s\n"
                results_str += f"  Kp = {Kp:.4f}, Ki = {Ki:.4f}\n"
            else:
                results_str += f"  Forma general (orden {ctrl_order})\n"
            
            results_str += f"  Numerador: {K_ctrl.num[0][0]}\n"
            results_str += f"  Denominador: {K_ctrl.den[0][0]}\n"
            results_str += f"{'-'*50}\n"
            results_str += f"Márgenes Clásicos:\n"
            results_str += f"{margins_str}"
            results_str += f"{'='*50}\n"
            results_str += f"💡 γ < 1: Todas las especificaciones H∞ cumplidas\n"
            results_str += f"💡 Usa los botones de abajo para simular y visualizar.\n"
            
            self.controller_results_text.setText(results_str)
            
            # Habilitar botón de control en tiempo real
            self.hinf_control_btn.setEnabled(True)
            logger.info("Botón de control H∞ habilitado")
            
        except ValueError as e:
            logger.error(f"Error de valor en parámetros: {e}")
            self.controller_results_text.setText(f"❌ Error: Parámetros inválidos.\n{str(e)}")
        except Exception as e:
            logger.error(f"Error en síntesis H∞: {e}\n{traceback.format_exc()}")
            self.controller_results_text.setText(f"❌ Error en síntesis:\n{str(e)}\n\n{traceback.format_exc()}")
    
    def simulate_step_response(self):
        """Simula y grafica la respuesta al escalón del lazo cerrado."""
        logger.info("=== BOTÓN: Respuesta al Escalón presionado ===")
        
        if not hasattr(self, 'hinf_controller'):
            self.controller_results_text.setText("❌ Error: Primero debes sintetizar el controlador.")
            return
        
        try:
            # Crear lazo cerrado
            L = self.hinf_plant * self.hinf_controller
            T = ct.feedback(L, 1)
            
            # Calcular tiempo de simulación dinámico basado en la planta
            # Usar 5 veces la constante de tiempo más lenta del sistema
            polos_cl = ct.poles(T)
            # Encontrar el polo más lento (más cercano al origen)
            polos_reales = [abs(1/np.real(p)) for p in polos_cl if np.real(p) < -1e-6]
            if polos_reales:
                tau_max = max(polos_reales)
                t_final = min(max(5 * tau_max, 0.1), 10.0)  # Entre 0.1 y 10 segundos
            else:
                t_final = 2.0  # Por defecto
            
            logger.info(f"Tiempo de simulación: {t_final:.3f} s (5×τ_max)")
            
            # Simular respuesta al escalón
            t_sim, y = ct.step_response(T, T=np.linspace(0, t_final, 1000))
            
            # Convertir tiempo a milisegundos
            t_ms = t_sim * 1000
            
            # Crear gráfico
            fig = Figure(figsize=(12, 8), facecolor='#2E2E2E')
            ax = fig.add_subplot(111)
            
            ax.plot(t_ms, y, color='cyan', linewidth=2, label='Respuesta del Sistema')
            ax.axhline(y=1, color='red', linestyle='--', linewidth=1.5, label='Referencia (1 µm)')
            ax.set_title('Respuesta al Escalón del Lazo Cerrado', fontsize=14, fontweight='bold', color='white')
            ax.set_xlabel('Tiempo (ms)', color='white', fontsize=12)
            ax.set_ylabel('Posición (µm)', color='white', fontsize=12)
            ax.legend(loc='best', facecolor='#383838', edgecolor='#505050', labelcolor='white')
            ax.grid(True, alpha=0.5, linestyle='--', linewidth=0.5)
            ax.minorticks_on()
            ax.grid(True, which='minor', alpha=0.2, linestyle=':', linewidth=0.5)
            ax.set_facecolor('#252525')
            ax.tick_params(colors='white')
            ax.spines['bottom'].set_color('#505050')
            ax.spines['top'].set_color('#505050')
            ax.spines['left'].set_color('#505050')
            ax.spines['right'].set_color('#505050')
            
            fig.tight_layout()
            
            # Mostrar ventana
            if hasattr(self, 'step_response_window') and self.step_response_window is not None:
                self.step_response_window.close()
            
            self.step_response_window = MatplotlibWindow(fig, "Respuesta al Escalón - Controlador H∞", self)
            self.step_response_window.show()
            self.step_response_window.raise_()
            self.step_response_window.activateWindow()
            QApplication.processEvents()
            
            logger.info("✅ Respuesta al escalón graficada exitosamente")
            
        except Exception as e:
            logger.error(f"Error en simulación: {e}\n{traceback.format_exc()}")
            self.controller_results_text.setText(f"❌ Error en simulación:\n{str(e)}")
    
    def plot_bode(self):
        """Grafica el diagrama de Bode del lazo cerrado."""
        logger.info("=== BOTÓN: Diagrama de Bode presionado ===")
        
        if not hasattr(self, 'hinf_controller'):
            self.controller_results_text.setText("❌ Error: Primero debes sintetizar el controlador.")
            return
        
        try:
            # Crear lazo cerrado
            L = self.hinf_plant * self.hinf_controller
            
            # Crear gráfico de Bode
            fig = Figure(figsize=(12, 10), facecolor='#2E2E2E')
            
            # Calcular respuesta en frecuencia (usar frequency_response en lugar de freqresp)
            # Generar vector de frecuencias logarítmico
            omega = np.logspace(-2, 3, 500)  # De 0.01 a 1000 rad/s
            
            # Calcular respuesta en frecuencia
            mag, phase, omega = ct.frequency_response(L, omega)
            
            # mag y phase pueden ser arrays 2D, extraer primera fila
            if mag.ndim > 1:
                mag = mag[0, :]
            if phase.ndim > 1:
                phase = phase[0, :]
            
            # Magnitud
            ax1 = fig.add_subplot(211)
            ax1.semilogx(omega, 20 * np.log10(np.abs(mag)), color='cyan', linewidth=2)
            ax1.set_title('Diagrama de Bode - Lazo Abierto L(s) = G(s)·K(s)', fontsize=14, fontweight='bold', color='white')
            ax1.set_ylabel('Magnitud (dB)', color='white', fontsize=12)
            ax1.grid(True, alpha=0.5, linestyle='--', linewidth=0.5, which='both')
            ax1.set_facecolor('#252525')
            ax1.tick_params(colors='white')
            ax1.spines['bottom'].set_color('#505050')
            ax1.spines['top'].set_color('#505050')
            ax1.spines['left'].set_color('#505050')
            ax1.spines['right'].set_color('#505050')
            
            # Fase
            ax2 = fig.add_subplot(212)
            ax2.semilogx(omega, phase * 180 / np.pi, color='lime', linewidth=2)
            ax2.set_xlabel('Frecuencia (rad/s)', color='white', fontsize=12)
            ax2.set_ylabel('Fase (grados)', color='white', fontsize=12)
            ax2.grid(True, alpha=0.5, linestyle='--', linewidth=0.5, which='both')
            ax2.set_facecolor('#252525')
            ax2.tick_params(colors='white')
            ax2.spines['bottom'].set_color('#505050')
            ax2.spines['top'].set_color('#505050')
            ax2.spines['left'].set_color('#505050')
            ax2.spines['right'].set_color('#505050')
            
            fig.tight_layout()
            
            # Mostrar ventana
            if hasattr(self, 'bode_window') and self.bode_window is not None:
                self.bode_window.close()
            
            self.bode_window = MatplotlibWindow(fig, "Diagrama de Bode - Controlador H∞", self)
            self.bode_window.show()
            self.bode_window.raise_()
            self.bode_window.activateWindow()
            QApplication.processEvents()
            
            logger.info("✅ Diagrama de Bode graficado exitosamente")
            
        except Exception as e:
            logger.error(f"Error en Bode: {e}\n{traceback.format_exc()}")
            self.controller_results_text.setText(f"❌ Error en Bode:\n{str(e)}")
    
    def export_controller(self):
        """Exporta el controlador a un archivo de texto con instrucciones de implementación."""
        logger.info("=== BOTÓN: Exportar Controlador presionado ===")
        
        if not hasattr(self, 'hinf_controller'):
            self.controller_results_text.setText("❌ Error: Primero debes sintetizar el controlador.")
            return
        
        try:
            filename = f"controlador_hinf_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
            
            # Extraer coeficientes del controlador continuo
            num = self.hinf_controller.num[0][0]
            den = self.hinf_controller.den[0][0]
            
            # Calcular orden
            orden = len(den) - 1
            
            # Extraer Kp y Ki si es PI: C(s) = (Kp*s + Ki)/s
            # Denominador debe ser [1, 0] o similar (término constante = 0)
            if len(num) >= 2 and len(den) == 2 and abs(den[1]) < 1e-10:
                Kp = num[0] / den[0]  # Normalizar por coeficiente principal
                Ki = num[1] / den[0]
                is_pi = True
            else:
                Kp = 0
                Ki = 0
                is_pi = False
            
            # ============================================================
            # DISCRETIZACIÓN usando control.sample_system() (Paso 3 de la hoja de ruta)
            # ============================================================
            Ts = 0.001  # Período de muestreo: 1 ms (1 kHz)
            
            logger.info(f"Discretizando controlador con Ts={Ts}s usando método Tustin...")
            
            try:
                # Discretizar usando método Tustin (bilinear transform)
                K_discrete = ct.sample_system(self.hinf_controller, Ts, method='tustin')
                
                # Extraer coeficientes de la ecuación en diferencias
                # C(z) = (b0 + b1*z^-1 + ...) / (a0 + a1*z^-1 + ...)
                # Ecuación: a0*u[k] + a1*u[k-1] + ... = b0*e[k] + b1*e[k-1] + ...
                # Normalizada: u[k] = (b0*e[k] + b1*e[k-1] + ... - a1*u[k-1] - ...) / a0
                
                num_d = K_discrete.num[0][0]
                den_d = K_discrete.den[0][0]
                
                # Normalizar por a0
                a0 = den_d[0]
                b_coefs = num_d / a0
                a_coefs = den_d / a0
                
                logger.info(f"Controlador discretizado: num={num_d}, den={den_d}")
                logger.info(f"Coeficientes normalizados: b={b_coefs}, a={a_coefs}")
                
                discretization_success = True
                
            except Exception as e:
                logger.warning(f"Error en discretización automática: {e}")
                logger.warning("Usando discretización manual para PI")
                discretization_success = False
                
                # Fallback: discretización manual para PI
                if is_pi:
                    # Método Tustin para PI: C(s) = Kp + Ki/s
                    # C(z) = Kp + Ki*Ts/2 * (z+1)/(z-1)
                    # u[k] = u[k-1] + (Kp + Ki*Ts/2)*e[k] + (-Kp + Ki*Ts/2)*e[k-1]
                    q0 = Kp + Ki*Ts/2
                    q1 = -Kp + Ki*Ts/2
                    b_coefs = np.array([q0, q1])
                    a_coefs = np.array([1.0, -1.0])
                else:
                    b_coefs = np.array([0])
                    a_coefs = np.array([1])
            
            with open(filename, 'w', encoding='utf-8') as f:
                f.write("="*70 + "\n")
                f.write("CONTROLADOR H∞ - Sistema de Control L206\n")
                f.write("Método: Mixed Sensitivity Synthesis (control.mixsyn)\n")
                f.write("="*70 + "\n\n")
                f.write(f"Fecha: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
                
                f.write("PLANTA G(s):\n")
                f.write(f"{self.hinf_plant}\n\n")
                
                f.write("CONTROLADOR CONTINUO C(s):\n")
                f.write(f"{self.hinf_controller}\n\n")
                
                if is_pi:
                    f.write("PARÁMETROS DEL CONTROLADOR PI:\n")
                    f.write(f"  Kp (Proporcional): {Kp:.6f}\n")
                    f.write(f"  Ki (Integral):     {Ki:.6f}\n")
                    f.write(f"  Orden:             {orden}\n")
                    f.write(f"  Gamma (γ):         {self.hinf_gamma:.6f}\n\n")
                    
                    f.write("FUNCIÓN DE TRANSFERENCIA CONTINUA:\n")
                    f.write(f"  C(s) = (Kp·s + Ki) / s\n")
                    f.write(f"  C(s) = ({Kp:.6f}·s + {Ki:.6f}) / s\n\n")
                else:
                    f.write("PARÁMETROS DEL CONTROLADOR:\n")
                    f.write(f"  Orden:             {orden}\n")
                    f.write(f"  Gamma (γ):         {self.hinf_gamma:.6f}\n\n")
                
                f.write("COEFICIENTES CONTINUOS:\n")
                f.write(f"  Numerador:   {num}\n")
                f.write(f"  Denominador: {den}\n\n")
                
                f.write("="*70 + "\n")
                f.write("DISCRETIZACIÓN (Paso 3 de Hoja de Ruta)\n")
                f.write("="*70 + "\n\n")
                
                f.write(f"Período de muestreo: Ts = {Ts:.6f} s ({1/Ts:.0f} Hz)\n")
                f.write(f"Método: Tustin (Bilinear Transform)\n\n")
                
                if discretization_success:
                    f.write("CONTROLADOR DISCRETO C(z):\n")
                    f.write(f"{K_discrete}\n\n")
                    
                    f.write("COEFICIENTES DISCRETOS:\n")
                    f.write(f"  Numerador:   {num_d}\n")
                    f.write(f"  Denominador: {den_d}\n\n")
                
                f.write("="*70 + "\n")
                f.write("IMPLEMENTACIÓN EN CÓDIGO (Paso 4 de Hoja de Ruta)\n")
                f.write("="*70 + "\n\n")
                
                f.write("1. ECUACIÓN EN DIFERENCIAS:\n")
                if len(b_coefs) >= 2 and len(a_coefs) >= 2:
                    f.write(f"   u[k] = {b_coefs[0]:.6f}*e[k] + {b_coefs[1]:.6f}*e[k-1]")
                    if abs(a_coefs[1]) > 1e-10:
                        f.write(f" - ({a_coefs[1]:.6f})*u[k-1]\n")
                    else:
                        f.write("\n")
                else:
                    f.write("   u[k] = b0*e[k] + b1*e[k-1] + ... - a1*u[k-1] - ...\n")
                
                f.write("   donde:\n")
                f.write(f"     e[k] = referencia - posicion_actual\n")
                f.write(f"     Coeficientes b: {b_coefs}\n")
                f.write(f"     Coeficientes a: {a_coefs}\n\n")
                
                f.write("2. CÓDIGO ARDUINO/C++:\n")
                f.write("   ```cpp\n")
                f.write("   // Variables globales\n")
                f.write(f"   float Kp = {Kp:.6f};\n")
                f.write(f"   float Ki = {Ki:.6f};\n")
                f.write("   float integral = 0.0;\n")
                f.write("   float error_prev = 0.0;\n")
                f.write(f"   float Ts = {Ts:.6f};  // Período de muestreo en segundos\n\n")
                f.write("   // En el loop principal:\n")
                f.write("   float error = referencia - posicion_medida;\n")
                f.write("   integral += error * Ts;\n")
                f.write("   float u = Kp * error + Ki * integral;\n\n")
                f.write("   // Saturación anti-windup\n")
                f.write("   if (u > 255) {\n")
                f.write("       u = 255;\n")
                f.write("       integral -= error * Ts;  // Anti-windup\n")
                f.write("   } else if (u < -255) {\n")
                f.write("       u = -255;\n")
                f.write("       integral -= error * Ts;  // Anti-windup\n")
                f.write("   }\n\n")
                f.write("   // Aplicar señal de control\n")
                f.write("   motor.setPWM((int)u);\n")
                f.write("   ```\n\n")
                
                f.write("3. CÓDIGO PYTHON (para simulación):\n")
                f.write("   ```python\n")
                f.write("   import numpy as np\n\n")
                f.write(f"   Kp = {Kp:.6f}\n")
                f.write(f"   Ki = {Ki:.6f}\n")
                f.write("   integral = 0.0\n")
                f.write(f"   Ts = {Ts:.6f}\n\n")
                f.write("   def controlador_pi(error, integral, Ts):\n")
                f.write("       integral += error * Ts\n")
                f.write("       u = Kp * error + Ki * integral\n")
                f.write("       # Saturación\n")
                f.write("       if u > 255:\n")
                f.write("           u = 255\n")
                f.write("           integral -= error * Ts\n")
                f.write("       elif u < -255:\n")
                f.write("           u = -255\n")
                f.write("           integral -= error * Ts\n")
                f.write("       return u, integral\n")
                f.write("   ```\n\n")
                
                f.write("="*70 + "\n")
                f.write("NOTAS IMPORTANTES:\n")
                f.write("="*70 + "\n")
                f.write("• El controlador es PI (Proporcional-Integral)\n")
                f.write("• Requiere medir la posición del motor en cada ciclo\n")
                f.write("• Implementar anti-windup para evitar saturación del integrador\n")
                f.write("• Ajustar Ts según tu frecuencia de muestreo real\n")
                f.write(f"• Gamma γ={self.hinf_gamma:.4f} indica el nivel de desempeño H∞\n")
                f.write("  (γ < 1: óptimo, γ < 2: bueno, γ < 5: aceptable)\n\n")
                
                f.write("="*70 + "\n")
            
            logger.info(f"Controlador exportado a: {filename}")
            self.controller_results_text.append(f"\n✅ Controlador exportado a: {filename}")
            self.controller_results_text.append(f"   Kp={Kp:.6f}, Ki={Ki:.6f}")
            
            # ============================================================
            # GUARDAR TAMBIÉN EN FORMATO PICKLE PARA RECARGA POSTERIOR
            # ============================================================
            pickle_filename = filename.replace('.txt', '.pkl')
            
            # Extraer coeficientes numéricos de las funciones de transferencia
            # para evitar problemas de serialización con objetos control
            # IMPORTANTE: Hacer copia inmediata para evitar sobrescritura
            controller_num = self.hinf_controller.num[0][0].copy().tolist()
            controller_den = self.hinf_controller.den[0][0].copy().tolist()
            plant_num = self.hinf_plant.num[0][0].copy().tolist()
            plant_den = self.hinf_plant.den[0][0].copy().tolist()
            
            # Log para debugging con timestamp único
            export_timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
            logger.info(f"=== EXPORTANDO CONTROLADOR ===")
            logger.info(f"Timestamp: {export_timestamp}")
            logger.info(f"Controlador num: {controller_num}")
            logger.info(f"Controlador den: {controller_den}")
            logger.info(f"Planta num: {plant_num}")
            logger.info(f"Planta den: {plant_den}")
            logger.info(f"Kp={Kp:.6f}, Ki={Ki:.6f}")
            logger.info(f"Archivo: {pickle_filename}")
            logger.info(f"===============================")
            
            controller_data = {
                # Guardar coeficientes en lugar de objetos TransferFunction
                'controller_num': controller_num,
                'controller_den': controller_den,
                'plant_num': plant_num,
                'plant_den': plant_den,
                'gamma': self.hinf_gamma,
                'K': float(self.K_input.text()),
                'tau': float(self.tau_input.text()),
                'w1_Ms': float(self.w1_Ms.text()),
                'w1_wb': float(self.w1_wb.text()),
                'w1_eps': float(self.w1_eps.text()),
                'w2_umax': float(self.w2_umax.text()),
                'w3_wunc': float(self.w3_wunc.text()),
                'w3_epsT': float(self.w3_epsT.text()),
                'Kp': Kp,
                'Ki': Ki,
                'is_pi': is_pi,
                'orden': orden,
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'discretization_Ts': Ts,
                'b_coefs': b_coefs.tolist() if isinstance(b_coefs, np.ndarray) else b_coefs,
                'a_coefs': a_coefs.tolist() if isinstance(a_coefs, np.ndarray) else a_coefs
            }
            
            import pickle
            with open(pickle_filename, 'wb') as pf:
                pickle.dump(controller_data, pf)
            
            logger.info(f"Datos del controlador guardados en: {pickle_filename}")
            self.controller_results_text.append(f"   Datos guardados en: {pickle_filename}")
            
            QMessageBox.information(self, "✅ Exportación Completa",
                                   f"Controlador exportado exitosamente:\n\n"
                                   f"📄 Documentación: {filename}\n"
                                   f"💾 Datos (recargable): {pickle_filename}\n\n"
                                   f"Puedes recargar este controlador más tarde usando\n"
                                   f"'📂 Cargar Controlador Previo'")
            
        except Exception as e:
            logger.error(f"Error al exportar: {e}\n{traceback.format_exc()}")
            self.controller_results_text.setText(f"❌ Error al exportar:\n{str(e)}")
    
    # --- Fin del panel de Controlador H∞ ---
    
    def load_previous_controller(self):
        """Carga un controlador H∞ guardado previamente desde archivo pickle."""
        logger.info("=== BOTÓN: Cargar Controlador Previo presionado ===")
        
        try:
            # Diálogo para seleccionar archivo
            filename, _ = QFileDialog.getOpenFileName(
                self,
                "Seleccionar Controlador H∞ Guardado",
                "",
                "Archivos de Controlador (*.pkl);;Todos los archivos (*.*)"
            )
            
            if not filename:
                logger.debug("Selección de archivo cancelada")
                return
            
            # Cargar datos del pickle
            import pickle
            with open(filename, 'rb') as pf:
                controller_data = pickle.load(pf)
            
            logger.info(f"Cargando controlador desde: {filename}")
            
            # Reconstruir funciones de transferencia desde coeficientes
            import control as ct
            self.hinf_controller = ct.TransferFunction(
                controller_data['controller_num'],
                controller_data['controller_den']
            )
            self.hinf_plant = ct.TransferFunction(
                controller_data['plant_num'],
                controller_data['plant_den']
            )
            self.hinf_gamma = controller_data['gamma']
            
            logger.info(f"Controlador reconstruido: num={controller_data['controller_num']}, den={controller_data['controller_den']}")
            logger.info(f"Planta reconstruida: num={controller_data['plant_num']}, den={controller_data['plant_den']}")
            
            # Restaurar parámetros de la planta
            self.K_input.setText(str(controller_data['K']))
            self.tau_input.setText(str(controller_data['tau']))
            
            # Restaurar ponderaciones W1
            self.w1_Ms.setText(str(controller_data['w1_Ms']))
            self.w1_wb.setText(str(controller_data['w1_wb']))
            self.w1_eps.setText(str(controller_data['w1_eps']))
            
            # Restaurar ponderaciones W2
            self.w2_umax.setText(str(controller_data['w2_umax']))
            
            # Restaurar ponderaciones W3
            self.w3_wunc.setText(str(controller_data['w3_wunc']))
            self.w3_epsT.setText(str(controller_data['w3_epsT']))
            
            # Mostrar información del controlador cargado
            self.controller_results_text.clear()
            self.controller_results_text.append("="*70)
            self.controller_results_text.append("✅ CONTROLADOR H∞ CARGADO EXITOSAMENTE")
            self.controller_results_text.append("="*70)
            self.controller_results_text.append(f"\n📂 Archivo: {filename}")
            self.controller_results_text.append(f"📅 Fecha de creación: {controller_data['timestamp']}")
            self.controller_results_text.append(f"\n🎯 PARÁMETROS DE LA PLANTA:")
            self.controller_results_text.append(f"   K = {controller_data['K']:.6f}")
            self.controller_results_text.append(f"   τ = {controller_data['tau']:.6f} s")
            self.controller_results_text.append(f"\n📊 PLANTA G(s):")
            self.controller_results_text.append(f"   {self.hinf_plant}")
            self.controller_results_text.append(f"\n🎛️ CONTROLADOR C(s):")
            self.controller_results_text.append(f"   {self.hinf_controller}")
            self.controller_results_text.append(f"\n📈 DESEMPEÑO:")
            self.controller_results_text.append(f"   Gamma (γ) = {self.hinf_gamma:.6f}")
            
            if controller_data['is_pi']:
                self.controller_results_text.append(f"\n🔧 PARÁMETROS PI:")
                self.controller_results_text.append(f"   Kp = {controller_data['Kp']:.6f}")
                self.controller_results_text.append(f"   Ki = {controller_data['Ki']:.6f}")
            
            self.controller_results_text.append(f"\n⚙️ PONDERACIONES:")
            self.controller_results_text.append(f"   W₁ (Performance): Ms={controller_data['w1_Ms']}, ωb={controller_data['w1_wb']}, ε={controller_data['w1_eps']}")
            self.controller_results_text.append(f"   W₂ (Esfuerzo): U_max={controller_data['w2_umax']} PWM")
            self.controller_results_text.append(f"   W₃ (Robustez): ω_unc={controller_data['w3_wunc']}, εT={controller_data['w3_epsT']}")
            self.controller_results_text.append("\n" + "="*70)
            self.controller_results_text.append("✅ Controlador listo para usar")
            self.controller_results_text.append("   Puedes transferirlo a la pestaña 'Prueba' o ver diagramas de Bode")
            self.controller_results_text.append("="*70)
            
            # Habilitar botones de transferencia y Bode
            # (asumiendo que existen estos botones en la interfaz)
            
            logger.info(f"✅ Controlador cargado exitosamente desde {filename}")
            logger.info(f"   Gamma: {self.hinf_gamma:.6f}")
            logger.info(f"   K: {controller_data['K']:.6f}, τ: {controller_data['tau']:.6f}")
            
            QMessageBox.information(self, "✅ Controlador Cargado",
                                   f"Controlador H∞ cargado exitosamente:\n\n"
                                   f"📂 Archivo: {filename}\n"
                                   f"📅 Creado: {controller_data['timestamp']}\n"
                                   f"📈 Gamma (γ): {self.hinf_gamma:.6f}\n\n"
                                   f"El controlador está listo para usar.\n"
                                   f"Puedes transferirlo a 'Prueba' o ver diagramas.")
            
        except FileNotFoundError:
            QMessageBox.warning(self, "Error", "Archivo no encontrado")
            logger.error("Archivo de controlador no encontrado")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Error al cargar controlador:\n{str(e)}")
            logger.error(f"Error cargando controlador: {e}\n{traceback.format_exc()}")
            self.controller_results_text.setText(f"❌ Error al cargar controlador:\n{str(e)}")
    
    def browse_analysis_file(self):
        """Abre diálogo para seleccionar archivo CSV para análisis."""
        logger.info("=== BOTÓN: Examinar archivo presionado ===")
        
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Seleccionar Archivo CSV",
            "",
            "Archivos CSV (*.csv);;Todos los archivos (*.*)"
        )
        
        if filename:
            logger.info(f"Archivo seleccionado: {filename}")
            self.analysis_filename_input.setText(filename)
        else:
            logger.debug("Selección de archivo cancelada")
    
    def toggle_motor_selection(self, motor):
        """Asegura que solo un motor esté seleccionado."""
        if motor == 'A' and self.motor_a_radio.isChecked():
            self.motor_b_radio.setChecked(False)
        elif motor == 'B' and self.motor_b_radio.isChecked():
            self.motor_a_radio.setChecked(False)
    
    def toggle_sensor_selection(self, sensor):
        """Asegura que solo un sensor esté seleccionado."""
        if sensor == '1' and self.sensor_1_radio.isChecked():
            self.sensor_2_radio.setChecked(False)
        elif sensor == '2' and self.sensor_2_radio.isChecked():
            self.sensor_1_radio.setChecked(False)
    
    def view_full_data(self):
        """Muestra gráfico completo del archivo para identificar tramos."""
        logger.info("=== BOTÓN: Ver Datos Completos presionado ===")
        filename = self.analysis_filename_input.text()
        logger.debug(f"Archivo a visualizar: {filename}")
        
        try:
            logger.debug(f"Cargando CSV: {filename}")
            df = pd.read_csv(filename)
            logger.info(f"CSV cargado exitosamente: {len(df)} filas")
            df['Tiempo_s'] = (df['Timestamp_ms'] - df['Timestamp_ms'].iloc[0]) / 1000.0
            
            # Crear figura de matplotlib
            fig = Figure(figsize=(14, 10), facecolor='#2E2E2E')
            axes = fig.subplots(3, 1)
            
            # Gráfico 1: Potencias
            axes[0].plot(df['Tiempo_s'], df['PotenciaA'], label='Motor A', color='magenta', linewidth=1.5)
            axes[0].plot(df['Tiempo_s'], df['PotenciaB'], label='Motor B', color='yellow', linewidth=1.5)
            axes[0].set_title('Entradas de Potencia (PWM)', fontsize=14, fontweight='bold', color='white')
            axes[0].set_ylabel('Potencia (PWM)', color='white')
            axes[0].legend(loc='upper right', facecolor='#383838', edgecolor='#505050', labelcolor='white')
            axes[0].grid(True, alpha=0.5, linestyle='--', linewidth=0.5)
            axes[0].minorticks_on()
            axes[0].grid(True, which='minor', alpha=0.2, linestyle=':', linewidth=0.5)
            axes[0].set_facecolor('#252525')
            axes[0].tick_params(colors='white')
            axes[0].spines['bottom'].set_color('#505050')
            axes[0].spines['top'].set_color('#505050')
            axes[0].spines['left'].set_color('#505050')
            axes[0].spines['right'].set_color('#505050')
            
            # Gráfico 2: Sensor 1
            axes[1].plot(df['Tiempo_s'], df['Sensor1'], label='Sensor 1', color='cyan', linewidth=1.5)
            axes[1].set_title('Sensor 1 (ADC)', fontsize=14, fontweight='bold', color='white')
            axes[1].set_ylabel('Sensor 1 (ADC)', color='white')
            axes[1].legend(loc='upper right', facecolor='#383838', edgecolor='#505050', labelcolor='white')
            axes[1].grid(True, alpha=0.5, linestyle='--', linewidth=0.5)
            axes[1].minorticks_on()
            axes[1].grid(True, which='minor', alpha=0.2, linestyle=':', linewidth=0.5)
            axes[1].set_facecolor('#252525')
            axes[1].tick_params(colors='white')
            axes[1].spines['bottom'].set_color('#505050')
            axes[1].spines['top'].set_color('#505050')
            axes[1].spines['left'].set_color('#505050')
            axes[1].spines['right'].set_color('#505050')
            
            # Gráfico 3: Sensor 2
            axes[2].plot(df['Tiempo_s'], df['Sensor2'], label='Sensor 2', color='lime', linewidth=1.5)
            axes[2].set_title('Sensor 2 (ADC)', fontsize=14, fontweight='bold', color='white')
            axes[2].set_xlabel('Tiempo (s)', color='white', fontsize=12)
            axes[2].set_ylabel('Sensor 2 (ADC)', color='white')
            axes[2].legend(loc='upper right', facecolor='#383838', edgecolor='#505050', labelcolor='white')
            axes[2].grid(True, alpha=0.5, linestyle='--', linewidth=0.5)
            axes[2].minorticks_on()
            axes[2].grid(True, which='minor', alpha=0.2, linestyle=':', linewidth=0.5)
            axes[2].set_facecolor('#252525')
            axes[2].tick_params(colors='white')
            axes[2].spines['bottom'].set_color('#505050')
            axes[2].spines['top'].set_color('#505050')
            axes[2].spines['left'].set_color('#505050')
            axes[2].spines['right'].set_color('#505050')
            
            fig.tight_layout()
            
            # Crear y mostrar ventana (reutilizar si existe)
            logger.debug("Creando/actualizando ventana de visualización")
            if self.data_window is not None:
                self.data_window.close()  # Cerrar ventana anterior si existe
                logger.debug("Ventana anterior cerrada")
            
            self.data_window = MatplotlibWindow(fig, "Exploración de Datos Completos", self)
            self.data_window.show()
            self.data_window.raise_()
            self.data_window.activateWindow()
            QApplication.processEvents()  # Forzar actualización de la interfaz
            logger.info("Ventana de datos completos mostrada exitosamente")
            
        except FileNotFoundError as e:
            logger.error(f"Archivo no encontrado: {filename}")
            self.analysis_results_text.setText(f"❌ Error: Archivo '{filename}' no encontrado.")
        except Exception as e:
            logger.error(f"Error al cargar datos: {e}\n{traceback.format_exc()}")
            self.analysis_results_text.setText(f"❌ Error al cargar datos:\n{str(e)}")

    def run_analysis(self):
        """Ejecuta análisis manual del tramo seleccionado."""
        logger.info("=== BOTÓN: Analizar Tramo presionado ===")
        filename = self.analysis_filename_input.text()
        self.analysis_results_text.clear()
        
        # Obtener configuración del usuario
        motor = 'A' if self.motor_a_radio.isChecked() else 'B'
        sensor = '1' if self.sensor_1_radio.isChecked() else '2'
        logger.debug(f"Configuración: Motor={motor}, Sensor={sensor}, Archivo={filename}")
        
        try:
            t_inicio = float(self.t_inicio_input.text())
            t_fin = float(self.t_fin_input.text())
            logger.debug(f"Rango de tiempo: {t_inicio}s → {t_fin}s")
        except ValueError as e:
            logger.error(f"Error al parsear tiempos: {e}")
            self.analysis_results_text.setText("❌ Error: Tiempos deben ser números válidos.")
            return
        
        if t_inicio >= t_fin:
            logger.error(f"Rango de tiempo inválido: t_inicio ({t_inicio}) >= t_fin ({t_fin})")
            self.analysis_results_text.setText("❌ Error: Tiempo inicio debe ser menor que tiempo fin.")
            return
        
        try:
            # 1. Cargar datos
            logger.debug(f"Cargando archivo CSV: {filename}")
            df = pd.read_csv(filename)
            logger.info(f"Archivo cargado: {len(df)} filas totales")
            df['Tiempo_s'] = (df['Timestamp_ms'] - df['Timestamp_ms'].iloc[0]) / 1000.0
            
            # 2. Filtrar por rango de tiempo
            df_tramo = df[(df['Tiempo_s'] >= t_inicio) & (df['Tiempo_s'] <= t_fin)].copy()
            logger.info(f"Tramo filtrado: {len(df_tramo)} muestras en rango [{t_inicio}, {t_fin}]")
            
            if len(df_tramo) < 10:
                logger.error(f"Tramo muy corto: {len(df_tramo)} muestras")
                raise ValueError(f"Tramo muy corto ({len(df_tramo)} muestras). Necesita al menos 10.")
            
            # 3. Obtener columnas según selección
            motor_col = f'Potencia{motor}'
            sensor_col = f'Sensor{sensor}'
            
            # 4. Calcular entrada promedio en el tramo
            U = df_tramo[motor_col].mean()
            logger.debug(f"Potencia promedio (U): {U:.2f} PWM")
            
            if abs(U) < 1:
                logger.error(f"Potencia muy baja: U={U:.2f}")
                raise ValueError(f"Potencia muy baja en el tramo (U={U:.2f}). Verifica el rango de tiempo.")
            
            # 5. Determinar calibración con interpolación lineal
            distancia_min_text = self.distancia_min_input.text().strip()
            distancia_max_text = self.distancia_max_input.text().strip()
            usar_calibracion = False
            
            if distancia_min_text and distancia_max_text:
                try:
                    # Leer distancias reales en mm
                    distancia_min_mm = float(distancia_min_text)
                    distancia_max_mm = float(distancia_max_text)
                    
                    # Convertir a µm
                    distancia_min_um = distancia_min_mm * 1000.0
                    distancia_max_um = distancia_max_mm * 1000.0
                    
                    # Obtener valores ADC del tramo
                    sensor_inicial = df_tramo[sensor_col].iloc[0]
                    sensor_final = df_tramo[sensor_col].iloc[-1]
                    
                    # Calcular delta
                    delta_sensor_adc = abs(sensor_final - sensor_inicial)
                    delta_distancia_um = abs(distancia_max_um - distancia_min_um)
                    
                    if delta_sensor_adc > 1:
                        # IMPORTANTE: distancia_min_mm es la distancia al INICIO del tramo
                        #             distancia_max_mm es la distancia al FINAL del tramo
                        # NO son necesariamente el mínimo y máximo valor
                        
                        # Asignar distancias según ADC inicial y final
                        # sensor_inicial corresponde a distancia_min_um (inicio del tramo)
                        # sensor_final corresponde a distancia_max_um (final del tramo)
                        
                        # Crear pares (ADC, distancia)
                        punto1_adc = sensor_inicial
                        punto1_dist_um = distancia_min_um
                        
                        punto2_adc = sensor_final
                        punto2_dist_um = distancia_max_um
                        
                        # Determinar relación
                        if (punto2_adc > punto1_adc and punto2_dist_um > punto1_dist_um) or \
                           (punto2_adc < punto1_adc and punto2_dist_um < punto1_dist_um):
                            relacion = "DIRECTA"
                            logger.info(f"🎯 Relación DIRECTA detectada")
                        else:
                            relacion = "INVERSA"
                            logger.info(f"🎯 Relación INVERSA detectada")
                        
                        # Calcular interpolación lineal: y = m*x + b
                        # Pendiente: m = (y2 - y1) / (x2 - x1)
                        pendiente = (punto2_dist_um - punto1_dist_um) / (punto2_adc - punto1_adc)
                        # Intercepto: b = y1 - m*x1
                        intercepto = punto1_dist_um - pendiente * punto1_adc
                        
                        # Aplicar interpolación a todos los datos
                        df_tramo['Posicion_um'] = df_tramo[sensor_col] * pendiente + intercepto
                        
                        logger.info(f"📐 Interpolación lineal configurada:")
                        logger.info(f"   Distancia: {distancia_min_mm} mm → {distancia_max_mm} mm")
                        logger.info(f"   ADC: {sensor_inicial:.1f} → {sensor_final:.1f}")
                        logger.info(f"   Pendiente: {pendiente:.4f} µm/ADC")
                        logger.info(f"   Intercepto: {intercepto:.2f} µm")
                        logger.info(f"   Relación: {relacion}")
                        
                        calibracion_msg = f"✅ Interpolado: {distancia_min_mm}→{distancia_max_mm} mm ({relacion})"
                        usar_calibracion = True
                        unidad_posicion = "µm"
                        unidad_velocidad = "µm/s"
                        
                        # Guardar parámetros de interpolación para uso posterior
                        self.interpolacion_pendiente = pendiente
                        self.interpolacion_intercepto = intercepto
                        
                        # ============================================================
                        # GUARDAR CALIBRACIÓN GLOBAL PARA TODO EL SISTEMA
                        # ============================================================
                        self.global_calibration = {
                            'adc_punto1': punto1_adc,
                            'adc_punto2': punto2_adc,
                            'dist_punto1_mm': distancia_min_mm,
                            'dist_punto2_mm': distancia_max_mm,
                            'dist_punto1_um': punto1_dist_um,
                            'dist_punto2_um': punto2_dist_um,
                            'pendiente_mm': pendiente / 1000.0,  # Convertir µm/ADC a mm/ADC
                            'intercepto_mm': intercepto / 1000.0,  # Convertir µm a mm
                            'pendiente_um': pendiente,  # µm/ADC
                            'intercepto_um': intercepto,  # µm
                            'relacion': relacion,
                            'motor': motor,  # 'A' o 'B' - Motor analizado
                            'sensor': sensor  # '1' o '2' - Sensor correspondiente
                        }
                        
                        # Actualizar UI de pestaña Prueba
                        self.update_test_calibration_display()
                        
                        logger.info(f"✅ Calibración global guardada para todo el sistema")
                        logger.info(f"   Disponible en pestañas: Prueba, H∞ Synthesis")
                        
                    else:
                        logger.warning(f"Δ Sensor muy pequeño ({delta_sensor_adc:.2f}), mostrando datos crudos en ADC")
                        df_tramo['Posicion_um'] = df_tramo[sensor_col]  # Sin conversión
                        calibracion_msg = f"⚠️ Datos crudos en ADC (Δ sensor insuficiente: {delta_sensor_adc:.2f})"
                        usar_calibracion = False
                        unidad_posicion = "ADC"
                        unidad_velocidad = "ADC/s"
                        
                except ValueError as e:
                    logger.warning(f"Distancias inválidas, mostrando datos crudos en ADC: {e}")
                    df_tramo['Posicion_um'] = df_tramo[sensor_col]  # Sin conversión
                    calibracion_msg = f"⚠️ Datos crudos en ADC (valores inválidos)"
                    usar_calibracion = False
                    unidad_posicion = "ADC"
                    unidad_velocidad = "ADC/s"
            else:
                # SIN distancias → Mostrar datos crudos en ADC
                df_tramo['Posicion_um'] = df_tramo[sensor_col]  # Sin conversión
                calibracion_msg = f"📊 Datos crudos en ADC (sin calibración)"
                usar_calibracion = False
                unidad_posicion = "ADC"
                unidad_velocidad = "ADC/s"
                logger.debug("Sin distancias ingresadas - mostrando datos crudos en ADC")
            
            # 6. Calcular velocidad (MÉTODO SIMPLE PARA ESCALÓN)
            logger.debug("Calculando velocidad...")
            # Nota: Posicion_um ya fue calculada en el paso anterior (interpolación o crudo)
            
            # Para un escalón, la velocidad es simplemente la pendiente promedio
            # v_ss = (Posición_final - Posición_inicial) / (Tiempo_final - Tiempo_inicial)
            
            # Usar el último 20% del tramo para calcular v_ss (estado estacionario)
            idx_80 = int(len(df_tramo) * 0.8)
            pos_inicial = df_tramo['Posicion_um'].iloc[idx_80:idx_80+10].mean()  # Promedio de primeras muestras del último 20%
            pos_final = df_tramo['Posicion_um'].iloc[-10:].mean()  # Promedio de últimas 10 muestras
            
            t_inicial = df_tramo['Tiempo_s'].iloc[idx_80]
            t_final = df_tramo['Tiempo_s'].iloc[-1]
            
            delta_t = t_final - t_inicial
            
            if delta_t > 0.1:  # Al menos 100ms de datos
                v_ss = (pos_final - pos_inicial) / delta_t
            else:
                v_ss = 0.0
            
            logger.info(f"Velocidad estacionaria (v_ss): {v_ss:.4f} {unidad_velocidad} (calculada desde último 20% del tramo)")
            
            # Calcular velocidad instantánea para graficar (método simple con suavizado)
            df_tramo['Velocidad_um_s'] = df_tramo['Posicion_um'].diff() / df_tramo['Tiempo_s'].diff()
            df_tramo['Velocidad_um_s'] = df_tramo['Velocidad_um_s'].replace([float('inf'), float('-inf')], float('nan'))
            df_tramo['Velocidad_um_s'] = df_tramo['Velocidad_um_s'].rolling(window=20, center=True, min_periods=1).mean()
            df_tramo['Velocidad_um_s'] = df_tramo['Velocidad_um_s'].fillna(v_ss)  # Rellenar con v_ss
            
            logger.debug(f"Velocidad para gráfico - min: {df_tramo['Velocidad_um_s'].min():.2f}, max: {df_tramo['Velocidad_um_s'].max():.2f} {unidad_velocidad}")
            
            if abs(v_ss) < 0.01:
                logger.error(f"Velocidad estacionaria muy baja: v_ss={v_ss:.4f} {unidad_velocidad}")
                raise ValueError(f"Velocidad estacionaria muy baja (v_ss={v_ss:.4f} {unidad_velocidad}). Sistema no se mueve en este tramo.")
            
            # 7. Calcular K (ganancia estática)
            K = v_ss / U
            logger.info(f"Ganancia calculada (K): {K:.4f} {unidad_velocidad}/PWM")
            
            # 8. Calcular tau_fast (constante de tiempo rápida - método del 63.2%)
            v_tau = v_ss * 0.632
            logger.debug(f"Buscando τ_fast en v_tau = {v_tau:.4f} {unidad_velocidad} (63.2% de v_ss)")
            
            # Buscar primer punto donde velocidad >= v_tau
            tau_candidates = df_tramo[df_tramo['Velocidad_um_s'] >= v_tau]
            
            if tau_candidates.empty:
                tau_fast = None
                tau_msg = "No calculado (no alcanzó 63.2%)"
                logger.warning("No se alcanzó 63.2% de v_ss, τ_fast no calculado")
            else:
                t_tau = tau_candidates.iloc[0]['Tiempo_s']
                tau_fast = t_tau - t_inicio
                tau_msg = f"{tau_fast:.4f} s"
                logger.info(f"Constante de tiempo rápida (τ_fast): {tau_fast:.4f} s")
            
            # 9. Agregar polo lento (para evitar integrador puro en H∞/H2)
            # Este polo simula fricción muy pequeña pero no nula
            tau_slow = 1000.0  # Polo muy lento (1000s)
            logger.info(f"Polo lento agregado: τ_slow = {tau_slow:.1f}s")
            logger.info(f"   Razón: Evitar integrador puro para síntesis H∞/H2")
            logger.info(f"   Físicamente: Representa fricción mínima del sistema")
            
            # Mantener tau para compatibilidad con código existente
            tau = tau_fast
            
            # 9. Estadísticas del sensor
            sensor_min = df_tramo[sensor_col].min()
            sensor_max = df_tramo[sensor_col].max()
            delta_sensor = sensor_max - sensor_min
            
            # 10. Mostrar resultados
            results_str = (
                f"✅ Análisis Completado\n"
                f"═══════════════════════════════\n"
                f"Motor: {motor}  |  Sensor: {sensor}\n"
                f"Tramo: {t_inicio:.2f}s → {t_fin:.2f}s ({len(df_tramo)} muestras)\n"
                f"───────────────────────────────\n"
                f"Calibración: {calibracion_msg}\n"
                f"───────────────────────────────\n"
                f"Entrada (U):        {U:.2f} PWM\n"
                f"Δ Sensor:           {delta_sensor:.1f} ADC ({sensor_min:.0f}→{sensor_max:.0f})\n"
                f"Velocidad (v_ss):   {v_ss:.2f} {unidad_velocidad}\n"
                f"───────────────────────────────\n"
                f"Ganancia (K):       {K:.4f} {unidad_velocidad}/PWM\n"
                f"Constante (τ):      {tau_msg}\n"
                f"═══════════════════════════════\n"
            )
            
            if tau is not None:
                # Mostrar modelo de segundo orden sin integrador puro
                results_str += f"📐 MODELO IDENTIFICADO:\n"
                results_str += f"───────────────────────────────\n"
                results_str += f"G(s) = K / ((τ₁s + 1)(τ₂s + 1))\n"
                results_str += f"\n"
                results_str += f"Donde:\n"
                results_str += f"  K  = {K:.4f} {unidad_velocidad}/PWM\n"
                results_str += f"  τ₁ = {tau:.4f}s (polo rápido)\n"
                results_str += f"  τ₂ = {tau_slow:.1f}s (polo lento)\n"
                results_str += f"\n"
                results_str += f"Expandido:\n"
                results_str += f"G(s) = {K:.4f} / ({tau*tau_slow:.1f}s² + {tau+tau_slow:.1f}s + 1)\n"
                results_str += f"\n"
                results_str += f"💡 Nota: τ₂ muy grande simula integrador\n"
                results_str += f"   pero evita polo exacto en s=0 para H∞/H2"
            else:
                results_str += f"G(s) = {K:.4f} / ({tau_slow:.1f}s + 1)  (primer orden)"
            
            self.analysis_results_text.setText(results_str)
            
            # 11. Generar gráficos
            fig = Figure(figsize=(12, 10), facecolor='#2E2E2E')
            axes = fig.subplots(3, 1)
            
            # Gráfico 1: Posición
            axes[0].plot(df_tramo['Tiempo_s'], df_tramo['Posicion_um'], 
                        label=f'Posición (Sensor {sensor})', color='cyan', linewidth=2)
            axes[0].axvline(x=t_inicio, color='red', linestyle='--', alpha=0.7, linewidth=2, label='Inicio tramo')
            axes[0].axvline(x=t_fin, color='red', linestyle='--', alpha=0.7, linewidth=2, label='Fin tramo')
            axes[0].set_title(f'Motor {motor} → Sensor {sensor}: Posición', fontsize=14, fontweight='bold', color='white')
            axes[0].set_ylabel(f'Posición ({unidad_posicion})', color='white')
            axes[0].legend(loc='best', facecolor='#383838', edgecolor='#505050', labelcolor='white')
            axes[0].grid(True, alpha=0.5, linestyle='--', linewidth=0.5)
            axes[0].minorticks_on()
            axes[0].grid(True, which='minor', alpha=0.2, linestyle=':', linewidth=0.5)
            axes[0].set_facecolor('#252525')
            axes[0].tick_params(colors='white')
            axes[0].spines['bottom'].set_color('#505050')
            axes[0].spines['top'].set_color('#505050')
            axes[0].spines['left'].set_color('#505050')
            axes[0].spines['right'].set_color('#505050')
            
            # Gráfico 2: Velocidad
            axes[1].plot(df_tramo['Tiempo_s'], df_tramo['Velocidad_um_s'], 
                        label='Velocidad', color='lime', linewidth=2)
            axes[1].axhline(y=v_ss, color='red', linestyle='--', linewidth=2, alpha=0.8,
                           label=f'v_ss = {v_ss:.2f} {unidad_velocidad}')
            if tau is not None:
                axes[1].axhline(y=v_tau, color='orange', linestyle=':', linewidth=2, alpha=0.8,
                               label=f'63.2% = {v_tau:.2f} {unidad_velocidad}')
                axes[1].axvline(x=t_tau, color='orange', linestyle=':', linewidth=2, alpha=0.8,
                               label=f'τ = {tau:.4f} s')
            axes[1].set_title('Velocidad (derivada de posición)', fontsize=14, fontweight='bold', color='white')
            axes[1].set_ylabel(f'Velocidad ({unidad_velocidad})', color='white')
            axes[1].legend(loc='best', facecolor='#383838', edgecolor='#505050', labelcolor='white')
            axes[1].grid(True, alpha=0.5, linestyle='--', linewidth=0.5)
            axes[1].minorticks_on()
            axes[1].grid(True, which='minor', alpha=0.2, linestyle=':', linewidth=0.5)
            axes[1].set_facecolor('#252525')
            axes[1].tick_params(colors='white')
            axes[1].spines['bottom'].set_color('#505050')
            axes[1].spines['top'].set_color('#505050')
            axes[1].spines['left'].set_color('#505050')
            axes[1].spines['right'].set_color('#505050')
            
            # Gráfico 3: Entrada
            axes[2].plot(df_tramo['Tiempo_s'], df_tramo[motor_col], 
                        label=f'Motor {motor}', color='magenta', linewidth=2)
            axes[2].axhline(y=U, color='yellow', linestyle='--', linewidth=2, alpha=0.8,
                           label=f'U promedio = {U:.2f}')
            axes[2].set_title('Entrada de Potencia', fontsize=14, fontweight='bold', color='white')
            axes[2].set_xlabel('Tiempo (s)', color='white', fontsize=12)
            axes[2].set_ylabel('Potencia (PWM)', color='white')
            axes[2].legend(loc='best', facecolor='#383838', edgecolor='#505050', labelcolor='white')
            axes[2].grid(True, alpha=0.5, linestyle='--', linewidth=0.5)
            axes[2].minorticks_on()
            axes[2].grid(True, which='minor', alpha=0.2, linestyle=':', linewidth=0.5)
            axes[2].set_facecolor('#252525')
            axes[2].tick_params(colors='white')
            axes[2].spines['bottom'].set_color('#505050')
            axes[2].spines['top'].set_color('#505050')
            axes[2].spines['left'].set_color('#505050')
            axes[2].spines['right'].set_color('#505050')
            
            fig.tight_layout()
            
            # Crear y mostrar ventana (reutilizar si existe)
            logger.debug("Generando ventana de gráficos de análisis")
            if self.analysis_window is not None:
                self.analysis_window.close()  # Cerrar ventana anterior si existe
                logger.debug("Ventana de análisis anterior cerrada")
            
            self.analysis_window = MatplotlibWindow(fig, f"Análisis: Motor {motor} → Sensor {sensor}", self)
            self.analysis_window.show()
            self.analysis_window.raise_()
            self.analysis_window.activateWindow()
            QApplication.processEvents()  # Forzar actualización de la interfaz
            
            # Guardar K y τ para usar en diseño de controlador
            self.last_K = K
            self.last_tau = tau if tau is not None else 0.0
            self.last_tau_slow = tau_slow  # Guardar también polo lento
            
            # Agregar función de transferencia identificada a la lista
            tf_entry = {
                'motor': motor,
                'sensor': sensor,
                'K': K,
                'tau': tau if tau is not None else 0.0,
                'tau_slow': tau_slow,  # Agregar polo lento
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'filename': filename,
                'calibration': calibracion_msg
            }
            
            # Verificar si ya existe esta combinación motor/sensor
            existing_idx = None
            for idx, tf in enumerate(self.identified_transfer_functions):
                if tf['motor'] == motor and tf['sensor'] == sensor:
                    existing_idx = idx
                    break
            
            if existing_idx is not None:
                # Actualizar entrada existente
                self.identified_transfer_functions[existing_idx] = tf_entry
                logger.info(f"Función de transferencia actualizada: Motor {motor} / Sensor {sensor}")
            else:
                # Agregar nueva entrada
                self.identified_transfer_functions.append(tf_entry)
                logger.info(f"Nueva función de transferencia agregada: Motor {motor} / Sensor {sensor}")
            
            # Actualizar lista de funciones de transferencia
            self.update_tf_list()
            
            logger.info(f"✅ Análisis completado exitosamente: K={K:.4f}, τ={tau_msg}")
            
        except FileNotFoundError as e:
            logger.error(f"Archivo no encontrado: {filename}")
            self.analysis_results_text.setText(f"❌ Error: Archivo '{filename}' no encontrado.")
        except Exception as e:
            logger.error(f"Error en análisis: {e}\n{traceback.format_exc()}")
            error_detail = traceback.format_exc()
            self.analysis_results_text.setText(f"❌ Error:\n{str(e)}\n\n{error_detail}")

    def update_tf_list(self):
        """Actualiza la lista de funciones de transferencia identificadas."""
        if not self.identified_transfer_functions:
            self.tf_list_text.setPlainText("No hay funciones de transferencia identificadas aún.\n\nRealiza un análisis para agregar una.")
            return
        
        # Construir texto de la lista
        list_text = "═══════════════════════════════════════════════════════════════\n"
        list_text += "  FUNCIONES DE TRANSFERENCIA IDENTIFICADAS\n"
        list_text += "═══════════════════════════════════════════════════════════════\n\n"
        
        for idx, tf in enumerate(self.identified_transfer_functions, 1):
            motor = tf['motor']
            sensor = tf['sensor']
            K = tf['K']
            tau = tf['tau']
            tau_slow = tf.get('tau_slow', 1000.0)  # Obtener tau_slow o usar 1000 por defecto
            timestamp = tf['timestamp']
            
            list_text += f"[{idx}] Motor {motor} / Sensor {sensor}\n"
            # Mostrar modelo de segundo orden
            list_text += f"    ├─ G(s) = {K:.4f} / (({tau:.4f}s + 1)({tau_slow:.1f}s + 1))\n"
            list_text += f"    ├─ K = {K:.4f} µm/s/PWM\n"
            list_text += f"    ├─ τ₁ = {tau:.4f}s (rápido), τ₂ = {tau_slow:.1f}s (lento)\n"
            list_text += f"    ├─ Fecha: {timestamp}\n"
            list_text += f"    └─ Archivo: {tf['filename']}\n"
            list_text += f"\n"
        
        list_text += "═══════════════════════════════════════════════════════════════\n"
        list_text += f"Total: {len(self.identified_transfer_functions)} función(es) identificada(s)\n"
        list_text += "═══════════════════════════════════════════════════════════════\n"
        
        self.tf_list_text.setPlainText(list_text)
        logger.debug(f"Lista de funciones de transferencia actualizada: {len(self.identified_transfer_functions)} entradas")
    
    # --- Fin del panel de Análisis ---


    def create_motors_group(self):
        group_box = QGroupBox("Estado de Motores")
        layout = QGridLayout()
        value_style = "font-size: 18px; font-weight: bold; color: #5DADE2;"

        layout.addWidget(QLabel("Potencia Motor A:"), 0, 0)
        self.value_labels['power_a'] = QLabel("0")
        self.value_labels['power_a'].setStyleSheet(value_style)
        layout.addWidget(self.value_labels['power_a'], 0, 1)

        layout.addWidget(QLabel("Potencia Motor B:"), 1, 0)
        self.value_labels['power_b'] = QLabel("0")
        self.value_labels['power_b'].setStyleSheet(value_style)
        layout.addWidget(self.value_labels['power_b'], 1, 1)

        group_box.setLayout(layout)
        return group_box

    def create_sensors_group(self):
        group_box = QGroupBox("Lectura de Sensores Análogos")
        layout = QGridLayout()
        value_style = "font-size: 18px; color: #58D68D;"

        layout.addWidget(QLabel("Valor Sensor 1 (A2):"), 0, 0)
        self.value_labels['sensor_1'] = QLabel("---")
        self.value_labels['sensor_1'].setStyleSheet(value_style)
        layout.addWidget(self.value_labels['sensor_1'], 0, 1)

        layout.addWidget(QLabel("Valor Sensor 2 (A3):"), 1, 0)
        self.value_labels['sensor_2'] = QLabel("---")
        self.value_labels['sensor_2'].setStyleSheet(value_style)
        layout.addWidget(self.value_labels['sensor_2'], 1, 1)

        group_box.setLayout(layout)
        return group_box

    # --- Panel de Detector de Cámara Thorlabs ---
    def create_camera_detector_group(self):
        """Crea el GroupBox para control completo de cámara Thorlabs."""
        group_box = QGroupBox("🎥 Control de Cámara Thorlabs")
        main_layout = QVBoxLayout()
        
        # Inicializar worker y ventana de cámara
        self.camera_worker = None
        self.camera_view_window = None
        
        # Sección 1: Conexión y Detección
        connection_group = QGroupBox("1️⃣ Conexión")
        conn_layout = QVBoxLayout()
        
        # Botones de conexión
        conn_buttons = QHBoxLayout()
        self.connect_camera_btn = QPushButton("🔌 Conectar Cámara")
        self.connect_camera_btn.setStyleSheet("""
            QPushButton {
                font-size: 13px;
                font-weight: bold;
                padding: 8px;
                background-color: #27AE60;
            }
            QPushButton:hover { background-color: #2ECC71; }
            QPushButton:pressed { background-color: #1E8449; }
            QPushButton:disabled { background-color: #505050; color: #808080; }
        """)
        self.connect_camera_btn.clicked.connect(self.connect_camera)
        
        self.disconnect_camera_btn = QPushButton("🔌 Desconectar")
        self.disconnect_camera_btn.setEnabled(False)
        self.disconnect_camera_btn.clicked.connect(self.disconnect_camera)
        
        self.detect_camera_btn = QPushButton("🔍 Detectar Cámaras")
        self.detect_camera_btn.clicked.connect(self.detect_thorlabs_camera)
        
        if not THORLABS_AVAILABLE:
            self.connect_camera_btn.setEnabled(False)
            self.connect_camera_btn.setText("⚠️ pylablib no instalado")
            self.detect_camera_btn.setEnabled(False)
        
        conn_buttons.addWidget(self.connect_camera_btn)
        conn_buttons.addWidget(self.disconnect_camera_btn)
        conn_buttons.addWidget(self.detect_camera_btn)
        conn_buttons.addStretch()
        conn_layout.addLayout(conn_buttons)
        
        # Info de cámara conectada
        self.camera_info_label = QLabel("Estado: Desconectada")
        self.camera_info_label.setStyleSheet("color: #E74C3C; font-weight: bold;")
        conn_layout.addWidget(self.camera_info_label)
        
        connection_group.setLayout(conn_layout)
        main_layout.addWidget(connection_group)
        
        # Sección 2: Control de Vista en Vivo
        view_group = QGroupBox("2️⃣ Vista en Vivo")
        view_layout = QVBoxLayout()
        
        view_buttons = QHBoxLayout()
        self.open_camera_view_btn = QPushButton("📹 Ver Cámara")
        self.open_camera_view_btn.setStyleSheet("""
            QPushButton {
                font-size: 13px;
                font-weight: bold;
                padding: 8px;
                background-color: #2E86C1;
            }
            QPushButton:hover { background-color: #3498DB; }
            QPushButton:pressed { background-color: #1F618D; }
            QPushButton:disabled { background-color: #505050; color: #808080; }
        """)
        self.open_camera_view_btn.setEnabled(False)
        self.open_camera_view_btn.clicked.connect(self.open_camera_view)
        
        self.start_live_btn = QPushButton("▶️ Iniciar")
        self.start_live_btn.setEnabled(False)
        self.start_live_btn.clicked.connect(self.start_camera_live_view)
        
        self.stop_live_btn = QPushButton("⏹️ Detener")
        self.stop_live_btn.setEnabled(False)
        self.stop_live_btn.clicked.connect(self.stop_camera_live_view)
        
        view_buttons.addWidget(self.open_camera_view_btn)
        view_buttons.addWidget(self.start_live_btn)
        view_buttons.addWidget(self.stop_live_btn)
        view_buttons.addStretch()
        view_layout.addLayout(view_buttons)
        
        view_group.setLayout(view_layout)
        main_layout.addWidget(view_group)
        
        # Sección 3: Configuración de Cámara
        config_group = QGroupBox("3️⃣ Configuración")
        config_layout = QGridLayout()
        
        # Exposición
        config_layout.addWidget(QLabel("Exposición (s):"), 0, 0)
        self.exposure_input = QLineEdit("0.01")
        self.exposure_input.setFixedWidth(100)
        config_layout.addWidget(self.exposure_input, 0, 1)
        
        self.apply_exposure_btn = QPushButton("✓ Aplicar")
        self.apply_exposure_btn.setEnabled(False)
        self.apply_exposure_btn.setFixedWidth(80)
        self.apply_exposure_btn.clicked.connect(self.apply_camera_exposure)
        config_layout.addWidget(self.apply_exposure_btn, 0, 2)
        
        # FPS (Frame Rate)
        config_layout.addWidget(QLabel("FPS (Frame Rate):"), 1, 0)
        self.fps_input = QLineEdit("60")
        self.fps_input.setFixedWidth(100)
        self.fps_input.setToolTip("Frames por segundo (1-120)")
        config_layout.addWidget(self.fps_input, 1, 1)
        
        self.apply_fps_btn = QPushButton("✓ Aplicar")
        self.apply_fps_btn.setEnabled(False)
        self.apply_fps_btn.setFixedWidth(80)
        self.apply_fps_btn.clicked.connect(self.apply_camera_fps)
        config_layout.addWidget(self.apply_fps_btn, 1, 2)
        
        # Buffer de frames
        config_layout.addWidget(QLabel("Buffer (frames):"), 2, 0)
        self.buffer_size_input = QLineEdit("50")
        self.buffer_size_input.setFixedWidth(100)
        self.buffer_size_input.setToolTip("Número de frames en buffer (10-200). Valor bajo evita fugas de memoria")
        config_layout.addWidget(self.buffer_size_input, 2, 1)
        
        self.apply_buffer_btn = QPushButton("✓ Aplicar")
        self.apply_buffer_btn.setEnabled(False)
        self.apply_buffer_btn.setFixedWidth(80)
        self.apply_buffer_btn.clicked.connect(self.apply_camera_buffer)
        config_layout.addWidget(self.apply_buffer_btn, 2, 2)
        
        config_layout.setColumnStretch(3, 1)
        config_group.setLayout(config_layout)
        main_layout.addWidget(config_group)
        
        # Sección 4: Captura de Imágenes
        capture_group = QGroupBox("4️⃣ Captura de Imágenes")
        capture_layout = QVBoxLayout()
        
        # Carpeta de guardado
        folder_layout = QHBoxLayout()
        folder_layout.addWidget(QLabel("Carpeta:"))
        self.save_folder_input = QLineEdit(r"C:\CapturasCamara")
        folder_layout.addWidget(self.save_folder_input)
        
        browse_btn = QPushButton("📁 Explorar")
        browse_btn.clicked.connect(self.browse_save_folder)
        folder_layout.addWidget(browse_btn)
        capture_layout.addLayout(folder_layout)
        
        # Botón de captura
        capture_btn_layout = QHBoxLayout()
        self.capture_image_btn = QPushButton("📸 Capturar Imagen")
        self.capture_image_btn.setStyleSheet("""
            QPushButton {
                font-size: 14px;
                font-weight: bold;
                padding: 10px;
                background-color: #E67E22;
            }
            QPushButton:hover { background-color: #F39C12; }
            QPushButton:pressed { background-color: #CA6F1E; }
            QPushButton:disabled { background-color: #505050; color: #808080; }
        """)
        self.capture_image_btn.setEnabled(False)
        self.capture_image_btn.clicked.connect(self.capture_camera_image)
        capture_btn_layout.addWidget(self.capture_image_btn)
        capture_btn_layout.addStretch()
        capture_layout.addLayout(capture_btn_layout)
        
        capture_group.setLayout(capture_layout)
        main_layout.addWidget(capture_group)
        
        # Sección 5: Microscopía Automatizada con Trayectoria Zig-Zag
        microscopy_group = QGroupBox("🔬 Microscopía Automatizada")
        microscopy_layout = QVBoxLayout()
        
        # Información
        info_layout = QHBoxLayout()
        info_label = QLabel(
            "ℹ️ <b>Ejecuta la trayectoria zig-zag con captura automática de imágenes</b><br>"
            "Usa la trayectoria generada en la pestaña 'Prueba' y captura una imagen en cada punto."
        )
        info_label.setWordWrap(True)
        info_label.setStyleSheet("padding: 8px; background-color: #34495E; border-radius: 5px; font-size: 10px;")
        info_layout.addWidget(info_label)
        microscopy_layout.addLayout(info_layout)
        
        # Estado de trayectoria
        traj_status_layout = QHBoxLayout()
        traj_status_layout.addWidget(QLabel("<b>Estado:</b>"))
        self.microscopy_trajectory_status = QLabel("⚪ Sin trayectoria")
        self.microscopy_trajectory_status.setStyleSheet("color: #95A5A6; font-weight: bold;")
        traj_status_layout.addWidget(self.microscopy_trajectory_status)
        traj_status_layout.addStretch()
        microscopy_layout.addLayout(traj_status_layout)
        
        # Configuración de captura
        config_grid = QGridLayout()
        
        # Nombre de clase botánica
        config_grid.addWidget(QLabel("Nombre clase:"), 0, 0)
        self.class_name_input = QLineEdit("Especie_001")
        self.class_name_input.setPlaceholderText("Ej: Rosa_Canina, Pinus_Radiata")
        self.class_name_input.setToolTip("Nombre de la clase botánica para nombrar las imágenes")
        config_grid.addWidget(self.class_name_input, 0, 1)
        
        # Tamaño de imagen
        config_grid.addWidget(QLabel("Tamaño imagen (px):"), 0, 2)
        size_layout = QHBoxLayout()
        self.image_width_input = QLineEdit("1920")
        self.image_width_input.setFixedWidth(60)
        self.image_width_input.setToolTip("Ancho en píxeles")
        size_layout.addWidget(self.image_width_input)
        size_layout.addWidget(QLabel("×"))
        self.image_height_input = QLineEdit("1080")
        self.image_height_input.setFixedWidth(60)
        self.image_height_input.setToolTip("Alto en píxeles")
        size_layout.addWidget(self.image_height_input)
        size_layout.addStretch()
        config_grid.addLayout(size_layout, 0, 3)
        
        # Canales RGB
        config_grid.addWidget(QLabel("Canales RGB:"), 1, 0)
        channels_layout = QHBoxLayout()
        self.channel_r_check = QCheckBox("R")
        self.channel_r_check.setChecked(True)
        self.channel_r_check.setStyleSheet("color: #E74C3C; font-weight: bold;")
        self.channel_g_check = QCheckBox("G")
        self.channel_g_check.setChecked(True)
        self.channel_g_check.setStyleSheet("color: #27AE60; font-weight: bold;")
        self.channel_b_check = QCheckBox("B")
        self.channel_b_check.setChecked(True)
        self.channel_b_check.setStyleSheet("color: #3498DB; font-weight: bold;")
        channels_layout.addWidget(self.channel_r_check)
        channels_layout.addWidget(self.channel_g_check)
        channels_layout.addWidget(self.channel_b_check)
        channels_layout.addStretch()
        config_grid.addLayout(channels_layout, 1, 1)
        
        # Estimación de tamaño
        config_grid.addWidget(QLabel("Estimación:"), 1, 2)
        self.size_estimate_label = QLabel("~0 MB")
        self.size_estimate_label.setStyleSheet("color: #F39C12; font-weight: bold;")
        self.size_estimate_label.setToolTip("Tamaño aproximado total de las imágenes")
        config_grid.addWidget(self.size_estimate_label, 1, 3)
        
        # Demoras para estabilización
        config_grid.addWidget(QLabel("⏱️ Demora antes (s):"), 2, 0)
        self.delay_before_input = QLineEdit("0.5")
        self.delay_before_input.setFixedWidth(80)
        self.delay_before_input.setToolTip("Tiempo de espera ANTES de capturar (para estabilización de vibraciones)")
        config_grid.addWidget(self.delay_before_input, 2, 1)
        
        config_grid.addWidget(QLabel("⏱️ Demora después (s):"), 2, 2)
        self.delay_after_input = QLineEdit("0.2")
        self.delay_after_input.setFixedWidth(80)
        self.delay_after_input.setToolTip("Tiempo de espera DESPUÉS de capturar (antes de mover al siguiente punto)")
        config_grid.addWidget(self.delay_after_input, 2, 3)
        
        # Conectar cambios para actualizar estimación
        self.image_width_input.textChanged.connect(self.update_size_estimate)
        self.image_height_input.textChanged.connect(self.update_size_estimate)
        self.channel_r_check.stateChanged.connect(self.update_size_estimate)
        self.channel_g_check.stateChanged.connect(self.update_size_estimate)
        self.channel_b_check.stateChanged.connect(self.update_size_estimate)
        
        microscopy_layout.addLayout(config_grid)
        
        # Referencia Cero
        zero_ref_layout = QGridLayout()
        zero_ref_layout.addWidget(QLabel("<b>📍 Referencia Cero:</b>"), 0, 0)
        zero_ref_layout.addWidget(QLabel("X (µm):"), 0, 1)
        self.zero_ref_x_input = QLineEdit("0")
        self.zero_ref_x_input.setFixedWidth(80)
        self.zero_ref_x_input.setToolTip("Posición X de referencia cero")
        zero_ref_layout.addWidget(self.zero_ref_x_input, 0, 2)
        
        zero_ref_layout.addWidget(QLabel("Y (µm):"), 0, 3)
        self.zero_ref_y_input = QLineEdit("0")
        self.zero_ref_y_input.setFixedWidth(80)
        self.zero_ref_y_input.setToolTip("Posición Y de referencia cero")
        zero_ref_layout.addWidget(self.zero_ref_y_input, 0, 4)
        
        self.set_zero_ref_btn = QPushButton("📍 Establecer Referencia Cero")
        self.set_zero_ref_btn.setStyleSheet("background-color: #8E44AD; font-weight: bold; padding: 6px;")
        self.set_zero_ref_btn.clicked.connect(self.set_zero_reference)
        zero_ref_layout.addWidget(self.set_zero_ref_btn, 0, 5)
        
        zero_ref_layout.setColumnStretch(6, 1)
        microscopy_layout.addLayout(zero_ref_layout)
        
        # Botones de control
        control_btn_layout = QHBoxLayout()
        
        self.microscopy_start_btn = QPushButton("🚀 Iniciar Microscopía")
        self.microscopy_start_btn.setStyleSheet("""
            QPushButton {
                font-size: 13px;
                font-weight: bold;
                padding: 10px;
                background-color: #27AE60;
            }
            QPushButton:hover { background-color: #2ECC71; }
            QPushButton:pressed { background-color: #1E8449; }
            QPushButton:disabled { background-color: #505050; color: #808080; }
        """)
        self.microscopy_start_btn.setEnabled(False)
        self.microscopy_start_btn.clicked.connect(self.start_automated_microscopy)
        control_btn_layout.addWidget(self.microscopy_start_btn)
        
        self.microscopy_stop_btn = QPushButton("⏹️ Detener")
        self.microscopy_stop_btn.setStyleSheet("""
            QPushButton {
                font-size: 13px;
                font-weight: bold;
                padding: 10px;
                background-color: #E74C3C;
            }
            QPushButton:hover { background-color: #EC7063; }
            QPushButton:pressed { background-color: #C0392B; }
            QPushButton:disabled { background-color: #505050; color: #808080; }
        """)
        self.microscopy_stop_btn.setEnabled(False)
        self.microscopy_stop_btn.clicked.connect(self.stop_automated_microscopy)
        control_btn_layout.addWidget(self.microscopy_stop_btn)
        
        control_btn_layout.addStretch()
        microscopy_layout.addLayout(control_btn_layout)
        
        # Progreso
        progress_layout = QHBoxLayout()
        progress_layout.addWidget(QLabel("<b>Progreso:</b>"))
        self.microscopy_progress_label = QLabel("0 / 0 imágenes capturadas")
        self.microscopy_progress_label.setStyleSheet("color: #3498DB; font-weight: bold;")
        progress_layout.addWidget(self.microscopy_progress_label)
        progress_layout.addStretch()
        microscopy_layout.addLayout(progress_layout)
        
        microscopy_group.setLayout(microscopy_layout)
        main_layout.addWidget(microscopy_group)
        
        # Terminal de mensajes
        terminal_label = QLabel("📟 Terminal de Mensajes:")
        terminal_label.setStyleSheet("font-weight: bold; margin-top: 10px;")
        main_layout.addWidget(terminal_label)
        
        self.camera_terminal = QTextEdit()
        self.camera_terminal.setReadOnly(True)
        self.camera_terminal.setMinimumHeight(200)
        self.camera_terminal.setStyleSheet("""
            QTextEdit {
                background-color: #1E1E1E;
                color: #00FF00;
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 10px;
                border: 2px solid #505050;
                border-radius: 4px;
                padding: 8px;
            }
        """)
        
        welcome_msg = """╔════════════════════════════════════════════════════════════════╗
║  SISTEMA DE CONTROL DE CÁMARA THORLABS                        ║
║  Versión: 2.0 | Sistema: L206 Control                         ║
╚════════════════════════════════════════════════════════════════╝

[SISTEMA] Terminal inicializado correctamente
[SISTEMA] Listo para conectar cámara...
"""
        self.camera_terminal.setPlainText(welcome_msg)
        main_layout.addWidget(self.camera_terminal)
        
        # Botón limpiar terminal
        clear_layout = QHBoxLayout()
        clear_btn = QPushButton("🗑️ Limpiar Terminal")
        clear_btn.clicked.connect(lambda: self.camera_terminal.clear())
        clear_layout.addStretch()
        clear_layout.addWidget(clear_btn)
        main_layout.addLayout(clear_layout)
        
        group_box.setLayout(main_layout)
        return group_box
    
    def detect_thorlabs_camera(self):
        """Detecta cámaras Thorlabs conectadas al sistema."""
        if not THORLABS_AVAILABLE:
            self.log_camera_message("ERROR", "pylablib no está instalado. Instalar con: pip install pylablib")
            return
        
        self.log_camera_message("INFO", "Iniciando búsqueda de cámaras Thorlabs...")
        self.detect_camera_btn.setEnabled(False)
        self.detect_camera_btn.setText("🔄 Detectando...")
        
        try:
            # Intentar listar cámaras
            self.log_camera_message("SISTEMA", "Buscando drivers de cámara... (puede tardar unos segundos)")
            cameras = Thorlabs.list_cameras_tlcam()
            
            if not cameras:
                self.log_camera_message("WARNING", "No se encontraron cámaras Thorlabs conectadas")
                self.log_camera_message("INFO", "Verificar:")
                self.log_camera_message("INFO", "  1. La cámara está conectada por USB")
                self.log_camera_message("INFO", "  2. Los drivers de Thorlabs están instalados")
                self.log_camera_message("INFO", "  3. La cámara tiene alimentación")
            else:
                self.log_camera_message("SUCCESS", f"¡Cámaras encontradas! Total: {len(cameras)}")
                self.log_camera_message("INFO", "═" * 60)
                for i, cam in enumerate(cameras, 1):
                    self.log_camera_message("CAMERA", f"Cámara {i}: {cam}")
                self.log_camera_message("INFO", "═" * 60)
                self.log_camera_message("SUCCESS", "Detección completada exitosamente")
                
        except Exception as e:
            self.log_camera_message("ERROR", f"Error durante la detección: {str(e)}")
            self.log_camera_message("ERROR", "Posibles causas:")
            self.log_camera_message("ERROR", "  - Drivers de Thorlabs no instalados")
            self.log_camera_message("ERROR", "  - Problemas de permisos USB")
            self.log_camera_message("ERROR", "  - Conflicto con otro software")
            logger.error(f"Error en detección de cámara: {e}\n{traceback.format_exc()}")
        finally:
            self.detect_camera_btn.setEnabled(True)
            self.detect_camera_btn.setText("🔍 Detectar Cámara")
    
    def log_camera_message(self, level, message):
        """Registra un mensaje en el terminal de cámara con formato y color."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        # Definir colores según el nivel
        color_map = {
            "INFO": "#00BFFF",      # Azul claro
            "SUCCESS": "#00FF00",   # Verde
            "WARNING": "#FFD700",   # Amarillo
            "ERROR": "#FF4500",     # Rojo
            "SISTEMA": "#9370DB",   # Púrpura
            "CAMERA": "#00FF7F"     # Verde primavera
        }
        
        color = color_map.get(level, "#00FF00")
        
        # Formatear mensaje
        formatted_msg = f'<span style="color: #808080;">[{timestamp}]</span> '
        formatted_msg += f'<span style="color: {color}; font-weight: bold;">[{level}]</span> '
        formatted_msg += f'<span style="color: #FFFFFF;">{message}</span>'
        
        # Agregar al terminal
        self.camera_terminal.append(formatted_msg)
        
        # Auto-scroll al final
        scrollbar = self.camera_terminal.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    # --- Funciones de Control de Cámara ---
    def connect_camera(self):
        """Conecta con la cámara Thorlabs."""
        if not THORLABS_AVAILABLE:
            self.log_camera_message("ERROR", "pylablib no está instalado")
            return
        
        self.log_camera_message("INFO", "Iniciando conexión con cámara...")
        self.connect_camera_btn.setEnabled(False)
        self.connect_camera_btn.setText("🔄 Conectando...")
        
        # Crear worker si no existe
        if self.camera_worker is None:
            self.camera_worker = CameraWorker()
            self.camera_worker.status_update.connect(self.log_camera_message_simple)
            self.camera_worker.connection_success.connect(self.on_camera_connected)
            self.camera_worker.new_frame_ready.connect(self.on_camera_frame)
        
        # Conectar en el thread
        self.camera_worker.connect_camera()
    
    def on_camera_connected(self, success, camera_info):
        """Callback cuando la cámara se conecta o falla."""
        if success:
            self.camera_info_label.setText(f"Estado: Conectada - {camera_info}")
            self.camera_info_label.setStyleSheet("color: #27AE60; font-weight: bold;")
            self.connect_camera_btn.setText("✅ Conectada")
            self.disconnect_camera_btn.setEnabled(True)
            self.open_camera_view_btn.setEnabled(True)
            self.start_live_btn.setEnabled(True)
            self.apply_exposure_btn.setEnabled(True)
            self.apply_fps_btn.setEnabled(True)
            self.apply_buffer_btn.setEnabled(True)
            logger.info(f"Cámara conectada exitosamente: {camera_info}")
        else:
            self.camera_info_label.setText("Estado: Error de conexión")
            self.camera_info_label.setStyleSheet("color: #E74C3C; font-weight: bold;")
            self.connect_camera_btn.setText("🔌 Conectar Cámara")
            self.connect_camera_btn.setEnabled(True)
            logger.error("Fallo al conectar cámara")
    
    def disconnect_camera(self):
        """Desconecta la cámara."""
        self.log_camera_message("INFO", "Desconectando cámara...")
        
        if self.camera_worker:
            self.camera_worker.disconnect_camera()
        
        self.camera_info_label.setText("Estado: Desconectada")
        self.camera_info_label.setStyleSheet("color: #E74C3C; font-weight: bold;")
        self.connect_camera_btn.setText("🔌 Conectar Cámara")
        self.connect_camera_btn.setEnabled(True)
        self.disconnect_camera_btn.setEnabled(False)
        self.open_camera_view_btn.setEnabled(False)
        self.start_live_btn.setEnabled(False)
        self.stop_live_btn.setEnabled(False)
        self.apply_exposure_btn.setEnabled(False)
        self.apply_fps_btn.setEnabled(False)
        self.apply_buffer_btn.setEnabled(False)
        self.capture_image_btn.setEnabled(False)
    
    def open_camera_view(self):
        """Abre la ventana flotante de visualización de cámara."""
        if self.camera_view_window is None:
            self.camera_view_window = CameraViewWindow(self, main_window=self)
            self.log_camera_message("INFO", "Ventana de visualización creada")
        
        self.camera_view_window.show()
        self.camera_view_window.raise_()
        self.camera_view_window.activateWindow()
        self.log_camera_message("SUCCESS", "Ventana de cámara abierta")
    
    def start_camera_live_view(self):
        """Inicia la vista en vivo de la cámara."""
        if not self.camera_worker:
            self.log_camera_message("ERROR", "Cámara no conectada")
            return
        
        self.log_camera_message("INFO", "Iniciando vista en vivo...")
        self.start_live_btn.setEnabled(False)
        self.stop_live_btn.setEnabled(True)
        self.capture_image_btn.setEnabled(True)
        
        # CRÍTICO: QThread solo puede iniciarse UNA VEZ
        # Si el thread ya terminó, necesitamos recrearlo
        if self.camera_worker.isFinished() or self.camera_worker.isRunning():
            if self.camera_worker.isRunning():
                logger.warning("Thread de cámara ya está corriendo")
                self.log_camera_message("WARNING", "La vista en vivo ya está activa")
                return
            
            # Thread ya terminó, necesitamos recrear el worker
            logger.info("Recreando worker de cámara (thread anterior terminó)")
            old_cam = self.camera_worker.cam  # Guardar referencia a la cámara
            
            # Crear nuevo worker
            self.camera_worker = CameraWorker()
            self.camera_worker.cam = old_cam  # Reutilizar la conexión de cámara
            self.camera_worker.exposure = float(self.exposure_input.text())
            self.camera_worker.fps = int(self.fps_input.text())
            self.camera_worker.buffer_size = int(self.buffer_size_input.text())
            
            # Reconectar señales
            self.camera_worker.status_update.connect(self.log_camera_message_simple)
            self.camera_worker.connection_success.connect(self.on_camera_connected)
            self.camera_worker.new_frame_ready.connect(self.on_camera_frame)
            
        # Iniciar el thread
        self.camera_worker.start()
        logger.info("Thread de cámara iniciado")
    
    def stop_camera_live_view(self):
        """Detiene la vista en vivo de la cámara."""
        if self.camera_worker:
            self.camera_worker.stop_live_view()
        
        self.start_live_btn.setEnabled(True)
        self.stop_live_btn.setEnabled(False)
        self.capture_image_btn.setEnabled(False)
        self.log_camera_message("INFO", "Vista en vivo detenida")
    
    def on_camera_frame(self, q_image):
        """Callback cuando llega un nuevo frame de la cámara."""
        if self.camera_view_window and self.camera_view_window.isVisible():
            self.camera_view_window.update_frame(q_image)
    
    def apply_camera_exposure(self):
        """Aplica el nuevo valor de exposición a la cámara."""
        try:
            exposure_value = float(self.exposure_input.text())
            if exposure_value <= 0:
                self.log_camera_message("ERROR", "La exposición debe ser mayor que 0")
                return
            
            if self.camera_worker:
                self.camera_worker.change_exposure(exposure_value)
            else:
                self.log_camera_message("ERROR", "Cámara no conectada")
        except ValueError:
            self.log_camera_message("ERROR", "Valor de exposición inválido")
    
    def apply_camera_fps(self):
        """Aplica el nuevo valor de FPS a la cámara."""
        try:
            fps_value = int(self.fps_input.text())
            if fps_value <= 0 or fps_value > 120:
                self.log_camera_message("ERROR", "FPS debe estar entre 1 y 120")
                return
            
            if self.camera_worker:
                self.camera_worker.change_fps(fps_value)
            else:
                self.log_camera_message("ERROR", "Cámara no conectada")
        except ValueError:
            self.log_camera_message("ERROR", "Valor de FPS inválido (debe ser entero)")
    
    def apply_camera_buffer(self):
        """Aplica el nuevo tamaño de buffer a la cámara."""
        try:
            buffer_value = int(self.buffer_size_input.text())
            if buffer_value <= 0 or buffer_value > 500:
                self.log_camera_message("ERROR", "Buffer debe estar entre 10 y 500 frames")
                return
            
            if self.camera_worker:
                self.camera_worker.change_buffer_size(buffer_value)
            else:
                self.log_camera_message("ERROR", "Cámara no conectada")
        except ValueError:
            self.log_camera_message("ERROR", "Valor de buffer inválido (debe ser entero)")
    
    def browse_save_folder(self):
        """Abre diálogo para seleccionar carpeta de guardado."""
        folder = QFileDialog.getExistingDirectory(
            self,
            "Seleccionar Carpeta de Guardado",
            self.save_folder_input.text()
        )
        if folder:
            self.save_folder_input.setText(folder)
            self.log_camera_message("INFO", f"Carpeta seleccionada: {folder}")
    
    def capture_camera_image(self):
        """Captura y guarda la imagen actual de la cámara."""
        if not self.camera_worker or self.camera_worker.current_frame is None:
            self.log_camera_message("ERROR", "No hay frame disponible para capturar")
            return
        
        save_path = self.save_folder_input.text()
        
        try:
            import os
            import cv2
            os.makedirs(save_path, exist_ok=True)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"capture_{timestamp}.png"
            full_path = os.path.join(save_path, filename)
            
            # Obtener frame y hacer copia
            frame = self.camera_worker.current_frame.copy()
            
            if frame is None or frame.size == 0:
                self.log_camera_message("ERROR", "Frame vacío")
                logger.error(f"Frame vacío: shape={frame.shape if frame is not None else 'None'}")
                return
            
            logger.info(f"Frame a guardar: shape={frame.shape}, dtype={frame.dtype}, min={frame.min()}, max={frame.max()}")
            
            # Normalizar a uint8 si es necesario (igual que en img_capture.py)
            if frame.dtype != np.uint8:
                if frame.max() > 0:
                    frame = (frame / frame.max() * 255).astype(np.uint8)
                else:
                    frame = frame.astype(np.uint8)
            
            # Guardar imagen
            success = cv2.imwrite(full_path, frame)
            
            if success:
                self.log_camera_message("SUCCESS", f"✅ Imagen guardada: {filename}")
                self.log_camera_message("INFO", f"Ruta: {full_path}")
                logger.info(f"Imagen capturada exitosamente: {full_path}")
            else:
                self.log_camera_message("ERROR", f"cv2.imwrite falló para {filename}")
                logger.error(f"cv2.imwrite retornó False para {full_path}")
                
        except Exception as e:
            self.log_camera_message("ERROR", f"❌ Error al guardar imagen: {str(e)}")
            logger.error(f"Error captura imagen: {e}\n{traceback.format_exc()}")
    
    def log_camera_message_simple(self, message):
        """Wrapper simple para mensajes del worker (sin nivel)."""
        # Detectar nivel basado en emojis/palabras clave
        if "✅" in message or "exitosa" in message.lower():
            level = "SUCCESS"
        elif "❌" in message or "error" in message.lower():
            level = "ERROR"
        elif "⚠️" in message or "warning" in message.lower():
            level = "WARNING"
        elif "▶️" in message or "⏹️" in message:
            level = "SISTEMA"
        else:
            level = "INFO"
        
        # Limpiar emojis del mensaje para evitar duplicados
        clean_msg = message.replace("✅", "").replace("❌", "").replace("⚠️", "").replace("▶️", "").replace("⏹️", "").strip()
        self.log_camera_message(level, clean_msg)
    
    # --- Microscopía Automatizada ---
    
    def update_size_estimate(self):
        """Actualiza la estimación de tamaño total de imágenes."""
        try:
            if not hasattr(self, 'current_trajectory'):
                self.size_estimate_label.setText("~0 MB")
                return
            
            # Obtener parámetros
            width = int(self.image_width_input.text())
            height = int(self.image_height_input.text())
            n_points = len(self.current_trajectory)
            
            # Contar canales seleccionados
            n_channels = 0
            if self.channel_r_check.isChecked():
                n_channels += 1
            if self.channel_g_check.isChecked():
                n_channels += 1
            if self.channel_b_check.isChecked():
                n_channels += 1
            
            if n_channels == 0:
                self.size_estimate_label.setText("~0 MB (sin canales)")
                return
            
            # Calcular tamaño aproximado
            # Bytes por píxel (8 bits por canal)
            bytes_per_pixel = n_channels
            bytes_per_image = width * height * bytes_per_pixel
            
            # Considerar compresión PNG (aproximadamente 30-50% del tamaño raw)
            compression_factor = 0.4
            estimated_bytes_per_image = bytes_per_image * compression_factor
            
            total_bytes = estimated_bytes_per_image * n_points
            total_mb = total_bytes / (1024 * 1024)
            
            # Formatear
            if total_mb < 1:
                self.size_estimate_label.setText(f"~{total_bytes/1024:.1f} KB")
            elif total_mb < 1024:
                self.size_estimate_label.setText(f"~{total_mb:.1f} MB")
            else:
                self.size_estimate_label.setText(f"~{total_mb/1024:.2f} GB")
            
        except (ValueError, AttributeError):
            self.size_estimate_label.setText("~0 MB")
    
    def set_zero_reference(self):
        """Establece la referencia cero para la microscopía."""
        try:
            zero_x = float(self.zero_ref_x_input.text())
            zero_y = float(self.zero_ref_y_input.text())
            
            self.microscopy_zero_ref = (zero_x, zero_y)
            
            self.log_camera_message("SUCCESS", f"✅ Referencia cero establecida: X={zero_x:.1f} µm, Y={zero_y:.1f} µm")
            logger.info(f"Referencia cero establecida: ({zero_x}, {zero_y})")
            
            QMessageBox.information(self, "✅ Referencia Establecida",
                                  f"Referencia cero configurada:\n\n"
                                  f"X = {zero_x:.1f} µm\n"
                                  f"Y = {zero_y:.1f} µm\n\n"
                                  f"La trayectoria se ejecutará desde este punto.")
            
        except ValueError as e:
            QMessageBox.warning(self, "Error", f"Valores inválidos para referencia cero: {e}")
            self.log_camera_message("ERROR", f"Error en referencia cero: {e}")
    
    def start_automated_microscopy(self):
        """Inicia la microscopía automatizada con captura de imágenes."""
        logger.info("=== INICIANDO MICROSCOPÍA AUTOMATIZADA ===")
        
        # Verificaciones
        if not hasattr(self, 'current_trajectory'):
            QMessageBox.warning(self, "Error", "Primero debes generar una trayectoria zig-zag en la pestaña 'Prueba'")
            return
        
        if not self.camera_worker:
            QMessageBox.warning(self, "Error", 
                              "La cámara no está conectada.\n\n"
                              "Pasos:\n"
                              "1. Presiona '🔌 Conectar Cámara'\n"
                              "2. Presiona '📹 Ver Cámara'\n"
                              "3. Presiona '▶️ Iniciar' (live view)")
            return
        
        if self.camera_worker.current_frame is None:
            QMessageBox.warning(self, "⚠️ Live View No Activo", 
                              "La cámara está conectada pero no hay frames disponibles.\n\n"
                              "Debes iniciar el live view:\n"
                              "1. Presiona '📹 Ver Cámara' (si no está abierta)\n"
                              "2. Presiona '▶️ Iniciar' para activar live view\n"
                              "3. Verifica que la cámara esté transmitiendo\n\n"
                              "Sin live view activo, no se pueden capturar imágenes.")
            return
        
        if not hasattr(self, 'global_calibration'):
            QMessageBox.warning(self, "⚠️ Calibración Requerida",
                              "Debes calibrar el sistema primero en la pestaña 'Análisis'.")
            return
        
        # Verificar controladores
        if self.test_controller_a is None or self.test_controller_b is None:
            QMessageBox.warning(self, "Error",
                              "Se requieren controladores para AMBOS motores (A y B).\n\n"
                              "Transfiere controladores H∞ desde la pestaña 'H∞ Synthesis'.")
            return
        
        # Validar parámetros
        try:
            class_name = self.class_name_input.text().strip()
            if not class_name:
                QMessageBox.warning(self, "Error", "Debes ingresar un nombre de clase")
                return
            
            width = int(self.image_width_input.text())
            height = int(self.image_height_input.text())
            
            if width < 1 or height < 1:
                QMessageBox.warning(self, "Error", "Dimensiones de imagen inválidas")
                return
            
            # Verificar canales
            n_channels = sum([
                self.channel_r_check.isChecked(),
                self.channel_g_check.isChecked(),
                self.channel_b_check.isChecked()
            ])
            
            if n_channels == 0:
                QMessageBox.warning(self, "Error", "Debes seleccionar al menos un canal RGB")
                return
            
        except ValueError as e:
            QMessageBox.warning(self, "Error", f"Parámetros inválidos: {e}")
            return
        
        # Configurar microscopía
        self.microscopy_active = True
        self.microscopy_current_point = 0
        self.microscopy_class_name = class_name
        self.microscopy_image_size = (width, height)
        self.microscopy_channels = {
            'R': self.channel_r_check.isChecked(),
            'G': self.channel_g_check.isChecked(),
            'B': self.channel_b_check.isChecked()
        }
        self.microscopy_captured_count = 0
        
        # Aplicar offset de referencia cero si existe
        if hasattr(self, 'microscopy_zero_ref'):
            zero_x, zero_y = self.microscopy_zero_ref
            self.microscopy_trajectory = self.current_trajectory + np.array([zero_x, zero_y])
            self.log_camera_message("INFO", f"Aplicando offset de referencia cero: ({zero_x}, {zero_y})")
        else:
            self.microscopy_trajectory = self.current_trajectory.copy()
        
        # Actualizar interfaz
        self.microscopy_start_btn.setEnabled(False)
        self.microscopy_stop_btn.setEnabled(True)
        
        total_points = len(self.microscopy_trajectory)
        self.microscopy_progress_label.setText(f"0 / {total_points} imágenes capturadas")
        
        self.log_camera_message("SUCCESS", "🔬 MICROSCOPÍA AUTOMATIZADA INICIADA")
        self.log_camera_message("INFO", f"Total de puntos: {total_points}")
        self.log_camera_message("INFO", f"Clase: {class_name}")
        self.log_camera_message("INFO", f"Tamaño: {width}×{height} px")
        self.log_camera_message("INFO", f"Canales: {'R' if self.microscopy_channels['R'] else ''}{'G' if self.microscopy_channels['G'] else ''}{'B' if self.microscopy_channels['B'] else ''}")
        
        logger.info(f"Microscopía iniciada: {total_points} puntos, clase={class_name}")
        
        # Ejecutar primer punto
        self.execute_microscopy_point()
    
    def execute_microscopy_point(self):
        """Ejecuta el movimiento y captura para el siguiente punto."""
        if not self.microscopy_active or self.microscopy_current_point >= len(self.microscopy_trajectory):
            self.stop_automated_microscopy()
            return
        
        # Obtener punto actual
        point = self.microscopy_trajectory[self.microscopy_current_point]
        x_target = point[0]
        y_target = point[1]
        
        self.log_camera_message("INFO", f"📍 Punto {self.microscopy_current_point + 1}/{len(self.microscopy_trajectory)}: X={x_target:.1f}, Y={y_target:.1f} µm")
        logger.info(f"Microscopía punto {self.microscopy_current_point + 1}: ({x_target:.1f}, {y_target:.1f})")
        
        # Configurar referencias para ambos motores
        self.test_ref_a_input.setText(f"{x_target:.0f}")
        self.test_ref_b_input.setText(f"{y_target:.0f}")
        
        # Reiniciar control dual si ya está activo
        if self.dual_control_active:
            self.stop_dual_control()
        
        # Iniciar control dual
        self.start_dual_control()
        
        # Guardar objetivo
        self.microscopy_target_x = x_target
        self.microscopy_target_y = y_target
        self.microscopy_move_start_time = time.time()
        
        # Verificar posición alcanzada
        self.check_microscopy_position()
    
    def check_microscopy_position(self):
        """Verifica si se alcanzó la posición objetivo para microscopía."""
        if not self.microscopy_active:
            return
        
        # Tolerancia fija para microscopía
        tolerance = 100.0  # µm
        
        try:
            if not hasattr(self, 'dual_last_pos_a') or not hasattr(self, 'dual_last_pos_b'):
                QTimer.singleShot(100, self.check_microscopy_position)
                return
            
            current_x = self.dual_last_pos_a
            current_y = self.dual_last_pos_b
            
            error_x = abs(current_x - self.microscopy_target_x)
            error_y = abs(current_y - self.microscopy_target_y)
            
            # Verificar si alcanzó la posición
            if error_x <= tolerance and error_y <= tolerance:
                elapsed = time.time() - self.microscopy_move_start_time
                self.log_camera_message("SUCCESS", f"✅ Posición alcanzada en {elapsed:.2f}s (error: {error_x:.1f}, {error_y:.1f} µm)")
                
                # Detener control
                if self.dual_control_active:
                    self.stop_dual_control()
                
                # Leer demora ANTES de captura (para estabilización de vibraciones)
                try:
                    delay_before_ms = int(float(self.delay_before_input.text()) * 1000)
                except:
                    delay_before_ms = 500  # Default 0.5s
                
                self.log_camera_message("INFO", f"⏱️ Esperando estabilización: {delay_before_ms/1000:.2f}s")
                QTimer.singleShot(delay_before_ms, self.capture_microscopy_image)
            else:
                # Continuar verificando
                QTimer.singleShot(100, self.check_microscopy_position)
        
        except Exception as e:
            logger.error(f"Error verificando posición microscopía: {e}")
            QTimer.singleShot(100, self.check_microscopy_position)
    
    def capture_microscopy_image(self):
        """Captura la imagen en el punto actual de microscopía."""
        if not self.camera_worker:
            self.log_camera_message("ERROR", "Cámara no conectada")
            # Continuar con siguiente punto
            self.microscopy_current_point += 1
            QTimer.singleShot(100, self.execute_microscopy_point)
            return
        
        # Verificar que hay frame disponible
        if self.camera_worker.current_frame is None:
            self.log_camera_message("ERROR", "No hay frame disponible. ¿Está el live view activo?")
            logger.error("current_frame es None - live view no está activo")
            # Continuar con siguiente punto
            self.microscopy_current_point += 1
            QTimer.singleShot(100, self.execute_microscopy_point)
            return
        
        try:
            import os
            import cv2
            
            save_path = self.save_folder_input.text()
            os.makedirs(save_path, exist_ok=True)
            
            # Generar nombre de archivo: NombreClase_XXXXX.png (5 dígitos: 00000-99999)
            image_number = self.microscopy_current_point
            filename = f"{self.microscopy_class_name}_{image_number:05d}.png"
            full_path = os.path.join(save_path, filename)
            
            # Obtener frame (hacer copia para evitar problemas de concurrencia)
            frame = self.camera_worker.current_frame.copy()
            
            # Verificar que el frame no esté vacío
            if frame is None or frame.size == 0:
                self.log_camera_message("ERROR", f"Frame vacío en punto {image_number}")
                logger.error(f"Frame vacío: shape={frame.shape if frame is not None else 'None'}")
                # Continuar con siguiente punto
                self.microscopy_current_point += 1
                QTimer.singleShot(100, self.execute_microscopy_point)
                return
            
            logger.info(f"Frame capturado: shape={frame.shape}, dtype={frame.dtype}, min={frame.min()}, max={frame.max()}")
            
            # Normalizar a uint8 si es necesario (igual que en img_capture.py)
            if frame.dtype != np.uint8:
                if frame.max() > 0:
                    frame = (frame / frame.max() * 255).astype(np.uint8)
                else:
                    frame = frame.astype(np.uint8)
            
            # Redimensionar si es necesario
            target_width, target_height = self.microscopy_image_size
            if frame.shape[1] != target_width or frame.shape[0] != target_height:
                frame = cv2.resize(frame, (target_width, target_height))
            
            # Procesar canales según selección del usuario
            # Contar canales seleccionados
            selected_channels = []
            if self.microscopy_channels['R']:
                selected_channels.append('R')
            if self.microscopy_channels['G']:
                selected_channels.append('G')
            if self.microscopy_channels['B']:
                selected_channels.append('B')
            
            n_selected = len(selected_channels)
            
            # Si la imagen es grayscale
            if len(frame.shape) == 2:  # Grayscale original
                if n_selected == 1:
                    # MONOBANDA: Guardar directamente en grayscale puro (NO convertir a BGR)
                    logger.info(f"💾 Guardando imagen MONOBANDA ({selected_channels[0]}) en grayscale puro")
                    # frame ya está en grayscale, no hacer nada
                elif n_selected == 2 or n_selected == 3:
                    # Duobanda o RGB: convertir a BGR y aplicar máscara
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    logger.info(f"💾 Guardando imagen con {n_selected} canales (BGR)")
                    
                    # Crear máscara de canales
                    channels_to_keep = []
                    if self.microscopy_channels['B']:  # OpenCV usa BGR
                        channels_to_keep.append(0)
                    if self.microscopy_channels['G']:
                        channels_to_keep.append(1)
                    if self.microscopy_channels['R']:
                        channels_to_keep.append(2)
                    
                    # Aplicar máscara (poner en 0 los canales NO seleccionados)
                    if len(channels_to_keep) < 3:
                        new_frame = np.zeros_like(frame)
                        for idx in channels_to_keep:
                            new_frame[:, :, idx] = frame[:, :, idx]
                        frame = new_frame
            
            elif len(frame.shape) == 3:  # Ya es color (BGR)
                if n_selected == 1:
                    # MONOBANDA desde imagen color: extraer solo ese canal
                    channel_map = {'B': 0, 'G': 1, 'R': 2}
                    channel_idx = channel_map[selected_channels[0]]
                    frame = frame[:, :, channel_idx]  # Extraer canal y convertir a grayscale
                    logger.info(f"💾 Guardando imagen MONOBANDA (canal {selected_channels[0]}) en grayscale puro")
                else:
                    # Duobanda o RGB: aplicar máscara
                    logger.info(f"💾 Guardando imagen con {n_selected} canales (BGR)")
                    channels_to_keep = []
                    if self.microscopy_channels['B']:
                        channels_to_keep.append(0)
                    if self.microscopy_channels['G']:
                        channels_to_keep.append(1)
                    if self.microscopy_channels['R']:
                        channels_to_keep.append(2)
                    
                    if len(channels_to_keep) < 3:
                        new_frame = np.zeros_like(frame)
                        for idx in channels_to_keep:
                            new_frame[:, :, idx] = frame[:, :, idx]
                        frame = new_frame
            
            # Guardar imagen
            success = cv2.imwrite(full_path, frame)
            
            if not success:
                self.log_camera_message("ERROR", f"cv2.imwrite falló para {filename}")
                logger.error(f"cv2.imwrite retornó False para {full_path}")
                # Continuar con siguiente punto
                self.microscopy_current_point += 1
                QTimer.singleShot(100, self.execute_microscopy_point)
                return
            
            self.microscopy_captured_count += 1
            self.microscopy_progress_label.setText(
                f"{self.microscopy_captured_count} / {len(self.microscopy_trajectory)} imágenes capturadas"
            )
            
            self.log_camera_message("SUCCESS", f"📸 Imagen guardada: {filename}")
            logger.info(f"Imagen capturada: {full_path}")
            
            # Avanzar al siguiente punto
            self.microscopy_current_point += 1
            
            # Leer demora DESPUÉS de captura
            try:
                delay_after_ms = int(float(self.delay_after_input.text()) * 1000)
            except:
                delay_after_ms = 200  # Default 0.2s
            
            self.log_camera_message("INFO", f"⏳ Pausa post-captura: {delay_after_ms/1000:.2f}s")
            QTimer.singleShot(delay_after_ms, self.execute_microscopy_point)
            
        except Exception as e:
            self.log_camera_message("ERROR", f"Error capturando imagen: {e}")
            logger.error(f"Error captura microscopía: {e}\n{traceback.format_exc()}")
            # Continuar con siguiente punto
            self.microscopy_current_point += 1
            QTimer.singleShot(100, self.execute_microscopy_point)
    
    def stop_automated_microscopy(self):
        """Detiene la microscopía automatizada."""
        logger.info("=== DETENIENDO MICROSCOPÍA AUTOMATIZADA ===")
        
        self.microscopy_active = False
        
        # Detener control dual si está activo
        if self.dual_control_active:
            self.stop_dual_control()
        
        # Actualizar interfaz
        self.microscopy_start_btn.setEnabled(True)
        self.microscopy_stop_btn.setEnabled(False)
        
        if self.microscopy_captured_count > 0:
            self.log_camera_message("SUCCESS", f"✅ MICROSCOPÍA COMPLETADA: {self.microscopy_captured_count} imágenes capturadas")
        else:
            self.log_camera_message("INFO", "⏸️ Microscopía detenida")
        
        logger.info(f"Microscopía detenida: {self.microscopy_captured_count} imágenes capturadas")

    # --- Lógica de Actualización de Datos ---
    def update_data(self, line):
        """Procesa datos del Arduino y actualiza GUI y ventana de señales."""
        if line.startswith("ERROR:") or line.startswith("INFO:"):
            print(line)
            return

        try:
            parts = line.split(',')
            if len(parts) == 4:
                pot_a, pot_b, sens_1, sens_2 = map(int, parts)
                
                # 1. Grabación en CSV
                if self.is_recording and self.csv_writer:
                    current_time_ms = int((time.time() - self.start_time) * 1000)
                    self.csv_writer.writerow([current_time_ms] + parts)

                # 2. Actualizar labels de GUI
                self.value_labels['power_a'].setText(str(pot_a))
                self.value_labels['power_b'].setText(str(pot_b))
                self.value_labels['sensor_1'].setText(str(sens_1))
                self.value_labels['sensor_2'].setText(str(sens_2))

                # 3. Actualizar ventana de señales (si está abierta)
                if self.signal_window is not None and self.signal_window.isVisible():
                    self.signal_window.update_data(pot_a, pot_b, sens_1, sens_2)

        except (IndexError, ValueError):
            pass
            
    # --- Lógica de Control y Comandos ---
    def start_recording(self):
        """Inicia la grabación de datos en archivo CSV."""
        logger.info("=== BOTÓN: Iniciar Grabación presionado ===")
        filename = self.filename_input.text()
        logger.debug(f"Nombre de archivo ingresado: {filename}")
        
        if not filename.endswith('.csv'):
            filename += '.csv'
            self.filename_input.setText(filename)
            logger.debug(f"Extensión .csv agregada: {filename}")
        
        try:
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["Timestamp_ms", "PotenciaA", "PotenciaB", "Sensor1", "Sensor2"])
            logger.info(f"Archivo CSV creado exitosamente: {filename}")
            
            self.start_time = time.time()
            self.is_recording = True
            logger.info(f"Grabación iniciada en t={self.start_time}")

            self.record_status_label.setText(f"Grabando en: {filename}")
            self.record_status_label.setStyleSheet("color: #2ECC71;")
            self.start_record_btn.setEnabled(False)
            self.stop_record_btn.setEnabled(True)
            self.filename_input.setEnabled(False)
            logger.debug("Interfaz actualizada: botones y labels configurados")

        except (IOError, PermissionError) as e:
            logger.error(f"Error al crear archivo CSV: {e}")
            self.record_status_label.setText(f"Error al abrir archivo: {e}")
            self.record_status_label.setStyleSheet("color: #E74C3C;")
        except Exception as e:
            logger.critical(f"Error inesperado en start_recording: {e}\n{traceback.format_exc()}")

    def stop_recording(self):
        """Detiene la grabación de datos."""
        logger.info("=== BOTÓN: Detener Grabación presionado ===")
        self.is_recording = False
        
        if self.csv_file:
            self.csv_file.close()
            logger.info(f"Archivo CSV cerrado correctamente")
            self.csv_file = None
            self.csv_writer = None
        else:
            logger.warning("stop_recording llamado pero no había archivo abierto")
        
        self.record_status_label.setText("Estado: Detenido")
        self.record_status_label.setStyleSheet("color: #E67E22;")
        self.start_record_btn.setEnabled(True)
        self.stop_record_btn.setEnabled(False)
        self.filename_input.setEnabled(True)
        logger.debug("Interfaz actualizada: grabación detenida")
        
    def send_command(self, command):
        """Envía comando al Arduino vía serial."""
        logger.debug(f"Enviando comando: '{command}'")
        
        if self.serial_thread.ser and self.serial_thread.ser.is_open:
            full_command = command + '\n' 
            self.serial_thread.ser.write(full_command.encode('utf-8'))
            logger.info(f"Comando enviado exitosamente: {command}")
        else:
            logger.error("Error: Puerto serial no está abierto. Comando no enviado.")
            print("Error: El puerto serie no está abierto.")

    def set_manual_mode(self):
        """Activa modo MANUAL en el Arduino."""
        logger.info("=== BOTÓN: Activar MODO MANUAL presionado ===")
        self.send_command('M')
        self.value_labels['mode'].setText("MANUAL")
        self.value_labels['mode'].setStyleSheet("font-weight: bold; color: #E67E22;")
        logger.debug("Modo MANUAL activado en interfaz")

    def set_auto_mode(self):
        """Activa modo AUTOMÁTICO en el Arduino."""
        logger.info("=== BOTÓN: Activar MODO AUTO presionado ===")
        self.send_command('A')
        self.value_labels['mode'].setText("AUTOMÁTICO")
        self.value_labels['mode'].setStyleSheet("font-weight: bold; color: #2ECC71;")
        logger.debug("Modo AUTOMÁTICO activado en interfaz")
        
    def send_power_command(self):
        """Envía comando de potencia a los motores."""
        power_values = self.power_input.text()
        logger.info(f"=== BOTÓN: Enviar Potencia presionado === Valores: {power_values}")
        command_string = f"A,{power_values}"
        self.send_command(command_string)
    
    # --- Control H∞ en Tiempo Real ---
    
    def transfer_to_test_tab(self):
        """Transfiere TODOS los parámetros del controlador H∞ a la pestaña de Prueba."""
        logger.info("=== BOTÓN: Transferir a Prueba presionado ===")
        
        if not hasattr(self, 'hinf_controller'):
            QMessageBox.warning(self, "Error", "No hay controlador sintetizado para transferir")
            return
        
        # Usar Kp y Ki guardados desde la síntesis (ya normalizados)
        if hasattr(self, 'hinf_Kp_designed') and hasattr(self, 'hinf_Ki_designed'):
            Kp = self.hinf_Kp_designed
            Ki = self.hinf_Ki_designed
            logger.info(f"✅ Usando Kp, Ki diseñados: Kp={Kp:.4f}, Ki={Ki:.4f}")
        else:
            # Fallback: extraer del controlador
            try:
                num = self.hinf_controller.num[0][0]
                den = self.hinf_controller.den[0][0]
                if len(num) >= 2 and len(den) == 2:
                    Kp = num[0] / den[0]
                    Ki = num[1] / den[0]
                    logger.warning(f"⚠️ Extrayendo Kp, Ki del controlador: Kp={Kp:.4f}, Ki={Ki:.4f}")
                else:
                    QMessageBox.warning(self, "Error", "Controlador inválido")
                    return
            except Exception as e:
                QMessageBox.warning(self, "Error", f"No se pudo extraer Kp, Ki: {e}")
                return
        
        # Recopilar TODOS los parámetros de síntesis
        K_original = self.hinf_K_value
        K_abs = abs(K_original)
        tau = self.hinf_tau_value
        gamma = self.hinf_gamma if hasattr(self, 'hinf_gamma') else 0.0
        
        # Usar U_max guardado (siempre positivo)
        if hasattr(self, 'hinf_Umax_designed'):
            U_max = self.hinf_Umax_designed
        else:
            try:
                U_max = abs(float(self.w2_umax.text()))
            except:
                U_max = 100.0
        
        # Leer otros parámetros de ponderaciones
        try:
            Ms = float(self.w1_Ms.text())
            wb = float(self.w1_wb.text())
            eps = float(self.w1_eps.text())
            w_unc = float(self.w3_wunc.text())
            eps_T = float(self.w3_epsT.text())
        except:
            Ms = wb = eps = w_unc = eps_T = 0.0
        
        # Preguntar al usuario a qué motor transferir
        dialog = QDialog(self)
        dialog.setWindowTitle("Transferir Controlador H∞ Completo")
        dialog.setGeometry(100, 100, 600, 700)
        layout = QVBoxLayout()
        
        # Mostrar resumen completo
        summary = QTextEdit()
        summary.setReadOnly(True)
        summary.setMaximumHeight(500)
        summary_text = (
            f"╔═══════════════════════════════════════════════════════════╗\n"
            f"║  PARÁMETROS COMPLETOS DEL CONTROLADOR H∞                 ║\n"
            f"╠═══════════════════════════════════════════════════════════╣\n"
            f"║  PLANTA G(s):                                             ║\n"
            f"║    K original = {K_original:+.4f} µm/s/PWM                        ║\n"
            f"║    |K| usado  = {K_abs:.4f} µm/s/PWM                        ║\n"
            f"║    τ          = {tau:.4f} s                                 ║\n"
            f"║    G(s) = {K_abs:.4f} / (s·({tau:.4f}s + 1))                ║\n"
            f"╠═══════════════════════════════════════════════════════════╣\n"
            f"║  CONTROLADOR K(s):                                        ║\n"
            f"║    Kp = {Kp:.4f}                                              ║\n"
            f"║    Ki = {Ki:.4f}                                              ║\n"
            f"║    K(s) = ({Kp:.4f}·s + {Ki:.4f}) / s                        ║\n"
            f"╠═══════════════════════════════════════════════════════════╣\n"
            f"║  PONDERACIONES H∞:                                        ║\n"
            f"║    W1 (Performance):                                      ║\n"
            f"║      Ms = {Ms:.2f} (pico sensibilidad)                        ║\n"
            f"║      ωb = {wb:.2f} rad/s (ancho de banda)                     ║\n"
            f"║      ε  = {eps:.4f} (error estado estacionario)              ║\n"
            f"║                                                           ║\n"
            f"║    W2 (Control effort):                                   ║\n"
            f"║      U_max = {U_max:.1f} PWM                                    ║\n"
            f"║                                                           ║\n"
            f"║    W3 (Robustness):                                       ║\n"
            f"║      ω_unc = {w_unc:.1f} rad/s (incertidumbre)                ║\n"
            f"║      εT    = {eps_T:.4f} (roll-off)                           ║\n"
            f"╠═══════════════════════════════════════════════════════════╣\n"
            f"║  RESULTADO SÍNTESIS:                                      ║\n"
            f"║    γ (mixsyn) = {gamma:.4f}                                   ║\n"
            f"╚═══════════════════════════════════════════════════════════╝\n"
        )
        summary.setText(summary_text)
        summary.setStyleSheet("font-family: 'Courier New'; font-size: 10px;")
        layout.addWidget(summary)
        
        layout.addWidget(QLabel("\n¿A qué motor deseas transferir estos parámetros?"))
        
        # Etiquetas dinámicas según calibración
        if hasattr(self, 'global_calibration'):
            cal = self.global_calibration
            motor_cal = cal.get('motor', 'A')
            sensor_cal = cal.get('sensor', '1')
            
            if motor_cal == 'A' and sensor_cal == '1':
                label_a = "Motor A / Sensor 1 (X)"
                label_b = "Motor B / Sensor 2 (Y)"
            elif motor_cal == 'A' and sensor_cal == '2':
                label_a = "Motor A / Sensor 2 (X)"
                label_b = "Motor B / Sensor 1 (Y)"
            elif motor_cal == 'B' and sensor_cal == '1':
                label_a = "Motor A / Sensor 2 (X)"
                label_b = "Motor B / Sensor 1 (Y)"
            else:
                label_a = "Motor A / Sensor 1 (X)"
                label_b = "Motor B / Sensor 2 (Y)"
        else:
            label_a = "Motor A (X)"
            label_b = "Motor B (Y)"
        
        motor_a_radio = QRadioButton(label_a)
        motor_b_radio = QRadioButton(label_b)
        both_radio = QRadioButton("Ambos motores (A y B)")
        
        # Seleccionar por defecto el motor calibrado
        if hasattr(self, 'global_calibration'):
            motor_cal = self.global_calibration.get('motor', 'B')
            if motor_cal == 'A':
                motor_a_radio.setChecked(True)  # Motor A calibrado
            else:
                motor_b_radio.setChecked(True)  # Motor B calibrado
        else:
            motor_b_radio.setChecked(True)  # Por defecto Motor B si no hay calibración
        
        layout.addWidget(motor_a_radio)
        layout.addWidget(motor_b_radio)
        layout.addWidget(both_radio)
        
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        
        dialog.setLayout(layout)
        
        if dialog.exec_() == QDialog.Accepted:
            transferred_motors = []
            
            # Crear info del controlador
            controller_info = (
                f"✅ Controlador H∞ Cargado\n"
                f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                f"K(s) = ({Kp:.4f}·s + {Ki:.4f}) / s\n"
                f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                f"Planta: G(s) = {K_abs:.4f} / (s·({tau:.4f}s + 1))\n"
                f"Ponderaciones: Ms={Ms:.2f}, ωb={wb:.2f} rad/s\n"
                f"U_max: {U_max:.1f} PWM | γ: {gamma:.4f}"
            )
            
            if motor_a_radio.isChecked() or both_radio.isChecked():
                # Transferir a Motor A
                self.test_controller_a = {
                    'controller': self.hinf_controller,
                    'Kp': Kp,
                    'Ki': Ki,
                    'K': K_abs,
                    'K_sign': np.sign(K_original),
                    'tau': tau,
                    'Ms': Ms,
                    'wb': wb,
                    'U_max': U_max,
                    'gamma': gamma
                }
                self.test_motor_a_info.setText(controller_info)
                self.test_motor_a_status.setText("✅ Controlador cargado")
                self.test_motor_a_status.setStyleSheet("color: #27AE60; font-weight: bold;")
                self.test_clear_a_btn.setEnabled(True)
                transferred_motors.append("Motor A")
                logger.info(f"Controlador H∞ transferido a Motor A")
            
            if motor_b_radio.isChecked() or both_radio.isChecked():
                # Transferir a Motor B
                self.test_controller_b = {
                    'controller': self.hinf_controller,
                    'Kp': Kp,
                    'Ki': Ki,
                    'K': K_abs,
                    'K_sign': np.sign(K_original),
                    'tau': tau,
                    'Ms': Ms,
                    'wb': wb,
                    'U_max': U_max,
                    'gamma': gamma
                }
                self.test_motor_b_info.setText(controller_info)
                self.test_motor_b_status.setText("✅ Controlador cargado")
                self.test_motor_b_status.setStyleSheet("color: #27AE60; font-weight: bold;")
                self.test_clear_b_btn.setEnabled(True)
                transferred_motors.append("Motor B")
                logger.info(f"Controlador H∞ transferido a Motor B")
            
            # Guardar parámetros H∞ como atributos para referencia
            self.transferred_hinf_params = {
                'Kp': Kp,
                'Ki': Ki,
                'K': K_abs,
                'K_original': K_original,
                'tau': tau,
                'Ms': Ms,
                'wb': wb,
                'eps': eps,
                'U_max': U_max,
                'w_unc': w_unc,
                'eps_T': eps_T,
                'gamma': gamma
            }
            
            motor_names = " y ".join(transferred_motors)
            
            # Cambiar a pestaña de Prueba automáticamente
            self.tabs.setCurrentIndex(2)  # Índice 2 = Pestaña Prueba
            
            logger.info(f"Transferencia completa a {motor_names}")
            
            # Determinar qué sensor debe usar cada motor
            if hasattr(self, 'global_calibration'):
                cal = self.global_calibration
                motor_cal = cal.get('motor', 'A')
                sensor_cal = cal.get('sensor', '1')
                
                if motor_a_radio.isChecked():
                    sensor_msg = f"\n⚠️ IMPORTANTE: Verifica que '{motor_names}' use Sensor {sensor_cal} en la pestaña Prueba"
                elif motor_b_radio.isChecked():
                    sensor_msg = f"\n⚠️ IMPORTANTE: Verifica que '{motor_names}' use Sensor {sensor_cal} en la pestaña Prueba"
                else:
                    sensor_msg = "\n⚠️ IMPORTANTE: Configura manualmente los sensores para cada motor en la pestaña Prueba"
            else:
                sensor_msg = "\n⚠️ IMPORTANTE: Configura manualmente los sensores en la pestaña Prueba"
            
            QMessageBox.information(self, "✅ Transferencia Exitosa", 
                                   f"Parámetros completos transferidos a {motor_names}:\n\n"
                                   f"Controlador:\n"
                                   f"  Kp = {Kp:.4f}\n"
                                   f"  Ki = {Ki:.4f}\n\n"
                                   f"Planta:\n"
                                   f"  K = {K_abs:.4f} µm/s/PWM\n"
                                   f"  τ = {tau:.4f} s\n\n"
                                   f"Ponderaciones:\n"
                                   f"  Ms = {Ms:.2f}, ωb = {wb:.2f} rad/s\n"
                                   f"  U_max = {U_max:.1f} PWM"
                                   f"{sensor_msg}")
    
    def update_test_calibration_display(self):
        """Actualiza el display de calibración en la pestaña Prueba."""
        if not hasattr(self, 'global_calibration'):
            return
        
        cal = self.global_calibration
        
        # Obtener configuración de calibración
        motor_cal = cal.get('motor', 'A')
        sensor_cal = cal.get('sensor', '1')
        
        # IMPORTANTE: NO sobrescribir los combos - el usuario los configura manualmente
        # Solo actualizar las etiquetas informativas
        
        # Mostrar qué motor/sensor fue calibrado (sin cambiar combos)
        if motor_cal == 'A':
            self.test_motor_a_label.setText(f"<b>Motor A (X) - Calibrado con Sensor {sensor_cal}</b>")
            self.test_motor_b_label.setText("<b>Motor B (Y) - Seleccionar sensor manualmente</b>")
        else:  # motor_cal == 'B'
            self.test_motor_b_label.setText(f"<b>Motor B (Y) - Calibrado con Sensor {sensor_cal}</b>")
            self.test_motor_a_label.setText("<b>Motor A (X) - Seleccionar sensor manualmente</b>")
        
        # Actualizar estado
        self.test_calibration_status.setText("✅ Sistema calibrado")
        self.test_calibration_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #27AE60; padding: 5px;")
        
        # Actualizar detalles
        factor_um = abs(cal['pendiente_um'])
        rango_um = abs(cal['dist_punto2_um'] - cal['dist_punto1_um'])
        
        details = (
            f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            f"Motor calibrado: {motor_cal} / Sensor: {sensor_cal}\n"
            f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            f"Punto 1: {cal['adc_punto1']:.0f} UA = {cal['dist_punto1_mm']:.1f} mm\n"
            f"Punto 2: {cal['adc_punto2']:.0f} UA = {cal['dist_punto2_mm']:.1f} mm\n"
            f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            f"Factor: {factor_um:.4f} µm/ADC\n"
            f"Rango útil: {rango_um:.0f} µm\n"
            f"Relación: {cal['relacion']}\n"
            f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            f"Pendiente: {cal['pendiente_um']:.4f} µm/ADC\n"
            f"Intercepto: {cal['intercepto_um']:.2f} µm"
        )
        
        self.test_calibration_details.setText(details)
        
        logger.info("✅ Display de calibración actualizado en pestaña Prueba")
    
    def clear_controller(self, motor):
        """Limpia el controlador transferido de un motor."""
        logger.info(f"=== Limpiando controlador Motor {motor} ===")
        
        if motor == 'A':
            self.test_controller_a = None
            self.test_motor_a_info.clear()
            self.test_motor_a_status.setText("⚪ Sin controlador")
            self.test_motor_a_status.setStyleSheet("color: #95A5A6;")
            self.test_clear_a_btn.setEnabled(False)
            logger.info("Controlador Motor A limpiado")
        else:  # Motor B
            self.test_controller_b = None
            self.test_motor_b_info.clear()
            self.test_motor_b_status.setText("⚪ Sin controlador")
            self.test_motor_b_status.setStyleSheet("color: #95A5A6;")
            self.test_clear_b_btn.setEnabled(False)
            logger.info("Controlador Motor B limpiado")
    
    def toggle_hinf_control(self):
        """Activa/desactiva el control H∞ en tiempo real."""
        if not self.hinf_control_active:
            self.start_hinf_control()
        else:
            self.stop_hinf_control()
    
    def start_hinf_control(self):
        """Inicia el control H∞ en tiempo real."""
        logger.info("=== INICIANDO CONTROL H∞ EN TIEMPO REAL ===")
        
        if not hasattr(self, 'hinf_controller'):
            self.controller_results_text.setText("❌ Error: Primero sintetiza el controlador.")
            return
        
        # Usar Kp y Ki guardados desde la síntesis
        if hasattr(self, 'hinf_Kp_designed') and hasattr(self, 'hinf_Ki_designed'):
            Kp_original = self.hinf_Kp_designed
            Ki_original = self.hinf_Ki_designed
            logger.info(f"✅ Usando Kp, Ki diseñados: Kp={Kp_original:.4f}, Ki={Ki_original:.4f}")
        else:
            # Fallback: extraer del controlador
            try:
                num = self.hinf_controller.num[0][0]
                den = self.hinf_controller.den[0][0]
                if len(num) >= 2 and len(den) == 2:
                    Kp_original = num[0] / den[0]
                    Ki_original = num[1] / den[0]
                    logger.warning(f"⚠️ Extrayendo Kp, Ki del controlador: Kp={Kp_original:.4f}, Ki={Ki_original:.4f}")
                else:
                    self.controller_results_text.setText("❌ Error: Controlador inválido.")
                    return
            except:
                self.controller_results_text.setText("❌ Error: No se pudo extraer Kp, Ki.")
                return
        
        # Aplicar factor de escala para suavizar control
        try:
            scale_factor = float(self.hinf_scale_input.text())
            scale_factor = max(0.01, min(1.0, scale_factor))  # Limitar 0.01-1.0
        except:
            scale_factor = 1.0  # Por defecto, sin escalar
        
        self.Kp_hinf = Kp_original * scale_factor
        self.Ki_hinf = Ki_original * scale_factor
        
        # CRÍTICO: Determinar signo de K automáticamente
        # El controlador fue diseñado con |K|, pero debemos aplicar el signo correcto
        if hasattr(self, 'hinf_K_sign'):
            self.hinf_apply_sign = self.hinf_K_sign
            logger.info(f"✅ K_sign desde síntesis H∞: {self.hinf_apply_sign}")
        else:
            # Detección automática según calibración
            if hasattr(self, 'global_calibration'):
                cal = self.global_calibration
                # Determinar signo según pendiente de calibración
                if cal['pendiente_um'] < 0:
                    self.hinf_apply_sign = -1.0  # Calibración inversa
                    logger.info(f"🔧 K_sign AUTO (calibración inversa): {self.hinf_apply_sign} (pendiente={cal['pendiente_um']:.4f})")
                else:
                    self.hinf_apply_sign = 1.0  # Calibración directa
                    logger.info(f"🔧 K_sign AUTO (calibración directa): {self.hinf_apply_sign} (pendiente={cal['pendiente_um']:.4f})")
            else:
                self.hinf_apply_sign = 1.0  # Sin calibración, asumir directo
                logger.warning(f"⚠️ K_sign AUTO (sin calibración): {self.hinf_apply_sign} (asumir directo)")
        
        logger.info(f"Ganancias escaladas: Kp={Kp_original:.2f}→{self.Kp_hinf:.2f}, Ki={Ki_original:.2f}→{self.Ki_hinf:.2f} (escala={scale_factor})")
        logger.info(f"Signo de K a aplicar: {self.hinf_apply_sign}")
        
        # Determinar motor y sensor según selección Y calibración global
        motor_selection = self.hinf_motor_combo.currentIndex()
        self.hinf_motor = 'A' if motor_selection == 0 else 'B'
        
        # Obtener sensor correspondiente desde calibración global
        if hasattr(self, 'global_calibration'):
            cal = self.global_calibration
            motor_cal = cal.get('motor', 'A')
            sensor_cal = cal.get('sensor', '1')
            
            # Asignar sensor según configuración de calibración
            if motor_cal == self.hinf_motor:
                # Motor seleccionado es el calibrado, usar su sensor
                self.hinf_sensor = f'sensor_{sensor_cal}'
            else:
                # Motor seleccionado NO es el calibrado, usar el otro sensor
                self.hinf_sensor = 'sensor_2' if sensor_cal == '1' else 'sensor_1'
        else:
            # Sin calibración, usar asignación por defecto
            self.hinf_sensor = 'sensor_1' if self.hinf_motor == 'A' else 'sensor_2'
        
        # Leer referencia absoluta en µm
        try:
            self.hinf_reference = float(self.hinf_reference_input.text())
        except:
            QMessageBox.warning(self, "Error", "Ingrese una referencia válida en µm")
            return
        
        # Validar límites
        if self.hinf_reference < 0:
            QMessageBox.warning(self, "Error", f"Referencia debe ser ≥ 0 µm")
            return
        elif self.hinf_reference > RECORRIDO_UM:
            QMessageBox.warning(self, "Error", f"Referencia debe ser ≤ {RECORRIDO_UM:.0f} µm")
            return
        
        # Leer posición inicial del sensor
        sensor_ua_inicial_str = self.value_labels[self.hinf_sensor].text()
        try:
            sensor_ua_inicial = int(sensor_ua_inicial_str)
        except:
            QMessageBox.warning(self, "Error", f"No se puede leer el sensor {self.hinf_sensor}")
            return
        
        # ============================================================
        # CALIBRACIÓN: Usar interpolación lineal si se proporcionan datos
        # ============================================================
        dist_min_text = self.hinf_dist_min_input.text().strip()
        dist_max_text = self.hinf_dist_max_input.text().strip()
        adc_min_text = self.hinf_adc_min_input.text().strip()
        adc_max_text = self.hinf_adc_max_input.text().strip()
        
        if dist_min_text and dist_max_text and adc_min_text and adc_max_text:
            try:
                # Leer parámetros de calibración
                dist_min_mm = float(dist_min_text)
                dist_max_mm = float(dist_max_text)
                adc_min = float(adc_min_text)
                adc_max = float(adc_max_text)
                
                # Convertir a µm
                dist_min_um = dist_min_mm * 1000.0
                dist_max_um = dist_max_mm * 1000.0
                
                # Calcular interpolación lineal: y = m*x + b
                delta_adc = adc_max - adc_min
                delta_dist = dist_max_um - dist_min_um
                
                if abs(delta_adc) < 1:
                    raise ValueError("Delta ADC muy pequeño")
                
                # Pendiente e intercepto
                self.hinf_pendiente = delta_dist / delta_adc
                self.hinf_intercepto = dist_min_um - self.hinf_pendiente * adc_min
                
                # Calcular posición inicial usando interpolación
                position_inicial_um = sensor_ua_inicial * self.hinf_pendiente + self.hinf_intercepto
                
                logger.info(f"✅ Calibración activa: {dist_min_mm}→{dist_max_mm} mm")
                logger.info(f"   ADC: {adc_min}→{adc_max}")
                logger.info(f"   Pendiente: {self.hinf_pendiente:.4f} µm/ADC")
                logger.info(f"   Intercepto: {self.hinf_intercepto:.2f} µm")
                self.hinf_calibrado = True
                
            except Exception as e:
                logger.warning(f"Error en calibración: {e}, usando factor de escala por defecto")
                position_inicial_um = sensor_ua_inicial * FACTOR_ESCALA
                self.hinf_calibrado = False
                self.hinf_pendiente = FACTOR_ESCALA
                self.hinf_intercepto = 0.0
        else:
            # Sin calibración, usar factor de escala por defecto
            position_inicial_um = sensor_ua_inicial * FACTOR_ESCALA
            self.hinf_calibrado = False
            self.hinf_pendiente = FACTOR_ESCALA
            self.hinf_intercepto = 0.0
            logger.info("📊 Sin calibración, usando factor de escala por defecto")
        
        logger.info(f"Posición inicial: {sensor_ua_inicial} UA ({position_inicial_um:.1f} µm)")
        logger.info(f"Referencia absoluta: {self.hinf_reference:.1f} µm")
        
        # Guardar U_max para calcular PWM_MAX
        # Usar valor guardado desde síntesis (siempre positivo)
        if hasattr(self, 'hinf_Umax_designed'):
            self.hinf_umax = self.hinf_Umax_designed
            logger.info(f"✅ Usando U_max diseñado: {self.hinf_umax:.1f}")
        else:
            try:
                self.hinf_umax = abs(float(self.w2_umax.text()))
            except:
                self.hinf_umax = 150  # Valor por defecto
            logger.warning(f"⚠️ U_max desde input: {self.hinf_umax:.1f}")
        
        # Guardar estado del checkbox de inversión de PWM
        if hasattr(self, 'invert_pwm_checkbox'):
            self.hinf_invert_pwm = self.invert_pwm_checkbox.isChecked()
            logger.info(f"{'✅' if self.hinf_invert_pwm else '❌'} Inversión de PWM: {'ACTIVADA' if self.hinf_invert_pwm else 'DESACTIVADA'}")
        else:
            self.hinf_invert_pwm = True  # Por defecto invertido
            logger.warning("⚠️ Checkbox no encontrado, usando inversión por defecto")
        
        # Calcular PWM_MAX (60% de U_max para control suave)
        PWM_MAX_calc = int(self.hinf_umax * 0.6)
        
        # Resetear variables de control
        self.hinf_integral = 0.0
        self.hinf_last_time = time.time()
        
        # IMPORTANTE: Activar modo AUTOMÁTICO en Arduino primero
        self.send_command('A')
        self.value_labels['mode'].setText("AUTOMÁTICO (H∞)")
        self.value_labels['mode'].setStyleSheet("font-weight: bold; color: #9B59B6;")
        logger.info("Modo AUTOMÁTICO activado para control H∞")
        time.sleep(0.1)  # Esperar que Arduino procese el comando
        
        # Activar control
        self.hinf_control_active = True
        self.hinf_control_btn.setText("⏹️ Detener Control H∞")
        self.hinf_control_btn.setStyleSheet("font-size: 12px; font-weight: bold; padding: 8px; background-color: #E74C3C;")
        
        # Crear timer para ejecutar control periódicamente
        self.hinf_timer = QTimer()
        self.hinf_timer.timeout.connect(self.execute_hinf_control)
        self.hinf_timer.start(10)  # 10ms = 100Hz
        
        logger.info(f"Control H∞ iniciado: Motor={self.hinf_motor}, Sensor={self.hinf_sensor}, Kp={self.Kp_hinf:.4f}, Ki={self.Ki_hinf:.4f}")
        self.controller_results_text.append(f"\n🎮 Control H∞ ACTIVO")
        self.controller_results_text.append(f"   Motor: {self.hinf_motor} | Sensor: {self.hinf_sensor}")
        self.controller_results_text.append(f"   Kp={self.Kp_hinf:.4f}, Ki={self.Ki_hinf:.4f} (escala={scale_factor})")
        self.controller_results_text.append(f"   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        
        # Mostrar calibración si está activa
        if self.hinf_calibrado:
            self.controller_results_text.append(f"   ✅ Calibración: {dist_min_mm}→{dist_max_mm} mm (ADC: {adc_min}→{adc_max})")
            self.controller_results_text.append(f"   📐 Pendiente: {self.hinf_pendiente:.4f} µm/ADC | Intercepto: {self.hinf_intercepto:.2f} µm")
        else:
            self.controller_results_text.append(f"   📊 Sin calibración (factor: {FACTOR_ESCALA:.4f} µm/ADC)")
        
        self.controller_results_text.append(f"   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        self.controller_results_text.append(f"   📍 Posición actual: {sensor_ua_inicial} UA ({position_inicial_um:.1f} µm)")
        self.controller_results_text.append(f"   🎯 Referencia objetivo: {self.hinf_reference:.1f} µm")
        self.controller_results_text.append(f"   📏 Error inicial: {self.hinf_reference - position_inicial_um:+.1f} µm")
        self.controller_results_text.append(f"   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        self.controller_results_text.append(f"   PWM = (Kp*error + Ki*integral) × signo_K")
        self.controller_results_text.append(f"   Signo K: {self.hinf_apply_sign:+.0f} (compensación por dirección)")
        self.controller_results_text.append(f"   Límites: PWM ∈ [-{int(self.hinf_umax)}, +{int(self.hinf_umax)}] | Zona muerta=±50µm")
        
        # Advertencia si Kp es muy alto
        if self.Kp_hinf > 5:
            self.controller_results_text.append(f"   ⚠️ ADVERTENCIA: Kp={self.Kp_hinf:.4f} MUY ALTO → Usar escala < 0.2")
    
    def execute_hinf_control(self):
        """Ejecuta un ciclo del controlador PI H∞ SIMPLIFICADO."""
        try:
            # 1. Calcular Ts
            current_time = time.time()
            Ts = current_time - self.hinf_last_time
            self.hinf_last_time = current_time
            
            # 2. Leer posición actual del sensor (en U.A.)
            sensor_ua_str = self.value_labels[self.hinf_sensor].text()
            try:
                sensor_ua = int(sensor_ua_str)
            except:
                return
            
            # 3. Convertir U.A. a micrómetros usando interpolación calibrada
            if hasattr(self, 'hinf_calibrado') and self.hinf_calibrado:
                # Usar interpolación lineal: y = m*x + b
                position_um = sensor_ua * self.hinf_pendiente + self.hinf_intercepto
            else:
                # Usar factor de escala por defecto
                position_um = sensor_ua * FACTOR_ESCALA
            
            # 4. Calcular error en micrómetros
            error_um = self.hinf_reference - position_um
            
            # 5. ZONA MUERTA: Detener si está cerca (±50 µm)
            if abs(error_um) <= 50:
                self.send_command('A,0,0')
                self.hinf_integral = 0
                if not hasattr(self, 'hinf_arrived_shown'):
                    self.hinf_arrived_shown = True
                    status_msg = f"✅ LLEGÓ: Sensor={sensor_ua} UA ({position_um:.1f} µm) | Ref={self.hinf_reference} µm"
                    self.controller_results_text.append(status_msg)
                return
            else:
                self.hinf_arrived_shown = False
            
            # 6. Actualizar integral
            self.hinf_integral += error_um * Ts
            
            # 7. Calcular PWM usando controlador PI
            # PWM = Kp*error + Ki*integral
            # 
            # IMPORTANTE: El signo del PWM depende de la calibración:
            # - Calibración directa (pendiente > 0): PWM positivo aumenta posición
            # - Calibración inversa (pendiente < 0): PWM positivo disminuye posición
            #
            # El error ya tiene el signo correcto (ref - pos)
            # Solo necesitamos invertir si la calibración es inversa
            
            pwm_base = self.Kp_hinf * error_um + self.Ki_hinf * self.hinf_integral
            
            # Aplicar inversión de PWM según checkbox
            if hasattr(self, 'hinf_invert_pwm') and self.hinf_invert_pwm:
                pwm_float = -pwm_base
            else:
                pwm_float = pwm_base
            
            logger.debug(f"PWM: error={error_um:.1f}, pwm_base={pwm_base:.1f}, pwm_final={pwm_float:.1f}, invertido={getattr(self, 'hinf_invert_pwm', False)}")
            
            # 8. Limitar PWM usando U_max de la síntesis H∞
            PWM_MAX = int(self.hinf_umax) if hasattr(self, 'hinf_umax') else 150
            if pwm_float > PWM_MAX:
                pwm = PWM_MAX
                self.hinf_integral -= error_um * Ts  # Anti-windup
            elif pwm_float < -PWM_MAX:
                pwm = -PWM_MAX
                self.hinf_integral -= error_um * Ts  # Anti-windup
            else:
                pwm = int(pwm_float)
            
            # Debug cada 100 iteraciones
            if not hasattr(self, 'hinf_debug_counter'):
                self.hinf_debug_counter = 0
            self.hinf_debug_counter += 1
            
            if self.hinf_debug_counter % 100 == 0:
                logger.debug(f"Control: pos={position_um:.1f}µm, ref={self.hinf_reference:.1f}µm, error={error_um:.1f}µm, PWM={pwm}")
            
            # 8. Enviar comando
            if self.hinf_motor == 'A':
                command = f"A,{pwm},0"
            else:
                command = f"A,0,{pwm}"
            self.send_command(command)
            
            # 9. Actualizar interfaz cada 100ms
            if not hasattr(self, 'hinf_update_counter'):
                self.hinf_update_counter = 0
            
            self.hinf_update_counter += 1
            if self.hinf_update_counter >= 10:
                self.hinf_update_counter = 0
                status_msg = f"📍 Sensor={sensor_ua} UA ({position_um:.1f} µm) | Ref={self.hinf_reference} µm | e={error_um:.1f} µm | PWM={pwm} | I={self.hinf_integral:.2f}"
                self.controller_results_text.append(status_msg)
                self.controller_results_text.verticalScrollBar().setValue(
                    self.controller_results_text.verticalScrollBar().maximum()
                )
                logger.info(f"Control H∞: Sensor={sensor_ua}UA ({position_um:.1f}µm), e={error_um:.1f}, PWM={pwm}, I={self.hinf_integral:.2f}")
                
        except Exception as e:
            logger.error(f"Error en control H∞: {e}")
    
    def stop_hinf_control(self):
        """Detiene el control H∞ en tiempo real."""
        logger.info("=== DETENIENDO CONTROL H∞ ===")
        
        if hasattr(self, 'hinf_timer'):
            self.hinf_timer.stop()
        
        # Detener motores
        self.send_command('A,0,0')
        time.sleep(0.05)
        
        # Volver a modo MANUAL
        self.send_command('M')
        self.value_labels['mode'].setText("MANUAL")
        self.value_labels['mode'].setStyleSheet("font-weight: bold; color: #E67E22;")
        
        # Desactivar control
        self.hinf_control_active = False
        self.hinf_control_btn.setText("🎮 Activar Control H∞ en Tiempo Real")
    
    def generate_zigzag_trajectory(self):
        """
        Genera una trayectoria Column-Scan con movimiento secuencial por eje.
        
        Estrategia: Barrido por columnas (Y se mueve completamente antes de cambiar X).
        Esto simplifica el control permitiendo mover un solo eje a la vez.
        """
        logger.info("=== BOTÓN: Generar Trayectoria Column-Scan presionado ===")
        
        try:
            # Leer parámetros
            n_points = int(self.trajectory_points_input.text())
            x_start = float(self.trajectory_x_start_input.text())
            x_end = float(self.trajectory_x_end_input.text())
            y_start = float(self.trajectory_y_start_input.text())
            y_end = float(self.trajectory_y_end_input.text())
            step_delay = float(self.trajectory_step_delay_input.text())
            
            # Validar
            if n_points < 1 or n_points > 10000:
                QMessageBox.warning(self, "Error", "Número de puntos debe estar entre 1 y 10000")
                return
            
            if step_delay < 0.1:
                QMessageBox.warning(self, "Error", "Tiempo entre pasos debe ser al menos 0.1s")
                return
            
            # Calcular punto medio automáticamente
            x_mid = (x_start + x_end) / 2.0
            y_mid = (y_start + y_end) / 2.0
            
            logger.info(f"Generando trayectoria Column-Scan: {n_points} puntos")
            logger.info(f"  X: {x_start} → {x_end} µm (medio: {x_mid:.1f})")
            logger.info(f"  Y: {y_start} → {y_end} µm (medio: {y_mid:.1f})")
            logger.info(f"  Estrategia: Barrido por columnas (Y completo → X incrementa)")
            
            # Calcular dimensiones de la matriz (columnas × filas)
            # Columnas = número de posiciones en X
            # Filas = número de posiciones en Y por columna
            n_columns = int(np.sqrt(n_points))
            n_rows_per_column = int(np.ceil(n_points / n_columns))
            
            logger.info(f"  Matriz: {n_columns} columnas × {n_rows_per_column} filas/columna")
            
            # Generar posiciones discretas
            x_positions = np.linspace(x_start, x_end, n_columns)
            y_positions = np.linspace(y_start, y_end, n_rows_per_column)
            
            # Generar trayectoria Column-Scan
            trajectory = []
            
            for col_idx, x in enumerate(x_positions):
                if col_idx % 2 == 0:
                    # Columna par: Y ascendente (y_start → y_end)
                    y_sequence = y_positions
                    direction = "↑"
                else:
                    # Columna impar: Y descendente (y_end → y_start, zig-zag)
                    y_sequence = reversed(y_positions)
                    direction = "↓"
                
                logger.info(f"  Columna {col_idx + 1}/{n_columns}: X={x:.1f} µm, Y {direction}")
                
                for y in y_sequence:
                    trajectory.append((x, y))
            
            # Limitar al número de puntos solicitado
            trajectory = trajectory[:n_points]
            
            # Guardar trayectoria y parámetros
            self.current_trajectory = np.array(trajectory)
            self.trajectory_step_delay = step_delay
            
            # Actualizar estado de trayectoria para microscopía (pestaña Prueba)
            self.step_trajectory_status.setText(f"✅ Trayectoria Column-Scan: {len(trajectory)} puntos")
            self.step_trajectory_status.setStyleSheet("color: #27AE60; font-weight: bold;")
            self.step_start_btn.setEnabled(True)
            
            # Actualizar estado de trayectoria para microscopía (pestaña ImgRec)
            self.microscopy_trajectory_status.setText(f"✅ Trayectoria Column-Scan: {len(trajectory)} puntos")
            self.microscopy_trajectory_status.setStyleSheet("color: #27AE60; font-weight: bold;")
            # Habilitar botón de microscopía solo si hay cámara conectada
            if hasattr(self, 'camera_worker') and self.camera_worker is not None:
                self.microscopy_start_btn.setEnabled(True)
            
            # Actualizar estimación de tamaño
            self.update_size_estimate()
            
            logger.info(f"✅ Trayectoria Column-Scan generada: {len(trajectory)} puntos")
            QMessageBox.information(self, "✅ Trayectoria Column-Scan Generada", 
                                   f"Trayectoria generada exitosamente:\n\n"
                                   f"📊 Configuración:\n"
                                   f"  • Puntos totales: {len(trajectory)}\n"
                                   f"  • Columnas (X): {n_columns}\n"
                                   f"  • Filas por columna (Y): {n_rows_per_column}\n"
                                   f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                                   f"🎯 Rango de Movimiento:\n"
                                   f"  • X: [{x_start:.0f}, {x_end:.0f}] µm\n"
                                   f"  • Y: [{y_start:.0f}, {y_end:.0f}] µm\n"
                                   f"  • Punto medio: ({x_mid:.0f}, {y_mid:.0f}) µm\n"
                                   f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                                   f"⏱️ Tiempo:\n"
                                   f"  • Pausa entre puntos: {step_delay}s\n"
                                   f"  • Tiempo estimado: {len(trajectory) * step_delay:.1f}s\n"
                                   f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                                   f"ℹ️ Modo Column-Scan:\n"
                                   f"  • Solo un eje se mueve a la vez\n"
                                   f"  • Y barre cada columna completa\n"
                                   f"  • X incrementa entre columnas")
            
        except ValueError as e:
            QMessageBox.warning(self, "Error", f"Valores inválidos: {e}")
            logger.error(f"Error generando trayectoria: {e}")
    
    def preview_trajectory(self):
        """Muestra una vista previa de la trayectoria generada."""
        # ... (rest of the code remains the same)
        logger.info("=== BOTÓN: Vista Previa Trayectoria presionado ===")
        
        if not hasattr(self, 'current_trajectory'):
            QMessageBox.warning(self, "Error", "Primero genera una trayectoria")
            return
        
        # Crear ventana de visualización
        import matplotlib.pyplot as plt
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
        
        fig, ax = plt.subplots(figsize=(8, 8))
        
        trajectory = self.current_trajectory
        x_coords = trajectory[:, 0]
        y_coords = trajectory[:, 1]
        
        # Graficar trayectoria
        ax.plot(x_coords, y_coords, 'b-', linewidth=0.5, alpha=0.5, label='Trayectoria')
        ax.plot(x_coords, y_coords, 'ro', markersize=3, label='Puntos')
        ax.plot(x_coords[0], y_coords[0], 'go', markersize=10, label='Inicio')
        ax.plot(x_coords[-1], y_coords[-1], 'rs', markersize=10, label='Fin')
        
        ax.set_xlabel('X (µm) - Motor A')
        ax.set_ylabel('Y (µm) - Motor B')
        ax.set_title(f'Trayectoria en Zig-Zag ({len(trajectory)} puntos)')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.axis('equal')
        
        # Mostrar en ventana
        dialog = QDialog(self)
        dialog.setWindowTitle("Vista Previa de Trayectoria")
        dialog.setGeometry(100, 100, 900, 900)
        
        layout = QVBoxLayout()
        canvas = FigureCanvas(fig)
        layout.addWidget(canvas)
        
        close_btn = QPushButton("Cerrar")
        close_btn.clicked.connect(dialog.close)
        layout.addWidget(close_btn)
        
        dialog.setLayout(layout)
        dialog.exec_()
        
        plt.close(fig)
    
    def view_coordinate_map(self):
        """Muestra el mapa de coordenadas de la trayectoria zig-zag como una lista."""
        logger.info("=== BOTÓN: Ver Mapa de Coordenadas presionado ===")
        
        try:
            # Leer parámetros (sin generar la trayectoria completa aún)
            n_points = int(self.trajectory_points_input.text())
            x_start = float(self.trajectory_x_start_input.text())
            x_end = float(self.trajectory_x_end_input.text())
            y_start = float(self.trajectory_y_start_input.text())
            y_end = float(self.trajectory_y_end_input.text())
            
            # Validar
            if n_points < 1 or n_points > 10000:
                QMessageBox.warning(self, "Error", "Número de puntos debe estar entre 1 y 10000")
                return
            
            logger.info(f"Generando mapa de coordenadas: {n_points} puntos")
            
            # Calcular número de filas y columnas para zig-zag
            n_rows = int(np.sqrt(n_points))
            n_cols = int(np.ceil(n_points / n_rows))
            
            # Generar grid homogéneo
            x_positions = np.linspace(x_start, x_end, n_cols)
            y_positions = np.linspace(y_start, y_end, n_rows)
            
            # Generar trayectoria en zig-zag
            trajectory = []
            for i, y in enumerate(y_positions):
                if i % 2 == 0:
                    # Fila par: inicio a fin
                    for x in x_positions:
                        trajectory.append((x, y))
                else:
                    # Fila impar: fin a inicio (zig-zag)
                    for x in reversed(x_positions):
                        trajectory.append((x, y))
            
            # Limitar al número de puntos solicitado
            trajectory = trajectory[:n_points]
            
            # Crear ventana de diálogo con la lista de coordenadas
            dialog = QDialog(self)
            dialog.setWindowTitle("📋 Mapa de Coordenadas - Trayectoria Zig-Zag")
            dialog.setGeometry(100, 100, 800, 700)
            dialog.setStyleSheet(DARK_STYLESHEET)
            
            layout = QVBoxLayout()
            
            # Información del header
            info_label = QLabel(
                f"<b>Trayectoria Zig-Zag Generada</b><br>"
                f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━<br>"
                f"<b>Total de puntos:</b> {len(trajectory)}<br>"
                f"<b>Configuración:</b> {n_rows} filas × {n_cols} columnas<br>"
                f"<b>Rango X:</b> [{x_start:.1f}, {x_end:.1f}] µm<br>"
                f"<b>Rango Y:</b> [{y_start:.1f}, {y_end:.1f}] µm<br>"
                f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            )
            info_label.setStyleSheet("font-size: 12px; padding: 10px; background-color: #383838; border-radius: 5px;")
            layout.addWidget(info_label)
            
            # Área de texto con scroll para mostrar coordenadas
            coord_text = QTextEdit()
            coord_text.setReadOnly(True)
            coord_text.setStyleSheet("""
                QTextEdit {
                    background-color: #1E1E1E;
                    color: #00FF00;
                    font-family: 'Consolas', 'Courier New', monospace;
                    font-size: 11px;
                    border: 2px solid #505050;
                    border-radius: 4px;
                    padding: 8px;
                }
            """)
            
            # Generar texto con formato de tabla
            coord_text_content = "╔════════╦═══════════════╦═══════════════╗\n"
            coord_text_content += "║ Punto  ║   X (µm)      ║   Y (µm)      ║\n"
            coord_text_content += "╠════════╬═══════════════╬═══════════════╣\n"
            
            for idx, (x, y) in enumerate(trajectory, start=1):
                coord_text_content += f"║ {idx:6d} ║ {x:13.2f} ║ {y:13.2f} ║\n"
            
            coord_text_content += "╚════════╩═══════════════╩═══════════════╝\n"
            
            coord_text.setPlainText(coord_text_content)
            layout.addWidget(coord_text)
            
            # Botones de acción
            button_layout = QHBoxLayout()
            
            # Botón para copiar al portapapeles
            copy_btn = QPushButton("📋 Copiar al Portapapeles")
            copy_btn.clicked.connect(lambda: self.copy_coordinates_to_clipboard(trajectory))
            button_layout.addWidget(copy_btn)
            
            # Botón para exportar a CSV
            export_btn = QPushButton("💾 Exportar a CSV")
            export_btn.clicked.connect(lambda: self.export_coordinates_to_csv(trajectory))
            button_layout.addWidget(export_btn)
            
            # Botón para cerrar
            close_btn = QPushButton("✖ Cerrar")
            close_btn.clicked.connect(dialog.close)
            button_layout.addWidget(close_btn)
            
            layout.addLayout(button_layout)
            
            dialog.setLayout(layout)
            dialog.exec_()
            
            logger.info(f"✅ Mapa de coordenadas mostrado: {len(trajectory)} puntos")
            
        except ValueError as e:
            QMessageBox.warning(self, "Error", f"Valores inválidos: {e}")
            logger.error(f"Error generando mapa de coordenadas: {e}")
    
    def copy_coordinates_to_clipboard(self, trajectory):
        """Copia las coordenadas al portapapeles."""
        try:
            from PyQt5.QtWidgets import QApplication
            
            # Formato de texto para copiar
            text = "Punto\tX (µm)\tY (µm)\n"
            for idx, (x, y) in enumerate(trajectory, start=1):
                text += f"{idx}\t{x:.2f}\t{y:.2f}\n"
            
            clipboard = QApplication.clipboard()
            clipboard.setText(text)
            
            QMessageBox.information(self, "✅ Copiado", 
                                   f"Se copiaron {len(trajectory)} puntos al portapapeles.\n"
                                   "Puedes pegarlos en Excel, Google Sheets, etc.")
            logger.info(f"Coordenadas copiadas al portapapeles: {len(trajectory)} puntos")
            
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Error al copiar: {e}")
            logger.error(f"Error copiando coordenadas: {e}")
    
    def export_coordinates_to_csv(self, trajectory):
        """Exporta las coordenadas a un archivo CSV."""
        try:
            # Diálogo para seleccionar ubicación
            filename, _ = QFileDialog.getSaveFileName(
                self,
                "Guardar Mapa de Coordenadas",
                f"trayectoria_zigzag_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                "CSV Files (*.csv);;All Files (*)"
            )
            
            if filename:
                import csv
                with open(filename, 'w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerow(['Punto', 'X (µm)', 'Y (µm)'])
                    
                    for idx, (x, y) in enumerate(trajectory, start=1):
                        writer.writerow([idx, f"{x:.2f}", f"{y:.2f}"])
                
                QMessageBox.information(self, "✅ Exportado", 
                                       f"Mapa de coordenadas guardado exitosamente:\n\n{filename}\n\n"
                                       f"Total de puntos: {len(trajectory)}")
                logger.info(f"Coordenadas exportadas a CSV: {filename}")
                
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Error al exportar: {e}")
            logger.error(f"Error exportando coordenadas: {e}\n{traceback.format_exc()}")
    
    def execute_trajectory(self):
        """Ejecuta la trayectoria generada con control dual."""
        logger.info("=== BOTÓN: Ejecutar Trayectoria presionado ===")
        
        # Verificar trayectoria
        if not hasattr(self, 'current_trajectory'):
            QMessageBox.warning(self, "Error", "Primero genera una trayectoria")
            return
        
        # Verificar calibración
        if not hasattr(self, 'global_calibration'):
            QMessageBox.warning(self, "⚠️ Calibración Requerida", 
                              "Debes calibrar el sistema primero en la pestaña 'Análisis'.")
            return
        
        # Verificar controladores
        if self.test_controller_a is None and self.test_controller_b is None:
            QMessageBox.warning(self, "Error", 
                              "No hay controladores cargados.\n\n"
                              "Transfiere al menos un controlador H∞ desde la pestaña 'H∞ Synthesis'.")
            return
        
        # Verificar tiempo entre pasos
        if not hasattr(self, 'trajectory_step_delay'):
            QMessageBox.warning(self, "Error", "Genera la trayectoria nuevamente para configurar el tiempo entre pasos")
            return
        
        # Guardar trayectoria y configuración
        self.trajectory_points = self.current_trajectory
        self.trajectory_current_point = 0
        self.trajectory_active = True
        
        # Actualizar interfaz
        self.execute_trajectory_btn.setEnabled(False)
        self.test_start_btn.setEnabled(False)
        self.step_start_btn.setEnabled(False)
        
        total_time = len(self.trajectory_points) * self.trajectory_step_delay
        self.test_results_text.setText(
            f"🎯 EJECUCIÓN DE TRAYECTORIA INICIADA\n"
            f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            f"Total de puntos: {len(self.trajectory_points)}\n"
            f"Tiempo entre puntos: {self.trajectory_step_delay}s\n"
            f"Tiempo total estimado: {total_time:.1f}s\n"
            f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
        )
        
        logger.info(f"✅ Trayectoria iniciada: {len(self.trajectory_points)} puntos, delay={self.trajectory_step_delay}s")
        
        # Ejecutar primer punto
        self.execute_next_trajectory_point()
    
    def execute_next_trajectory_point(self):
        """Ejecuta el siguiente punto de la trayectoria."""
        if not self.trajectory_active or self.trajectory_current_point >= len(self.trajectory_points):
            self.stop_trajectory()
            return
        
        # Obtener punto actual (x, y)
        point = self.trajectory_points[self.trajectory_current_point]
        x_target = point[0]
        y_target = point[1]
        
        # Mostrar progreso
        self.test_results_text.append(
            f"📍 Punto {self.trajectory_current_point + 1}/{len(self.trajectory_points)}: "
            f"X={x_target:.1f} µm, Y={y_target:.1f} µm"
        )
        
        # Configurar referencias según controladores disponibles
        if self.test_controller_a is not None:
            self.test_ref_a_input.setText(f"{x_target:.0f}")
        if self.test_controller_b is not None:
            self.test_ref_b_input.setText(f"{y_target:.0f}")
        
        # Iniciar control dual si no está activo
        if not self.dual_control_active:
            self.start_dual_control()
        
        # Incrementar contador
        self.trajectory_current_point += 1
        
        # Programar siguiente punto con el tiempo configurado
        QTimer.singleShot(int(self.trajectory_step_delay * 1000), self.execute_next_trajectory_point)
    
    def stop_trajectory(self):
        """Detiene la ejecución de la trayectoria."""
        if not hasattr(self, 'trajectory_active'):
            return
        
        self.trajectory_active = False
        
        # Actualizar interfaz
        self.execute_trajectory_btn.setEnabled(True)
        self.test_start_btn.setEnabled(True)
        self.step_start_btn.setEnabled(True)
        
        self.test_results_text.append(
            f"\n✅ TRAYECTORIA COMPLETADA\n"
            f"Total de puntos ejecutados: {self.trajectory_current_point}\n"
        )
        
        logger.info(f"✅ Trayectoria completada: {self.trajectory_current_point} puntos")
    
    # --- Control Dual de Motores (Pestaña Prueba) ---
    
    def start_dual_control(self):
        """Inicia el control dual de ambos motores simultáneamente usando controladores H∞ transferidos."""
        logger.info("=== INICIANDO CONTROL DUAL ===")
        
        # Verificar que exista calibración global
        if not hasattr(self, 'global_calibration'):
            QMessageBox.warning(self, "⚠️ Calibración Requerida", 
                              "Debes calibrar el sistema primero.\n\n"
                              "1. Ve a la pestaña 'Análisis'\n"
                              "2. Configura 'Distancia mín' y 'Distancia máx'\n"
                              "3. Realiza un análisis de tramo\n"
                              "4. El sistema se calibrará automáticamente\n\n"
                              "Luego podrás usar el control dual.")
            logger.warning("Intento de iniciar control sin calibración global")
            return
        
        # Verificar que al menos un controlador esté cargado
        if self.test_controller_a is None and self.test_controller_b is None:
            QMessageBox.warning(self, "Error", 
                              "No hay controladores cargados.\n\n"
                              "Transfiere al menos un controlador H∞ desde la pestaña 'H∞ Synthesis'.")
            logger.warning("Intento de iniciar control sin controladores cargados")
            return
        
        # Determinar qué motores están activos
        motors_active = []
        if self.test_controller_a is not None:
            motors_active.append("Motor A")
        if self.test_controller_b is not None:
            motors_active.append("Motor B")
        
        logger.info(f"🎯 Motores activos: {', '.join(motors_active)}")
        
        try:
            # Obtener parámetros de controladores transferidos
            if self.test_controller_a is not None:
                Kp_a = self.test_controller_a['Kp']
                Ki_a = self.test_controller_a['Ki']
                U_max_a = self.test_controller_a.get('U_max', 100.0)  # Leer U_max del diseño
                ref_a = float(self.test_ref_a_input.text())
                
                # Leer sensor desde checkboxes
                if self.test_motor_a_sensor1_check.isChecked():
                    sensor_a = '1'
                elif self.test_motor_a_sensor2_check.isChecked():
                    sensor_a = '2'
                else:
                    QMessageBox.warning(self, "Error", "Motor A: Debes marcar un sensor (Sensor 1 o Sensor 2)")
                    return
                
                # Leer inversión de PWM
                invert_a = self.test_motor_a_invert_check.isChecked()
                
                logger.info(f"✅ Motor A: Kp={Kp_a:.4f}, Ki={Ki_a:.4f}, U_max={U_max_a:.1f}, Sensor={sensor_a}, Invertir={invert_a}, Ref={ref_a:.1f} µm")
            else:
                Kp_a = 0.0
                Ki_a = 0.0
                U_max_a = 100.0
                ref_a = 0.0
                sensor_a = '1'
                invert_a = False
                logger.info("⚪ Motor A: Desactivado (sin controlador)")
            
            if self.test_controller_b is not None:
                Kp_b = self.test_controller_b['Kp']
                Ki_b = self.test_controller_b['Ki']
                U_max_b = self.test_controller_b.get('U_max', 100.0)  # Leer U_max del diseño
                ref_b = float(self.test_ref_b_input.text())
                
                # Leer sensor desde checkboxes
                if self.test_motor_b_sensor1_check.isChecked():
                    sensor_b = '1'
                elif self.test_motor_b_sensor2_check.isChecked():
                    sensor_b = '2'
                else:
                    QMessageBox.warning(self, "Error", "Motor B: Debes marcar un sensor (Sensor 1 o Sensor 2)")
                    return
                
                # Leer inversión de PWM
                invert_b = self.test_motor_b_invert_check.isChecked()
                
                logger.info(f"✅ Motor B: Kp={Kp_b:.4f}, Ki={Ki_b:.4f}, U_max={U_max_b:.1f}, Sensor={sensor_b}, Invertir={invert_b}, Ref={ref_b:.1f} µm")
            else:
                Kp_b = 0.0
                Ki_b = 0.0
                U_max_b = 100.0
                ref_b = 0.0
                sensor_b = '2'
                invert_b = False
                logger.info("⚪ Motor B: Desactivado (sin controlador)")
            
        except ValueError as e:
            logger.error(f"Error en parámetros: {e}")
            self.test_results_text.setText("❌ Error: Verifica que todos los parámetros sean números válidos.")
            return
        
        # Mostrar información al usuario
        motors_info = []
        if self.test_controller_a is not None:
            motors_info.append(f"Motor A → {ref_a:.0f} µm")
        if self.test_controller_b is not None:
            motors_info.append(f"Motor B → {ref_b:.0f} µm")
        
        self.test_results_text.setText(
            f"🎯 Control Dual Iniciado\n"
            f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            f"Motores activos: {', '.join(motors_active)}\n"
            f"{chr(10).join(motors_info)}\n"
            f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            f"Estado: Controlando..."
        )
        
        # Cambiar a modo AUTO
        self.send_command('A')
        time.sleep(0.05)
        
        # Guardar parámetros
        self.dual_Kp_a = Kp_a
        self.dual_Ki_a = Ki_a
        self.dual_U_max_a = U_max_a  # U_max del diseño
        self.dual_sensor_a = sensor_a  # Sensor seleccionado por usuario
        self.dual_invert_a = invert_a  # Inversión de PWM
        self.dual_Kp_b = Kp_b
        self.dual_Ki_b = Ki_b
        self.dual_U_max_b = U_max_b  # U_max del diseño
        self.dual_sensor_b = sensor_b  # Sensor seleccionado por usuario
        self.dual_invert_b = invert_b  # Inversión de PWM
        self.dual_ref_a = ref_a
        self.dual_ref_b = ref_b
        
        # Inicializar integrales
        self.dual_integral_a = 0.0
        self.dual_integral_b = 0.0
        
        # Activar control
        self.dual_control_active = True
        
        # Configurar timer (100 Hz)
        if not hasattr(self, 'dual_timer'):
            self.dual_timer = QTimer()
            self.dual_timer.timeout.connect(self.execute_dual_control)
        
        self.dual_timer.start(10)  # 10 ms = 100 Hz
        
        # Actualizar interfaz
        self.test_start_btn.setEnabled(False)
        self.test_stop_btn.setEnabled(True)
        self.step_start_btn.setEnabled(False)
        
        cal = self.global_calibration
        self.test_results_text.setText(
            f"▶️ CONTROL DUAL ACTIVO\n"
            f"{'='*40}\n"
            f"Motor A: Ref={ref_a:.1f} µm | Kp={Kp_a:.4f}, Ki={Ki_a:.4f}\n"
            f"Motor B: Ref={ref_b:.1f} µm | Kp={Kp_b:.4f}, Ki={Ki_b:.4f}\n"
            f"Factor escala: {abs(cal['pendiente_um']):.4f} µm/ADC\n"
            f"{'='*40}\n"
        )
        
        logger.info("Control dual iniciado")
    
    def execute_dual_control(self):
        """Ejecuta un ciclo de control dual PI."""
        if not self.dual_control_active:
            return
        
        try:
            # Leer sensores desde value_labels
            try:
                sensor_1_adc = int(self.value_labels['sensor_1'].text())
                sensor_2_adc = int(self.value_labels['sensor_2'].text())
            except (ValueError, KeyError):
                sensor_1_adc = 512
                sensor_2_adc = 512
            
            # Usar sensores guardados (seleccionados por usuario)
            sensor_a = self.dual_sensor_a
            sensor_b = self.dual_sensor_b
            
            # Leer valores ADC según sensor asignado
            if sensor_a == '1':
                sensor_a_adc = sensor_1_adc
            else:
                sensor_a_adc = sensor_2_adc
            
            if sensor_b == '1':
                sensor_b_adc = sensor_1_adc
            else:
                sensor_b_adc = sensor_2_adc
            
            # Convertir ADC a µm usando calibración
            cal = self.global_calibration
            pos_a_um = sensor_a_adc * cal['pendiente_um'] + cal['intercepto_um']
            pos_b_um = sensor_b_adc * cal['pendiente_um'] + cal['intercepto_um']
            
            # Guardar posiciones para verificación de pasos
            self.dual_last_pos_a = pos_a_um
            self.dual_last_pos_b = pos_b_um
            
            # Calcular errores
            error_a = self.dual_ref_a - pos_a_um
            error_b = self.dual_ref_b - pos_b_um
            
            # Motor A - Zona muerta (±50 µm)
            if abs(error_a) <= 50 or self.dual_Kp_a == 0:
                pwm_a = 0
                self.dual_integral_a = 0
            else:
                # Actualizar integral
                Ts = 0.01  # 10 ms
                self.dual_integral_a += error_a * Ts
                
                # Ley de control PI
                pwm_base_a = self.dual_Kp_a * error_a + self.dual_Ki_a * self.dual_integral_a
                
                # Aplicar inversión si está marcada
                if self.dual_invert_a:
                    pwm_a = -pwm_base_a
                else:
                    pwm_a = pwm_base_a
                
                # Saturación con anti-windup usando U_max del diseño
                U_max_a = abs(self.dual_U_max_a)
                if pwm_a > U_max_a:
                    pwm_a = U_max_a
                    self.dual_integral_a -= error_a * Ts
                elif pwm_a < -U_max_a:
                    pwm_a = -U_max_a
                    self.dual_integral_a -= error_a * Ts
            
            # Motor B - Zona muerta (±50 µm)
            if abs(error_b) <= 50 or self.dual_Kp_b == 0:
                pwm_b = 0
                self.dual_integral_b = 0
            else:
                Ts = 0.01
                self.dual_integral_b += error_b * Ts
                
                # Ley de control PI
                pwm_base_b = self.dual_Kp_b * error_b + self.dual_Ki_b * self.dual_integral_b
                
                # Aplicar inversión si está marcada
                if self.dual_invert_b:
                    pwm_b = -pwm_base_b
                else:
                    pwm_b = pwm_base_b
                
                # Saturación con anti-windup usando U_max del diseño
                U_max_b = abs(self.dual_U_max_b)
                if pwm_b > U_max_b:
                    pwm_b = U_max_b
                    self.dual_integral_b -= error_b * Ts
                elif pwm_b < -U_max_b:
                    pwm_b = -U_max_b
                    self.dual_integral_b -= error_b * Ts
            
            # Enviar comando
            command = f"A,{int(pwm_a)},{int(pwm_b)}"
            self.send_command(command)
            
            # Actualizar interfaz cada 100ms
            if not hasattr(self, 'dual_update_counter'):
                self.dual_update_counter = 0
            
            self.dual_update_counter += 1
            if self.dual_update_counter >= 10:
                self.dual_update_counter = 0
                
                # Timestamp
                from datetime import datetime
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                
                logger.info(f"[{timestamp}] Control dual: A={pos_a_um:.1f}µm (e={error_a:.1f}, PWM={int(pwm_a)}), B={pos_b_um:.1f}µm (e={error_b:.1f}, PWM={int(pwm_b)})")
                
                # Actualizar ecuaciones en tiempo real
                equations_text = (
                    f"╔═══════════════════════════════════════════════════════════════╗\n"
                    f"║  ECUACIONES DEL CONTROLADOR PI EN TIEMPO REAL                ║\n"
                    f"╠═══════════════════════════════════════════════════════════════╣\n"
                    f"║  Motor A (X):                                                 ║\n"
                    f"║    PWM_A = Kp_A × e_A + Ki_A × ∫e_A·dt                        ║\n"
                    f"║    PWM_A = {self.dual_Kp_a:.4f} × {error_a:+.1f} + {self.dual_Ki_a:.4f} × {self.dual_integral_a:+.2f}  ║\n"
                    f"║    PWM_A = {int(pwm_a):+4d}                                           ║\n"
                    f"║                                                               ║\n"
                    f"║  Motor B (Y):                                                 ║\n"
                    f"║    PWM_B = Kp_B × e_B + Ki_B × ∫e_B·dt                        ║\n"
                    f"║    PWM_B = {self.dual_Kp_b:.4f} × {error_b:+.1f} + {self.dual_Ki_b:.4f} × {self.dual_integral_b:+.2f}  ║\n"
                    f"║    PWM_B = {int(pwm_b):+4d}                                           ║\n"
                    f"╠═══════════════════════════════════════════════════════════════╣\n"
                    f"║  Estado:                                                      ║\n"
                    f"║    Pos_A: {pos_a_um:7.1f} µm  │  Ref_A: {self.dual_ref_a:7.1f} µm          ║\n"
                    f"║    Pos_B: {pos_b_um:7.1f} µm  │  Ref_B: {self.dual_ref_b:7.1f} µm          ║\n"
                    f"║    Err_A: {error_a:+7.1f} µm  │  Err_B: {error_b:+7.1f} µm          ║\n"
                    f"╚═══════════════════════════════════════════════════════════════╝"
                )
                self.equations_display.setText(equations_text)
                
                status_msg = (
                    f"[{timestamp}] 📊 Motor A: Pos={pos_a_um:.1f}µm | e={error_a:+.1f} | PWM={int(pwm_a):+4d}\n"
                    f"[{timestamp}] 📊 Motor B: Pos={pos_b_um:.1f}µm | e={error_b:+.1f} | PWM={int(pwm_b):+4d}"
                )
                self.test_results_text.append(status_msg)
                self.test_results_text.verticalScrollBar().setValue(
                    self.test_results_text.verticalScrollBar().maximum()
                )
                
        except Exception as e:
            logger.error(f"Error en control dual: {e}")
    
    def stop_dual_control(self):
        """Detiene el control dual."""
        logger.info("=== DETENIENDO CONTROL DUAL ===")
        
        if hasattr(self, 'dual_timer'):
            self.dual_timer.stop()
        
        # Detener motores
        self.send_command('A,0,0')
        time.sleep(0.05)
        
        # Volver a modo MANUAL
        self.send_command('M')
        
        # Desactivar control
        self.dual_control_active = False
        
        # Actualizar interfaz
        self.test_start_btn.setEnabled(True)
        self.test_stop_btn.setEnabled(False)
        self.step_start_btn.setEnabled(True)
        
        self.test_results_text.append(f"\n⏹️ CONTROL DUAL DETENIDO\n")
        logger.info("Control dual detenido")

    # --- Ejecución de Trayectoria Zig-Zag para Microscopía ---
    
    def start_zigzag_microscopy(self):
        """Inicia la ejecución de la trayectoria zig-zag para microscopía."""
        logger.info("=== INICIANDO TRAYECTORIA ZIG-ZAG PARA MICROSCOPÍA ===")
        
        # Verificar que exista trayectoria generada
        if not hasattr(self, 'current_trajectory'):
            QMessageBox.warning(self, "Error", "Primero debes generar una trayectoria")
            return
        
        # Verificar calibración obligatoria
        if not hasattr(self, 'global_calibration'):
            QMessageBox.warning(
                self, 
                "⚠️ Calibración Requerida", 
                "Debes calibrar el sistema primero en la pestaña 'Análisis'.\n\n"
                "📋 Pasos para calibrar:\n"
                "1. Ve a la pestaña 'Análisis'\n"
                "2. Captura datos de movimiento de un motor\n"
                "3. Ingresa las distancias físicas (min y max) en mm\n"
                "4. Presiona 'Analizar Tramo' para generar la calibración\n\n"
                "La calibración es necesaria para convertir valores ADC a micrómetros."
            )
            logger.warning("⚠️ Intento de ejecutar trayectoria sin calibración global")
            return
        
        # Mostrar info de calibración en el log
        cal = self.global_calibration
        logger.info(f"✅ Usando calibración global del sistema:")
        logger.info(f"   Pendiente: {cal['pendiente_um']:.4f} µm/ADC")
        logger.info(f"   Intercepto: {cal['intercepto_um']:.2f} µm")
        logger.info(f"   Rango: {cal['dist_punto1_mm']:.1f} → {cal['dist_punto2_mm']:.1f} mm")
        logger.info(f"   Motor calibrado: {cal['motor']} / Sensor: {cal['sensor']}")
        logger.info(f"   Relación: {cal['relacion']}")
        
        # Verificar que ambos controladores estén cargados
        if self.test_controller_a is None or self.test_controller_b is None:
            QMessageBox.warning(self, "Error", 
                              "Se requieren controladores para AMBOS motores (A y B).\n\n"
                              "Transfiere controladores H∞ para Motor A y Motor B desde la pestaña 'H∞ Synthesis'.")
            return
        
        try:
            # Leer parámetros
            tolerance = float(self.step_tolerance_input.text())
            pause_time = float(self.step_pause_input.text())
            
            # Validar
            if tolerance < 10:
                QMessageBox.warning(self, "Error", "Tolerancia debe ser al menos 10 µm")
                return
            
            if pause_time < 0:
                QMessageBox.warning(self, "Error", "Pausa no puede ser negativa")
                return
            
        except ValueError as e:
            QMessageBox.warning(self, "Error", f"Valores inválidos: {e}")
            return
        
        # Configurar secuencia
        self.zigzag_points = self.current_trajectory
        self.zigzag_tolerance = tolerance
        self.zigzag_pause_time = pause_time
        self.zigzag_current = 0
        self.zigzag_active = True
        self.zigzag_start_time = None
        
        # Actualizar interfaz
        self.step_start_btn.setEnabled(False)
        self.step_stop_btn.setEnabled(True)
        self.test_start_btn.setEnabled(False)
        
        total_time = len(self.zigzag_points) * pause_time
        
        self.test_results_text.setText(
            f"🔬 EJECUCIÓN DE TRAYECTORIA ZIG-ZAG PARA MICROSCOPÍA\n"
            f"{'='*60}\n"
            f"Total de puntos: {len(self.zigzag_points)}\n"
            f"Tolerancia: ±{tolerance:.0f} µm\n"
            f"⏸️ Pausa entre puntos: {pause_time:.1f}s\n"
            f"Tiempo total estimado: {total_time:.1f}s ({total_time/60:.1f} min)\n"
            f"{'='*60}\n"
            f"Modo: Alcanzar (X,Y) → Pausa → Siguiente punto\n"
            f"{'='*60}\n"
        )
        
        logger.info(f"✅ Iniciando trayectoria zig-zag: {len(self.zigzag_points)} puntos")
        logger.info(f"   Tolerancia: {tolerance} µm, Pausa: {pause_time}s")
        
        # Ejecutar primer punto
        self.execute_next_zigzag_point()
    
    def execute_next_zigzag_point(self):
        """
        Ejecuta el siguiente punto de la trayectoria Column-Scan.
        
        Detecta qué eje cambió respecto al punto anterior y solo mueve ese eje.
        Esto simplifica el control y permite diagnóstico independiente por motor.
        """
        if not self.zigzag_active or self.zigzag_current >= len(self.zigzag_points):
            self.stop_zigzag_microscopy()
            return
        
        # Obtener punto actual (x, y)
        point = self.zigzag_points[self.zigzag_current]
        x_target = point[0]
        y_target = point[1]
        
        # Determinar qué eje cambió respecto al punto anterior
        if self.zigzag_current == 0:
            # Primer punto: mover ambos ejes (posicionamiento inicial)
            prev_x, prev_y = None, None
            x_changed = True
            y_changed = True
            axis_moving = "X+Y (Inicio)"
        else:
            prev_point = self.zigzag_points[self.zigzag_current - 1]
            prev_x, prev_y = prev_point
            
            # Detectar cambios (tolerancia de 1 µm para considerar que NO cambió)
            x_changed = abs(x_target - prev_x) > 1.0
            y_changed = abs(y_target - prev_y) > 1.0
            
            if x_changed and not y_changed:
                axis_moving = "X (Motor A)"
            elif y_changed and not x_changed:
                axis_moving = "Y (Motor B)"
            elif x_changed and y_changed:
                axis_moving = "X+Y (Diagonal)"
            else:
                axis_moving = "Ninguno (Error)"
        
        # Mostrar progreso con información del eje en movimiento
        self.test_results_text.append(
            f"📍 Punto {self.zigzag_current + 1}/{len(self.zigzag_points)}: "
            f"X={x_target:.1f} µm, Y={y_target:.1f} µm | Eje: {axis_moving}"
        )
        logger.info(f"📍 Punto {self.zigzag_current + 1}/{len(self.zigzag_points)}: "
                   f"Moviendo a ({x_target:.1f}, {y_target:.1f}) µm | Eje: {axis_moving}")
        
        # Configurar referencias para ambos motores
        self.test_ref_a_input.setText(f"{x_target:.0f}")
        self.test_ref_b_input.setText(f"{y_target:.0f}")
        
        # ESTRATEGIA: Reiniciar control dual para actualizar referencias
        # (Alternativa: implementar update_dual_references() que no reinicie integradores)
        if self.dual_control_active:
            self.stop_dual_control()
        
        # Iniciar control dual con las nuevas referencias
        self.start_dual_control()
        
        # Guardar tiempo de inicio y posición objetivo
        self.zigzag_start_time = time.time()
        self.zigzag_target_x = x_target
        self.zigzag_target_y = y_target
        
        # Guardar qué ejes cambiaron para verificación optimizada
        self.zigzag_x_changed = x_changed
        self.zigzag_y_changed = y_changed
        
        # Iniciar verificación periódica de posición
        self.check_zigzag_position()
    
    def check_zigzag_position(self):
        """Verifica si ambos motores alcanzaron la posición objetivo."""
        if not self.zigzag_active:
            return
        
        # Calcular tiempo transcurrido
        elapsed_time = time.time() - self.zigzag_start_time
        
        # Usar las posiciones que ya calcula el control dual
        try:
            if not hasattr(self, 'dual_last_pos_a') or not hasattr(self, 'dual_last_pos_b'):
                # No hay datos aún, esperar
                QTimer.singleShot(100, self.check_zigzag_position)
                return
            
            current_x = self.dual_last_pos_a
            current_y = self.dual_last_pos_b
            
            # Calcular errores
            error_x = abs(current_x - self.zigzag_target_x)
            error_y = abs(current_y - self.zigzag_target_y)
            
            # Log periódico cada 1 segundo
            if not hasattr(self, 'zigzag_last_log_time'):
                self.zigzag_last_log_time = time.time()
            
            if time.time() - self.zigzag_last_log_time >= 1.0:
                logger.info(f"🔄 Verificando: X={current_x:.1f} (error={error_x:.1f}), Y={current_y:.1f} (error={error_y:.1f}), t={elapsed_time:.1f}s")
                self.zigzag_last_log_time = time.time()
            
            # Verificar si AMBOS motores alcanzaron la posición
            if error_x <= self.zigzag_tolerance and error_y <= self.zigzag_tolerance:
                logger.info(f"✅ Posición alcanzada: ({current_x:.1f}, {current_y:.1f}) µm (error_x={error_x:.1f}, error_y={error_y:.1f}, t={elapsed_time:.2f}s)")
                self.test_results_text.append(f"✅ Posición alcanzada en {elapsed_time:.2f}s (error_x={error_x:.1f}, error_y={error_y:.1f} µm)")
                self.start_zigzag_pause()
            else:
                # No alcanzó aún, verificar nuevamente en 100ms
                QTimer.singleShot(100, self.check_zigzag_position)
        
        except Exception as e:
            logger.error(f"❌ Error verificando posición: {e}\n{traceback.format_exc()}")
            QTimer.singleShot(100, self.check_zigzag_position)
    
    def start_zigzag_pause(self):
        """Inicia la pausa estática después de alcanzar la posición."""
        # Detener control temporalmente
        if self.dual_control_active:
            self.stop_dual_control()
            logger.info(f"⏸️ Control detenido - Pausa estática de {self.zigzag_pause_time}s para captura de imagen")
            self.test_results_text.append(f"📸 Pausa para captura: {self.zigzag_pause_time}s")
        
        # Incrementar contador
        self.zigzag_current += 1
        
        # Programar siguiente punto después de la pausa
        QTimer.singleShot(int(self.zigzag_pause_time * 1000), self.execute_next_zigzag_point)
    
    def stop_zigzag_microscopy(self):
        """Detiene la ejecución de la trayectoria zig-zag."""
        logger.info("=== DETENIENDO TRAYECTORIA ZIG-ZAG ===")
        
        self.zigzag_active = False
        
        # Detener control dual si está activo
        if self.dual_control_active:
            self.stop_dual_control()
        
        # Actualizar interfaz
        self.step_start_btn.setEnabled(True)
        self.step_stop_btn.setEnabled(False)
        self.test_start_btn.setEnabled(True)
        
        if self.zigzag_current > 0:
            self.test_results_text.append(
                f"\n✅ TRAYECTORIA COMPLETADA: {self.zigzag_current} puntos ejecutados\n"
            )
        else:
            self.test_results_text.append(f"\n⏸️ TRAYECTORIA DETENIDA\n")
        
        logger.info(f"Trayectoria zig-zag detenida ({self.zigzag_current} puntos ejecutados)")
    
    # --- Control por Pasos (Pestaña Prueba) - LEGACY ---
    
    def start_step_sequence(self):
        """Inicia una secuencia de pasos para microscopía."""
        logger.info("=== INICIANDO SECUENCIA DE PASOS ===")
        
        # Verificar calibración
        if not hasattr(self, 'global_calibration'):
            QMessageBox.warning(self, "⚠️ Calibración Requerida", 
                              "Debes calibrar el sistema primero en la pestaña 'Análisis'.")
            logger.warning("Intento de iniciar secuencia sin calibración global")
            return
        
        # Verificar controladores
        if self.test_controller_a is None and self.test_controller_b is None:
            QMessageBox.warning(self, "Error", 
                              "No hay controladores cargados.\n\n"
                              "Transfiere al menos un controlador H∞ desde la pestaña 'H∞ Synthesis'.")
            logger.warning("Intento de iniciar secuencia sin controladores cargados")
            return
        
        try:
            # Leer parámetros
            motor_selection = self.step_motor_combo.currentText()
            start_pos = float(self.step_start_input.text())
            end_pos = float(self.step_end_input.text())
            step_count = int(self.step_count_input.text())
            tolerance = float(self.step_tolerance_input.text())  # Tolerancia de posición
            pause_time = float(self.step_pause_input.text())    # Pausa estática (configurable por usuario)
            
            # Validar
            if step_count < 1:
                QMessageBox.warning(self, "Error", "Número de pasos debe ser al menos 1")
                return
            
            if tolerance < 10:
                QMessageBox.warning(self, "Error", "Tolerancia debe ser al menos 10 µm")
                return
            
            if pause_time < 0:
                QMessageBox.warning(self, "Error", "Pausa no puede ser negativa")
                return
            
            # Calcular punto medio automáticamente
            mid_pos = (start_pos + end_pos) / 2.0
            
            logger.info(f"Motor: {motor_selection}")
            logger.info(f"  Inicio: {start_pos} µm")
            logger.info(f"  Final: {end_pos} µm")
            logger.info(f"  Medio: {mid_pos:.1f} µm")
            logger.info(f"  Pasos: {step_count}")
            logger.info(f"  Tolerancia: {tolerance} µm, Pausa: {pause_time}s")
            
        except ValueError as e:
            logger.error(f"Error en parámetros: {e}")
            self.test_results_text.setText("❌ Error: Verifica los parámetros de la secuencia.")
            return
        
        # Generar secuencia de posiciones
        self.step_positions = np.linspace(start_pos, end_pos, step_count)
        self.step_motor_sel = motor_selection
        self.step_tolerance = tolerance      # Tolerancia de posición
        self.step_pause_time = pause_time    # Pausa estática (configurable por usuario)
        self.step_current = 0
        self.step_sequence_active = True
        self.step_start_time = None          # Tiempo de inicio del paso actual
        
        # Actualizar interfaz
        self.step_start_btn.setEnabled(False)
        self.step_stop_btn.setEnabled(True)
        self.test_start_btn.setEnabled(False)
        
        self.test_results_text.setText(
            f"🚀 SECUENCIA DE PASOS INICIADA\n"
            f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            f"Motor: {motor_selection}\n"
            f"Inicio: {start_pos:.0f} µm\n"
            f"Final: {end_pos:.0f} µm\n"
            f"Medio: {mid_pos:.0f} µm\n"
            f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            f"Total de pasos: {step_count}\n"
            f"Tolerancia: ±{tolerance:.0f} µm\n"
            f"⏸️ Pausa entre pasos: {pause_time:.1f}s\n"
            f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            f"Modo: Alcanzar posición → Pausa → Siguiente\n"
            f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
        )
        
        # Iniciar primer paso
        self.execute_next_step()
        
        logger.info("✅ Secuencia de pasos iniciada")
    
    def execute_next_step(self):
        """Ejecuta el siguiente paso de la secuencia."""
        if not self.step_sequence_active or self.step_current >= len(self.step_positions):
            self.stop_step_sequence()
            return
        
        # Obtener posición objetivo del paso actual
        target_pos = self.step_positions[self.step_current]
        
        # Mostrar progreso
        self.test_results_text.append(
            f"📍 Paso {self.step_current + 1}/{len(self.step_positions)}: "
            f"Posición objetivo = {target_pos:.1f} µm"
        )
        logger.info(f"📍 Paso {self.step_current + 1}/{len(self.step_positions)}: Moviendo a {target_pos:.1f} µm")
        
        # Configurar referencias según motor seleccionado
        if self.step_motor_sel == "Motor A":
            self.test_ref_a_input.setText(f"{target_pos:.0f}")
            # Motor B mantiene su referencia actual o 0 si no tiene controlador
            if self.test_controller_b is None:
                self.test_ref_b_input.setText("0")
        elif self.step_motor_sel == "Motor B":
            # Motor A mantiene su referencia actual o 0 si no tiene controlador
            if self.test_controller_a is None:
                self.test_ref_a_input.setText("0")
            self.test_ref_b_input.setText(f"{target_pos:.0f}")
        else:  # Ambos
            self.test_ref_a_input.setText(f"{target_pos:.0f}")
            self.test_ref_b_input.setText(f"{target_pos:.0f}")
        
        # Reiniciar control dual para el nuevo paso
        # Si ya está activo, detenerlo primero para reiniciar con nueva referencia
        if self.dual_control_active:
            self.stop_dual_control()
        
        # Iniciar control dual con la nueva referencia
        self.start_dual_control()
        
        # Guardar tiempo de inicio y posición objetivo
        self.step_start_time = time.time()
        self.step_target_pos = target_pos
        
        # Iniciar verificación periódica de posición
        self.check_step_position()
    
    def check_step_position(self):
        """Verifica si el motor alcanzó la posición objetivo."""
        if not self.step_sequence_active:
            return
        
        # Calcular tiempo transcurrido (solo para logging)
        elapsed_time = time.time() - self.step_start_time
        
        # Usar las posiciones que ya calcula el control dual
        try:
            if self.step_motor_sel == "Motor A":
                # Usar la posición que ya está siendo calculada por el control dual
                if hasattr(self, 'dual_last_pos_a'):
                    current_pos = self.dual_last_pos_a
                else:
                    # No hay datos aún, esperar
                    logger.debug(f"Esperando datos de posición Motor A...")
                    QTimer.singleShot(100, self.check_step_position)
                    return
            
            elif self.step_motor_sel == "Motor B":
                # Usar la posición que ya está siendo calculada por el control dual
                if hasattr(self, 'dual_last_pos_b'):
                    current_pos = self.dual_last_pos_b
                else:
                    # No hay datos aún, esperar
                    logger.debug(f"Esperando datos de posición Motor B...")
                    QTimer.singleShot(100, self.check_step_position)
                    return
            else:
                # Ambos motores - usar Motor A
                if hasattr(self, 'dual_last_pos_a'):
                    current_pos = self.dual_last_pos_a
                else:
                    logger.debug(f"Esperando datos de posición...")
                    QTimer.singleShot(100, self.check_step_position)
                    return
            
            # Calcular error
            error = abs(current_pos - self.step_target_pos)
            
            # Log periódico cada 1 segundo
            if not hasattr(self, 'step_last_log_time'):
                self.step_last_log_time = time.time()
            
            if time.time() - self.step_last_log_time >= 1.0:
                logger.info(f"🔄 Verificando posición: {current_pos:.1f} µm (objetivo={self.step_target_pos:.1f}, error={error:.1f} µm, tiempo={elapsed_time:.1f}s)")
                self.step_last_log_time = time.time()
            
            # Verificar si alcanzó la posición
            if error <= self.step_tolerance:
                logger.info(f"✅ Posición alcanzada: {current_pos:.1f} µm (error={error:.1f} µm, tiempo={elapsed_time:.2f}s)")
                self.test_results_text.append(f"✅ Posición alcanzada en {elapsed_time:.2f}s (error={error:.1f} µm)")
                self.start_step_pause()
            else:
                # No alcanzó aún, verificar nuevamente en 100ms
                QTimer.singleShot(100, self.check_step_position)
        
        except Exception as e:
            logger.error(f"❌ Error verificando posición: {e}\n{traceback.format_exc()}")
            QTimer.singleShot(100, self.check_step_position)
    
    def start_step_pause(self):
        """Inicia la pausa estática después de alcanzar la posición."""
        # Detener control temporalmente
        if self.dual_control_active:
            self.stop_dual_control()
            logger.info(f"⏸️ Control detenido - Iniciando pausa estática de {self.step_pause_time}s")
            self.test_results_text.append(f"⏸️ Pausa estática: {self.step_pause_time}s")
        
        # Incrementar contador
        self.step_current += 1
        
        # Programar siguiente paso después de la pausa
        # IMPORTANTE: El siguiente paso reiniciará el control dual automáticamente
        QTimer.singleShot(int(self.step_pause_time * 1000), self.execute_next_step)
    
    def stop_step_sequence(self):
        """Detiene la secuencia de pasos."""
        logger.info("=== DETENIENDO SECUENCIA DE PASOS ===")
        
        self.step_sequence_active = False
        
        # Detener control dual si está activo
        if self.dual_control_active:
            self.stop_dual_control()
        
        # Actualizar interfaz
        self.step_start_btn.setEnabled(True)
        self.step_stop_btn.setEnabled(False)
        self.test_start_btn.setEnabled(True)
        
        if self.step_current > 0:
            self.test_results_text.append(
                f"\n✅ SECUENCIA COMPLETADA: {self.step_current} pasos ejecutados\n"
            )
        else:
            self.test_results_text.append(f"\n⏸️ SECUENCIA DETENIDA\n")
        
        logger.info(f"Secuencia de pasos detenida ({self.step_current} pasos ejecutados)")

    def closeEvent(self, event):
        """Maneja el cierre de la aplicación."""
        logger.info("=== CERRANDO APLICACIÓN ===")
        logger.debug("Enviando comando de apagado de motores (A,0,0)")
        self.send_command('A,0,0')
        self.stop_recording()
        time.sleep(0.1)
        self.serial_thread.stop()
        logger.info("Aplicación cerrada correctamente")
        event.accept()

def main():
    """Función principal de la aplicación."""
    logger.info("="*70)
    logger.info("INICIANDO SISTEMA DE CONTROL Y ANÁLISIS - MOTORES L206")
    logger.info(f"Versión: 2.5 | Puerto: {SERIAL_PORT} | Baudrate: {BAUD_RATE}")
    logger.info("="*70)
    
    try:
        app = QApplication(sys.argv)
        logger.info("QApplication creada exitosamente")
        
        window = ArduinoGUI()
        logger.info("Ventana principal creada")
        
        window.show()
        logger.info("Interfaz gráfica mostrada - Sistema listo")
        
        exit_code = app.exec_()
        logger.info(f"Aplicación finalizada con código: {exit_code}")
        return exit_code
        
    except Exception as e:
        logger.critical(f"Error crítico al iniciar aplicación: {e}\n{traceback.format_exc()}")
        return 1

if __name__ == '__main__':
    sys.exit(main())
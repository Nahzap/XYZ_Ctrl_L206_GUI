import sys
import os
import time
import cv2
import numpy as np
import serial
from pylablib.devices import Thorlabs
from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QLineEdit, QTextEdit, QFormLayout, QGroupBox, QGridLayout)
from PyQt6.QtCore import QThread, pyqtSignal, QObject, Qt
from PyQt6.QtGui import QImage, QPixmap

# --- CONFIGURACIÓN ARDUINO ---
SERIAL_PORT = 'COM3'
BAUD_RATE = 1000000


# =========================================================================
# --- Worker para la Cámara (se ejecuta en un hilo separado) ---
# =========================================================================
class CameraWorker(QObject):
    status_update = pyqtSignal(str)
    connection_success = pyqtSignal(bool)
    new_frame_ready = pyqtSignal(QImage)

    def __init__(self):
        super().__init__()
        self.cam = None
        self.running = False
        self.exposure = 0.01

    def connect_camera(self):
        try:
            self.status_update.emit("Conectando con la cámara Thorlabs...")
            self.cam = Thorlabs.ThorlabsTLCamera()
            info = self.cam.get_device_info()
            self.status_update.emit(f"Conexión exitosa con: {info.serial_number}")
            self.connection_success.emit(True)
        except Exception as e:
            self.status_update.emit(f"Error al conectar: {e}")
            self.connection_success.emit(False)

    def start_live_view(self):
        if not self.cam or not self.cam.is_opened():
            self.status_update.emit("Error: La cámara no está conectada.")
            return

        try:
            self.status_update.emit("Iniciando vista en vivo...")
            self.cam.set_exposure(self.exposure)
            self.cam.set_trigger_mode("int")
            self.cam.start_acquisition()
            self.running = True

            while self.running:
                # wait_for_frame() solo espera a que haya un frame disponible (retorna True/False)
                # NO retorna el frame en sí
                frame_available = self.cam.wait_for_frame(timeout=0.5)

                if frame_available:
                    # Ahora sí leemos el frame con read_oldest_image()
                    frame = self.cam.read_oldest_image()
                    
                    if frame is not None:
                        # CORRECCIÓN CRÍTICA: Hacer una copia explícita del frame
                        # para evitar que el driver lo sobrescriba y cause un fallo de memoria.
                        frame_copy = frame.copy()

                        if frame_copy.dtype != np.uint8:
                            frame_copy = (frame_copy / frame_copy.max() * 255).astype(np.uint8)

                        h, w = frame_copy.shape
                        bytes_per_line = w
                        
                        # Crear QImage con copia de datos para evitar problemas de memoria
                        q_image = QImage(frame_copy.data, w, h, bytes_per_line, QImage.Format.Format_Grayscale8).copy()

                        self.new_frame_ready.emit(q_image)
                elif frame_available is False:
                    # False indica que la adquisición se detuvo
                    self.status_update.emit("La adquisición se detuvo inesperadamente.")
                    break

        except Exception as e:
            self.status_update.emit(f"Error en la vista en vivo: {e}")
        finally:
            if self.cam and self.cam.is_opened() and self.cam.is_acquisition_setup():
                self.cam.stop_acquisition()
            self.status_update.emit("Vista en vivo detenida.")

    def stop_live_view(self):
        self.running = False

    def change_exposure(self, exposure_value):
        """Cambiar exposición en tiempo real"""
        try:
            if self.cam and self.cam.is_opened():
                self.exposure = exposure_value
                self.cam.set_exposure(exposure_value)
                self.status_update.emit(f"Exposición cambiada a {exposure_value} s")
            else:
                self.status_update.emit("Error: Cámara no conectada")
        except Exception as e:
            self.status_update.emit(f"Error al cambiar exposición: {e}")

    def disconnect_camera(self):
        if self.cam and self.cam.is_opened():
            self.stop_live_view()
            self.status_update.emit("Cerrando conexión...")
            self.cam.close()
            self.status_update.emit("Cámara cerrada.")


# =========================================================================
# --- Worker para Arduino Serial (se ejecuta en un hilo separado) ---
# =========================================================================
class SerialReaderThread(QThread):
    """
    Clase para leer el puerto serie en un hilo separado para no bloquear la GUI.
    """
    data_received = pyqtSignal(str)  # Señal que se emite cuando se recibe una línea

    def __init__(self, port, baudrate):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.ser = None

    def run(self):
        """
        El cuerpo del hilo. Se conecta y lee datos continuamente.
        """
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Dar tiempo para que se establezca la conexión

            while self.running and self.ser.is_open:
                line = self.ser.readline()
                if line:
                    try:
                        decoded_line = line.decode('utf-8').strip()
                        self.data_received.emit(decoded_line)
                    except UnicodeDecodeError:
                        pass  # Ignorar bytes corruptos

            if self.ser.is_open:
                self.ser.close()
        except serial.SerialException:
            self.data_received.emit(f"ERROR: Puerto {self.port} no encontrado.")

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.wait()


# =========================================================================
# --- Interfaz Gráfica Principal (GUI) ---
# =========================================================================
class CameraControlGUI(QWidget):
    # Señales para comunicarse con el worker
    start_live_view_signal = pyqtSignal()
    change_exposure_signal = pyqtSignal(float)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Control Integrado - Cámara Thorlabs + Arduino XYZ")
        self.setGeometry(100, 100, 1400, 750)
        self.setMinimumSize(1400, 750)
        self.setMaximumSize(1400, 750)  # Fijar tamaño de ventana
        self.current_frame_pixmap = None  # Almacenará la última imagen para guardarla

        # Worker de cámara
        self.worker_thread = QThread()
        self.camera_worker = CameraWorker()
        self.camera_worker.moveToThread(self.worker_thread)

        # Conexiones de señales y slots de cámara
        self.worker_thread.started.connect(self.camera_worker.connect_camera)
        self.worker_thread.finished.connect(self.camera_worker.disconnect_camera)
        self.camera_worker.status_update.connect(self.log_status)
        self.camera_worker.connection_success.connect(self.on_connection_result)
        self.camera_worker.new_frame_ready.connect(self.update_video_frame)
        self.start_live_view_signal.connect(self.camera_worker.start_live_view)
        self.change_exposure_signal.connect(self.camera_worker.change_exposure)

        # Worker de Arduino
        self.serial_thread = SerialReaderThread(SERIAL_PORT, BAUD_RATE)
        self.serial_thread.data_received.connect(self.update_arduino_data)
        
        # Diccionario para las etiquetas de Arduino
        self.arduino_labels = {}

        self.init_ui()

    def init_ui(self):
        main_layout = QHBoxLayout(self)  # Layout principal horizontal

        # ===== COLUMNA IZQUIERDA: CÁMARA =====
        left_column = QVBoxLayout()
        
        # Botones de control de cámara
        camera_controls = QHBoxLayout()
        self.connect_button = QPushButton("Conectar Cámara")
        self.start_view_button = QPushButton("Iniciar Vista")
        self.stop_view_button = QPushButton("Detener Vista")
        self.record_button = QPushButton("Record")

        self.connect_button.clicked.connect(self.start_connection)
        self.start_view_button.clicked.connect(self.start_live_view)
        self.stop_view_button.clicked.connect(self.stop_live_view)
        self.record_button.clicked.connect(self.save_current_frame)

        camera_controls.addWidget(self.connect_button)
        camera_controls.addWidget(self.start_view_button)
        camera_controls.addWidget(self.stop_view_button)
        camera_controls.addWidget(self.record_button)
        self.start_view_button.setEnabled(False)
        self.stop_view_button.setEnabled(False)
        self.record_button.setEnabled(False)
        left_column.addLayout(camera_controls)

        # Configuración de guardado
        save_layout = QHBoxLayout()
        save_layout.addWidget(QLabel("Carpeta:"))
        self.save_path_input = QLineEdit(r"C:\CapturasCamara")
        save_layout.addWidget(self.save_path_input)
        left_column.addLayout(save_layout)

        # Área de video
        self.video_label = QLabel("El video de la cámara aparecerá aquí")
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setStyleSheet("background-color: black; color: white; border: 1px solid gray;")
        self.video_label.setFixedSize(700, 500)
        self.video_label.setScaledContents(False)
        left_column.addWidget(self.video_label)

        # Fila inferior izquierda: Log y Parámetros
        camera_bottom = QHBoxLayout()
        
        # Log de estado
        log_container = QVBoxLayout()
        log_container.addWidget(QLabel("Log de Estado:"))
        self.status_log = QTextEdit()
        self.status_log.setReadOnly(True)
        self.status_log.setFixedHeight(150)
        self.status_log.setFixedWidth(350)
        log_container.addWidget(self.status_log)
        camera_bottom.addLayout(log_container)

        # Parámetros de cámara
        params_container = QVBoxLayout()
        params_container.addWidget(QLabel("Parámetros de Cámara:"))
        
        params_form = QFormLayout()
        
        # Exposición
        self.exposure_input = QLineEdit("0.01")
        self.exposure_input.setFixedWidth(80)
        exposure_layout = QHBoxLayout()
        exposure_layout.addWidget(self.exposure_input)
        self.apply_exposure_button = QPushButton("Aplicar")
        self.apply_exposure_button.setFixedWidth(70)
        self.apply_exposure_button.clicked.connect(self.apply_exposure)
        self.apply_exposure_button.setEnabled(False)
        exposure_layout.addWidget(self.apply_exposure_button)
        exposure_layout.addStretch()
        params_form.addRow("Exposición (s):", exposure_layout)
        
        # Trigger Mode
        self.trigger_mode_input = QLineEdit("int")
        self.trigger_mode_input.setFixedWidth(80)
        self.trigger_mode_input.setEnabled(False)
        params_form.addRow("Trigger:", self.trigger_mode_input)
        
        params_container.addLayout(params_form)
        params_container.addStretch()
        camera_bottom.addLayout(params_container)
        
        left_column.addLayout(camera_bottom)
        
        # ===== COLUMNA DERECHA: ARDUINO =====
        right_column = QVBoxLayout()
        right_column.addWidget(QLabel("<b>Control Arduino XYZ</b>"))
        
        # Potenciómetros
        right_column.addWidget(self.create_pots_group())
        
        # Sensores X, Y, Z
        right_column.addWidget(self.create_sensor_group('X'))
        right_column.addWidget(self.create_sensor_group('Y'))
        right_column.addWidget(self.create_sensor_group('Z'))
        
        # Terminal COM
        right_column.addWidget(QLabel("Terminal COM:"))
        self.com_terminal = QTextEdit()
        self.com_terminal.setReadOnly(True)
        self.com_terminal.setFixedHeight(200)
        self.com_terminal.setFixedWidth(350)
        self.com_terminal.setStyleSheet("background-color: #1E1E1E; color: #00FF00; font-family: Consolas;")
        right_column.addWidget(self.com_terminal)
        
        # Agregar columnas al layout principal
        main_layout.addLayout(left_column)
        main_layout.addLayout(right_column)
        
        # Iniciar hilo de Arduino
        self.serial_thread.start()

    def start_connection(self):
        if not self.worker_thread.isRunning():
            self.connect_button.setEnabled(False)
            self.connect_button.setText("Conectando...")
            self.worker_thread.start()

    def on_connection_result(self, success):
        if success:
            self.connect_button.setText("Conectado")
            self.start_view_button.setEnabled(True)
            self.apply_exposure_button.setEnabled(True)
        else:
            self.connect_button.setText("Reintentar Conexión")
            self.connect_button.setEnabled(True)

    def start_live_view(self):
        self.start_view_button.setEnabled(False)
        self.stop_view_button.setEnabled(True)
        self.record_button.setEnabled(True)
        try:
            self.camera_worker.exposure = float(self.exposure_input.text())
        except ValueError:
            self.log_status("Error: Valor de exposición inválido.")
            return

        # Emitir la señal para que el worker inicie la vista en su propio hilo
        self.start_live_view_signal.emit()

    def stop_live_view(self):
        self.camera_worker.stop_live_view()
        self.start_view_button.setEnabled(True)
        self.stop_view_button.setEnabled(False)
        self.record_button.setEnabled(False)
        self.video_label.setText("Vista en vivo detenida")  # Limpiar visor

    def update_video_frame(self, q_image):
        self.current_frame_pixmap = QPixmap.fromImage(q_image)
        # Escalar al tamaño fijo del label manteniendo aspect ratio
        scaled_pixmap = self.current_frame_pixmap.scaled(700, 500,
                                                          Qt.AspectRatioMode.KeepAspectRatio,
                                                          Qt.TransformationMode.SmoothTransformation)
        self.video_label.setPixmap(scaled_pixmap)

    def create_pots_group(self):
        """Crea el GroupBox para los potenciómetros."""
        group_box = QGroupBox("Potenciómetros")
        layout = QGridLayout()

        value_style = "font-size: 16px; font-weight: bold; color: #2E86C1;"

        layout.addWidget(QLabel("Valor A:"), 0, 0)
        self.arduino_labels['pot_a'] = QLabel("---")
        self.arduino_labels['pot_a'].setStyleSheet(value_style)
        layout.addWidget(self.arduino_labels['pot_a'], 0, 1)

        layout.addWidget(QLabel("Valor B:"), 1, 0)
        self.arduino_labels['pot_b'] = QLabel("---")
        self.arduino_labels['pot_b'].setStyleSheet(value_style)
        layout.addWidget(self.arduino_labels['pot_b'], 1, 1)

        group_box.setLayout(layout)
        return group_box

    def create_sensor_group(self, axis_name):
        """Crea un GroupBox para un sensor (X, Y, o Z)."""
        group_box = QGroupBox(f"Sensor Eje {axis_name}")
        layout = QGridLayout()

        raw_style = "font-size: 14px; color: #808080;"
        filtered_style = "font-size: 16px; font-weight: bold; color: #1E8449;"

        layout.addWidget(QLabel("Crudo:"), 0, 0)
        self.arduino_labels[f'raw_{axis_name.lower()}'] = QLabel("---")
        self.arduino_labels[f'raw_{axis_name.lower()}'].setStyleSheet(raw_style)
        layout.addWidget(self.arduino_labels[f'raw_{axis_name.lower()}'], 0, 1)

        layout.addWidget(QLabel("Filtrado:"), 1, 0)
        self.arduino_labels[f'filtered_{axis_name.lower()}'] = QLabel("---")
        self.arduino_labels[f'filtered_{axis_name.lower()}'].setStyleSheet(filtered_style)
        layout.addWidget(self.arduino_labels[f'filtered_{axis_name.lower()}'], 1, 1)

        group_box.setLayout(layout)
        return group_box

    def update_arduino_data(self, line):
        """Actualiza los datos de Arduino en la GUI y en el terminal COM."""
        # Mostrar en terminal COM
        self.com_terminal.append(line)
        
        # Manejar mensaje de error
        if line.startswith("ERROR:"):
            for label in self.arduino_labels.values():
                label.setText(line)
                label.setStyleSheet("color: red; font-size: 12px;")
            return

        # Parsear y actualizar etiquetas
        try:
            if line.startswith("Pots:"):
                parts = line.split(',')
                pot_a_val = parts[0].split('=')[1]
                pot_b_val = parts[1].split('=')[1]
                self.arduino_labels['pot_a'].setText(pot_a_val)
                self.arduino_labels['pot_b'].setText(pot_b_val)

            elif "->" in line:
                axis = line.strip().split(' ')[0]
                parts = line.split('|')
                raw_val = parts[0].split(':')[1].strip()
                filtered_val = parts[1].split(':')[1].strip()

                if axis == 'X':
                    self.arduino_labels['raw_x'].setText(raw_val)
                    self.arduino_labels['filtered_x'].setText(filtered_val)
                elif axis == 'Y':
                    self.arduino_labels['raw_y'].setText(raw_val)
                    self.arduino_labels['filtered_y'].setText(filtered_val)
                elif axis == 'Z':
                    self.arduino_labels['raw_z'].setText(raw_val)
                    self.arduino_labels['filtered_z'].setText(filtered_val)
        except IndexError:
            pass

    def apply_exposure(self):
        """Aplicar nuevo valor de exposición"""
        try:
            exposure_value = float(self.exposure_input.text())
            if exposure_value <= 0:
                self.log_status("Error: La exposición debe ser mayor que 0")
                return
            self.change_exposure_signal.emit(exposure_value)
        except ValueError:
            self.log_status("Error: Valor de exposición inválido")

    def save_current_frame(self):
        if self.current_frame_pixmap is None:
            self.log_status("No hay imagen para guardar.")
            return

        save_path = self.save_path_input.text()
        try:
            os.makedirs(save_path, exist_ok=True)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"record_{timestamp}.png"
            full_path = os.path.join(save_path, filename)

            # Guardar el QPixmap directamente
            self.current_frame_pixmap.save(full_path, "PNG")
            self.log_status(f"✅ Imagen guardada: {full_path}")
        except Exception as e:
            self.log_status(f"❌ Error al guardar la imagen: {e}")

    def log_status(self, message):
        self.status_log.append(message)

    def closeEvent(self, event):
        self.log_status("Cerrando aplicación...")
        # Detener cámara
        self.camera_worker.stop_live_view()
        self.worker_thread.quit()
        self.worker_thread.wait(3000)
        # Detener Arduino
        self.serial_thread.stop()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CameraControlGUI()
    window.show()
    sys.exit(app.exec())
import sys
import serial
import time
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QGridLayout, QLabel, QGroupBox
from PyQt6.QtCore import QThread, pyqtSignal, Qt

# --- CONFIGURACIÓN ---
# Reemplaza 'COM3' con el puerto serie de tu Arduino.
SERIAL_PORT = 'COM3'
BAUD_RATE = 1000000


# --------------------

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


class ArduinoGUI(QWidget):
    """
    Clase principal de la interfaz gráfica.
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle('Monitor de Sensores y Potenciómetros - Arduino')
        self.setGeometry(100, 100, 500, 400)

        # --- Layouts y Widgets ---
        self.main_layout = QVBoxLayout(self)
        self.value_labels = {}  # Diccionario para acceder fácilmente a las etiquetas de valor

        # Crear y añadir los grupos de visualización
        self.main_layout.addWidget(self.create_pots_group())
        self.main_layout.addWidget(self.create_sensor_group('X'))
        self.main_layout.addWidget(self.create_sensor_group('Y'))
        self.main_layout.addWidget(self.create_sensor_group('Z'))

        # --- Hilo de comunicación serie ---
        self.serial_thread = SerialReaderThread(SERIAL_PORT, BAUD_RATE)
        self.serial_thread.data_received.connect(self.update_data)
        self.serial_thread.start()

    def create_pots_group(self):
        """Crea el GroupBox para los potenciómetros."""
        group_box = QGroupBox("Potenciómetros")
        layout = QGridLayout()

        # Estilo para las etiquetas de valor
        value_style = "font-size: 18px; font-weight: bold; color: #2E86C1;"

        # Potenciómetro A
        layout.addWidget(QLabel("Valor A:"), 0, 0)
        self.value_labels['pot_a'] = QLabel("---")
        self.value_labels['pot_a'].setStyleSheet(value_style)
        layout.addWidget(self.value_labels['pot_a'], 0, 1)

        # Potenciómetro B
        layout.addWidget(QLabel("Valor B:"), 1, 0)
        self.value_labels['pot_b'] = QLabel("---")
        self.value_labels['pot_b'].setStyleSheet(value_style)
        layout.addWidget(self.value_labels['pot_b'], 1, 1)

        group_box.setLayout(layout)
        return group_box

    def create_sensor_group(self, axis_name):
        """Crea un GroupBox para un sensor (X, Y, o Z)."""
        group_box = QGroupBox(f"Sensor Eje {axis_name}")
        layout = QGridLayout()

        # Estilos para las etiquetas
        raw_style = "font-size: 16px; color: #808080;"
        filtered_style = "font-size: 18px; font-weight: bold; color: #1E8449;"

        # Valor Crudo
        layout.addWidget(QLabel("Crudo:"), 0, 0)
        self.value_labels[f'raw_{axis_name.lower()}'] = QLabel("---")
        self.value_labels[f'raw_{axis_name.lower()}'].setStyleSheet(raw_style)
        layout.addWidget(self.value_labels[f'raw_{axis_name.lower()}'], 0, 1)

        # Valor Filtrado
        layout.addWidget(QLabel("Filtrado:"), 1, 0)
        self.value_labels[f'filtered_{axis_name.lower()}'] = QLabel("---")
        self.value_labels[f'filtered_{axis_name.lower()}'].setStyleSheet(filtered_style)
        layout.addWidget(self.value_labels[f'filtered_{axis_name.lower()}'], 1, 1)

        group_box.setLayout(layout)
        return group_box

    def update_data(self, line):
        """
        Slot que recibe los datos del hilo y actualiza la GUI.
        """
        # Manejar mensaje de error del hilo
        if line.startswith("ERROR:"):
            # Mostrar error en todas las etiquetas para que sea visible
            for label in self.value_labels.values():
                label.setText(line)
                label.setStyleSheet("color: red; font-size: 12px;")
            return

        # Parsear la línea y actualizar la etiqueta correspondiente
        try:
            if line.startswith("Pots:"):
                parts = line.split(',')
                pot_a_val = parts[0].split('=')[1]
                pot_b_val = parts[1].split('=')[1]
                self.value_labels['pot_a'].setText(pot_a_val)
                self.value_labels['pot_b'].setText(pot_b_val)

            elif "->" in line:
                axis = line.strip().split(' ')[0]  # X, Y, o Z
                parts = line.split('|')
                raw_val = parts[0].split(':')[1].strip()
                filtered_val = parts[1].split(':')[1].strip()

                if axis == 'X':
                    self.value_labels['raw_x'].setText(raw_val)
                    self.value_labels['filtered_x'].setText(filtered_val)
                elif axis == 'Y':
                    self.value_labels['raw_y'].setText(raw_val)
                    self.value_labels['filtered_y'].setText(filtered_val)
                elif axis == 'Z':
                    self.value_labels['raw_z'].setText(raw_val)
                    self.value_labels['filtered_z'].setText(filtered_val)
        except IndexError:
            # Si una línea está incompleta, simplemente la ignoramos
            pass

    def closeEvent(self, event):
        """
        Asegurarse de que el hilo se detenga al cerrar la ventana.
        """
        print("Cerrando el hilo de comunicación...")
        self.serial_thread.stop()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ArduinoGUI()
    window.show()
    sys.exit(app.exec())
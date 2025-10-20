import sys
import serial
import time
from collections import deque

# Importar PyQTGraph
import pyqtgraph as pg

from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QGridLayout, 
                             QLabel, QGroupBox, QPushButton, QLineEdit, QCheckBox, QHBoxLayout)
from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtGui import QPalette, QColor # ¡NUEVO! Para el tema oscuro

# --- CONFIGURACIÓN ---
SERIAL_PORT = 'COM5'
BAUD_RATE = 1000000
PLOT_LENGTH = 200 

# --------------------

# --- ¡NUEVO! Hoja de estilos para el TEMA OSCURO ---
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
    margin-top: 1ex; /* Dejar espacio para el título */
    font-weight: bold;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top center;
    padding: 0 3px;
}
QLabel {
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
QLineEdit {
    background-color: #505050;
    border: 1px solid #606060;
    border-radius: 4px;
    padding: 3px;
}
QCheckBox {
    spacing: 5px;
}
QCheckBox::indicator {
    width: 13px;
    height: 13px;
}
"""

class SerialReaderThread(QThread):
    data_received = pyqtSignal(str)

    def __init__(self, port, baudrate):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)
            self.data_received.emit("INFO: Conectado exitosamente.")
            
            while self.running and self.ser.is_open:
                line = self.ser.readline()
                if line:
                    try:
                        decoded_line = line.decode('utf-8').strip()
                        if decoded_line:
                            self.data_received.emit(decoded_line)
                    except UnicodeDecodeError:
                        pass

            if self.ser and self.ser.is_open:
                self.ser.close()
        except serial.SerialException:
            self.data_received.emit(f"ERROR: Puerto {self.port} no encontrado.")

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.wait()

class ArduinoGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Panel de Control y Gráfico en Tiempo Real - Arduino')
        self.setGeometry(100, 100, 700, 800)

        # Aplicar el tema oscuro
        self.setStyleSheet(DARK_STYLESHEET)

        # --- Layouts y Widgets ---
        self.main_layout = QVBoxLayout(self)
        self.value_labels = {}
        
        self.main_layout.addWidget(self.create_control_group())
        self.main_layout.addWidget(self.create_motors_group())
        self.main_layout.addWidget(self.create_sensors_group())
        self.main_layout.addWidget(self.create_plot_group())

        self.init_plot_data()

        self.serial_thread = SerialReaderThread(SERIAL_PORT, BAUD_RATE)
        self.serial_thread.data_received.connect(self.update_data)
        self.serial_thread.start()

    def init_plot_data(self):
        """Prepara las variables y las líneas del gráfico."""
        self.data = {
            'power_a': deque([0] * PLOT_LENGTH, maxlen=PLOT_LENGTH),
            'power_b': deque([0] * PLOT_LENGTH, maxlen=PLOT_LENGTH),
            'sensor_1': deque([0] * PLOT_LENGTH, maxlen=PLOT_LENGTH),
            'sensor_2': deque([0] * PLOT_LENGTH, maxlen=PLOT_LENGTH),
        }
        # --- ¡NUEVO! Colores de alto contraste para el gráfico oscuro ---
        self.plot_lines = {
            'power_a': self.plot_widget.plot(pen=pg.mkPen('#00FFFF', width=2), name="Potencia A"), # Cyan
            'power_b': self.plot_widget.plot(pen=pg.mkPen('#FF00FF', width=2), name="Potencia B"), # Magenta
            'sensor_1': self.plot_widget.plot(pen=pg.mkPen('#FFFF00', width=2), name="Sensor 1"), # Amarillo
            'sensor_2': self.plot_widget.plot(pen=pg.mkPen('#00FF00', width=2), name="Sensor 2"), # Verde brillante
        }

    def create_plot_group(self):
        """Crea el GroupBox que contiene el gráfico y los checkboxes."""
        group_box = QGroupBox("Gráfico en Tiempo Real")
        layout = QVBoxLayout()
        
        # --- ¡MODIFICADO! Configuración del gráfico para tema oscuro ---
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#252525') # Fondo oscuro para el gráfico
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel('left', 'Valor (ADC)', color='#CCCCCC', size='12pt')
        self.plot_widget.setLabel('bottom', 'Muestras', color='#CCCCCC', size='12pt')
        
        # Cambiar color de los ejes
        styles = {'color':'#CCCCCC', 'font-size':'10pt'}
        self.plot_widget.getAxis('left').setTextPen(color='#CCCCCC')
        self.plot_widget.getAxis('bottom').setTextPen(color='#CCCCCC')

        legend = self.plot_widget.addLegend()
        legend.setLabelTextColor('#F0F0F0') # Color del texto de la leyenda

        self.plot_widget.setYRange(0, 1023, padding=0)
        
        layout.addWidget(self.plot_widget)

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
        group_box.setLayout(layout)
        return group_box

    def update_plot_visibility(self):
        """Muestra u oculta las líneas del gráfico según los checkboxes."""
        for key, cb in self.checkboxes.items():
            if cb.isChecked():
                self.plot_lines[key].show()
            else:
                self.plot_lines[key].hide()

    def update_data(self, line):
        if line.startswith("ERROR:") or line.startswith("INFO:"):
            print(line)
            return

        try:
            parts = line.split(',')
            if len(parts) == 4:
                pot_a, pot_b, sens_1, sens_2 = map(int, parts)
                
                self.value_labels['power_a'].setText(str(pot_a))
                self.value_labels['power_b'].setText(str(pot_b))
                self.value_labels['sensor_1'].setText(str(sens_1))
                self.value_labels['sensor_2'].setText(str(sens_2))

                self.data['power_a'].append(abs(pot_a))
                self.data['power_b'].append(abs(pot_b))
                self.data['sensor_1'].append(sens_1)
                self.data['sensor_2'].append(sens_2)

                self.plot_lines['power_a'].setData(list(self.data['power_a']))
                self.plot_lines['power_b'].setData(list(self.data['power_b']))
                self.plot_lines['sensor_1'].setData(list(self.data['sensor_1']))
                self.plot_lines['sensor_2'].setData(list(self.data['sensor_2']))

        except (IndexError, ValueError):
            pass
            
    # --- El resto de las funciones no cambian ---
    def create_control_group(self):
        group_box = QGroupBox("Panel de Control")
        layout = QGridLayout()

        layout.addWidget(QLabel("Modo Actual:"), 0, 0)
        self.value_labels['mode'] = QLabel("MANUAL")
        self.value_labels['mode'].setStyleSheet("font-weight: bold; color: #E67E22;") # Naranja
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

    def create_motors_group(self):
        group_box = QGroupBox("Estado de Motores")
        layout = QGridLayout()
        value_style = "font-size: 18px; font-weight: bold; color: #5DADE2;" # Azul claro

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
        value_style = "font-size: 18px; color: #58D68D;" # Verde claro

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
        
    def send_command(self, command):
        if self.serial_thread.ser and self.serial_thread.ser.is_open:
            full_command = command + '\n' 
            self.serial_thread.ser.write(full_command.encode('utf-8'))
            print(f"Comando enviado: {command}")
        else:
            print("Error: El puerto serie no está abierto.")

    def set_manual_mode(self):
        self.send_command('M')
        self.value_labels['mode'].setText("MANUAL")
        self.value_labels['mode'].setStyleSheet("font-weight: bold; color: #E67E22;")

    def set_auto_mode(self):
        self.send_command('A')
        self.value_labels['mode'].setText("AUTOMÁTICO")
        self.value_labels['mode'].setStyleSheet("font-weight: bold; color: #2ECC71;")
        
    def send_power_command(self):
        power_values = self.power_input.text()
        command_string = f"A,{power_values}"
        self.send_command(command_string)

    def closeEvent(self, event):
        print("Cerrando aplicación...")
        self.send_command('A,0,0')
        time.sleep(0.1)
        self.serial_thread.stop()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ArduinoGUI()
    window.show()
    sys.exit(app.exec())
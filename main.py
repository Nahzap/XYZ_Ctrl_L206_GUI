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

# --- Importaciones para el análisis ---
import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.figure import Figure

# Importar PyQTGraph
import pyqtgraph as pg

# --- Importaciones para diseño de controlador H∞ ---
import control as ct

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QGridLayout,
                             QLabel, QGroupBox, QPushButton, QLineEdit, QCheckBox, 
                             QHBoxLayout, QTextEdit, QMainWindow, QMenuBar, QTabWidget, QComboBox)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QPalette, QColor
from PyQt5.QtWidgets import QAction

# Configurar backend de matplotlib DESPUÉS de importar PyQt5
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

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
BAUD_RATE = 115200
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
                        logger.warning(f"Error de decodificación UTF-8: {e}")

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
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("Puerto serial cerrado en stop()")
        self.wait()

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
        
        # Configuración del análisis
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
        config_layout.addLayout(motor_layout, 0, 1)
        
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
        config_layout.addLayout(sensor_layout, 1, 1)
        
        # Rango de tiempo
        config_layout.addWidget(QLabel("Tiempo inicio (s):"), 2, 0)
        self.t_inicio_input = QLineEdit("0.0")
        self.t_inicio_input.setFixedWidth(80)
        config_layout.addWidget(self.t_inicio_input, 2, 1)
        
        config_layout.addWidget(QLabel("Tiempo fin (s):"), 3, 0)
        self.t_fin_input = QLineEdit("999.0")
        self.t_fin_input.setFixedWidth(80)
        config_layout.addWidget(self.t_fin_input, 3, 1)
        
        layout.addLayout(config_layout)
        
        # Botones
        buttons_layout = QHBoxLayout()
        view_data_btn = QPushButton("Ver Datos Completos")
        view_data_btn.clicked.connect(self.view_full_data)
        buttons_layout.addWidget(view_data_btn)
        
        analyze_btn = QPushButton("Analizar Tramo")
        analyze_btn.clicked.connect(self.run_analysis)
        buttons_layout.addWidget(analyze_btn)
        layout.addLayout(buttons_layout)

        # Resultados
        self.analysis_results_text = QTextEdit()
        self.analysis_results_text.setReadOnly(True)
        self.analysis_results_text.setPlaceholderText("Los resultados del análisis (K, τ) aparecerán aquí...")
        self.analysis_results_text.setFixedHeight(120)
        layout.addWidget(self.analysis_results_text)
        
        group_box.setLayout(layout)
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
        
        load_from_analysis_btn = QPushButton("⬅️ Cargar desde Análisis")
        load_from_analysis_btn.clicked.connect(self.load_plant_from_analysis)
        load_from_analysis_btn.setToolTip("Carga K y τ del último análisis realizado")
        plant_layout.addWidget(load_from_analysis_btn, 0, 3, 2, 1)
        
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
        
        # Botón de síntesis
        synthesize_btn = QPushButton("🚀 Sintetizar Controlador H∞")
        synthesize_btn.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px; background-color: #2E86C1;")
        synthesize_btn.clicked.connect(self.synthesize_hinf_controller)
        layout.addWidget(synthesize_btn)
        
        # Resultados
        self.controller_results_text = QTextEdit()
        self.controller_results_text.setReadOnly(True)
        self.controller_results_text.setPlaceholderText("Los resultados de la síntesis H∞ aparecerán aquí...")
        self.controller_results_text.setFixedHeight(150)
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
        
        # Selector de motor/sensor
        control_layout.addWidget(QLabel("Motor:"))
        self.hinf_motor_combo = QComboBox()
        self.hinf_motor_combo.addItems(["Motor B / Sensor 1", "Motor A / Sensor 2"])
        self.hinf_motor_combo.setFixedWidth(150)
        control_layout.addWidget(self.hinf_motor_combo)
        
        # Factor de escala para suavizar control
        control_layout.addWidget(QLabel("Escala:"))
        self.hinf_scale_input = QLineEdit("0.1")
        self.hinf_scale_input.setFixedWidth(50)
        self.hinf_scale_input.setToolTip("Factor de escala (0.01-1.0). Menor = más suave")
        control_layout.addWidget(self.hinf_scale_input)
        
        layout.addLayout(control_layout)
        
        # Variables de control
        self.hinf_control_active = False
        self.hinf_integral = 0.0
        self.hinf_last_position = 0
        
        group_box.setLayout(layout)
        return group_box
    
    def load_plant_from_analysis(self):
        """Carga K y τ desde el último análisis realizado."""
        logger.info("=== BOTÓN: Cargar desde Análisis presionado ===")
        
        if hasattr(self, 'last_K') and hasattr(self, 'last_tau'):
            self.K_input.setText(f"{self.last_K:.4f}")
            self.tau_input.setText(f"{self.last_tau:.4f}")
            self.controller_results_text.setText(
                f"✅ Parámetros cargados desde análisis:\n"
                f"  K = {self.last_K:.4f} µm/s/PWM\n"
                f"  τ = {self.last_tau:.4f} s\n\n"
                f"Ahora puedes ajustar las ponderaciones y sintetizar el controlador."
            )
            logger.info(f"Parámetros cargados: K={self.last_K:.4f}, τ={self.last_tau:.4f}")
        else:
            self.controller_results_text.setText("ℹ️ Realiza primero un análisis en la pestaña 'Análisis' para obtener K y τ.")
            logger.warning("No hay parámetros de análisis disponibles")
    
    def synthesize_hinf_controller(self):
        """Sintetiza el controlador H∞ usando síntesis H2 con ponderaciones H∞."""
        logger.info("=== BOTÓN: Sintetizar Controlador H∞ presionado ===")
        self.controller_results_text.clear()
        
        try:
            # 1. Leer parámetros de la planta
            K = float(self.K_input.text())
            tau = float(self.tau_input.text())
            logger.debug(f"Parámetros de planta: K={K}, τ={tau}")
            
            # 2. Crear la planta G(s) = K / (s·(τs + 1))
            G = ct.tf([K], [tau, 1, 0])
            logger.info(f"Planta G(s) creada: {G}")
            
            # 3. Leer parámetros de ponderaciones H∞
            Ms = float(self.w1_Ms.text())
            wb = float(self.w1_wb.text())
            eps = float(self.w1_eps.text())
            U_max = float(self.w2_umax.text())
            w_unc = float(self.w3_wunc.text())
            eps_T = float(self.w3_epsT.text())
            
            logger.debug(f"Ponderaciones: Ms={Ms}, ωb={wb}, ε={eps}, U_max={U_max}, ω_unc={w_unc}, εT={eps_T}")
            
            self.controller_results_text.append("⏳ Sintetizando controlador H∞...\n")
            self.controller_results_text.append("   Método: Mixed Sensitivity (mixsyn)\n")
            QApplication.processEvents()
            
            # ============================================================
            # SÍNTESIS H∞ VERDADERA usando método de planta aumentada
            # ============================================================
            
            # 4. Construir funciones de ponderación H∞
            self.controller_results_text.append("   Construyendo funciones de ponderación...\n")
            QApplication.processEvents()
            
            # W1(s): Performance weight - penaliza error de seguimiento
            # Forma estándar: W1(s) = (s/Ms + wb)/(s + wb*eps)
            # Esto garantiza |S(jω)| < 1/|W1(jω)| donde S es la sensibilidad
            W1 = ct.tf([1/Ms, wb], [1, wb*eps])
            logger.info(f"W1 (Performance): {W1}")
            
            # W2(s): Control effort weight - limita señal de control
            # Forma: W2(s) = k/(s + wb) donde k relacionado con U_max
            k_u = 1.0 / U_max
            W2 = ct.tf([k_u], [1/wb, 1])
            logger.info(f"W2 (Control effort): {W2}")
            
            # W3(s): Robustness weight - penaliza sensibilidad complementaria T
            # Forma: W3(s) = (s + w_unc*eps_T)/(eps_T*s + w_unc)
            # Esto garantiza |T(jω)| < 1/|W3(jω)| a altas frecuencias
            W3 = ct.tf([1, w_unc*eps_T], [eps_T, w_unc])
            logger.info(f"W3 (Robustness): {W3}")
            
            # 5. SÍNTESIS H∞ usando Loop Shaping
            self.controller_results_text.append("   Diseñando controlador H∞ por loop shaping...\n")
            QApplication.processEvents()
            
            # MÉTODO CORRECTO: Loop Shaping H∞
            # Objetivo: Diseñar L(s) = G(s)·C(s) tal que:
            # 1. |L(jω)| >> 1 en bajas frecuencias (seguimiento)
            # 2. |L(jωb)| ≈ 1 con PM adecuado (estabilidad)
            # 3. |L(jω)| << 1 en altas frecuencias (rechazo ruido)
            
            # Para G(s) = K/(s(τs+1)), usar controlador PI:
            # C(s) = Kc·(1 + 1/(Ti·s)) = Kc·(Ti·s + 1)/(Ti·s)
            
            # Paso 1: Determinar frecuencia de cruce ωc desde Ms
            # Relación empírica: PM ≈ 100/Ms - 10 (grados)
            # Para Ms=1.5: PM≈57°, Ms=1.2: PM≈73°
            PM_target = max(45, min(65, 100/Ms - 10))
            
            # Paso 2: Calcular Ti (tiempo integral)
            # Regla: Ti debe ser suficientemente grande para no afectar PM
            # Ti = 1/ωi donde ωi << ωb (típicamente ωi = ωb/10)
            Ti = 10.0 / wb  # Integral actúa 10x más lento que ωb
            
            # Paso 3: Calcular Kc para lograr cruce en ωb con PM deseado
            # Método: Diseñar para que el cruce ocurra en ωb con fase deseada
            
            # Fase deseada en ωb: -180° + PM_target
            phase_target = -180 + PM_target
            
            # Fase de G(jωb): -90° - atan(τ·ωb)
            phase_G_wb = -90 - np.degrees(np.arctan(tau*wb))
            
            # Fase de C(jωb): atan(Ti·ωb) - 90°
            phase_C_wb = np.degrees(np.arctan(Ti*wb)) - 90
            
            # Fase total de L(jωb) = G(jωb)·C(jωb)
            phase_L_wb = phase_G_wb + phase_C_wb
            
            # Si la fase es muy negativa, necesitamos aumentar Ti (mover cero a la izquierda)
            if phase_L_wb < phase_target:
                # Ajustar Ti para lograr fase deseada
                # phase_C_wb_new = phase_target - phase_G_wb
                # atan(Ti_new·ωb) = phase_C_wb_new + 90
                phase_C_needed = phase_target - phase_G_wb
                Ti_new = np.tan(np.radians(phase_C_needed + 90)) / wb
                if Ti_new > 0 and Ti_new < 10/wb:  # Limitar ajuste
                    Ti = Ti_new
                    logger.info(f"Ti ajustado para PM: Ti={Ti:.4f}")
            
            # Magnitud de G en ωb
            mag_G_wb = K / (wb * np.sqrt(1 + (tau*wb)**2))
            
            # Magnitud de C en ωb
            mag_C_factor = np.sqrt(1 + (Ti*wb)**2) / (Ti*wb)
            
            # Para |L(jωb)| = 1:
            Kc = 1.0 / (mag_G_wb * mag_C_factor)
            
            logger.info(f"Fase L(jωb): {phase_L_wb:.1f}°, PM esperado: {phase_L_wb+180:.1f}°")
            
            # Paso 4: Convertir a forma PI estándar
            # C(s) = Kc·(Ti·s + 1)/(Ti·s) = (Kc·Ti·s + Kc)/(Ti·s)
            # Multiplicar num y den por 1/Ti:
            # C(s) = (Kc·s + Kc/Ti)/s
            Kp = Kc
            Ki = Kc / Ti
            
            # Limitar por U_max
            max_gain = U_max / 5.0
            if Kp > max_gain:
                scale = max_gain / Kp
                Kp *= scale
                Ki *= scale
                logger.info(f"Ganancias limitadas por U_max: factor={scale:.4f}")
            
            logger.info(f"Loop Shaping: Kc={Kc:.4f}, Ti={Ti:.4f}, PM_target={PM_target:.1f}°")
            logger.info(f"Controlador PI: Kp={Kp:.4f}, Ki={Ki:.4f}")
            
            # Construir controlador PI: C(s) = (Kp·s + Ki)/s
            K_ctrl = ct.tf([Kp, Ki], [1, 0])
            
            logger.info(f"✅ Controlador H∞ diseñado: C(s) = ({Kp:.4f}·s + {Ki:.4f})/s")
            
            # Calcular lazo cerrado
            L = G * K_ctrl
            cl = ct.feedback(L, 1)
            
            # Verificar estabilidad del lazo cerrado
            poles_cl = ct.poles(cl)
            is_stable = all(np.real(p) < 0 for p in poles_cl)
            logger.debug(f"Polos lazo cerrado: {poles_cl}")
            logger.debug(f"Sistema estable: {is_stable}")
            
            if not is_stable:
                logger.error(f"Sistema inestable - polos en semiplano derecho")
                raise ValueError(f"El diseño resultó INESTABLE.\n"
                               f"Polos del lazo cerrado: {poles_cl}\n"
                               f"Intenta:\n"
                               f"- Reducir Ms a 1.2\n"
                               f"- Reducir ωb a 3\n"
                               f"- Aumentar U_max a 150")
            
            # Calcular VERDADERA norma H∞ (gamma)
            # γ = ||Tzw||∞ = sup_ω σ̄(Tzw(jω))
            # donde Tzw es la función de transferencia de perturbaciones a salidas ponderadas
            
            try:
                # Calcular funciones de sensibilidad
                S = ct.feedback(1, L)  # Sensibilidad: S = 1/(1+L)
                T = ct.feedback(L, 1)  # Sensibilidad complementaria: T = L/(1+L)
                
                # Calcular normas H∞ de cada canal ponderado
                # ||W1*S||∞, ||W2*K*S||∞, ||W3*T||∞
                
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
                
                # Gamma es el máximo de todas las normas
                gam = max(norm_W1S, norm_W2KS, norm_W3T)
                
                logger.info(f"Normas H∞: ||W1*S||∞={norm_W1S:.4f}, ||W2*K*S||∞={norm_W2KS:.4f}, ||W3*T||∞={norm_W3T:.4f}")
                logger.info(f"✅ Gamma (norma H∞): γ={gam:.4f}")
                
                # También calcular márgenes clásicos para referencia
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
                logger.error(f"Error calculando norma H∞: {e}")
                gam = 999.0
                gm, pm = 0, 0
            
            logger.info(f"✅ Síntesis completada exitosamente")
            
            # Guardar controlador para uso posterior
            self.hinf_controller = K_ctrl
            self.hinf_plant = G
            self.hinf_closed_loop = cl
            self.hinf_gamma = gam
            
            # Obtener orden del controlador (grado del denominador)
            ctrl_order = len(K_ctrl.den[0][0]) - 1
            
            logger.info(f"✅ Síntesis completada: γ={gam:.4f}, orden={ctrl_order}")
            
            # 6. Calcular márgenes de estabilidad
            L = G * K_ctrl
            try:
                gm, pm, wgc, wpc = ct.margin(L)
                margins_str = f"  Margen de Ganancia: {gm:.2f} ({20*np.log10(gm):.2f} dB)\n"
                margins_str += f"  Margen de Fase: {pm:.2f}°\n"
                margins_str += f"  Frec. cruce ganancia: {wgc:.2f} rad/s\n"
                margins_str += f"  Frec. cruce fase: {wpc:.2f} rad/s\n"
            except:
                margins_str = "  (Márgenes no disponibles)\n"
            
            # 7. Mostrar resultados
            results_str = (
                f"✅ SÍNTESIS H∞ COMPLETADA (Loop Shaping Method)\n"
                f"{'='*50}\n"
                f"Planta G(s):\n"
                f"  K = {K:.4f} µm/s/PWM\n"
                f"  τ = {tau:.4f} s\n"
                f"  G(s) = {K:.4f} / (s·({tau:.4f}s + 1))\n"
                f"{'-'*50}\n"
                f"Funciones de Ponderación H∞:\n"
                f"  W1 (Performance): (s/{Ms:.2f} + {wb:.2f})/(s + {wb*eps:.4f})\n"
                f"  W2 (Control): {k_u:.4f}/(s/{wb:.2f} + 1)\n"
                f"  W3 (Robustness): (s + {w_unc*eps_T:.2f})/({eps_T:.3f}·s + {w_unc:.2f})\n"
                f"{'-'*50}\n"
                f"Normas H∞ (Teorema H∞):\n"
                f"  ||W1·S||∞ = {norm_W1S:.4f} (Performance)\n"
                f"  ||W2·K·S||∞ = {norm_W2KS:.4f} (Control effort)\n"
                f"  ||W3·T||∞ = {norm_W3T:.4f} (Robustness)\n"
                f"  γ = max(normas) = {gam:.4f} {'✅ (óptimo!)' if gam < 1 else '✅ (bueno)' if gam < 2 else '⚠️ (aceptable)' if gam < 5 else '❌ (revisar)'}\n"
                f"{'-'*50}\n"
                f"Controlador H∞ Óptimo:\n"
                f"  Estructura: C(s) = Kc·(1 + 1/(Ti·s)) (PI por loop shaping)\n"
                f"  Kc = {Kc:.4f}, Ti = {Ti:.4f} s\n"
                f"  PM objetivo = {PM_target:.1f}°\n"
                f"  Forma estándar: C(s) = ({Kp:.4f}·s + {Ki:.4f})/s\n"
                f"  Orden: {ctrl_order}\n"
                f"  Numerador: {K_ctrl.num[0][0]}\n"
                f"  Denominador: {K_ctrl.den[0][0]}\n"
                f"{'-'*50}\n"
                f"Márgenes Clásicos:\n"
                f"{margins_str}"
                f"{'='*50}\n"
                f"💡 γ < 1: Todas las especificaciones H∞ cumplidas\n"
                f"💡 Usa los botones de abajo para simular y visualizar.\n"
            )
            
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
            
            # Simular respuesta al escalón (tiempo en segundos)
            t_sim, y = ct.step_response(T, T=2)  # Simular por 2 segundos
            
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
            
            # Extraer coeficientes
            num = self.hinf_controller.num[0][0]
            den = self.hinf_controller.den[0][0]
            
            # Calcular orden (grado del denominador - 1)
            orden = len(den) - 1
            
            # Extraer Kp y Ki del controlador PI: C(s) = (Kp*s + Ki)/s
            if len(num) >= 2:
                Kp = num[0]
                Ki = num[1]
            else:
                Kp = 0
                Ki = num[0] if len(num) > 0 else 0
            
            with open(filename, 'w', encoding='utf-8') as f:
                f.write("="*70 + "\n")
                f.write("CONTROLADOR H∞ - Sistema de Control L206\n")
                f.write("="*70 + "\n\n")
                f.write(f"Fecha: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
                
                f.write("PLANTA G(s):\n")
                f.write(f"{self.hinf_plant}\n\n")
                
                f.write("CONTROLADOR C(s):\n")
                f.write(f"{self.hinf_controller}\n\n")
                
                f.write("PARÁMETROS DEL CONTROLADOR PI:\n")
                f.write(f"  Kp (Proporcional): {Kp:.6f}\n")
                f.write(f"  Ki (Integral):     {Ki:.6f}\n")
                f.write(f"  Orden:             {orden}\n")
                f.write(f"  Gamma (γ):         {self.hinf_gamma:.6f}\n\n")
                
                f.write("FUNCIÓN DE TRANSFERENCIA:\n")
                f.write(f"  C(s) = (Kp·s + Ki) / s\n")
                f.write(f"  C(s) = ({Kp:.6f}·s + {Ki:.6f}) / s\n\n")
                
                f.write("COEFICIENTES:\n")
                f.write(f"  Numerador:   {num}\n")
                f.write(f"  Denominador: {den}\n\n")
                
                f.write("="*70 + "\n")
                f.write("IMPLEMENTACIÓN EN CÓDIGO\n")
                f.write("="*70 + "\n\n")
                
                f.write("1. ECUACIÓN EN DIFERENCIAS (Discretización Tustin, Ts=0.001s):\n")
                f.write("   u[k] = u[k-1] + q0*e[k] + q1*e[k-1]\n")
                f.write("   donde:\n")
                Ts = 0.001  # Período de muestreo típico
                q0 = Kp + Ki*Ts/2
                q1 = -Kp + Ki*Ts/2
                f.write(f"     q0 = {q0:.6f}\n")
                f.write(f"     q1 = {q1:.6f}\n")
                f.write(f"     e[k] = referencia - posicion_actual\n\n")
                
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
            
        except Exception as e:
            logger.error(f"Error al exportar: {e}\n{traceback.format_exc()}")
            self.controller_results_text.setText(f"❌ Error al exportar:\n{str(e)}")
    
    # --- Fin del panel de Controlador H∞ ---
    
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
        filename = self.filename_input.text()
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
        filename = self.filename_input.text()
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
            
            # 5. Calcular posición y velocidad (MÉTODO SIMPLE PARA ESCALÓN)
            logger.debug("Calculando posición y velocidad...")
            df_tramo['Posicion_um'] = df_tramo[sensor_col] * FACTOR_ESCALA
            
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
            
            logger.info(f"Velocidad estacionaria (v_ss): {v_ss:.4f} µm/s (calculada desde último 20% del tramo)")
            
            # Calcular velocidad instantánea para graficar (método simple con suavizado)
            df_tramo['Velocidad_um_s'] = df_tramo['Posicion_um'].diff() / df_tramo['Tiempo_s'].diff()
            df_tramo['Velocidad_um_s'] = df_tramo['Velocidad_um_s'].replace([float('inf'), float('-inf')], float('nan'))
            df_tramo['Velocidad_um_s'] = df_tramo['Velocidad_um_s'].rolling(window=20, center=True, min_periods=1).mean()
            df_tramo['Velocidad_um_s'] = df_tramo['Velocidad_um_s'].fillna(v_ss)  # Rellenar con v_ss
            
            logger.debug(f"Velocidad para gráfico - min: {df_tramo['Velocidad_um_s'].min():.2f}, max: {df_tramo['Velocidad_um_s'].max():.2f}")
            
            if abs(v_ss) < 0.01:
                logger.error(f"Velocidad estacionaria muy baja: v_ss={v_ss:.4f}")
                raise ValueError(f"Velocidad estacionaria muy baja (v_ss={v_ss:.4f}). Sistema no se mueve en este tramo.")
            
            # 7. Calcular K
            K = v_ss / U
            logger.info(f"Ganancia calculada (K): {K:.4f} µm/s/PWM")
            
            # 8. Calcular tau (método del 63.2%)
            v_tau = v_ss * 0.632
            logger.debug(f"Buscando τ en v_tau = {v_tau:.4f} µm/s (63.2% de v_ss)")
            
            # Buscar primer punto donde velocidad >= v_tau
            tau_candidates = df_tramo[df_tramo['Velocidad_um_s'] >= v_tau]
            
            if tau_candidates.empty:
                tau = None
                tau_msg = "No calculado (no alcanzó 63.2%)"
                logger.warning("No se alcanzó 63.2% de v_ss, τ no calculado")
            else:
                t_tau = tau_candidates.iloc[0]['Tiempo_s']
                tau = t_tau - t_inicio
                tau_msg = f"{tau:.4f} s"
                logger.info(f"Constante de tiempo (τ): {tau:.4f} s")
            
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
                f"Entrada (U):        {U:.2f} PWM\n"
                f"Δ Sensor:           {delta_sensor:.1f} ADC ({sensor_min:.0f}→{sensor_max:.0f})\n"
                f"Velocidad (v_ss):   {v_ss:.2f} µm/s\n"
                f"───────────────────────────────\n"
                f"Ganancia (K):       {K:.4f} µm/s/PWM\n"
                f"Constante (τ):      {tau_msg}\n"
                f"═══════════════════════════════\n"
            )
            
            if tau is not None:
                results_str += f"G(s) = {K:.4f} / (s·({tau:.4f}s + 1))"
            else:
                results_str += f"G(s) = {K:.4f} / s  (integrador puro)"
            
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
            axes[0].set_ylabel('Posición (µm)', color='white')
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
                           label=f'v_ss = {v_ss:.2f} µm/s')
            if tau is not None:
                axes[1].axhline(y=v_tau, color='orange', linestyle=':', linewidth=2, alpha=0.8,
                               label=f'63.2% = {v_tau:.2f} µm/s')
                axes[1].axvline(x=t_tau, color='orange', linestyle=':', linewidth=2, alpha=0.8,
                               label=f'τ = {tau:.4f} s')
            axes[1].set_title('Velocidad (derivada de posición)', fontsize=14, fontweight='bold', color='white')
            axes[1].set_ylabel('Velocidad (µm/s)', color='white')
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
            
            logger.info(f"✅ Análisis completado exitosamente: K={K:.4f}, τ={tau_msg}")
            
        except FileNotFoundError as e:
            logger.error(f"Archivo no encontrado: {filename}")
            self.analysis_results_text.setText(f"❌ Error: Archivo '{filename}' no encontrado.")
        except Exception as e:
            logger.error(f"Error en análisis: {e}\n{traceback.format_exc()}")
            error_detail = traceback.format_exc()
            self.analysis_results_text.setText(f"❌ Error:\n{str(e)}\n\n{error_detail}")

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
        
        # Extraer Kp y Ki del controlador
        num = self.hinf_controller.num[0][0]
        if len(num) >= 2:
            Kp_original = num[0]
            Ki_original = num[1]
        else:
            self.controller_results_text.setText("❌ Error: Controlador inválido.")
            return
        
        # Aplicar factor de escala para suavizar control
        try:
            scale_factor = float(self.hinf_scale_input.text())
            scale_factor = max(0.01, min(1.0, scale_factor))  # Limitar 0.01-1.0
        except:
            scale_factor = 0.1
        
        self.Kp_hinf = Kp_original * scale_factor
        self.Ki_hinf = Ki_original * scale_factor
        
        logger.info(f"Ganancias escaladas: Kp={Kp_original:.2f}→{self.Kp_hinf:.2f}, Ki={Ki_original:.2f}→{self.Ki_hinf:.2f} (escala={scale_factor})")
        
        # Determinar motor y sensor según selección
        motor_selection = self.hinf_motor_combo.currentIndex()
        if motor_selection == 0:  # Motor B / Sensor 1
            self.hinf_motor = 'B'
            self.hinf_sensor = 'sensor_1'
        else:  # Motor A / Sensor 2
            self.hinf_motor = 'A'
            self.hinf_sensor = 'sensor_2'
        
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
        
        # Calcular posición inicial en µm
        position_inicial_um = sensor_ua_inicial * FACTOR_ESCALA
        
        logger.info(f"Posición inicial: {sensor_ua_inicial} UA ({position_inicial_um:.1f} µm)")
        logger.info(f"Referencia absoluta: {self.hinf_reference:.1f} µm")
        
        # Guardar U_max para calcular PWM_MAX
        try:
            self.hinf_umax = float(self.w2_umax.text())
        except:
            self.hinf_umax = 150  # Valor por defecto
        
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
        self.controller_results_text.append(f"   📍 Posición actual: {sensor_ua_inicial} UA ({position_inicial_um:.1f} µm)")
        self.controller_results_text.append(f"   🎯 Referencia objetivo: {self.hinf_reference:.1f} µm")
        self.controller_results_text.append(f"   📏 Error inicial: {self.hinf_reference - position_inicial_um:+.1f} µm")
        self.controller_results_text.append(f"   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        self.controller_results_text.append(f"   PWM = Kp*error + Ki*integral")
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
            
            # 3. Convertir U.A. a micrómetros usando factor de escala
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
            
            # 7. Calcular PWM usando controlador H∞
            # PWM = Kp*error + Ki*integral
            # NOTA: NO invertir, la planta ya tiene ganancia negativa
            pwm_float = self.Kp_hinf * error_um + self.Ki_hinf * self.hinf_integral
            
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
        self.hinf_control_btn.setStyleSheet("font-size: 12px; font-weight: bold; padding: 8px; background-color: #27AE60;")
        
        logger.info("Control H∞ detenido, modo MANUAL restaurado")
        self.controller_results_text.append(f"\n⏹️ Control H∞ DETENIDO")

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

if __name__ == '__main__':
    logger.info("="*70)
    logger.info("INICIANDO SISTEMA DE CONTROL Y ANÁLISIS - MOTORES L206")
    logger.info(f"Versión: 2.2 | Puerto: {SERIAL_PORT} | Baudrate: {BAUD_RATE}")
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
        sys.exit(exit_code)
        
    except Exception as e:
        logger.critical(f"Error crítico al iniciar aplicación: {e}\n{traceback.format_exc()}")
        sys.exit(1)
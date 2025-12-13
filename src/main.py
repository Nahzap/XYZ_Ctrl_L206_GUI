# CRITICAL: Set OpenMP environment variable BEFORE any imports
# This fixes the conflict between PyTorch (libiomp5md.dll) and SciPy (libomp.dll)
import os
os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'
os.environ['MKL_THREADING_LAYER'] = 'GNU'

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
# --- IMPORTACIONES DE MÓDULOS PROPIOS (Fases 1-6) ---
# =========================================================================
# Fase 1: Configuración
from config.constants import *
from config.settings import setup_logging

# Fase 2: Estilos
from gui.styles.dark_theme import DARK_STYLESHEET

# Fase 3: Comunicación Serial
from core.communication.serial_handler import SerialHandler
from core.communication.protocol import MotorProtocol

# Fase 4: Ventanas Auxiliares
from gui.windows import MatplotlibWindow, SignalWindow, CameraViewWindow

# Fase 5: Hardware - Cámara
from hardware.camera import CameraWorker

# Fase 6: Grabación de Datos
from data import DataRecorder

# Fase 7: Análisis de Transferencia
from core.analysis import TransferFunctionAnalyzer

# Fase 8: Controladores
from core.controllers import HInfController

# Fase 9: Trayectorias
from core.trajectory import TrajectoryGenerator

# Fase 10: Pestañas GUI (Tabs modulares) - Integrado en Fase 12
from gui.tabs import (ControlTab, RecordingTab, AnalysisTab, 
                      CameraTab, TestTab, HInfTab, ImgAnalysisTab)

# Fase 11: Detección U2-Net (Singleton - carga única)
from core.detection import U2NetDetector

# Fase 12: Servicios Asíncronos
from core.services import DetectionService, AutofocusService

# =========================================================================
# --- INICIALIZAR SISTEMA DE LOGGING ---
# =========================================================================
logger = setup_logging()

# =========================================================================
# --- Procesamiento de Datos en Tiempo Real (Sin Buffers Intermedios) ---
# =========================================================================
# NOTA: Se eliminó OptimizedSignalBuffer para máxima responsividad
# Los datos fluyen directamente: Serial -> update_data() -> SignalWindow
# Esto asegura latencia mínima y visualización en tiempo real

# =========================================================================
# --- Importaciones adicionales para funcionalidades específicas ---
# =========================================================================

# --- Diseño de controlador H∞ ---
import control as ct

# --- Cámara Thorlabs ---
try:
    import pylablib as pll
    # Configurar la ruta del SDK de Thorlabs
    pll.par["devices/dlls/thorlabs_tlcam"] = r"C:\Program Files\Thorlabs\ThorImageCAM\Bin"
    from pylablib.devices import Thorlabs
    THORLABS_AVAILABLE = True
except ImportError:
    THORLABS_AVAILABLE = False
    logger.warning("pylablib no está instalado. Funcionalidad de cámara deshabilitada.")
except Exception as e:
    THORLABS_AVAILABLE = False
    logger.warning(f"Error al configurar Thorlabs SDK: {e}")

# =========================================================================
# --- Las clases MatplotlibWindow, SignalWindow, CameraWorker y 
# --- CameraViewWindow ahora están en sus módulos correspondientes
# =========================================================================



# =========================================================================
# --- Interfaz Principal con Pestañas ---
# =========================================================================
class ArduinoGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Sistema de Control y Análisis - Motores L206')
        self.setGeometry(100, 100, 800, 700)
        self.setStyleSheet(DARK_STYLESHEET)

        # Inicializar grabador de datos (Fase 6)
        self.data_recorder = DataRecorder()
        
        # Inicializar analizador de transferencia (Fase 7)
        self.tf_analyzer = TransferFunctionAnalyzer()
        
        # Inicializar controlador H∞ (Fase 8)
        self.hinf_designer = HInfController()
        
        # Inicializar generador de trayectorias (Fase 9)
        self.trajectory_gen = TrajectoryGenerator()
        
        self.value_labels = {}
        self.identified_transfer_functions = []  # Mantenido para compatibilidad
        self.current_trajectory = None  # Para compatibilidad con código existente
        
        # Variables de control H∞ en tiempo real
        self.hinf_control_active = False
        self.hinf_integral = 0.0
        self.hinf_last_position = 0
        
        # Controladores transferidos a TestTab
        self.test_controller_a = None
        self.test_controller_b = None
        
        # Ventanas de visualización (inicialmente None)
        self.signal_window = None
        self.data_window = None
        self.analysis_window = None
        
        # Inicializar detector U2-Net (Singleton - carga única al inicio)
        logger.info("Inicializando detector U2-Net...")
        self.u2net_detector = U2NetDetector.get_instance()
        
        # Inicializar servicios de detección y autofoco
        self.detection_service = DetectionService()
        self.autofocus_service = AutofocusService()
        
        # Iniciar comunicación serial ANTES de crear tabs (necesario para ControlTab)
        self.serial_thread = SerialHandler(SERIAL_PORT, BAUD_RATE)
        
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
        
        # Pestaña 1: Control y Monitoreo (usando ControlTab modular - Fase 12)
        self.control_tab = ControlTab(serial_handler=self.serial_thread, parent=self)
        # Conectar señal de reconexión serial
        self.control_tab.serial_reconnect_requested.connect(self._on_serial_reconnect)
        # --- NUEVAS CONEXIONES PARA POSITION HOLD ---
        self.control_tab.position_hold_requested.connect(self._on_position_hold)
        self.control_tab.brake_requested.connect(self._on_brake)
        self.control_tab.settling_config_requested.connect(self._on_settling_config)
        self.tabs.addTab(self.control_tab, "🎮 Control")
        
        # Pestaña 2: Grabación (usando RecordingTab modular - Fase 12)
        self.recording_tab = RecordingTab(data_recorder=self.data_recorder, parent=self)
        # Conectar señales de RecordingTab
        self.recording_tab.recording_started.connect(self._on_recording_started)
        self.recording_tab.recording_stopped.connect(self._on_recording_stopped)
        self.tabs.addTab(self.recording_tab, "📹 Grabación")
        
        # Pestaña 3: Análisis (AnalysisTab modular)
        self.analysis_tab = AnalysisTab(tf_analyzer=self.tf_analyzer, parent=self)
        self.analysis_tab.analysis_completed.connect(self._on_analysis_completed)
        self.analysis_tab.show_plot_requested.connect(self._on_show_plot)
        self.tabs.addTab(self.analysis_tab, "📈 Análisis")
        
        # Pestaña 5: Prueba (TestTab modular) - CREAR ANTES para poder referenciarla
        self.test_tab = TestTab(trajectory_generator=self.trajectory_gen, parent=self)
        # Configurar callbacks de hardware para control en tiempo real
        self.test_tab.set_hardware_callbacks(
            send_command=self.send_command,
            get_sensor_value=lambda key: int(self.control_tab.value_labels[key].text()) if key in self.control_tab.value_labels else None,
            get_mode_label=lambda: self.control_tab.value_labels.get('mode', None)
        )
        # TestTab maneja sus operaciones internamente
        self.test_tab.controller_clear_requested.connect(lambda motor: self.test_tab.clear_controller(motor))
        
        # Pestaña 4: H∞ Synthesis (HInfTab modular) - CREAR DESPUÉS de TestTab
        self.hinf_tab = HInfTab(hinf_controller=self.hinf_designer, tf_analyzer=self.tf_analyzer, parent=self)
        # Configurar callbacks de hardware para control en tiempo real
        self.hinf_tab.set_hardware_callbacks(
            send_command=self.send_command,
            get_sensor_value=lambda key: int(self.control_tab.value_labels[key].text()) if key in self.control_tab.value_labels else None,
            get_mode_label=lambda: self.control_tab.value_labels.get('mode', None)
        )
        # Configurar referencia a TestTab para transferencias
        self.hinf_tab.set_test_tab_reference(self.test_tab)
        # HInfTab llama directamente a sus métodos internos
        self.hinf_tab.control_toggle_requested.connect(lambda: self.hinf_tab.toggle_hinf_control())
        
        # Agregar tabs en orden correcto
        self.tabs.addTab(self.hinf_tab, "🎛️ H∞ Synthesis")
        self.tabs.addTab(self.test_tab, "🧪 Prueba")
        
        # Pestaña 6: Cámara (CameraTab modular - auto-contenida)
        self.camera_tab = CameraTab(thorlabs_available=THORLABS_AVAILABLE, parent=self)
        # Conectar TestTab con CameraTab para sincronizar trayectoria
        self.camera_tab.set_test_tab_reference(self.test_tab)
        self.tabs.addTab(self.camera_tab, "🎥 ImgRec")
        
        # Conectar servicios de detección con CameraTab
        self._setup_detection_services()
        
        # Conectar señales de microscopia
        self.camera_tab.microscopy_start_requested.connect(self._start_microscopy)
        self.camera_tab.microscopy_stop_requested.connect(self._stop_microscopy)
        
        # Variables de microscopia
        self.microscopy_active = False
        self.microscopy_current_point = 0
        self.microscopy_config = None
        
        # Variables de C-Focus (autofoco usa AutofocusService)
        self.cfocus_controller = None
        self.cfocus_enabled = False
        
        # Pestaña 7: Análisis de Imagen (ImgAnalysisTab - Índice de Nitidez)
        self.img_analysis_tab = ImgAnalysisTab(parent=self)
        self.tabs.addTab(self.img_analysis_tab, "🔬 Img Analysis")
        
        # Exponer SmartFocusScorer para CameraViewWindow (usa el mismo que ImgAnalysisTab)
        self.smart_focus_scorer = self.img_analysis_tab.scorer
        
        main_layout.addWidget(self.tabs)

        # Conectar señal de datos seriales y arrancar thread
        self.serial_thread.data_received.connect(self.update_data)
        self.serial_thread.start()
        
        # Actualizar estado inicial de conexión en ControlTab
        self._update_connection_status()
    
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

    # NOTA: create_control_group(), create_motors_group(), create_sensors_group() 
    # ELIMINADOS - Reemplazados por ControlTab modular (Fase 12)
    # ============================================================================
    # ============================================================================
    
    def _on_serial_reconnect(self, port: str, baudrate: int):
        """Maneja la reconexión serial desde ControlTab."""
        logger.info(f"=== RECONEXIÓN SERIAL SOLICITADA: {port} @ {baudrate} ===")
        
        try:
            # Detener thread anterior
            if self.serial_thread and self.serial_thread.isRunning():
                logger.debug("Deteniendo thread serial anterior")
                self.serial_thread.stop()
                self.serial_thread.wait(1000)  # Esperar máximo 1 segundo
            
            # Crear nuevo thread con los nuevos parámetros
            logger.debug(f"Creando nuevo SerialHandler: {port} @ {baudrate}")
            self.serial_thread = SerialHandler(port, baudrate)
            
            # Reconectar señal de datos
            self.serial_thread.data_received.connect(self.update_data)
            
            # Actualizar referencia en ControlTab
            self.control_tab.serial_handler = self.serial_thread
            
            # Iniciar thread
            self.serial_thread.start()
            
            # Esperar un momento para que intente conectar
            QTimer.singleShot(500, self._update_connection_status)
            
            logger.info(f"✅ Reconexión iniciada: {port} @ {baudrate}")
            
        except Exception as e:
            logger.error(f"Error en reconexión serial: {e}")
            self.control_tab.set_connection_status(False)
    
    def _update_connection_status(self):
        """Actualiza el estado de conexión en ControlTab."""
        if self.serial_thread and self.serial_thread.ser and self.serial_thread.ser.is_open:
            port = self.serial_thread.ser.port
            self.control_tab.set_connection_status(True, port)
            logger.info(f"Estado conexión actualizado: Conectado a {port}")
        else:
            self.control_tab.set_connection_status(False)
            logger.info("Estado conexión actualizado: Desconectado")
    
    def update_data(self, line):
        """
        PROCESAMIENTO de datos del Arduino con VALIDACIÓN.
        Formato viejo: pot_a,pot_b,sens_1,sens_2 (4 enteros CSV)
        Formato nuevo: pot_a,pot_b,sens_1,sens_2,estado,settled (6 campos)
        Filtra datos inválidos y actualiza estado de ControlTab.
        """
        # Mensajes de sistema o cabecera
        if line.startswith(("ERROR:", "INFO:", "Potencia")):
            logger.info(line)
            return

        try:
            # Parseo con validación - manejar ambos formatos
            parts = line.split(',')
            
            if len(parts) == 6:
                # Formato nuevo con estado y settled
                parsed_data = MotorProtocol.parse_sensor_data_with_status(line)
                if parsed_data:
                    pot_a = parsed_data['pot_a']
                    pot_b = parsed_data['pot_b']
                    sens_1 = parsed_data['sens_1']
                    sens_2 = parsed_data['sens_2']
                    
                    # Actualizar valores en ControlTab
                    self.control_tab.update_motor_values(pot_a, pot_b)
                    self.control_tab.update_sensor_values(sens_1, sens_2)
                    
                    # Actualizar estado del Arduino en ControlTab
                    self.control_tab.update_arduino_status(parsed_data['state'], parsed_data['settled'])
                    
                    # Actualizar SignalWindow (si está visible)
                    if self.signal_window and self.signal_window.isVisible():
                        self.signal_window.update_data(pot_a, pot_b, sens_1, sens_2)
                    
                    # Grabar datos (si está grabando)
                    if self.data_recorder.is_recording:
                        self.data_recorder.write_data_point(pot_a, pot_b, sens_1, sens_2)
                    
            elif len(parts) == 4:
                # Formato viejo - firmware sin Position Hold
                pot_a, pot_b, sens_1, sens_2 = map(int, parts)
                
                # Validar rangos
                if not (-255 <= pot_a <= 255 and -255 <= pot_b <= 255 and 
                       0 <= sens_1 <= 1023 and 0 <= sens_2 <= 1023):
                    logger.warning(f"Datos fuera de rango: {line}")
                    return
                
                # Actualizar valores en ControlTab
                self.control_tab.update_motor_values(pot_a, pot_b)
                self.control_tab.update_sensor_values(sens_1, sens_2)
                
                # Mostrar estado LEGACY para indicar firmware viejo
                self.control_tab.update_arduino_status("LEGACY", False)
                
                # Actualizar SignalWindow (si está visible)
                if self.signal_window and self.signal_window.isVisible():
                    self.signal_window.update_data(pot_a, pot_b, sens_1, sens_2)
                
                # Grabar datos (si está grabando)
                if self.data_recorder.is_recording:
                    self.data_recorder.write_data_point(pot_a, pot_b, sens_1, sens_2)
                    
            else:
                logger.warning(f"Formato de datos inválido ({len(parts)} campos): {line}")
                return
                
        except (ValueError, IndexError) as e:
            logger.warning(f"Error parseando datos: '{line}' - {e}")
            return
    
    def _on_show_plot(self, fig, title):
        """Muestra una ventana con un gráfico de matplotlib."""
        logger.info(f"=== MOSTRANDO GRÁFICO: {title} ===")
        
        # Cerrar ventana anterior si existe
        if self.data_window is not None:
            self.data_window.close()
        
        # Crear y mostrar nueva ventana
        self.data_window = MatplotlibWindow(fig, title, self)
        self.data_window.show()
        self.data_window.raise_()
        self.data_window.activateWindow()
        QApplication.processEvents()
        logger.info(f"Ventana '{title}' mostrada exitosamente")
            
    # --- Lógica de Control y Comandos ---
    # Toda la lógica de grabación está ahora en RecordingTab
    def _on_recording_started(self, filename: str):
        """Callback cuando RecordingTab inicia grabación (Fase 12)."""
        logger.info(f"RecordingTab: Grabación iniciada - {filename}")
    
    def _on_recording_stopped(self):
        """Callback cuando RecordingTab detiene grabación (Fase 12)."""
        logger.info("RecordingTab: Grabación detenida")
    
    def _on_analysis_completed(self, results: dict):
        """Callback cuando AnalysisTab completa análisis."""
        logger.info(f"AnalysisTab: Análisis completado - K={results.get('K', 0):.4f}")
    
    def _on_show_plot(self, fig, title="Gráfico"):
        """Callback para mostrar plot desde AnalysisTab."""
        logger.debug(f"AnalysisTab: Mostrando plot - {title}")
        
        # Cerrar ventana anterior si existe
        if self.data_window is not None:
            self.data_window.close()
        
        # Crear y mostrar nueva ventana
        self.data_window = MatplotlibWindow(fig, title, self)
        self.data_window.show()
        self.data_window.raise_()
        self.data_window.activateWindow()
        QApplication.processEvents()
        logger.info(f"Ventana '{title}' mostrada exitosamente")
    
    # HInfTab ahora llama directamente a su método synthesize_hinf_controller()
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

    # --- NUEVOS HANDLERS PARA POSITION HOLD ---
    
    def _on_position_hold(self, sensor1_target: int, sensor2_target: int):
        """Maneja solicitud de position hold desde ControlTab."""
        logger.info(f"=== POSITION HOLD SOLICITADO: S1={sensor1_target}, S2={sensor2_target} ===")
        command = MotorProtocol.format_position_hold(sensor1_target, sensor2_target)
        self.send_command(command)
    
    def _on_brake(self):
        """Maneja solicitud de freno activo desde ControlTab."""
        logger.info("=== FRENO ACTIVO SOLICITADO ===")
        command = MotorProtocol.format_brake_command()
        self.send_command(command)
    
    def _on_settling_config(self, threshold: int):
        """Maneja configuración de umbral de asentamiento desde ControlTab."""
        logger.info(f"=== CONFIGURANDO UMBRAL DE ASENTAMIENTO: {threshold} ===")
        command = MotorProtocol.format_settling_config(threshold)
        self.send_command(command)

    # NOTA: set_manual_mode(), set_auto_mode(), send_power_command() 
    # ELIMINADOS - Ahora están en ControlTab
    
    # --- Microscopia Automatizada ---
    # Flujo: Mover -> Esperar posicion -> DELAY_BEFORE -> Capturar -> DELAY_AFTER -> Siguiente
    
    def _start_microscopy(self, config: dict):
        """Inicia la microscopia automatizada con la configuracion dada."""
        logger.info("=== INICIANDO MICROSCOPIA AUTOMATIZADA ===")
        logger.info(f"Config: {config}")
        
        # Verificar que hay trayectoria en TestTab
        if not hasattr(self.test_tab, 'current_trajectory') or self.test_tab.current_trajectory is None:
            self.camera_tab.log_message("Error: No hay trayectoria en TestTab")
            self.camera_tab._stop_microscopy()
            return
        
        # Verificar controladores
        if self.test_tab.controller_a is None or self.test_tab.controller_b is None:
            self.camera_tab.log_message("Error: Se requieren controladores para ambos motores")
            self.camera_tab._stop_microscopy()
            return
        
        # Guardar configuracion
        self.microscopy_config = config
        self.microscopy_active = True
        self.microscopy_current_point = 0
        self.microscopy_trajectory = self.test_tab.current_trajectory.copy()
        self._microscopy_position_checks = 0  # Contador para timeout
        
        # Obtener delays de config
        self._delay_before_ms = int(config.get('delay_before', 2.0) * 1000)
        self._delay_after_ms = int(config.get('delay_after', 0.2) * 1000)
        
        total = len(self.microscopy_trajectory)
        self.camera_tab.log_message(f"Iniciando microscopia: {total} puntos")
        self.camera_tab.log_message(f"Delay antes: {self._delay_before_ms}ms, Delay despues: {self._delay_after_ms}ms")
        logger.info(f"Microscopia: {total} puntos, delay_before={self._delay_before_ms}ms, delay_after={self._delay_after_ms}ms")
        
        # Ejecutar primer punto
        self._microscopy_move_to_point()
    
    def _microscopy_move_to_point(self):
        """PASO 1: Mueve al punto actual."""
        if not self.microscopy_active:
            return
        
        if self.microscopy_current_point >= len(self.microscopy_trajectory):
            self._finish_microscopy()
            return
        
        # Obtener punto actual
        point = self.microscopy_trajectory[self.microscopy_current_point]
        x_target = point[0]
        y_target = point[1]
        
        n = self.microscopy_current_point + 1
        total = len(self.microscopy_trajectory)
        self.camera_tab.log_message(f"[{n}/{total}] Moviendo a X={x_target:.1f}, Y={y_target:.1f} um")
        logger.info(f"Microscopia punto {n}: ({x_target:.1f}, {y_target:.1f})")
        
        # Configurar referencias en TestTab
        self.test_tab.ref_a_input.setText(f"{x_target:.0f}")
        self.test_tab.ref_b_input.setText(f"{y_target:.0f}")
        
        # Detener control dual si esta activo
        if self.test_tab.dual_control_active:
            self.test_tab.stop_dual_control()
        
        # Resetear contador de checks
        self._microscopy_position_checks = 0
        
        # Iniciar control dual para mover a la posicion
        self.test_tab.start_dual_control()
        
        # Comenzar a verificar si llego a la posicion
        QTimer.singleShot(200, self._microscopy_check_position)
    
    def _microscopy_check_position(self):
        """PASO 2: Verifica si llego a la posicion objetivo."""
        if not self.microscopy_active:
            return
        
        self._microscopy_position_checks += 1
        
        # Verificar si el control dual reporta posicion alcanzada
        # Usamos _position_reached de TestTab o verificamos error < tolerancia
        position_reached = getattr(self.test_tab, '_position_reached', False)
        
        # Timeout: maximo 10 segundos esperando posicion (100 checks * 100ms)
        if self._microscopy_position_checks > 100:
            self.camera_tab.log_message("  Timeout esperando posicion - continuando")
            logger.warning(f"Microscopia: timeout en punto {self.microscopy_current_point + 1}")
            position_reached = True  # Forzar continuar
        
        if position_reached:
            # Posicion alcanzada - detener motores y aplicar freno
            self.test_tab.stop_dual_control()
            
            # PASO 3: Aplicar DELAY_BEFORE para estabilizacion mecanica
            self.camera_tab.log_message(f"  Posicion alcanzada - Esperando {self._delay_before_ms}ms para estabilizar...")
            logger.info(f"Microscopia: posicion alcanzada, delay_before={self._delay_before_ms}ms")
            
            QTimer.singleShot(self._delay_before_ms, self._microscopy_capture)
        else:
            # Seguir esperando - verificar cada 100ms
            QTimer.singleShot(100, self._microscopy_check_position)
    
    def _microscopy_capture(self):
        """PASO 3: Captura la imagen despues del delay de estabilizacion."""
        if not self.microscopy_active:
            return
        
        # Si autofoco está habilitado, usar servicio de autofoco
        if self.microscopy_config.get('autofocus_enabled', False) and self.cfocus_enabled:
            self._microscopy_capture_with_autofocus()
            return
        
        # Captura normal (sin autofoco)
        self.camera_tab.log_message(f"  Capturando imagen...")
        success = self.camera_tab.capture_microscopy_image(
            self.microscopy_config, 
            self.microscopy_current_point
        )
        
        if success:
            logger.info(f"Microscopia: imagen {self.microscopy_current_point + 1} capturada")
        else:
            self.camera_tab.log_message(f"  ERROR: Fallo captura imagen {self.microscopy_current_point + 1}")
            logger.error(f"Microscopia: fallo captura imagen {self.microscopy_current_point + 1}")
        
        # Actualizar progreso
        self.camera_tab.set_microscopy_progress(
            self.microscopy_current_point + 1,
            len(self.microscopy_trajectory)
        )
        
        # Avanzar al siguiente punto
        self.microscopy_current_point += 1
        
        # PASO 4: Aplicar DELAY_AFTER antes de mover al siguiente punto
        if self.microscopy_current_point < len(self.microscopy_trajectory):
            self.camera_tab.log_message(f"  Pausa post-captura: {self._delay_after_ms}ms")
            QTimer.singleShot(self._delay_after_ms, self._microscopy_move_to_point)
        else:
            # Era el ultimo punto
            self._finish_microscopy()
    
    def _stop_microscopy(self):
        """Detiene la microscopia automatizada."""
        logger.info("=== DETENIENDO MICROSCOPIA ===")
        self.microscopy_active = False
        
        # Detener control dual si esta activo
        if self.test_tab.dual_control_active:
            self.test_tab.stop_dual_control()
        
        self.camera_tab.log_message("Microscopia detenida por usuario")
    
    def _finish_microscopy(self):
        """Finaliza la microscopia automatizada."""
        logger.info("=== MICROSCOPIA COMPLETADA ===")
        self.microscopy_active = False
        
        # Detener control dual
        if self.test_tab.dual_control_active:
            self.test_tab.stop_dual_control()
        
        total = len(self.microscopy_trajectory)
        self.camera_tab.log_message(f"MICROSCOPIA COMPLETADA: {total} imagenes capturadas")
        self.camera_tab._stop_microscopy()  # Actualizar UI
    
    # --- Control H∞ en Tiempo Real ---
    
    # --- Servicios de Detección U2-Net ---
    
    def _setup_detection_services(self):
        """Configura los servicios de detección y autofoco."""
        # Conectar señales del servicio de detección
        self.detection_service.detection_ready.connect(self._on_detection_ready)
        self.detection_service.status_changed.connect(self._on_detection_status)
        
        # Conectar señales del servicio de autofoco
        self.autofocus_service.scan_started.connect(self._on_autofocus_started)
        self.autofocus_service.z_changed.connect(self._on_autofocus_z_changed)
        self.autofocus_service.object_focused.connect(self._on_object_focused)
        self.autofocus_service.scan_complete.connect(self._on_autofocus_complete)
        
        logger.info("Servicios de detección configurados")
    
    def _on_detection_ready(self, saliency_map, objects):
        """Callback cuando hay nuevos resultados de detección."""
        # Actualizar visualización en CameraTab si tiene el widget
        if hasattr(self.camera_tab, 'saliency_widget') and self.camera_tab.saliency_widget:
            self.camera_tab.saliency_widget.update_detection(saliency_map, objects)
    
    def _on_detection_status(self, status):
        """Callback cuando cambia el estado del servicio de detección."""
        self.camera_tab.log_message(f"🔍 {status}")
    
    def _on_autofocus_started(self, obj_index, total):
        """Callback cuando inicia autofoco de un objeto."""
        self.camera_tab.log_message(f"🎯 Enfocando objeto {obj_index + 1}/{total}...")
    
    def _on_autofocus_z_changed(self, z, score, roi_frame):
        """Callback en cada posición Z evaluada."""
        # Actualizar visualización si está disponible
        if hasattr(self.camera_tab, 'saliency_widget') and self.camera_tab.saliency_widget:
            self.camera_tab.saliency_widget.update_autofocus_state(z, score, 0)
    
    def _on_object_focused(self, obj_index, z_optimal, score):
        """Callback cuando se encuentra el foco óptimo de un objeto."""
        self.camera_tab.log_message(f"  ✓ Obj{obj_index}: Z={z_optimal:.1f}µm, S={score:.1f}")
    
    def _on_autofocus_complete(self, results):
        """Callback cuando termina todo el proceso de autofoco."""
        n_results = len(results)
        
        # Mostrar resultados de cada objeto
        for r in results:
            self.camera_tab.log_message(f"   Obj{r.object_index}: Z={r.z_optimal:.1f}µm, Score={r.focus_score:.1f}")
        
        # Verificar posición Z actual del piezo
        if self.cfocus_enabled and self.cfocus_controller:
            current_z = self.cfocus_controller.read_z()
            if current_z is not None:
                self.camera_tab.log_message(f"📍 Posición Z actual: {current_z:.1f}µm (BPoF)")
        
        self.camera_tab.log_message(f"✅ Autofoco completado: {n_results} objetos enfocados")
        
        # Limpiar estado de autofoco en visualización
        if hasattr(self.camera_tab, 'saliency_widget') and self.camera_tab.saliency_widget:
            self.camera_tab.saliency_widget.clear_autofocus_state()
        
        # Si hay captura pendiente (desde botón "Capturar Imagen"), ejecutarla
        if hasattr(self.camera_tab, '_pending_capture') and self.camera_tab._pending_capture:
            self.camera_tab._pending_capture = False
            self.camera_tab.log_message("📸 Capturando imagen con mejor foco...")
            self.camera_tab._do_capture_image()
            return
        
        # Si estamos en microscopía, capturar imagen y avanzar al siguiente punto
        if self.microscopy_active:
            # Capturar imagen con el mejor foco usando método manual
            self.camera_tab.log_message(f"📸 Capturando imagen con BPoF...")
            self.camera_tab._do_capture_image()
            logger.info(f"Microscopia: imagen {self.microscopy_current_point + 1} capturada con autofoco")
            
            self._advance_microscopy_point()
    
    def start_realtime_detection(self):
        """Inicia detección en tiempo real."""
        if not self.camera_tab.camera_worker:
            self.camera_tab.log_message("⚠️ Conecta la cámara primero")
            return
        
        self.detection_service.start_detection()
    
    def stop_realtime_detection(self):
        """Detiene detección en tiempo real."""
        self.detection_service.stop_detection()
    
    # --- Métodos de Autofoco C-Focus ---
    
    def connect_cfocus(self):
        """Conecta con el piezo C-Focus."""
        from hardware.cfocus.cfocus_controller import CFocusController
        
        if self.cfocus_controller is None:
            self.cfocus_controller = CFocusController()
        
        success, message = self.cfocus_controller.connect()
        
        if success:
            self.cfocus_enabled = True
            self.camera_tab.log_message(f"✅ C-Focus: {message}")
            logger.info(f"C-Focus conectado: {message}")
        else:
            self.cfocus_enabled = False
            self.camera_tab.log_message(f"❌ C-Focus: {message}")
            logger.error(f"Error C-Focus: {message}")
        
        return success
    
    def disconnect_cfocus(self):
        """Desconecta el piezo C-Focus."""
        if self.cfocus_controller:
            self.cfocus_controller.disconnect()
            self.cfocus_enabled = False
            self.cfocus_controller = None
            logger.info("C-Focus desconectado")
    
    def initialize_autofocus(self):
        """Inicializa el servicio de autofoco con C-Focus y cámara."""
        if not self.cfocus_enabled:
            self.camera_tab.log_message("⚠️ C-Focus no conectado")
            return False
        
        if self.camera_tab.camera_worker is None:
            self.camera_tab.log_message("⚠️ Cámara no conectada")
            return False
        
        # Configurar AutofocusService con hardware
        self.autofocus_service.configure(
            cfocus_controller=self.cfocus_controller,
            get_frame_callback=lambda: self.camera_tab.camera_worker.current_frame
        )
        
        self.camera_tab.log_message("✅ Autofoco configurado (U2-Net + C-Focus)")
        logger.info("AutofocusService configurado con C-Focus y cámara")
        return True
    
    def _microscopy_capture_with_autofocus(self):
        """
        Captura con pre-detección U2-Net y autofoco asíncrono.
        Usa SmartFocusScorer para detección y AutofocusService para enfoque.
        """
        if not self.microscopy_active:
            return
        
        point_idx = self.microscopy_current_point
        
        # Obtener frame actual
        frame = self.camera_tab.camera_worker.current_frame
        if frame is None:
            self.camera_tab.log_message("⚠️ Sin frame disponible")
            self._advance_microscopy_point()
            return
        
        # Convertir frame uint16 -> uint8 para detección
        if frame.dtype == np.uint16:
            frame_max = frame.max()
            if frame_max > 0:
                frame_uint8 = (frame / frame_max * 255).astype(np.uint8)
            else:
                frame_uint8 = np.zeros_like(frame, dtype=np.uint8)
        else:
            frame_uint8 = frame.astype(np.uint8)
        
        if len(frame_uint8.shape) == 2:
            frame_bgr = cv2.cvtColor(frame_uint8, cv2.COLOR_GRAY2BGR)
        else:
            frame_bgr = frame_uint8
        
        # Detectar objetos con SmartFocusScorer (usa ObjectInfo compatible)
        self.camera_tab.log_message(f"🔍 Detectando objetos...")
        result = self.smart_focus_scorer.assess_image(frame_bgr)
        all_objects = result.objects if result.objects else []
        
        # Filtrar por rango de área configurado en CameraTab
        min_area = self.camera_tab.min_pixels_spin.value()
        max_area = self.camera_tab.max_pixels_spin.value()
        objects = [obj for obj in all_objects if min_area <= obj.area <= max_area]
        
        n_objects = len(objects)
        
        if n_objects == 0:
            self.camera_tab.log_message(f"  ⚠️ Sin objetos en rango [{min_area}-{max_area}] px - saltando punto")
            logger.info(f"Punto {point_idx}: sin objetos en rango (detectados: {len(all_objects)})")
            self._advance_microscopy_point()
            return
        
        self.camera_tab.log_message(f"  ✓ {n_objects} objeto(s) en rango (de {len(all_objects)} detectados)")
        logger.info(f"Punto {point_idx}: {n_objects} objetos válidos")
        
        # Iniciar autofoco asíncrono (hill-climbing rápido)
        self.autofocus_service.start_autofocus(objects)
        # El callback _on_autofocus_complete manejará la captura y avance
    
    def _advance_microscopy_point(self):
        """Avanza al siguiente punto de microscopía."""
        self.camera_tab.set_microscopy_progress(
            self.microscopy_current_point + 1,
            len(self.microscopy_trajectory)
        )
        
        self.microscopy_current_point += 1
        
        if self.microscopy_current_point < len(self.microscopy_trajectory):
            QTimer.singleShot(self._delay_after_ms, self._microscopy_move_to_point)
        else:
            self._finish_microscopy()
    
    def closeEvent(self, event):
        """Maneja el cierre de la aplicación."""
        logger.info("=== CERRANDO APLICACIÓN ===")
        logger.debug("Enviando comando de apagado de motores (A,0,0)")
        self.send_command('A,0,0')
        
        # Detener grabación si está activa
        if self.data_recorder.is_recording:
            logger.debug("Deteniendo grabación activa")
            self.data_recorder.stop_recording()
        
        # Desconectar C-Focus si está conectado
        if self.cfocus_controller:
            logger.debug("Desconectando C-Focus")
            self.disconnect_cfocus()
        
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


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
from core.services import DetectionService, AutofocusService, CameraService
from core.services.microscopy_service import MicroscopyService

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

# --- Cámara Thorlabs (centralizado) ---
from config.hardware_availability import THORLABS_AVAILABLE

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
        
        # Inicializar servicios de detección, cámara y autofoco
        self.detection_service = DetectionService()
        self.camera_service = CameraService(parent=self)
        self.autofocus_service = AutofocusService()
        
        # Iniciar comunicación serial ANTES de crear tabs (necesario para ControlTab)
        # Detectar puerto automáticamente o usar el configurado
        initial_port = self._detect_arduino_port() or SERIAL_PORT
        self.serial_thread = SerialHandler(initial_port, BAUD_RATE)
        
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
        
        # Pestaña 7: Análisis de Imagen (ImgAnalysisTab - Índice de Nitidez)
        # DEBE CREARSE ANTES de CameraOrchestrator porque necesita smart_focus_scorer
        self.img_analysis_tab = ImgAnalysisTab(parent=self)
        self.tabs.addTab(self.img_analysis_tab, "🔬 Img Analysis")
        
        # Exponer SmartFocusScorer para CameraViewWindow (usa el mismo que ImgAnalysisTab)
        self.smart_focus_scorer = self.img_analysis_tab.scorer
        
        # Crear CameraOrchestrator (coordina cámara + detección + autofoco)
        # AHORA smart_focus_scorer ya existe
        from core.services import CameraOrchestrator
        self.camera_orchestrator = CameraOrchestrator(
            camera_service=self.camera_service,
            detection_service=self.detection_service,
            autofocus_service=self.autofocus_service,
            smart_focus_scorer=self.smart_focus_scorer
        )
        
        # Pestaña 6: Cámara (CameraTab modular - auto-contenida)
        self.camera_tab = CameraTab(
            thorlabs_available=THORLABS_AVAILABLE,
            parent=self,
            camera_service=self.camera_service,
            camera_orchestrator=self.camera_orchestrator,
        )
        # Conectar CameraService con CameraTab (solo orquestación desde main)
        self.camera_service.connected.connect(self.camera_tab._on_camera_connected)
        self.camera_service.frame_ready.connect(self.camera_tab.on_camera_frame)
        self.camera_service.status_changed.connect(self.camera_tab.log_message)
        self.camera_service.disconnected.connect(lambda: self.camera_tab.set_connected(False))
        # Conectar TestTab con CameraTab para sincronizar trayectoria
        self.camera_tab.set_test_tab_reference(self.test_tab)
        self.tabs.addTab(self.camera_tab, "🎥 ImgRec")
        
        # Conectar servicios de detección con CameraTab
        self._setup_detection_services()

        # Servicio de microscopia (orquesta trayectoria, captura y autofoco)
        self.microscopy_service = MicroscopyService(
            parent=self,
            get_trajectory=lambda: getattr(self.test_tab, 'current_trajectory', None),
            get_trajectory_params=lambda: {
                'tolerance_um': getattr(self.test_tab, 'trajectory_tolerance', 25.0),
                'pause_s': getattr(self.test_tab, 'trajectory_pause', 2.0)
            },
            set_dual_refs=lambda x, y: (
                self.test_tab.ref_a_input.setText(f"{x:.0f}"),
                self.test_tab.ref_b_input.setText(f"{y:.0f}")
            ),
            start_dual_control=self.test_tab.start_dual_control,
            stop_dual_control=self.test_tab.stop_dual_control,
            is_dual_control_active=lambda: self.test_tab.dual_control_active,
            is_position_reached=lambda: getattr(self.test_tab, '_position_reached', False),
            capture_microscopy_image=self.camera_tab.capture_microscopy_image,
            autofocus_service=self.autofocus_service,
            cfocus_enabled_getter=lambda: self.cfocus_enabled,
            get_current_frame=lambda: self.camera_tab.camera_worker.current_frame
            if self.camera_tab.camera_worker is not None
            else None,
            smart_focus_scorer=self.smart_focus_scorer,
            get_area_range=lambda: (
                self.camera_tab.min_pixels_spin.value(),
                self.camera_tab.max_pixels_spin.value(),
            ),
            controllers_ready_getter=lambda: (
                getattr(self.test_tab, 'controller_a', None) is not None
                and getattr(self.test_tab, 'controller_b', None) is not None
            ),
            test_service=self.test_tab.test_service,
        )

        # Conectar señales de microscopía
        self.microscopy_service.status_changed.connect(self.camera_tab.log_message)
        self.microscopy_service.progress_changed.connect(self._on_microscopy_progress)
        self.microscopy_service.finished.connect(self._on_microscopy_finished)
        
        # Conectar señales de máscaras de autofoco con CameraViewWindow
        self.microscopy_service.show_masks.connect(self._on_show_autofocus_masks)
        self.microscopy_service.clear_masks.connect(self._on_clear_autofocus_masks)

        # Aprendizaje asistido: popup de confirmación
        if hasattr(self.microscopy_service, 'learning_confirmation_requested'):
            self.microscopy_service.learning_confirmation_requested.connect(
                self._on_learning_confirmation_requested
            )

        # Conectar señales de microscopia desde CameraTab hacia el servicio
        self.camera_tab.microscopy_start_requested.connect(
            self.microscopy_service.start_microscopy
        )
        self.camera_tab.microscopy_stop_requested.connect(
            self.microscopy_service.stop_microscopy
        )
        
        # Variables de C-Focus (autofoco usa AutofocusService)
        self.cfocus_controller = None
        self.cfocus_enabled = False
        
        main_layout.addWidget(self.tabs)

        # Conectar señal de datos seriales y arrancar thread
        self.serial_thread.data_received.connect(self.update_data)
        self.serial_thread.start()
        
        # Actualizar estado inicial de conexión en ControlTab
        self._update_connection_status()

    def _on_learning_confirmation_requested(self, frame_bgr, obj, class_name, confidence, count, target):
        """Muestra popup de confirmación de ROI y retorna la respuesta al servicio."""
        try:
            from gui.dialogs import LearningConfirmationDialog

            dialog = LearningConfirmationDialog(self)
            roi_bbox = getattr(obj, 'bounding_box', (0, 0, 0, 0))
            roi_mask = getattr(obj, 'mask', None)
            area = int(getattr(obj, 'area', 0))
            score = float(getattr(obj, 'focus_score', 0.0))

            response = dialog.show_roi_for_confirmation(
                frame_bgr,
                roi_bbox,
                roi_mask,
                area,
                score,
                count,
                target,
            )

            # Enviar respuesta de usuario de vuelta al servicio
            # Permitir respuesta enriquecida (dict) con ROIs manuales
            if isinstance(response, dict):
                self.microscopy_service.confirm_learning_step(response, class_name)
            else:
                self.microscopy_service.confirm_learning_step(bool(response), class_name)
        except Exception as e:
            logging.getLogger('MotorControl_L206').error(
                f"Error en _on_learning_confirmation_requested: {e}\n{traceback.format_exc()}"
            )
            # Auto-aceptar en caso de error para no bloquear el flujo
            self.microscopy_service.confirm_learning_step(True, class_name)
    
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
    
    def _detect_arduino_port(self):
        """
        Detecta automáticamente el puerto del Arduino.
        
        Returns:
            str: Puerto detectado (ej: 'COM3') o None si no se encuentra
        """
        import serial.tools.list_ports
        
        ports = serial.tools.list_ports.comports()
        if not ports:
            logger.warning("No se encontraron puertos seriales disponibles")
            return None
        
        # Buscar Arduino por descripción
        for port in ports:
            desc_lower = port.description.lower()
            if any(x in desc_lower for x in ['arduino', 'ch340', 'ch341', 'ftdi', 'usb serial']):
                logger.info(f"Arduino detectado automáticamente en: {port.device} ({port.description})")
                return port.device
        
        # Si no se encuentra Arduino, usar el primer puerto disponible
        first_port = ports[0].device
        logger.warning(f"Arduino no detectado. Usando primer puerto disponible: {first_port}")
        return first_port

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
    
    
    # --- Control H∞ en Tiempo Real ---
    
    # --- Servicios de Detección U2-Net ---
    
    def _setup_detection_services(self):
        """Configura los servicios de detección y autofoco."""
        # Conectar señales del servicio de detección
        self.detection_service.detection_ready.connect(self.camera_tab.on_detection_ready)
        self.detection_service.status_changed.connect(self.camera_tab.on_detection_status)
        
        # Conectar señales del servicio de autofoco
        self.autofocus_service.scan_started.connect(self.camera_tab.on_autofocus_started)
        self.autofocus_service.scan_started.connect(self._on_autofocus_started)
        self.autofocus_service.z_changed.connect(self.camera_tab.on_autofocus_z_changed)
        self.autofocus_service.object_focused.connect(self.camera_tab.on_object_focused)
        self.autofocus_service.scan_complete.connect(self._on_autofocus_complete)
        # Errores de autofoco → mostrarlos en CameraTab
        self.autofocus_service.error_occurred.connect(
            lambda msg: self.camera_tab.log_message(f"❌ Autofoco: {msg}")
        )
        
        # Conectar señales para overlay de score en ventana de cámara
        self.autofocus_service.score_updated.connect(self._on_autofocus_score_updated)
        self.autofocus_service.status_message.connect(self._on_autofocus_status_message)
        
        # Conectar señal de progreso
        self.autofocus_service.progress_updated.connect(self._on_autofocus_progress)
        
        logger.info("Servicios de detección configurados")
    
    def _on_autofocus_started(self, obj_index: int, total_objects: int):
        """Callback cuando inicia el autofoco - activa overlay en ventana de cámara."""
        if self.camera_tab.camera_view_window:
            self.camera_tab.camera_view_window.set_autofocus_active(True)
        logger.info(f"[Main] Autofoco iniciado: objeto {obj_index+1}/{total_objects}")
    
    def _on_autofocus_score_updated(self, z_position: float, score: float):
        """Callback para actualizar overlay de score en ventana de cámara."""
        if self.camera_tab.camera_view_window:
            self.camera_tab.camera_view_window.update_autofocus_score(z_position, score)
    
    def _on_autofocus_status_message(self, message: str):
        """Callback para mensajes de estado del autofoco."""
        # Mostrar en ventana de cámara
        if self.camera_tab.camera_view_window:
            self.camera_tab.camera_view_window.set_autofocus_status(message)
        # También mostrar en log de CameraTab
        self.camera_tab.log_message(message)
    
    def _on_autofocus_progress(self, current_step: int, total_steps: int, phase_name: str):
        """Callback para actualizar progreso del autofoco."""
        percentage = int((current_step / total_steps) * 100) if total_steps > 0 else 0
        progress_msg = f"⏳ {phase_name}: {current_step}/{total_steps} ({percentage}%)"
        
        # Actualizar en ventana de cámara
        if self.camera_tab.camera_view_window:
            self.camera_tab.camera_view_window.set_autofocus_status(progress_msg)
        
        # Log cada 10% para no saturar
        if current_step == 1 or percentage % 10 == 0 or current_step == total_steps:
            self.camera_tab.log_message(progress_msg)
    
    def _on_autofocus_complete(self, results):
        """Callback cuando termina todo el proceso de autofoco."""
        # Desactivar overlay de score
        if self.camera_tab.camera_view_window:
            self.camera_tab.camera_view_window.set_autofocus_active(False)
        
        n_results = len(results)
        
        # Mostrar resultados de cada objeto
        for r in results:
            self.camera_tab.log_message(
                f"   Obj{r.object_index}: Z={r.z_optimal:.1f}µm, Score={r.focus_score:.1f}"
            )
        
        # Verificar posición Z actual del piezo
        if self.cfocus_enabled and self.cfocus_controller:
            current_z = self.cfocus_controller.read_z()
            if current_z is not None:
                self.camera_tab.log_message(
                    f"📍 Posición Z actual: {current_z:.1f}µm (BPoF)"
                )
        
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

        # Si estamos en microscopia, delegar captura y avance al servicio
        # Pasar los resultados del autofoco que incluyen el frame ya capturado en BPoF
        if hasattr(self, 'microscopy_service') and self.microscopy_service.is_running():
            self.microscopy_service.handle_autofocus_complete(results)
    
    def _on_show_autofocus_masks(self, masks_data):
        """Muestra máscaras de autofoco en la ventana de cámara."""
        if self.camera_tab.camera_view_window:
            self.camera_tab.camera_view_window.show_autofocus_masks(masks_data)
    
    def _on_clear_autofocus_masks(self):
        """Limpia máscaras de autofoco de la ventana de cámara."""
        if self.camera_tab.camera_view_window:
            self.camera_tab.camera_view_window.clear_autofocus_masks()
    
    def _on_microscopy_progress(self, current, total):
        """Handler de progreso de microscopía."""
        self.camera_tab.set_microscopy_progress(current, total)
    
    def _on_microscopy_finished(self):
        """Handler cuando termina la microscopía."""
        logger.info("Microscopía finalizada")
        self.camera_tab.log_message("✅ Microscopía completada")
        self.camera_tab.set_trajectory_status(ready=True)
    
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
            
            # Configurar C-Focus en ventana de cámara para lectura Z en tiempo real
            if self.camera_tab.camera_view_window:
                self.camera_tab.camera_view_window.set_cfocus_controller(self.cfocus_controller)
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
    
    def calibrate_cfocus(self):
        """Ejecuta calibración de límites del C-Focus."""
        if not self.cfocus_enabled or not self.cfocus_controller:
            self.camera_tab.log_message("⚠️ C-Focus no conectado")
            return
        
        self.camera_tab.log_message("🔧 Iniciando calibración de C-Focus...")
        logger.info("[Main] Iniciando calibración de C-Focus")
        
        try:
            result = self.cfocus_controller.calibrate_limits()
            
            if result:
                msg = (f"✅ Calibración completada:\n"
                       f"   Mín: {result['z_min']:.2f} µm\n"
                       f"   Máx: {result['z_max']:.2f} µm\n"
                       f"   Centro: {result['z_center']:.2f} µm\n"
                       f"   Rango: {result['z_range']:.2f} µm")
                self.camera_tab.log_message(msg)
                logger.info(f"[Main] Calibración exitosa: {result}")
                
                # CRÍTICO: Configurar AutofocusService después de calibrar
                self.initialize_autofocus()
            else:
                self.camera_tab.log_message("❌ Error en calibración")
                
        except Exception as e:
            self.camera_tab.log_message(f"❌ Error: {e}")
            logger.error(f"[Main] Error en calibración: {e}", exc_info=True)
    
    def initialize_autofocus(self):
        """Inicializa el servicio de autofoco con C-Focus y cámara."""
        if not self.cfocus_enabled:
            self.camera_tab.log_message("⚠️ C-Focus no conectado")
            return False
        
        # Obtener worker de cámara desde el servicio (preferente) o desde la Tab
        worker = None
        if hasattr(self, 'camera_service') and self.camera_service.worker is not None:
            worker = self.camera_service.worker
        elif self.camera_tab.camera_worker is not None:
            worker = self.camera_tab.camera_worker

        if worker is None:
            self.camera_tab.log_message("⚠️ Cámara no conectada")
            return False

        # Configurar AutofocusService con hardware
        self.autofocus_service.configure(
            cfocus_controller=self.cfocus_controller,
            get_frame_callback=lambda: worker.current_frame
        )
        
        self.camera_tab.log_message("✅ Autofoco configurado (U2-Net + C-Focus)")
        logger.info("AutofocusService configurado con C-Focus y cámara")
        return True
    
    
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


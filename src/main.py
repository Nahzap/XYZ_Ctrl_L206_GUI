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
from pathlib import Path

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
from config.mcu_profiles import apply_mcu_profile, load_saved_mcu

# Perfil MCU activo (STM32 o Arduino). Persistido en config/mcu_prefs.json.
apply_mcu_profile(load_saved_mcu())

# Fase 2: Estilos
from gui.styles.dark_theme import DARK_STYLESHEET

# Fase 3: Comunicación Serial
from core.communication.serial_handler import SerialHandler
from core.communication.protocol import MotorProtocol
from core.control import SensorBuffer

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
from core.persistence import SessionStore

# Fase 10: Pestañas GUI (Tabs modulares) - Integrado en Fase 12
from gui.tabs import (ControlTab, RecordingTab, AnalysisTab, 
                      CameraTab, TestTab, HInfTab, ImgAnalysisTab, CanvasGenTab)

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
class CTRL_GUI(QMainWindow):
    """Ventana principal del sistema de control (STM32F767ZI + host Python)."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle('CTRL_GUI — MycoViT XY / STM32F767ZI')
        self.setGeometry(100, 100, 800, 700)
        self.setStyleSheet(DARK_STYLESHEET)

        # Inicializar grabador de datos (Fase 6)
        self.data_recorder = DataRecorder()
        
        # Inicializar analizador de transferencia (Fase 7)
        self.tf_analyzer = TransferFunctionAnalyzer()
        self.session_store = SessionStore()
        
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
        self.sensor_buffer = SensorBuffer()
        # Plano MÁQUINA: el hilo RX llena el buffer a tasa completa (Fase 1),
        # así el control lee medida fresca sin depender del repintado de la UI.
        self.serial_thread.set_sensor_buffer(self.sensor_buffer)
        # Compuerta de refresco de UI (~30 Hz) — no afecta medida ni control.
        self._last_ui_update_mono = 0.0
        
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
        self.control_tab.mcu_profile_changed.connect(self._on_mcu_profile_changed)
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
            get_sensor_value=self._get_sensor_adc,
            get_mode_label=lambda: self.control_tab.value_labels.get('mode', None)
        )
        self.test_tab.test_service.set_sensor_buffer(self.sensor_buffer)
        # TestTab maneja sus operaciones internamente
        self.test_tab.controller_clear_requested.connect(lambda motor: self.test_tab.clear_controller(motor))
        
        # Pestaña 4: H∞ Synthesis (HInfTab modular) - CREAR DESPUÉS de TestTab
        # HInfTab ahora crea automáticamente HInfTrackingController (Zhou & Doyle)
        self.hinf_tab = HInfTab(hinf_controller=None, tf_analyzer=self.tf_analyzer, parent=self)
        # Configurar callbacks de hardware para control en tiempo real
        self.hinf_tab.set_hardware_callbacks(
            send_command=self.send_command,
            get_sensor_value=self._get_sensor_adc,
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
        # Direct: CameraService y CameraTab viven en el hilo GUI; evita 2ª cola
        self.camera_service.frame_ready.connect(
            self.camera_tab.on_camera_frame, Qt.DirectConnection
        )
        self.camera_service.status_changed.connect(self.camera_tab.log_message)
        self.camera_service.disconnected.connect(lambda: self.camera_tab.set_connected(False))
        # Conectar TestTab con CameraTab para sincronizar trayectoria
        self.camera_tab.set_test_tab_reference(self.test_tab)
        self.tabs.addTab(self.camera_tab, "🎥 ImgRec")

        self.canvas_gen_tab = CanvasGenTab(parent=self, test_tab=self.test_tab)
        self.tabs.addTab(self.canvas_gen_tab, "🧩 CanvasGen")
        
        # Conectar servicios de detección con CameraTab
        self._setup_detection_services()

        # Servicio de microscopia (orquesta trayectoria, captura y autofoco)
        self.microscopy_service = MicroscopyService(
            parent=self,
            get_trajectory=lambda: getattr(self.test_tab, 'current_trajectory', None),
            get_trajectory_params=lambda: self.test_tab.get_trajectory_execution_params(),
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
            get_current_frame=lambda: (
                self.camera_service.acquire_scientific_frame(timeout_s=1.5).image16
                if self.camera_service is not None
                else None
            ),
            smart_focus_scorer=self.smart_focus_scorer,
            get_area_range=self.camera_tab.get_area_range,
            controllers_ready_getter=lambda: (
                getattr(self.test_tab, 'controller_a', None) is not None
                and getattr(self.test_tab, 'controller_b', None) is not None
            ),
            test_service=self.test_tab.test_service,
            send_command=self.send_command,
        )

        # Conectar señales de microscopía
        self.microscopy_service.status_changed.connect(self.camera_tab.log_message)
        self.microscopy_service.progress_changed.connect(self._on_microscopy_progress)
        self.microscopy_service.finished.connect(self._on_microscopy_finished)
        self.microscopy_service.resume_suggested.connect(
            self.camera_tab.suggest_resume_point
        )
        self.microscopy_service.stopped.connect(
            lambda: self.camera_tab._update_resume_button_label()
        )
        
        # Conectar señales de máscaras de autofoco con CameraViewWindow
        self.microscopy_service.show_masks.connect(self._on_show_autofocus_masks)
        self.microscopy_service.clear_masks.connect(self._on_clear_autofocus_masks)
        
        # CRÍTICO: Conectar señal de detección de microscopia para actualizar lista de objetos
        self.microscopy_service.detection_complete.connect(self.camera_tab._on_microscopy_detection_complete)

        # Autodetección en vivo: encolar solo cuando XY no actúa (FOV/approach libres).
        self.detection_service.set_motion_busy_gate(
            lambda: bool(
                getattr(self.test_tab, "test_service", None) is not None
                and self.test_tab.test_service.is_xy_motion_active()
            )
        )

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

        # Restaurar sesión persistida (análisis + H∞ + TestTab)
        self._load_session_state()
        
        # Estado inicial: el puerto se abre en el hilo RX; consultar tras el open.
        QTimer.singleShot(600, self._update_connection_status)

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

            self.signal_window.refresh_adc_range()
            self.signal_window.show()
            self.signal_window.raise_()
            self.signal_window.activateWindow()
            logger.info("Ventana de señales mostrada y activada")
        except Exception as e:
            logger.error(f"Error al abrir ventana de señales: {e}\n{traceback.format_exc()}")
    
    def _detect_arduino_port(self):
        """
        Detecta automáticamente el puerto del controlador XY (STM32 ST-Link VCP u otros).
        
        Returns:
            str: Puerto detectado (ej: 'COM5') o None si no se encuentra
        """
        import serial.tools.list_ports
        
        ports = serial.tools.list_ports.comports()
        if not ports:
            logger.warning("No se encontraron puertos seriales disponibles")
            return None
        
        keywords = (
            'stlink', 'st-link', 'stm', 'stmicroelectronics', 'virtual com',
            'arduino', 'ch340', 'ch341', 'ftdi', 'usb serial',
        )
        for port in ports:
            desc_lower = port.description.lower()
            mfg = (port.manufacturer or '').lower()
            haystack = f"{desc_lower} {mfg}"
            if any(x in haystack for x in keywords):
                logger.info(f"Controlador XY detectado en: {port.device} ({port.description})")
                return port.device
        
        first_port = ports[0].device
        logger.warning(f"Controlador XY no detectado por descripción. Usando: {first_port}")
        return first_port

    # NOTA: create_control_group(), create_motors_group(), create_sensors_group() 
    # ELIMINADOS - Reemplazados por ControlTab modular (Fase 12)
    # ============================================================================
    # ============================================================================
    
    def _on_mcu_profile_changed(self, mcu_id: str):
        """Aplica perfil STM32/Arduino y sincroniza flags de control FOV."""
        profile = apply_mcu_profile(mcu_id)
        logger.info(
            "Perfil MCU aplicado: %s (cz=%s, stiction=[%s,%s])",
            mcu_id,
            profile.get("supports_cz"),
            profile.get("stiction_pwm_min"),
            profile.get("stiction_pwm_max"),
        )
        try:
            ts = getattr(self, "test_service", None)
            if ts is not None and getattr(ts, "step_controller", None) is not None:
                cfg = ts.step_controller.config
                cfg.use_mcu_cz_loop = bool(profile.get("use_mcu_cz_loop", False))
                logger.info("TestService.use_mcu_cz_loop -> %s", cfg.use_mcu_cz_loop)
        except Exception as e:
            logger.debug("No se pudo sync use_mcu_cz_loop: %s", e)
        if hasattr(self, "control_tab") and self.control_tab is not None:
            hint = profile.get("firmware_hint", mcu_id)
            self.control_tab.firmware_status_label.setText(
                f"Firmware perfil: {hint} | {profile.get('telemetry', '')}"
            )
        if self.signal_window is not None:
            self.signal_window.refresh_adc_range()

    def _on_serial_reconnect(self, port: str, baudrate: int, allow_retry: bool = True):
        """Maneja la reconexión serial desde ControlTab."""
        logger.info(f"=== RECONEXIÓN SERIAL SOLICITADA: {port} @ {baudrate} ===")
        
        try:
            old = self.serial_thread
            if old is not None:
                logger.debug("Deteniendo thread serial anterior")
                try:
                    old.data_received.disconnect(self.update_data)
                except (TypeError, RuntimeError):
                    pass
                old.stop()
                if not old.wait(3000):
                    logger.warning("SerialHandler anterior no terminó a tiempo; forzando cierre")
                    try:
                        if old.ser and old.ser.is_open:
                            old.ser.close()
                    except Exception:
                        pass
                # Dar tiempo a Windows a liberar el handle del VCP ST-Link.
                time.sleep(0.35)

            logger.debug(f"Creando nuevo SerialHandler: {port} @ {baudrate}")
            self.serial_thread = SerialHandler(port, baudrate)
            self.serial_thread.set_sensor_buffer(self.sensor_buffer)
            self.serial_thread.data_received.connect(self.update_data)
            self.control_tab.serial_handler = self.serial_thread
            self.serial_thread.start()

            # Verificar tras abrir; un reintento si el VCP aún está ocupado.
            if allow_retry:
                QTimer.singleShot(
                    600,
                    lambda: self._update_connection_status(
                        retry_port=port, retry_baud=baudrate
                    ),
                )
            else:
                QTimer.singleShot(600, self._update_connection_status)
            
            logger.info(f"✅ Reconexión iniciada: {port} @ {baudrate}")
            
        except Exception as e:
            logger.error(f"Error en reconexión serial: {e}")
            self.control_tab.set_connection_status(False)
    
    def _update_connection_status(self, retry_port: str = None, retry_baud: int = None):
        """Actualiza el estado de conexión en ControlTab."""
        if self.serial_thread and self.serial_thread.ser and self.serial_thread.ser.is_open:
            port = self.serial_thread.ser.port
            self.control_tab.set_connection_status(True, port)
            logger.info(f"Estado conexión actualizado: Conectado a {port}")
            return

        self.control_tab.set_connection_status(False)
        logger.info("Estado conexión actualizado: Desconectado")
        if retry_port and retry_baud and not getattr(self, "_serial_retry_armed", False):
            self._serial_retry_armed = True
            logger.warning(
                "COM ocupado tras reconexión; reintento único en 1.2 s (%s @ %s)",
                retry_port,
                retry_baud,
            )
            QTimer.singleShot(
                1200,
                lambda: self._retry_serial_once(retry_port, retry_baud),
            )

    def _retry_serial_once(self, port: str, baudrate: int):
        """Un solo reintento automático si el VCP quedó bloqueado."""
        self._serial_retry_armed = False
        if self.serial_thread and self.serial_thread.ser and self.serial_thread.ser.is_open:
            return
        logger.info("Reintento automático de apertura serial: %s @ %s", port, baudrate)
        self._on_serial_reconnect(port, baudrate, allow_retry=False)
    
    def _get_sensor_adc(self, key: str):
        """ADC de sensor para el CONTROL (plano máquina).

        Fuente primaria: SensorBuffer (llenado en el hilo RX a tasa completa,
        independiente del refresco de UI). Fallback: label (compatibilidad).
        """
        buf = getattr(self, 'sensor_buffer', None)
        if buf is not None:
            adc = buf.get_adc(key)
            if adc is not None:
                return int(adc)
        # Fallback legacy: texto del label (solo si el buffer aún no tiene dato)
        labels = getattr(self.control_tab, 'value_labels', {})
        if key not in labels:
            return None
        text = str(labels[key].text()).strip()
        if not text or text == '---':
            return None
        try:
            return int(float(text))
        except ValueError:
            return None

    def _ui_refresh_due(self) -> bool:
        """True si toca refrescar la UI (~UI_REFRESH_HZ).

        Desacopla el repintado de la tasa de telemetría: la medida (SensorBuffer)
        y la grabación van a tasa completa; los widgets/plots a ~30 Hz.
        """
        from config.constants import UI_REFRESH_HZ
        now = time.perf_counter()
        if (now - self._last_ui_update_mono) >= (1.0 / float(UI_REFRESH_HZ)):
            self._last_ui_update_mono = now
            return True
        return False

    def update_data(self, line):
        """
        PROCESAMIENTO de telemetría STM32/Arduino con VALIDACIÓN.
        Formato LEGACY: pot_a,pot_b,sens_1,sens_2 (4 enteros CSV)
        Formato STM32: pot_a,pot_b,sens_1,sens_2,estado,settled (6 campos)
        Descarta líneas corruptas; sensores 12-bit en ruta de 6 campos.

        Nota (Fase 1): el SensorBuffer ya se llena en el hilo RX
        (SerialHandler) a tasa completa. Aquí solo se hace grabación
        (tasa completa) y refresco de UI (~30 Hz vía _ui_refresh_due()).
        """
        from config.constants import ADC_MAX

        if line.startswith(("ERROR:", "INFO:", "Potencia")):
            logger.info(line)
            return

        try:
            parts = line.split(',')
            adc_hi = int(ADC_MAX)

            if len(parts) >= 6:
                parsed_data = MotorProtocol.parse_sensor_data_with_status(line)
                if not parsed_data:
                    logger.debug(f"Línea telemetría no parseable: {line}")
                    return

                pot_a = parsed_data['pot_a']
                pot_b = parsed_data['pot_b']
                sens_1 = parsed_data['sens_1']
                sens_2 = parsed_data['sens_2']

                if not (-255 <= pot_a <= 255 and -255 <= pot_b <= 255 and
                        0 <= sens_1 <= adc_hi and 0 <= sens_2 <= adc_hi):
                    logger.debug(f"Datos fuera de rango (descartados): {line}")
                    return

                # Grabación: tasa COMPLETA (dato crudo, no UI)
                if self.data_recorder.is_recording:
                    self.data_recorder.write_data_point(pot_a, pot_b, sens_1, sens_2)

                # UI ~30 Hz: labels + plots (no interfiere en medida/control)
                if self._ui_refresh_due():
                    self.control_tab.update_motor_values(pot_a, pot_b)
                    self.control_tab.update_sensor_values(sens_1, sens_2)
                    self.control_tab.update_arduino_status(parsed_data['state'], parsed_data['settled'])
                    if self.signal_window and self.signal_window.isVisible():
                        self.signal_window.update_data(pot_a, pot_b, sens_1, sens_2)

            elif len(parts) == 4:
                pot_a, pot_b, sens_1, sens_2 = map(int, parts)

                if not (-255 <= pot_a <= 255 and -255 <= pot_b <= 255 and
                       0 <= sens_1 <= adc_hi and 0 <= sens_2 <= adc_hi):
                    logger.debug(f"Datos LEGACY fuera de rango: {line}")
                    return

                # Grabación: tasa COMPLETA (dato crudo, no UI)
                if self.data_recorder.is_recording:
                    self.data_recorder.write_data_point(pot_a, pot_b, sens_1, sens_2)

                # UI ~30 Hz: labels + plots (no interfiere en medida/control)
                if self._ui_refresh_due():
                    self.control_tab.update_motor_values(pot_a, pot_b)
                    self.control_tab.update_sensor_values(sens_1, sens_2)
                    self.control_tab.update_arduino_status("LEGACY", False)
                    if self.signal_window and self.signal_window.isVisible():
                        self.signal_window.update_data(pot_a, pot_b, sens_1, sens_2)

            else:
                logger.debug(f"Formato inválido ({len(parts)} campos), descartado: {line}")
                return

        except (ValueError, IndexError) as e:
            logger.debug(f"Error parseando datos (descartado): '{line}' - {e}")
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
        try:
            self._persist_analysis_result(results)
            self._save_session_state()
        except Exception as e:
            logger.error(f"Error persistiendo resultado de análisis: {e}")

    def _persist_analysis_result(self, results: dict):
        """Guarda en sesión el contexto y resultado del último análisis."""
        context = results.get('analysis_context') or self.analysis_tab.get_current_analysis_context()
        if not isinstance(context, dict):
            return

        slot_key = context.get('slot_key') or SessionStore.slot_key(
            context.get('motor', 'A'), context.get('sensor', '1')
        )

        payload = {
            **context,
            "last_identification": {
                "K": float(results.get("K", 0.0)),
                "tau": float(results.get("tau", 0.0)),
                "tau_slow": float(results.get("tau_slow", 1000.0)),
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "v_ss": float(results.get("v_ss", 0.0)),
                "U": float(results.get("U", 0.0)),
                "calibration_msg": results.get("calibracion_msg", ""),
            },
        }
        self.session_store.set_analysis_slot(slot_key, payload)
        self.session_store.set_identified_functions(
            self.tf_analyzer.get_identified_functions_serializable()
        )

    def _persist_hinf_state(self):
        """Guarda snapshot H∞ actual en sesión."""
        snapshot = self.hinf_tab.build_hinf_snapshot()
        if snapshot:
            slot_key = snapshot.get("slot_key", self.hinf_tab.get_active_slot_key())
            self.session_store.set_hinf_slot(slot_key, snapshot)

    def _persist_test_state(self):
        """Guarda controladores transferidos y preferencias de TestTab."""
        controllers = self.test_tab.get_serializable_controllers()
        self.session_store.set_test_controller("A", controllers.get("A"))
        self.session_store.set_test_controller("B", controllers.get("B"))
        sensor_map, invert_map = self.test_tab.get_controller_preferences()
        self.session_store.set_test_preferences(sensor_map, invert_map)

    def _save_session_state(self):
        """Guarda estado global de sesión en disco."""
        context = self.analysis_tab.get_current_analysis_context()
        if isinstance(context, dict):
            slot_key = context.get("slot_key") or SessionStore.slot_key(
                context.get("motor", "A"), context.get("sensor", "1")
            )
            current_slots = self.session_store.get_session().get("analysis", {}).get("slots", {})
            previous = current_slots.get(slot_key, {})
            payload = {**context}
            if "last_identification" in previous:
                payload["last_identification"] = previous["last_identification"]
            self.session_store.set_analysis_slot(slot_key, payload)

        self._persist_hinf_state()
        self._persist_test_state()
        self.session_store.set_identified_functions(
            self.tf_analyzer.get_identified_functions_serializable()
        )
        self.session_store.save()

    def _load_session_state(self):
        """Restaura estado persistido al iniciar la aplicación."""
        data = self.session_store.load()
        analysis_data = data.get("analysis", {})
        hinf_data = data.get("hinf", {})
        test_data = data.get("test", {})

        # 1) Restaurar funciones de transferencia identificadas
        identified = analysis_data.get("identified_functions", [])
        self.tf_analyzer.restore_identified_functions(identified)
        self.analysis_tab.update_tf_list()

        # 2) Restaurar contexto de análisis del slot más relevante
        slots = analysis_data.get("slots", {})
        selected_slot = hinf_data.get("last_slot")
        if not selected_slot and isinstance(slots, dict) and slots:
            selected_slot = next(iter(slots.keys()))
        if selected_slot and selected_slot in slots:
            self.analysis_tab.apply_analysis_context(slots[selected_slot])
            csv_path = slots[selected_slot].get("csv_path")
            if csv_path and not Path(csv_path).exists():
                logger.warning(f"CSV de sesión no encontrado: {csv_path}")

        # 3) Restaurar modelo H∞ del último slot
        hinf_slots = hinf_data.get("slots", {})
        if selected_slot and selected_slot in hinf_slots:
            restored = self.hinf_tab.apply_hinf_snapshot(hinf_slots[selected_slot])
            if restored:
                logger.info(f"Modelo H∞ restaurado para slot {selected_slot}")
        elif isinstance(hinf_slots, dict) and hinf_slots:
            first_slot = next(iter(hinf_slots.keys()))
            restored = self.hinf_tab.apply_hinf_snapshot(hinf_slots[first_slot])
            if restored:
                logger.info(f"Modelo H∞ restaurado para slot {first_slot}")

        # 4) Restaurar preferencias y controladores transferidos
        self.test_tab.apply_controller_preferences(
            test_data.get("sensor_map", {}),
            test_data.get("invert_map", {}),
        )
        controllers = test_data.get("controllers", {})
        hinf_slots = hinf_data.get("slots", {}) if isinstance(hinf_data, dict) else {}

        # Si A y B quedaron idénticos por el bug de transferencia "ambos"/default-B,
        # recuperar desde slots H∞ distintos (A_* / B_*).
        ctrl_a = controllers.get("A") if isinstance(controllers.get("A"), dict) else None
        ctrl_b = controllers.get("B") if isinstance(controllers.get("B"), dict) else None
        same_ab = (
            isinstance(ctrl_a, dict)
            and isinstance(ctrl_b, dict)
            and abs(float(ctrl_a.get("Kp", 0)) - float(ctrl_b.get("Kp", 0))) < 1e-9
            and abs(float(ctrl_a.get("Ki", 0)) - float(ctrl_b.get("Ki", 0))) < 1e-9
        )
        if same_ab and isinstance(hinf_slots, dict) and hinf_slots:
            slot_a = next((k for k in sorted(hinf_slots) if str(k).upper().startswith("A")), None)
            slot_b = next((k for k in sorted(hinf_slots) if str(k).upper().startswith("B")), None)
            if slot_a and slot_b:
                data_a = self.hinf_tab._controller_data_from_hinf_snapshot(hinf_slots[slot_a])
                data_b = self.hinf_tab._controller_data_from_hinf_snapshot(hinf_slots[slot_b])
                if abs(data_a["Kp"] - data_b["Kp"]) > 1e-9 or abs(data_a["Ki"] - data_b["Ki"]) > 1e-9:
                    logger.warning(
                        "Test controllers A/B idénticos en sesión; "
                        f"restaurando desde slots {slot_a} / {slot_b}"
                    )
                    self.hinf_tab._push_controller_to_test("A", data_a)
                    self.hinf_tab._push_controller_to_test("B", data_b)
                    self._persist_test_state()
                    self.session_store.save()
                    controllers = {}  # ya aplicados

        if isinstance(controllers.get("A"), dict):
            self.test_tab.set_controller("A", controllers["A"])
        if isinstance(controllers.get("B"), dict):
            self.test_tab.set_controller("B", controllers["B"])
        if controllers:
            logger.info("Controladores de TestTab restaurados desde sesión")
    
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
        """Encola comando al MCU (SerialTxQueue: coalesce A,*, prioridad F/I/N/B)."""
        # El log INFO de control lo hace SerialHandler al escribir al puerto.
        if not self.serial_thread.send_command(command):
            logger.error("Error: Puerto serial no está abierto. Comando no enviado.")

    # --- NUEVOS HANDLERS PARA POSITION HOLD ---
    
    def _on_position_hold(self, sensor1_target: int, sensor2_target: int):
        """Position Hold no está implementado en firmware STM32 (H,s1,s2)."""
        logger.warning(
            "Position Hold ignorado: firmware STM32 no soporta H,<s1>,<s2> "
            f"(pedido S1={sensor1_target}, S2={sensor2_target}). Usar control PC vía A,<pwm>."
        )
    
    def _on_brake(self):
        """Maneja solicitud de freno activo desde ControlTab."""
        logger.info("=== FRENO ACTIVO SOLICITADO ===")
        command = MotorProtocol.format_brake_command()
        self.send_command(command)
    
    def _on_settling_config(self, threshold: int):
        """Settling config no está implementado en firmware STM32 (S,threshold)."""
        logger.warning(
            f"Configuración S,{threshold} ignorada: firmware STM32 no soporta comando S. "
            "Settling se gestiona en PC (use_arduino_settled=false)."
        )

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
        # Overlay: solo 1ª línea (dumps de tabla BPoF son multilínea)
        overlay = (message.splitlines()[0] if message else "").strip()
        if self.camera_tab.camera_view_window and overlay:
            self.camera_tab.camera_view_window.set_autofocus_status(overlay)
        # Log/terminal GUI: mensaje completo (incluye lista de candidatos)
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
        
        # Limpiar estado de autofoco en visualización
        if hasattr(self.camera_tab, 'saliency_widget') and self.camera_tab.saliency_widget:
            self.camera_tab.saliency_widget.clear_autofocus_state()

        # Microscopía recibe los frames ya capturados; el piezo volvió al origen.
        if hasattr(self, 'microscopy_service') and self.microscopy_service.is_running():
            self.microscopy_service.handle_autofocus_complete(results)
            return

        # Autofoco manual: los N planos ya están en memoria. Guardarlos todos;
        # no sustituirlos por una captura única posterior.
        if results:
            saved = self.camera_tab.save_manual_autofocus_stacks(results)
            expected_per_object = int(
                getattr(self.autofocus_service, "n_captures", 0) or 0
            )
            expected_total = expected_per_object * len(results)
            if saved != expected_total:
                self.camera_tab.log_message(
                    f"❌ AF manual: se esperaban {expected_total} archivos "
                    f"({expected_per_object} por objeto) y se guardaron {saved}"
                )
            else:
                self.camera_tab.log_message(
                    f"📸 AF manual: {saved}/{expected_total} planos guardados"
                )
        if hasattr(self.camera_tab, '_pending_capture'):
            self.camera_tab._pending_capture = False
        orchestrator = getattr(self.camera_tab, "orchestrator", None)
        if orchestrator is not None:
            orchestrator.clear_pending_capture()

        if self.cfocus_enabled and self.cfocus_controller:
            current_z = self.cfocus_controller.read_z()
            if current_z is not None:
                self.camera_tab.log_message(
                    f"📍 Posición Z actual: {current_z:.1f}µm "
                    "(origen calibrado)"
                )
        if results:
            self.camera_tab.log_message(
                f"🎯 BPoF calculado: {results[0].z_optimal:.1f}µm"
            )

        self.camera_tab.log_message(
            f"✅ Autofoco completado: {n_results} objetos enfocados"
        )
    
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
        n = int(getattr(self.camera_tab, "_trajectory_n_points", 0) or 0)
        if n > 0:
            self.camera_tab.set_trajectory_status(True, n)
            if self.camera_tab.resume_point_spin is not None:
                self.camera_tab.resume_point_spin.setValue(1)
            self.camera_tab._update_resume_button_label()
        else:
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
        elif self.cfocus_controller.is_connected:
            self.camera_tab.log_message("✅ C-Focus: ya conectado")
            return True
        else:
            # Reintento limpio tras fallo previo (handle sticky / USB en uso)
            try:
                self.cfocus_controller.disconnect()
            except Exception:
                pass
        
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
                calib_info = self.cfocus_controller.get_calibration_info()
                hw_range = float(calib_info.get('z_range_hw', 0.0) or 0.0)
                span = float(result.get('z_range', 0.0) or 0.0)
                span_ratio = (span / hw_range) if hw_range > 0 else 0.0

                msg = (f"✅ Calibración completada:\n"
                       f"   Mín: {result['z_min']:.2f} µm\n"
                       f"   Máx: {result['z_max']:.2f} µm\n"
                       f"   Centro: {result['z_center']:.2f} µm\n"
                       f"   Rango: {result['z_range']:.2f} µm\n"
                       f"   Rango HW: {hw_range:.2f} µm")
                self.camera_tab.log_message(msg)
                logger.info(f"[Main] Calibración exitosa: {result}")
                logger.info(
                    f"[Main] Verificación calibración -> hw_range={hw_range:.2f}µm, "
                    f"span={span:.2f}µm, span_ratio={span_ratio:.3f}"
                )
                if hw_range > 0 and span_ratio < 0.6:
                    warn_msg = (
                        "⚠️ Calibración con span útil bajo respecto al rango hardware. "
                        "Verificar topes mecánicos/referencias."
                    )
                    self.camera_tab.log_message(warn_msg)
                    logger.warning(f"[Main] {warn_msg}")
                
                # Cablear hardware y reaplicar params desde UI/JSON (única vía)
                if self.initialize_autofocus():
                    self.camera_tab.sync_runtime_params_from_ui()
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

        # Configurar AutofocusService: única vía CMOS = acquire_scientific_frame
        self.autofocus_service.configure(
            cfocus_controller=self.cfocus_controller,
            get_exposure_s_callback=lambda: float(
                getattr(worker, "exposure", 0.1) or 0.1
            ),
            # CRÍTICO: contador de grabs LIVE (frame_count). NO usar
            # current_raw_frame_count: solo avanza al acquire científico y el
            # flush OPTICAL se queda en timeout eterno → S=0 en todo el COARSE.
            get_frame_count_callback=lambda: int(
                getattr(worker, "frame_count", 0) or 0
            ),
            acquire_scientific_frame_callback=(
                lambda timeout_s=2.0: self.camera_service.acquire_scientific_frame(
                    timeout_s=timeout_s
                )
            ),
        )
        
        self.camera_tab.log_message("✅ Autofoco configurado (U2-Net + C-Focus)")
        logger.info("AutofocusService configurado con C-Focus y cámara")
        return True
    
    
    def closeEvent(self, event):
        """Maneja el cierre de la aplicación."""
        logger.info("=== CERRANDO APLICACIÓN ===")
        try:
            self._save_session_state()
        except Exception as e:
            logger.error(f"No se pudo guardar sesión al cerrar: {e}")
        try:
            self.camera_tab.save_camera_tab_settings()
        except Exception as e:
            logger.error(f"No se pudieron guardar opciones de cámara al cerrar: {e}")
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
    logger.info("INICIANDO CTRL_GUI — MycoViT XY (STM32F767ZI)")
    logger.info(f"Versión: 2.3 | Puerto: {SERIAL_PORT} | Baudrate: {BAUD_RATE}")
    logger.info("="*70)
    
    try:
        app = QApplication(sys.argv)
        logger.info("QApplication creada exitosamente")
        
        window = CTRL_GUI()
        logger.info("Ventana principal CTRL_GUI creada")
        
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


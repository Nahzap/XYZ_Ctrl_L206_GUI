"""
Pestaña de Prueba de Controladores y Trayectorias.

Encapsula la UI para prueba de controladores H∞ y generación de trayectorias.
Usa TrajectoryGenerator para la lógica de trayectorias.
Usa TestService para la lógica de control (separación GUI/lógica).

MEJORAS 2025-12-17:
- Calibración dinámica desde config/constants.py
- Zona muerta reducida (DEADZONE_ADC)
- Verificación de settling antes de avanzar
- Tolerancia de posición configurable

REFACTORIZACIÓN 2025-12-17:
- Lógica de control movida a TestService
- TestTab solo contiene GUI y actualización de UI
- Comunicación por señales PyQt
"""

import csv
import logging
import time
import numpy as np
from typing import Optional

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QGroupBox, QLabel, QLineEdit, QPushButton,
                             QTextEdit, QCheckBox, QComboBox, QScrollArea,
                             QRadioButton, QMessageBox, QFrame, QFileDialog,
                             QDialog, QTableWidget, QTableWidgetItem, 
                             QSplitter, QHeaderView)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from config.constants import (
    CALIBRATION_X, CALIBRATION_Y, 
    DEADZONE_ADC, POSITION_TOLERANCE_UM, SETTLING_CYCLES,
    MAX_ATTEMPTS_PER_POINT, FALLBACK_TOLERANCE_MULTIPLIER,
    um_to_adc, adc_to_um, reload_calibration, get_calibration_info
)
from core.services.test_service import TestService, ControllerConfig, TrajectoryConfig

logger = logging.getLogger('MotorControl_L206')


class TestTab(QWidget):
    """
    Pestaña para prueba de controladores y ejecución de trayectorias.
    
    Signals:
        dual_control_start_requested: Solicita iniciar control dual (ref_a, ref_b)
        dual_control_stop_requested: Solicita detener control dual
        trajectory_generate_requested: Solicita generar trayectoria (config dict)
        trajectory_preview_requested: Solicita vista previa de trayectoria
        zigzag_start_requested: Solicita iniciar ejecución zig-zag
        zigzag_stop_requested: Solicita detener ejecución zig-zag
        controller_clear_requested: Solicita limpiar controlador (motor: 'A' o 'B')
    """
    
    dual_control_start_requested = pyqtSignal(float, float)  # ref_a, ref_b
    dual_control_stop_requested = pyqtSignal()
    trajectory_generate_requested = pyqtSignal(dict)
    trajectory_preview_requested = pyqtSignal()
    zigzag_start_requested = pyqtSignal()
    zigzag_stop_requested = pyqtSignal()
    controller_clear_requested = pyqtSignal(str)  # 'A' or 'B'
    trajectory_changed = pyqtSignal(int)  # n_points - emitido cuando cambia la trayectoria
    
    def __init__(self, trajectory_generator=None, parent=None):
        """
        Inicializa la pestaña de prueba.
        
        Args:
            trajectory_generator: Instancia de TrajectoryGenerator
            parent: Widget padre (ArduinoGUI)
        """
        super().__init__(parent)
        self.trajectory_gen = trajectory_generator
        self.parent_gui = parent
        
        # Callbacks de hardware (inyección de dependencias)
        self.send_command_callback = None
        self.get_sensor_value_callback = None
        self.get_mode_label_callback = None
        
        # === SERVICIO DE CONTROL (NUEVA ARQUITECTURA) ===
        self.test_service = TestService(parent=self)
        self._connect_service_signals()
        
        # Controladores transferidos (datos para UI)
        self.controller_a = None
        self.controller_b = None
        self._has_controller_a = False
        self._has_controller_b = False
        
        # Variables de control dual (para compatibilidad con código existente)
        self.dual_control_active = False
        self.dual_control_timer = None
        self.dual_integral_a = 0.0
        self.dual_integral_b = 0.0
        self.dual_last_time = None
        
        # Variables de trayectoria
        self.current_trajectory = None
        self.trajectory_index = 0
        self.trajectory_timer = None
        self.trajectory_active = False
        
        # Calibración
        self.calibration_data = None
        
        self._setup_ui()
        logger.debug("TestTab inicializado con TestService")
    
    def _connect_service_signals(self):
        """Conecta las señales del TestService con los métodos de actualización de UI."""
        # Control dual
        self.test_service.dual_control_started.connect(self._on_dual_control_started)
        self.test_service.dual_control_stopped.connect(self._on_dual_control_stopped)
        self.test_service.dual_position_update.connect(self._on_dual_position_update)
        self.test_service.dual_position_reached.connect(self._on_dual_position_reached)
        self.test_service.dual_position_lost.connect(self._on_dual_position_lost)
        
        # Trayectoria
        self.test_service.trajectory_started.connect(self._on_trajectory_started)
        self.test_service.trajectory_stopped.connect(self._on_trajectory_stopped)
        self.test_service.trajectory_completed.connect(self._on_trajectory_completed)
        self.test_service.trajectory_point_reached.connect(self._on_trajectory_point_reached)
        self.test_service.trajectory_feedback.connect(self._on_trajectory_feedback)
        
        # General
        self.test_service.log_message.connect(self._on_log_message)
        self.test_service.error_occurred.connect(self._on_error_occurred)
    
    def set_hardware_callbacks(self, send_command, get_sensor_value, get_mode_label):
        """
        Configura callbacks de hardware para control en tiempo real.
        
        Args:
            send_command: Función para enviar comandos al Arduino
            get_sensor_value: Función para leer valor de sensor
            get_mode_label: Función para obtener/modificar label de modo
        """
        self.send_command_callback = send_command
        self.get_sensor_value_callback = get_sensor_value
        self.get_mode_label_callback = get_mode_label
        
        # Configurar también el servicio
        self.test_service.set_hardware_callbacks(send_command, get_sensor_value)
        
        logger.debug("Callbacks de hardware configurados en TestTab y TestService")
    
    def _setup_ui(self):
        """Configura la interfaz de usuario."""
        main_layout = QVBoxLayout(self)
        
        # Scroll area para contenido extenso
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        
        scroll_widget = QWidget()
        layout = QVBoxLayout(scroll_widget)
        
        # Sección 1: Controladores H∞ Transferidos
        controllers_group = self._create_controllers_section()
        layout.addWidget(controllers_group)
        
        # Sección 2: Asignación Motor-Sensor
        motor_sensor_group = self._create_motor_sensor_section()
        layout.addWidget(motor_sensor_group)
        
        # Sección 3: Calibración
        calibration_group = self._create_calibration_section()
        layout.addWidget(calibration_group)
        
        # Sección 4: Control por Posición
        position_group = self._create_position_control_section()
        layout.addWidget(position_group)
        
        # Sección 5: Generador de Trayectorias
        trajectory_group = self._create_trajectory_section()
        layout.addWidget(trajectory_group)
        
        # Sección 6: Ejecución Zig-Zag
        zigzag_group = self._create_zigzag_section()
        layout.addWidget(zigzag_group)
        
        # Área de resultados
        self.results_text = QTextEdit()
        self.results_text.setReadOnly(True)
        self.results_text.setPlaceholderText("Los resultados aparecerán aquí...")
        self.results_text.setMinimumHeight(100)
        self.results_text.setMaximumHeight(150)
        self.results_text.setStyleSheet(
            "font-family: 'Consolas', monospace; font-size: 12px; "
            "background-color: white; color: black;"
        )
        layout.addWidget(self.results_text)
        
        scroll.setWidget(scroll_widget)
        main_layout.addWidget(scroll)
    
    def _create_controllers_section(self):
        """Crea sección de controladores transferidos."""
        group = QGroupBox("📦 Controladores H∞ Transferidos")
        layout = QVBoxLayout()
        
        # Motor A
        motor_a_frame = QFrame()
        motor_a_frame.setFrameStyle(QFrame.Box | QFrame.Raised)
        motor_a_layout = QVBoxLayout()
        
        header_a = QHBoxLayout()
        self.motor_a_label = QLabel("<b>Motor A (X)</b>")
        header_a.addWidget(self.motor_a_label)
        header_a.addStretch()
        self.motor_a_status = QLabel("⚪ Sin controlador")
        self.motor_a_status.setStyleSheet("color: #95A5A6;")
        header_a.addWidget(self.motor_a_status)
        motor_a_layout.addLayout(header_a)
        
        self.motor_a_info = QTextEdit()
        self.motor_a_info.setReadOnly(True)
        self.motor_a_info.setMaximumHeight(70)
        self.motor_a_info.setStyleSheet("font-family: monospace; font-size: 11px; background: white; color: black;")
        self.motor_a_info.setPlaceholderText("Transfiere un controlador desde 'H∞ Synthesis'...")
        motor_a_layout.addWidget(self.motor_a_info)
        
        btn_a = QHBoxLayout()
        self.clear_a_btn = QPushButton("🗑️ Limpiar")
        self.clear_a_btn.clicked.connect(lambda: self.controller_clear_requested.emit('A'))
        self.clear_a_btn.setEnabled(False)
        btn_a.addWidget(self.clear_a_btn)
        btn_a.addStretch()
        motor_a_layout.addLayout(btn_a)
        
        motor_a_frame.setLayout(motor_a_layout)
        layout.addWidget(motor_a_frame)
        
        # Motor B
        motor_b_frame = QFrame()
        motor_b_frame.setFrameStyle(QFrame.Box | QFrame.Raised)
        motor_b_layout = QVBoxLayout()
        
        header_b = QHBoxLayout()
        self.motor_b_label = QLabel("<b>Motor B (Y)</b>")
        header_b.addWidget(self.motor_b_label)
        header_b.addStretch()
        self.motor_b_status = QLabel("⚪ Sin controlador")
        self.motor_b_status.setStyleSheet("color: #95A5A6;")
        header_b.addWidget(self.motor_b_status)
        motor_b_layout.addLayout(header_b)
        
        self.motor_b_info = QTextEdit()
        self.motor_b_info.setReadOnly(True)
        self.motor_b_info.setMaximumHeight(70)
        self.motor_b_info.setStyleSheet("font-family: monospace; font-size: 11px; background: white; color: black;")
        self.motor_b_info.setPlaceholderText("Transfiere un controlador desde 'H∞ Synthesis'...")
        motor_b_layout.addWidget(self.motor_b_info)
        
        btn_b = QHBoxLayout()
        self.clear_b_btn = QPushButton("🗑️ Limpiar")
        self.clear_b_btn.clicked.connect(lambda: self.controller_clear_requested.emit('B'))
        self.clear_b_btn.setEnabled(False)
        btn_b.addWidget(self.clear_b_btn)
        btn_b.addStretch()
        motor_b_layout.addLayout(btn_b)
        
        motor_b_frame.setLayout(motor_b_layout)
        layout.addWidget(motor_b_frame)
        
        group.setLayout(layout)
        return group
    
    def _create_motor_sensor_section(self):
        """Crea sección de asignación motor-sensor."""
        group = QGroupBox("🔧 Asignación Motor ↔ Sensor")
        layout = QVBoxLayout()
        
        # Motor A
        row_a = QHBoxLayout()
        row_a.addWidget(QLabel("<b>Motor A lee:</b>"))
        self.motor_a_sensor1 = QCheckBox("Sensor 1")
        self.motor_a_sensor2 = QCheckBox("Sensor 2")
        self.motor_a_sensor1.toggled.connect(lambda c: self.motor_a_sensor2.setChecked(False) if c else None)
        self.motor_a_sensor2.toggled.connect(lambda c: self.motor_a_sensor1.setChecked(False) if c else None)
        row_a.addWidget(self.motor_a_sensor1)
        row_a.addWidget(self.motor_a_sensor2)
        self.motor_a_invert = QCheckBox("⇄ Invertir PWM")
        row_a.addWidget(self.motor_a_invert)
        row_a.addStretch()
        layout.addLayout(row_a)
        
        # Motor B
        row_b = QHBoxLayout()
        row_b.addWidget(QLabel("<b>Motor B lee:</b>"))
        self.motor_b_sensor1 = QCheckBox("Sensor 1")
        self.motor_b_sensor2 = QCheckBox("Sensor 2")
        self.motor_b_sensor1.toggled.connect(lambda c: self.motor_b_sensor2.setChecked(False) if c else None)
        self.motor_b_sensor2.toggled.connect(lambda c: self.motor_b_sensor1.setChecked(False) if c else None)
        row_b.addWidget(self.motor_b_sensor1)
        row_b.addWidget(self.motor_b_sensor2)
        self.motor_b_invert = QCheckBox("⇄ Invertir PWM")
        row_b.addWidget(self.motor_b_invert)
        row_b.addStretch()
        layout.addLayout(row_b)
        
        info = QLabel("⚠️ Configura sensor e inversión ANTES de iniciar control.")
        info.setStyleSheet("padding: 5px; background: #FFF3CD; border: 1px solid #FFC107; border-radius: 3px;")
        layout.addWidget(info)
        
        group.setLayout(layout)
        return group
    
    def _create_calibration_section(self):
        """Crea sección de calibración con información dinámica desde calibration.json."""
        group = QGroupBox("📏 Calibración del Sistema")
        layout = QVBoxLayout()
        
        info = QLabel("<b>ℹ️ Calibración automática desde 'Análisis' → calibration.json</b>")
        info.setStyleSheet("padding: 8px; background: #34495E; border-radius: 5px;")
        layout.addWidget(info)
        
        self.calibration_status = QLabel("⚪ Cargando calibración...")
        self.calibration_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #95A5A6;")
        layout.addWidget(self.calibration_status)
        
        self.calibration_details = QTextEdit()
        self.calibration_details.setReadOnly(True)
        self.calibration_details.setMaximumHeight(80)
        self.calibration_details.setStyleSheet("font-family: monospace; font-size: 11px; background: white; color: black;")
        layout.addWidget(self.calibration_details)
        
        # Botón para recargar calibración
        btn_layout = QHBoxLayout()
        self.reload_cal_btn = QPushButton("🔄 Recargar Calibración")
        self.reload_cal_btn.setStyleSheet("padding: 5px;")
        self.reload_cal_btn.clicked.connect(self._reload_calibration)
        btn_layout.addWidget(self.reload_cal_btn)
        btn_layout.addStretch()
        layout.addLayout(btn_layout)
        
        group.setLayout(layout)
        
        # Cargar calibración inicial
        self._update_calibration_display()
        
        return group
    
    def _create_position_control_section(self):
        """Crea sección de control por posición."""
        group = QGroupBox("📍 Control por Posición")
        layout = QGridLayout()
        
        layout.addWidget(QLabel("Referencia Motor A (µm):"), 0, 0)
        self.ref_a_input = QLineEdit("15000")
        self.ref_a_input.setFixedWidth(100)
        layout.addWidget(self.ref_a_input, 0, 1)
        
        layout.addWidget(QLabel("Referencia Motor B (µm):"), 0, 2)
        self.ref_b_input = QLineEdit("15000")
        self.ref_b_input.setFixedWidth(100)
        layout.addWidget(self.ref_b_input, 0, 3)
        
        btn_layout = QHBoxLayout()
        self.start_dual_btn = QPushButton("▶️ Iniciar Control Dual")
        self.start_dual_btn.setStyleSheet("font-weight: bold; padding: 8px; background: #27AE60;")
        self.start_dual_btn.clicked.connect(self._start_dual_control)
        self.start_dual_btn.setEnabled(False)  # Deshabilitado hasta que haya controlador
        btn_layout.addWidget(self.start_dual_btn)
        
        self.stop_dual_btn = QPushButton("⏹️ Detener Control")
        self.stop_dual_btn.setStyleSheet("font-weight: bold; padding: 8px; background: #E74C3C;")
        self.stop_dual_btn.setEnabled(False)
        self.stop_dual_btn.clicked.connect(self.stop_dual_control)  # Conectar directamente
        btn_layout.addWidget(self.stop_dual_btn)
        
        layout.addLayout(btn_layout, 1, 0, 1, 4)
        
        group.setLayout(layout)
        return group
    
    def _create_trajectory_section(self):
        """Crea sección de generación de trayectorias."""
        group = QGroupBox("🔀 Generador de Trayectorias Zig-Zag")
        layout = QGridLayout()
        
        layout.addWidget(QLabel("Número de puntos:"), 0, 0)
        self.points_input = QLineEdit("100")
        self.points_input.setFixedWidth(80)
        layout.addWidget(self.points_input, 0, 1)
        
        layout.addWidget(QLabel("X inicio (µm):"), 0, 2)
        self.x_start_input = QLineEdit("10000")
        self.x_start_input.setFixedWidth(80)
        layout.addWidget(self.x_start_input, 0, 3)
        
        layout.addWidget(QLabel("X fin (µm):"), 1, 0)
        self.x_end_input = QLineEdit("20000")
        self.x_end_input.setFixedWidth(80)
        layout.addWidget(self.x_end_input, 1, 1)
        
        layout.addWidget(QLabel("Y inicio (µm):"), 1, 2)
        self.y_start_input = QLineEdit("10000")
        self.y_start_input.setFixedWidth(80)
        layout.addWidget(self.y_start_input, 1, 3)
        
        layout.addWidget(QLabel("Y fin (µm):"), 2, 0)
        self.y_end_input = QLineEdit("20000")
        self.y_end_input.setFixedWidth(80)
        layout.addWidget(self.y_end_input, 2, 1)
        
        layout.addWidget(QLabel("Delay (s):"), 2, 2)
        self.delay_input = QLineEdit("0.5")
        self.delay_input.setFixedWidth(80)
        layout.addWidget(self.delay_input, 2, 3)
        
        btn_layout = QHBoxLayout()
        gen_btn = QPushButton("🎯 Generar Trayectoria")
        gen_btn.setStyleSheet("background: #3498DB; font-weight: bold;")
        gen_btn.clicked.connect(self._generate_trajectory)
        btn_layout.addWidget(gen_btn)
        
        preview_btn = QPushButton("👁️ Vista Previa")
        preview_btn.clicked.connect(self._preview_trajectory)
        btn_layout.addWidget(preview_btn)
        
        # Botones CSV
        export_btn = QPushButton("💾 Exportar CSV")
        export_btn.setToolTip("Guardar trayectoria en archivo CSV")
        export_btn.clicked.connect(self._export_trajectory_csv)
        btn_layout.addWidget(export_btn)
        
        import_btn = QPushButton("📂 Importar CSV")
        import_btn.setToolTip("Cargar trayectoria desde archivo CSV")
        import_btn.clicked.connect(self._import_trajectory_csv)
        btn_layout.addWidget(import_btn)
        
        layout.addLayout(btn_layout, 3, 0, 1, 4)
        
        group.setLayout(layout)
        return group
    
    def _create_zigzag_section(self):
        """Crea sección de ejecución zig-zag con feedback visual mejorado."""
        group = QGroupBox("🔬 Ejecución de Trayectoria Zig-Zag")
        layout = QVBoxLayout()
        
        # Estado de trayectoria
        status_layout = QHBoxLayout()
        status_layout.addWidget(QLabel("<b>Estado:</b>"))
        self.trajectory_status = QLabel("⚪ Sin trayectoria")
        self.trajectory_status.setStyleSheet("color: #95A5A6; font-weight: bold;")
        status_layout.addWidget(self.trajectory_status)
        status_layout.addStretch()
        layout.addLayout(status_layout)
        
        # Progreso de ejecución (NUEVO)
        progress_layout = QHBoxLayout()
        progress_layout.addWidget(QLabel("<b>Progreso:</b>"))
        self.trajectory_progress_label = QLabel("-- / --")
        self.trajectory_progress_label.setStyleSheet("font-weight: bold; color: #3498DB;")
        progress_layout.addWidget(self.trajectory_progress_label)
        progress_layout.addWidget(QLabel("<b>Punto actual:</b>"))
        self.current_point_label = QLabel("(---, ---) µm")
        self.current_point_label.setStyleSheet("font-family: monospace;")
        progress_layout.addWidget(self.current_point_label)
        progress_layout.addStretch()
        layout.addLayout(progress_layout)
        
        # Error actual (NUEVO)
        error_layout = QHBoxLayout()
        error_layout.addWidget(QLabel("<b>Error:</b>"))
        self.error_x_label = QLabel("X: --- µm")
        self.error_x_label.setStyleSheet("font-family: monospace; color: #E74C3C;")
        error_layout.addWidget(self.error_x_label)
        self.error_y_label = QLabel("Y: --- µm")
        self.error_y_label.setStyleSheet("font-family: monospace; color: #E74C3C;")
        error_layout.addWidget(self.error_y_label)
        self.settling_label = QLabel("Settling: --/--")
        self.settling_label.setStyleSheet("font-family: monospace; color: #F39C12;")
        error_layout.addWidget(self.settling_label)
        error_layout.addStretch()
        layout.addLayout(error_layout)
        
        # Parámetros
        params_layout = QHBoxLayout()
        params_layout.addWidget(QLabel("Tolerancia (µm):"))
        self.tolerance_input = QLineEdit(str(POSITION_TOLERANCE_UM))
        self.tolerance_input.setFixedWidth(80)
        self.tolerance_input.setToolTip(f"Error máximo permitido (desde calibration.json: {POSITION_TOLERANCE_UM}µm)")
        params_layout.addWidget(self.tolerance_input)
        params_layout.addWidget(QLabel("Pausa (s):"))
        self.pause_input = QLineEdit("2.0")
        self.pause_input.setFixedWidth(80)
        params_layout.addWidget(self.pause_input)
        params_layout.addStretch()
        layout.addLayout(params_layout)
        
        # Botones
        btn_layout = QHBoxLayout()
        self.zigzag_start_btn = QPushButton("🚀 Ejecutar Trayectoria")
        self.zigzag_start_btn.setStyleSheet("font-weight: bold; padding: 8px; background: #3498DB;")
        self.zigzag_start_btn.setEnabled(False)
        self.zigzag_start_btn.clicked.connect(self.start_trajectory_execution)
        btn_layout.addWidget(self.zigzag_start_btn)
        
        self.zigzag_stop_btn = QPushButton("⏹️ Detener")
        self.zigzag_stop_btn.setStyleSheet("font-weight: bold; padding: 8px; background: #E74C3C;")
        self.zigzag_stop_btn.setEnabled(False)
        self.zigzag_stop_btn.clicked.connect(self.stop_trajectory_execution)
        btn_layout.addWidget(self.zigzag_stop_btn)
        
        layout.addLayout(btn_layout)
        
        group.setLayout(layout)
        return group
    
    def _start_dual_control(self):
        """Inicia control dual - llama directamente al método de control."""
        logger.info("Botón 'Iniciar Control Dual' presionado")
        self.start_dual_control()  # Llamar directamente al método que hace el trabajo
    
    def _generate_trajectory(self):
        """Genera trayectoria con parámetros actuales usando TrajectoryGenerator."""
        logger.info("=== BOTÓN: Generar Trayectoria presionado ===")
        
        if not self.trajectory_gen:
            self.results_text.append("❌ Error: TrajectoryGenerator no disponible")
            logger.error("TrajectoryGenerator no disponible")
            return
        
        try:
            # Leer parámetros de la UI
            n_points = int(self.points_input.text())
            x_min = float(self.x_start_input.text())
            x_max = float(self.x_end_input.text())
            y_min = float(self.y_start_input.text())
            y_max = float(self.y_end_input.text())
            step_delay = float(self.delay_input.text())
            
            logger.info(f"Parámetros: {n_points} puntos, X=[{x_min},{x_max}], Y=[{y_min},{y_max}], delay={step_delay}s")
            
            # Generar trayectoria
            result = self.trajectory_gen.generate_zigzag_by_points(
                n_points=n_points,
                x_min=x_min,
                x_max=x_max,
                y_min=y_min,
                y_max=y_max,
                step_delay=step_delay
            )
            
            if result['success']:
                self.current_trajectory = result['points']
                self.trajectory_step_delay = step_delay
                self.trajectory_index = 0
                
                # Actualizar UI
                self.set_trajectory_status(True, len(self.current_trajectory))
                self.results_text.append(f"✅ {result['message']}")
                self.results_text.append(f"   Grid: {result['n_rows']}x{result['n_cols']}")
                
                # Guardar figura para vista previa
                self._trajectory_figure = result.get('figure')
                
                logger.info(f"✅ Trayectoria generada: {len(self.current_trajectory)} puntos")
            else:
                self.results_text.append(f"❌ {result['message']}")
                logger.error(f"Error generando trayectoria: {result['message']}")
                
        except ValueError as e:
            self.results_text.append(f"❌ Error: Valores inválidos - {e}")
            logger.error(f"Parámetros de trayectoria inválidos: {e}")
    
    def _export_trajectory_csv(self):
        """Exporta la trayectoria actual a un archivo CSV."""
        if self.current_trajectory is None or len(self.current_trajectory) == 0:
            self.results_text.append("❌ Error: No hay trayectoria para exportar")
            return
        
        filename, _ = QFileDialog.getSaveFileName(
            self, "Guardar Trayectoria CSV", 
            "trayectoria.csv", 
            "CSV Files (*.csv)"
        )
        
        if filename:
            try:
                with open(filename, 'w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    # Header
                    writer.writerow(['Punto', 'X_um', 'Y_um'])
                    # Data
                    for i, point in enumerate(self.current_trajectory):
                        writer.writerow([i+1, f"{point[0]:.2f}", f"{point[1]:.2f}"])
                
                self.results_text.append(f"✅ Trayectoria exportada: {filename}")
                self.results_text.append(f"   {len(self.current_trajectory)} puntos guardados")
                logger.info(f"Trayectoria exportada a {filename}")
            except Exception as e:
                self.results_text.append(f"❌ Error exportando: {e}")
                logger.error(f"Error exportando trayectoria: {e}")
    
    def _import_trajectory_csv(self):
        """Importa una trayectoria desde un archivo CSV."""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Cargar Trayectoria CSV", 
            "", 
            "CSV Files (*.csv);;All Files (*)"
        )
        
        if filename:
            try:
                points = []
                with open(filename, 'r', encoding='utf-8') as f:
                    reader = csv.reader(f)
                    header = next(reader)  # Skip header
                    
                    for row in reader:
                        if len(row) >= 3:
                            # Formato: Punto, X_um, Y_um
                            x = float(row[1])
                            y = float(row[2])
                            points.append([x, y])
                        elif len(row) >= 2:
                            # Formato simple: X, Y
                            x = float(row[0])
                            y = float(row[1])
                            points.append([x, y])
                
                if len(points) > 0:
                    self.current_trajectory = np.array(points)
                    self.trajectory_index = 0
                    
                    # Actualizar UI
                    self.set_trajectory_status(True, len(self.current_trajectory))
                    self.results_text.append(f"✅ Trayectoria importada: {filename}")
                    self.results_text.append(f"   {len(self.current_trajectory)} puntos cargados")
                    
                    # Mostrar rango
                    x_min, x_max = self.current_trajectory[:, 0].min(), self.current_trajectory[:, 0].max()
                    y_min, y_max = self.current_trajectory[:, 1].min(), self.current_trajectory[:, 1].max()
                    self.results_text.append(f"   Rango X: [{x_min:.0f}, {x_max:.0f}] µm")
                    self.results_text.append(f"   Rango Y: [{y_min:.0f}, {y_max:.0f}] µm")
                    
                    logger.info(f"Trayectoria importada desde {filename}: {len(points)} puntos")
                else:
                    self.results_text.append("❌ Error: El archivo CSV no contiene puntos válidos")
                    
            except Exception as e:
                self.results_text.append(f"❌ Error importando: {e}")
                logger.error(f"Error importando trayectoria: {e}")
    
    def _preview_trajectory(self):
        """Muestra vista previa de la trayectoria generada con gráfico XY."""
        logger.info("=== BOTÓN: Vista Previa presionado ===")
        
        if self.current_trajectory is None or len(self.current_trajectory) == 0:
            self.results_text.append("❌ Error: Genera una trayectoria primero")
            return
        
        # Crear ventana de vista previa
        dialog = QDialog(self)
        dialog.setWindowTitle("📊 Vista Previa de Trayectoria")
        dialog.setGeometry(100, 100, 1200, 700)
        dialog.setStyleSheet("background-color: #2E2E2E; color: white;")
        
        main_layout = QHBoxLayout()
        
        # === LADO IZQUIERDO: GRÁFICO XY ===
        fig = Figure(figsize=(8, 7), facecolor='#2E2E2E')
        ax = fig.add_subplot(111)
        
        x_coords = self.current_trajectory[:, 0]
        y_coords = self.current_trajectory[:, 1]
        
        # Trayectoria (línea azul)
        ax.plot(x_coords, y_coords, '-', color='#3498DB', linewidth=2, label='Trayectoria', zorder=1)
        
        # Puntos a visitar (puntos rojos)
        ax.scatter(x_coords, y_coords, c='red', s=50, zorder=2, label=f'Puntos ({len(x_coords)})')
        
        # Marcar inicio (verde) y fin (amarillo)
        ax.scatter(x_coords[0], y_coords[0], c='#2ECC71', s=150, marker='s', zorder=3, label='Inicio')
        ax.scatter(x_coords[-1], y_coords[-1], c='#F1C40F', s=150, marker='*', zorder=3, label='Fin')
        
        # Numerar algunos puntos clave
        for i in range(0, len(x_coords), max(1, len(x_coords)//10)):
            ax.annotate(f'{i+1}', (x_coords[i], y_coords[i]), fontsize=8, color='white',
                       xytext=(5, 5), textcoords='offset points')
        
        # Configurar ejes
        ax.set_xlabel('Posición X (µm)', color='white', fontsize=12)
        ax.set_ylabel('Posición Y (µm)', color='white', fontsize=12)
        ax.set_title(f'Trayectoria Zig-Zag - {len(self.current_trajectory)} puntos', 
                    fontsize=14, fontweight='bold', color='white')
        
        # Límites con margen
        x_min, x_max = x_coords.min(), x_coords.max()
        y_min, y_max = y_coords.min(), y_coords.max()
        margin_x = (x_max - x_min) * 0.1 if x_max > x_min else 500
        margin_y = (y_max - y_min) * 0.1 if y_max > y_min else 500
        ax.set_xlim(x_min - margin_x, x_max + margin_x)
        ax.set_ylim(y_min - margin_y, y_max + margin_y)
        
        # Estilo
        ax.set_facecolor('#1a1a1a')
        ax.tick_params(colors='white', labelsize=10)
        ax.grid(True, alpha=0.3, linestyle='--', color='#555555')
        ax.legend(loc='upper right', facecolor='#383838', edgecolor='#555555', 
                 labelcolor='white', fontsize=9)
        
        for spine in ax.spines.values():
            spine.set_color('#555555')
        
        fig.tight_layout()
        
        canvas = FigureCanvas(fig)
        
        # === LADO DERECHO: LISTA CNC ===
        right_layout = QVBoxLayout()
        
        title_label = QLabel("📋 Lista de Puntos")
        title_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 5px;")
        right_layout.addWidget(title_label)
        
        # Tabla de puntos (sin G-Code)
        table = QTableWidget(len(self.current_trajectory), 3)
        table.setHorizontalHeaderLabels(["#", "X (µm)", "Y (µm)"])
        table.setStyleSheet("""
            QTableWidget {
                background-color: #1a1a1a;
                color: white;
                gridline-color: #444444;
                font-family: 'Courier New';
                font-size: 11px;
            }
            QHeaderView::section {
                background-color: #3498DB;
                color: white;
                padding: 5px;
                font-weight: bold;
            }
        """)
        
        # Llenar tabla con coordenadas
        for i, point in enumerate(self.current_trajectory):
            x, y = point[0], point[1]
            
            # Número de punto
            item_n = QTableWidgetItem(f"{i+1:03d}")
            item_n.setTextAlignment(Qt.AlignCenter)
            table.setItem(i, 0, item_n)
            
            # Coordenada X
            item_x = QTableWidgetItem(f"{x:.1f}")
            item_x.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            table.setItem(i, 1, item_x)
            
            # Coordenada Y
            item_y = QTableWidgetItem(f"{y:.1f}")
            item_y.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            table.setItem(i, 2, item_y)
        
        # Ajustar columnas
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        
        right_layout.addWidget(table)
        
        # Info resumen
        info_label = QLabel(f"Total: {len(self.current_trajectory)} puntos | "
                           f"X: [{x_min:.0f}, {x_max:.0f}] µm | "
                           f"Y: [{y_min:.0f}, {y_max:.0f}] µm")
        info_label.setStyleSheet("font-size: 11px; color: #888888; padding: 5px;")
        right_layout.addWidget(info_label)
        
        # Agregar widgets al layout principal
        right_widget = QWidget()
        right_widget.setLayout(right_layout)
        right_widget.setMinimumWidth(350)
        
        main_layout.addWidget(canvas, stretch=2)
        main_layout.addWidget(right_widget, stretch=1)
        
        dialog.setLayout(main_layout)
        dialog.exec_()
        
        self.results_text.append(f"📊 Vista previa mostrada: {len(self.current_trajectory)} puntos")
        logger.info("Vista previa mostrada")
    
    # === Métodos para actualizar estado ===
    
    def set_controller_a(self, info: str, has_controller: bool):
        """Actualiza estado del controlador A."""
        self._has_controller_a = has_controller
        if has_controller:
            self.motor_a_status.setText("✅ Controlador cargado")
            self.motor_a_status.setStyleSheet("color: #27AE60; font-weight: bold;")
            self.motor_a_info.setText(info)
            self.clear_a_btn.setEnabled(True)
        else:
            self.motor_a_status.setText("⚪ Sin controlador")
            self.motor_a_status.setStyleSheet("color: #95A5A6;")
            self.motor_a_info.clear()
            self.clear_a_btn.setEnabled(False)
        self._update_control_buttons()
    
    def set_controller_b(self, info: str, has_controller: bool):
        """Actualiza estado del controlador B."""
        self._has_controller_b = has_controller
        if has_controller:
            self.motor_b_status.setText("✅ Controlador cargado")
            self.motor_b_status.setStyleSheet("color: #27AE60; font-weight: bold;")
            self.motor_b_info.setText(info)
            self.clear_b_btn.setEnabled(True)
        else:
            self.motor_b_status.setText("⚪ Sin controlador")
            self.motor_b_status.setStyleSheet("color: #95A5A6;")
            self.motor_b_info.clear()
            self.clear_b_btn.setEnabled(False)
        self._update_control_buttons()
    
    def _update_control_buttons(self):
        """Habilita/deshabilita botones de control según estado de controladores."""
        has_a = getattr(self, '_has_controller_a', False)
        has_b = getattr(self, '_has_controller_b', False)
        has_any = has_a or has_b
        
        logger.debug(f"_update_control_buttons: A={has_a}, B={has_b}, any={has_any}, active={self.dual_control_active}")
        
        # Habilitar botón de control dual si hay al menos un controlador
        if hasattr(self, 'start_dual_btn'):
            should_enable = has_any and not self.dual_control_active
            self.start_dual_btn.setEnabled(should_enable)
            logger.info(f"Botón 'Iniciar Control Dual' habilitado: {should_enable}")
    
    def set_trajectory_status(self, has_trajectory: bool, n_points: int = 0):
        """Actualiza estado de trayectoria y notifica a CameraTab."""
        if has_trajectory:
            self.trajectory_status.setText(f"✅ Trayectoria lista: {n_points} puntos")
            self.trajectory_status.setStyleSheet("color: #27AE60; font-weight: bold;")
            self.zigzag_start_btn.setEnabled(True)
        else:
            self.trajectory_status.setText("⚪ Sin trayectoria")
            self.trajectory_status.setStyleSheet("color: #95A5A6;")
            self.zigzag_start_btn.setEnabled(False)
        
        # Emitir señal para sincronizar con CameraTab
        self.trajectory_changed.emit(n_points if has_trajectory else 0)
    
    def set_calibration(self, calibrated: bool, details: str = ""):
        """Actualiza estado de calibración (método legacy)."""
        if calibrated:
            self.calibration_status.setText("✅ Sistema calibrado")
            self.calibration_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #27AE60;")
            self.calibration_details.setText(details)
        else:
            self.calibration_status.setText("⚪ Sin calibración")
            self.calibration_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #95A5A6;")
            self.calibration_details.clear()
    
    def _update_calibration_display(self):
        """Actualiza el display de calibración con datos de calibration.json."""
        try:
            cal_info = get_calibration_info()
            
            cal_x = cal_info['x']
            cal_y = cal_info['y']
            
            # Verificar si hay calibración válida
            has_calibration = cal_x.get('intercept', 0) > 0 and cal_y.get('intercept', 0) > 0
            
            if has_calibration:
                self.calibration_status.setText("✅ Calibración cargada desde JSON")
                self.calibration_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #27AE60;")
                
                details = f"📐 EJE X (Motor A): intercept={cal_x['intercept']:.1f}µm, slope={cal_x['slope']:.4f}µm/ADC\n"
                details += f"📐 EJE Y (Motor B): intercept={cal_y['intercept']:.1f}µm, slope={cal_y['slope']:.4f}µm/ADC\n"
                details += f"⚙️ Deadzone={cal_info['deadzone_adc']}ADC, Tolerancia={cal_info['tolerance_um']}µm, Settling={cal_info['settling_cycles']} ciclos"
                self.calibration_details.setText(details)
            else:
                self.calibration_status.setText("⚠️ Calibración por defecto")
                self.calibration_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #F39C12;")
                self.calibration_details.setText("Ejecuta análisis en 'Análisis' para calibrar automáticamente")
                
            logger.info(f"Display de calibración actualizado: X={cal_x}, Y={cal_y}")
            
        except Exception as e:
            logger.error(f"Error actualizando display de calibración: {e}")
            self.calibration_status.setText("❌ Error cargando calibración")
            self.calibration_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #E74C3C;")
    
    def _reload_calibration(self):
        """Recarga la calibración desde calibration.json."""
        try:
            reload_calibration()
            self._update_calibration_display()
            self.results_text.append("🔄 Calibración recargada desde calibration.json")
            logger.info("Calibración recargada manualmente")
        except Exception as e:
            self.results_text.append(f"❌ Error recargando calibración: {e}")
            logger.error(f"Error recargando calibración: {e}")
    
    def append_result(self, text: str):
        """Agrega texto al área de resultados."""
        self.results_text.append(text)
    
    def set_dual_control_active(self, active: bool):
        """Actualiza estado de control dual."""
        self.dual_control_active = active
        self.stop_dual_btn.setEnabled(active)
        self._update_control_buttons()  # Actualiza start_dual_btn según controladores
    
    def set_zigzag_active(self, active: bool):
        """Actualiza estado de ejecución zig-zag."""
        self.zigzag_start_btn.setEnabled(not active)
        self.zigzag_stop_btn.setEnabled(active)
    
    # ============================================================
    # MÉTODOS DE LÓGICA (usando callbacks de hardware)
    # ============================================================
    
    def set_controller(self, motor: str, controller_data: dict):
        """
        Guarda un controlador transferido desde HInfTab.
        
        Args:
            motor: 'A' o 'B'
            controller_data: Dict con 'Kp', 'Ki', 'K', 'U_max', etc.
        """
        if motor == 'A':
            self.controller_a = controller_data
            info = f"Kp={controller_data['Kp']:.4f}, Ki={controller_data['Ki']:.4f}\n"
            info += f"γ={controller_data.get('gamma', 0):.4f}, U_max={controller_data.get('U_max', 100):.1f}"
            self.set_controller_a(info, True)
            
            # Configurar en TestService
            sensor_key = 'sensor_2' if self.motor_a_sensor2.isChecked() else 'sensor_1'
            config = ControllerConfig(
                Kp=controller_data['Kp'],
                Ki=controller_data['Ki'],
                U_max=controller_data.get('U_max', 150),
                invert=self.motor_a_invert.isChecked(),
                sensor_key=sensor_key
            )
            self.test_service.set_controller_a(config)
            logger.info(f"Controlador A guardado en TestTab y TestService")
        else:
            self.controller_b = controller_data
            info = f"Kp={controller_data['Kp']:.4f}, Ki={controller_data['Ki']:.4f}\n"
            info += f"γ={controller_data.get('gamma', 0):.4f}, U_max={controller_data.get('U_max', 100):.1f}"
            self.set_controller_b(info, True)
            
            # Configurar en TestService
            sensor_key = 'sensor_1' if self.motor_b_sensor1.isChecked() else 'sensor_2'
            config = ControllerConfig(
                Kp=controller_data['Kp'],
                Ki=controller_data['Ki'],
                U_max=controller_data.get('U_max', 150),
                invert=self.motor_b_invert.isChecked(),
                sensor_key=sensor_key
            )
            self.test_service.set_controller_b(config)
            logger.info(f"Controlador B guardado en TestTab y TestService")
    
    def clear_controller(self, motor: str):
        """Limpia el controlador de un motor."""
        logger.info(f"Limpiando controlador Motor {motor}")
        
        if motor == 'A':
            self.controller_a = None
            self.set_controller_a("", False)
            self.test_service.set_controller_a(None)
        else:
            self.controller_b = None
            self.set_controller_b("", False)
            self.test_service.set_controller_b(None)
    
    def set_calibration(self, calibration_data: dict):
        """Guarda datos de calibración desde AnalysisTab."""
        self.calibration_data = calibration_data
        details = f"K={calibration_data.get('K', 0):.4f} µm/s/PWM\n"
        details += f"τ={calibration_data.get('tau', 0):.4f} s"
        self.set_calibration(True, details)
        logger.info("Calibración guardada en TestTab")
    
    def generate_zigzag_trajectory(self):
        """Genera trayectoria en zig-zag usando TrajectoryGenerator."""
        logger.info("=== Generando Trayectoria Zig-Zag ===")
        
        if not self.trajectory_gen:
            self.results_text.append("❌ Error: TrajectoryGenerator no disponible")
            return
        
        try:
            # Leer parámetros desde UI
            n_rows = int(self.zigzag_rows_input.text())
            n_cols = int(self.zigzag_cols_input.text())
            spacing = float(self.zigzag_spacing_input.text())
            delay = int(self.zigzag_delay_input.text())
            
            # Generar trayectoria
            result = self.trajectory_gen.generate_zigzag(n_rows, n_cols, spacing)
            
            self.current_trajectory = result['points']
            self.trajectory_index = 0
            
            self.results_text.append(f"✅ Trayectoria generada: {len(self.current_trajectory)} puntos")
            self.set_trajectory_status(True, len(self.current_trajectory))
            
            logger.info(f"Trayectoria generada: {n_rows}x{n_cols}, {len(self.current_trajectory)} puntos")
            
        except ValueError as e:
            QMessageBox.warning(self.parent_gui, "Error", f"Valores inválidos: {e}")
            logger.error(f"Error generando trayectoria: {e}")
    
    def preview_trajectory(self):
        """Muestra vista previa de la trayectoria."""
        logger.info("=== Vista Previa Trayectoria ===")
        
        if not self.current_trajectory:
            self.results_text.append("❌ Error: Genera una trayectoria primero")
            return
        
        # Crear figura de matplotlib
        fig = Figure(figsize=(8, 6), facecolor='#2E2E2E')
        ax = fig.add_subplot(111)
        
        # Extraer coordenadas
        x_coords = [p[0] for p in self.current_trajectory]
        y_coords = [p[1] for p in self.current_trajectory]
        
        # Graficar
        ax.plot(x_coords, y_coords, 'o-', color='cyan', linewidth=2, markersize=4)
        ax.set_facecolor('#252525')
        ax.set_title('Vista Previa Trayectoria Zig-Zag', color='white', fontsize=14)
        ax.set_xlabel('X (µm)', color='white')
        ax.set_ylabel('Y (µm)', color='white')
        ax.tick_params(colors='white')
        ax.grid(True, alpha=0.3)
        
        # Emitir señal para mostrar (main.py maneja la ventana)
        self.results_text.append(f"📊 Mostrando {len(self.current_trajectory)} puntos")
        
        logger.info("Vista previa de trayectoria generada")
    
    # ============================================================
    # CONTROL DUAL EN TIEMPO REAL (delegado a TestService)
    # ============================================================
    
    def start_dual_control(self):
        """Inicia control dual de ambos motores usando TestService."""
        logger.info("=== INICIANDO CONTROL DUAL (via TestService) ===")
        
        # Obtener referencias desde UI
        try:
            ref_a = float(self.ref_a_input.text()) if self.controller_a else 0
            ref_b = float(self.ref_b_input.text()) if self.controller_b else 0
        except ValueError:
            QMessageBox.warning(self.parent_gui, "Error", "Referencias inválidas")
            return
        
        # Guardar referencias para compatibilidad
        self.ref_a_um = ref_a
        self.ref_b_um = ref_b
        self._position_reached = False
        
        # Actualizar configuración de sensores en el servicio antes de iniciar
        if self.controller_a:
            sensor_key = 'sensor_2' if self.motor_a_sensor2.isChecked() else 'sensor_1'
            self.test_service.update_controller_a_sensor(sensor_key, self.motor_a_invert.isChecked())
        if self.controller_b:
            sensor_key = 'sensor_1' if self.motor_b_sensor1.isChecked() else 'sensor_2'
            self.test_service.update_controller_b_sensor(sensor_key, self.motor_b_invert.isChecked())
        
        # Delegar al servicio
        self.test_service.start_dual_control(ref_a, ref_b)
    
    def execute_dual_control(self):
        """
        Ejecuta un ciclo del control dual PI con detección de posición alcanzada.
        
        MEJORAS 2025-12-17:
        - Usa calibración dinámica desde config/constants.py
        - Zona muerta configurable (DEADZONE_ADC)
        - Tolerancia configurable (POSITION_TOLERANCE_UM)
        - Verificación de settling (SETTLING_CYCLES)
        """
        try:
            # Calcular Ts
            current_time = time.time()
            Ts = current_time - self.dual_last_time
            self.dual_last_time = current_time
            
            pwm_a = 0
            pwm_b = 0
            error_a_um = 0
            error_b_um = 0
            ref_adc_a = 0
            ref_adc_b = 0
            sensor_adc_a = 0
            sensor_adc_b = 0
            
            # Control Motor A (usa Sensor 2 según análisis) - EJE X
            if self.controller_a:
                sensor_key = 'sensor_2' if self.motor_a_sensor2.isChecked() else 'sensor_1'
                sensor_adc_a = self.get_sensor_value_callback(sensor_key)
                
                if sensor_adc_a is not None:
                    # MEJORA: Usar calibración dinámica para eje X
                    ref_adc_a = um_to_adc(self.ref_a_um, axis='x')
                    
                    # Error en ADC y µm
                    error_a = ref_adc_a - sensor_adc_a
                    error_a_um = error_a * CALIBRATION_X['slope']  # Convertir a µm
                    
                    # MEJORA: Zona muerta configurable
                    if abs(error_a) > DEADZONE_ADC:
                        self.dual_integral_a += error_a * Ts
                        
                        Kp_a = self.controller_a['Kp']
                        Ki_a = self.controller_a['Ki']
                        pwm_base_a = Kp_a * error_a + Ki_a * self.dual_integral_a
                        
                        if self.motor_a_invert.isChecked():
                            pwm_a = -int(pwm_base_a)
                        else:
                            pwm_a = int(pwm_base_a)
                        
                        U_max_a = int(self.controller_a.get('U_max', 150))
                        if abs(pwm_a) > U_max_a:
                            self.dual_integral_a -= error_a * Ts
                            pwm_a = max(-U_max_a, min(U_max_a, pwm_a))
                    else:
                        pwm_a = 0
            
            # Control Motor B (usa Sensor 1) - EJE Y
            if self.controller_b:
                sensor_key = 'sensor_1' if self.motor_b_sensor1.isChecked() else 'sensor_2'
                sensor_adc_b = self.get_sensor_value_callback(sensor_key)
                
                if sensor_adc_b is not None:
                    # MEJORA: Usar calibración dinámica para eje Y
                    ref_adc_b = um_to_adc(self.ref_b_um, axis='y')
                    
                    error_b = ref_adc_b - sensor_adc_b
                    error_b_um = error_b * CALIBRATION_Y['slope']  # Convertir a µm
                    
                    # MEJORA: Zona muerta configurable
                    if abs(error_b) > DEADZONE_ADC:
                        self.dual_integral_b += error_b * Ts
                        
                        Kp_b = self.controller_b['Kp']
                        Ki_b = self.controller_b['Ki']
                        pwm_base_b = Kp_b * error_b + Ki_b * self.dual_integral_b
                        
                        if self.motor_b_invert.isChecked():
                            pwm_b = -int(pwm_base_b)
                        else:
                            pwm_b = int(pwm_base_b)
                        
                        U_max_b = int(self.controller_b.get('U_max', 150))
                        if abs(pwm_b) > U_max_b:
                            self.dual_integral_b -= error_b * Ts
                            pwm_b = max(-U_max_b, min(U_max_b, pwm_b))
                    else:
                        pwm_b = 0
            
            # MEJORA: Tolerancia configurable desde constants.py
            a_at_target = abs(error_a_um) < POSITION_TOLERANCE_UM if self.controller_a else True
            b_at_target = abs(error_b_um) < POSITION_TOLERANCE_UM if self.controller_b else True
            both_at_target = a_at_target and b_at_target
            
            # Inicializar flags si no existen
            if not hasattr(self, '_position_reached'):
                self._position_reached = False
            if not hasattr(self, '_settling_counter'):
                self._settling_counter = 0
            
            # MEJORA: Verificación de settling antes de declarar posición alcanzada
            if both_at_target:
                self._settling_counter += 1
                
                if self._settling_counter >= SETTLING_CYCLES and not self._position_reached:
                    # ¡POSICIÓN ALCANZADA Y ESTABLE!
                    self._position_reached = True
                    self.send_command_callback('B')  # Freno activo
                    time.sleep(0.02)
                    self.send_command_callback('A,0,0')
                    
                    self.results_text.append(
                        f"✅ POSICIÓN ALCANZADA (estable {SETTLING_CYCLES} ciclos): "
                        f"A={self.ref_a_um:.0f}µm (err={error_a_um:.1f}), "
                        f"B={self.ref_b_um:.0f}µm (err={error_b_um:.1f})"
                    )
                    logger.info(f"✅ Posición alcanzada y estable - Motores bloqueados")
                    return
            else:
                # Resetear contador de settling si sale del objetivo
                self._settling_counter = 0
                
                if self._position_reached:
                    self._position_reached = False
                    self.results_text.append("🔄 Posición perdida - Reactivando control...")
                
                # Enviar comando de control
                self.send_command_callback(f"A,{pwm_a},{pwm_b}")
            
            # Log cada 50 ciclos
            if not hasattr(self, '_dual_log_counter'):
                self._dual_log_counter = 0
            self._dual_log_counter += 1
            if self._dual_log_counter % 50 == 0:
                status = "✅" if self._position_reached else ("⏳" if both_at_target else "🔄")
                settling_info = f" [settling: {self._settling_counter}/{SETTLING_CYCLES}]" if both_at_target and not self._position_reached else ""
                self.results_text.append(
                    f"{status} A: {error_a_um:.1f}µm | B: {error_b_um:.1f}µm | PWM: ({pwm_a},{pwm_b}){settling_info}"
                )
            
        except Exception as e:
            logger.error(f"Error en control dual: {e}")
    
    def stop_dual_control(self):
        """Detiene el control dual con freno activo (delegado a TestService)."""
        logger.info("=== DETENIENDO CONTROL DUAL (via TestService) ===")
        self.test_service.stop_dual_control()
    
    # ============================================================
    # EJECUCIÓN DE TRAYECTORIA ZIG-ZAG
    # ============================================================
    
    def start_trajectory_execution(self):
        """Inicia la ejecución de la trayectoria zig-zag (delegado a TestService)."""
        logger.info("=== INICIANDO EJECUCIÓN DE TRAYECTORIA (via TestService) ===")
        
        if self.current_trajectory is None or len(self.current_trajectory) == 0:
            self.results_text.append("❌ Error: Genera una trayectoria primero")
            return
        
        # Obtener parámetros
        try:
            tolerance = float(self.tolerance_input.text())
            pause = float(self.pause_input.text())
        except ValueError:
            tolerance = POSITION_TOLERANCE_UM
            pause = 2.0
        
        # Guardar para compatibilidad
        self.trajectory_tolerance = tolerance
        self.trajectory_pause = pause
        self.trajectory_index = 0
        
        # Actualizar configuración de sensores en el servicio
        if self.controller_a:
            sensor_key = 'sensor_2' if self.motor_a_sensor2.isChecked() else 'sensor_1'
            self.test_service.update_controller_a_sensor(sensor_key, self.motor_a_invert.isChecked())
        if self.controller_b:
            sensor_key = 'sensor_1' if self.motor_b_sensor1.isChecked() else 'sensor_2'
            self.test_service.update_controller_b_sensor(sensor_key, self.motor_b_invert.isChecked())
        
        # Convertir trayectoria a lista de tuplas
        trajectory_list = [(p[0], p[1]) for p in self.current_trajectory]
        
        # Delegar al servicio
        self.test_service.start_trajectory(trajectory_list, tolerance, pause)
    
    def _detect_axis_lock(self, current_idx: int) -> tuple:
        """
        Detecta si algún eje debe bloquearse porque su coordenada no cambia.
        
        MEJORA 2025-12-17: Bloqueo inteligente de ejes
        - Si X no cambia entre punto actual y siguiente → bloquear Motor A
        - Si Y no cambia entre punto actual y siguiente → bloquear Motor B
        - Mejora la rectitud de trayectorias en filas/columnas
        
        Returns:
            tuple: (lock_x: bool, lock_y: bool)
        """
        if self.current_trajectory is None or len(self.current_trajectory) == 0 or current_idx >= len(self.current_trajectory):
            return (False, False)
        
        current = self.current_trajectory[current_idx]
        
        # Comparar con punto anterior (si existe)
        if current_idx > 0:
            prev = self.current_trajectory[current_idx - 1]
            # Tolerancia para considerar "mismo valor" (1 µm)
            lock_x = abs(current[0] - prev[0]) < 1.0
            lock_y = abs(current[1] - prev[1]) < 1.0
            return (lock_x, lock_y)
        
        return (False, False)
    
    def execute_trajectory_step(self):
        """
        Ejecuta un paso del control de trayectoria.
        
        MEJORAS 2025-12-17:
        - Calibración dinámica desde config/constants.py
        - Zona muerta configurable (DEADZONE_ADC)
        - Tolerancia configurable (POSITION_TOLERANCE_UM)
        - Verificación de settling antes de avanzar
        - BLOQUEO INTELIGENTE DE EJES: Si un eje no cambia, se bloquea
        
        MECANISMO DE BLOQUEO:
        1. Mientras no llega al punto → Control PI activo, envía PWM
        2. Cuando llega al punto → Espera SETTLING_CYCLES ciclos estables
        3. Después de settling → DETIENE motores (freno), inicia pausa
        4. Durante la pausa → NO envía comandos, motores quietos
        5. Después de la pausa → Siguiente punto, reinicia control
        
        BLOQUEO DE EJES:
        - Si X es constante → Motor A bloqueado (PWM=0), solo Y se mueve
        - Si Y es constante → Motor B bloqueado (PWM=0), solo X se mueve
        - Mejora rectitud en trayectorias zig-zag
        """
        try:
            if not self.trajectory_active:
                return
            
            current_time = time.time()
            
            # Si estamos en pausa, solo verificar si terminó
            if self.trajectory_waiting:
                if current_time - self.trajectory_wait_start >= self.trajectory_pause:
                    # Pausa completada, siguiente punto
                    self.trajectory_index += 1
                    self.trajectory_waiting = False
                    self.dual_integral_a = 0
                    self.dual_integral_b = 0
                    self._traj_settling_counter = 0  # Resetear settling para nuevo punto
                    self._traj_near_attempts = 0     # Resetear intentos cerca para nuevo punto
                    logger.info(f"⏭️ Pausa completada, avanzando a punto {self.trajectory_index + 1}")
                # Durante la pausa NO enviamos comandos - motores quietos
                return
            
            # Calcular Ts para control PI
            Ts = current_time - self.dual_last_time
            self.dual_last_time = current_time
            
            # Verificar si completamos la trayectoria
            if self.trajectory_index >= len(self.current_trajectory):
                self.stop_trajectory_execution()
                self.results_text.append("✅ Trayectoria completada!")
                return
            
            target = self.current_trajectory[self.trajectory_index]
            target_x = target[0]  # µm
            target_y = target[1]  # µm
            
            # MEJORA 2025-12-17: Detectar bloqueo de ejes
            lock_x, lock_y = self._detect_axis_lock(self.trajectory_index)
            
            # MEJORA: Usar calibración dinámica desde constants.py
            ref_adc_x = um_to_adc(target_x, axis='x')
            ref_adc_y = um_to_adc(target_y, axis='y')
            
            pwm_a = 0
            pwm_b = 0
            error_x_um = 0
            error_y_um = 0
            
            # Control Motor A (eje X, sensor 2) - SOLO SI NO ESTÁ BLOQUEADO
            if self.controller_a and not lock_x:
                sensor_key = 'sensor_2' if self.motor_a_sensor2.isChecked() else 'sensor_1'
                sensor_adc = self.get_sensor_value_callback(sensor_key)
                
                if sensor_adc is not None:
                    error_adc = ref_adc_x - sensor_adc
                    error_x_um = error_adc * CALIBRATION_X['slope']  # Convertir a µm
                    
                    # MEJORA: Zona muerta configurable
                    if abs(error_adc) > DEADZONE_ADC:
                        self.dual_integral_a += error_adc * Ts
                        Kp = self.controller_a['Kp']
                        Ki = self.controller_a['Ki']
                        pwm_base = Kp * error_adc + Ki * self.dual_integral_a
                        
                        if self.motor_a_invert.isChecked():
                            pwm_a = -int(pwm_base)
                        else:
                            pwm_a = int(pwm_base)
                        
                        U_max = int(self.controller_a.get('U_max', 150))
                        if abs(pwm_a) > U_max:
                            self.dual_integral_a -= error_adc * Ts
                            pwm_a = max(-U_max, min(U_max, pwm_a))
                    else:
                        pwm_a = 0
            elif lock_x:
                # Eje X bloqueado - leer error pero no enviar PWM
                sensor_key = 'sensor_2' if self.motor_a_sensor2.isChecked() else 'sensor_1'
                sensor_adc = self.get_sensor_value_callback(sensor_key)
                if sensor_adc is not None:
                    error_adc = ref_adc_x - sensor_adc
                    error_x_um = error_adc * CALIBRATION_X['slope']
                pwm_a = 0  # BLOQUEADO
            
            # Control Motor B (eje Y, sensor 1) - SOLO SI NO ESTÁ BLOQUEADO
            if self.controller_b and not lock_y:
                sensor_key = 'sensor_1' if self.motor_b_sensor1.isChecked() else 'sensor_2'
                sensor_adc = self.get_sensor_value_callback(sensor_key)
                
                if sensor_adc is not None:
                    error_adc = ref_adc_y - sensor_adc
                    error_y_um = error_adc * CALIBRATION_Y['slope']  # Convertir a µm
                    
                    # MEJORA: Zona muerta configurable
                    if abs(error_adc) > DEADZONE_ADC:
                        self.dual_integral_b += error_adc * Ts
                        Kp = self.controller_b['Kp']
                        Ki = self.controller_b['Ki']
                        pwm_base = Kp * error_adc + Ki * self.dual_integral_b
                        
                        if self.motor_b_invert.isChecked():
                            pwm_b = -int(pwm_base)
                        else:
                            pwm_b = int(pwm_base)
                        
                        U_max = int(self.controller_b.get('U_max', 150))
                        if abs(pwm_b) > U_max:
                            self.dual_integral_b -= error_adc * Ts
                            pwm_b = max(-U_max, min(U_max, pwm_b))
                    else:
                        pwm_b = 0
            elif lock_y:
                # Eje Y bloqueado - leer error pero no enviar PWM
                sensor_key = 'sensor_1' if self.motor_b_sensor1.isChecked() else 'sensor_2'
                sensor_adc = self.get_sensor_value_callback(sensor_key)
                if sensor_adc is not None:
                    error_adc = ref_adc_y - sensor_adc
                    error_y_um = error_adc * CALIBRATION_Y['slope']
                pwm_b = 0  # BLOQUEADO
            
            # MEJORA: Tolerancia configurable (usa la de UI si está definida, sino la de constants)
            tolerance = getattr(self, 'trajectory_tolerance', POSITION_TOLERANCE_UM)
            fallback_tolerance = tolerance * FALLBACK_TOLERANCE_MULTIPLIER
            
            # MEJORA 2025-12-17: Si un eje está bloqueado, IGNORAR su error para settling
            # El eje bloqueado se asume en posición correcta (no se mueve, no se mide)
            # Esto evita que perturbaciones cruzadas entre sensores afecten el settling
            if lock_x and lock_y:
                # Ambos bloqueados = ya estamos en posición (caso raro)
                at_target = True
                at_fallback_target = True
            elif lock_x:
                # X bloqueado → solo importa error de Y
                at_target = abs(error_y_um) < tolerance
                at_fallback_target = abs(error_y_um) < fallback_tolerance
            elif lock_y:
                # Y bloqueado → solo importa error de X
                at_target = abs(error_x_um) < tolerance
                at_fallback_target = abs(error_x_um) < fallback_tolerance
            else:
                # Ninguno bloqueado → ambos errores importan
                at_target = abs(error_x_um) < tolerance and abs(error_y_um) < tolerance
                at_fallback_target = abs(error_x_um) < fallback_tolerance and abs(error_y_um) < fallback_tolerance
            
            # Inicializar contadores si no existen
            if not hasattr(self, '_traj_settling_counter'):
                self._traj_settling_counter = 0
            if not hasattr(self, '_traj_near_attempts'):
                self._traj_near_attempts = 0  # Intentos DESPUÉS de llegar cerca
            
            # MEJORA 2025-12-17: Lógica de settling basada en CONDICIONES, no timers
            # El contador de intentos SOLO se incrementa cuando ya estamos CERCA del punto
            # Esto evita evaluar prematuramente mientras aún nos movemos hacia el objetivo
            #
            # Flujo:
            # 1. at_target (tolerancia normal) → settling normal (4 ciclos estables)
            # 2. at_fallback_target (cerca pero no exacto) → contar intentos cerca
            #    - Si settling falla 5 veces estando cerca → aceptar con fallback
            # 3. Lejos del punto → seguir moviendo, NO contar nada
            
            if at_target:
                # Estamos en tolerancia normal → contar settling
                self._traj_settling_counter += 1
                self._traj_near_attempts += 1  # También cuenta como intento cerca
                
                if self._traj_settling_counter >= SETTLING_CYCLES:
                    # ¡LLEGAMOS AL PUNTO Y ESTAMOS ESTABLES!
                    self._accept_current_point(target_x, target_y, error_x_um, error_y_um, "✅ Estable")
                else:
                    # En objetivo pero esperando settling - seguir enviando control suave
                    self.send_command_callback(f"A,{pwm_a},{pwm_b}")
                    
            elif at_fallback_target:
                # Estamos CERCA pero no exactamente en tolerancia normal
                # Resetear settling (salimos de tolerancia estricta)
                self._traj_settling_counter = 0
                self._traj_near_attempts += 1  # Contar intento cerca
                
                # Si llevamos muchos intentos cerca sin lograr settling estable → fallback
                if self._traj_near_attempts >= MAX_ATTEMPTS_PER_POINT:
                    self._accept_current_point(target_x, target_y, error_x_um, error_y_um, 
                                               f"⚠️ Fallback ({self._traj_near_attempts} intentos)")
                    logger.warning(f"⚠️ Punto {self.trajectory_index + 1} aceptado con fallback - {self._traj_near_attempts} intentos cerca")
                else:
                    # Seguir intentando acercarse más
                    self.send_command_callback(f"A,{pwm_a},{pwm_b}")
            else:
                # Lejos del punto → resetear todo y seguir moviendo
                # NO contamos intentos mientras estamos lejos
                self._traj_settling_counter = 0
                self._traj_near_attempts = 0
                self.send_command_callback(f"A,{pwm_a},{pwm_b}")
            
            # MEJORA: Actualizar feedback visual en UI (incluyendo estado de bloqueo)
            self._update_trajectory_feedback(target_x, target_y, error_x_um, error_y_um, lock_x, lock_y)
            
        except Exception as e:
            logger.error(f"Error en ejecución de trayectoria: {e}")
    
    def _accept_current_point(self, target_x: float, target_y: float, error_x: float, error_y: float, status: str):
        """
        Acepta el punto actual y prepara para el siguiente.
        
        Args:
            target_x, target_y: Coordenadas objetivo
            error_x, error_y: Errores actuales en µm
            status: Mensaje de estado (ej: "✅ Estable", "⚠️ Fallback")
        """
        current_time = time.time()
        
        # Freno activo
        self.send_command_callback('B')
        time.sleep(0.05)
        self.send_command_callback('A,0,0')
        
        # Iniciar pausa
        self.trajectory_waiting = True
        self.trajectory_wait_start = current_time
        self._traj_settling_counter = 0
        self._traj_near_attempts = 0  # Resetear para siguiente punto
        
        self.results_text.append(
            f"📍 Punto {self.trajectory_index + 1}/{len(self.current_trajectory)}: "
            f"({target_x:.0f}, {target_y:.0f})µm {status} "
            f"[Error: X={error_x:.1f}, Y={error_y:.1f}µm] "
            f"- Pausa {self.trajectory_pause}s"
        )
        logger.info(f"{status} Punto {self.trajectory_index + 1} - Pausa {self.trajectory_pause}s")
    
    def _update_trajectory_feedback(self, target_x: float, target_y: float, error_x: float, error_y: float, 
                                      lock_x: bool = False, lock_y: bool = False):
        """Actualiza los labels de feedback visual durante ejecución de trayectoria."""
        try:
            # Progreso
            total = len(self.current_trajectory) if self.current_trajectory is not None else 0
            current = self.trajectory_index + 1
            self.trajectory_progress_label.setText(f"{current} / {total}")
            
            # Punto actual con indicador de bloqueo
            lock_indicator = ""
            if lock_x and lock_y:
                lock_indicator = " 🔒XY"
            elif lock_x:
                lock_indicator = " 🔒X"
            elif lock_y:
                lock_indicator = " 🔒Y"
            self.current_point_label.setText(f"({target_x:.0f}, {target_y:.0f}) µm{lock_indicator}")
            
            # Errores con colores según magnitud
            tolerance = getattr(self, 'trajectory_tolerance', POSITION_TOLERANCE_UM)
            
            # Error X - mostrar si está bloqueado
            if lock_x:
                self.error_x_label.setStyleSheet("font-family: monospace; color: #3498DB;")  # Azul = bloqueado
                self.error_x_label.setText(f"X: 🔒 LOCK")
            elif abs(error_x) < tolerance:
                self.error_x_label.setStyleSheet("font-family: monospace; color: #27AE60;")  # Verde
                self.error_x_label.setText(f"X: {error_x:+.1f} µm")
            elif abs(error_x) < tolerance * 2:
                self.error_x_label.setStyleSheet("font-family: monospace; color: #F39C12;")  # Amarillo
                self.error_x_label.setText(f"X: {error_x:+.1f} µm")
            else:
                self.error_x_label.setStyleSheet("font-family: monospace; color: #E74C3C;")  # Rojo
                self.error_x_label.setText(f"X: {error_x:+.1f} µm")
            
            # Error Y - mostrar si está bloqueado
            if lock_y:
                self.error_y_label.setStyleSheet("font-family: monospace; color: #3498DB;")  # Azul = bloqueado
                self.error_y_label.setText(f"Y: 🔒 LOCK")
            elif abs(error_y) < tolerance:
                self.error_y_label.setStyleSheet("font-family: monospace; color: #27AE60;")
                self.error_y_label.setText(f"Y: {error_y:+.1f} µm")
            elif abs(error_y) < tolerance * 2:
                self.error_y_label.setStyleSheet("font-family: monospace; color: #F39C12;")
                self.error_y_label.setText(f"Y: {error_y:+.1f} µm")
            else:
                self.error_y_label.setStyleSheet("font-family: monospace; color: #E74C3C;")
                self.error_y_label.setText(f"Y: {error_y:+.1f} µm")
            
            # Settling
            settling = getattr(self, '_traj_settling_counter', 0)
            if settling > 0:
                self.settling_label.setStyleSheet("font-family: monospace; color: #27AE60;")
            else:
                self.settling_label.setStyleSheet("font-family: monospace; color: #F39C12;")
            self.settling_label.setText(f"Settling: {settling}/{SETTLING_CYCLES}")
            
        except Exception as e:
            logger.debug(f"Error actualizando feedback: {e}")
    
    def _reset_trajectory_feedback(self):
        """Resetea los labels de feedback a estado inicial."""
        self.trajectory_progress_label.setText("-- / --")
        self.current_point_label.setText("(---, ---) µm")
        self.error_x_label.setText("X: --- µm")
        self.error_x_label.setStyleSheet("font-family: monospace; color: #E74C3C;")
        self.error_y_label.setText("Y: --- µm")
        self.error_y_label.setStyleSheet("font-family: monospace; color: #E74C3C;")
        self.settling_label.setText("Settling: --/--")
        self.settling_label.setStyleSheet("font-family: monospace; color: #F39C12;")
    
    def stop_trajectory_execution(self):
        """Detiene la ejecución de la trayectoria con freno activo (delegado a TestService)."""
        logger.info("=== DETENIENDO EJECUCIÓN DE TRAYECTORIA (via TestService) ===")
        self.test_service.stop_trajectory()
    
    # ============================================================
    # HANDLERS DE SEÑALES DEL TESTSERVICE
    # ============================================================
    
    def _on_dual_control_started(self):
        """Handler: Control dual iniciado."""
        self.dual_control_active = True
        self.set_dual_control_active(True)
        
        # Actualizar label de modo
        mode_label = self.get_mode_label_callback()
        if mode_label:
            mode_label.setText("AUTOMÁTICO (Dual)")
            mode_label.setStyleSheet("font-weight: bold; color: #8E44AD;")
    
    def _on_dual_control_stopped(self):
        """Handler: Control dual detenido."""
        self.dual_control_active = False
        self.set_dual_control_active(False)
        
        # Actualizar label de modo
        mode_label = self.get_mode_label_callback()
        if mode_label:
            mode_label.setText("MANUAL")
            mode_label.setStyleSheet("font-weight: bold; color: #E67E22;")
    
    def _on_dual_position_update(self, error_a: float, error_b: float, pwm_a: int, pwm_b: int):
        """Handler: Actualización de posición durante control dual."""
        pass
    
    def _on_dual_position_reached(self, ref_a: float, ref_b: float, error_a: float, error_b: float):
        """Handler: Posición alcanzada y estable."""
        self._position_reached = True
    
    def _on_dual_position_lost(self):
        """Handler: Posición perdida."""
        self._position_reached = False
    
    def _on_trajectory_started(self, total_points: int):
        """Handler: Trayectoria iniciada."""
        self.trajectory_active = True
        self.set_zigzag_active(True)
    
    def _on_trajectory_stopped(self, current_point: int, total_points: int):
        """Handler: Trayectoria detenida."""
        self.trajectory_active = False
        self.set_zigzag_active(False)
        self._reset_trajectory_feedback()
    
    def _on_trajectory_completed(self, total_points: int):
        """Handler: Trayectoria completada."""
        self.trajectory_active = False
        self.set_zigzag_active(False)
        self._reset_trajectory_feedback()
    
    def _on_trajectory_point_reached(self, index: int, x: float, y: float, status: str):
        """Handler: Punto de trayectoria alcanzado."""
        self.trajectory_index = index
    
    def _on_trajectory_feedback(self, target_x: float, target_y: float, 
                                 error_x: float, error_y: float,
                                 lock_x: bool, lock_y: bool, settling: int):
        """Handler: Actualización de feedback visual de trayectoria."""
        self._update_trajectory_feedback(target_x, target_y, error_x, error_y, lock_x, lock_y)
    
    def _on_log_message(self, message: str):
        """Handler: Mensaje de log del servicio."""
        self.results_text.append(message)
    
    def _on_error_occurred(self, error: str):
        """Handler: Error del servicio."""
        self.results_text.append(f"❌ Error: {error}")
        QMessageBox.warning(self.parent_gui, "Error", error)

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

import logging
import numpy as np

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QTextEdit, QScrollArea,
                             QMessageBox, QFileDialog)
from PyQt5.QtCore import Qt, pyqtSignal

from config.constants import (
    POSITION_TOLERANCE_UM, SETTLING_CYCLES
)
from core.services.test_service import TestService, ControllerConfig
from gui.utils.trajectory_preview import show_trajectory_preview
from gui.utils.csv_utils import export_trajectory_csv, import_trajectory_csv
from gui.utils.test_tab_ui_builder import (
    create_calibration_analysis_section,
    create_controllers_section,
    create_motor_sensor_section,
    create_position_control_section,
    create_trajectory_section,
    create_zigzag_section
)
from core.services.calibration_analysis_service import CalibrationAnalysisService
from gui.windows import MatplotlibWindow

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
        
        # Estado de control (sincronizado con TestService via señales)
        self.dual_control_active = False
        self._position_reached = False
        
        # Variables de trayectoria (UI state)
        self.current_trajectory = None
        self.trajectory_index = 0
        self.trajectory_active = False
        self.trajectory_tolerance = POSITION_TOLERANCE_UM
        self.trajectory_pause = 2.0
        
        # Calibración
        self.calibration_data = None
        
        self._setup_ui()
        self._map_widgets()
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
        """Configura la interfaz de usuario usando builders externos."""
        main_layout = QVBoxLayout(self)
        
        # Scroll area para contenido extenso
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        
        scroll_widget = QWidget()
        layout = QVBoxLayout(scroll_widget)
        
        # Diccionario para almacenar referencias a widgets
        self._widgets = {}
        
        # Sección 0: Análisis de Calibración (botón superior)
        calibration_analysis_widget = create_calibration_analysis_section(
            self._widgets,
            self.show_calibration_analysis
        )
        layout.addWidget(calibration_analysis_widget)
        
        # Sección 1: Controladores H∞ Transferidos
        controllers_group = create_controllers_section(
            self._widgets, 
            lambda motor: self.controller_clear_requested.emit(motor)
        )
        layout.addWidget(controllers_group)
        
        # Sección 2: Asignación Motor-Sensor
        motor_sensor_group = create_motor_sensor_section(self._widgets)
        layout.addWidget(motor_sensor_group)
        
        # Sección 4: Control por Posición
        position_group = create_position_control_section(
            self._widgets, self._start_dual_control, self.stop_dual_control
        )
        layout.addWidget(position_group)
        
        # Sección 5: Generador de Trayectorias
        trajectory_group = create_trajectory_section(
            self._widgets, 
            self._generate_trajectory, 
            self._preview_trajectory,
            self._export_trajectory_csv,
            self._import_trajectory_csv
        )
        layout.addWidget(trajectory_group)
        
        # Sección 6: Ejecución Zig-Zag
        zigzag_group = create_zigzag_section(
            self._widgets, 
            self.start_trajectory_execution, 
            self.stop_trajectory_execution
        )
        layout.addWidget(zigzag_group)
        
        # Mapear widgets al objeto para acceso directo
        self._map_widgets()
        
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
    
    def _map_widgets(self):
        """Mapea widgets del diccionario a atributos del objeto para compatibilidad."""
        # Controladores
        self.motor_a_label = self._widgets.get('motor_a_label')
        self.motor_a_status = self._widgets.get('motor_a_status')
        self.motor_a_info = self._widgets.get('motor_a_info')
        self.clear_a_btn = self._widgets.get('clear_a_btn')
        self.motor_b_label = self._widgets.get('motor_b_label')
        self.motor_b_status = self._widgets.get('motor_b_status')
        self.motor_b_info = self._widgets.get('motor_b_info')
        self.clear_b_btn = self._widgets.get('clear_b_btn')
        
        # Motor-Sensor
        self.motor_a_sensor1 = self._widgets.get('motor_a_sensor1')
        self.motor_a_sensor2 = self._widgets.get('motor_a_sensor2')
        self.motor_a_invert = self._widgets.get('motor_a_invert')
        self.motor_b_sensor1 = self._widgets.get('motor_b_sensor1')
        self.motor_b_sensor2 = self._widgets.get('motor_b_sensor2')
        self.motor_b_invert = self._widgets.get('motor_b_invert')
        
        
        # Control por posición
        self.ref_a_input = self._widgets.get('ref_a_input')
        self.ref_b_input = self._widgets.get('ref_b_input')
        self.start_dual_btn = self._widgets.get('start_dual_btn')
        self.stop_dual_btn = self._widgets.get('stop_dual_btn')
        
        # Trayectorias
        self.points_input = self._widgets.get('points_input')
        self.x_start_input = self._widgets.get('x_start_input')
        self.x_end_input = self._widgets.get('x_end_input')
        self.y_start_input = self._widgets.get('y_start_input')
        self.y_end_input = self._widgets.get('y_end_input')
        self.delay_input = self._widgets.get('delay_input')
        
        # Ejecución
        self.trajectory_status = self._widgets.get('trajectory_status')
        self.tolerance_input = self._widgets.get('tolerance_input')
        self.pause_input = self._widgets.get('pause_input')
        self.trajectory_progress_label = self._widgets.get('trajectory_progress_label')
        self.current_point_label = self._widgets.get('current_point_label')
        self.error_x_label = self._widgets.get('error_x_label')
        self.error_y_label = self._widgets.get('error_y_label')
        self.settling_label = self._widgets.get('settling_label')
        self.zigzag_start_btn = self._widgets.get('zigzag_start_btn')
        self.zigzag_stop_btn = self._widgets.get('zigzag_stop_btn')
    
    # ============================================================
    # MÉTODOS DE ACCIÓN (callbacks de botones)
    # ============================================================
    
    def _start_dual_control(self):
        """Inicia control dual - llama directamente al método de control."""
        logger.info("Botón 'Iniciar Control Dual' presionado")
        self.start_dual_control()
    
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
        """Exporta la trayectoria actual a un archivo CSV usando utilidad externa."""
        if self.current_trajectory is None or len(self.current_trajectory) == 0:
            self.results_text.append("❌ Error: No hay trayectoria para exportar")
            return
        
        filename, _ = QFileDialog.getSaveFileName(
            self, "Guardar Trayectoria CSV", 
            "trayectoria.csv", 
            "CSV Files (*.csv)"
        )
        
        if filename:
            success, message = export_trajectory_csv(self.current_trajectory, filename)
            if success:
                self.results_text.append(f"✅ {message}")
            else:
                self.results_text.append(f"❌ {message}")
    
    def _import_trajectory_csv(self):
        """Importa una trayectoria desde un archivo CSV usando utilidad externa."""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Cargar Trayectoria CSV", 
            "", 
            "CSV Files (*.csv);;All Files (*)"
        )
        
        if filename:
            success, message, trajectory = import_trajectory_csv(filename)
            if success and trajectory is not None:
                self.current_trajectory = trajectory
                self.trajectory_index = 0
                self.set_trajectory_status(True, len(self.current_trajectory))
                self.results_text.append(f"✅ {message}")
            else:
                self.results_text.append(f"❌ {message}")
    
    def _preview_trajectory(self):
        """Muestra vista previa de la trayectoria generada con gráfico XY."""
        logger.info("=== BOTÓN: Vista Previa presionado ===")
        
        if self.current_trajectory is None or len(self.current_trajectory) == 0:
            self.results_text.append("❌ Error: Genera una trayectoria primero")
            return
        
        # Usar función de utilidad para mostrar vista previa
        if show_trajectory_preview(self, self.current_trajectory):
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
    
    def update_calibration_data(self, calibration_data: dict):
        """Guarda datos de calibración desde AnalysisTab (método legacy - ya no usado)."""
        self.calibration_data = calibration_data
        logger.info("Calibración guardada en TestTab (legacy)")
    
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
        """Muestra vista previa de la trayectoria (alias de _preview_trajectory)."""
        self._preview_trajectory()
    
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
    
    # =========================================================================
    # ANÁLISIS DE CALIBRACIÓN
    # =========================================================================
    
    def show_calibration_analysis(self):
        """Muestra gráficos de análisis de calibración para ambos motores."""
        logger.info("Generando gráficos de análisis de calibración...")
        
        try:
            # Generar análisis usando el servicio
            result = CalibrationAnalysisService.generate_calibration_analysis()
            
            if not result['success']:
                QMessageBox.warning(
                    self.parent_gui,
                    "Error en Análisis",
                    result['message']
                )
                return
            
            # Mostrar gráfico de Motor A
            if 'motor_a' in result:
                window_a = MatplotlibWindow(
                    result['motor_a'],
                    "Análisis de Calibración - Motor A (Eje X)",
                    self.parent_gui
                )
                window_a.show()
                window_a.raise_()
            
            # Mostrar gráfico de Motor B
            if 'motor_b' in result:
                window_b = MatplotlibWindow(
                    result['motor_b'],
                    "Análisis de Calibración - Motor B (Eje Y)",
                    self.parent_gui
                )
                window_b.show()
                window_b.raise_()
            
            self.results_text.append("✅ Gráficos de calibración generados exitosamente")
            logger.info("✅ Gráficos de calibración mostrados")
            
        except Exception as e:
            error_msg = f"Error al generar gráficos de calibración: {str(e)}"
            logger.error(error_msg, exc_info=True)
            QMessageBox.critical(
                self.parent_gui,
                "Error",
                error_msg
            )

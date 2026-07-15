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
from PyQt5.QtCore import Qt, pyqtSignal, QTimer

from config.constants import (
    POSITION_TOLERANCE_UM, SETTLING_CYCLES,
    DEFAULT_FOV_X_UM, DEFAULT_FOV_Y_UM
)
from core.control.step_config import load_step_control_config
from core.control.controller_config import ControllerConfig
from core.services.test_service import TestService
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
from utils.parameter_manager import get_parameter_manager

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
            parent: Widget padre (CTRL_GUI)
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
        self._load_default_parameters()
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

        if self.motor_a_invert is not None:
            self.motor_a_invert.toggled.connect(
                lambda checked: self._on_invert_toggled('A', checked)
            )
        if self.motor_b_invert is not None:
            self.motor_b_invert.toggled.connect(
                lambda checked: self._on_invert_toggled('B', checked)
            )
        
        # Control por posición
        self.ref_a_input = self._widgets.get('ref_a_input')
        self.ref_b_input = self._widgets.get('ref_b_input')
        self.start_dual_btn = self._widgets.get('start_dual_btn')
        self.stop_dual_btn = self._widgets.get('stop_dual_btn')
        
        # Trayectorias
        self.fov_x_input = self._widgets.get('fov_x_input')
        self.fov_y_input = self._widgets.get('fov_y_input')
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
        self.homogeneous_steps_cb = self._widgets.get('homogeneous_steps_cb')
        self.trajectory_progress_label = self._widgets.get('trajectory_progress_label')
        self.current_point_label = self._widgets.get('current_point_label')
        self.error_x_label = self._widgets.get('error_x_label')
        self.error_y_label = self._widgets.get('error_y_label')
        self.settling_label = self._widgets.get('settling_label')
        self.zigzag_start_btn = self._widgets.get('zigzag_start_btn')
        self.zigzag_stop_btn = self._widgets.get('zigzag_stop_btn')
    
    def _load_default_parameters(self):
        """Carga parámetros por defecto desde ParameterManager."""
        try:
            pm = get_parameter_manager()
            defaults = pm.get_trajectory_defaults()
            
            # Cargar valores en los widgets
            if self.points_input:
                saved_points = defaults.get('points')
                self.points_input.setText(str(saved_points) if saved_points else "--")
            if self.fov_x_input:
                fov = defaults.get('fov', {})
                self.fov_x_input.setText(str(fov.get('x', DEFAULT_FOV_X_UM)))
            if self.fov_y_input:
                fov = defaults.get('fov', {})
                self.fov_y_input.setText(str(fov.get('y', DEFAULT_FOV_Y_UM)))
            if self.x_start_input:
                self.x_start_input.setText(str(defaults.get('x_range', {}).get('min', 10000.0)))
            if self.x_end_input:
                self.x_end_input.setText(str(defaults.get('x_range', {}).get('max', 19500.0)))
            if self.y_start_input:
                self.y_start_input.setText(str(defaults.get('y_range', {}).get('min', 10000.0)))
            if self.y_end_input:
                self.y_end_input.setText(str(defaults.get('y_range', {}).get('max', 19500.0)))
            if self.delay_input:
                self.delay_input.setText(str(defaults.get('delay_between_points', 0.5)))
            if self.homogeneous_steps_cb:
                step_cfg = load_step_control_config()
                self.homogeneous_steps_cb.setChecked(step_cfg.enabled)
            
            logger.info("✅ Parámetros de trayectoria cargados desde configuración")
        except Exception as e:
            logger.warning(f"No se pudieron cargar parámetros por defecto: {e}")
    
    def _save_trajectory_parameters(self, n_points: int, x_min: float, x_max: float,
                                    y_min: float, y_max: float, delay: float,
                                    fov_x: float, fov_y: float):
        """Guarda parámetros de trayectoria en ParameterManager."""
        try:
            pm = get_parameter_manager()
            pm.update_trajectory(
                points=n_points,
                x_min=x_min,
                x_max=x_max,
                y_min=y_min,
                y_max=y_max,
                delay=delay,
                fov_x=fov_x,
                fov_y=fov_y
            )
            logger.info("📝 Parámetros de trayectoria guardados")
        except Exception as e:
            logger.warning(f"No se pudieron guardar parámetros: {e}")
    
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
            fov_x = float(self.fov_x_input.text())
            fov_y = float(self.fov_y_input.text())
            x_min = float(self.x_start_input.text())
            x_max = float(self.x_end_input.text())
            y_min = float(self.y_start_input.text())
            y_max = float(self.y_end_input.text())
            step_delay = float(self.delay_input.text())
            
            logger.info(
                f"Parámetros: FOV={fov_x}x{fov_y} µm, "
                f"X=[{x_min},{x_max}], Y=[{y_min},{y_max}], delay={step_delay}s"
            )
            
            # Generar trayectoria
            result = self.trajectory_gen.generate_zigzag_by_fov(
                x_min=x_min,
                x_max=x_max,
                y_min=y_min,
                y_max=y_max,
                fov_x=fov_x,
                fov_y=fov_y,
                step_delay=step_delay
            )
            
            if result['success']:
                self.current_trajectory = result['points']
                self.trajectory_step_delay = step_delay
                self.trajectory_index = 0
                n_points = result['n_points']
                
                # Guardar parámetros para autocompletado futuro
                self._save_trajectory_parameters(
                    n_points, x_min, x_max, y_min, y_max, step_delay, fov_x, fov_y
                )
                
                # Actualizar UI
                if self.points_input:
                    self.points_input.setText(str(n_points))
                self.set_trajectory_status(True, len(self.current_trajectory))
                self.results_text.append(f"✅ {result['message']}")
                
                # Guardar figura para vista previa
                self._trajectory_figure = result.get('figure')
                
                logger.info(f"✅ {result['message']}")
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
        from copy import deepcopy
        if not isinstance(controller_data, dict):
            logger.error(f"set_controller({motor}): datos inválidos")
            return

        # Copia independiente de escalares (A y B no deben compartir dict)
        tf_obj = controller_data.get('controller')
        data = deepcopy({k: v for k, v in controller_data.items() if k != 'controller'})
        if tf_obj is not None:
            data['controller'] = tf_obj

        if motor == 'A':
            self.controller_a = data
            info = f"Kp={data['Kp']:.4f}, Ki={data['Ki']:.4f}\n"
            info += f"γ={data.get('gamma', 0):.4f}, U_max={data.get('U_max', 100):.1f}"
            if data.get('slot_key'):
                info += f"\n[{data.get('slot_key')}]"
            self.set_controller_a(info, True)
            
            sensor_key = 'sensor_2' if self.motor_a_sensor2.isChecked() else 'sensor_1'
            config = ControllerConfig(
                Kp=float(data['Kp']),
                Ki=float(data['Ki']),
                U_max=float(data.get('U_max', 150)),
                invert=self.motor_a_invert.isChecked(),
                sensor_key=sensor_key,
                K_plant=float(data.get('K', 1.0)),
            )
            self.test_service.set_controller_a(config)
            logger.info(
                f"Controlador A guardado: Kp={data['Kp']:.4f}, Ki={data['Ki']:.4f}, "
                f"slot={data.get('slot_key')}"
            )
        else:
            self.controller_b = data
            info = f"Kp={data['Kp']:.4f}, Ki={data['Ki']:.4f}\n"
            info += f"γ={data.get('gamma', 0):.4f}, U_max={data.get('U_max', 100):.1f}"
            if data.get('slot_key'):
                info += f"\n[{data.get('slot_key')}]"
            self.set_controller_b(info, True)
            
            sensor_key = 'sensor_1' if self.motor_b_sensor1.isChecked() else 'sensor_2'
            config = ControllerConfig(
                Kp=float(data['Kp']),
                Ki=float(data['Ki']),
                U_max=float(data.get('U_max', 150)),
                invert=self.motor_b_invert.isChecked(),
                sensor_key=sensor_key,
                K_plant=float(data.get('K', 1.0)),
            )
            self.test_service.set_controller_b(config)
            logger.info(
                f"Controlador B guardado: Kp={data['Kp']:.4f}, Ki={data['Ki']:.4f}, "
                f"slot={data.get('slot_key')}"
            )

    def get_controller_preferences(self):
        """Retorna preferencias de sensor/inversión por motor."""
        sensor_map = {
            'A': 'sensor_2' if self.motor_a_sensor2.isChecked() else 'sensor_1',
            'B': 'sensor_1' if self.motor_b_sensor1.isChecked() else 'sensor_2',
        }
        invert_map = {
            'A': bool(self.motor_a_invert.isChecked()),
            'B': bool(self.motor_b_invert.isChecked()),
        }
        return sensor_map, invert_map

    def apply_controller_preferences(self, sensor_map=None, invert_map=None):
        """Aplica preferencias guardadas de sensor/inversión."""
        sensor_map = sensor_map or {}
        invert_map = invert_map or {}

        sensor_a = sensor_map.get('A')
        if sensor_a == 'sensor_2':
            self.motor_a_sensor2.setChecked(True)
        elif sensor_a == 'sensor_1':
            self.motor_a_sensor1.setChecked(True)

        sensor_b = sensor_map.get('B')
        if sensor_b == 'sensor_1':
            self.motor_b_sensor1.setChecked(True)
        elif sensor_b == 'sensor_2':
            self.motor_b_sensor2.setChecked(True)

        # invert_map vacío/None → no tocar checkboxes (respetar UI del operador)
        if invert_map:
            if 'A' in invert_map:
                self.motor_a_invert.setChecked(bool(invert_map.get('A')))
            if 'B' in invert_map:
                self.motor_b_invert.setChecked(bool(invert_map.get('B')))

    def _serializable_controller(self, controller_data):
        """Convierte controlador de UI a formato serializable."""
        if not isinstance(controller_data, dict):
            return None
        return {
            'Kp': float(controller_data.get('Kp', 0.0)),
            'Ki': float(controller_data.get('Ki', 0.0)),
            'K': float(controller_data.get('K', 0.0)),
            'tau': float(controller_data.get('tau', 0.0)),
            'U_max': float(controller_data.get('U_max', 100.0)),
            'gamma': float(controller_data.get('gamma', 0.0)),
            'K_sign': float(controller_data.get('K_sign', 1.0)),
            'Ms': float(controller_data.get('Ms', 0.0)),
            'wb': float(controller_data.get('wb', 0.0)),
        }

    def get_serializable_controllers(self):
        """Retorna controladores A/B en formato serializable."""
        return {
            'A': self._serializable_controller(self.controller_a),
            'B': self._serializable_controller(self.controller_b),
        }
    
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
    
    def _enforce_canonical_axis_mapping(self) -> list:
        """
        Fuerza solo el mapa físico de calibración/síntesis:
          Motor A → Sensor 2 (eje X), Motor B → Sensor 1 (eje Y).

        Invertir PWM lo decide el operador en la UI (no se pisa con el slot).
        """
        notes = []
        want_a = 'sensor_2'
        want_b = 'sensor_1'

        cur_a = 'sensor_2' if self.motor_a_sensor2.isChecked() else 'sensor_1'
        cur_b = 'sensor_1' if self.motor_b_sensor1.isChecked() else 'sensor_2'

        # Respetar checkboxes de inversión (antes se sobreescribían con invert_pwm del slot)
        inv_a = bool(self.motor_a_invert.isChecked())
        inv_b = bool(self.motor_b_invert.isChecked())

        if cur_a != want_a or cur_b != want_b:
            notes.append(
                f"Mapa sensor corregido: A {cur_a}→{want_a}, B {cur_b}→{want_b} "
                "(calibración / TF A_2 y B_1)"
            )
            self.apply_controller_preferences(
                {'A': want_a, 'B': want_b},
                None,  # no tocar invert
            )
        else:
            # Asegurar preferencias de sensor sin tocar invert
            self.apply_controller_preferences({'A': want_a, 'B': want_b}, None)

        if self.controller_a:
            self.test_service.update_controller_a_sensor(want_a, inv_a)
            if isinstance(self.controller_a, dict):
                self.controller_a['invert_pwm'] = inv_a
        if self.controller_b:
            self.test_service.update_controller_b_sensor(want_b, inv_b)
            if isinstance(self.controller_b, dict):
                self.controller_b['invert_pwm'] = inv_b

        if notes:
            for n in notes:
                self.results_text.append(f"⚠️ {n}")
                logger.warning(n)
        logger.info(
            f"Ejes listos: A→{want_a} invert={inv_a}, B→{want_b} invert={inv_b}"
        )
        return notes

    def _on_invert_toggled(self, motor: str, checked: bool) -> None:
        """Aplica Invertir PWM al instante (también con control ya activo)."""
        motor = motor.upper()
        if motor == 'A':
            sensor_key = 'sensor_2' if self.motor_a_sensor2.isChecked() else 'sensor_1'
            if self.controller_a:
                self.test_service.update_controller_a_sensor(sensor_key, bool(checked))
                if isinstance(self.controller_a, dict):
                    self.controller_a['invert_pwm'] = bool(checked)
            logger.info(f"Invert PWM Motor A = {checked}")
        else:
            sensor_key = 'sensor_1' if self.motor_b_sensor1.isChecked() else 'sensor_2'
            if self.controller_b:
                self.test_service.update_controller_b_sensor(sensor_key, bool(checked))
                if isinstance(self.controller_b, dict):
                    self.controller_b['invert_pwm'] = bool(checked)
            logger.info(f"Invert PWM Motor B = {checked}")
        if hasattr(self.parent_gui, '_save_session_state'):
            self.parent_gui._save_session_state()

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
        
        self._enforce_canonical_axis_mapping()
        
        # Delegar al servicio
        self.test_service.start_dual_control(ref_a, ref_b)
    
    def stop_dual_control(self):
        """Detiene el control dual con freno activo (delegado a TestService)."""
        logger.info("=== DETENIENDO CONTROL DUAL (via TestService) ===")
        self.test_service.stop_dual_control()
    
    # ============================================================
    # EJECUCIÓN DE TRAYECTORIA ZIG-ZAG
    # ============================================================
    
    def _confirm_tolerance_vs_fov(self, tolerance_um: float) -> bool:
        """True si la tolerancia de cierre es segura frente al FOV, o el usuario confirma.

        Regla operativa: tolerancia ≤ FOV/10 para conservar el solape del mosaico.
        """
        try:
            fov_min = min(
                float(self.fov_x_input.text()), float(self.fov_y_input.text())
            )
        except (ValueError, AttributeError):
            return True
        if fov_min <= 0:
            return True
        limit = fov_min / 10.0
        if tolerance_um <= limit:
            return True
        pct = tolerance_um / fov_min * 100.0
        reply = QMessageBox.warning(
            self.parent_gui,
            "Tolerancia demasiado amplia",
            f"La tolerancia de cierre ({tolerance_um:.0f} µm) es {pct:.0f}% del FOV "
            f"({fov_min:.0f} µm). Recomendado ≤ {limit:.0f} µm para no perder solape.\n\n"
            "¿Continuar de todas formas?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        return reply == QMessageBox.Yes

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
        
        # Guardrail: la tolerancia de cierre no debe acercarse al tamaño del FOV.
        if not self._confirm_tolerance_vs_fov(tolerance):
            self.results_text.append("⏹️ Ejecución cancelada: ajusta la tolerancia de cierre")
            return

        # Guardar para compatibilidad
        self.trajectory_tolerance = tolerance
        self.trajectory_pause = pause
        self.trajectory_index = 0
        
        # CRÍTICO: sin el mapa canónico el error µm no coincide con Synthesis
        # y el punto 1 nunca entra en tolerancia (p.ej. A→S1 muestra ~+200µm fantasma).
        self._enforce_canonical_axis_mapping()
        
        # Convertir trayectoria a lista de tuplas
        trajectory_list = [(p[0], p[1]) for p in self.current_trajectory]
        
        # Delegar al servicio con auto_advance=True para TestTab standalone
        if self.homogeneous_steps_cb is not None:
            self.test_service.set_step_control_enabled(self.homogeneous_steps_cb.isChecked())
        self.test_service.start_trajectory(trajectory_list, tolerance, pause, auto_advance=True)
    
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

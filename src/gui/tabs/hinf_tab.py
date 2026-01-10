"""
Pestaña de Diseño de Controlador H∞.

Encapsula la UI para diseño de controladores robustos H∞/H2.
Usa HInfController para la lógica de síntesis.
"""

import logging
import pickle
import traceback
import time
import numpy as np
import control as ct
from datetime import datetime
from matplotlib.figure import Figure
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QGroupBox, QLabel, QLineEdit, QPushButton,
                             QTextEdit, QCheckBox, QComboBox, QApplication,
                             QMessageBox, QFileDialog, QDialog, QRadioButton,
                             QDialogButtonBox)
from PyQt5.QtCore import pyqtSignal, QTimer
from gui.windows import MatplotlibWindow
from core.services.hinf_service import (
    simulate_step_response as hinf_simulate_step_response,
    plot_bode as hinf_plot_bode,
    export_controller as hinf_export_controller,
    load_previous_controller as hinf_load_previous_controller,
    start_hinf_control as hinf_start_control,
    execute_hinf_control as hinf_execute_control,
    stop_hinf_control as hinf_stop_control,
    synthesize_hinf_controller as hinf_synthesize_controller,
)

logger = logging.getLogger('MotorControl_L206')


class HInfTab(QWidget):
    """
    Pestaña para diseño de controladores H∞/H2.
    
    Signals:
        synthesis_requested: Solicita síntesis de controlador (config dict)
        load_from_analysis_requested: Solicita cargar K, τ desde análisis
        step_response_requested: Solicita simular respuesta al escalón
        bode_requested: Solicita diagrama de Bode
        export_requested: Solicita exportar controlador
        transfer_to_test_requested: Solicita transferir a pestaña Prueba
        control_toggle_requested: Solicita activar/desactivar control (bool)
    """
    
    synthesis_requested = pyqtSignal(dict)
    load_from_analysis_requested = pyqtSignal()
    step_response_requested = pyqtSignal()
    bode_requested = pyqtSignal()
    export_requested = pyqtSignal()
    transfer_to_test_requested = pyqtSignal()
    control_toggle_requested = pyqtSignal(bool)
    
    # Referencia a TestTab para transferencia directa
    test_tab_reference = None
    
    def __init__(self, hinf_controller=None, tf_analyzer=None, parent=None):
        """
        Inicializa la pestaña H∞.
        
        Args:
            hinf_controller: Instancia de HInfTrackingController (Zhou & Doyle)
            tf_analyzer: Instancia de TransferFunctionAnalyzer
            parent: Widget padre (ArduinoGUI)
        """
        super().__init__(parent)
        
        # Usar HInfController (implementación que FUNCIONA)
        if hinf_controller is None:
            from core.controllers.hinf_controller import HInfController
            self.hinf_controller = HInfController()
        else:
            self.hinf_controller = hinf_controller
        
        self.tf_analyzer = tf_analyzer
        self.parent_gui = parent
        
        # Variables para almacenar resultado de síntesis
        self.synthesized_controller = None
        self.synthesized_plant = None
        
        # Callbacks de hardware (inyección de dependencias)
        self.send_command_callback = None
        self.get_sensor_value_callback = None
        self.get_mode_label_callback = None
        
        # Variables de control en tiempo real
        self.control_active = False
        self.control_timer = None
        self.control_integral = 0.0
        self.control_last_time = None
        self.gamma = None
        
        # Ventanas auxiliares
        self.step_response_window = None
        self.bode_window = None
        
        self._setup_ui()
        logger.debug("HInfTab inicializado")
    
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
        logger.debug("Callbacks de hardware configurados en HInfTab")
    
    def _setup_ui(self):
        """Configura la interfaz de usuario."""
        layout = QVBoxLayout(self)
        
        # Sección 1: Parámetros de Planta
        plant_group = self._create_plant_section()
        layout.addWidget(plant_group)
        
        # Sección 2: Ponderaciones
        weights_group = self._create_weights_section()
        layout.addWidget(weights_group)
        
        # Warning label (oculto inicialmente)
        self.warning_label = QLabel("")
        self.warning_label.setStyleSheet(
            "background: #E74C3C; color: white; font-weight: bold; "
            "padding: 10px; border-radius: 5px;"
        )
        self.warning_label.setWordWrap(True)
        self.warning_label.setVisible(False)
        layout.addWidget(self.warning_label)
        
        # Info y método
        info = QLabel(
            "💡 Ms → amortiguamiento (1.2-1.7) | ωb → velocidad | U_max → límite PWM"
        )
        info.setStyleSheet("color: #5DADE2; font-size: 10px;")
        layout.addWidget(info)
        
        method_layout = QHBoxLayout()
        method_layout.addWidget(QLabel("Método:"))
        self.method_combo = QComboBox()
        self.method_combo.addItems(["H∞ (mixsyn)", "H2 (h2syn)"])
        method_layout.addWidget(self.method_combo)
        method_layout.addStretch()
        layout.addLayout(method_layout)
        
        # Botón síntesis
        synth_btn = QPushButton("🚀 Sintetizar Controlador Robusto")
        synth_btn.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px; background: #2E86C1;")
        synth_btn.clicked.connect(self._request_synthesis)
        layout.addWidget(synth_btn)
        
        # Resultados
        self.results_text = QTextEdit()
        self.results_text.setReadOnly(True)
        self.results_text.setPlaceholderText("Los resultados de la síntesis aparecerán aquí...")
        self.results_text.setMinimumHeight(300)
        layout.addWidget(self.results_text)
        
        # Botones de simulación
        sim_layout = QHBoxLayout()
        
        step_btn = QPushButton("📊 Respuesta al Escalón")
        step_btn.clicked.connect(self.simulate_step_response)
        sim_layout.addWidget(step_btn)
        
        bode_btn = QPushButton("📈 Diagrama de Bode")
        bode_btn.clicked.connect(self.plot_bode)
        sim_layout.addWidget(bode_btn)
        
        export_btn = QPushButton("💾 Exportar")
        export_btn.clicked.connect(self.export_controller)
        sim_layout.addWidget(export_btn)
        
        load_btn = QPushButton("📂 Cargar Previo")
        load_btn.clicked.connect(self.load_previous_controller)
        sim_layout.addWidget(load_btn)
        
        self.transfer_btn = QPushButton("➡️ Transferir a Prueba")
        self.transfer_btn.setStyleSheet("background: #27AE60; font-weight: bold;")
        self.transfer_btn.clicked.connect(self.transfer_to_test)  # Llamar directamente al método
        self.transfer_btn.setEnabled(False)
        sim_layout.addWidget(self.transfer_btn)
        
        layout.addLayout(sim_layout)
        
        # Control en tiempo real
        control_layout = QHBoxLayout()
        
        self.control_btn = QPushButton("🎮 Activar Control H∞")
        self.control_btn.setStyleSheet("font-weight: bold; padding: 8px; background: #27AE60;")
        self.control_btn.clicked.connect(self._toggle_control)
        self.control_btn.setEnabled(False)
        control_layout.addWidget(self.control_btn)
        
        control_layout.addWidget(QLabel("Ref (µm):"))
        self.reference_input = QLineEdit("5000")
        self.reference_input.setFixedWidth(80)
        control_layout.addWidget(self.reference_input)
        
        control_layout.addWidget(QLabel("Motor:"))
        self.motor_combo = QComboBox()
        self.motor_combo.addItems(["Motor A", "Motor B"])
        control_layout.addWidget(self.motor_combo)
        
        control_layout.addWidget(QLabel("Escala:"))
        self.scale_input = QLineEdit("0.1")
        self.scale_input.setFixedWidth(50)
        control_layout.addWidget(self.scale_input)
        
        layout.addLayout(control_layout)
    
    def _create_plant_section(self):
        """Crea sección de parámetros de planta."""
        group = QGroupBox("📐 Parámetros de la Planta G(s)")
        layout = QGridLayout()
        
        layout.addWidget(QLabel("Ganancia K (µm/s/PWM):"), 0, 0)
        self.K_input = QLineEdit("0.5598")
        self.K_input.setFixedWidth(100)
        layout.addWidget(self.K_input, 0, 1)
        
        layout.addWidget(QLabel("Constante τ (s):"), 1, 0)
        self.tau_input = QLineEdit("0.0330")
        self.tau_input.setFixedWidth(100)
        layout.addWidget(self.tau_input, 1, 1)
        
        layout.addWidget(QLabel("G(s) = K / (s·(τs + 1))"), 0, 2, 2, 1)
        
        btn_layout = QVBoxLayout()
        load_btn = QPushButton("⬅️ Cargar desde Análisis")
        load_btn.clicked.connect(self.load_plant_from_analysis)
        btn_layout.addWidget(load_btn)
        
        load_prev_btn = QPushButton("📂 Cargar Controlador Previo")
        load_prev_btn.setStyleSheet("background: #8E44AD; font-weight: bold;")
        btn_layout.addWidget(load_prev_btn)
        
        layout.addLayout(btn_layout, 0, 3, 2, 1)
        
        group.setLayout(layout)
        return group
    
    def _create_weights_section(self):
        """Crea sección de ponderaciones."""
        group = QGroupBox("⚖️ Funciones de Ponderación")
        layout = QGridLayout()
        
        # W1 - Performance
        layout.addWidget(QLabel("W₁ (Performance):"), 0, 0)
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
        layout.addLayout(w1_layout, 0, 1)
        
        # W2 - Control Effort
        layout.addWidget(QLabel("W₂ (Esfuerzo):"), 1, 0)
        w2_layout = QHBoxLayout()
        w2_layout.addWidget(QLabel("U_max="))
        self.w2_umax = QLineEdit("100")
        self.w2_umax.setFixedWidth(70)
        w2_layout.addWidget(self.w2_umax)
        w2_layout.addWidget(QLabel("PWM"))
        self.invert_pwm = QCheckBox("⇄ Invertir PWM")
        self.invert_pwm.setChecked(True)
        w2_layout.addWidget(self.invert_pwm)
        w2_layout.addStretch()
        layout.addLayout(w2_layout, 1, 1)
        
        # W3 - Robustness
        layout.addWidget(QLabel("W₃ (Robustez):"), 2, 0)
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
        layout.addLayout(w3_layout, 2, 1)
        
        group.setLayout(layout)
        return group
    
    def _request_synthesis(self):
        """Ejecuta síntesis con parámetros actuales."""
        # Ahora llama directamente al método local en lugar de emitir señal
        self.synthesize_hinf_controller()
    
    def _toggle_control(self):
        """Alterna estado de control."""
        # El estado actual se maneja en el padre
        self.control_toggle_requested.emit(True)
    
    # === Métodos para actualizar estado ===
    
    def set_plant_params(self, K: float, tau: float):
        """Establece parámetros de planta."""
        self.K_input.setText(f"{K:.4f}")
        self.tau_input.setText(f"{tau:.4f}")
    
    def set_results(self, text: str):
        """Establece texto de resultados."""
        self.results_text.setText(text)
    
    def append_results(self, text: str):
        """Agrega texto a resultados."""
        self.results_text.append(text)
    
    def set_warning(self, text: str, visible: bool = True):
        """Muestra/oculta advertencia."""
        self.warning_label.setText(text)
        self.warning_label.setVisible(visible)
    
    def enable_transfer(self, enabled: bool):
        """Habilita/deshabilita botón de transferencia."""
        self.transfer_btn.setEnabled(enabled)
    
    def enable_control(self, enabled: bool):
        """Habilita/deshabilita botón de control."""
        self.control_btn.setEnabled(enabled)
    
    def set_control_active(self, active: bool):
        """Actualiza estado visual del control."""
        if active:
            self.control_btn.setText("⏹️ Detener Control H∞")
            self.control_btn.setStyleSheet("font-weight: bold; padding: 8px; background: #E74C3C;")
        else:
            self.control_btn.setText("🎮 Activar Control H∞")
            self.control_btn.setStyleSheet("font-weight: bold; padding: 8px; background: #27AE60;")
    
    def get_reference(self) -> float:
        """Obtiene referencia actual."""
        try:
            return float(self.reference_input.text())
        except ValueError:
            return 5000.0
    
    def get_motor(self) -> str:
        """Obtiene motor seleccionado."""
        return 'A' if self.motor_combo.currentIndex() == 0 else 'B'
    
    def get_scale(self) -> float:
        """Obtiene factor de escala."""
        try:
            return float(self.scale_input.text())
        except ValueError:
            return 0.1
    
    def load_plant_from_analysis(self):
        """Carga K y τ desde funciones de transferencia identificadas."""
        logger.info("HInfTab: Cargando planta desde análisis")
        
        if not self.tf_analyzer:
            self.results_text.setText("❌ Error: No hay analizador disponible")
            logger.error("tf_analyzer no disponible")
            return
        
        tf_list = self.tf_analyzer.identified_functions
        
        if not tf_list:
            self.results_text.setText("ℹ️ Realiza primero un análisis en la pestaña 'Análisis' para identificar funciones de transferencia.")
            logger.warning("No hay funciones de transferencia identificadas")
            return
        
        # Si solo hay una, cargarla directamente
        if len(tf_list) == 1:
            tf = tf_list[0]
            self.set_plant_params(tf['K'], tf['tau'])
            
            tau_slow = tf.get('tau_slow', 1000.0)
            msg = (
                f"✅ Parámetros cargados:\n"
                f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                f"  Motor {tf['motor']} / Sensor {tf['sensor']}\n"
                f"  Fecha: {tf['timestamp']}\n\n"
                f"📐 MODELO:\n"
                f"  G(s) = K / ((τ₁s + 1)(τ₂s + 1))\n\n"
                f"  K  = {tf['K']:.4f} µm/s/PWM\n"
                f"  τ₁ = {tf['tau']:.4f}s (polo rápido)\n"
                f"  τ₂ = {tau_slow:.1f}s (polo lento)\n\n"
                f"Ahora puedes ajustar las ponderaciones y sintetizar el controlador."
            )
            self.results_text.setText(msg)
            logger.info(f"Parámetros cargados: Motor {tf['motor']}/Sensor {tf['sensor']}, K={tf['K']:.4f}, τ={tf['tau']:.4f}")
            return
        
        # Si hay múltiples, mostrar solo la más reciente
        tf = tf_list[-1]  # La más reciente
        self.set_plant_params(tf['K'], tf['tau'])
        
        msg = (
            f"✅ Parámetros cargados (última función):\n"
            f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            f"  Motor {tf['motor']} / Sensor {tf['sensor']}\n"
            f"  K  = {tf['K']:.4f} µm/s/PWM\n"
            f"  τ  = {tf['tau']:.4f}s\n\n"
            f"💡 Hay {len(tf_list)} funciones identificadas. Usando la más reciente."
        )
        self.results_text.setText(msg)
        logger.info(f"Cargada función más reciente: K={tf['K']:.4f}, τ={tf['tau']:.4f}")
    
    # ================================================================
    # LÓGICA DE CONTROLADOR H∞ (movida desde main.py)
    # ================================================================
    
    def set_synthesis_result(self, controller, plant, gamma):
        """
        Guarda el resultado de síntesis para uso posterior.
        
        Args:
            controller: TransferFunction del controlador sintetizado
            plant: TransferFunction de la planta
            gamma: Valor gamma de la síntesis
        """
        self.synthesized_controller = controller
        self.synthesized_plant = plant
        self.gamma = gamma
        logger.debug(f"Resultado de síntesis guardado: γ={gamma:.4f}")
        
        # Habilitar botones de control y transferencia
        self.control_btn.setEnabled(True)
        self.transfer_btn.setEnabled(True)
        logger.info("Botones de control y transferencia habilitados")
    
    def simulate_step_response(self):
        """Simula y grafica la respuesta al escalón del lazo cerrado."""
        hinf_simulate_step_response(self)
    
    def plot_bode(self):
        """Grafica el diagrama de Bode del lazo abierto."""
        hinf_plot_bode(self)
    
    def export_controller(self):
        """Exporta el controlador a archivo de texto y pickle."""
        hinf_export_controller(self)
    
    def load_previous_controller(self):
        """Carga un controlador H∞ guardado desde archivo pickle."""
        hinf_load_previous_controller(self)
    
    # ============================================================
    # CONTROL H∞ EN TIEMPO REAL (usando callbacks de hardware)
    # ============================================================
    
    def toggle_hinf_control(self):
        """Activa/desactiva control H∞ en tiempo real."""
        if not self.control_active:
            self.start_hinf_control()
        else:
            self.stop_hinf_control()
    
    def start_hinf_control(self):
        """Inicia control H∞ en tiempo real usando callbacks."""
        hinf_start_control(self)
    
    def execute_hinf_control(self):
        """Ejecuta un ciclo del controlador PI H∞."""
        hinf_execute_control(self)
    
    def stop_hinf_control(self):
        """Detiene el control H∞ en tiempo real."""
        hinf_stop_control(self)
    
    def synthesize_hinf_controller(self):
        """Sintetiza el controlador H∞ usando control.mixsyn() - Método estándar."""
        hinf_synthesize_controller(self)


    def set_test_tab_reference(self, test_tab):
        """Configura la referencia a TestTab para transferencias."""
        self.test_tab_reference = test_tab
        logger.debug(f"TestTab reference configurada en HInfTab")
    
    def transfer_to_test(self):
        """Transfiere el controlador sintetizado a TestTab."""
        logger.info("HInfTab: Iniciando transferencia a TestTab")
        
        # Verificar que hay controlador sintetizado
        if self.synthesized_controller is None:
            QMessageBox.warning(self, "Error", "No hay controlador sintetizado para transferir")
            logger.warning("No hay controlador sintetizado")
            return
        
        # Verificar que TestTab está configurado
        if self.test_tab_reference is None:
            QMessageBox.warning(self, "Error", "TestTab no está configurado")
            logger.error("TestTab reference no configurada")
            return
        
        # Obtener parámetros del controlador con verificación detallada
        try:
            # Verificar atributos críticos uno por uno
            if not hasattr(self, 'Kp_designed'):
                raise AttributeError("Kp_designed no está definido. Sintetiza o carga un controlador primero.")
            if not hasattr(self, 'Ki_designed'):
                raise AttributeError("Ki_designed no está definido. Sintetiza o carga un controlador primero.")
            if not hasattr(self, 'K_value'):
                raise AttributeError("K_value no está definido. Carga la planta desde Análisis primero.")
            if not hasattr(self, 'tau_value'):
                raise AttributeError("tau_value no está definido. Carga la planta desde Análisis primero.")
            if not hasattr(self, 'Umax_designed'):
                raise AttributeError("Umax_designed no está definido. Sintetiza o carga un controlador primero.")
            
            Kp = self.Kp_designed
            Ki = self.Ki_designed
            K_abs = abs(self.K_value)
            K_original = self.K_value
            tau = self.tau_value
            Ms = float(self.w1_Ms.text())
            wb = float(self.w1_wb.text())
            U_max = self.Umax_designed
            gamma = self.gamma
            
            logger.info(f"Parámetros a transferir: Kp={Kp:.4f}, Ki={Ki:.4f}, K={K_abs:.4f}, τ={tau:.4f}")
        except AttributeError as e:
            QMessageBox.warning(self, "Error", f"Parámetros incompletos:\n\n{str(e)}\n\nPasos necesarios:\n1. Cargar planta desde Análisis\n2. Sintetizar controlador H∞\n3. Transferir a Prueba")
            logger.error(f"Error obteniendo parámetros: {e}")
            return
        except Exception as e:
            QMessageBox.warning(self, "Error", f"No se pudieron obtener parámetros: {e}")
            logger.error(f"Error obteniendo parámetros: {e}")
            return
        
        # Preguntar a qué motor transferir
        dialog = QDialog(self)
        dialog.setWindowTitle("Transferir Controlador H∞")
        dialog.setGeometry(100, 100, 500, 600)
        layout = QVBoxLayout()
        
        # Mostrar resumen
        summary = QTextEdit()
        summary.setReadOnly(True)
        summary.setMaximumHeight(400)
        summary_text = (
            f"╔══════════════════════════════════════════════════╗\n"
            f"║  PARÁMETROS DEL CONTROLADOR H∞                   ║\n"
            f"╠══════════════════════════════════════════════════╣\n"
            f"║  PLANTA G(s):                                    ║\n"
            f"║    K = {K_original:+.4f} µm/s/PWM                     ║\n"
            f"║    τ = {tau:.4f} s                                    ║\n"
            f"║    G(s) = {K_abs:.4f} / (s·({tau:.4f}s + 1))         ║\n"
            f"╠══════════════════════════════════════════════════╣\n"
            f"║  CONTROLADOR K(s):                               ║\n"
            f"║    Kp = {Kp:.4f}                                     ║\n"
            f"║    Ki = {Ki:.4f}                                     ║\n"
            f"║    K(s) = ({Kp:.4f}·s + {Ki:.4f}) / s               ║\n"
            f"╠══════════════════════════════════════════════════╣\n"
            f"║  PONDERACIONES:                                  ║\n"
            f"║    Ms = {Ms:.2f}, ωb = {wb:.2f} rad/s                 ║\n"
            f"║    U_max = {U_max:.1f} PWM                             ║\n"
            f"║    γ = {gamma:.4f}                                    ║\n"
            f"╚══════════════════════════════════════════════════╝\n"
        )
        summary.setText(summary_text)
        summary.setStyleSheet("font-family: 'Courier New'; font-size: 10px;")
        layout.addWidget(summary)
        
        layout.addWidget(QLabel("\n¿A qué motor deseas transferir?"))
        
        motor_a_radio = QRadioButton("Motor A (X)")
        motor_b_radio = QRadioButton("Motor B (Y)")
        both_radio = QRadioButton("Ambos motores")
        motor_b_radio.setChecked(True)  # Por defecto Motor B
        
        layout.addWidget(motor_a_radio)
        layout.addWidget(motor_b_radio)
        layout.addWidget(both_radio)
        
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        
        dialog.setLayout(layout)
        
        if dialog.exec_() == QDialog.Accepted:
            transferred_motors = []
            
            # Crear info del controlador
            controller_data = {
                'controller': self.synthesized_controller,
                'Kp': Kp,
                'Ki': Ki,
                'K': K_abs,
                'K_sign': np.sign(K_original),
                'tau': tau,
                'Ms': Ms,
                'wb': wb,
                'U_max': U_max,
                'gamma': gamma
            }
            
            if motor_a_radio.isChecked() or both_radio.isChecked():
                self.test_tab_reference.set_controller('A', controller_data)
                transferred_motors.append("Motor A")
                logger.info("Controlador transferido a Motor A")
            
            if motor_b_radio.isChecked() or both_radio.isChecked():
                self.test_tab_reference.set_controller('B', controller_data)
                transferred_motors.append("Motor B")
                logger.info("Controlador transferido a Motor B")
            
            motor_names = " y ".join(transferred_motors)
            
            QMessageBox.information(self, "✅ Transferencia Exitosa",
                                   f"Controlador transferido a {motor_names}:\n\n"
                                   f"Kp = {Kp:.4f}\n"
                                   f"Ki = {Ki:.4f}\n\n"
                                   f"Planta: K = {K_abs:.4f} µm/s/PWM, τ = {tau:.4f} s\n\n"
                                   f"Revisa la pestaña 'Prueba' para usar el controlador.")
            
            logger.info(f"Transferencia completada a {motor_names}")
    

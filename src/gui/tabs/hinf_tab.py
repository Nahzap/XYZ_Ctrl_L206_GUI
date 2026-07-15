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
            parent: Widget padre (CTRL_GUI)
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
        self.current_slot_key = None  # Ej: "A_2"
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
        load_prev_btn.clicked.connect(self.load_previous_controller)
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
        """Carga K y τ desde funciones de transferencia identificadas (elige A/B)."""
        logger.info("HInfTab: Cargando planta desde análisis")
        
        if not self.tf_analyzer:
            self.results_text.setText("❌ Error: No hay analizador disponible")
            logger.error("tf_analyzer no disponible")
            return
        
        tf_list = list(self.tf_analyzer.identified_functions)
        
        if not tf_list:
            self.results_text.setText(
                "ℹ️ Realiza primero un análisis en la pestaña 'Análisis' "
                "para identificar funciones de transferencia."
            )
            logger.warning("No hay funciones de transferencia identificadas")
            return
        
        tf = None
        if len(tf_list) == 1:
            tf = tf_list[0]
        else:
            # Diálogo: no asumir siempre la última (antes B bloqueaba a A)
            dialog = QDialog(self)
            dialog.setWindowTitle("Seleccionar planta (Análisis)")
            dialog.setMinimumWidth(420)
            layout = QVBoxLayout(dialog)
            layout.addWidget(QLabel(
                "Hay varias TF identificadas. Elige cuál cargar en H∞:"
            ))
            radios = []
            for i, entry in enumerate(tf_list):
                motor = entry.get('motor', '?')
                sensor = entry.get('sensor', '?')
                K = float(entry.get('K', 0.0))
                tau = float(entry.get('tau', 0.0))
                ts = entry.get('timestamp', '')
                label = (
                    f"Motor {motor} / Sensor {sensor}  |  "
                    f"K={K:.4f}  τ={tau:.4f}s  |  {ts}"
                )
                rb = QRadioButton(label)
                rb.setProperty('tf_index', i)
                if str(motor).upper() == 'A' and str(sensor) in ('2', 'S2'):
                    # Preferir A_2 si el combo de control está en A
                    prefer_a = (
                        hasattr(self, 'control_motor_combo')
                        and 'A' in str(self.control_motor_combo.currentText()).upper()
                    )
                    if prefer_a or not radios:
                        rb.setChecked(True)
                elif not any(r.isChecked() for r in radios):
                    rb.setChecked(True)
                radios.append(rb)
                layout.addWidget(rb)
            # Si ninguna quedó marcada, marcar la última
            if radios and not any(r.isChecked() for r in radios):
                radios[-1].setChecked(True)

            buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
            buttons.accepted.connect(dialog.accept)
            buttons.rejected.connect(dialog.reject)
            layout.addWidget(buttons)

            if dialog.exec_() != QDialog.Accepted:
                return
            chosen = next((r for r in radios if r.isChecked()), None)
            if chosen is None:
                return
            tf = tf_list[int(chosen.property('tf_index'))]

        self._apply_loaded_plant_tf(tf)

    def _apply_loaded_plant_tf(self, tf: dict) -> None:
        """Aplica K/τ de una TF identificada y avisa si no es sintetizable."""
        motor = str(tf.get('motor', '?'))
        sensor = str(tf.get('sensor', '?'))
        K = float(tf.get('K', 0.0))
        tau = float(tf.get('tau', 0.0))
        tau_slow = float(tf.get('tau_slow', 1000.0))
        self.set_plant_params(K, tau)
        self.current_slot_key = f"{motor}_{sensor}"

        # Sincronizar combo de motor de control H∞ live
        if hasattr(self, 'control_motor_combo'):
            want = f"Motor {motor}"
            idx = self.control_motor_combo.findText(want)
            if idx < 0:
                idx = self.control_motor_combo.findText(motor)
            if idx >= 0:
                self.control_motor_combo.setCurrentIndex(idx)

        warn = ""
        if tau <= 1e-6:
            warn = (
                "\n\n⚠️ BLOQUEO: τ≈0 — esta identificación no es usable para síntesis H∞.\n"
                "Re-identifica Motor "
                f"{motor}/Sensor {sensor} en Análisis (necesitas τ>0) "
                "o carga un controlador previo / slot guardado."
            )
            logger.warning(
                f"Planta {motor}_{sensor} con τ={tau} — síntesis quedará bloqueada"
            )

        msg = (
            f"✅ Parámetros cargados:\n"
            f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            f"  Slot: {self.current_slot_key}\n"
            f"  Motor {motor} / Sensor {sensor}\n"
            f"  Fecha: {tf.get('timestamp', '')}\n\n"
            f"📐 MODELO:\n"
            f"  K  = {K:.4f} µm/s/PWM\n"
            f"  τ₁ = {tau:.4f}s (polo rápido)\n"
            f"  τ₂ = {tau_slow:.1f}s (polo lento)"
            f"{warn}\n\n"
            f"Luego: Sintetizar → Transferir a Prueba → Solo Motor {motor}."
        )
        self.results_text.setText(msg)
        logger.info(
            f"Parámetros cargados: Motor {motor}/Sensor {sensor}, "
            f"K={K:.4f}, τ={tau:.4f}, slot={self.current_slot_key}"
        )
    
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
        if hasattr(self.parent_gui, '_save_session_state'):
            self.parent_gui._save_session_state()
    
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
        if self.synthesized_controller is not None and hasattr(self.parent_gui, '_save_session_state'):
            self.parent_gui._save_session_state()


    def set_test_tab_reference(self, test_tab):
        """Configura la referencia a TestTab para transferencias."""
        self.test_tab_reference = test_tab
        logger.debug(f"TestTab reference configurada en HInfTab")

    def get_active_slot_key(self):
        """Retorna slot activo (motor_sensor) para persistencia."""
        return self.current_slot_key or "DEFAULT"

    def set_active_slot_key(self, slot_key):
        """Permite fijar slot activo desde persistencia."""
        if isinstance(slot_key, str) and slot_key:
            self.current_slot_key = slot_key

    def _tf_to_num_den(self, system):
        """Convierte sistema control.TransferFunction/StateSpace a num/den serializable."""
        if system is None:
            return None, None

        if hasattr(system, 'A') and not hasattr(system, 'num'):
            tf_system = ct.ss2tf(system)
            num = np.array(tf_system.num[0][0]).flatten().tolist()
            den = np.array(tf_system.den[0][0]).flatten().tolist()
            return num, den

        num = np.array(system.num[0][0]).flatten().tolist()
        den = np.array(system.den[0][0]).flatten().tolist()
        return num, den

    def build_hinf_snapshot(self):
        """
        Construye snapshot serializable del estado H∞ actual.

        Returns:
            dict o None si no hay síntesis válida.
        """
        if self.synthesized_controller is None or self.synthesized_plant is None:
            return None

        try:
            controller_num, controller_den = self._tf_to_num_den(self.synthesized_controller)
            plant_num, plant_den = self._tf_to_num_den(self.synthesized_plant)
            snapshot = {
                'slot_key': self.get_active_slot_key(),
                'plant': {
                    'K': float(self.K_input.text()),
                    'tau': float(self.tau_input.text()),
                    'num': plant_num,
                    'den': plant_den,
                },
                'weights': {
                    'method': self.method_combo.currentText(),
                    'Ms': float(self.w1_Ms.text()),
                    'wb': float(self.w1_wb.text()),
                    'eps': float(self.w1_eps.text()),
                    'U_max': float(self.w2_umax.text()),
                    'w_unc': float(self.w3_wunc.text()),
                    'eps_T': float(self.w3_epsT.text()),
                    'invert_pwm': bool(self.invert_pwm.isChecked()),
                },
                'result': {
                    'gamma': float(self.gamma) if self.gamma is not None else 0.0,
                    'Kp': float(getattr(self, 'Kp_designed', 0.0)),
                    'Ki': float(getattr(self, 'Ki_designed', 0.0)),
                    'K_value': float(getattr(self, 'K_value', self.K_input.text())),
                    'tau_value': float(getattr(self, 'tau_value', self.tau_input.text())),
                    'Umax_designed': float(getattr(self, 'Umax_designed', self.w2_umax.text())),
                    'controller_num': controller_num,
                    'controller_den': controller_den,
                }
            }
            return snapshot
        except Exception as e:
            logger.error(f"No se pudo construir snapshot H∞: {e}")
            return None

    def apply_hinf_snapshot(self, snapshot):
        """Restaura estado H∞ desde snapshot serializable."""
        if not isinstance(snapshot, dict):
            return False

        try:
            self.set_active_slot_key(snapshot.get('slot_key', 'DEFAULT'))

            plant = snapshot.get('plant', {})
            weights = snapshot.get('weights', {})
            result = snapshot.get('result', {})

            if 'K' in plant:
                self.K_input.setText(str(plant.get('K')))
            if 'tau' in plant:
                self.tau_input.setText(str(plant.get('tau')))

            if 'method' in weights:
                idx = self.method_combo.findText(str(weights.get('method')))
                if idx >= 0:
                    self.method_combo.setCurrentIndex(idx)
            if 'Ms' in weights:
                self.w1_Ms.setText(str(weights.get('Ms')))
            if 'wb' in weights:
                self.w1_wb.setText(str(weights.get('wb')))
            if 'eps' in weights:
                self.w1_eps.setText(str(weights.get('eps')))
            if 'U_max' in weights:
                self.w2_umax.setText(str(weights.get('U_max')))
            if 'w_unc' in weights:
                self.w3_wunc.setText(str(weights.get('w_unc')))
            if 'eps_T' in weights:
                self.w3_epsT.setText(str(weights.get('eps_T')))
            self.invert_pwm.setChecked(bool(weights.get('invert_pwm', True)))

            controller_num = result.get('controller_num')
            controller_den = result.get('controller_den')
            plant_num = plant.get('num')
            plant_den = plant.get('den')
            if not all([controller_num, controller_den, plant_num, plant_den]):
                return False

            self.synthesized_controller = ct.TransferFunction(controller_num, controller_den)
            self.synthesized_plant = ct.TransferFunction(plant_num, plant_den)
            self.gamma = float(result.get('gamma', 0.0))

            self.Kp_designed = float(result.get('Kp', 0.0))
            self.Ki_designed = float(result.get('Ki', 0.0))
            self.K_value = float(result.get('K_value', plant.get('K', 0.0)))
            self.tau_value = float(result.get('tau_value', plant.get('tau', 0.0)))
            self.Umax_designed = float(result.get('Umax_designed', weights.get('U_max', 100.0)))

            self.control_btn.setEnabled(True)
            self.transfer_btn.setEnabled(True)

            self.results_text.append(
                f"\n♻️ Modelo H∞ restaurado ({self.get_active_slot_key()}) | "
                f"Kp={self.Kp_designed:.4f}, Ki={self.Ki_designed:.4f}, γ={self.gamma:.4f}"
            )
            return True
        except Exception as e:
            logger.error(f"Error restaurando snapshot H∞: {e}\n{traceback.format_exc()}")
            return False
    
    def transfer_to_test(self):
        """Transfiere el controlador sintetizado a TestTab (A y B independientes)."""
        logger.info("HInfTab: Iniciando transferencia a TestTab")
        
        if self.synthesized_controller is None:
            QMessageBox.warning(self, "Error", "No hay controlador sintetizado para transferir")
            logger.warning("No hay controlador sintetizado")
            return
        
        if self.test_tab_reference is None:
            QMessageBox.warning(self, "Error", "TestTab no está configurado")
            logger.error("TestTab reference no configurada")
            return
        
        try:
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
            slot_key = self.get_active_slot_key()
            invert_pwm = bool(self.invert_pwm.isChecked())
            
            logger.info(
                f"Parámetros a transferir [{slot_key}]: "
                f"Kp={Kp:.4f}, Ki={Ki:.4f}, K={K_abs:.4f}, τ={tau:.4f}, invert={invert_pwm}"
            )
        except AttributeError as e:
            QMessageBox.warning(
                self, "Error",
                f"Parámetros incompletos:\n\n{str(e)}\n\n"
                "Pasos necesarios:\n1. Cargar planta desde Análisis\n"
                "2. Sintetizar controlador H∞\n3. Transferir a Prueba"
            )
            logger.error(f"Error obteniendo parámetros: {e}")
            return
        except Exception as e:
            QMessageBox.warning(self, "Error", f"No se pudieron obtener parámetros: {e}")
            logger.error(f"Error obteniendo parámetros: {e}")
            return
        
        dialog = QDialog(self)
        dialog.setWindowTitle("Transferir Controlador H∞")
        dialog.setGeometry(100, 100, 520, 640)
        layout = QVBoxLayout()
        
        summary = QTextEdit()
        summary.setReadOnly(True)
        summary.setMaximumHeight(360)
        summary_text = (
            f"Slot activo: {slot_key}\n"
            f"Invert PWM (síntesis): {invert_pwm}\n\n"
            f"Planta: K={K_original:+.4f} µm/s/PWM, τ={tau:.4f} s\n"
            f"Controlador: Kp={Kp:.4f}, Ki={Ki:.4f}\n"
            f"Ms={Ms:.2f}, ωb={wb:.2f}, U_max={U_max:.1f}, γ={gamma:.4f}\n"
        )
        summary.setText(summary_text)
        summary.setStyleSheet("font-family: 'Courier New'; font-size: 10px;")
        layout.addWidget(summary)
        
        layout.addWidget(QLabel("¿A qué motor deseas transferir?"))
        
        motor_a_radio = QRadioButton("Solo Motor A (X) — deja B intacto")
        motor_b_radio = QRadioButton("Solo Motor B (Y) — deja A intacto")
        both_same_radio = QRadioButton("Ambos con ESTE diseño (idénticos — no recomendado)")
        both_slots_radio = QRadioButton("Slots guardados: A_* → Motor A y B_* → Motor B")
        
        # Destino por defecto = letra del slot (A_2→A, B_1→B). Nunca default a B a ciegas.
        default_motor = 'A' if str(slot_key).upper().startswith('A') else 'B'
        if default_motor == 'A':
            motor_a_radio.setChecked(True)
        else:
            motor_b_radio.setChecked(True)
        
        layout.addWidget(motor_a_radio)
        layout.addWidget(motor_b_radio)
        layout.addWidget(both_same_radio)
        layout.addWidget(both_slots_radio)
        
        hint = QLabel(
            "Para controladores distintos: sintetiza/carga A_2 → Solo Motor A, "
            "luego B_1 → Solo Motor B.\n"
            "O usa 'Slots guardados' si ambos ya están en la sesión."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #AAAAAA; font-size: 10px;")
        layout.addWidget(hint)
        
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        dialog.setLayout(layout)
        
        if dialog.exec_() != QDialog.Accepted:
            return

        if both_slots_radio.isChecked():
            ok, msg = self._transfer_slots_to_test()
            if ok:
                QMessageBox.information(self, "Transferencia Exitosa", msg)
            else:
                QMessageBox.warning(self, "Transferencia incompleta", msg)
            return

        controller_data = {
            'controller': self.synthesized_controller,
            'Kp': float(Kp),
            'Ki': float(Ki),
            'K': float(K_abs),
            'K_sign': float(np.sign(K_original)),
            'tau': float(tau),
            'Ms': float(Ms),
            'wb': float(wb),
            'U_max': float(U_max),
            'gamma': float(gamma),
            'slot_key': slot_key,
            'invert_pwm': invert_pwm,
        }

        transferred_motors = []
        if motor_a_radio.isChecked() or both_same_radio.isChecked():
            self._push_controller_to_test('A', controller_data)
            transferred_motors.append(f"Motor A (Kp={Kp:.4f}, Ki={Ki:.4f})")
        if motor_b_radio.isChecked() or both_same_radio.isChecked():
            self._push_controller_to_test('B', controller_data)
            transferred_motors.append(f"Motor B (Kp={Kp:.4f}, Ki={Ki:.4f})")

        motor_names = "\n".join(f"• {m}" for m in transferred_motors)
        warn = ""
        if both_same_radio.isChecked():
            warn = (
                "\n\nAVISO: Ambos motores recibieron el MISMO diseño. "
                "Si querías A≠B, usa transferencias por separado o 'Slots guardados'."
            )
        QMessageBox.information(
            self, "Transferencia Exitosa",
            f"Transferido:\n{motor_names}{warn}\n\nRevisa la pestaña Prueba."
        )
        logger.info(f"Transferencia completada: {transferred_motors}")
        if hasattr(self.parent_gui, '_save_session_state'):
            self.parent_gui._save_session_state()

    def _controller_data_from_hinf_snapshot(self, snapshot: dict) -> dict:
        """Convierte snapshot de slot H∞ a dict usable por TestTab."""
        result = snapshot.get('result', {}) or {}
        weights = snapshot.get('weights', {}) or {}
        plant = snapshot.get('plant', {}) or {}
        K_val = float(result.get('K_value', plant.get('K', 1.0)))
        return {
            'Kp': float(result.get('Kp', 0.0)),
            'Ki': float(result.get('Ki', 0.0)),
            'K': abs(K_val),
            'K_sign': float(np.sign(K_val) if K_val != 0 else 1.0),
            'tau': float(result.get('tau_value', plant.get('tau', 0.0))),
            'Ms': float(weights.get('Ms', 0.0)),
            'wb': float(weights.get('wb', 0.0)),
            'U_max': float(result.get('Umax_designed', weights.get('U_max', 150.0))),
            'gamma': float(result.get('gamma', 0.0)),
            'slot_key': str(snapshot.get('slot_key', '')),
            'invert_pwm': bool(weights.get('invert_pwm', False)),
        }

    def _push_controller_to_test(self, motor: str, controller_data: dict) -> None:
        """Entrega una COPIA independiente al motor indicado + preferencias de slot."""
        from copy import deepcopy
        tf_obj = controller_data.get('controller')
        payload = deepcopy({k: v for k, v in controller_data.items() if k != 'controller'})
        if tf_obj is not None:
            payload['controller'] = tf_obj

        motor = motor.upper()
        slot_key = str(payload.get('slot_key', '') or '')
        # Sensor canónico: A→2, B→1. Invert del slot solo como valor inicial en UI.
        if motor == 'A':
            sensor = '2'
            if '_' in slot_key:
                sensor = slot_key.split('_', 1)[1][:1] or '2'
            prefs_invert = {'A': bool(payload.get('invert_pwm', False))}
            self.test_tab_reference.apply_controller_preferences(
                {'A': f'sensor_{sensor}'},
                prefs_invert,
            )
        else:
            sensor = '1'
            if '_' in slot_key:
                sensor = slot_key.split('_', 1)[1][:1] or '1'
            prefs_invert = {'B': bool(payload.get('invert_pwm', False))}
            self.test_tab_reference.apply_controller_preferences(
                {'B': f'sensor_{sensor}'},
                prefs_invert,
            )

        self.test_tab_reference.set_controller(motor, payload)

    def _transfer_slots_to_test(self):
        """
        Transfiere controladores distintos desde hinf.slots:
        primer slot A_* → Motor A, primer slot B_* → Motor B.
        """
        parent = self.parent_gui
        if parent is None or not hasattr(parent, 'session_store'):
            return False, "No hay session_store disponible."

        session = parent.session_store.get_session()
        slots = (session.get('hinf') or {}).get('slots') or {}
        if not slots:
            return False, "No hay slots H∞ guardados en la sesión."

        slot_a = next((k for k in sorted(slots) if str(k).upper().startswith('A')), None)
        slot_b = next((k for k in sorted(slots) if str(k).upper().startswith('B')), None)
        if not slot_a and not slot_b:
            return False, f"Slots presentes sin prefijo A_/B_: {list(slots.keys())}"

        lines = []
        if slot_a:
            data_a = self._controller_data_from_hinf_snapshot(slots[slot_a])
            self._push_controller_to_test('A', data_a)
            lines.append(
                f"Motor A ← {slot_a}: Kp={data_a['Kp']:.4f}, Ki={data_a['Ki']:.4f}, "
                f"invert={data_a['invert_pwm']}"
            )
        if slot_b:
            data_b = self._controller_data_from_hinf_snapshot(slots[slot_b])
            self._push_controller_to_test('B', data_b)
            lines.append(
                f"Motor B ← {slot_b}: Kp={data_b['Kp']:.4f}, Ki={data_b['Ki']:.4f}, "
                f"invert={data_b['invert_pwm']}"
            )

        if hasattr(parent, '_save_session_state'):
            parent._save_session_state()

        if slot_a and slot_b:
            same = (
                abs(float(slots[slot_a]['result']['Kp']) - float(slots[slot_b]['result']['Kp'])) < 1e-9
                and abs(float(slots[slot_a]['result']['Ki']) - float(slots[slot_b]['result']['Ki'])) < 1e-9
            )
            if same:
                lines.append(
                    "\nAVISO: A_* y B_* tienen los mismos Kp/Ki en la sesión "
                    "(re-sintetiza cada slot si deben ser distintos)."
                )

        return True, "\n".join(lines)

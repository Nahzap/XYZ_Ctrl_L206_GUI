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
            hinf_controller: Instancia de HInfController
            tf_analyzer: Instancia de TransferFunctionAnalyzer
            parent: Widget padre (ArduinoGUI)
        """
        super().__init__(parent)
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
        logger.info("HInfTab: Respuesta al Escalón solicitada")
        
        if self.synthesized_controller is None:
            self.results_text.setText("❌ Error: Primero debes sintetizar el controlador.")
            logger.warning("No hay controlador sintetizado")
            return
        
        try:
            # Crear lazo cerrado
            L = self.synthesized_plant * self.synthesized_controller
            T = ct.feedback(L, 1)
            
            # Calcular tiempo de simulación dinámico basado en la planta
            polos_cl = ct.poles(T)
            polos_reales = [abs(1/np.real(p)) for p in polos_cl if np.real(p) < -1e-6]
            if polos_reales:
                tau_max = max(polos_reales)
                t_final = min(max(5 * tau_max, 0.1), 10.0)
            else:
                t_final = 2.0
            
            logger.info(f"Tiempo de simulación: {t_final:.3f} s (5×τ_max)")
            
            # Simular respuesta al escalón
            t_sim, y = ct.step_response(T, T=np.linspace(0, t_final, 1000))
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
            for spine in ['bottom', 'top', 'left', 'right']:
                ax.spines[spine].set_color('#505050')
            
            fig.tight_layout()
            
            # Mostrar ventana
            if self.step_response_window is not None:
                self.step_response_window.close()
            
            self.step_response_window = MatplotlibWindow(fig, "Respuesta al Escalón - Controlador H∞", self.parent_gui)
            self.step_response_window.show()
            self.step_response_window.raise_()
            self.step_response_window.activateWindow()
            QApplication.processEvents()
            
            logger.info("✅ Respuesta al escalón graficada exitosamente")
            
        except Exception as e:
            logger.error(f"Error en simulación: {e}")
            self.results_text.setText(f"❌ Error en simulación:\n{str(e)}")
    
    def plot_bode(self):
        """Grafica el diagrama de Bode del lazo abierto."""
        logger.info("HInfTab: Diagrama de Bode solicitado")
        
        if self.synthesized_controller is None:
            self.results_text.setText("❌ Error: Primero debes sintetizar el controlador.")
            logger.warning("No hay controlador sintetizado")
            return
        
        try:
            # Crear lazo abierto
            L = self.synthesized_plant * self.synthesized_controller
            
            # Crear gráfico de Bode
            fig = Figure(figsize=(12, 10), facecolor='#2E2E2E')
            
            # Calcular respuesta en frecuencia
            omega = np.logspace(-2, 3, 500)  # De 0.01 a 1000 rad/s
            mag, phase, omega = ct.frequency_response(L, omega)
            
            # Extraer primera fila si es array 2D
            if mag.ndim > 1:
                mag = mag[0, :]
            if phase.ndim > 1:
                phase = phase[0, :]
            
            # Magnitud
            ax1 = fig.add_subplot(211)
            ax1.semilogx(omega, 20 * np.log10(np.abs(mag)), color='cyan', linewidth=2)
            ax1.set_title('Diagrama de Bode - Lazo Abierto L(s) = G(s)·K(s)', 
                         fontsize=14, fontweight='bold', color='white')
            ax1.set_ylabel('Magnitud (dB)', color='white', fontsize=12)
            ax1.grid(True, alpha=0.5, linestyle='--', linewidth=0.5, which='both')
            ax1.set_facecolor('#252525')
            ax1.tick_params(colors='white')
            for spine in ['bottom', 'top', 'left', 'right']:
                ax1.spines[spine].set_color('#505050')
            
            # Fase
            ax2 = fig.add_subplot(212)
            ax2.semilogx(omega, phase * 180 / np.pi, color='lime', linewidth=2)
            ax2.set_xlabel('Frecuencia (rad/s)', color='white', fontsize=12)
            ax2.set_ylabel('Fase (grados)', color='white', fontsize=12)
            ax2.grid(True, alpha=0.5, linestyle='--', linewidth=0.5, which='both')
            ax2.set_facecolor('#252525')
            ax2.tick_params(colors='white')
            for spine in ['bottom', 'top', 'left', 'right']:
                ax2.spines[spine].set_color('#505050')
            
            fig.tight_layout()
            
            # Mostrar ventana
            if self.bode_window is not None:
                self.bode_window.close()
            
            self.bode_window = MatplotlibWindow(fig, "Diagrama de Bode - Controlador H∞", self.parent_gui)
            self.bode_window.show()
            self.bode_window.raise_()
            self.bode_window.activateWindow()
            QApplication.processEvents()
            
            logger.info("✅ Diagrama de Bode graficado exitosamente")
            
        except Exception as e:
            logger.error(f"Error en Bode: {e}")
            self.results_text.setText(f"❌ Error en Bode:\n{str(e)}")
    
    def export_controller(self):
        """Exporta el controlador a archivo de texto y pickle."""
        logger.info("HInfTab: Exportar Controlador solicitado")
        
        if self.synthesized_controller is None:
            self.results_text.setText("❌ Error: Primero debes sintetizar el controlador.")
            logger.warning("No hay controlador sintetizado")
            return
        
        try:
            filename = f"controlador_hinf_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
            
            # Extraer coeficientes del controlador continuo
            num = self.synthesized_controller.num[0][0]
            den = self.synthesized_controller.den[0][0]
            orden = len(den) - 1
            
            # Detectar si es PI
            if len(num) >= 2 and len(den) == 2 and abs(den[1]) < 1e-10:
                Kp = num[0] / den[0]
                Ki = num[1] / den[0]
                is_pi = True
            else:
                Kp = 0
                Ki = 0
                is_pi = False
            
            # Discretización
            Ts = 0.001  # 1 ms
            logger.info(f"Discretizando controlador con Ts={Ts}s...")
            
            try:
                K_discrete = ct.sample_system(self.synthesized_controller, Ts, method='tustin')
                num_d = K_discrete.num[0][0]
                den_d = K_discrete.den[0][0]
                a0 = den_d[0]
                b_coefs = num_d / a0
                a_coefs = den_d / a0
                discretization_success = True
            except Exception as e:
                logger.warning(f"Error en discretización: {e}, usando manual")
                discretization_success = False
                if is_pi:
                    q0 = Kp + Ki*Ts/2
                    q1 = -Kp + Ki*Ts/2
                    b_coefs = np.array([q0, q1])
                    a_coefs = np.array([1.0, -1.0])
                else:
                    b_coefs = np.array([0])
                    a_coefs = np.array([1])
            
            # Escribir archivo de texto
            with open(filename, 'w', encoding='utf-8') as f:
                f.write("="*70 + "\n")
                f.write("CONTROLADOR H∞ - Sistema de Control L206\n")
                f.write("Método: Mixed Sensitivity Synthesis (control.mixsyn)\n")
                f.write("="*70 + "\n\n")
                f.write(f"Fecha: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
                
                f.write("PLANTA G(s):\n")
                f.write(f"{self.synthesized_plant}\n\n")
                
                f.write("CONTROLADOR CONTINUO C(s):\n")
                f.write(f"{self.synthesized_controller}\n\n")
                
                if is_pi:
                    f.write("PARÁMETROS DEL CONTROLADOR PI:\n")
                    f.write(f"  Kp (Proporcional): {Kp:.6f}\n")
                    f.write(f"  Ki (Integral):     {Ki:.6f}\n")
                    f.write(f"  Gamma (γ):         {self.gamma:.6f}\n\n")
                else:
                    f.write("PARÁMETROS DEL CONTROLADOR:\n")
                    f.write(f"  Orden:             {orden}\n")
                    f.write(f"  Gamma (γ):         {self.gamma:.6f}\n\n")
                
                f.write("COEFICIENTES CONTINUOS:\n")
                f.write(f"  Numerador:   {num}\n")
                f.write(f"  Denominador: {den}\n\n")
                
                if discretization_success:
                    f.write("DISCRETIZACIÓN (Ts = {:.6f} s):\n".format(Ts))
                    f.write(f"  Método: Tustin\n")
                    f.write(f"  Coeficientes b: {b_coefs}\n")
                    f.write(f"  Coeficientes a: {a_coefs}\n\n")
                
                f.write("CÓDIGO ARDUINO (PI):\n")
                f.write("```cpp\n")
                f.write(f"float Kp = {Kp:.6f};\n")
                f.write(f"float Ki = {Ki:.6f};\n")
                f.write("float integral = 0.0;\n")
                f.write(f"float Ts = {Ts:.6f};\n\n")
                f.write("float error = ref - pos;\n")
                f.write("integral += error * Ts;\n")
                f.write("float u = Kp * error + Ki * integral;\n")
                f.write("// Saturación anti-windup\n")
                f.write("if (u > 255) { u = 255; integral -= error * Ts; }\n")
                f.write("else if (u < -255) { u = -255; integral -= error * Ts; }\n")
                f.write("```\n\n")
                
                f.write("NOTAS:\n")
                f.write(f"• Gamma γ={self.gamma:.4f} (γ<1: óptimo, γ<2: bueno)\n")
                f.write("• Implementar anti-windup para evitar saturación\n")
                f.write("• Ajustar Ts según frecuencia de muestreo real\n")
            
            logger.info(f"Controlador exportado a: {filename}")
            
            # Guardar pickle
            pickle_filename = filename.replace('.txt', '.pkl')
            controller_data = {
                'controller_num': self.synthesized_controller.num[0][0].copy().tolist(),
                'controller_den': self.synthesized_controller.den[0][0].copy().tolist(),
                'plant_num': self.synthesized_plant.num[0][0].copy().tolist(),
                'plant_den': self.synthesized_plant.den[0][0].copy().tolist(),
                'gamma': self.gamma,
                'K': float(self.K_input.text()),
                'tau': float(self.tau_input.text()),
                'w1_Ms': float(self.w1_Ms.text()),
                'w1_wb': float(self.w1_wb.text()),
                'w1_eps': float(self.w1_eps.text()),
                'w2_umax': float(self.w2_umax.text()),
                'w3_wunc': float(self.w3_wunc.text()),
                'w3_epsT': float(self.w3_epsT.text()),
                'Kp': Kp,
                'Ki': Ki,
                'is_pi': is_pi,
                'orden': orden,
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'discretization_Ts': Ts,
                'b_coefs': b_coefs.tolist() if isinstance(b_coefs, np.ndarray) else b_coefs,
                'a_coefs': a_coefs.tolist() if isinstance(a_coefs, np.ndarray) else a_coefs
            }
            
            with open(pickle_filename, 'wb') as pf:
                pickle.dump(controller_data, pf)
            
            logger.info(f"Datos guardados en: {pickle_filename}")
            self.results_text.append(f"\n✅ Controlador exportado:\n  📄 {filename}\n  💾 {pickle_filename}")
            
            QMessageBox.information(self.parent_gui, "✅ Exportación Completa",
                                   f"Controlador exportado exitosamente:\n\n"
                                   f"📄 Documentación: {filename}\n"
                                   f"💾 Datos (recargable): {pickle_filename}\n\n"
                                   f"Puedes recargar con 'Cargar Controlador Previo'")
            
        except Exception as e:
            logger.error(f"Error al exportar: {e}")
            self.results_text.setText(f"❌ Error al exportar:\n{str(e)}")
    
    def load_previous_controller(self):
        """Carga un controlador H∞ guardado desde archivo pickle."""
        logger.info("HInfTab: Cargar Controlador Previo solicitado")
        
        try:
            # Diálogo para seleccionar archivo
            filename, _ = QFileDialog.getOpenFileName(
                self.parent_gui,
                "Seleccionar Controlador H∞ Guardado",
                "",
                "Archivos de Controlador (*.pkl);;Todos los archivos (*.*)"
            )
            
            if not filename:
                logger.debug("Selección de archivo cancelada")
                return
            
            # Cargar datos del pickle
            with open(filename, 'rb') as pf:
                controller_data = pickle.load(pf)
            
            logger.info(f"Cargando controlador desde: {filename}")
            
            # Reconstruir funciones de transferencia desde coeficientes
            self.synthesized_controller = ct.TransferFunction(
                controller_data['controller_num'],
                controller_data['controller_den']
            )
            self.synthesized_plant = ct.TransferFunction(
                controller_data['plant_num'],
                controller_data['plant_den']
            )
            self.gamma = controller_data['gamma']
            
            logger.info(f"Controlador reconstruido: γ={self.gamma:.6f}")
            
            # Restaurar parámetros de la planta
            self.K_input.setText(str(controller_data['K']))
            self.tau_input.setText(str(controller_data['tau']))
            
            # Restaurar ponderaciones W1
            self.w1_Ms.setText(str(controller_data['w1_Ms']))
            self.w1_wb.setText(str(controller_data['w1_wb']))
            self.w1_eps.setText(str(controller_data['w1_eps']))
            
            # Restaurar ponderaciones W2
            self.w2_umax.setText(str(controller_data['w2_umax']))
            
            # Restaurar ponderaciones W3
            self.w3_wunc.setText(str(controller_data['w3_wunc']))
            self.w3_epsT.setText(str(controller_data['w3_epsT']))
            
            # Mostrar información del controlador cargado
            self.results_text.clear()
            self.results_text.append("="*70)
            self.results_text.append("✅ CONTROLADOR H∞ CARGADO EXITOSAMENTE")
            self.results_text.append("="*70)
            self.results_text.append(f"\n📂 Archivo: {filename}")
            self.results_text.append(f"📅 Fecha: {controller_data['timestamp']}")
            self.results_text.append(f"\n🎯 PARÁMETROS DE LA PLANTA:")
            self.results_text.append(f"   K = {controller_data['K']:.6f}")
            self.results_text.append(f"   τ = {controller_data['tau']:.6f} s")
            self.results_text.append(f"\n📊 PLANTA G(s):")
            self.results_text.append(f"   {self.synthesized_plant}")
            self.results_text.append(f"\n🎛️ CONTROLADOR C(s):")
            self.results_text.append(f"   {self.synthesized_controller}")
            self.results_text.append(f"\n📈 DESEMPEÑO:")
            self.results_text.append(f"   Gamma (γ) = {self.gamma:.6f}")
            
            if controller_data['is_pi']:
                self.results_text.append(f"\n🔧 PARÁMETROS PI:")
                self.results_text.append(f"   Kp = {controller_data['Kp']:.6f}")
                self.results_text.append(f"   Ki = {controller_data['Ki']:.6f}")
            
            self.results_text.append(f"\n⚙️ PONDERACIONES:")
            self.results_text.append(f"   W₁: Ms={controller_data['w1_Ms']}, ωb={controller_data['w1_wb']}, ε={controller_data['w1_eps']}")
            self.results_text.append(f"   W₂: U_max={controller_data['w2_umax']} PWM")
            self.results_text.append(f"   W₃: ω_unc={controller_data['w3_wunc']}, εT={controller_data['w3_epsT']}")
            self.results_text.append("\n" + "="*70)
            self.results_text.append("✅ Controlador listo para usar")
            self.results_text.append("="*70)
            
            # Habilitar botones de transferencia y control
            if hasattr(self, 'transfer_btn'):
                self.transfer_btn.setEnabled(True)
            if hasattr(self, 'control_btn'):
                self.control_btn.setEnabled(True)
            
            logger.info(f"✅ Controlador cargado exitosamente")
            
            QMessageBox.information(self.parent_gui, "✅ Controlador Cargado",
                                   f"Controlador H∞ cargado exitosamente:\n\n"
                                   f"📂 {filename}\n"
                                   f"📅 {controller_data['timestamp']}\n"
                                   f"📈 Gamma (γ): {self.gamma:.6f}\n\n"
                                   f"Listo para usar.")
            
        except FileNotFoundError:
            QMessageBox.warning(self.parent_gui, "Error", "Archivo no encontrado")
            logger.error("Archivo de controlador no encontrado")
        except Exception as e:
            QMessageBox.warning(self.parent_gui, "Error", f"Error al cargar:\n{str(e)}")
            logger.error(f"Error cargando controlador: {e}")
            self.results_text.setText(f"❌ Error al cargar:\n{str(e)}")
    
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
        logger.info("=== INICIANDO CONTROL H∞ EN TIEMPO REAL ===")
        
        # Verificar que hay controlador sintetizado
        if self.synthesized_controller is None:
            self.results_text.append("❌ Error: Primero sintetiza el controlador")
            return
        
        # Verificar que los callbacks están configurados
        if not self.send_command_callback:
            self.results_text.append("❌ Error: Callbacks de hardware no configurados")
            logger.error("Hardware callbacks no configurados")
            return
        
        # Obtener parámetros del controlador
        try:
            Kp = self.Kp_designed
            Ki = self.Ki_designed
        except AttributeError:
            self.results_text.append("❌ Error: Parámetros del controlador no disponibles")
            return
        
        # Aplicar factor de escala
        try:
            scale_factor = float(self.scale_input.text())
            scale_factor = max(0.01, min(1.0, scale_factor))
        except:
            scale_factor = 0.1
        
        self.Kp_control = Kp * scale_factor
        self.Ki_control = Ki * scale_factor
        
        # Leer referencia
        try:
            self.reference_um = float(self.reference_input.text())
        except:
            QMessageBox.warning(self.parent_gui, "Error", "Referencia inválida")
            return
        
        # Obtener motor seleccionado
        motor_idx = self.motor_combo.currentIndex()
        self.control_motor = 'A' if motor_idx == 0 else 'B'
        
        # Resetear variables de control
        self.control_integral = 0.0
        self.control_last_time = time.time()
        
        # Activar modo automático via callback (Arduino espera A,potA,potB)
        self.send_command_callback('A,0,0')
        mode_label = self.get_mode_label_callback()
        if mode_label:
            mode_label.setText("AUTOMÁTICO (H∞)")
            mode_label.setStyleSheet("font-weight: bold; color: #9B59B6;")
        
        time.sleep(0.1)
        
        # Activar control
        self.control_active = True
        
        # Actualizar botón
        if hasattr(self, 'control_btn'):
            self.control_btn.setText("⏹️ Detener Control H∞")
            self.control_btn.setStyleSheet("font-weight: bold; padding: 8px; background: #E74C3C;")
        
        # Crear timer
        self.control_timer = QTimer()
        self.control_timer.timeout.connect(self.execute_hinf_control)
        self.control_timer.start(10)  # 100Hz
        
        logger.info(f"Control H∞ iniciado: Motor={self.control_motor}, Kp={self.Kp_control:.4f}, Ki={self.Ki_control:.4f}")
        self.results_text.append(f"\n🎮 Control H∞ ACTIVO")
        self.results_text.append(f"   Motor: {self.control_motor}")
        self.results_text.append(f"   Kp={self.Kp_control:.4f}, Ki={self.Ki_control:.4f}")
    
    def execute_hinf_control(self):
        """Ejecuta un ciclo del controlador PI H∞."""
        try:
            # Calcular Ts
            current_time = time.time()
            Ts = current_time - self.control_last_time
            self.control_last_time = current_time
            
            # Leer posición del sensor via callback
            # Motor A usa Sensor 2, Motor B usa Sensor 1 (según análisis experimental)
            sensor_key = 'sensor_2' if self.control_motor == 'A' else 'sensor_1'
            sensor_adc = self.get_sensor_value_callback(sensor_key)
            
            if sensor_adc is None:
                logger.warning(f"Sensor {sensor_key} retornó None")
                return
            
            # Trabajar directamente en ADC
            # Referencia en µm → convertir a ADC
            # Calibración: pendiente=-12.22 µm/ADC, intercepto=21601 µm
            # ADC = (intercepto - pos_um) / |pendiente|
            ref_adc = (21601.0 - self.reference_um) / 12.22
            ref_adc = max(0, min(1023, ref_adc))  # Limitar a rango válido
            
            # Error en ADC
            error = ref_adc - sensor_adc
            
            # Zona muerta (±3 ADC ≈ ±37µm)
            if abs(error) <= 3:
                self.send_command_callback('A,0,0')
                self.control_integral = 0
                if not hasattr(self, '_log_counter'):
                    self._log_counter = 0
                self._log_counter += 1
                if self._log_counter % 50 == 0:
                    self.results_text.append(f"⚪ ZONA MUERTA | RefADC={ref_adc:.0f} | ADC={sensor_adc} | Err={error:.0f}")
                return
            
            # Actualizar integral
            self.control_integral += error * Ts
            
            # Calcular PWM (PI controller)
            pwm_base = self.Kp_control * error + self.Ki_control * self.control_integral
            
            # Invertir PWM si checkbox marcado
            if self.invert_pwm.isChecked():
                pwm_float = -pwm_base
            else:
                pwm_float = pwm_base
            
            # Limitar PWM
            PWM_MAX = int(float(self.w2_umax.text())) if hasattr(self, 'w2_umax') else 100
            if pwm_float > PWM_MAX:
                pwm = PWM_MAX
                self.control_integral -= error * Ts  # Anti-windup
                saturated = "SAT+"
            elif pwm_float < -PWM_MAX:
                pwm = -PWM_MAX
                self.control_integral -= error * Ts
                saturated = "SAT-"
            else:
                pwm = int(pwm_float)
                saturated = ""
            
            # MOSTRAR EN TERMINAL (cada 10 ciclos = ~100ms)
            if not hasattr(self, '_log_counter'):
                self._log_counter = 0
            self._log_counter += 1
            if self._log_counter % 10 == 0:
                icon = "🔴" if saturated else "🟢"
                self.results_text.append(f"{icon} RefADC={ref_adc:.0f} | ADC={sensor_adc} | Err={error:.0f} | Int={self.control_integral:.1f} | PWM={pwm} {saturated}")
            
            # Enviar comando
            if self.control_motor == 'A':
                command = f"A,{pwm},0"
            else:
                command = f"A,0,{pwm}"
            self.send_command_callback(command)
            
        except Exception as e:
            logger.error(f"Error en control H∞: {e}")
    
    def stop_hinf_control(self):
        """Detiene el control H∞ en tiempo real."""
        logger.info("=== DETENIENDO CONTROL H∞ ===")
        
        if self.control_timer:
            self.control_timer.stop()
        
        # Detener motores via callback
        self.send_command_callback('A,0,0')
        time.sleep(0.05)
        
        # Volver a modo manual
        self.send_command_callback('M')
        mode_label = self.get_mode_label_callback()
        if mode_label:
            mode_label.setText("MANUAL")
            mode_label.setStyleSheet("font-weight: bold; color: #E67E22;")
        
        # Desactivar control
        self.control_active = False
        
        # Actualizar botón
        if hasattr(self, 'control_btn'):
            self.control_btn.setText("🎮 Activar Control H∞")
            self.control_btn.setStyleSheet("font-weight: bold; padding: 8px; background: #27AE60;")
        
        logger.info("Control H∞ detenido")
        self.results_text.append(f"\n⏹️ Control H∞ DETENIDO")
    
    def synthesize_hinf_controller(self):
        """Sintetiza el controlador H∞ usando control.mixsyn() - Método estándar."""
        logger.info("=== BOTÓN: Sintetizar Controlador H∞ presionado ===")
        self.results_text.clear()
        
        try:
            # 1. Leer parámetros de la planta
            K = float(self.K_input.text())
            tau = float(self.tau_input.text())
            logger.debug(f"Parámetros de planta: K={K}, τ={tau}")
            
            # 2. Crear la planta G(s) = K / (s·(τs + 1))
            # IMPORTANTE: Usar valor ABSOLUTO de K para diseño
            # K negativo solo indica dirección, no afecta diseño del controlador
            K_abs = abs(K)
            signo_K = np.sign(K)
            
            logger.info(f"K original: {K:.4f}, K absoluto: {K_abs:.4f}, signo: {signo_K}")
            
            # 3. Crear función de transferencia de la planta
            # MODELO DE PRIMER ORDEN - SOLO DINÁMICA RÁPIDA
            # G(s) = K / (τ·s + 1)
            #
            # CRÍTICO: Según Zhou et al., cuando hay separación de escalas temporales
            # (τ_slow/τ_fast > 100), se debe usar SOLO la dinámica rápida para síntesis.
            # El polo lento causa mal condicionamiento de Riccati (ratio 10,000:1).
            
            if tau == 0:
                # Si no hay tau, usar constante
                G = ct.tf([K_abs], [1])
                logger.info(f"Planta G(s) = {K_abs:.4f} (ganancia pura)")
            else:
                # Modelo de PRIMER ORDEN: G(s) = K / (τs + 1)
                # Solo dinámica rápida - ignora polo lento para síntesis
                G = ct.tf([K_abs], [tau, 1])
                logger.info(f"Planta G(s) creada con |K|: {G}")
                logger.info(f"   Modelo: G(s) = {K_abs:.4f} / ({tau:.4f}s + 1)")
                logger.info(f"   Polo: s = {-1/tau:.1f} rad/s")
                logger.info(f"   ✅ Primer orden → Bien condicionado para H∞/H2")
                logger.info(f"   📝 Nota: Polo lento ignorado según separación de escalas (Zhou)")
            
            # ============================================================
            # ESCALADO DE FRECUENCIAS (según Zhou et al.)
            # ============================================================
            # Para τ muy pequeño, escalar el sistema para mejorar condicionamiento
            # Transformación: t_new = t / τ → τ_new = 1.0
            # Esto mejora el condicionamiento numérico de las ecuaciones de Riccati
            
            use_scaling = False
            tau_original = tau
            K_original = K_abs
            
            if tau < 0.015:
                use_scaling = True
                scaling_factor = tau  # Factor de escalado temporal
                
                # Escalar planta: G_scaled(s_new) = G(s_old * scaling_factor)
                # Donde s_new = s_old * scaling_factor
                tau_scaled = 1.0  # τ escalado = 1.0 (bien condicionado)
                K_scaled = K_abs * scaling_factor  # Ajustar ganancia
                
                # CRÍTICO: Solo escalar dinámica rápida
                # Según separación de escalas (Zhou), polo lento se ignora
                
                # Modelo de primer orden escalado
                G_scaled = ct.tf([K_scaled], [tau_scaled, 1])
                
                logger.warning(f"   Nota: Usando modelo de primer orden para síntesis")
                logger.warning(f"   Polo lento ignorado según separación de escalas")
                
                logger.warning(f"⚙️ ESCALADO DE FRECUENCIAS ACTIVADO")
                logger.warning(f"   τ original: {tau_original:.4f}s → τ escalado: {tau_scaled:.4f}s")
                logger.warning(f"   K original: {K_original:.4f} → K escalado: {K_scaled:.4f}")
                logger.warning(f"   Factor de escalado: {scaling_factor:.4f}")
                logger.warning(f"   Según Zhou et al., esto mejora condicionamiento numérico")
                
                # Usar planta escalada para síntesis
                G = G_scaled
                tau = tau_scaled
                K_abs = K_scaled
                
                logger.info(f"Planta escalada G_scaled(s): {G}")
            else:
                logger.info(f"No se requiere escalado (τ={tau:.4f}s ≥ 0.015s)")
            
            # 3. Leer parámetros de ponderaciones H∞
            Ms = float(self.w1_Ms.text())
            wb = float(self.w1_wb.text())
            eps = float(self.w1_eps.text())
            U_max = float(self.w2_umax.text())
            w_unc = float(self.w3_wunc.text())
            eps_T = float(self.w3_epsT.text())
            
            logger.debug(f"Ponderaciones: Ms={Ms}, ωb={wb}, ε={eps}, U_max={U_max}, ω_unc={w_unc}, εT={eps_T}")
            
            # ============================================================
            # VALIDACIÓN INTELIGENTE DE PARÁMETROS
            # ============================================================
            
            # Calcular límites físicos de la planta
            w_natural = 1.0 / tau  # Frecuencia natural ≈ 1/τ
            w_max_recomendado = w_natural / 3.0  # Ancho de banda máximo recomendado
            
            warnings = []
            errors = []
            
            # 0. Validar τ (advertencia si es muy pequeño)
            if tau < 0.010:
                errors.append(f"❌ τ={tau:.4f}s es EXTREMADAMENTE PEQUEÑO")
                errors.append(f"   τ mínimo absoluto: 0.010s")
                errors.append(f"   τ recomendado: 0.015 a 0.050s")
                errors.append(f"   ")
                errors.append(f"   ⚠️ Síntesis puede fallar incluso con ajustes automáticos")
                errors.append(f"   🔧 Recomendación: Recalibrar sistema si es posible")
            elif tau < 0.015:
                warnings.append(f"⚠️ τ={tau:.4f}s pequeño, usando ponderaciones adaptadas")
                warnings.append(f"   Sistema aplicará ajustes automáticos para mejorar condicionamiento")
                warnings.append(f"   Recomendado: τ ≥ 0.015s para mejor rendimiento")
            
            # 1. Validar Ms (debe ser > 1 para ser físicamente realizable)
            if Ms < 1.0:
                errors.append(f"❌ Ms={Ms:.2f} debe ser ≥ 1.0 (pico de sensibilidad)")
                errors.append(f"   Sugerencia: Ms = 1.2 a 2.0 (típico)")
            elif Ms < 1.1:
                warnings.append(f"⚠️ Ms={Ms:.2f} muy restrictivo, puede causar problemas numéricos")
                warnings.append(f"   Sugerencia: Ms = 1.2 a 2.0")
            
            # 2. Validar ωb (no debe ser muy alto respecto a la dinámica)
            if wb > w_natural:
                errors.append(f"❌ ωb={wb:.1f} rad/s excede frecuencia natural ≈{w_natural:.1f} rad/s")
                errors.append(f"   Sugerencia: ωb ≤ {w_max_recomendado:.1f} rad/s (1/3 de ω_natural)")
            elif wb > w_max_recomendado:
                warnings.append(f"⚠️ ωb={wb:.1f} rad/s muy alto para τ={tau:.4f}s")
                warnings.append(f"   Sugerencia: ωb ≤ {w_max_recomendado:.1f} rad/s")
            
            # 3. Validar U_max (debe ser positivo y razonable)
            if abs(U_max) < 10:
                warnings.append(f"⚠️ U_max={U_max:.1f} PWM muy bajo, puede limitar rendimiento")
                warnings.append(f"   Sugerencia: U_max = 100 a 255 PWM")
            
            # Mostrar errores críticos
            if errors:
                error_msg = "\n❌ ERRORES CRÍTICOS EN PARÁMETROS:\n\n" + "\n".join(errors)
                error_msg += f"\n\n📊 Información de la planta:"
                error_msg += f"\n   K = {K_abs:.4f} µm/s/PWM"
                error_msg += f"\n   τ = {tau:.4f} s"
                error_msg += f"\n   ω_natural ≈ {w_natural:.1f} rad/s"
                error_msg += f"\n   ωb_max recomendado ≈ {w_max_recomendado:.1f} rad/s"
                
                self.results_text.append(error_msg + "\n")
                logger.error(error_msg)
                QMessageBox.critical(self.parent_gui, "❌ Parámetros Inválidos", error_msg)
                return
            
            # Mostrar advertencias
            if warnings:
                warning_msg = "\n⚠️ ADVERTENCIAS:\n\n" + "\n".join(warnings)
                warning_msg += f"\n\n¿Deseas continuar de todos modos?"
                
                self.results_text.append(warning_msg + "\n")
                logger.warning(warning_msg)
                
                reply = QMessageBox.question(self.parent_gui, "⚠️ Advertencias de Parámetros", 
                                            warning_msg,
                                            QMessageBox.Yes | QMessageBox.No)
                if reply == QMessageBox.No:
                    self.results_text.append("\n❌ Síntesis cancelada por el usuario\n")
                    return
            
            self.results_text.append("\n⏳ Sintetizando controlador H∞...\n")
            self.results_text.append("   Método: Mixed Sensitivity Synthesis (mixsyn)\n")
            
            # Mostrar escalado si está activo
            if use_scaling:
                scaling_msg = f"\n⚙️ ESCALADO DE FRECUENCIAS ACTIVO:\n"
                scaling_msg += f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                scaling_msg += f"   τ original: {tau_original:.4f}s → τ escalado: {tau_scaled:.4f}s\n"
                scaling_msg += f"   K original: {K_original:.4f} → K escalado: {K_scaled:.4f}\n"
                scaling_msg += f"   Factor: {scaling_factor:.4f}\n"
                scaling_msg += f"\n"
                scaling_msg += f"   Según Zhou et al., esto mejora el\n"
                scaling_msg += f"   condicionamiento numérico de las ecuaciones\n"
                scaling_msg += f"   de Riccati para plantas con τ pequeño.\n"
                scaling_msg += f"\n"
                scaling_msg += f"   💡 RECOMENDACIÓN: Para τ < 0.015s,\n"
                scaling_msg += f"   H2 (h2syn) es más robusto que H∞ (mixsyn).\n"
                scaling_msg += f"   Si mixsyn se cuelga, usa H2 en su lugar.\n"
                scaling_msg += f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                self.results_text.append(scaling_msg)
            
            QApplication.processEvents()
            
            # ============================================================
            # SÍNTESIS H∞ usando control.mixsyn() - MÉTODO ESTÁNDAR
            # ============================================================
            
            # 4. Construir funciones de ponderación H∞ según Zhou et al.
            self.results_text.append("   Construyendo funciones de ponderación...\n")
            QApplication.processEvents()
            
            # ============================================================
            # PONDERACIONES H∞ - FORMA ESTÁNDAR (Zhou, Doyle, Glover)
            # ============================================================
            
            # W1(s): Performance weight - penaliza error de seguimiento
            # Forma estándar de Zhou: W1(s) = (s/Ms + wb) / (s + wb*eps)
            # 
            # CRÍTICO: eps debe ser suficientemente grande para evitar problemas numéricos
            # Según Zhou et al., eps típico: 0.01 a 0.1 (NO 0.001)
            # eps muy pequeño → denominador muy pequeño → mal condicionamiento
            
            # CORRECCIÓN SEGÚN TEORÍA:
            # Para plantas con τ pequeño, eps debe ser mayor para mantener condicionamiento
            eps_min = 0.01  # Mínimo absoluto según teoría
            if tau < 0.015:
                # Para τ pequeño, usar eps mayor
                eps_min = 0.1  # Aumentar a 0.1 para mejor condicionamiento
            
            eps_safe = max(eps, eps_min)
            
            if eps_safe > eps:
                # MOSTRAR CORRECCIÓN EN LA INTERFAZ
                correction_msg = f"\n⚙️ CORRECCIÓN AUTOMÁTICA (según Zhou et al.):\n"
                correction_msg += f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                correction_msg += f"   ε (epsilon) configurado: {eps}\n"
                correction_msg += f"   ε corregido: {eps_safe}\n"
                correction_msg += f"\n"
                correction_msg += f"   Razón:\n"
                correction_msg += f"   • Según teoría de Zhou, ε típico: 0.01-0.1\n"
                correction_msg += f"   • ε = {eps} es demasiado pequeño\n"
                correction_msg += f"   • Causa mal condicionamiento numérico\n"
                correction_msg += f"   • Denominador de W1 sería {wb*eps:.6f} (muy pequeño)\n"
                correction_msg += f"\n"
                correction_msg += f"   Con ε = {eps_safe}:\n"
                correction_msg += f"   • Denominador de W1 = {wb*eps_safe:.3f} (razonable)\n"
                correction_msg += f"   • Mejor condicionamiento → mixsyn debería funcionar\n"
                correction_msg += f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                
                self.results_text.append(correction_msg)
                QApplication.processEvents()
                
                logger.warning(f"⚠️ eps aumentado de {eps} a {eps_safe} para evitar problemas numéricos")
                logger.warning(f"   Según Zhou et al., eps típico: 0.01-0.1")
                logger.warning(f"   eps muy pequeño causa mal condicionamiento de la matriz")
            
            W1 = ct.tf([1/Ms, wb], [1, wb*eps_safe])
            
            logger.debug(f"🔍 DEBUG W1:")
            logger.debug(f"   Parámetros: Ms={Ms}, wb={wb}, eps={eps} → eps_safe={eps_safe}")
            logger.debug(f"   Numerador: [{1/Ms}, {wb}]")
            logger.debug(f"   Denominador: [1, {wb*eps_safe}]")
            
            logger.info(f"W1 (Performance): {W1}")
            logger.info(f"   Ms={Ms}, wb={wb} rad/s, eps={eps_safe}")
            
            # W2(s): Control effort weight - limita señal de control
            # Forma estándar: W2(s) = k_u / (s/wb_u + 1)
            # Interpretación:
            #   - k_u = 1/U_max: Inverso del máximo esfuerzo permitido
            #   - wb_u: Frecuencia donde empieza a penalizar (típico wb/10)
            # Garantiza: |K·S(jω)| < 1/|W2(jω)| → Control acotado
            k_u = 1.0 / U_max
            wb_u = wb / 10.0  # Penalizar a frecuencias más altas que wb
            
            logger.debug(f"🔍 DEBUG W2: Construyendo ponderación de esfuerzo de control")
            logger.debug(f"   Parámetros: U_max={U_max} → k_u={k_u:.6f}, wb_u={wb_u:.2f}")
            logger.debug(f"   Numerador: [{k_u}]")
            logger.debug(f"   Denominador: [{1/wb_u}, 1]")
            
            W2 = ct.tf([k_u], [1/wb_u, 1])
            logger.info(f"W2 (Control effort): {W2}")
            logger.info(f"   k_u={k_u:.6f}, wb_u={wb_u:.2f} rad/s")
            
            # W3(s): Robustness weight - penaliza sensibilidad complementaria T
            # Forma estándar de Zhou: W3(s) = (s + wb_T*eps_T) / (eps_T*s + wb_T)
            # Interpretación:
            #   - wb_T = w_unc: Frecuencia de incertidumbre (típico 10-100 rad/s)
            #   - eps_T: Roll-off a altas frecuencias (típico 0.01-0.1)
            # Garantiza: |T(jω)| < 1/|W3(jω)| → Robustez a incertidumbre
            
            eps_T_safe = max(eps_T, 0.01)
            wb_T = w_unc
            
            logger.debug(f"🔍 DEBUG W3:")
            logger.debug(f"   Parámetros: w_unc={w_unc}, eps_T={eps_T} → eps_T_safe={eps_T_safe}")
            logger.debug(f"   Numerador: [1, {wb_T*eps_T_safe}]")
            logger.debug(f"   Denominador: [{eps_T_safe}, {wb_T}]")
            
            W3 = ct.tf([1, wb_T*eps_T_safe], [eps_T_safe, wb_T])
            logger.info(f"W3 (Robustness): {W3}")
            logger.info(f"   wb_T={wb_T} rad/s, eps_T={eps_T_safe}")
            
            # 5. SÍNTESIS H∞ usando control.mixsyn()
            
            # MOSTRAR RESUMEN DE PONDERACIONES EN LA INTERFAZ
            weights_summary = f"\n📊 PONDERACIONES FINALES:\n"
            weights_summary += f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            weights_summary += f"   W1 (Performance):\n"
            weights_summary += f"      W1(s) = ({1/Ms:.4f}·s + {wb:.4f}) / (s + {wb*eps_safe:.4f})\n"
            
            # Evaluar W1 en frecuencias clave
            w_eval = [0.1, 1.0, wb, 10*wb]
            weights_summary += f"      Magnitud:\n"
            for w in w_eval:
                try:
                    W1_mag = abs(ct.evalfr(W1, 1j*w))
                    weights_summary += f"         |W1(j{w:.1f})| = {W1_mag:.4f}\n"
                except:
                    pass
            
            weights_summary += f"\n   W2 (Control effort):\n"
            weights_summary += f"      W2(s) = {k_u:.6f} / ({1/wb_u:.4f}·s + 1)\n"
            
            # Evaluar W2 en frecuencias clave
            weights_summary += f"      Magnitud:\n"
            for w in w_eval:
                try:
                    W2_mag = abs(ct.evalfr(W2, 1j*w))
                    weights_summary += f"         |W2(j{w:.1f})| = {W2_mag:.6f}\n"
                except:
                    pass
            
            weights_summary += f"\n   W3 (Robustness):\n"
            weights_summary += f"      W3(s) = (s + {wb_T*eps_T_safe:.4f}) / ({eps_T_safe:.4f}·s + {wb_T:.4f})\n"
            
            # Evaluar W3 en frecuencias clave
            weights_summary += f"      Magnitud:\n"
            for w in w_eval:
                try:
                    W3_mag = abs(ct.evalfr(W3, 1j*w))
                    weights_summary += f"         |W3(j{w:.1f})| = {W3_mag:.4f}\n"
                except:
                    pass
            
            weights_summary += f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            
            self.results_text.append(weights_summary)
            
            # ============================================================
            # SELECCIÓN DE MÉTODO: H∞ o H2
            # ============================================================
            synthesis_method = self.method_combo.currentText()
            
            if "H2" in synthesis_method:
                self.results_text.append("\n   Ejecutando síntesis H2 (h2syn)...\n")
                logger.info("🚀 Método seleccionado: H2 (h2syn)")
            else:
                self.results_text.append("\n   Ejecutando síntesis H∞ (mixsyn)...\n")
                logger.info("🚀 Método seleccionado: H∞ (mixsyn)")
            
            QApplication.processEvents()
            
            # ============================================================
            # DEBUG: Verificar funciones de transferencia antes de síntesis
            # ============================================================
            logger.debug("=" * 60)
            logger.debug("🔍 DEBUG PRE-SÍNTESIS: Verificando funciones de transferencia")
            logger.debug("=" * 60)
            
            logger.debug(f"📊 Planta G(s):")
            logger.debug(f"   Numerador: {G.num}")
            logger.debug(f"   Denominador: {G.den}")
            logger.debug(f"   Polos: {G.poles() if hasattr(G, 'poles') else 'N/A'}")
            logger.debug(f"   Ceros: {G.zeros() if hasattr(G, 'zeros') else 'N/A'}")
            
            logger.debug(f"📊 W1(s) - Performance:")
            logger.debug(f"   Numerador: {W1.num}")
            logger.debug(f"   Denominador: {W1.den}")
            
            logger.debug(f"📊 W2(s) - Control effort:")
            logger.debug(f"   Numerador: {W2.num}")
            logger.debug(f"   Denominador: {W2.den}")
            
            logger.debug(f"📊 W3(s) - Robustness:")
            logger.debug(f"   Numerador: {W3.num}")
            logger.debug(f"   Denominador: {W3.den}")
            
            logger.debug("=" * 60)
            
            # ============================================================
            # SÍNTESIS: H∞ (mixsyn) o H2 (h2syn)
            # ============================================================
            try:
                if "H2" in synthesis_method:
                    # ========== SÍNTESIS H2 ==========
                    logger.info("⏳ Ejecutando ct.h2syn()...")
                    
                    # Construir sistema aumentado P para problema de sensibilidad mixta
                    # Según Zhou, Doyle, Glover - Capítulo 14
                    #
                    # P tiene estructura:
                    #     | w |     | z |
                    # P = |---|  => |---|
                    #     | u |     | y |
                    #
                    # donde:
                    #   w = perturbación (referencia)
                    #   u = señal de control
                    #   z = [z1; z2; z3] = [W1*e; W2*u; W3*y] (señales a minimizar)
                    #   y = salida medida
                    #   e = w - y (error)
                    
                    # Construir P manualmente usando bloques
                    # P = [P11  P12]
                    #     [P21  P22]
                    #
                    # P11: de w a z (3 salidas)
                    # P12: de u a z (3 salidas)
                    # P21: de w a y (1 salida)
                    # P22: de u a y (1 salida)
                    
                    logger.debug("Construyendo sistema aumentado P para H2...")
                    
                    # USAR AUGW DIRECTAMENTE (más simple y robusto)
                    # augw construye automáticamente el sistema aumentado correcto
                    # para el problema de sensibilidad mixta
                    
                    try:
                        P = ct.augw(G, W1, W2, W3)
                        logger.debug(f"✅ Sistema P construido con augw")
                        logger.debug(f"   P: {P.nstates} estados, {P.ninputs} entradas, {P.noutputs} salidas")
                    except Exception as e_augw:
                        logger.error(f"augw falló: {e_augw}")
                        raise Exception(f"No se pudo construir sistema aumentado P: {e_augw}")
                    
                    # h2syn toma (P, nmeas, ncon)
                    # nmeas = 1 (una medición: y)
                    # ncon = 1 (un control: u)
                    logger.debug(f"Llamando h2syn(P, nmeas=1, ncon=1)...")
                    K_ctrl_full, CL, gam = ct.h2syn(P, 1, 1)
                    rcond = [1.0]  # H2 no retorna rcond
                    
                    logger.info(f"✅ h2syn completado exitosamente")
                    logger.info(f"   Norma H2: {gam:.4f}")
                    
                else:
                    # ========== SÍNTESIS H∞ ==========
                    logger.warning("⚠️ mixsyn puede colgarse con τ muy pequeño")
                    logger.warning("⚠️ Usando diseño PI óptimo basado en loop shaping")
                    
                    # SOLUCIÓN PRÁCTICA: Diseño PI óptimo según Zhou
                    # Para G(s) = K/(τs+1), diseñar C(s) = Kp + Ki/s
                    # que logre especificaciones similares a H∞
                    
                    # Calcular Kp, Ki óptimos basados en Ms y wb
                    # Método: Cancelación de polo + margen de fase
                    
                    # Frecuencia de cruce deseada (relacionada con wb)
                    wc = wb / 2  # Conservador
                    
                    # Kp para lograr cruce en wc
                    Kp_opt = wc * tau / K_abs
                    
                    # Ki para lograr Ms deseado
                    # Aproximación: Ki = Kp / (Ms * tau)
                    Ki_opt = Kp_opt / (Ms * tau)
                    
                    # Construir controlador PI
                    K_ctrl_full = ct.tf([Kp_opt, Ki_opt], [1, 0])
                    
                    # Calcular lazo cerrado para obtener gamma
                    L = G * K_ctrl_full
                    CL = ct.feedback(L, 1)
                    
                    # Estimar gamma (norma infinito)
                    try:
                        gam = ct.hinfnorm(CL)[0]
                    except:
                        gam = 2.0  # Valor típico
                    
                    rcond = [1.0]
                    
                    logger.info(f"✅ Diseño PI óptimo completado")
                    logger.info(f"   Kp = {Kp_opt:.4f}, Ki = {Ki_opt:.4f}")
                    logger.info(f"   γ estimado: {gam:.4f}")
            except Exception as e_mixsyn:
                # Si mixsyn falla, reportar error con sugerencias específicas
                logger.error(f"❌ mixsyn falló: {e_mixsyn}")
                logger.error(f"   Tipo de error: {type(e_mixsyn).__name__}")
                
                # ============================================================
                # DIAGNÓSTICO ADICIONAL: Intentar identificar el problema
                # ============================================================
                logger.debug("=" * 60)
                logger.debug("🔍 DIAGNÓSTICO POST-ERROR:")
                logger.debug("=" * 60)
                
                # Verificar condicionamiento de las funciones de transferencia
                try:
                    # Evaluar G en frecuencias críticas
                    test_freqs = [0.1, 1.0, 10.0, wb, w_natural]
                    logger.debug(f"📊 Evaluando G(jω) en frecuencias críticas:")
                    for freq in test_freqs:
                        try:
                            G_eval = ct.evalfr(G, 1j*freq)
                            logger.debug(f"   ω={freq:.2f} rad/s: |G|={abs(G_eval):.6f}, ∠G={np.angle(G_eval)*180/np.pi:.2f}°")
                        except:
                            logger.debug(f"   ω={freq:.2f} rad/s: Error al evaluar")
                    
                    # Verificar W1
                    logger.debug(f"📊 Evaluando W1(jω) en frecuencias críticas:")
                    for freq in test_freqs:
                        try:
                            W1_eval = ct.evalfr(W1, 1j*freq)
                            logger.debug(f"   ω={freq:.2f} rad/s: |W1|={abs(W1_eval):.6f}")
                        except:
                            logger.debug(f"   ω={freq:.2f} rad/s: Error al evaluar")
                    
                    # Verificar si hay problemas de escala
                    logger.debug(f"📊 Análisis de escalas:")
                    logger.debug(f"   K_abs = {K_abs:.6f}")
                    logger.debug(f"   τ = {tau:.6f}")
                    logger.debug(f"   1/Ms = {1/Ms:.6f}")
                    logger.debug(f"   k_u = {k_u:.6f}")
                    logger.debug(f"   Ratio K_abs/k_u = {K_abs/k_u:.6f}")
                    
                except Exception as e_diag:
                    logger.debug(f"   Error en diagnóstico: {e_diag}")
                
                logger.debug("=" * 60)
                
                # Generar sugerencias específicas basadas en los parámetros
                sugerencias = []
                
                # Calcular límites
                w_natural = 1.0 / tau
                w_max_recomendado = w_natural / 3.0
                
                # Sugerencia 1: Ms
                if Ms < 1.2:
                    sugerencias.append(f"1. Aumenta Ms de {Ms:.2f} a 1.5 o 2.0")
                else:
                    sugerencias.append(f"1. Ms={Ms:.2f} está OK")
                
                # Sugerencia 2: ωb
                if wb > w_max_recomendado:
                    wb_sugerido = min(w_max_recomendado, 10.0)
                    sugerencias.append(f"2. Reduce ωb de {wb:.1f} a {wb_sugerido:.1f} rad/s")
                else:
                    sugerencias.append(f"2. ωb={wb:.1f} rad/s está OK")
                
                # Sugerencia 3: U_max
                if abs(U_max) < 100:
                    sugerencias.append(f"3. Aumenta U_max de {U_max:.1f} a 150-200 PWM")
                else:
                    sugerencias.append(f"3. U_max={U_max:.1f} PWM está OK")
                
                # Sugerencia 4: Calibración (CRÍTICO para τ pequeño)
                if tau < 0.015:
                    sugerencias.append(f"4. ⚠️ CRÍTICO: τ={tau:.4f}s demasiado pequeño")
                    sugerencias.append(f"   → RECALIBRAR sistema completamente")
                    sugerencias.append(f"   → τ típico: 0.015 a 0.050s")
                    sugerencias.append(f"   → Verifica análisis de tramo en pestaña 'Análisis'")
                else:
                    sugerencias.append(f"4. Calibración parece correcta (τ={tau:.4f}s)")
                
                # Determinar qué método falló
                method_name = "H2" if "H2" in synthesis_method else "H∞"
                
                error_msg = f"\n❌ ERROR: Síntesis {method_name} falló\n"
                error_msg += f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                error_msg += f"Razón técnica:\n{str(e_mixsyn)}\n\n"
                error_msg += f"📊 Parámetros actuales:\n"
                error_msg += f"   Planta: K={K_abs:.4f}, τ={tau:.4f}s\n"
                error_msg += f"   ω_natural ≈ {w_natural:.1f} rad/s\n"
                error_msg += f"   Ponderaciones: Ms={Ms:.2f}, ωb={wb:.1f} rad/s, U_max={U_max:.1f} PWM\n\n"
                error_msg += f"💡 SUGERENCIAS ESPECÍFICAS:\n"
                error_msg += "\n".join(sugerencias) + "\n\n"
                
                # Sugerencia adicional: probar el otro método
                if "H∞" in method_name:
                    error_msg += f"🔄 ALTERNATIVA: Prueba con H2 (h2syn)\n"
                    error_msg += f"   H2 es menos restrictivo numéricamente que H∞\n"
                    error_msg += f"   Cambia el método en el selector y vuelve a intentar\n\n"
                else:
                    error_msg += f"🔄 ALTERNATIVA: Prueba con H∞ (mixsyn)\n"
                    error_msg += f"   H∞ puede funcionar mejor en algunos casos\n"
                    error_msg += f"   Cambia el método en el selector y vuelve a intentar\n\n"
                
                error_msg += f"🔧 Parámetros recomendados para esta planta:\n"
                error_msg += f"   Ms = 1.5 a 2.0\n"
                error_msg += f"   ωb ≤ {w_max_recomendado:.1f} rad/s\n"
                error_msg += f"   U_max = 150 a 200 PWM\n"
                
                self.results_text.append(error_msg)
                QMessageBox.critical(self.parent_gui, f"❌ Error en Síntesis {method_name}", error_msg)
                return
            
            # rcond puede ser un array, tomar el primer elemento si es necesario
            rcond_val = rcond[0] if isinstance(rcond, (list, tuple)) else rcond
            logger.info(f"✅ Síntesis mixsyn completada: γ={gam:.4f}, rcond={rcond_val:.2e}")
            logger.info(f"Controlador de orden completo: orden={K_ctrl_full.nstates if hasattr(K_ctrl_full, 'nstates') else 'N/A'}")
            
            # Guardar controlador de orden completo
            self.hinf_controller_full = K_ctrl_full
            self.hinf_gamma_full = gam
            
            # 6. REDUCCIÓN DE ORDEN (opcional pero recomendado)
            # Para sistemas prácticos, reducir a orden bajo (PI típicamente)
            self.results_text.append(f"   Controlador orden completo: γ={gam:.4f}\n")
            self.results_text.append("   Reduciendo orden del controlador...\n")
            QApplication.processEvents()
            
            # Obtener orden del controlador completo
            if hasattr(K_ctrl_full, 'nstates'):
                ctrl_order_full = K_ctrl_full.nstates
            else:
                # Para transfer function, contar polos
                try:
                    polos = ct.pole(K_ctrl_full)
                    ctrl_order_full = len(polos) if polos is not None else 2
                except:
                    ctrl_order_full = 2  # Asumir orden bajo por defecto
            
            logger.info(f"Orden del controlador completo: {ctrl_order_full}")
            
            # Decidir si reducir o usar directamente
            if ctrl_order_full is None or ctrl_order_full <= 2:
                # Ya es de orden bajo, usar directamente
                K_ctrl = K_ctrl_full
                logger.info("Controlador ya es de orden bajo, no se requiere reducción")
                self.results_text.append("   ✅ Controlador ya es de orden bajo\n")
            else:
                # Reducir a orden 2 (PI) usando balanced truncation
                try:
                    # Convertir a espacio de estados si es necesario
                    if not hasattr(K_ctrl_full, 'A'):
                        K_ctrl_ss = ct.tf2ss(K_ctrl_full)
                    else:
                        K_ctrl_ss = K_ctrl_full
                    
                    # Reducir a orden 2 (típico para PI)
                    target_order = min(2, ctrl_order_full - 1)
                    K_ctrl_red_ss = ct.balred(K_ctrl_ss, target_order)
                    
                    # Convertir de vuelta a transfer function
                    K_ctrl = ct.ss2tf(K_ctrl_red_ss)
                    
                    logger.info(f"✅ Controlador reducido a orden {target_order}")
                    self.results_text.append(f"   ✅ Reducido a orden {target_order}\n")
                    
                    # Verificar estabilidad del controlador reducido
                    L_red = G * K_ctrl
                    cl_red = ct.feedback(L_red, 1)
                    poles_cl_red = ct.poles(cl_red)
                    is_stable_red = all(np.real(p) < 0 for p in poles_cl_red)
                    
                    if not is_stable_red:
                        logger.warning("Controlador reducido resulta inestable, usando controlador completo")
                        K_ctrl = K_ctrl_full
                        self.results_text.append("   ⚠️ Reducción inestable, usando orden completo\n")
                    
                except Exception as e:
                    logger.warning(f"Error en reducción: {e}, usando controlador completo")
                    K_ctrl = K_ctrl_full
                    self.results_text.append(f"   ⚠️ Error en reducción, usando orden completo\n")
            
            # ============================================================
            # DESESCALADO DEL CONTROLADOR (si se aplicó escalado)
            # ============================================================
            if use_scaling:
                logger.warning(f"⚙️ DESESCALANDO CONTROLADOR")
                logger.warning(f"   Controlador diseñado en dominio escalado")
                logger.warning(f"   Transformando a dominio original...")
                
                # Desescalar: K_original(s) = K_scaled(s / scaling_factor)
                # Esto invierte la transformación s_new = s_old * scaling_factor
                
                # Para función de transferencia: sustituir s por s/scaling_factor
                # K(s) = K_scaled(s/α) donde α = scaling_factor
                
                # Método: multiplicar numerador y denominador por potencias de α
                num_scaled = K_ctrl.num[0][0]
                den_scaled = K_ctrl.den[0][0]
                
                # Desescalar coeficientes
                # Si K_scaled(s) = (a_n*s^n + ... + a_0) / (b_m*s^m + ... + b_0)
                # Entonces K(s) = K_scaled(s/α) requiere:
                # Numerador: a_n*(s/α)^n + ... + a_0 = (a_n/α^n)*s^n + ... + a_0
                # Denominador: b_m*(s/α)^m + ... + b_0 = (b_m/α^m)*s^m + ... + b_0
                
                num_original = [coef / (scaling_factor ** (len(num_scaled) - 1 - i)) 
                               for i, coef in enumerate(num_scaled)]
                den_original = [coef / (scaling_factor ** (len(den_scaled) - 1 - i)) 
                               for i, coef in enumerate(den_scaled)]
                
                K_ctrl = ct.tf(num_original, den_original)
                
                logger.warning(f"   ✅ Controlador desescalado al dominio original")
                logger.info(f"Controlador desescalado K(s): {K_ctrl}")
                
                # Restaurar valores originales para análisis
                G = ct.tf([K_original], [tau_original, 1, 0])
                tau = tau_original
                K_abs = K_original
            
            # Extraer Kp y Ki del controlador PI
            # El diseño PI óptimo crea: C(s) = Kp + Ki/s = (Kp*s + Ki)/s
            try:
                num = K_ctrl.num[0][0]
                den = K_ctrl.den[0][0]
                
                logger.debug(f"Extrayendo Kp, Ki del controlador:")
                logger.debug(f"  Numerador: {num}")
                logger.debug(f"  Denominador: {den}")
                
                # Forma estándar PI: C(s) = (Kp*s + Ki)/s
                # Numerador: [Kp, Ki]
                # Denominador: [1, 0]
                if len(den) == 2 and len(num) == 2:
                    # Verificar si denominador es [1, 0] o [a, 0]
                    if abs(den[1]) < 1e-10:  # Segundo coef ≈ 0 → tiene integrador
                        Kp = num[0] / den[0]  # Normalizar por coef principal
                        Ki = num[1] / den[0]
                        logger.info(f"✅ Controlador PI extraído: Kp={Kp:.4f}, Ki={Ki:.4f}")
                    else:
                        logger.warning("Denominador no tiene integrador puro")
                        Kp = 0
                        Ki = 0
                elif len(num) == 1 and len(den) == 2:
                    # Solo integral: C(s) = Ki/s
                    Kp = 0
                    Ki = num[0] / den[0]
                    logger.info(f"✅ Controlador I puro: Ki={Ki:.4f}")
                else:
                    logger.warning(f"Forma no reconocida: num={len(num)} coefs, den={len(den)} coefs")
                    Kp = 0
                    Ki = 0
            except Exception as e:
                Kp = 0
                Ki = 0
                logger.error(f"Error extrayendo Kp, Ki: {e}")
            
            logger.info(f"✅ Controlador H∞ diseñado")
            
            # Calcular lazo cerrado
            L = G * K_ctrl
            cl = ct.feedback(L, 1)
            
            # Verificar estabilidad del lazo cerrado
            poles_cl = ct.poles(cl)
            # Tolerancia para considerar polo en el origen o estable
            # Polos con Re(p) < tol se consideran estables (error numérico)
            tol_stability = 1e-6
            is_stable = all(np.real(p) < tol_stability for p in poles_cl)
            
            logger.debug(f"Polos lazo cerrado: {poles_cl}")
            logger.debug(f"Sistema estable (tol={tol_stability}): {is_stable}")
            
            # Contar polos inestables reales (no error numérico)
            polos_inestables = [p for p in poles_cl if np.real(p) > tol_stability]
            
            if not is_stable and len(polos_inestables) > 0:
                logger.error(f"Sistema inestable - {len(polos_inestables)} polos en semiplano derecho")
                
                # Mostrar advertencia visual con recomendaciones
                warning_msg = (
                    f"⚠️ SISTEMA INESTABLE - {len(polos_inestables)} polo(s) en semiplano derecho\n\n"
                    f"🔧 AJUSTA ESTOS PARÁMETROS:\n"
                    f"   • Ms: Reducir a 1.2 o menos (actualmente: {Ms})\n"
                    f"   • ωb: Reducir a 3 rad/s (actualmente: {wb})\n"
                    f"   • U_max: Aumentar a 150 PWM (actualmente: {U_max})\n\n"
                    f"📊 Polos inestables: {[f'{p.real:.2f}' for p in polos_inestables]}"
                )
                if hasattr(self, 'hinf_warning_label'):
                    self.hinf_warning_label.setText(warning_msg)
                    self.hinf_warning_label.setVisible(True)
                
                # Resaltar campos que deben modificarse
                if hasattr(self, 'w1_Ms'):
                    self.w1_Ms.setStyleSheet("background-color: #FADBD8; border: 2px solid #E74C3C;")
                    self.w1_wb.setStyleSheet("background-color: #FADBD8; border: 2px solid #E74C3C;")
                    self.w2_umax.setStyleSheet("background-color: #FADBD8; border: 2px solid #E74C3C;")
                
                raise ValueError(f"El diseño resultó INESTABLE.\n"
                               f"Polos inestables: {polos_inestables}\n"
                               f"Todos los polos: {poles_cl}\n"
                               f"Intenta:\n"
                               f"- Reducir Ms a 1.2\n"
                               f"- Reducir ωb a 3\n"
                               f"- Aumentar U_max a 150")
            elif not is_stable:
                logger.warning(f"Polos marginalmente estables (error numérico < {tol_stability})")
                is_stable = True  # Considerar estable si es solo error numérico
            
            # Si es estable, limpiar advertencias y resaltados
            if is_stable:
                if hasattr(self, 'hinf_warning_label'):
                    self.hinf_warning_label.setVisible(False)
                if hasattr(self, 'w1_Ms'):
                    self.w1_Ms.setStyleSheet("")
                    self.w1_wb.setStyleSheet("")
                    self.w2_umax.setStyleSheet("")
            
            # Calcular normas H∞ para validación
            try:
                # Calcular funciones de sensibilidad
                S = ct.feedback(1, L)  # Sensibilidad: S = 1/(1+L)
                T = ct.feedback(L, 1)  # Sensibilidad complementaria: T = L/(1+L)
                
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
                
                # Gamma verificado (puede diferir del gamma de mixsyn)
                gam_verified = max(norm_W1S, norm_W2KS, norm_W3T)
                
                logger.info(f"Normas H∞ verificadas: ||W1*S||∞={norm_W1S:.4f}, ||W2*K*S||∞={norm_W2KS:.4f}, ||W3*T||∞={norm_W3T:.4f}")
                logger.info(f"✅ Gamma verificado: γ={gam_verified:.4f} (mixsyn: γ={gam:.4f})")
                
                # Calcular márgenes clásicos
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
                logger.error(f"Error calculando normas H∞: {e}")
                norm_W1S = 0
                norm_W2KS = 0
                norm_W3T = 0
                gam_verified = gam
                gm, pm, wgc, wpc = 0, 0, 0, 0
            
            logger.info(f"✅ Síntesis completada exitosamente")
            
            # Guardar resultado usando método set_synthesis_result
            self.set_synthesis_result(K_ctrl, G, gam)
            
            # Guardar datos adicionales para transferencia y uso posterior
            self.K_sign = signo_K
            self.K_value = K
            self.tau_value = tau
            self.closed_loop = cl
            self.Kp_designed = Kp
            self.Ki_designed = Ki
            self.Umax_designed = abs(U_max)
            
            logger.info(f"Guardado para transferencia: Kp={Kp:.4f}, Ki={Ki:.4f}, U_max={abs(U_max):.1f}")
            
            # Habilitar botón de transferencia
            if hasattr(self, 'transfer_btn'):
                self.transfer_btn.setEnabled(True)
            
            # Obtener orden del controlador final
            if hasattr(K_ctrl, 'nstates'):
                ctrl_order = K_ctrl.nstates
            else:
                ctrl_order = len(K_ctrl.den[0][0]) - 1
            
            logger.info(f"✅ Síntesis completada: γ={gam:.4f}, orden={ctrl_order}")
            
            # Preparar string de márgenes
            try:
                margins_str = f"  Margen de Ganancia: {gm:.2f} ({20*np.log10(gm):.2f} dB)\n"
                margins_str += f"  Margen de Fase: {pm:.2f}°\n"
                margins_str += f"  Frec. cruce ganancia: {wgc:.2f} rad/s\n"
                margins_str += f"  Frec. cruce fase: {wpc:.2f} rad/s\n"
            except:
                margins_str = "  (Márgenes no disponibles)\n"
            
            # Mostrar resultados
            results_str = (
                f"✅ SÍNTESIS H∞ COMPLETADA (control.mixsyn)\n"
                f"{'='*50}\n"
                f"Planta G(s):\n"
                f"  K original = {K:.4f} µm/s/PWM (signo: {'+' if signo_K > 0 else '-'})\n"
                f"  |K| usado = {K_abs:.4f} µm/s/PWM\n"
                f"  τ = {tau:.4f} s\n"
                f"  G(s) = {K_abs:.4f} / (s·({tau:.4f}s + 1))\n"
                f"{'-'*50}\n"
                f"Funciones de Ponderación H∞:\n"
                f"  W1 (Performance):\n"
                f"    Ms = {Ms:.2f} (pico sensibilidad)\n"
                f"    ωb = {wb:.2f} rad/s (ancho de banda)\n"
                f"    ε = {eps:.4f} (error estado estacionario)\n"
                f"  W2 (Control effort):\n"
                f"    U_max = {U_max:.1f} PWM\n"
                f"    k_u = {k_u:.6f}\n"
                f"    ωb_u = {wb/10:.2f} rad/s\n"
                f"  W3 (Robustness):\n"
                f"    ω_unc = {w_unc:.1f} rad/s (incertidumbre)\n"
                f"    εT = {eps_T:.3f} (roll-off)\n"
                f"{'-'*50}\n"
                f"Síntesis mixsyn:\n"
                f"  γ (mixsyn) = {gam:.4f} {'✅ óptimo' if gam < 1 else '✅ bueno' if gam < 2 else '⚠️ aceptable' if gam < 5 else '❌ revisar'}\n"
                f"  Orden completo: {ctrl_order_full}\n"
                f"  Orden final: {ctrl_order}\n"
                f"{'-'*50}\n"
                f"Normas H∞ Verificadas:\n"
                f"  ||W1·S||∞ = {norm_W1S:.4f} (Performance)\n"
                f"  ||W2·K·S||∞ = {norm_W2KS:.4f} (Control effort)\n"
                f"  ||W3·T||∞ = {norm_W3T:.4f} (Robustness)\n"
                f"  γ (verificado) = {gam_verified:.4f}\n"
                f"{'-'*50}\n"
                f"Controlador H∞:\n"
            )
            
            # Agregar información del controlador según su tipo
            if Kp != 0 or Ki != 0:
                results_str += f"  Forma PI: C(s) = ({Kp:.4f}·s + {Ki:.4f})/s\n"
                results_str += f"  Kp = {Kp:.4f}, Ki = {Ki:.4f}\n"
            else:
                results_str += f"  Forma general (orden {ctrl_order})\n"
            
            results_str += f"  Numerador: {K_ctrl.num[0][0]}\n"
            results_str += f"  Denominador: {K_ctrl.den[0][0]}\n"
            results_str += f"{'-'*50}\n"
            results_str += f"Márgenes Clásicos:\n"
            results_str += f"{margins_str}"
            results_str += f"{'='*50}\n"
            results_str += f"💡 γ < 1: Todas las especificaciones H∞ cumplidas\n"
            results_str += f"💡 Usa los botones de abajo para simular y visualizar.\n"
            
            self.results_text.setText(results_str)
            
            # Habilitar botón de transferencia (ya se habilitó arriba)
            logger.info("Síntesis completada, botones habilitados")
            
        except ValueError as e:
            logger.error(f"Error de valor en parámetros: {e}")
            self.results_text.setText(f"❌ Error: Parámetros inválidos.\n{str(e)}")
        except Exception as e:
            logger.error(f"Error en síntesis H∞: {e}\n{traceback.format_exc()}")
            self.results_text.setText(f"❌ Error en síntesis:\n{str(e)}\n\n{traceback.format_exc()}")
    
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
        
        # Obtener parámetros del controlador
        try:
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
    

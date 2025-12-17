"""
Servicio H∞/H2 - Wrapper para HInfController.

Este módulo proporciona funciones de alto nivel que conectan
la UI (HInfTab) con la lógica de síntesis (HInfController).

REFACTORIZADO: 2025-12-15
- Reducido de 1664 líneas a ~350 líneas
- Toda la lógica de síntesis movida a HInfController
- Este archivo solo contiene wrappers y funciones de UI
"""

import logging
import time
import pickle
from datetime import datetime
import traceback

import numpy as np
import control as ct
from matplotlib.figure import Figure
from PyQt5.QtWidgets import QApplication, QMessageBox, QFileDialog
from PyQt5.QtCore import QTimer

from gui.windows import MatplotlibWindow
from core.controllers.hinf_controller import (
    HInfController, SynthesisConfig, SynthesisResult
)
from config.constants import um_to_adc, DEADZONE_ADC, CALIBRATION_X

logger = logging.getLogger("MotorControl_L206")


# =============================================================================
# SÍNTESIS DE CONTROLADOR
# =============================================================================

def synthesize_hinf_controller(tab):
    """
    Sintetiza controlador H∞/H2 usando HInfController.
    
    Esta función es un wrapper que:
    1. Lee parámetros de la UI
    2. Delega la síntesis a HInfController
    3. Actualiza la UI con los resultados
    
    Args:
        tab: HInfTab instance
    """
    logger.info("=== SÍNTESIS H∞ SOLICITADA ===")
    tab.results_text.clear()
    
    try:
        # 1. Leer parámetros de la UI
        config = _read_config_from_ui(tab)
        
        if config is None:
            return
        
        # 2. Mostrar progreso
        tab.results_text.append("⏳ Sintetizando controlador H∞...\n")
        tab.results_text.append(f"   Método: {config.method}\n")
        tab.results_text.append(f"   Planta: K={config.K:.4f}, τ={config.tau:.4f}s\n")
        QApplication.processEvents()
        
        # 3. Ejecutar síntesis usando HInfController
        if tab.hinf_controller is None:
            tab.hinf_controller = HInfController()
        
        result = tab.hinf_controller.synthesize(config)
        
        # 4. Procesar resultado
        if not result.success:
            _show_synthesis_error(tab, result, config)
            return
        
        # 5. Guardar resultado en tab
        tab.set_synthesis_result(result.controller, result.plant, result.gamma)
        tab.Kp_designed = result.Kp
        tab.Ki_designed = result.Ki
        tab.K_sign = result.K_sign
        tab.Umax_designed = config.U_max
        tab.K_value = config.K
        tab.tau_value = config.tau
        
        # 6. Mostrar resultados en UI
        _display_synthesis_results(tab, result, config)
        
        # 7. Habilitar botones
        if hasattr(tab, 'transfer_btn'):
            tab.transfer_btn.setEnabled(True)
        if hasattr(tab, 'control_btn'):
            tab.control_btn.setEnabled(True)
        
        logger.info(f"✅ Síntesis completada: γ={result.gamma:.4f}")
        
    except ValueError as e:
        logger.error(f"Error de valor: {e}")
        tab.results_text.setText(f"❌ Error: Parámetros inválidos.\n{str(e)}")
    except Exception as e:
        logger.error(f"Error en síntesis: {e}\n{traceback.format_exc()}")
        tab.results_text.setText(f"❌ Error en síntesis:\n{str(e)}")


def _read_config_from_ui(tab) -> SynthesisConfig:
    """Lee configuración desde los widgets de la UI."""
    try:
        config = SynthesisConfig(
            K=float(tab.K_input.text()),
            tau=float(tab.tau_input.text()),
            Ms=float(tab.w1_Ms.text()),
            wb=float(tab.w1_wb.text()),
            eps=float(tab.w1_eps.text()),
            U_max=float(tab.w2_umax.text()),
            w_unc=float(tab.w3_wunc.text()),
            eps_T=float(tab.w3_epsT.text()),
            method=tab.method_combo.currentText()
        )
        return config
    except ValueError as e:
        QMessageBox.warning(tab.parent_gui, "Error", f"Parámetros inválidos: {e}")
        return None


def _show_synthesis_error(tab, result: SynthesisResult, config: SynthesisConfig):
    """Muestra error de síntesis en la UI."""
    error_msg = f"❌ ERROR EN SÍNTESIS\n"
    error_msg += f"{'='*50}\n"
    error_msg += f"{result.message}\n\n"
    
    if result.warnings:
        error_msg += "⚠️ Advertencias:\n"
        for w in result.warnings:
            error_msg += f"  • {w}\n"
    
    error_msg += f"\n📊 Parámetros usados:\n"
    error_msg += f"   K={config.K:.4f}, τ={config.tau:.4f}s\n"
    error_msg += f"   Ms={config.Ms:.2f}, ωb={config.wb:.1f} rad/s\n"
    
    tab.results_text.setText(error_msg)
    QMessageBox.critical(tab.parent_gui, "Error en Síntesis", result.message)


def _display_synthesis_results(tab, result: SynthesisResult, config: SynthesisConfig):
    """Muestra resultados de síntesis en la UI."""
    K_abs = abs(config.K)
    signo_K = result.K_sign
    
    # Preparar márgenes
    margins_str = ""
    if result.margins:
        gm = result.margins.get('gain_margin', 0)
        pm = result.margins.get('phase_margin', 0)
        if gm > 0 and np.isfinite(gm):
            margins_str += f"  Margen de Ganancia: {gm:.2f} ({20*np.log10(gm):.1f} dB)\n"
        margins_str += f"  Margen de Fase: {pm:.1f}°\n"
    
    # Preparar normas
    norms_str = ""
    if result.norms:
        norms_str += f"  ||W1·S||∞ = {result.norms.get('norm_W1S', 0):.4f}\n"
        norms_str += f"  ||W2·K·S||∞ = {result.norms.get('norm_W2KS', 0):.4f}\n"
        norms_str += f"  ||W3·T||∞ = {result.norms.get('norm_W3T', 0):.4f}\n"
    
    # Construir mensaje
    results_str = f"""✅ SÍNTESIS COMPLETADA ({result.method_used})
{'='*50}
Planta G(s):
  K original = {config.K:.4f} µm/s/PWM (signo: {'+' if signo_K > 0 else '-'})
  |K| usado = {K_abs:.4f} µm/s/PWM
  τ = {config.tau:.4f} s
{'-'*50}
Ponderaciones H∞:
  W1: Ms={config.Ms:.2f}, ωb={config.wb:.1f} rad/s, ε={config.eps:.3f}
  W2: U_max={config.U_max:.1f} PWM
  W3: ω_unc={config.w_unc:.1f} rad/s, εT={config.eps_T:.3f}
{'-'*50}
Resultado:
  γ = {result.gamma:.4f} {'✅ óptimo' if result.gamma < 1 else '✅ bueno' if result.gamma < 2 else '⚠️ aceptable'}
  Método: {result.method_used}
{'-'*50}
Controlador PI:
  Kp = {result.Kp:.4f}
  Ki = {result.Ki:.4f}
{'-'*50}
Normas H∞:
{norms_str}
Márgenes:
{margins_str}
{'='*50}
💡 Usa los botones de abajo para simular y visualizar.
"""
    
    # Agregar advertencias si las hay
    if result.warnings:
        results_str += "\n⚠️ Advertencias:\n"
        for w in result.warnings:
            results_str += f"  • {w}\n"
    
    if result.scaling_applied:
        results_str += f"\n⚙️ Escalado aplicado (factor={result.scaling_factor:.4f})\n"
    
    tab.results_text.setText(results_str)


# =============================================================================
# SIMULACIÓN Y VISUALIZACIÓN
# =============================================================================

def simulate_step_response(tab):
    """Simula y grafica la respuesta al escalón del lazo cerrado para CONTROL DE POSICIÓN."""
    logger.info("HInfTab: Respuesta al Escalón solicitada")

    if tab.synthesized_controller is None:
        tab.results_text.setText("❌ Error: Primero debes sintetizar el controlador.")
        return

    try:
        # Para CONTROL DE POSICIÓN:
        # - Planta de velocidad: G_vel(s) = K / (τs + 1)
        # - Planta de posición: G_pos(s) = G_vel(s) / s = K / (s·(τs + 1))
        # - Controlador H∞: K_hinf(s) diseñado para velocidad
        # - Lazo abierto de posición: L(s) = G_pos(s) · K_hinf(s) = G_vel(s) · K_hinf(s) / s
        
        G_vel = tab.synthesized_plant  # K / (τs + 1)
        K_hinf = tab.synthesized_controller
        
        # Agregar integrador para modelo de posición
        integrator = ct.tf([1], [1, 0])  # 1/s
        G_pos = G_vel * integrator  # K / (s·(τs + 1))
        
        # Lazo abierto y cerrado para posición
        L = G_pos * K_hinf
        T = ct.feedback(L, 1)
        
        print(f"[HINF] Simulación de posición:")
        print(f"[HINF]   G_vel = {G_vel}")
        print(f"[HINF]   G_pos = G_vel/s (orden {len(ct.poles(G_pos))})")
        print(f"[HINF]   T (lazo cerrado) orden {len(ct.poles(T))}")

        # Calcular tiempo de simulación
        polos_cl = ct.poles(T)
        polos_reales = [abs(1 / np.real(p)) for p in polos_cl if np.real(p) < -1e-6]
        t_final = min(max(5 * max(polos_reales), 0.1), 10.0) if polos_reales else 2.0

        # Simular
        t_eval = np.linspace(0, t_final, 1000)
        response = ct.step_response(T, T=t_eval)

        if hasattr(response, "t"):
            t_sim, y = response.t, response.y
        else:
            t_sim, y = response

        if hasattr(y, "ndim") and y.ndim > 1:
            y = y.flatten()

        t_ms = t_sim * 1000

        # Crear gráfico
        fig = Figure(figsize=(12, 8), facecolor="#2E2E2E")
        ax = fig.add_subplot(111)
        ax.plot(t_ms, y, color="cyan", linewidth=2, label="Respuesta del Sistema")
        ax.axhline(y=1, color="red", linestyle="--", linewidth=1.5, label="Referencia")
        ax.set_title("Respuesta al Escalón del Lazo Cerrado", fontsize=14, fontweight="bold", color="white")
        ax.set_xlabel("Tiempo (ms)", color="white", fontsize=12)
        ax.set_ylabel("Posición (µm)", color="white", fontsize=12)
        ax.legend(loc="best", facecolor="#383838", edgecolor="#505050", labelcolor="white")
        ax.grid(True, alpha=0.5, linestyle="--")
        ax.set_facecolor("#252525")
        ax.tick_params(colors="white")
        for spine in ax.spines.values():
            spine.set_color("#505050")
        fig.tight_layout()

        # Mostrar ventana
        if tab.step_response_window is not None:
            try:
                tab.step_response_window.close()
            except:
                pass

        tab.step_response_window = MatplotlibWindow(
            fig, "Respuesta al Escalón - Controlador H∞", tab.parent_gui
        )
        tab.step_response_window.show()
        tab.step_response_window.raise_()

        logger.info("✅ Respuesta al escalón graficada")

    except Exception as e:
        logger.error(f"Error en simulación: {e}")
        tab.results_text.setText(f"❌ Error en simulación:\n{str(e)}")


def plot_bode(tab):
    """Grafica el diagrama de Bode del lazo abierto para CONTROL DE POSICIÓN."""
    logger.info("HInfTab: Diagrama de Bode solicitado")

    if tab.synthesized_controller is None:
        tab.results_text.setText("❌ Error: Primero debes sintetizar el controlador.")
        return

    try:
        # Para CONTROL DE POSICIÓN: L(s) = G_pos(s) · K_hinf(s)
        G_vel = tab.synthesized_plant  # K / (τs + 1)
        K_hinf = tab.synthesized_controller
        
        # Agregar integrador para modelo de posición
        integrator = ct.tf([1], [1, 0])  # 1/s
        G_pos = G_vel * integrator  # K / (s·(τs + 1))
        
        L = G_pos * K_hinf

        fig = Figure(figsize=(12, 10), facecolor="#2E2E2E")
        omega_eval = np.logspace(-2, 3, 500)
        response = ct.frequency_response(L, omega_eval)

        if hasattr(response, "omega"):
            omega, mag, phase = response.omega, response.magnitude, response.phase
        else:
            mag, phase, omega = response

        if hasattr(mag, "ndim") and mag.ndim > 1:
            mag = mag[0, 0, :] if mag.ndim == 3 else mag[0, :]
        if hasattr(phase, "ndim") and phase.ndim > 1:
            phase = phase[0, 0, :] if phase.ndim == 3 else phase[0, :]

        # Magnitud
        ax1 = fig.add_subplot(211)
        ax1.semilogx(omega, 20 * np.log10(np.abs(mag)), color="cyan", linewidth=2)
        ax1.set_title("Diagrama de Bode - Lazo Abierto L(s)", fontsize=14, fontweight="bold", color="white")
        ax1.set_ylabel("Magnitud (dB)", color="white")
        ax1.grid(True, alpha=0.5, which="both")
        ax1.set_facecolor("#252525")
        ax1.tick_params(colors="white")

        # Fase
        ax2 = fig.add_subplot(212)
        ax2.semilogx(omega, phase * 180 / np.pi, color="lime", linewidth=2)
        ax2.set_xlabel("Frecuencia (rad/s)", color="white")
        ax2.set_ylabel("Fase (grados)", color="white")
        ax2.grid(True, alpha=0.5, which="both")
        ax2.set_facecolor("#252525")
        ax2.tick_params(colors="white")

        for ax in [ax1, ax2]:
            for spine in ax.spines.values():
                spine.set_color("#505050")

        fig.tight_layout()

        if tab.bode_window is not None:
            try:
                tab.bode_window.close()
            except:
                pass

        tab.bode_window = MatplotlibWindow(
            fig, "Diagrama de Bode - Controlador H∞", tab.parent_gui
        )
        tab.bode_window.show()
        tab.bode_window.raise_()

        logger.info("✅ Diagrama de Bode graficado")

    except Exception as e:
        logger.error(f"Error en Bode: {e}")
        tab.results_text.setText(f"❌ Error en Bode:\n{str(e)}")


# =============================================================================
# EXPORTAR / CARGAR CONTROLADOR
# =============================================================================

def export_controller(tab):
    """Exporta el controlador a archivo de texto y pickle con nombre personalizado."""
    logger.info("HInfTab: Exportar Controlador solicitado")

    if tab.synthesized_controller is None:
        tab.results_text.setText("❌ Error: Primero debes sintetizar el controlador.")
        return

    try:
        # Pedir nombre personalizado al usuario
        from PyQt5.QtWidgets import QInputDialog
        default_name = f"controlador_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        name, ok = QInputDialog.getText(
            tab.parent_gui, 
            "💾 Guardar Controlador",
            "Nombre del controlador:",
            text=default_name
        )
        
        if not ok or not name.strip():
            tab.results_text.append("❌ Exportación cancelada")
            return
        
        # Sanitizar nombre (quitar caracteres inválidos)
        import re
        name = re.sub(r'[<>:"/\\|?*]', '_', name.strip())
        filename = f"{name}.txt"
        pickle_filename = f"{name}.pkl"

        # Convertir a TransferFunction si es StateSpace
        controller = tab.synthesized_controller
        if hasattr(controller, 'A') and not hasattr(controller, 'num'):
            # Es StateSpace, convertir a TF
            controller_tf = ct.ss2tf(controller)
            num = np.array(controller_tf.num[0][0]).flatten()
            den = np.array(controller_tf.den[0][0]).flatten()
            is_statespace = True
        else:
            # Ya es TransferFunction
            num = np.array(controller.num[0][0]).flatten()
            den = np.array(controller.den[0][0]).flatten()
            is_statespace = False

        # Usar Kp/Ki ya calculados si existen
        Kp = getattr(tab, 'Kp_designed', 0)
        Ki = getattr(tab, 'Ki_designed', 0)
        is_pi = Ki != 0

        # Convertir planta también si es necesario
        plant = tab.synthesized_plant
        if hasattr(plant, 'A') and not hasattr(plant, 'num'):
            plant_tf = ct.ss2tf(plant)
            plant_num = np.array(plant_tf.num[0][0]).flatten()
            plant_den = np.array(plant_tf.den[0][0]).flatten()
        else:
            plant_num = np.array(plant.num[0][0]).flatten()
            plant_den = np.array(plant.den[0][0]).flatten()

        # Escribir archivo de texto
        with open(filename, 'w', encoding='utf-8') as f:
            f.write("=" * 70 + "\n")
            f.write("CONTROLADOR H∞ - Sistema de Control L206\n")
            f.write("=" * 70 + "\n\n")
            f.write(f"Nombre: {name}\n")
            f.write(f"Fecha: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            f.write(f"PLANTA G(s):\n  Numerador: {plant_num}\n  Denominador: {plant_den}\n\n")
            f.write(f"CONTROLADOR C(s):\n  Numerador: {num}\n  Denominador: {den}\n")
            if is_statespace:
                f.write(f"  (Convertido desde StateSpace de orden {len(controller.A)})\n")
            f.write(f"\nPARÁMETROS PI EQUIVALENTES:\n")
            f.write(f"  Kp = {Kp:.6f}\n")
            f.write(f"  Ki = {Ki:.6f}\n")
            f.write(f"  Gamma (γ) = {tab.gamma:.6f}\n\n")
            f.write(f"PARÁMETROS DE SÍNTESIS:\n")
            f.write(f"  K = {tab.K_input.text()} µm/s/PWM\n")
            f.write(f"  τ = {tab.tau_input.text()} s\n")
            f.write(f"  Ms = {tab.w1_Ms.text()}\n")
            f.write(f"  ωb = {tab.w1_wb.text()} rad/s\n")
            f.write(f"  U_max = {tab.w2_umax.text()} PWM\n")

        # Guardar pickle
        controller_data = {
            'name': name,
            'controller_num': num.tolist(),
            'controller_den': den.tolist(),
            'plant_num': plant_num.tolist(),
            'plant_den': plant_den.tolist(),
            'gamma': tab.gamma,
            'K': float(tab.K_input.text()),
            'tau': float(tab.tau_input.text()),
            'w1_Ms': float(tab.w1_Ms.text()),
            'w1_wb': float(tab.w1_wb.text()),
            'w1_eps': float(tab.w1_eps.text()),
            'w2_umax': float(tab.w2_umax.text()),
            'w3_wunc': float(tab.w3_wunc.text()),
            'w3_epsT': float(tab.w3_epsT.text()),
            'Kp': Kp,
            'Ki': Ki,
            'is_pi': is_pi,
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        }

        with open(pickle_filename, 'wb') as pf:
            pickle.dump(controller_data, pf)

        tab.results_text.append(f"\n✅ Controlador '{name}' exportado:\n  📄 {filename}\n  💾 {pickle_filename}")
        logger.info(f"Controlador exportado: {filename}")

    except Exception as e:
        logger.error(f"Error al exportar: {e}")
        import traceback
        logger.error(traceback.format_exc())
        tab.results_text.setText(f"❌ Error al exportar:\n{str(e)}")


def load_previous_controller(tab):
    """Carga un controlador H∞ guardado desde archivo pickle."""
    logger.info("HInfTab: Cargar Controlador Previo solicitado")

    try:
        filename, _ = QFileDialog.getOpenFileName(
            tab.parent_gui, "Seleccionar Controlador H∞ Guardado", "",
            "Archivos de Controlador (*.pkl);;Todos (*.*)"
        )

        if not filename:
            return

        with open(filename, 'rb') as pf:
            data = pickle.load(pf)

        # Reconstruir funciones de transferencia
        tab.synthesized_controller = ct.TransferFunction(
            data['controller_num'], data['controller_den']
        )
        tab.synthesized_plant = ct.TransferFunction(
            data['plant_num'], data['plant_den']
        )
        tab.gamma = data['gamma']

        # Restaurar parámetros en UI
        tab.K_input.setText(str(data['K']))
        tab.tau_input.setText(str(data['tau']))
        tab.w1_Ms.setText(str(data['w1_Ms']))
        tab.w1_wb.setText(str(data['w1_wb']))
        tab.w1_eps.setText(str(data['w1_eps']))
        tab.w2_umax.setText(str(data['w2_umax']))
        tab.w3_wunc.setText(str(data['w3_wunc']))
        tab.w3_epsT.setText(str(data['w3_epsT']))

        # Mostrar info
        tab.results_text.clear()
        tab.results_text.append("=" * 50)
        tab.results_text.append("✅ CONTROLADOR H∞ CARGADO")
        tab.results_text.append("=" * 50)
        tab.results_text.append(f"\n📂 Archivo: {filename}")
        tab.results_text.append(f"📅 Fecha: {data['timestamp']}")
        tab.results_text.append(f"\n🎯 Planta: K={data['K']:.4f}, τ={data['tau']:.4f}s")
        tab.results_text.append(f"📈 Gamma (γ): {tab.gamma:.4f}")
        if data.get('is_pi'):
            tab.results_text.append(f"\n🔧 PI: Kp={data['Kp']:.4f}, Ki={data['Ki']:.4f}")

        # Habilitar botones
        if hasattr(tab, 'transfer_btn'):
            tab.transfer_btn.setEnabled(True)
        if hasattr(tab, 'control_btn'):
            tab.control_btn.setEnabled(True)

        logger.info(f"✅ Controlador cargado desde {filename}")

    except Exception as e:
        logger.error(f"Error cargando controlador: {e}")
        QMessageBox.warning(tab.parent_gui, "Error", f"Error al cargar:\n{str(e)}")


# =============================================================================
# CONTROL EN TIEMPO REAL
# =============================================================================

def start_hinf_control(tab):
    """Inicia control H∞ en tiempo real."""
    logger.info("=== INICIANDO CONTROL H∞ ===")

    if tab.synthesized_controller is None:
        tab.results_text.append("❌ Error: Primero sintetiza el controlador")
        return

    if not tab.send_command_callback:
        tab.results_text.append("❌ Error: Callbacks de hardware no configurados")
        return

    try:
        Kp = tab.Kp_designed
        Ki = tab.Ki_designed
    except AttributeError:
        tab.results_text.append("❌ Error: Parámetros del controlador no disponibles")
        return

    # Aplicar factor de escala
    try:
        scale_factor = float(tab.scale_input.text())
        scale_factor = max(0.01, min(1.0, scale_factor))
    except:
        scale_factor = 0.1

    tab.Kp_control = Kp * scale_factor
    tab.Ki_control = Ki * scale_factor

    # Leer referencia
    try:
        tab.reference_um = float(tab.reference_input.text())
    except:
        QMessageBox.warning(tab.parent_gui, "Error", "Referencia inválida")
        return

    # Motor seleccionado
    tab.control_motor = 'A' if tab.motor_combo.currentIndex() == 0 else 'B'

    # Resetear variables
    tab.control_integral = 0.0
    tab.control_last_time = time.time()

    # Activar modo automático
    tab.send_command_callback('A,0,0')
    mode_label = tab.get_mode_label_callback()
    if mode_label:
        mode_label.setText("AUTOMÁTICO (H∞)")
        mode_label.setStyleSheet("font-weight: bold; color: #9B59B6;")

    time.sleep(0.1)
    tab.control_active = True

    # Actualizar botón
    if hasattr(tab, 'control_btn'):
        tab.control_btn.setText("⏹️ Detener Control H∞")
        tab.control_btn.setStyleSheet("font-weight: bold; padding: 8px; background: #E74C3C;")

    # Crear timer
    tab.control_timer = QTimer()
    tab.control_timer.timeout.connect(lambda: execute_hinf_control(tab))
    tab.control_timer.start(10)

    tab.results_text.append(f"\n🎮 Control H∞ ACTIVO")
    tab.results_text.append(f"   Motor: {tab.control_motor}, Kp={tab.Kp_control:.4f}, Ki={tab.Ki_control:.4f}")


def execute_hinf_control(tab):
    """Ejecuta un ciclo del controlador PI H∞."""
    try:
        current_time = time.time()
        Ts = current_time - tab.control_last_time
        tab.control_last_time = current_time

        # Leer sensor
        sensor_key = 'sensor_2' if tab.control_motor == 'A' else 'sensor_1'
        sensor_adc = tab.get_sensor_value_callback(sensor_key)

        if sensor_adc is None:
            logger.warning(f"Sensor {sensor_key} retornó None")
            return

        # Convertir referencia a ADC usando calibración dinámica
        axis = 'x' if tab.control_motor == 'A' else 'y'
        ref_adc = um_to_adc(tab.reference_um, axis=axis)

        # Error en ADC
        error = ref_adc - sensor_adc

        # Inicializar contador de log si no existe
        if not hasattr(tab, '_log_counter'):
            tab._log_counter = 0

        # Zona muerta configurable desde constants.py
        if abs(error) <= DEADZONE_ADC:
            tab.send_command_callback('A,0,0')
            tab.control_integral = 0
            tab._log_counter += 1
            if tab._log_counter % 50 == 0:
                tab.results_text.append(f"⚪ ZONA MUERTA | RefADC={ref_adc:.0f} | ADC={sensor_adc} | Err={error:.0f}")
            return

        # Actualizar integral
        tab.control_integral += error * Ts

        # Calcular PWM (PI controller)
        pwm_base = tab.Kp_control * error + tab.Ki_control * tab.control_integral

        # Invertir si necesario
        if hasattr(tab, 'invert_pwm') and tab.invert_pwm.isChecked():
            pwm_float = -pwm_base
        else:
            pwm_float = pwm_base

        # Limitar PWM
        PWM_MAX = int(tab.Umax_designed) if hasattr(tab, 'Umax_designed') else 100
        saturated = ""
        if pwm_float > PWM_MAX:
            pwm = PWM_MAX
            tab.control_integral -= error * Ts  # Anti-windup
            saturated = "SAT+"
        elif pwm_float < -PWM_MAX:
            pwm = -PWM_MAX
            tab.control_integral -= error * Ts  # Anti-windup
            saturated = "SAT-"
        else:
            pwm = int(pwm_float)

        # MOSTRAR EN TERMINAL (cada 10 ciclos ≈ 100ms)
        tab._log_counter += 1
        if tab._log_counter % 10 == 0:
            icon = "🔴" if saturated else "🟢"
            tab.results_text.append(
                f"{icon} RefADC={ref_adc:.0f} | ADC={sensor_adc} | Err={error:.0f} | Int={tab.control_integral:.1f} | PWM={pwm} {saturated}"
            )

        # Enviar comando
        if tab.control_motor == 'A':
            tab.send_command_callback(f"A,{pwm},0")
        else:
            tab.send_command_callback(f"A,0,{pwm}")

    except Exception as e:
        logger.error(f"Error en control H∞: {e}")


def stop_hinf_control(tab):
    """Detiene el control H∞."""
    logger.info("=== DETENIENDO CONTROL H∞ ===")

    if getattr(tab, 'control_timer', None):
        tab.control_timer.stop()

    tab.send_command_callback('A,0,0')
    time.sleep(0.05)
    tab.send_command_callback('M')

    mode_label = tab.get_mode_label_callback()
    if mode_label:
        mode_label.setText("MANUAL")
        mode_label.setStyleSheet("font-weight: bold; color: #E67E22;")

    tab.control_active = False

    if hasattr(tab, 'control_btn'):
        tab.control_btn.setText("🎮 Activar Control H∞")
        tab.control_btn.setStyleSheet("font-weight: bold; padding: 8px; background: #27AE60;")

    tab.results_text.append("\n⏹️ Control H∞ DETENIDO")

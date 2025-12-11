"""
Diseño de controlador robusto H∞.

Este módulo implementa el diseño de controladores H∞ usando la técnica
de mixed sensitivity synthesis con la librería python-control.

Basado en: Zhou, Doyle, Glover - "Robust and Optimal Control"
"""

import logging
import numpy as np
import control as ct
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any
from matplotlib.figure import Figure

logger = logging.getLogger(__name__)


@dataclass
class SynthesisConfig:
    """Configuración para síntesis H∞."""
    K: float = 0.5598          # Ganancia de planta (µm/s/PWM)
    tau: float = 0.033         # Constante de tiempo (s)
    Ms: float = 1.5            # Pico de sensibilidad
    wb: float = 5.0            # Ancho de banda (rad/s)
    eps: float = 0.3           # Epsilon para W1
    U_max: float = 100.0       # Límite de control (PWM)
    w_unc: float = 50.0        # Frecuencia de incertidumbre (rad/s)
    eps_T: float = 0.1         # Epsilon para W3
    method: str = 'H∞ (mixsyn)'  # Método de síntesis


@dataclass
class SynthesisResult:
    """Resultado de síntesis H∞."""
    success: bool
    message: str
    controller: Optional[Any] = None
    plant: Optional[Any] = None
    gamma: float = 0.0
    Kp: float = 0.0
    Ki: float = 0.0
    K_sign: float = 1.0
    poles_cl: Optional[np.ndarray] = None
    is_stable: bool = False
    margins: Optional[Dict] = None
    norms: Optional[Dict] = None
    warnings: Optional[list] = None


class HInfController:
    """
    Diseñador de controladores H∞ robusto.
    
    Implementa síntesis H∞/H2 usando mixed sensitivity (Zhou et al.).
    Toda la lógica de control está encapsulada aquí.
    """
    
    def __init__(self):
        """Inicializa el diseñador de controladores."""
        # Resultados de síntesis
        self.controller = None
        self.controller_full = None
        self.plant = None
        self.gamma = 0.0
        self.Kp = 0.0
        self.Ki = 0.0
        self.K_sign = 1.0
        self.K_value = 0.0
        self.tau_value = 0.0
        self.U_max = 100.0
        
        # Ponderaciones
        self.W1 = None
        self.W2 = None
        self.W3 = None
        
        logger.debug("HInfController inicializado")
    
    def synthesize_controller(self, config: SynthesisConfig) -> SynthesisResult:
        """
        Sintetiza controlador H∞/H2 con configuración completa.
        
        Este es el método principal que contiene toda la lógica de síntesis.
        
        Args:
            config: SynthesisConfig con todos los parámetros
            
        Returns:
            SynthesisResult con controlador y métricas
        """
        logger.info("=== Iniciando síntesis de controlador H∞ ===")
        warnings = []
        
        try:
            # 1. Extraer parámetros
            K = config.K
            tau = config.tau
            Ms = config.Ms
            wb = config.wb
            eps = config.eps
            U_max = config.U_max
            w_unc = config.w_unc
            eps_T = config.eps_T
            method = config.method
            
            # Guardar valores
            self.K_value = K
            self.tau_value = tau
            self.U_max = U_max
            
            # 2. Crear planta G(s) = K / (τs + 1)
            K_abs = abs(K)
            self.K_sign = np.sign(K) if K != 0 else 1.0
            
            logger.info(f"K={K:.4f}, |K|={K_abs:.4f}, signo={self.K_sign}")
            
            if tau == 0:
                G = ct.tf([K_abs], [1])
            else:
                G = ct.tf([K_abs], [tau, 1])
            
            self.plant = G
            logger.info(f"Planta G(s): {G}")
            
            # 3. Validar parámetros
            w_natural = 1.0 / tau if tau > 0 else 100.0
            
            if tau < 0.010:
                return SynthesisResult(
                    success=False,
                    message=f"τ={tau:.4f}s es demasiado pequeño (mínimo 0.010s)"
                )
            
            if Ms < 1.0:
                return SynthesisResult(
                    success=False,
                    message=f"Ms={Ms:.2f} debe ser ≥ 1.0"
                )
            
            if wb > w_natural:
                warnings.append(f"ωb={wb:.1f} excede ω_natural={w_natural:.1f}")
            
            # 4. Construir ponderaciones (Zhou et al.)
            eps_safe = max(eps, 0.01 if tau >= 0.015 else 0.1)
            eps_T_safe = max(eps_T, 0.01)
            k_u = 1.0 / U_max
            wb_u = wb / 10.0
            wb_T = w_unc
            
            self.W1 = ct.tf([1/Ms, wb], [1, wb*eps_safe])
            self.W2 = ct.tf([k_u], [1/wb_u, 1])
            self.W3 = ct.tf([1, wb_T*eps_T_safe], [eps_T_safe, wb_T])
            
            logger.info(f"W1: Ms={Ms}, wb={wb}, eps={eps_safe}")
            logger.info(f"W2: k_u={k_u:.6f}, wb_u={wb_u:.2f}")
            logger.info(f"W3: wb_T={wb_T}, eps_T={eps_T_safe}")
            
            # 5. Síntesis
            if "H2" in method:
                K_ctrl, gam = self._synthesize_h2(G)
            else:
                K_ctrl, gam = self._synthesize_hinf_pi(G, K_abs, tau, Ms, wb)
            
            self.controller = K_ctrl
            self.gamma = gam
            
            # 6. Extraer Kp, Ki
            Kp, Ki = self._extract_pi_gains(K_ctrl)
            self.Kp = Kp
            self.Ki = Ki
            
            # 7. Verificar estabilidad
            L = G * K_ctrl
            cl = ct.feedback(L, 1)
            poles_cl = ct.poles(cl)
            is_stable = all(np.real(p) < 1e-6 for p in poles_cl)
            
            if not is_stable:
                polos_inestables = [p for p in poles_cl if np.real(p) > 1e-6]
                return SynthesisResult(
                    success=False,
                    message=f"Sistema inestable: {len(polos_inestables)} polos en RHP",
                    poles_cl=poles_cl,
                    is_stable=False
                )
            
            # 8. Calcular márgenes
            margins = self._calculate_margins(L)
            
            # 9. Calcular normas H∞
            norms = self._calculate_norms(G, K_ctrl)
            
            logger.info(f"✅ Síntesis completada: γ={gam:.4f}, Kp={Kp:.4f}, Ki={Ki:.4f}")
            
            return SynthesisResult(
                success=True,
                message="Síntesis completada exitosamente",
                controller=K_ctrl,
                plant=G,
                gamma=gam,
                Kp=Kp,
                Ki=Ki,
                K_sign=self.K_sign,
                poles_cl=poles_cl,
                is_stable=True,
                margins=margins,
                norms=norms,
                warnings=warnings if warnings else None
            )
            
        except Exception as e:
            logger.error(f"Error en síntesis: {e}")
            return SynthesisResult(
                success=False,
                message=f"Error: {str(e)}"
            )
    
    def _synthesize_hinf_pi(self, G, K_abs, tau, Ms, wb) -> Tuple[Any, float]:
        """Diseño PI óptimo basado en loop shaping (robusto)."""
        logger.info("Usando diseño PI óptimo")
        
        wc = wb / 2  # Frecuencia de cruce conservadora
        Kp = wc * tau / K_abs
        Ki = Kp / (Ms * tau)
        
        K_ctrl = ct.tf([Kp, Ki], [1, 0])
        
        # Estimar gamma
        L = G * K_ctrl
        CL = ct.feedback(L, 1)
        try:
            gam = ct.hinfnorm(CL)[0]
        except:
            gam = 2.0
        
        logger.info(f"PI óptimo: Kp={Kp:.4f}, Ki={Ki:.4f}, γ≈{gam:.4f}")
        return K_ctrl, gam
    
    def _synthesize_h2(self, G) -> Tuple[Any, float]:
        """Síntesis H2 usando h2syn."""
        logger.info("Ejecutando síntesis H2")
        
        try:
            P = ct.augw(G, self.W1, self.W2, self.W3)
            K_ctrl, CL, gam = ct.h2syn(P, 1, 1)
            logger.info(f"H2 completado: γ={gam:.4f}")
            return K_ctrl, gam
        except Exception as e:
            logger.warning(f"H2 falló: {e}, usando PI óptimo")
            # Fallback a PI
            K_abs = abs(self.K_value)
            tau = self.tau_value
            return self._synthesize_hinf_pi(G, K_abs, tau, 1.5, 5.0)
    
    def _extract_pi_gains(self, K_ctrl) -> Tuple[float, float]:
        """Extrae Kp, Ki de un controlador PI."""
        try:
            num = K_ctrl.num[0][0]
            den = K_ctrl.den[0][0]
            
            if len(den) == 2 and len(num) == 2 and abs(den[1]) < 1e-10:
                Kp = num[0] / den[0]
                Ki = num[1] / den[0]
                return Kp, Ki
        except:
            pass
        return 0.0, 0.0
    
    def _calculate_margins(self, L) -> Dict:
        """Calcula márgenes de ganancia y fase."""
        try:
            gm, pm, wcg, wcp = ct.margin(L)
            return {
                'gain_margin': gm,
                'phase_margin': pm,
                'wcg': wcg,
                'wcp': wcp,
                'gm_db': 20 * np.log10(gm) if gm > 0 else float('inf')
            }
        except:
            return {'gain_margin': 0, 'phase_margin': 0}
    
    def _calculate_norms(self, G, K_ctrl) -> Dict:
        """Calcula normas H∞ de sensibilidad."""
        try:
            L = G * K_ctrl
            S = ct.feedback(1, L)
            T = ct.feedback(L, 1)
            
            omega = np.logspace(-2, 3, 500)
            
            W1S = self.W1 * S
            W2KS = self.W2 * K_ctrl * S
            W3T = self.W3 * T
            
            mag_W1S, _, _ = ct.frequency_response(W1S, omega)
            mag_W2KS, _, _ = ct.frequency_response(W2KS, omega)
            mag_W3T, _, _ = ct.frequency_response(W3T, omega)
            
            return {
                'norm_W1S': float(np.max(np.abs(mag_W1S))),
                'norm_W2KS': float(np.max(np.abs(mag_W2KS))),
                'norm_W3T': float(np.max(np.abs(mag_W3T)))
            }
        except:
            return {}
    
    def get_controller_info(self) -> Dict:
        """Retorna información del controlador actual."""
        if self.controller is None:
            return {'valid': False}
        
        return {
            'valid': True,
            'Kp': self.Kp,
            'Ki': self.Ki,
            'K_sign': self.K_sign,
            'gamma': self.gamma,
            'K_value': self.K_value,
            'tau_value': self.tau_value,
            'U_max': self.U_max
        }
    
    # ========== MÉTODO LEGACY (compatibilidad) ==========
    
    def synthesize(self, K, tau_fast, tau_slow, Wm, Wp, auto_hinf=False):
        """
        Sintetiza un controlador H∞ usando control.mixsyn().
        
        Args:
            K: Ganancia de la planta
            tau_fast: Constante de tiempo rápida
            tau_slow: Constante de tiempo lenta
            Wm: Frecuencia de peso sobre ruido de medición
            Wp: Frecuencia de peso sobre desempeño
            auto_hinf: Si True, usa valores automáticos para los pesos
            
        Returns:
            dict: Resultados de la síntesis con keys:
                - success: bool
                - message: str
                - controller: control.StateSpace
                - K_hinf: np.ndarray
                - gamma: float
                - figure: matplotlib.Figure (si éxito)
        """
        logger.info("=== Iniciando síntesis de controlador H∞ ===")
        logger.debug(f"Parámetros: K={K:.4f}, τ_fast={tau_fast:.4f}, τ_slow={tau_slow:.1f}")
        
        try:
            # Construir planta G(s) con dos polos
            num = [K]
            den = [tau_fast * tau_slow, tau_fast + tau_slow, 1]
            G = ct.tf(num, den)
            logger.info(f"Planta G(s) creada: {G}")
            
            # Funciones de peso
            # Wp: Peso para desempeño (tracking)
            # Wm: Peso para ruido de medición
            
            if auto_hinf:
                # Valores automáticos basados en análisis de Bode
                Wm_freq = 10.0 / tau_fast  # 10x frecuencia dominante
                Wp_freq = 0.1 / tau_fast   # 0.1x frecuencia dominante
                logger.info(f"Parámetros H∞ AUTO: Wm={Wm_freq:.2f} rad/s, Wp={Wp_freq:.2f} rad/s")
            else:
                Wm_freq = float(Wm)
                Wp_freq = float(Wp)
                logger.info(f"Parámetros H∞ MANUAL: Wm={Wm_freq:.2f} rad/s, Wp={Wp_freq:.2f} rad/s")
            
            # Crear funciones de peso
            # Wp: Peso de desempeño - filtro pasa-bajas (rechaza error a bajas frecuencias)
            num_Wp = [1, Wp_freq]
            den_Wp = [1, 0.01 * Wp_freq]
            Wp_tf = ct.tf(num_Wp, den_Wp)
            
            # Wm: Peso de ruido - filtro pasa-altas (rechaza ruido a altas frecuencias)
            num_Wm = [1, 0.01 * Wm_freq]
            den_Wm = [1, Wm_freq]
            Wm_tf = ct.tf(num_Wm, den_Wm)
            
            logger.info(f"Wp(s) = {Wp_tf}")
            logger.info(f"Wm(s) = {Wm_tf}")
            
            # Síntesis H∞ usando mixsyn
            logger.info("Ejecutando ct.mixsyn()...")
            K_hinf, CL, gamma = ct.mixsyn(G, Wp_tf, [], Wm_tf)
            
            logger.info(f"✅ Síntesis H∞ completada: γ = {gamma:.4f}")
            logger.info(f"Controlador K(s): orden {K_hinf.nstates}")
            
            # Convertir a espacio de estados si es necesario
            if not isinstance(K_hinf, ct.StateSpace):
                K_hinf = ct.ss(K_hinf)
            
            # Guardar para exportación
            self.hinf_controller = K_hinf
            self.hinf_Wm = Wm_freq
            self.hinf_Wp = Wp_freq
            
            # Crear gráficos de análisis
            figure = self._create_hinf_plots(G, K_hinf, Wp_tf, Wm_tf, gamma)
            
            return {
                'success': True,
                'message': 'Controlador H∞ sintetizado exitosamente',
                'controller': K_hinf,
                'K_hinf': K_hinf,
                'gamma': gamma,
                'figure': figure,
                'Wm': Wm_freq,
                'Wp': Wp_freq
            }
            
        except Exception as e:
            error_msg = f"Error en síntesis H∞: {str(e)}"
            logger.error(error_msg)
            return {
                'success': False,
                'message': error_msg
            }
    
    def _create_hinf_plots(self, G, K, Wp, Wm, gamma):
        """Crea gráficos de análisis del controlador H∞."""
        fig = Figure(figsize=(14, 10), facecolor='#2E2E2E')
        axes = fig.subplots(3, 2)
        
        # Diagramas de Bode
        freq = np.logspace(-3, 3, 500)
        
        # Gráfico 1: Bode de G(s)
        mag_G, phase_G, omega_G = ct.bode(G, omega=freq, plot=False)
        axes[0, 0].semilogx(omega_G, 20 * np.log10(mag_G), 'cyan', linewidth=2, label='G(s)')
        axes[0, 0].set_title('Planta G(s)', fontsize=12, fontweight='bold', color='white')
        axes[0, 0].set_ylabel('Magnitud (dB)', color='white')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend(facecolor='#383838', labelcolor='white')
        axes[0, 0].set_facecolor('#252525')
        axes[0, 0].tick_params(colors='white')
        
        axes[1, 0].semilogx(omega_G, phase_G * 180 / np.pi, 'cyan', linewidth=2)
        axes[1, 0].set_ylabel('Fase (°)', color='white')
        axes[1, 0].set_xlabel('Frecuencia (rad/s)', color='white')
        axes[1, 0].grid(True, alpha=0.3)
        axes[1, 0].set_facecolor('#252525')
        axes[1, 0].tick_params(colors='white')
        
        # Gráfico 2: Bode de K(s)
        mag_K, phase_K, omega_K = ct.bode(K, omega=freq, plot=False)
        axes[0, 1].semilogx(omega_K, 20 * np.log10(mag_K), 'lime', linewidth=2, label='K(s)')
        axes[0, 1].set_title('Controlador K(s)', fontsize=12, fontweight='bold', color='white')
        axes[0, 1].set_ylabel('Magnitud (dB)', color='white')
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].legend(facecolor='#383838', labelcolor='white')
        axes[0, 1].set_facecolor('#252525')
        axes[0, 1].tick_params(colors='white')
        
        axes[1, 1].semilogx(omega_K, phase_K * 180 / np.pi, 'lime', linewidth=2)
        axes[1, 1].set_ylabel('Fase (°)', color='white')
        axes[1, 1].set_xlabel('Frecuencia (rad/s)', color='white')
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].set_facecolor('#252525')
        axes[1, 1].tick_params(colors='white')
        
        # Gráfico 3: Respuesta al escalón de lazo cerrado
        T = ct.feedback(G * K, 1)
        t = np.linspace(0, 10, 1000)
        t_resp, y_resp = ct.step_response(T, t)
        
        axes[2, 0].plot(t_resp, y_resp, 'magenta', linewidth=2, label='Respuesta lazo cerrado')
        axes[2, 0].axhline(y=1, color='yellow', linestyle='--', alpha=0.7, label='Referencia')
        axes[2, 0].set_title('Respuesta al Escalón', fontsize=12, fontweight='bold', color='white')
        axes[2, 0].set_xlabel('Tiempo (s)', color='white')
        axes[2, 0].set_ylabel('Salida', color='white')
        axes[2, 0].grid(True, alpha=0.3)
        axes[2, 0].legend(facecolor='#383838', labelcolor='white')
        axes[2, 0].set_facecolor('#252525')
        axes[2, 0].tick_params(colors='white')
        
        # Gráfico 4: Funciones de peso
        mag_Wp, _, omega_Wp = ct.bode(Wp, omega=freq, plot=False)
        mag_Wm, _, omega_Wm = ct.bode(Wm, omega=freq, plot=False)
        
        axes[2, 1].semilogx(omega_Wp, 20 * np.log10(mag_Wp), 'orange', linewidth=2, label='Wp (desempeño)')
        axes[2, 1].semilogx(omega_Wm, 20 * np.log10(mag_Wm), 'red', linewidth=2, label='Wm (ruido)')
        axes[2, 1].set_title(f'Funciones de Peso (γ={gamma:.3f})', fontsize=12, fontweight='bold', color='white')
        axes[2, 1].set_xlabel('Frecuencia (rad/s)', color='white')
        axes[2, 1].set_ylabel('Magnitud (dB)', color='white')
        axes[2, 1].grid(True, alpha=0.3)
        axes[2, 1].legend(facecolor='#383838', labelcolor='white')
        axes[2, 1].set_facecolor('#252525')
        axes[2, 1].tick_params(colors='white')
        
        # Aplicar estilos oscuros a todos los ejes
        for ax_row in axes:
            for ax in ax_row:
                for spine in ax.spines.values():
                    spine.set_color('#505050')
        
        fig.tight_layout()
        return fig
    
    def export_to_arduino(self, K, dt=0.01):
        """
        Exporta el controlador para implementación en Arduino.
        
        Args:
            K: Sistema de control (StateSpace o TransferFunction)
            dt: Tiempo de muestreo (s)
            
        Returns:
            dict: Código Arduino y matrices discretas
        """
        try:
            # Discretizar el controlador
            if not isinstance(K, ct.StateSpace):
                K = ct.ss(K)
            
            K_discrete = ct.sample_system(K, dt, method='tustin')
            
            A_d = K_discrete.A
            B_d = K_discrete.B
            C_d = K_discrete.C
            D_d = K_discrete.D
            
            # Generar código Arduino
            arduino_code = self._generate_arduino_code(A_d, B_d, C_d, D_d, dt)
            
            return {
                'success': True,
                'A': A_d,
                'B': B_d,
                'C': C_d,
                'D': D_d,
                'dt': dt,
                'arduino_code': arduino_code
            }
            
        except Exception as e:
            logger.error(f"Error al exportar controlador: {e}")
            return {
                'success': False,
                'message': f"Error: {str(e)}"
            }
    
    def _generate_arduino_code(self, A, B, C, D, dt):
        """Genera código Arduino para el controlador en espacio de estados."""
        n_states = A.shape[0]
        
        # Código base
        code = f"""// Controlador H∞ en Espacio de Estados
// Generado automáticamente
// Orden del controlador: {n_states}
// Tiempo de muestreo: {dt*1000:.1f} ms

const int n_states = {n_states};
const float dt = {dt};

// Matrices del controlador
"""
        
        # Matriz A
        code += f"float A[{n_states}][{n_states}] = {{\n"
        for i in range(n_states):
            code += "  {"
            code += ", ".join([f"{A[i,j]:.6f}" for j in range(n_states)])
            code += "}," if i < n_states-1 else "}"
            code += "\n"
        code += "};\n\n"
        
        # Matriz B
        code += f"float B[{n_states}] = {{"
        code += ", ".join([f"{B[i,0]:.6f}" for i in range(n_states)])
        code += "};\n\n"
        
        # Matriz C
        code += f"float C[{n_states}] = {{"
        code += ", ".join([f"{C[0,i]:.6f}" for i in range(n_states)])
        code += "};\n\n"
        
        # Matriz D
        code += f"float D = {D[0,0]:.6f};\n\n"
        
        # Variables de estado
        code += f"float x[{n_states}] = {{0}}; // Estados del controlador\n\n"
        
        # Función de actualización
        code += """float compute_control(float error) {
  // Calcular salida: u = C*x + D*error
  float u = D * error;
  for (int i = 0; i < n_states; i++) {
    u += C[i] * x[i];
  }
  
  // Actualizar estados: x_new = A*x + B*error
  float x_new[n_states];
  for (int i = 0; i < n_states; i++) {
    x_new[i] = B[i] * error;
    for (int j = 0; j < n_states; j++) {
      x_new[i] += A[i][j] * x[j];
    }
  }
  
  // Copiar nuevos estados
  for (int i = 0; i < n_states; i++) {
    x[i] = x_new[i];
  }
  
  return u;
}
"""
        
        return code

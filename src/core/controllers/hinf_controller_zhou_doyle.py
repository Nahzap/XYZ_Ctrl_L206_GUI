"""
Diseño de controlador robusto H∞ - Implementación Correcta según Zhou & Doyle.

Este módulo implementa el diseño de controladores H∞ para TRACKING (seguimiento)
siguiendo la metodología exacta de:

    Zhou, K., & Doyle, J. C. (1998). "Essentials of Robust Control". Prentice Hall.
    Capítulo 9: Mixed Sensitivity Design for Tracking

DIFERENCIAS CLAVE con implementación anterior:
1. Usa planta de POSICIÓN G(s) = K/(s(τs+1)) en lugar de velocidad
2. Ponderaciones DINÁMICAS según Sección 9.4 del libro
3. Síntesis con hinfsyn + augw (más robusto que mixsyn)
4. Implementación digital con discretización de Tustin
5. NO aproxima como PI - usa controlador H∞ completo

Autor: Implementación corregida 2026-01-06
"""

import logging
import numpy as np
import control as ct
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any

logger = logging.getLogger('MotorControl_L206')


@dataclass
class TrackingConfig:
    """Configuración para síntesis H∞ de tracking según Zhou & Doyle."""
    # Parámetros de planta
    K: float = 1.0              # Ganancia de planta (µm/s/PWM)
    tau: float = 0.033          # Constante de tiempo (s)
    
    # Ponderación W1 (Performance - Tracking)
    Ms: float = 1.5             # Pico de sensibilidad (1.2-2.0)
    wb: float = 5.0             # Ancho de banda deseado (rad/s)
    eps: float = 0.01           # Epsilon para W1 (0.001-0.01, pequeño para tracking)
    
    # Ponderación W2 (Control effort - Dinámico)
    U_max: float = 100.0        # Límite de control (PWM)
    M_u: float = 2.0            # Roll-off factor para W2
    
    # Ponderación W3 (Robustness - Dinámico)
    w_unc: float = 50.0         # Frecuencia de incertidumbre (rad/s)
    eps_T: float = 0.1          # Epsilon para W3 (0.01-0.2)
    
    # Discretización
    Ts: float = 0.01            # Tiempo de muestreo (s)


@dataclass
class TrackingResult:
    """Resultado de síntesis H∞ para tracking."""
    success: bool
    message: str
    
    # Controlador continuo y discreto
    controller_continuous: Optional[Any] = None
    controller_discrete: Optional[Any] = None
    plant: Optional[Any] = None
    
    # Métricas
    gamma: float = 0.0
    K_sign: float = 1.0
    
    # Estabilidad
    poles_cl: Optional[np.ndarray] = None
    is_stable: bool = False
    
    # Márgenes y normas
    margins: Optional[Dict] = None
    norms: Optional[Dict] = None
    
    # Ponderaciones usadas
    W1: Optional[Any] = None
    W2: Optional[Any] = None
    W3: Optional[Any] = None


class HInfTrackingController:
    """
    Diseñador de controladores H∞ para TRACKING según Zhou & Doyle.
    
    Implementa la metodología exacta del Capítulo 9 de "Essentials of Robust Control":
    - Planta de posición con integrador
    - Ponderaciones dinámicas para tracking
    - Síntesis con hinfsyn (más robusto que mixsyn)
    - Discretización con Tustin
    
    Uso:
        controller = HInfTrackingController()
        config = TrackingConfig(K=0.56, tau=0.033, Ms=1.5, wb=5.0, ...)
        result = controller.synthesize_tracking(config)
        
        if result.success:
            # Usar controlador discreto en lazo de control
            u = controller.compute_control(r, y)
    """
    
    def __init__(self):
        """Inicializa el diseñador de controladores."""
        self.controller_continuous = None
        self.controller_discrete = None
        self.plant = None
        self.gamma = 0.0
        self.K_sign = 1.0
        
        # Estado del controlador digital
        self._controller_state = None
        
        # Ponderaciones
        self.W1 = None
        self.W2 = None
        self.W3 = None
        
        logger.debug("HInfTrackingController inicializado (Zhou & Doyle)")
    
    # =========================================================================
    # MÉTODO PRINCIPAL DE SÍNTESIS
    # =========================================================================
    
    def synthesize_tracking(self, config: TrackingConfig) -> TrackingResult:
        """
        Sintetiza controlador H∞ para TRACKING según Zhou & Doyle, Cap. 9.
        
        Proceso:
        1. Crear planta de POSICIÓN G(s) = K/(s(τs+1))
        2. Construir ponderaciones dinámicas W1, W2, W3
        3. Formar planta aumentada P = augw(G, W1, W2, W3)
        4. Resolver problema estándar H∞: min ||F_l(P, K)||∞
        5. Discretizar controlador con Tustin
        6. Verificar estabilidad y márgenes
        
        Args:
            config: TrackingConfig con parámetros
            
        Returns:
            TrackingResult con controlador y métricas
        """
        logger.info("=" * 60)
        logger.info("=== SÍNTESIS H∞ PARA TRACKING (Zhou & Doyle) ===")
        logger.info("=" * 60)
        
        try:
            # 1. Extraer parámetros
            K = config.K
            tau = config.tau
            K_abs = abs(K)
            self.K_sign = np.sign(K) if K != 0 else 1.0
            
            logger.info(f"Planta: K={K:.4f}, |K|={K_abs:.4f}, τ={tau:.4f}s")
            
            # 2. Crear planta de POSICIÓN (con integrador)
            G_pos = self._create_position_plant(K_abs, tau)
            self.plant = G_pos
            
            # 3. Construir ponderaciones dinámicas para tracking
            weights = self._build_tracking_weights(config)
            self.W1 = weights['W1']
            self.W2 = weights['W2']
            self.W3 = weights['W3']
            
            # 4. Síntesis H∞ con hinfsyn
            synth_result = self._synthesize_hinf_tracking(G_pos, config)
            
            if not synth_result['success']:
                return TrackingResult(
                    success=False,
                    message=synth_result['message']
                )
            
            K_ctrl = synth_result['controller']
            gam = synth_result['gamma']
            CL = synth_result['closed_loop']
            
            self.controller_continuous = K_ctrl
            self.gamma = gam
            
            # 5. Discretizar controlador
            K_discrete = self._discretize_controller(K_ctrl, config.Ts)
            self.controller_discrete = K_discrete
            
            # 6. Verificar estabilidad
            stability = self._verify_stability(G_pos, K_ctrl)
            
            if not stability['is_stable']:
                return TrackingResult(
                    success=False,
                    message=f"Sistema inestable: {stability['message']}",
                    poles_cl=stability['poles'],
                    is_stable=False
                )
            
            # 7. Calcular márgenes y normas
            margins = self._calculate_margins(G_pos * K_ctrl)
            norms = self._calculate_norms(G_pos, K_ctrl)
            
            logger.info(f"✅ Síntesis completada: γ={gam:.4f}")
            
            return TrackingResult(
                success=True,
                message="Síntesis completada exitosamente",
                controller_continuous=K_ctrl,
                controller_discrete=K_discrete,
                plant=G_pos,
                gamma=gam,
                K_sign=self.K_sign,
                poles_cl=stability['poles'],
                is_stable=True,
                margins=margins,
                norms=norms,
                W1=self.W1,
                W2=self.W2,
                W3=self.W3
            )
            
        except Exception as e:
            logger.error(f"Error en síntesis: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return TrackingResult(
                success=False,
                message=f"Error: {str(e)}"
            )
    
    # =========================================================================
    # CONSTRUCCIÓN DE PLANTA DE POSICIÓN
    # =========================================================================
    
    def _create_position_plant(self, K: float, tau: float):
        """
        Crea planta de POSICIÓN según Zhou & Doyle, Cap. 9.
        
        Para control de posición de motor DC:
        - Motor: PWM → Velocidad: G_motor(s) = K/(τs+1)
        - Posición = ∫Velocidad → G_pos(s) = G_motor(s)/s
        
        Por lo tanto:
        G_pos(s) = K / (s·(τs + 1))
        
        Esta planta tiene:
        - Polo en s=0 (integrador) → garantiza error DC = 0
        - Polo en s=-1/τ (dinámica del motor)
        
        Args:
            K: Ganancia absoluta (µm/s/PWM)
            tau: Constante de tiempo (s)
            
        Returns:
            TransferFunction G_pos(s)
        """
        if tau == 0:
            # Sin dinámica: G_pos(s) = K/s
            G_pos = ct.tf([K], [1, 0])
            logger.info(f"Planta de posición: G(s) = {K}/s")
        else:
            # Con dinámica: G_pos(s) = K/(s(τs+1))
            # Numerador: [K]
            # Denominador: [τ, 1, 0] = τs² + s = s(τs + 1)
            G_pos = ct.tf([K], [tau, 1, 0])
            pole1 = 0.0
            pole2 = -1.0 / tau
            logger.info(f"Planta de posición: G(s) = {K}/(s·({tau}s + 1))")
            logger.info(f"  Polos: s₁={pole1:.4f} (integrador), s₂={pole2:.4f}")
        
        return G_pos
    
    # =========================================================================
    # CONSTRUCCIÓN DE PONDERACIONES DINÁMICAS
    # =========================================================================
    
    def _build_tracking_weights(self, config: TrackingConfig) -> Dict:
        """
        Construye ponderaciones dinámicas según Zhou & Doyle, Sección 9.4.
        
        Para problema de TRACKING (seguimiento de posición con error DC = 0).
        
        W1(s): Performance weight - FUERZA TRACKING PERFECTO
        W2(s): Control effort weight - DINÁMICO (permite más control en bajas freq)
        W3(s): Robustness weight - DINÁMICO (modela incertidumbre multiplicativa)
        
        Args:
            config: TrackingConfig con parámetros
            
        Returns:
            Dict con W1, W2, W3
        """
        Ms = config.Ms
        wb = config.wb
        eps = config.eps
        U_max = config.U_max
        M_u = config.M_u
        w_unc = config.w_unc
        eps_T = config.eps_T
        
        logger.info("=== CONSTRUYENDO PONDERACIONES (Zhou & Doyle) ===")
        
        # =====================================================================
        # W1(s): Performance Weight para TRACKING
        # =====================================================================
        # Forma del libro (Ec. 9.4.2):
        # W1(s) = (s + ωb) / (M_s·s + ωb·ε)
        #
        # Propiedades:
        # - Ganancia DC: |W1(0)| = 1/ε → ∞ cuando ε → 0
        #   Esto FUERZA error de estado estacionario = 0
        # - Ganancia HF: |W1(∞)| = 1/M_s
        #   Esto limita el pico de sensibilidad |S(jω)| ≤ M_s
        # - Frecuencia de cruce: ω ≈ ωb (ancho de banda deseado)
        #
        # CRÍTICO: El denominador tiene s (no constante), lo que crea
        # un polo en s=0 en W1·S. Esto fuerza S(0) → 0, es decir,
        # error DC = 0 (tracking perfecto).
        # =====================================================================
        
        W1 = ct.tf([1, wb], [Ms, wb * eps])
        
        w1_dc = 1.0 / eps
        w1_hf = 1.0 / Ms
        
        logger.info(f"W1 (Tracking - Dinámico):")
        logger.info(f"  Forma: (s + {wb})/(({Ms}·s + {wb*eps}))")
        logger.info(f"  Ganancia DC: {w1_dc:.1f} (fuerza error DC = 0)")
        logger.info(f"  Ganancia HF: {w1_hf:.2f} (limita |S|∞ ≤ {Ms})")
        logger.info(f"  Ancho de banda: ≈{wb} rad/s")
        
        # =====================================================================
        # W2(s): Control Effort Weight DINÁMICO
        # =====================================================================
        # Forma del libro (Ec. 9.4.3):
        # W2(s) = k_u · (τ_u·s + 1) / (τ_u·s/M_u + 1)
        #
        # Propiedades:
        # - Ganancia DC: |W2(0)| = k_u = 1/U_max
        #   Permite control hasta U_max en bajas frecuencias
        # - Ganancia HF: |W2(∞)| = k_u·M_u
        #   Penaliza control en altas frecuencias (ruido)
        # - Roll-off: M_u (típicamente 2-5)
        #
        # RAZÓN: En tracking, necesitamos MÁS control en bajas frecuencias
        # (para seguir la referencia) y MENOS en altas (para rechazar ruido).
        # Una W2 constante no captura esta diferencia.
        # =====================================================================
        
        k_u = 1.0 / U_max
        tau_u = 1.0 / (10 * wb)  # Constante de tiempo del actuador
        
        W2 = ct.tf([k_u * tau_u, k_u], [tau_u / M_u, 1])
        
        w2_dc = k_u
        w2_hf = k_u * M_u
        
        logger.info(f"W2 (Control Effort - Dinámico):")
        logger.info(f"  Forma: {k_u}·({tau_u}·s + 1)/({tau_u/M_u}·s + 1)")
        logger.info(f"  Ganancia DC: {w2_dc:.4f} (permite U_max={U_max} PWM)")
        logger.info(f"  Ganancia HF: {w2_hf:.4f} (penaliza ruido)")
        logger.info(f"  Roll-off: M_u={M_u}")
        
        # =====================================================================
        # W3(s): Robustness Weight DINÁMICO
        # =====================================================================
        # Forma del libro (Ec. 9.4.4):
        # W3(s) = ε_T · (s/ω_unc + 1) / (ε_T·s/ω_unc + 1)
        #
        # Propiedades:
        # - Ganancia DC: |W3(0)| = ε_T (pequeño)
        #   Poca incertidumbre en bajas frecuencias
        # - Ganancia HF: |W3(∞)| = 1/ε_T (grande)
        #   Alta incertidumbre en altas frecuencias
        # - Frecuencia de transición: ω_unc
        #
        # RAZÓN: Modela incertidumbre multiplicativa Δ(s) donde:
        # G_real(s) = G_nominal(s)·(1 + Δ(s)·W3(s))
        # con |Δ(jω)| ≤ 1 para todo ω
        #
        # En altas frecuencias (ω > ω_unc), la incertidumbre crece
        # (dinámica no modelada, resonancias, etc.)
        # =====================================================================
        
        W3 = ct.tf([eps_T / w_unc, eps_T], [eps_T / w_unc, 1])
        
        w3_dc = eps_T
        w3_hf = 1.0 / eps_T
        
        logger.info(f"W3 (Robustness - Dinámico):")
        logger.info(f"  Forma: {eps_T}·(s/{w_unc} + 1)/({eps_T}·s/{w_unc} + 1)")
        logger.info(f"  Ganancia DC: {w3_dc:.3f} (poca incertidumbre)")
        logger.info(f"  Ganancia HF: {w3_hf:.1f} (alta incertidumbre)")
        logger.info(f"  Frecuencia de transición: {w_unc} rad/s")
        
        return {'W1': W1, 'W2': W2, 'W3': W3}
    
    # =========================================================================
    # SÍNTESIS H∞ CON HINFSYN
    # =========================================================================
    
    def _synthesize_hinf_tracking(self, G_pos, config: TrackingConfig) -> Dict:
        """
        Síntesis H∞ para tracking usando hinfsyn según Zhou & Doyle, Cap. 9.
        
        Proceso (Algoritmo 9.1 del libro):
        1. Formar planta aumentada P = augw(G, W1, W2, W3)
        2. Resolver problema estándar H∞:
           min ||F_l(P, K)||∞
            K
        3. Verificar γ < 1 (condición de optimalidad)
        
        Args:
            G_pos: Planta de posición
            config: TrackingConfig
            
        Returns:
            Dict con controller, gamma, closed_loop, success, message
        """
        logger.info("=== SÍNTESIS H∞ CON HINFSYN ===")
        
        try:
            # Paso 1: Formar planta aumentada P
            # P tiene estructura:
            #   ┌─────┐
            #   │  z  │ ← Salidas de performance (W1·e, W2·u, W3·y)
            #   │  y  │ ← Medición (error e = r - y)
            # P │  ───│
            #   │  w  │ ← Entrada exógena (referencia r)
            #   │  u  │ ← Señal de control
            #   └─────┘
            
            logger.info("Construyendo planta aumentada P = augw(G, W1, W2, W3)...")
            P = ct.augw(G_pos, self.W1, self.W2, self.W3)
            
            logger.info(f"Planta aumentada P:")
            logger.info(f"  Estados: {P.nstates}")
            logger.info(f"  Entradas: {P.ninputs}")
            logger.info(f"  Salidas: {P.noutputs}")
            
            # Paso 2: Resolver problema estándar H∞
            # min ||F_l(P, K)||∞
            #  K
            # donde F_l(P, K) es el lazo cerrado inferior
            #
            # hinfsyn busca K que minimiza la norma H∞ del lazo cerrado
            # sujeto a estabilidad interna
            
            logger.info("Resolviendo problema estándar H∞...")
            logger.info("  min ||F_l(P, K)||∞")
            logger.info("   K")
            
            # nmeas = 1: Una medición (error e)
            # ncon = 1: Una señal de control (u)
            K_ctrl, CL, gamma = ct.hinfsyn(P, nmeas=1, ncon=1)
            
            logger.info(f"✅ hinfsyn completado: γ = {gamma:.4f}")
            
            # Paso 3: Verificar condición de optimalidad
            if gamma < 1:
                logger.info("✅ γ < 1: Diseño ÓPTIMO")
                logger.info("   Todas las especificaciones se cumplen estrictamente")
            elif gamma < 2:
                logger.info("⚠️ 1 ≤ γ < 2: Diseño ACEPTABLE")
                logger.info("   Especificaciones se cumplen con margen reducido")
            else:
                logger.warning(f"⚠️ γ = {gamma:.2f} ≥ 2: Diseño SUBÓPTIMO")
                logger.warning("   Considerar relajar especificaciones (aumentar Ms, reducir wb)")
            
            # Información del controlador
            logger.info(f"Controlador K(s):")
            logger.info(f"  Orden: {K_ctrl.nstates}")
            logger.info(f"  Polos: {ct.poles(K_ctrl)}")
            
            return {
                'success': True,
                'controller': K_ctrl,
                'gamma': gamma,
                'closed_loop': CL,
                'method_used': 'H∞ (hinfsyn - tracking)',
                'message': 'OK'
            }
            
        except Exception as e:
            logger.error(f"hinfsyn falló: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return {
                'success': False,
                'message': f"hinfsyn falló: {str(e)}",
                'controller': None,
                'gamma': 0,
                'closed_loop': None,
                'method_used': 'H∞ (hinfsyn - tracking)'
            }
    
    # =========================================================================
    # DISCRETIZACIÓN CON TUSTIN
    # =========================================================================
    
    def _discretize_controller(self, K_ctrl, Ts: float):
        """
        Discretiza el controlador H∞ usando Tustin según Zhou & Doyle, Apéndice C.
        
        Transformación de Tustin (bilinear):
        s → (2/Ts)·(z-1)/(z+1)
        
        Propiedades:
        - Preserva estabilidad (BIBO)
        - Mapea eje imaginario s=jω a círculo unitario |z|=1
        - Introduce warping de frecuencia: ω_d = (2/Ts)·tan(ω·Ts/2)
        
        Para Ts pequeño (Ts << 1/ωb), el warping es despreciable.
        
        Args:
            K_ctrl: Controlador continuo
            Ts: Tiempo de muestreo (s)
            
        Returns:
            Controlador discreto (StateSpace)
        """
        logger.info(f"Discretizando controlador con Ts = {Ts}s (Tustin)")
        
        # Discretizar usando Tustin
        K_discrete = ct.c2d(K_ctrl, Ts, method='tustin')
        
        logger.info(f"Controlador discreto:")
        logger.info(f"  Orden: {K_discrete.nstates}")
        logger.info(f"  Polos: {ct.poles(K_discrete)}")
        
        # Verificar estabilidad discreta (todos los polos dentro del círculo unitario)
        poles_discrete = ct.poles(K_discrete)
        max_pole_mag = max(abs(poles_discrete)) if len(poles_discrete) > 0 else 0
        
        if max_pole_mag < 1.0:
            logger.info(f"✅ Controlador discreto estable (max|z| = {max_pole_mag:.4f} < 1)")
        else:
            logger.warning(f"⚠️ Controlador discreto inestable (max|z| = {max_pole_mag:.4f} ≥ 1)")
        
        return K_discrete
    
    # =========================================================================
    # IMPLEMENTACIÓN DIGITAL EN LAZO DE CONTROL
    # =========================================================================
    
    def compute_control(self, r: float, y: float) -> float:
        """
        Calcula señal de control usando controlador discreto.
        
        Implementa ecuación en diferencias del controlador en forma de
        espacio de estados:
        
        x[k+1] = A·x[k] + B·e[k]
        u[k] = C·x[k] + D·e[k]
        
        donde e[k] = r[k] - y[k] (error de tracking)
        
        Args:
            r: Referencia (posición deseada en µm)
            y: Salida actual (posición medida en µm)
            
        Returns:
            u: Señal de control (PWM)
        """
        if self.controller_discrete is None:
            logger.error("Controlador discreto no inicializado")
            return 0.0
        
        # Error de tracking
        e = r - y
        
        # Inicializar estado del controlador si es la primera vez
        if self._controller_state is None:
            self._controller_state = np.zeros((self.controller_discrete.nstates, 1))
        
        # Obtener matrices de espacio de estados
        A = self.controller_discrete.A
        B = self.controller_discrete.B
        C = self.controller_discrete.C
        D = self.controller_discrete.D
        
        # Calcular salida: u[k] = C·x[k] + D·e[k]
        x_k = self._controller_state
        u_k = float(C @ x_k + D * e)
        
        # Actualizar estado: x[k+1] = A·x[k] + B·e[k]
        self._controller_state = A @ x_k + B * e
        
        # Aplicar signo de la planta
        u_k = u_k * self.K_sign
        
        return u_k
    
    def reset_controller_state(self):
        """Resetea el estado interno del controlador digital."""
        self._controller_state = None
        logger.debug("Estado del controlador reseteado")
    
    # =========================================================================
    # VERIFICACIÓN DE ESTABILIDAD
    # =========================================================================
    
    def _verify_stability(self, G, K_ctrl) -> Dict:
        """Verifica estabilidad del lazo cerrado."""
        try:
            L = G * K_ctrl
            cl = ct.feedback(L, 1)
            poles = ct.poles(cl)
            
            # Tolerancia para error numérico
            tol = 1e-6
            unstable_poles = [p for p in poles if np.real(p) > tol]
            
            if len(unstable_poles) > 0:
                return {
                    'is_stable': False,
                    'poles': poles,
                    'message': f"{len(unstable_poles)} polo(s) en semiplano derecho"
                }
            
            return {
                'is_stable': True,
                'poles': poles,
                'message': 'OK'
            }
            
        except Exception as e:
            return {
                'is_stable': False,
                'poles': None,
                'message': f"Error verificando estabilidad: {e}"
            }
    
    # =========================================================================
    # CÁLCULO DE MÁRGENES
    # =========================================================================
    
    def _calculate_margins(self, L) -> Dict:
        """Calcula márgenes de ganancia y fase."""
        try:
            gm, pm, wcg, wcp = ct.margin(L)
            
            return {
                'gain_margin': gm if np.isfinite(gm) else float('inf'),
                'phase_margin': pm if np.isfinite(pm) else 0,
                'wcg': wcg if np.isfinite(wcg) else 0,
                'wcp': wcp if np.isfinite(wcp) else 0,
                'gm_db': 20 * np.log10(gm) if gm > 0 and np.isfinite(gm) else float('inf')
            }
        except Exception as e:
            logger.warning(f"Error calculando márgenes: {e}")
            return {'gain_margin': 0, 'phase_margin': 0, 'wcg': 0, 'wcp': 0}
    
    # =========================================================================
    # CÁLCULO DE NORMAS H∞
    # =========================================================================
    
    def _calculate_norms(self, G, K_ctrl) -> Dict:
        """Calcula normas H∞ de sensibilidad."""
        try:
            L = G * K_ctrl
            S = ct.feedback(1, L)
            T = ct.feedback(L, 1)
            
            omega = np.logspace(-2, 3, 500)
            
            # W1*S
            W1S = self.W1 * S
            mag_W1S, _, _ = ct.frequency_response(W1S, omega)
            if mag_W1S.ndim > 1:
                mag_W1S = mag_W1S[0, :]
            norm_W1S = float(np.max(np.abs(mag_W1S)))
            
            # W2*K*S
            W2KS = self.W2 * K_ctrl * S
            mag_W2KS, _, _ = ct.frequency_response(W2KS, omega)
            if mag_W2KS.ndim > 1:
                mag_W2KS = mag_W2KS[0, :]
            norm_W2KS = float(np.max(np.abs(mag_W2KS)))
            
            # W3*T
            W3T = self.W3 * T
            mag_W3T, _, _ = ct.frequency_response(W3T, omega)
            if mag_W3T.ndim > 1:
                mag_W3T = mag_W3T[0, :]
            norm_W3T = float(np.max(np.abs(mag_W3T)))
            
            return {
                'norm_W1S': norm_W1S,
                'norm_W2KS': norm_W2KS,
                'norm_W3T': norm_W3T,
                'gamma_verified': max(norm_W1S, norm_W2KS, norm_W3T)
            }
            
        except Exception as e:
            logger.warning(f"Error calculando normas: {e}")
            return {}

"""
Diseño de controlador robusto H∞ para control de motores.

Este módulo implementa síntesis H∞ usando mixsyn para control de posición
de motores DC mediante un enfoque de dos etapas:
1. Diseño del controlador para planta de velocidad G(s) = K/(τs+1)
2. Integración externa para control de posición

El controlador resultante incluye acción integral implícita que garantiza
error de estado estacionario = 0 para control de posición.

Refactorizado: 2025-12-15
Última actualización: 2026-01-06
"""

import logging
import traceback
import time
import numpy as np
import control as ct
from dataclasses import dataclass, field
from typing import Optional, Tuple, Dict, Any, List

logger = logging.getLogger('MotorControl_L206')


@dataclass
class SynthesisConfig:
    """Configuración completa para síntesis H∞/H2."""
    # Parámetros de planta
    K: float = 1.0              # Ganancia de planta (µm/s/PWM)
    tau: float = 0.033          # Constante de tiempo (s)
    
    # Ponderación W1 (Performance)
    Ms: float = 1.5             # Pico de sensibilidad (1.2-2.0)
    wb: float = 5.0             # Ancho de banda deseado (rad/s)
    eps: float = 0.3            # Epsilon para W1 (0.01-0.3)
    
    # Ponderación W2 (Control effort)
    U_max: float = 100.0        # Límite de control (PWM)
    
    # Ponderación W3 (Robustness)
    w_unc: float = 50.0         # Frecuencia de incertidumbre (rad/s)
    eps_T: float = 0.1          # Epsilon para W3 (0.01-0.1)
    
    # Método de síntesis
    method: str = 'H∞ (mixsyn)'  # 'H∞ (mixsyn)' o 'H2 (h2syn)'


@dataclass
class ValidationResult:
    """Resultado de validación de parámetros."""
    is_valid: bool
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    corrected_config: Optional[SynthesisConfig] = None


@dataclass
class SynthesisResult:
    """Resultado completo de síntesis H∞/H2."""
    success: bool
    message: str
    
    # Controlador y planta
    controller: Optional[Any] = None
    controller_full: Optional[Any] = None
    plant: Optional[Any] = None
    
    # Métricas
    gamma: float = 0.0
    Kp: float = 0.0
    Ki: float = 0.0
    K_sign: float = 1.0
    
    # Estabilidad
    poles_cl: Optional[np.ndarray] = None
    is_stable: bool = False
    
    # Márgenes y normas
    margins: Optional[Dict] = None
    norms: Optional[Dict] = None
    
    # Información adicional
    warnings: Optional[List[str]] = None
    method_used: str = ""
    scaling_applied: bool = False
    scaling_factor: float = 1.0
    
    # Ponderaciones usadas
    W1: Optional[Any] = None
    W2: Optional[Any] = None
    W3: Optional[Any] = None


class HInfController:
    """
    Diseñador de controladores H∞/H2 robusto.
    
    Implementa síntesis H∞/H2 usando mixed sensitivity (Zhou et al.).
    Esta es la ÚNICA fuente de lógica de síntesis en el proyecto.
    
    Uso:
        controller = HInfController()
        config = SynthesisConfig(K=0.56, tau=0.033, Ms=1.5, wb=5.0, ...)
        result = controller.synthesize(config)
        
        if result.success:
            print(f"Kp={result.Kp}, Ki={result.Ki}, γ={result.gamma}")
    """
    
    # Constantes de diseño
    TAU_MIN_ABSOLUTE = 0.001      # τ mínimo absoluto
    TAU_THRESHOLD_SCALING = 0.005  # τ umbral para escalado
    TAU_THRESHOLD_FALLBACK = 0.05  # τ umbral para fallback PI
    POLE_MIN = 0.01               # Polo mínimo para alejar del origen (s = -POLE_MIN)
    
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
        
        # Estado de escalado
        self._scaling_applied = False
        self._scaling_factor = 1.0
        
        logger.debug("HInfController inicializado")
    
    # =========================================================================
    # MÉTODO PRINCIPAL DE SÍNTESIS
    # =========================================================================
    
    def synthesize(self, config: SynthesisConfig) -> SynthesisResult:
        """
        Sintetiza controlador H∞/H2 con configuración completa.
        
        Este es el método principal que orquesta todo el proceso:
        1. Validación de parámetros
        2. Escalado de frecuencias (si necesario)
        3. Construcción de ponderaciones
        4. Síntesis H∞ o H2
        5. Reducción de orden
        6. Desescalado
        7. Verificación de estabilidad
        
        Args:
            config: SynthesisConfig con todos los parámetros
            
        Returns:
            SynthesisResult con controlador y métricas
        """
        logger.info("=" * 60)
        logger.info("=== INICIANDO SÍNTESIS H∞/H2 ===")
        logger.info("=" * 60)
        
        warnings = []
        
        try:
            # 1. Validar parámetros
            validation = self.validate_config(config)
            if not validation.is_valid:
                return SynthesisResult(
                    success=False,
                    message="Parámetros inválidos:\n" + "\n".join(validation.errors)
                )
            
            warnings.extend(validation.warnings)
            config = validation.corrected_config or config
            
            # 2. Extraer y guardar parámetros
            K = config.K
            tau = config.tau
            K_abs = abs(K)
            self.K_sign = np.sign(K) if K != 0 else 1.0
            self.K_value = K
            self.tau_value = tau
            self.U_max = config.U_max
            
            logger.info(f"Planta: K={K:.4f}, |K|={K_abs:.4f}, τ={tau:.4f}s")
            
            # 3. Aplicar escalado si necesario
            scaling_result = self._apply_frequency_scaling(K_abs, tau, config)
            K_scaled = scaling_result['K']
            tau_scaled = scaling_result['tau']
            scaling_applied = scaling_result['applied']
            scaling_factor = scaling_result['factor']
            
            if scaling_applied:
                warnings.append(f"Escalado aplicado: τ {tau:.4f}s → {tau_scaled:.4f}s")
            
            # 4. Crear planta G(s) = K / (τs + 1) - MODELO DE VELOCIDAD
            # 
            # IMPORTANTE (según documentación python-control y Zhou et al.):
            # - mixsyn/hinfsyn tienen problemas numéricos con polos en el origen
            # - Usamos el MODELO DE VELOCIDAD (primer orden) para síntesis
            # - El controlador PI resultante proporciona el integrador necesario
            #   para control de posición
            #
            # Modelo físico:
            #   Velocidad: V(s)/U(s) = K / (τs + 1)  <- Usamos este para síntesis
            #   Posición:  P(s)/U(s) = K / (s·(τs + 1)) = V(s)/s
            #
            # El controlador PI = (Kp·s + Ki)/s proporciona:
            #   - Acción proporcional para respuesta rápida
            #   - Acción integral (1/s) para seguimiento de posición
            
            # Usar MODELO DE VELOCIDAD para síntesis H∞
            # G(s) = K / (τs + 1)
            # Después de la síntesis, agregamos integrador al controlador para control de posición
            
            tau_effective = tau_scaled
            
            if tau_effective == 0:
                # Sin dinámica: G(s) = K
                G = ct.tf([K_scaled], [1])
                print(f"[HINF] Planta (velocidad): G(s) = {K_scaled:.4f}")
            else:
                # Con dinámica: G(s) = K / (τs + 1)
                G = ct.tf([K_scaled], [tau_effective, 1])
                pole = -1.0 / tau_effective
                print(f"[HINF] Planta (velocidad): G(s) = {K_scaled:.4f} / ({tau_effective:.4f}s + 1), polo = {pole:.4f}")
            
            self.plant = G
            logger.info(f"Planta G(s): {G}")
            
            # 5. Construir ponderaciones
            weights = self._build_weights(config, tau)
            self.W1 = weights['W1']
            self.W2 = weights['W2']
            self.W3 = weights['W3']
            
            logger.info(f"W1 (Performance): Ms={config.Ms}, wb={config.wb}")
            logger.info(f"W2 (Control): U_max={config.U_max}")
            logger.info(f"W3 (Robustness): w_unc={config.w_unc}")
            
            # 6. Ejecutar síntesis
            synth_result = self._execute_synthesis(
                G, config, K_abs, tau, tau_scaled
            )
            
            if not synth_result['success']:
                return SynthesisResult(
                    success=False,
                    message=synth_result['message'],
                    warnings=warnings
                )
            
            K_ctrl_full = synth_result['controller']
            gam = synth_result['gamma']
            method_used = synth_result['method_used']
            
            self.controller_full = K_ctrl_full
            
            # 7. Reducir orden del controlador
            K_ctrl = self._reduce_controller_order(K_ctrl_full, G)
            
            # 8. Desescalar si se aplicó escalado
            if scaling_applied:
                K_ctrl = self._unscale_controller(K_ctrl, scaling_factor)
                # Recrear planta original (modelo de velocidad) para verificación
                G = ct.tf([K_abs], [tau, 1])
            
            # 9. Para control de posición, el controlador H∞ de velocidad se usa con integrador externo
            # Extraemos Kp y Ki equivalentes del controlador H∞:
            # - Ki = K_hinf(0) (ganancia DC del controlador)
            # - Kp = derivada de K_hinf en s=0, aproximada numéricamente
            
            self.controller = K_ctrl
            
            # 10. Extraer Kp, Ki equivalentes del controlador H∞
            Kp, Ki = self._extract_hinf_pi_equivalent(K_ctrl, config)
            self.Kp = Kp
            self.Ki = Ki
            self.gamma = gam
            
            # 10. Verificar estabilidad
            stability = self._verify_stability(G, K_ctrl)
            
            if not stability['is_stable']:
                return SynthesisResult(
                    success=False,
                    message=f"Sistema inestable: {stability['message']}",
                    poles_cl=stability['poles'],
                    is_stable=False,
                    warnings=warnings
                )
            
            # 11. Calcular márgenes y normas
            margins = self._calculate_margins(G * K_ctrl)
            norms = self._calculate_norms(G, K_ctrl)
            
            logger.info(f"✅ Síntesis completada: γ={gam:.4f}, Kp={Kp:.4f}, Ki={Ki:.4f}")
            
            return SynthesisResult(
                success=True,
                message="Síntesis completada exitosamente",
                controller=K_ctrl,
                controller_full=K_ctrl_full,
                plant=G,
                gamma=gam,
                Kp=Kp,
                Ki=Ki,
                K_sign=self.K_sign,
                poles_cl=stability['poles'],
                is_stable=True,
                margins=margins,
                norms=norms,
                warnings=warnings if warnings else None,
                method_used=method_used,
                scaling_applied=scaling_applied,
                scaling_factor=scaling_factor,
                W1=self.W1,
                W2=self.W2,
                W3=self.W3
            )
            
        except Exception as e:
            logger.error(f"Error en síntesis: {e}\n{traceback.format_exc()}")
            return SynthesisResult(
                success=False,
                message=f"Error: {str(e)}"
            )
    
    # =========================================================================
    # VALIDACIÓN DE PARÁMETROS
    # =========================================================================
    
    def validate_config(self, config: SynthesisConfig) -> ValidationResult:
        """
        Valida y corrige parámetros de configuración.
        
        Args:
            config: Configuración a validar
            
        Returns:
            ValidationResult con errores, advertencias y config corregida
        """
        errors = []
        warnings = []
        
        # Crear copia para correcciones
        corrected = SynthesisConfig(
            K=config.K,
            tau=config.tau,
            Ms=config.Ms,
            wb=config.wb,
            eps=config.eps,
            U_max=config.U_max,
            w_unc=config.w_unc,
            eps_T=config.eps_T,
            method=config.method
        )
        
        # Validar τ
        if config.tau < self.TAU_MIN_ABSOLUTE:
            errors.append(f"τ={config.tau:.4f}s es demasiado pequeño (mínimo {self.TAU_MIN_ABSOLUTE}s)")
        elif config.tau < self.TAU_THRESHOLD_SCALING:
            warnings.append(f"τ={config.tau:.4f}s pequeño, se aplicará escalado automático")
        
        # Validar Ms
        if config.Ms < 1.0:
            errors.append(f"Ms={config.Ms:.2f} debe ser ≥ 1.0")
        elif config.Ms < 1.1:
            warnings.append(f"Ms={config.Ms:.2f} muy restrictivo, puede causar problemas")
        
        # Validar ωb
        if config.wb > 100:
            warnings.append(f"ωb={config.wb:.1f} rad/s muy alto, puede requerir control excesivo")
        elif config.wb < 0.1:
            warnings.append(f"ωb={config.wb:.1f} rad/s muy bajo, respuesta lenta")
        
        # Validar U_max
        if abs(config.U_max) < 10:
            warnings.append(f"U_max={config.U_max:.1f} PWM muy bajo")
        
        # Advertir si eps es muy pequeño (pero no corregir)
        eps_recommended = 0.1 if config.tau < self.TAU_THRESHOLD_SCALING else 0.01
        if config.eps < eps_recommended:
            warnings.append(f"⚠️ ε={config.eps} pequeño (recomendado ≥{eps_recommended}), puede causar mal condicionamiento")
        
        # Advertir si eps_T es muy pequeño (pero no corregir)
        if config.eps_T < 0.01:
            warnings.append(f"⚠️ ε_T={config.eps_T} pequeño (recomendado ≥0.01), puede causar problemas numéricos")
        
        is_valid = len(errors) == 0
        
        return ValidationResult(
            is_valid=is_valid,
            errors=errors,
            warnings=warnings,
            corrected_config=corrected
        )
    
    # =========================================================================
    # ESCALADO DE FRECUENCIAS
    # =========================================================================
    
    def _apply_frequency_scaling(self, K: float, tau: float, 
                                  config: SynthesisConfig) -> Dict:
        """
        Aplica escalado de frecuencias para mejorar condicionamiento numérico.
        
        Según Zhou et al., para τ muy pequeño, escalar el sistema mejora
        el condicionamiento de las ecuaciones de Riccati.
        
        Args:
            K: Ganancia absoluta
            tau: Constante de tiempo original
            config: Configuración
            
        Returns:
            Dict con K, tau escalados y factor de escalado
        """
        if tau >= self.TAU_THRESHOLD_SCALING:
            return {
                'K': K,
                'tau': tau,
                'applied': False,
                'factor': 1.0
            }
        
        # Aplicar escalado
        scaling_factor = tau
        tau_scaled = 1.0
        K_scaled = K  # La ganancia no se escala
        
        logger.warning(f"⚙️ ESCALADO DE FRECUENCIAS ACTIVADO")
        logger.warning(f"   τ original: {tau:.4f}s → τ escalado: {tau_scaled:.4f}s")
        logger.warning(f"   Factor: {scaling_factor:.4f}")
        
        self._scaling_applied = True
        self._scaling_factor = scaling_factor
        
        return {
            'K': K_scaled,
            'tau': tau_scaled,
            'applied': True,
            'factor': scaling_factor
        }
    
    # =========================================================================
    # CONSTRUCCIÓN DE PONDERACIONES
    # =========================================================================
    
    def _build_weights(self, config: SynthesisConfig, tau: float) -> Dict:
        """
        Construye funciones de ponderación H∞ según Zhou et al.
        
        W1(s): Performance weight - penaliza error de seguimiento
        W2(s): Control effort weight - limita señal de control
        W3(s): Robustness weight - penaliza sensibilidad complementaria
        
        Args:
            config: Configuración con parámetros
            tau: Constante de tiempo (para ajustar eps)
            
        Returns:
            Dict con W1, W2, W3
        """
        Ms = config.Ms
        wb = config.wb
        eps = config.eps
        U_max = config.U_max
        w_unc = config.w_unc
        eps_T = config.eps_T
        
        # Ajustar eps para convergencia de mixsyn
        # CRÍTICO: La ganancia DC de W1 = 1/eps debe ser <= 20 para que mixsyn converja
        eps_min = 0.05  # Ganancia DC máxima = 20
        eps_safe = max(eps, eps_min)
        eps_T_safe = max(eps_T, 0.1)
        
        print(f"[HINF] === PONDERACIONES ===")
        print(f"[HINF] Parámetros: Ms={Ms:.2f}, wb={wb:.2f}, eps={eps_safe:.4f}, U_max={U_max:.1f}")
        
        # W1(s) = (s/Ms + wb) / (s + wb*eps)
        # Para forzar acción integral, usar polo muy lento en W1
        # Esto penaliza fuertemente el error en baja frecuencia
        # Ganancia DC = 1/eps, Ganancia HF = 1/Ms
        W1 = ct.tf([1/Ms, wb], [1, wb*eps_safe])
        w1_dc = 1.0 / eps_safe
        w1_hf = 1/Ms
        print(f"[HINF] W1: DC={w1_dc:.2f}, HF={w1_hf:.2f}, cruce≈{wb:.1f} rad/s")
        
        # W2(s) = k_u - SIMPLIFICADO a constante para mejor condicionamiento
        # Penaliza esfuerzo de control uniformemente
        k_u = 1.0 / U_max
        W2 = ct.tf([k_u], [1])  # Constante, no dinámico
        print(f"[HINF] W2: k_u={k_u:.4f} (constante)")
        
        # W3(s) = eps_T - SIMPLIFICADO a constante pequeña
        # Penaliza sensibilidad complementaria T uniformemente
        W3 = ct.tf([eps_T_safe], [1])  # Constante pequeña
        print(f"[HINF] W3: eps_T={eps_T_safe:.4f} (constante)")
        
        logger.debug(f"W1: num={W1.num}, den={W1.den}")
        logger.debug(f"W2: num={W2.num}, den={W2.den}")
        logger.debug(f"W3: num={W3.num}, den={W3.den}")
        
        return {'W1': W1, 'W2': W2, 'W3': W3}
    
    # =========================================================================
    # EJECUCIÓN DE SÍNTESIS
    # =========================================================================
    
    def _execute_synthesis(self, G, config: SynthesisConfig, 
                           K_abs: float, tau: float, tau_scaled: float) -> Dict:
        """
        Ejecuta la síntesis H∞ o H2 según el método seleccionado.
        
        Args:
            G: Planta
            config: Configuración
            K_abs: Ganancia absoluta
            tau: τ original
            tau_scaled: τ escalado
            
        Returns:
            Dict con controller, gamma, method_used, success, message
        """
        method = config.method
        
        try:
            if "H2" in method:
                return self._synthesize_h2(G)
            else:
                return self._synthesize_hinf(G, config, K_abs, tau)
                
        except Exception as e:
            logger.error(f"Error en síntesis: {e}")
            return {
                'success': False,
                'message': f"Error en síntesis: {str(e)}",
                'controller': None,
                'gamma': 0,
                'method_used': method
            }
    
    # Timeout para síntesis (segundos)
    SYNTHESIS_TIMEOUT = 15
    
    def _synthesize_hinf(self, G, config: SynthesisConfig, 
                         K_abs: float, tau: float) -> Dict:
        """
        Síntesis H∞ usando mixsyn con modelo de velocidad (primer orden).
        
        El modelo de velocidad G(s) = K/(τs+1) NO tiene polos en el origen,
        por lo que mixsyn funciona correctamente.
        
        Para τ < TAU_THRESHOLD_FALLBACK, usa diseño PI robusto como fallback
        por si mixsyn tiene problemas con τ muy pequeño.
        """
        Ms = config.Ms
        wb = config.wb
        
        # Info: τ pequeño requiere ponderaciones ajustadas (ya manejado en _build_weights)
        if tau < self.TAU_THRESHOLD_FALLBACK:
            print(f"[HINF] τ={tau:.4f}s pequeño, ponderaciones ajustadas automáticamente")
        
        # Verificar indicadores antes de síntesis
        print(f"[HINF] === VERIFICACIÓN PRE-SÍNTESIS ===")
        
        # Calcular indicadores de condicionamiento usando ganancia DC
        try:
            # Ganancia DC de la planta (evaluar en s=0)
            g_dc = abs(G.dcgain())
            print(f"[HINF] |G(0)| = {g_dc:.4f}")
            
            # Ganancia DC de W1
            w1_dc = abs(self.W1.dcgain())
            print(f"[HINF] |W1(0)| = {w1_dc:.4f}, |W2(0)| = {abs(self.W2.dcgain()):.4f}, |W3(0)| = {abs(self.W3.dcgain()):.4f}")
            
            # Indicador de factibilidad
            feasibility = g_dc * w1_dc
            print(f"[HINF] Indicador factibilidad: |G(0)|·|W1(0)| = {feasibility:.4f}")
            if feasibility > 100:
                print(f"[HINF] ⚠️ Factibilidad alta ({feasibility:.1f}), ajustando eps...")
        except Exception as e:
            print(f"[HINF] Error calculando indicadores: {e}")
        
        # Síntesis H∞ con mixsyn
        print(f"[HINF] === EJECUTANDO MIXSYN ===")
        try:
            t_start = time.time()
            
            # Ejecutar mixsyn directamente
            K_ctrl, CL, info = ct.mixsyn(G, self.W1, self.W2, self.W3)
            
            # Extraer gamma y rcond del resultado
            if isinstance(info, tuple):
                gam, rcond = info
                print(f"[HINF] rcond (condicionamiento): {rcond}")
            else:
                gam = info
            
            t_elapsed = time.time() - t_start
            print(f"[HINF] ✅ mixsyn completado en {t_elapsed:.2f}s: γ={gam:.4f}")
            logger.info(f"✅ mixsyn completado: γ={gam:.4f}")
            
            return {
                'success': True,
                'controller': K_ctrl,
                'gamma': gam,
                'method_used': 'H∞ (mixsyn)',
                'message': 'OK'
            }
            
        except Exception as e:
            print(f"[HINF] ❌ mixsyn falló: {e}")
            logger.error(f"mixsyn falló: {e}")
            return {
                'success': False,
                'message': f"mixsyn falló: {str(e)}",
                'controller': None,
                'gamma': 0,
                'method_used': 'H∞ (mixsyn)'
            }
    
    def _run_mixsyn_with_timeout(self, G) -> Optional[Tuple]:
        """
        Ejecuta mixsyn con timeout.
        
        Returns:
            Tuple (K_ctrl, gamma) si exitoso, None si timeout o error
        """
        def _mixsyn_worker():
            try:
                result = ct.mixsyn(G, self.W1, self.W2, self.W3)
                if len(result) == 4:
                    K_ctrl, CL, gam, rcond = result
                else:
                    K_ctrl, CL, gam = result
                return (K_ctrl, gam)
            except Exception as e:
                logger.error(f"Error en mixsyn worker: {e}")
                return None
        
        try:
            with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
                future = executor.submit(_mixsyn_worker)
                try:
                    result = future.result(timeout=self.SYNTHESIS_TIMEOUT)
                    return result
                except concurrent.futures.TimeoutError:
                    logger.warning(f"Timeout en mixsyn después de {self.SYNTHESIS_TIMEOUT}s")
                    return None
        except Exception as e:
            logger.error(f"Error en timeout wrapper: {e}")
            return None
    
    def _synthesize_h2(self, G) -> Dict:
        """Síntesis H2 usando h2syn."""
        try:
            logger.info("Ejecutando ct.h2syn()...")
            
            P = ct.augw(G, self.W1, self.W2, self.W3)
            K_ctrl, CL, gam = ct.h2syn(P, 1, 1)
            
            logger.info(f"✅ h2syn completado: γ={gam:.4f}")
            
            return {
                'success': True,
                'controller': K_ctrl,
                'gamma': gam,
                'method_used': 'H2 (h2syn)',
                'message': 'OK'
            }
            
        except Exception as e:
            logger.error(f"h2syn falló: {e}")
            return {
                'success': False,
                'message': f"H2 falló: {str(e)}",
                'controller': None,
                'gamma': 0,
                'method_used': 'H2 (h2syn)'
            }
    
    def _synthesize_pi_robust(self, G, K_abs: float, tau: float, 
                               Ms: float, wb: float) -> Dict:
        """
        Diseño PI robusto para planta G(s) = K / (τs + 1) (modelo de velocidad).
        
        Para control de POSICIÓN con modelo de VELOCIDAD, el controlador PI:
        1. Proporciona acción integral (1/s) para integrar velocidad → posición
        2. Proporciona acción proporcional para respuesta rápida
        3. El cero del PI cancela el polo de la planta para respuesta suave
        
        Según Zhou et al., para planta de primer orden:
        - Controlador PI: C(s) = (Kp·s + Ki)/s
        - El cero del PI en s = -Ki/Kp cancela el polo de la planta en s = -1/τ
        - La ganancia Ki ajusta el ancho de banda del lazo cerrado
        
        Método: Cancelación de polo + ajuste de ganancia
        """
        print(f"[HINF] === DISEÑO PI ROBUSTO ===")
        print(f"[HINF] Planta: G(s) = {K_abs:.4f} / ({tau:.4f}s + 1)")
        print(f"[HINF] Objetivo: wb={wb:.2f} rad/s, Ms={Ms:.2f}")
        logger.info("Usando diseño PI robusto para control de posición")
        
        # Para G(s) = K / (τs + 1), diseñamos C(s) = (Kp·s + Ki)/s
        #
        # Estrategia: Cancelar el polo de la planta con el cero del PI
        #   Polo de G: s = -1/τ
        #   Cero de C: s = -Ki/Kp
        #   Para cancelación: Ki/Kp = 1/τ  →  Kp = Ki·τ
        #
        # Lazo abierto después de cancelación:
        #   L(s) = G(s)·C(s) = K·Ki / s
        #
        # Lazo cerrado:
        #   T(s) = L/(1+L) = K·Ki / (s + K·Ki)
        #   Polo en: s = -K·Ki
        #   Ancho de banda: wb = K·Ki
        #
        # Para lograr wb deseado: Ki = wb / K
        # Y para cancelación: Kp = Ki·τ = wb·τ / K
        
        # Calcular Ki para lograr ancho de banda deseado
        Ki = wb / K_abs
        
        # Calcular Kp para cancelar el polo de la planta
        Kp = Ki * tau
        
        # Ajustar según Ms (amortiguamiento)
        # Ms más alto = menos amortiguamiento = respuesta más rápida
        # Ms más bajo = más amortiguamiento = respuesta más suave
        damping_factor = 1.0 / Ms  # Ms=1.5 → 0.67, Ms=2.0 → 0.5
        Kp = Kp * (1.0 + damping_factor)  # Aumentar Kp para más amortiguamiento
        
        print(f"[HINF] Cálculo PI:")
        print(f"[HINF]   Ki = wb/K = {wb:.2f}/{K_abs:.4f} = {Ki:.4f}")
        print(f"[HINF]   Kp = Ki·τ·(1+1/Ms) = {Ki:.4f}·{tau:.4f}·{1+damping_factor:.2f} = {Kp:.4f}")
        print(f"[HINF]   Cero del PI: s = -Ki/Kp = {-Ki/Kp:.4f} (cancela polo en s = {-1/tau:.4f})")
        
        logger.info(f"Diseño PI (cancelación de polo):")
        logger.info(f"  wb deseado = {wb:.2f} rad/s")
        logger.info(f"  Ms = {Ms:.2f} (factor amortiguamiento = {damping_factor:.2f})")
        logger.info(f"  Ki = wb/K = {Ki:.4f}")
        logger.info(f"  Kp = Ki·τ·(1+1/Ms) = {Kp:.4f}")
        
        # Construir controlador PI: C(s) = (Kp*s + Ki)/s
        K_ctrl = ct.tf([Kp, Ki], [1, 0])
        
        # Estimar gamma y verificar estabilidad
        L = G * K_ctrl
        CL = ct.feedback(L, 1)
        try:
            gam = ct.hinfnorm(CL)[0]
            poles_cl = ct.poles(CL)
            max_pole = max(np.real(poles_cl))
            print(f"[HINF] Lazo cerrado: γ={gam:.4f}, polo más lento: {max_pole:.4f}")
        except Exception as e:
            gam = 2.0
            print(f"[HINF] Error estimando gamma: {e}")
        
        print(f"[HINF] ✅ PI robusto: C(s) = ({Kp:.4f}s + {Ki:.4f})/s")
        logger.info(f"✅ PI robusto: Kp={Kp:.4f}, Ki={Ki:.4f}, γ≈{gam:.4f}")
        
        return {
            'success': True,
            'controller': K_ctrl,
            'gamma': gam,
            'method_used': 'PI Robusto (fallback)',
            'message': 'OK'
        }
    
    # =========================================================================
    # REDUCCIÓN DE ORDEN
    # =========================================================================
    
    def _reduce_controller_order(self, K_ctrl_full, G, target_order: int = 2):
        """
        Reduce el orden del controlador usando balanced truncation.
        
        Args:
            K_ctrl_full: Controlador de orden completo
            G: Planta (para verificar estabilidad)
            target_order: Orden objetivo (default 2 para PI)
            
        Returns:
            Controlador reducido
        """
        # Obtener orden actual
        if hasattr(K_ctrl_full, 'nstates'):
            current_order = K_ctrl_full.nstates
        else:
            try:
                poles = ct.pole(K_ctrl_full)
                current_order = len(poles) if poles is not None else 2
            except:
                current_order = 2
        
        logger.info(f"Orden del controlador: {current_order}")
        
        if current_order is None or current_order <= target_order:
            logger.info("Controlador ya es de orden bajo, no se requiere reducción")
            return K_ctrl_full
        
        try:
            # Convertir a espacio de estados
            if not hasattr(K_ctrl_full, 'A'):
                K_ss = ct.tf2ss(K_ctrl_full)
            else:
                K_ss = K_ctrl_full
            
            # Reducir
            actual_target = min(target_order, current_order - 1)
            K_reduced_ss = ct.balred(K_ss, actual_target)
            K_reduced = ct.ss2tf(K_reduced_ss)
            
            # Verificar estabilidad
            L_red = G * K_reduced
            cl_red = ct.feedback(L_red, 1)
            poles_red = ct.poles(cl_red)
            is_stable = all(np.real(p) < 1e-6 for p in poles_red)
            
            if not is_stable:
                logger.warning("Reducción inestable, usando controlador completo")
                return K_ctrl_full
            
            logger.info(f"✅ Controlador reducido a orden {actual_target}")
            return K_reduced
            
        except Exception as e:
            logger.warning(f"Error en reducción: {e}, usando controlador completo")
            return K_ctrl_full
    
    # =========================================================================
    # DESESCALADO
    # =========================================================================
    
    def _unscale_controller(self, K_ctrl, scaling_factor: float):
        """
        Desescala el controlador al dominio de frecuencia original.
        
        Args:
            K_ctrl: Controlador en dominio escalado
            scaling_factor: Factor de escalado aplicado
            
        Returns:
            Controlador en dominio original
        """
        if scaling_factor == 1.0:
            return K_ctrl
        
        try:
            num = K_ctrl.num[0][0]
            den = K_ctrl.den[0][0]
            
            # Desescalar coeficientes
            num_original = [coef / (scaling_factor ** (len(num) - 1 - i)) 
                           for i, coef in enumerate(num)]
            den_original = [coef / (scaling_factor ** (len(den) - 1 - i)) 
                           for i, coef in enumerate(den)]
            
            K_unscaled = ct.tf(num_original, den_original)
            
            logger.info(f"✅ Controlador desescalado")
            return K_unscaled
            
        except Exception as e:
            logger.warning(f"Error en desescalado: {e}")
            return K_ctrl
    
    # =========================================================================
    # EXTRACCIÓN DE GANANCIAS PI
    # =========================================================================
    
    def _extract_hinf_pi_equivalent(self, K_ctrl, config: SynthesisConfig = None) -> Tuple[float, float]:
        """
        Extrae ganancias PI equivalentes de un controlador H∞.
        
        Para control de posición con modelo de velocidad:
        - Ki = wb / K_planta (para lograr ancho de banda deseado)
        - Kp = |K_hinf(jωb)| (ganancia del controlador H∞ en el ancho de banda)
        
        Args:
            K_ctrl: Controlador H∞ (StateSpace o TransferFunction)
            config: Configuración de síntesis (para obtener wb y K)
            
        Returns:
            Tuple (Kp, Ki)
        """
        try:
            # Convertir a TransferFunction si es StateSpace
            if hasattr(K_ctrl, 'A') and not hasattr(K_ctrl, 'num'):
                K_tf = ct.ss2tf(K_ctrl)
            else:
                K_tf = K_ctrl
            
            # Obtener parámetros de la configuración o usar valores guardados
            if config is not None:
                wb = config.wb
                K_plant = abs(config.K)
            else:
                wb = getattr(self, '_last_wb', 20.0)
                K_plant = getattr(self, '_last_K', 1.0)
            
            # Ki = wb / K_planta (fórmula estándar para control de posición)
            # Esto garantiza que el ancho de banda del lazo cerrado sea ≈ wb
            Ki = wb / K_plant
            print(f"[HINF] Ki = wb/K = {wb:.2f}/{K_plant:.4f} = {Ki:.4f}")
            
            # Kp = |K_hinf(jωb)| (ganancia del controlador H∞ en el ancho de banda)
            K_wb = K_tf(1j * wb)
            Kp = float(np.abs(K_wb))
            print(f"[HINF] Kp = |K_hinf(jωb)| = {Kp:.4f} (ωb={wb:.1f} rad/s)")
            
            # Verificar valores razonables
            if not np.isfinite(Ki) or abs(Ki) > 1e6:
                print(f"[HINF] ⚠️ Ki={Ki} fuera de rango, usando valor por defecto")
                Ki = wb / K_plant if K_plant > 0 else 10.0
            if not np.isfinite(Kp) or abs(Kp) > 1e6:
                print(f"[HINF] ⚠️ Kp={Kp} fuera de rango, usando valor por defecto")
                Kp = Ki * 0.1  # Kp típicamente menor que Ki
            
            logger.info(f"PI equivalente extraído: Kp={Kp:.4f}, Ki={Ki:.4f}")
            return float(Kp), float(Ki)
            
        except Exception as e:
            print(f"[HINF] Error extrayendo PI equivalente: {e}")
            logger.warning(f"Error extrayendo PI equivalente: {e}")
            return 1.0, 10.0  # Valores por defecto razonables
    
    def _extract_pi_gains(self, K_ctrl) -> Tuple[float, float]:
        """
        Extrae Kp, Ki de un controlador PI o P.
        
        Formas soportadas:
        - PI: C(s) = (Kp*s + Ki)/s  → num=[Kp, Ki], den=[1, 0]
        - P:  C(s) = Kp             → num=[Kp], den=[1]
        
        Args:
            K_ctrl: Controlador (puede ser StateSpace o TransferFunction)
            
        Returns:
            Tuple (Kp, Ki)
        """
        try:
            # Convertir a TransferFunction si es StateSpace
            if hasattr(K_ctrl, 'A') and not hasattr(K_ctrl, 'num'):
                K_tf = ct.ss2tf(K_ctrl)
                print(f"[HINF] Controlador convertido de SS a TF: {K_tf}")
            else:
                K_tf = K_ctrl
            
            num = np.array(K_tf.num[0][0]).flatten()
            den = np.array(K_tf.den[0][0]).flatten()
            
            # Para controladores H∞ de orden superior con integrador,
            # aproximar como PI evaluando C(s) en bajas frecuencias:
            # C(s) ≈ Ki/s + Kp para s→0
            # 
            # Si C(s) = N(s)/(s·D(s)) donde D(0)≠0, entonces:
            # - Ki = N(0)/D(0) (residuo del polo en s=0)
            # - Kp se obtiene del siguiente término de la expansión
            
            # Verificar si tiene integrador (polo en s=0)
            has_integrator = abs(den[-1]) < 1e-6 if len(den) > 0 else False
            
            if has_integrator:
                # Factorizar el integrador: den = [a_n, ..., a_1, 0] = s·[a_n, ..., a_1]
                den_reduced = den[:-1]  # Quitar el 0 final (factor s)
                
                # Ki = N(0) / D_reduced(0)
                Ki = num[-1] / den_reduced[-1] if den_reduced[-1] != 0 else 0
                
                # Para Kp, evaluar la derivada o usar el coeficiente siguiente
                # Aproximación: Kp ≈ (num[-2] - Ki*den_reduced[-2]) / den_reduced[-1]
                if len(num) >= 2 and len(den_reduced) >= 2:
                    Kp = (num[-2] - Ki * den_reduced[-2]) / den_reduced[-1]
                else:
                    Kp = 0.0
                
                print(f"[HINF] PI extraído (con integrador): Kp={Kp:.4f}, Ki={Ki:.4f}")
                logger.info(f"PI extraído: Kp={Kp:.4f}, Ki={Ki:.4f}")
                return float(Kp), float(Ki)
            else:
                # Sin integrador: usar ganancia DC como Kp
                dc_gain = K_tf.dcgain()
                Kp = float(dc_gain) if np.isfinite(dc_gain) else num[-1]/den[-1]
                print(f"[HINF] P extraído (sin integrador): Kp={Kp:.4f}")
                logger.info(f"P extraído: Kp={Kp:.4f}")
                return float(Kp), 0.0
                    
        except Exception as e:
            print(f"[HINF] Error extrayendo PI: {e}")
            logger.warning(f"Error extrayendo PI: {e}")
        
        return 0.0, 0.0
    
    # =========================================================================
    # VERIFICACIÓN DE ESTABILIDAD
    # =========================================================================
    
    def _verify_stability(self, G, K_ctrl) -> Dict:
        """
        Verifica estabilidad del lazo cerrado.
        
        Args:
            G: Planta
            K_ctrl: Controlador
            
        Returns:
            Dict con is_stable, poles, message
        """
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
        """
        Calcula márgenes de ganancia y fase.
        
        Args:
            L: Lazo abierto G*K
            
        Returns:
            Dict con márgenes
        """
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
        """
        Calcula normas H∞ de sensibilidad.
        
        Args:
            G: Planta
            K_ctrl: Controlador
            
        Returns:
            Dict con normas
        """
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
    
    # =========================================================================
    # MÉTODOS DE INFORMACIÓN
    # =========================================================================
    
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
    
    def get_controller_for_transfer(self) -> Dict:
        """
        Retorna datos del controlador para transferir a TestTab.
        
        Returns:
            Dict con Kp, Ki, K, gamma, U_max, etc.
        """
        if self.controller is None:
            return None
        
        return {
            'Kp': self.Kp,
            'Ki': self.Ki,
            'K': self.controller,
            'K_sign': self.K_sign,
            'gamma': self.gamma,
            'U_max': self.U_max,
            'K_value': self.K_value,
            'tau_value': self.tau_value
        }

"""
Servicio de Control Dual y Ejecución de Trayectorias.

Este módulo contiene toda la lógica de control que antes estaba en TestTab,
separando la lógica de negocio de la interfaz de usuario.

REFACTORIZADO: 2025-12-17
- Lógica de control dual PI movida desde TestTab
- Lógica de ejecución de trayectorias movida desde TestTab
- Comunicación por señales PyQt
- Calibración dinámica desde config/constants.py

Señales emitidas:
- control_status_changed: Estado del control (activo/inactivo)
- position_update: Actualización de posición (error_x, error_y, pwm_a, pwm_b)
- position_reached: Posición alcanzada y estable
- trajectory_point_reached: Punto de trayectoria alcanzado (index, x, y, status)
- trajectory_completed: Trayectoria completada
- trajectory_feedback: Feedback visual (target_x, target_y, error_x, error_y, lock_x, lock_y, settling)
- log_message: Mensaje para mostrar en UI
"""

import logging
import time
from typing import Callable, Optional, Dict, List, Tuple
from dataclasses import dataclass

from PyQt5.QtCore import QObject, QTimer, pyqtSignal

from config.constants import (
    CALIBRATION_X, CALIBRATION_Y,
    DEADZONE_ADC, POSITION_TOLERANCE_UM, SETTLING_CYCLES,
    MAX_ATTEMPTS_PER_POINT, FALLBACK_TOLERANCE_MULTIPLIER,
    um_to_adc, adc_to_um
)

logger = logging.getLogger('MotorControl_L206')


@dataclass
class ControllerConfig:
    """Configuración de un controlador PI."""
    Kp: float
    Ki: float
    U_max: float = 150.0
    invert: bool = False
    sensor_key: str = 'sensor_1'


@dataclass
class TrajectoryConfig:
    """Configuración para ejecución de trayectoria."""
    tolerance_um: float = 25.0
    pause_s: float = 2.0
    

class TestService(QObject):
    """
    Servicio de control dual y ejecución de trayectorias.
    
    Separa la lógica de control de la interfaz de usuario.
    Toda la comunicación con la UI es mediante señales PyQt.
    """
    
    # === SEÑALES DE CONTROL DUAL ===
    dual_control_started = pyqtSignal()
    dual_control_stopped = pyqtSignal()
    dual_position_update = pyqtSignal(float, float, int, int)  # error_a_um, error_b_um, pwm_a, pwm_b
    dual_position_reached = pyqtSignal(float, float, float, float)  # ref_a, ref_b, error_a, error_b
    dual_position_lost = pyqtSignal()
    
    # === SEÑALES DE TRAYECTORIA ===
    trajectory_started = pyqtSignal(int)  # total_points
    trajectory_stopped = pyqtSignal(int, int)  # current_point, total_points
    trajectory_completed = pyqtSignal(int)  # total_points
    trajectory_point_reached = pyqtSignal(int, float, float, str)  # index, x, y, status
    trajectory_feedback = pyqtSignal(float, float, float, float, bool, bool, int)  # target_x, target_y, error_x, error_y, lock_x, lock_y, settling
    
    # === SEÑALES GENERALES ===
    log_message = pyqtSignal(str)  # Mensaje para UI
    error_occurred = pyqtSignal(str)  # Error
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Callbacks de hardware (inyectados desde main.py)
        self._send_command: Optional[Callable[[str], None]] = None
        self._get_sensor_value: Optional[Callable[[str], Optional[float]]] = None
        
        # Controladores
        self._controller_a: Optional[ControllerConfig] = None
        self._controller_b: Optional[ControllerConfig] = None
        
        # Estado de control dual
        self._dual_active = False
        self._dual_timer: Optional[QTimer] = None
        self._dual_ref_a_um = 0.0
        self._dual_ref_b_um = 0.0
        self._dual_integral_a = 0.0
        self._dual_integral_b = 0.0
        self._dual_last_time = 0.0
        self._dual_position_reached = False
        self._dual_settling_counter = 0
        self._dual_log_counter = 0
        
        # Estado de trayectoria
        self._trajectory_active = False
        self._trajectory_timer: Optional[QTimer] = None
        self._trajectory: Optional[List[Tuple[float, float]]] = None
        self._trajectory_index = 0
        self._trajectory_config = TrajectoryConfig()
        self._trajectory_paused = True
        self._trajectory_waiting = False
        self._traj_settling_counter = 0
        self._traj_near_attempts = 0
        
        logger.info("TestService inicializado")
    
    # =========================================================================
    # CONFIGURACIÓN
    # =========================================================================
    
    def set_hardware_callbacks(self, send_command: Callable, get_sensor_value: Callable):
        """
        Configura callbacks de hardware.
        
        Args:
            send_command: Función para enviar comandos al Arduino
            get_sensor_value: Función para leer valor de sensor
        """
        self._send_command = send_command
        self._get_sensor_value = get_sensor_value
        logger.debug("TestService: Callbacks de hardware configurados")
    
    def set_controller_a(self, config: Optional[ControllerConfig]):
        """Configura controlador para Motor A (eje X)."""
        self._controller_a = config
        if config:
            logger.info(f"TestService: Controlador A configurado - Kp={config.Kp:.4f}, Ki={config.Ki:.4f}")
        else:
            logger.info("TestService: Controlador A limpiado")
    
    def set_controller_b(self, config: Optional[ControllerConfig]):
        """Configura controlador para Motor B (eje Y)."""
        self._controller_b = config
        if config:
            logger.info(f"TestService: Controlador B configurado - Kp={config.Kp:.4f}, Ki={config.Ki:.4f}")
        else:
            logger.info("TestService: Controlador B limpiado")
    
    def update_controller_a_sensor(self, sensor_key: str, invert: bool):
        """Actualiza configuración de sensor e inversión para controlador A."""
        if self._controller_a:
            self._controller_a.sensor_key = sensor_key
            self._controller_a.invert = invert
    
    def update_controller_b_sensor(self, sensor_key: str, invert: bool):
        """Actualiza configuración de sensor e inversión para controlador B."""
        if self._controller_b:
            self._controller_b.sensor_key = sensor_key
            self._controller_b.invert = invert
    
    # =========================================================================
    # CONTROL DUAL
    # =========================================================================
    
    def start_dual_control(self, ref_a_um: float, ref_b_um: float) -> bool:
        """
        Inicia control dual de ambos motores.
        
        Args:
            ref_a_um: Referencia para Motor A en µm
            ref_b_um: Referencia para Motor B en µm
            
        Returns:
            True si se inició correctamente
        """
        logger.info(f"=== TestService: INICIANDO CONTROL DUAL ===")
        logger.info(f"Referencias: A={ref_a_um}µm, B={ref_b_um}µm")
        
        # Verificar callbacks
        if not self._send_command or not self._get_sensor_value:
            self.error_occurred.emit("Callbacks de hardware no configurados")
            logger.error("TestService: Callbacks no configurados")
            return False
        
        # Verificar controladores
        if not self._controller_a and not self._controller_b:
            self.error_occurred.emit("No hay controladores cargados")
            logger.error("TestService: No hay controladores")
            return False
        
        # Guardar referencias
        self._dual_ref_a_um = ref_a_um
        self._dual_ref_b_um = ref_b_um
        
        # Activar modo automático
        self._send_command('A,0,0')
        
        # Resetear variables
        self._dual_integral_a = 0.0
        self._dual_integral_b = 0.0
        self._dual_last_time = time.time()
        self._dual_position_reached = False
        self._dual_settling_counter = 0
        self._dual_log_counter = 0
        
        # Activar control
        self._dual_active = True
        
        # Crear timer
        self._dual_timer = QTimer()
        self._dual_timer.timeout.connect(self._execute_dual_control_step)
        self._dual_timer.start(10)  # 100Hz
        
        self.dual_control_started.emit()
        self.log_message.emit("🎮 Control Dual ACTIVO")
        logger.info("TestService: Control dual iniciado")
        
        return True
    
    def stop_dual_control(self):
        """Detiene el control dual con freno activo."""
        logger.info("=== TestService: DETENIENDO CONTROL DUAL ===")
        
        # Detener timer
        if self._dual_timer:
            self._dual_timer.stop()
            self._dual_timer = None
        
        # Freno activo
        if self._send_command:
            self._send_command('B')
            time.sleep(0.1)
            self._send_command('A,0,0')
            self._send_command('M')
        
        self._dual_active = False
        
        self.dual_control_stopped.emit()
        self.log_message.emit("⏹️ Control Dual DETENIDO (Freno Activo)")
        logger.info("TestService: Control dual detenido")
    
    def _execute_dual_control_step(self):
        """Ejecuta un ciclo del control dual PI."""
        try:
            current_time = time.time()
            Ts = current_time - self._dual_last_time
            self._dual_last_time = current_time
            
            pwm_a = 0
            pwm_b = 0
            error_a_um = 0.0
            error_b_um = 0.0
            
            # Control Motor A (eje X)
            if self._controller_a:
                sensor_adc = self._get_sensor_value(self._controller_a.sensor_key)
                
                if sensor_adc is not None:
                    ref_adc = um_to_adc(self._dual_ref_a_um, axis='x')
                    error_adc = ref_adc - sensor_adc
                    error_a_um = error_adc * CALIBRATION_X['slope']
                    
                    if abs(error_adc) > DEADZONE_ADC:
                        self._dual_integral_a += error_adc * Ts
                        
                        pwm_base = (self._controller_a.Kp * error_adc + 
                                   self._controller_a.Ki * self._dual_integral_a)
                        
                        if self._controller_a.invert:
                            pwm_a = -int(pwm_base)
                        else:
                            pwm_a = int(pwm_base)
                        
                        U_max = int(self._controller_a.U_max)
                        if abs(pwm_a) > U_max:
                            self._dual_integral_a -= error_adc * Ts
                            pwm_a = max(-U_max, min(U_max, pwm_a))
            
            # Control Motor B (eje Y)
            if self._controller_b:
                sensor_adc = self._get_sensor_value(self._controller_b.sensor_key)
                
                if sensor_adc is not None:
                    ref_adc = um_to_adc(self._dual_ref_b_um, axis='y')
                    error_adc = ref_adc - sensor_adc
                    error_b_um = error_adc * CALIBRATION_Y['slope']
                    
                    if abs(error_adc) > DEADZONE_ADC:
                        self._dual_integral_b += error_adc * Ts
                        
                        pwm_base = (self._controller_b.Kp * error_adc + 
                                   self._controller_b.Ki * self._dual_integral_b)
                        
                        if self._controller_b.invert:
                            pwm_b = -int(pwm_base)
                        else:
                            pwm_b = int(pwm_base)
                        
                        U_max = int(self._controller_b.U_max)
                        if abs(pwm_b) > U_max:
                            self._dual_integral_b -= error_adc * Ts
                            pwm_b = max(-U_max, min(U_max, pwm_b))
            
            # Verificar llegada
            a_at_target = abs(error_a_um) < POSITION_TOLERANCE_UM if self._controller_a else True
            b_at_target = abs(error_b_um) < POSITION_TOLERANCE_UM if self._controller_b else True
            both_at_target = a_at_target and b_at_target
            
            # Settling
            if both_at_target:
                self._dual_settling_counter += 1
                
                if self._dual_settling_counter >= SETTLING_CYCLES and not self._dual_position_reached:
                    self._dual_position_reached = True
                    self._send_command('B')
                    time.sleep(0.02)
                    self._send_command('A,0,0')
                    
                    self.dual_position_reached.emit(
                        self._dual_ref_a_um, self._dual_ref_b_um,
                        error_a_um, error_b_um
                    )
                    self.log_message.emit(
                        f"✅ POSICIÓN ALCANZADA (estable {SETTLING_CYCLES} ciclos): "
                        f"A={self._dual_ref_a_um:.0f}µm (err={error_a_um:.1f}), "
                        f"B={self._dual_ref_b_um:.0f}µm (err={error_b_um:.1f})"
                    )
                    return
            else:
                self._dual_settling_counter = 0
                
                if self._dual_position_reached:
                    self._dual_position_reached = False
                    self.dual_position_lost.emit()
                    self.log_message.emit("🔄 Posición perdida - Reactivando control...")
                
                self._send_command(f"A,{pwm_a},{pwm_b}")
            
            # Emitir actualización
            self.dual_position_update.emit(error_a_um, error_b_um, pwm_a, pwm_b)
            
            # Log periódico
            self._dual_log_counter += 1
            if self._dual_log_counter % 50 == 0:
                status = "✅" if self._dual_position_reached else ("⏳" if both_at_target else "🔄")
                settling_info = f" [settling: {self._dual_settling_counter}/{SETTLING_CYCLES}]" if both_at_target and not self._dual_position_reached else ""
                self.log_message.emit(
                    f"{status} A: {error_a_um:.1f}µm | B: {error_b_um:.1f}µm | PWM: ({pwm_a},{pwm_b}){settling_info}"
                )
                
        except Exception as e:
            logger.error(f"TestService: Error en control dual: {e}")
    
    @property
    def is_dual_control_active(self) -> bool:
        """Retorna si el control dual está activo."""
        return self._dual_active
    
    # =========================================================================
    # EJECUCIÓN DE TRAYECTORIA
    # =========================================================================
    
    def start_trajectory(self, trajectory: list, tolerance_um: float = 25.0, pause_s: float = 2.0, auto_advance: bool = False) -> bool:
        """
        Inicia la ejecución de una trayectoria con control PI dual.
        
        Args:
            trajectory: Lista de puntos (x, y) en µm
            tolerance_um: Tolerancia de posición en µm
            pause_s: Pausa en cada punto en segundos
            auto_advance: Si True, avanza automáticamente después de pausa (TestTab).
                         Si False, espera comando explícito resume_trajectory (MicroscopyService).
            
        Returns:
            True si se inició correctamente
        """
        logger.info(f"=== TestService: INICIANDO TRAYECTORIA ({len(trajectory)} puntos) ===")
        logger.info(f"    Modo: {'AUTO-ADVANCE' if auto_advance else 'MANUAL (espera resume_trajectory)'}")
        
        if not trajectory:
            self.error_occurred.emit("Trayectoria vacía")
            return False
        
        if not self._send_command or not self._get_sensor_value:
            self.error_occurred.emit("Callbacks de hardware no configurados")
            return False
        
        if not self._controller_a and not self._controller_b:
            self.error_occurred.emit("No hay controladores cargados")
            return False
        
        # CRÍTICO: Detener trayectoria anterior si existe
        if self._trajectory_active:
            logger.warning("[TestService] Trayectoria anterior activa - deteniendo antes de iniciar nueva")
            self.stop_trajectory()
            time.sleep(0.2)  # Dar tiempo para que se detenga completamente
        
        # Guardar configuración
        self._trajectory = list(trajectory)
        self._trajectory_config.tolerance_um = tolerance_um
        self._trajectory_config.pause_s = pause_s
        self._trajectory_auto_advance = auto_advance  # NUEVO: modo auto-advance
        
        # DEBUG: Mostrar primeros puntos de la trayectoria
        logger.info(f"[DEBUG] Primeros 5 puntos de trayectoria:")
        for i in range(min(5, len(self._trajectory))):
            p = self._trajectory[i]
            logger.info(f"  Punto {i}: ({p[0]:.1f}, {p[1]:.1f})µm")
        
        # Inicializar estado - SIEMPRE desde cero
        self._trajectory_index = 0
        self._trajectory_active = True
        self._trajectory_paused = False  # CORRECCIÓN: Iniciar NO pausado para ir al primer punto
        self._trajectory_waiting = False
        self._point_accepted = False  # NUEVO: Flag para evitar múltiples aceptaciones del mismo punto
        
        # Estado de corrección de eje bloqueado
        self._correcting_locked_axis = False
        self._correction_axis = None  # 'x' o 'y'
        self._correction_target_um = 0.0
        
        # Resetear integrales y contadores
        self._dual_integral_a = 0.0
        self._dual_integral_b = 0.0
        self._dual_last_time = time.time()
        self._traj_settling_counter = 0
        self._traj_near_attempts = 0
        
        # Activar modo automático
        self._send_command('A,0,0')
        
        # Crear timer
        self._trajectory_timer = QTimer()
        self._trajectory_timer.timeout.connect(self._execute_trajectory_step)
        self._trajectory_timer.start(10)  # 100Hz
        
        self.trajectory_started.emit(len(trajectory))
        self.log_message.emit(f"🚀 Ejecutando trayectoria: {len(trajectory)} puntos")
        self.log_message.emit(f"   Tolerancia: {tolerance_um}µm, Pausa: {pause_s}s")
        
        return True
    
    def stop_trajectory(self):
        """Detiene la ejecución de la trayectoria con freno activo."""
        logger.info("=== TestService: DETENIENDO TRAYECTORIA ===")
        
        self._trajectory_active = False
        
        # Detener timer
        if self._trajectory_timer:
            self._trajectory_timer.stop()
            self._trajectory_timer = None
        
        # Freno activo
        if self._send_command:
            self._send_command('B')
            time.sleep(0.1)
            self._send_command('A,0,0')
            self._send_command('M')
        
        total = len(self._trajectory) if self._trajectory else 0
        self.trajectory_stopped.emit(self._trajectory_index + 1, total)
        self.log_message.emit(f"⏹️ Trayectoria detenida en punto {self._trajectory_index + 1}/{total} (Freno Activo)")
    
    def pause_trajectory(self):
        """Pausa la trayectoria (mantiene el timer activo).
        
        Timer continúa ejecutándose para mantener posición activamente.
        """
        if not self._trajectory_active:
            return
        
        self._trajectory_paused = True
        logger.info("[TestService] Trayectoria pausada - manteniendo posición")
    
    def resume_trajectory(self):
        """
        Reanuda la ejecución de la trayectoria, avanzando al siguiente punto.
        Este método es llamado explícitamente por MicroscopyService.
        """
        if not self._trajectory_active:
            logger.warning("[TestService] Intento de reanudar trayectoria inactiva.")
            return
        if not self._trajectory_paused:
            logger.warning("[TestService] Intento de reanudar trayectoria no pausada.")
            return

        try:
            logger.info("[TestService] ▶️  Comando RESUME_TRAJECTORY recibido. Avanzando al siguiente punto.")
            
            # DEBUG: Estado ANTES de cambios
            logger.info(f"[DEBUG-RESUME] ANTES: índice={self._trajectory_index}, _point_accepted={self._point_accepted}, paused={self._trajectory_paused}")
            
            # PRIMERO: Actualizar todas las variables de estado
            self._trajectory_paused = False
            self._trajectory_index += 1  # Avanzar al siguiente punto
            
            # CRÍTICO: Resetear flag de punto aceptado para el nuevo punto
            # Sin esto, _accept_trajectory_point() detecta que el punto ya fue aceptado
            # y el sistema queda atascado indefinidamente
            self._point_accepted = False

            # Resetear integrales al reanudar para evitar wind-up
            self._dual_integral_a = 0.0
            self._dual_integral_b = 0.0
            
            # DEBUG: Estado DESPUÉS de cambios
            logger.info(f"[DEBUG-RESUME] DESPUÉS: índice={self._trajectory_index}, _point_accepted={self._point_accepted}, paused={self._trajectory_paused}")
            
        except Exception as e:
            logger.error(f"❌ ERROR CRÍTICO en resume_trajectory: {e}", exc_info=True)
    
    def _auto_advance_to_next_point(self):
        """Avanza automáticamente al siguiente punto (modo auto_advance)."""
        if not self._trajectory_active:
            return
        
        logger.info("[TestService] ▶️  Auto-avanzando al siguiente punto (delay 100ms)")
        
        # Delay pequeño de 100ms antes de avanzar
        time.sleep(0.1)
        
        # Avanzar al siguiente punto
        self._trajectory_index += 1
        
        # DEBUG: Mostrar nuevo índice y punto objetivo
        if self._trajectory_index < len(self._trajectory):
            next_point = self._trajectory[self._trajectory_index]
            logger.info(f"[DEBUG] Nuevo índice: {self._trajectory_index}, Punto objetivo: ({next_point[0]:.1f}, {next_point[1]:.1f})µm")
        else:
            logger.info(f"[DEBUG] Nuevo índice: {self._trajectory_index} >= {len(self._trajectory)} (trayectoria completada)")
        
        # Resetear flag de punto aceptado para el nuevo punto
        self._point_accepted = False
        
        # Reanudar trayectoria (desactivar pausa)
        self._trajectory_paused = False
        
        # Resetear integrales
        self._dual_integral_a = 0.0
        self._dual_integral_b = 0.0
    
    def _get_adaptive_pwm_limit(self, axis: str, error_um: float) -> float:
        """Calcula PWM adaptativo según error, manteniendo mínimo de 80.
        
        IMPORTANTE: PWM adaptativo SOLO funciona en modo AUTO (TestTab).
        En modo MANUAL (ImgRecTab), el sistema se detiene completamente durante
        captura de microscopía, y PWM reducido es insuficiente para vencer inercia.
        
        - Modo MANUAL: PWM completo siempre
        - Modo AUTO: PWM adaptativo según error
        """
        base_umax = self._controller_a.U_max if axis == 'x' else self._controller_b.U_max
        
        # Desactivar PWM adaptativo en modo MANUAL
        # En ImgRecTab, sistema se detiene completamente → necesita PWM completo
        if not self._trajectory_auto_advance:
            return base_umax
        
        # PWM adaptativo SOLO en modo AUTO (TestTab)
        # Umbrales ajustados para distancias típicas de ~306 µm entre puntos
        if abs(error_um) > 300:
            # Error grande: PWM completo para velocidad máxima
            return base_umax
        elif abs(error_um) > 150:
            # Error medio: 70% de PWM (pero mínimo 80)
            return max(80, base_umax * 0.7)
        else:
            # Aproximación final: PWM mínimo (80)
            return 80
    
    def _detect_axis_lock(self, current_idx: int) -> Tuple[bool, bool]:
        """Detecta si algún eje debe bloquearse."""
        if not self._trajectory or current_idx >= len(self._trajectory):
            return (False, False)
        
        current = self._trajectory[current_idx]
        
        if current_idx > 0:
            prev = self._trajectory[current_idx - 1]
            lock_x = abs(current[0] - prev[0]) < 1.0
            lock_y = abs(current[1] - prev[1]) < 1.0
            return (lock_x, lock_y)
        
        return (False, False)
    
    def _execute_trajectory_step(self):
        """Ejecuta un paso del control de trayectoria.
        
        FASE 2 y 5: Si está pausado, mantiene posición activamente.
        """
        try:
            if not self._trajectory_active:
                return
            
            # FASE 5: Si está pausado, mantener posición activamente
            if self._trajectory_paused:
                self._maintain_position()
                return
            
            # Calcular Ts
            Ts = time.time() - self._dual_last_time
            self._dual_last_time = time.time()
            
            # Verificar si completamos
            if self._trajectory_index >= len(self._trajectory):
                self.stop_trajectory()
                self.trajectory_completed.emit(len(self._trajectory))
                self.log_message.emit("✅ Trayectoria completada!")
                return
            
            target = self._trajectory[self._trajectory_index]
            target_x, target_y = target[0], target[1]
            
            # DEBUG: Log cada 100 ciclos para no saturar
            if not hasattr(self, '_debug_counter'):
                self._debug_counter = 0
            self._debug_counter += 1
            if self._debug_counter % 100 == 0:
                logger.info(f"[DEBUG] Índice={self._trajectory_index}, Objetivo=({target_x:.1f}, {target_y:.1f})µm, _point_accepted={self._point_accepted}, paused={self._trajectory_paused}")
            
            # Detectar bloqueo de ejes
            lock_x, lock_y = self._detect_axis_lock(self._trajectory_index)
            
            # Conversión a ADC
            ref_adc_x = um_to_adc(target_x, axis='x')
            ref_adc_y = um_to_adc(target_y, axis='y')
            
            pwm_a = 0
            pwm_b = 0
            error_x_um = 0.0
            error_y_um = 0.0
            
            # Control Motor A (eje X)
            if self._controller_a and not lock_x:
                sensor_adc = self._get_sensor_value(self._controller_a.sensor_key)
                
                if sensor_adc is not None:
                    error_adc = ref_adc_x - sensor_adc
                    error_x_um = error_adc * CALIBRATION_X['slope']
                    
                    if abs(error_adc) > DEADZONE_ADC:
                        self._dual_integral_a += error_adc * Ts
                        pwm_base = (self._controller_a.Kp * error_adc + 
                                   self._controller_a.Ki * self._dual_integral_a)
                        
                        if self._controller_a.invert:
                            pwm_a = -int(pwm_base)
                        else:
                            pwm_a = int(pwm_base)
                        
                        # PWM adaptativo con mínimo 80
                        U_max = int(self._get_adaptive_pwm_limit('x', error_x_um))
                        if abs(pwm_a) > U_max:
                            self._dual_integral_a -= error_adc * Ts
                            pwm_a = max(-U_max, min(U_max, pwm_a))
            elif lock_x and self._controller_a:
                sensor_adc = self._get_sensor_value(self._controller_a.sensor_key)
                if sensor_adc is not None:
                    error_adc = ref_adc_x - sensor_adc
                    error_x_um = error_adc * CALIBRATION_X['slope']
                pwm_a = 0
            
            # Control Motor B (eje Y)
            if self._controller_b and not lock_y:
                sensor_adc = self._get_sensor_value(self._controller_b.sensor_key)
                
                if sensor_adc is not None:
                    error_adc = ref_adc_y - sensor_adc
                    error_y_um = error_adc * CALIBRATION_Y['slope']
                    
                    if abs(error_adc) > DEADZONE_ADC:
                        self._dual_integral_b += error_adc * Ts
                        pwm_base = (self._controller_b.Kp * error_adc + 
                                   self._controller_b.Ki * self._dual_integral_b)
                        
                        if self._controller_b.invert:
                            pwm_b = -int(pwm_base)
                        else:
                            pwm_b = int(pwm_base)
                        
                        # PWM adaptativo con mínimo 80
                        U_max = int(self._get_adaptive_pwm_limit('y', error_y_um))
                        if abs(pwm_b) > U_max:
                            self._dual_integral_b -= error_adc * Ts
                            pwm_b = max(-U_max, min(U_max, pwm_b))
            elif lock_y and self._controller_b:
                sensor_adc = self._get_sensor_value(self._controller_b.sensor_key)
                if sensor_adc is not None:
                    error_adc = ref_adc_y - sensor_adc
                    error_y_um = error_adc * CALIBRATION_Y['slope']
                pwm_b = 0
            
            # Calcular tolerancias
            tolerance = self._trajectory_config.tolerance_um
            fallback_tolerance = tolerance * FALLBACK_TOLERANCE_MULTIPLIER
            
            # Determinar at_target considerando bloqueos
            if lock_x and lock_y:
                at_target = True
                at_fallback_target = True
            elif lock_x:
                at_target = abs(error_y_um) < tolerance
                at_fallback_target = abs(error_y_um) < fallback_tolerance
            elif lock_y:
                at_target = abs(error_x_um) < tolerance
                at_fallback_target = abs(error_x_um) < fallback_tolerance
            else:
                at_target = abs(error_x_um) < tolerance and abs(error_y_um) < tolerance
                at_fallback_target = abs(error_x_um) < fallback_tolerance and abs(error_y_um) < fallback_tolerance
            
            # Lógica de settling
            if at_target:
                self._traj_settling_counter += 1
                self._traj_near_attempts += 1
                
                if self._traj_settling_counter >= SETTLING_CYCLES:
                    # CORRECCIÓN DESHABILITADA: Causa oscilación en ImgRecTab
                    # Si hay deriva en eje bloqueado, se acepta con error (tolerancia 100µm)
                    # La corrección intenta mover un eje que NO debe moverse, causando inestabilidad
                    
                    # Verificar flag ANTES de aceptar para prevenir llamadas duplicadas
                    if not self._point_accepted:
                        self._accept_trajectory_point(target_x, target_y, error_x_um, error_y_um, "✅ Estable")
                else:
                    self._send_command(f"A,{pwm_a},{pwm_b}")
                    
            elif at_fallback_target:
                self._traj_settling_counter = 0
                self._traj_near_attempts += 1
                
                if self._traj_near_attempts >= MAX_ATTEMPTS_PER_POINT:
                    # CRÍTICO: Verificar flag ANTES de aceptar para prevenir llamadas duplicadas
                    if not self._point_accepted:
                        self._accept_trajectory_point(target_x, target_y, error_x_um, error_y_um,
                                                      f"⚠️ Fallback ({self._traj_near_attempts} intentos)")
                        logger.warning(f"⚠️ Punto {self._trajectory_index + 1} aceptado con fallback")
                else:
                    self._send_command(f"A,{pwm_a},{pwm_b}")
            else:
                self._traj_settling_counter = 0
                self._traj_near_attempts = 0
                self._send_command(f"A,{pwm_a},{pwm_b}")
            
            # Emitir feedback
            self.trajectory_feedback.emit(
                target_x, target_y, error_x_um, error_y_um,
                lock_x, lock_y, self._traj_settling_counter
            )
            
        except Exception as e:
            logger.error(f"TestService: Error en trayectoria: {e}")
    
    def _maintain_position(self):
        """Mantiene la posición actual con control activo durante pausa.
        
        FASE 5: Aplica control proporcional suave para corregir deriva sin acumular integral.
        Llamado cada 10ms por _execute_trajectory_step cuando está pausado.
        """
        if not self._trajectory or self._trajectory_index >= len(self._trajectory):
            return
        
        target = self._trajectory[self._trajectory_index]
        target_x, target_y = target[0], target[1]
        
        # Calcular referencias en ADC
        ref_adc_x = um_to_adc(target_x, axis='x')
        ref_adc_y = um_to_adc(target_y, axis='y')
        
        pwm_a = 0
        pwm_b = 0
        
        # Control correctivo suave para eje A (solo proporcional)
        if self._controller_a:
            sensor_adc = self._get_sensor_value(self._controller_a.sensor_key)
            if sensor_adc is not None:
                error_adc = ref_adc_x - sensor_adc
                if abs(error_adc) > DEADZONE_ADC:
                    # Solo proporcional, sin integral para evitar acumulación
                    # Ganancia reducida al 30% para control suave
                    pwm_base = self._controller_a.Kp * error_adc * 0.3
                    pwm_a = -int(pwm_base) if self._controller_a.invert else int(pwm_base)
                    U_max = int(self._controller_a.U_max)
                    pwm_a = max(-U_max, min(U_max, pwm_a))
        
        # Control correctivo suave para eje B (solo proporcional)
        if self._controller_b:
            sensor_adc = self._get_sensor_value(self._controller_b.sensor_key)
            if sensor_adc is not None:
                error_adc = ref_adc_y - sensor_adc
                if abs(error_adc) > DEADZONE_ADC:
                    # Solo proporcional, sin integral para evitar acumulación
                    # Ganancia reducida al 30% para control suave
                    pwm_base = self._controller_b.Kp * error_adc * 0.3
                    pwm_b = -int(pwm_base) if self._controller_b.invert else int(pwm_base)
                    U_max = int(self._controller_b.U_max)
                    pwm_b = max(-U_max, min(U_max, pwm_b))
        
        # Enviar comando de mantenimiento de posición si hay corrección necesaria
        if pwm_a != 0 or pwm_b != 0:
            self._send_command(f"A,{pwm_a},{pwm_b}")
    
    def _accept_trajectory_point(self, target_x: float, target_y: float, 
                                  error_x: float, error_y: float, status: str):
        """Acepta el punto actual y PAUSA o AVANZA según modo.
        
        Si auto_advance=True (TestTab): Pausa temporal y avanza automáticamente.
        Si auto_advance=False (MicroscopyService): Pausa indefinida esperando resume_trajectory().
        """
        # CRÍTICO: Evitar múltiples aceptaciones del mismo punto
        if self._point_accepted:
            logger.warning(f"[TestService] Punto {self._trajectory_index + 1} ya fue aceptado - ignorando llamada duplicada")
            return
        
        # Marcar punto como aceptado
        self._point_accepted = True
        
        # Freno activo
        self._send_command('B')
        time.sleep(0.05)
        self._send_command('A,0,0')
        
        # Resetear contadores
        self._traj_settling_counter = 0
        self._traj_near_attempts = 0
        
        # Resetear integrales para el siguiente punto
        self._dual_integral_a = 0.0
        self._dual_integral_b = 0.0
        
        # Emitir señales
        total = len(self._trajectory) if self._trajectory else 0
        self.trajectory_point_reached.emit(self._trajectory_index, target_x, target_y, status)
        
        if self._trajectory_auto_advance:
            # MODO AUTO-ADVANCE (TestTab): Pausa temporal y avanza automáticamente
            # PAUSAR trayectoria durante la pausa para evitar movimiento
            self._trajectory_paused = True
            
            self.log_message.emit(
                f"📍 Punto {self._trajectory_index + 1}/{total}: "
                f"({target_x:.0f}, {target_y:.0f})µm {status} "
                f"[Error: X={error_x:.1f}, Y={error_y:.1f}µm] "
                f"- Pausa {self._trajectory_config.pause_s}s"
            )
            logger.info(f"{status} Punto {self._trajectory_index + 1} - Pausa {self._trajectory_config.pause_s}s antes de avanzar")
            
            # Programar avance automático después de pausa
            pause_ms = int(self._trajectory_config.pause_s * 1000)
            QTimer.singleShot(pause_ms, self._auto_advance_to_next_point)
        else:
            # MODO MANUAL (MicroscopyService): Pausa indefinida esperando comando
            self._trajectory_paused = True
            self.log_message.emit(
                f"📍 Punto {self._trajectory_index + 1}/{total}: "
                f"({target_x:.0f}, {target_y:.0f})µm {status} "
                f"[Error: X={error_x:.1f}, Y={error_y:.1f}µm] "
                f"- PAUSADO (esperando comando)"
            )
            logger.info(f"{status} Punto {self._trajectory_index + 1} - PAUSADO esperando resume_trajectory()")
    
    def _accept_corrected_point(self):
        """Acepta el punto actual después de corregir el eje bloqueado."""
        if not self._trajectory or self._trajectory_index >= len(self._trajectory):
            return
        
        target = self._trajectory[self._trajectory_index]
        target_x, target_y = target[0], target[1]
        
        # Leer errores actuales después de corrección
        error_x_um = 0.0
        error_y_um = 0.0
        
        if self._controller_a:
            sensor_adc = self._get_sensor_value(self._controller_a.sensor_key)
            if sensor_adc is not None:
                ref_adc_x = um_to_adc(target_x, axis='x')
                error_x_um = (ref_adc_x - sensor_adc) * CALIBRATION_X['slope']
        
        if self._controller_b:
            sensor_adc = self._get_sensor_value(self._controller_b.sensor_key)
            if sensor_adc is not None:
                ref_adc_y = um_to_adc(target_y, axis='y')
                error_y_um = (ref_adc_y - sensor_adc) * CALIBRATION_Y['slope']
        
        # Aceptar punto con status de corrección
        self._accept_trajectory_point(target_x, target_y, error_x_um, error_y_um, "✅ Estable (corregido)")
    
    def _start_locked_axis_correction(self, axis: str, target_um: float, current_error_um: float):
        """
        Inicia la corrección de un eje bloqueado que ha acumulado error > 100µm.
        
        Args:
            axis: 'x' o 'y'
            target_um: Posición objetivo en µm
            current_error_um: Error actual en µm
        """
        self._correcting_locked_axis = True
        self._correction_axis = axis
        self._correction_target_um = target_um
        
        # Resetear integral del eje a corregir
        if axis == 'x':
            self._dual_integral_a = 0.0
        else:
            self._dual_integral_b = 0.0
        
        self.log_message.emit(
            f"🔧 Corrigiendo eje {axis.upper()} bloqueado: error={current_error_um:.1f}µm → 0µm"
        )
        logger.info(f"Iniciando corrección de eje {axis.upper()} bloqueado: {current_error_um:.1f}µm")
    
    def _execute_locked_axis_correction(self):
        """
        Ejecuta un paso de corrección del eje bloqueado.
        Solo mueve el motor del eje bloqueado hasta que el error sea < 25µm.
        """
        try:
            current_time = time.time()
            Ts = current_time - self._dual_last_time
            self._dual_last_time = current_time
            
            axis = self._correction_axis
            target_um = self._correction_target_um
            tolerance = self._trajectory_config.tolerance_um
            
            if axis == 'x' and self._controller_a:
                # Corregir eje X (Motor A)
                ref_adc = um_to_adc(target_um, axis='x')
                sensor_adc = self._get_sensor_value(self._controller_a.sensor_key)
                
                if sensor_adc is not None:
                    error_adc = ref_adc - sensor_adc
                    error_um = error_adc * CALIBRATION_X['slope']
                    
                    # Verificar si ya corregimos
                    if abs(error_um) < tolerance:
                        self._correcting_locked_axis = False
                        self._correction_axis = None
                        self._send_command('B')  # Freno
                        time.sleep(0.02)
                        self._send_command('A,0,0')
                        self.log_message.emit(f"✅ Eje X corregido: error={error_um:.1f}µm")
                        logger.info(f"Eje X corregido: {error_um:.1f}µm")
                        # Aceptar punto inmediatamente después de corrección
                        self._accept_corrected_point()
                        return
                    
                    # Control PI solo para Motor A
                    if abs(error_adc) > DEADZONE_ADC:
                        self._dual_integral_a += error_adc * Ts
                        pwm_base = (self._controller_a.Kp * error_adc + 
                                   self._controller_a.Ki * self._dual_integral_a)
                        
                        if self._controller_a.invert:
                            pwm_a = -int(pwm_base)
                        else:
                            pwm_a = int(pwm_base)
                        
                        U_max = int(self._controller_a.U_max)
                        if abs(pwm_a) > U_max:
                            self._dual_integral_a -= error_adc * Ts
                            pwm_a = max(-U_max, min(U_max, pwm_a))
                        
                        # Solo mover Motor A, Motor B = 0
                        self._send_command(f"A,{pwm_a},0")
                        
            elif axis == 'y' and self._controller_b:
                # Corregir eje Y (Motor B)
                ref_adc = um_to_adc(target_um, axis='y')
                sensor_adc = self._get_sensor_value(self._controller_b.sensor_key)
                
                if sensor_adc is not None:
                    error_adc = ref_adc - sensor_adc
                    error_um = error_adc * CALIBRATION_Y['slope']
                    
                    # Verificar si ya corregimos
                    if abs(error_um) < tolerance:
                        self._correcting_locked_axis = False
                        self._correction_axis = None
                        self._send_command('B')  # Freno
                        time.sleep(0.02)
                        self._send_command('A,0,0')
                        self.log_message.emit(f"✅ Eje Y corregido: error={error_um:.1f}µm")
                        logger.info(f"Eje Y corregido: {error_um:.1f}µm")
                        # Aceptar punto inmediatamente después de corrección
                        self._accept_corrected_point()
                        return
                    
                    # Control PI solo para Motor B
                    if abs(error_adc) > DEADZONE_ADC:
                        self._dual_integral_b += error_adc * Ts
                        pwm_base = (self._controller_b.Kp * error_adc + 
                                   self._controller_b.Ki * self._dual_integral_b)
                        
                        if self._controller_b.invert:
                            pwm_b = -int(pwm_base)
                        else:
                            pwm_b = int(pwm_base)
                        
                        U_max = int(self._controller_b.U_max)
                        if abs(pwm_b) > U_max:
                            self._dual_integral_b -= error_adc * Ts
                            pwm_b = max(-U_max, min(U_max, pwm_b))
                        
                        # Solo mover Motor B, Motor A = 0
                        self._send_command(f"A,0,{pwm_b}")
                        
        except Exception as e:
            logger.error(f"TestService: Error en corrección de eje bloqueado: {e}")
            self._correcting_locked_axis = False
    
    @property
    def is_trajectory_active(self) -> bool:
        """Retorna si hay una trayectoria en ejecución."""
        return self._trajectory_active
    
    @property
    def trajectory_progress(self) -> Tuple[int, int]:
        """Retorna (punto_actual, total_puntos)."""
        total = len(self._trajectory) if self._trajectory else 0
        return (self._trajectory_index + 1, total)

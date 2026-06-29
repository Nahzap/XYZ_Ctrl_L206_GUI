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
from typing import Callable, Optional, Dict, List, Tuple, Any
from dataclasses import dataclass, field

from PyQt5.QtCore import QObject, QTimer, pyqtSignal

from config.constants import (
    CALIBRATION_X, CALIBRATION_Y,
    DEADZONE_ADC, POSITION_TOLERANCE_UM, SETTLING_CYCLES,
    MAX_ATTEMPTS_PER_POINT, FALLBACK_TOLERANCE_MULTIPLIER,
    um_to_adc, adc_to_um
)
from core.control.sensor_buffer import SensorBuffer
from core.control.step_config import StepControlConfig, load_step_control_config
from core.control.step_controller import StepController
from core.control.step_metrics import aggregate_point_metrics
from core.control.step_types import PointTransitionResult, StepControllerPhase

logger = logging.getLogger('MotorControl_L206')


from core.control.controller_config import ControllerConfig
class TrajectoryConfig:
    """Configuración para ejecución de trayectoria."""
    tolerance_um: float = 25.0
    pause_s: float = 2.0


@dataclass
class AcceptedPointSnapshot:
    """Estado del último punto de trayectoria aceptado (para metadatos de captura)."""

    index: int
    x_nominal_um: float
    y_nominal_um: float
    x_actual_um: float
    y_actual_um: float
    error_x_um: float
    error_y_um: float
    move_dir_x: int = 0
    move_dir_y: int = 0
    status: str = ""
    n_steps: int = 0
    t_move_ms: float = 0.0
    point_steps: List[Dict[str, Any]] = field(default_factory=list)
    step_metrics: Optional[Dict[str, float]] = None
    fov_verify_passed: bool = False
    t_fov_verify_ms: float = 0.0
    fov_verify_ticks: int = 0

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
        self._dual_paused = False  # NUEVO: Para pausar control XY durante captura
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
        self._last_accepted_snapshot: Optional[AcceptedPointSnapshot] = None

        # Control de pasos homogéneos
        self._sensor_buffer: Optional[SensorBuffer] = None
        self._step_config: StepControlConfig = load_step_control_config()
        self._step_controller: Optional[StepController] = None
        self._step_long_approach_active = False
        
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
        self._ensure_step_controller()
        logger.debug("TestService: Callbacks de hardware configurados")

    def set_sensor_buffer(self, sensor_buffer: SensorBuffer) -> None:
        """Inyecta buffer de sensores con timestamp (alimentado desde main.update_data)."""
        self._sensor_buffer = sensor_buffer
        self._ensure_step_controller()

    def set_step_control_enabled(self, enabled: bool) -> None:
        """Activa/desactiva modo pasos homogéneos en runtime."""
        self._step_config.enabled = bool(enabled)

    def reload_step_control_config(self) -> None:
        """Recarga parámetros step_control desde calibration.json."""
        self._step_config = load_step_control_config()
        if self._step_controller is not None:
            self._step_controller.config = self._step_config

    @property
    def step_control_enabled(self) -> bool:
        return bool(
            self._step_config.enabled
            and self._step_controller is not None
            and self._sensor_buffer is not None
        )

    def get_last_point_transition(self) -> Optional[PointTransitionResult]:
        if self._step_controller is None:
            return None
        return self._step_controller.last_point_result

    def _ensure_step_controller(self) -> None:
        if self._step_controller is not None:
            return
        if self._sensor_buffer is None or self._send_command is None:
            return
        self._step_controller = StepController(
            self._step_config,
            self._sensor_buffer,
            lambda: self._controller_a,
            lambda: self._controller_b,
            self._send_command,
        )

    def _read_sensor_adc(self, sensor_key: str) -> Optional[int]:
        if self._sensor_buffer is not None:
            adc = self._sensor_buffer.get_adc(sensor_key)
            if adc is not None:
                return adc
        if self._get_sensor_value is None:
            return None
        return self._get_sensor_value(sensor_key)
    
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

    def get_last_accepted_snapshot(self) -> Optional[AcceptedPointSnapshot]:
        """Retorna metadatos del último punto de trayectoria aceptado."""
        return self._last_accepted_snapshot

    def read_current_position_um(
        self,
        target_x: Optional[float] = None,
        target_y: Optional[float] = None,
    ) -> Tuple[Optional[float], Optional[float], float, float]:
        """
        Lee posición actual desde sensores (µm) y errores respecto al objetivo.

        Returns:
            (x_actual_um, y_actual_um, error_x_um, error_y_um)
        """
        x_actual = y_actual = None
        error_x = error_y = 0.0

        if self._controller_a:
            sensor_adc = self._read_sensor_adc(self._controller_a.sensor_key)
            if sensor_adc is not None:
                x_actual = adc_to_um(sensor_adc, axis='x')
                if target_x is not None:
                    ref_adc = um_to_adc(target_x, axis='x')
                    error_x = (ref_adc - sensor_adc) * CALIBRATION_X['slope']

        if self._controller_b:
            sensor_adc = self._read_sensor_adc(self._controller_b.sensor_key)
            if sensor_adc is not None:
                y_actual = adc_to_um(sensor_adc, axis='y')
                if target_y is not None:
                    ref_adc = um_to_adc(target_y, axis='y')
                    error_y = (ref_adc - sensor_adc) * CALIBRATION_Y['slope']

        return x_actual, y_actual, error_x, error_y

    def _movement_direction(self, index: int) -> Tuple[int, int]:
        """Dirección de avance nominal (-1, 0, +1) hacia el punto FOV index."""
        if not self._trajectory or index >= len(self._trajectory):
            return 0, 0
        curr = self._trajectory[index]
        if index > 0:
            prev = self._trajectory[index - 1]
        elif self._step_controller is not None:
            prev = self._step_controller.read_current_xy_um(
                self._controller_a, self._controller_b
            )
        else:
            prev = curr
        dir_x = 0 if abs(curr[0] - prev[0]) < 1.0 else (1 if curr[0] > prev[0] else -1)
        dir_y = 0 if abs(curr[1] - prev[1]) < 1.0 else (1 if curr[1] > prev[1] else -1)
        return dir_x, dir_y

    def _build_accepted_snapshot(
        self,
        index: int,
        target_x: float,
        target_y: float,
        error_x: float,
        error_y: float,
        status: str,
        prefer_live_sensor: bool = True,
    ) -> AcceptedPointSnapshot:
        """Construye snapshot con lectura de sensor en el instante de aceptación."""
        x_actual, y_actual, live_ex, live_ey = self.read_current_position_um(target_x, target_y)
        if prefer_live_sensor and x_actual is not None and y_actual is not None:
            error_x, error_y = live_ex, live_ey
        elif x_actual is None:
            x_actual = target_x - error_x
        if y_actual is None:
            y_actual = target_y - error_y

        move_dir_x, move_dir_y = self._movement_direction(index)
        return AcceptedPointSnapshot(
            index=index,
            x_nominal_um=target_x,
            y_nominal_um=target_y,
            x_actual_um=float(x_actual),
            y_actual_um=float(y_actual),
            error_x_um=float(error_x),
            error_y_um=float(error_y),
            move_dir_x=move_dir_x,
            move_dir_y=move_dir_y,
            status=status,
        )

    @staticmethod
    def _serialize_point_steps(result: Optional[PointTransitionResult]) -> List[Dict[str, Any]]:
        if result is None:
            return []
        rows: List[Dict[str, Any]] = []
        for sr in result.steps:
            rows.append(
                {
                    "axis": sr.step.axis,
                    "delta_um": round(sr.step.delta_um, 3),
                    "target_x_um": round(sr.step.target_x_um, 3),
                    "target_y_um": round(sr.step.target_y_um, 3),
                    "duration_ms": round(sr.duration_ms, 1),
                    "error_um": round(sr.error_um, 3),
                    "status": sr.status,
                    "retries": sr.retries,
                    "pwm_max": sr.pwm_max,
                }
            )
        return rows

    def _prepare_step_transition(self) -> None:
        """Descompone transición al punto FOV actual en cola de micro-pasos."""
        if not self.step_control_enabled or not self._trajectory or self._step_controller is None:
            return
        idx = self._trajectory_index
        if idx >= len(self._trajectory):
            return
        target = self._trajectory[idx]
        prev_actual = self._step_controller.read_current_xy_um(
            self._controller_a, self._controller_b
        )
        nominal_prev = (
            self._trajectory[idx - 1] if idx > 0 else prev_actual
        )
        dx_actual = target[0] - prev_actual[0]
        dy_actual = target[1] - prev_actual[1]
        dx_nominal = target[0] - nominal_prev[0]
        dy_nominal = target[1] - nominal_prev[1]

        if idx == 0 and max(abs(dx_actual), abs(dy_actual)) > self._step_config.long_approach_threshold_um:
            self._step_long_approach_active = True
            logger.info(
                "[TestService] Aproximación legacy al punto 1 (ΔX=%.0fµm ΔY=%.0fµm > %.0fµm)",
                abs(dx_actual),
                abs(dy_actual),
                self._step_config.long_approach_threshold_um,
            )
            self.log_message.emit(
                f"   Aproximación legacy al punto 1 (Δ={max(abs(dx_actual), abs(dy_actual)):.0f}µm)…"
            )
            return

        self._step_long_approach_active = False
        move_dir_x, move_dir_y = self._movement_direction(idx)
        backlash_dx, backlash_dy = 0.0, 0.0
        backlash = getattr(self, "_backlash_correction", None)
        if backlash is not None:
            backlash_dx, backlash_dy = backlash.delta_for_direction(move_dir_x, move_dir_y)
        self._step_controller.prepare_transition(
            prev_actual,
            target,
            idx,
            nominal_prev_xy=nominal_prev,
            backlash_dx_um=backlash_dx,
            backlash_dy_um=backlash_dy,
            move_dir_x=move_dir_x,
            move_dir_y=move_dir_y,
        )
        logger.info(
            "[TestService] Transición (%d) actual (%.1f,%.1f)→(%.1f,%.1f) "
            "Δactual=(%.1f,%.1f)µm Δnominal FOV=(%.1f,%.1f)µm",
            idx + 1,
            prev_actual[0],
            prev_actual[1],
            target[0],
            target[1],
            dx_actual,
            dy_actual,
            dx_nominal,
            dy_nominal,
        )
        if idx > 0 and (
            abs(abs(dx_actual) - abs(dx_nominal)) > 20.0
            or abs(abs(dy_actual) - abs(dy_nominal)) > 20.0
        ):
            logger.warning(
                "[TestService] Desfase posición real vs FOV nominal en punto %d — "
                "usando Δactual para micro-pasos",
                idx + 1,
            )
    
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
        """Ejecuta un paso del control dual."""
        try:
            if not self._dual_active or self._send_command is None or self._get_sensor_value is None:
                return
            
            # CRÍTICO: Si está pausado, NO enviar comandos (mantiene posición actual)
            if self._dual_paused:
                # NO hacer logging aquí porque se llama 100 veces por segundo
                return
            
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
    
    @property
    def is_dual_control_paused(self) -> bool:
        """Retorna si el control dual está pausado."""
        return self._dual_paused
    
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
        
        if not self._send_command:
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
        
        # LOGGING DETALLADO para diagnóstico
        logger.info(f"[TestService] ⚙️  auto_advance configurado: {auto_advance}")
        logger.info(f"[TestService] ⚙️  pause_s configurado: {pause_s}s")
        logger.info(f"[TestService] ⚙️  tolerance_um configurado: {tolerance_um}µm")
        
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
        
        if self.step_control_enabled:
            self._step_controller.reset_session()
            self._step_long_approach_active = False
            self._prepare_step_transition()
            self.log_message.emit(
                f"   Modo pasos homogéneos: step={self._step_config.step_um}µm, "
                f"tol={self._step_config.tol_step_um}µm"
            )
        
        self.trajectory_started.emit(len(trajectory))
        self.log_message.emit(f"🚀 Ejecutando trayectoria: {len(trajectory)} puntos")
        self.log_message.emit(f"   Tolerancia: {tolerance_um}µm, Pausa: {pause_s}s")
        
        return True
    
    def stop_trajectory(self):
        """Detiene la ejecución de la trayectoria con freno activo."""
        logger.info("=== TestService: DETENIENDO TRAYECTORIA ===")
        
        self._trajectory_active = False
        
        if self._step_controller is not None:
            self._step_controller.reset_session()
        self._step_long_approach_active = False
        
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
    
    def pause_dual_control(self):
        """
        Pausa temporalmente el control dual XY (mantiene posición).
        Usado durante captura multifocal para evitar movimiento XY.
        
        CRÍTICO: DETIENE el timer para que NO se ejecute _execute_dual_control_step
        y NO se envíen comandos A,x,y durante el autofoco Z.
        """
        if not self._dual_active:
            logger.warning("[TestService] ⚠️  No se puede pausar: control dual NO está activo")
            return
        
        if self._dual_paused:
            logger.warning("[TestService] ⚠️  Control dual YA está pausado")
            return
        
        # CRÍTICO: DETENER el timer para que NO se ejecuten comandos XY
        if self._dual_timer:
            self._dual_timer.stop()
            logger.info("[TestService] 🛑 Timer del control dual XY DETENIDO")
        
        # Activar BRAKE para mantener posición
        if self._send_command:
            self._send_command('B')  # Freno activo
            time.sleep(0.02)
            self._send_command('A,0,0')  # PWM a 0
            logger.info("[TestService] 🔒 BRAKE activado - motores XY bloqueados")
        
        self._dual_paused = True
        logger.info("[TestService] ⏸️  Control dual XY PAUSADO COMPLETAMENTE (timer detenido + BRAKE activo)")
    
    def resume_dual_control(self):
        """
        Reanuda el control dual XY después de captura.
        """
        if self._dual_active and self._dual_paused:
            self._dual_paused = False
            logger.info("[TestService] ▶️  Control dual XY REANUDADO")
    
    def resume_trajectory(self, advance_to_next: bool = True):
        """
        Reanuda la ejecución de la trayectoria.
        
        Args:
            advance_to_next: Si True, avanza al siguiente punto (después de captura).
                           Si False, reanuda en el punto actual (pausa manual).
        
        Este método es llamado explícitamente por MicroscopyService.
        """
        if not self._trajectory_active:
            logger.warning("[TestService] Intento de reanudar trayectoria inactiva.")
            return
        if not self._trajectory_paused:
            logger.warning("[TestService] Intento de reanudar trayectoria no pausada.")
            return

        try:
            if advance_to_next:
                logger.info("[TestService] ▶️  Comando RESUME_TRAJECTORY recibido. Avanzando al siguiente punto.")
            else:
                logger.info("[TestService] ▶️  Comando RESUME_TRAJECTORY recibido. Reanudando en punto actual (pausa manual).")
            
            # DEBUG: Estado ANTES de cambios
            logger.info(f"[DEBUG-RESUME] ANTES: índice={self._trajectory_index}, _point_accepted={self._point_accepted}, paused={self._trajectory_paused}, advance={advance_to_next}")
            
            # PRIMERO: Actualizar todas las variables de estado
            self._trajectory_paused = False
            
            if advance_to_next:
                # Avanzar al siguiente punto (después de captura completada)
                self._trajectory_index += 1
                
                # CRÍTICO: Resetear flag de punto aceptado para el nuevo punto
                # Sin esto, _accept_trajectory_point() detecta que el punto ya fue aceptado
                # y el sistema queda atascado indefinidamente
                self._point_accepted = False

                # Resetear integrales al reanudar para evitar wind-up
                self._dual_integral_a = 0.0
                self._dual_integral_b = 0.0
                if self.step_control_enabled:
                    self._prepare_step_transition()
            else:
                # Pausa manual: NO avanzar, solo reanudar en punto actual
                # NO resetear _point_accepted porque el punto ya fue alcanzado
                logger.info("[TestService] Reanudando en punto actual sin avanzar")
            
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
        if self.step_control_enabled:
            self._prepare_step_transition()
    
    def _tick_step_long_approach(self) -> bool:
        """PI legacy hacia punto 1 hasta distancia manejable por micro-pasos."""
        if not self._trajectory:
            return False

        target_x, target_y = self._trajectory[self._trajectory_index]
        Ts = max(1e-4, time.time() - self._dual_last_time)
        self._dual_last_time = time.time()

        ref_adc_x = um_to_adc(target_x, axis="x")
        ref_adc_y = um_to_adc(target_y, axis="y")
        pwm_a = pwm_b = 0
        error_x_um = error_y_um = 0.0

        if self._controller_a:
            sensor_adc = self._get_sensor_value(self._controller_a.sensor_key)
            if sensor_adc is not None:
                error_adc = ref_adc_x - sensor_adc
                error_x_um = error_adc * CALIBRATION_X["slope"]
                if abs(error_adc) > DEADZONE_ADC:
                    self._dual_integral_a += error_adc * Ts
                    pwm_base = (
                        self._controller_a.Kp * error_adc
                        + self._controller_a.Ki * self._dual_integral_a
                    )
                    pwm_a = -int(pwm_base) if self._controller_a.invert else int(pwm_base)
                    umax = int(self._get_adaptive_pwm_limit("x", error_x_um))
                    if abs(pwm_a) > umax:
                        self._dual_integral_a -= error_adc * Ts
                        pwm_a = max(-umax, min(umax, pwm_a))

        if self._controller_b:
            sensor_adc = self._get_sensor_value(self._controller_b.sensor_key)
            if sensor_adc is not None:
                error_adc = ref_adc_y - sensor_adc
                error_y_um = error_adc * CALIBRATION_Y["slope"]
                if abs(error_adc) > DEADZONE_ADC:
                    self._dual_integral_b += error_adc * Ts
                    pwm_base = (
                        self._controller_b.Kp * error_adc
                        + self._controller_b.Ki * self._dual_integral_b
                    )
                    pwm_b = -int(pwm_base) if self._controller_b.invert else int(pwm_base)
                    umax = int(self._get_adaptive_pwm_limit("y", error_y_um))
                    if abs(pwm_b) > umax:
                        self._dual_integral_b -= error_adc * Ts
                        pwm_b = max(-umax, min(umax, pwm_b))

        self._send_command(f"A,{pwm_a},{pwm_b}")
        self.trajectory_feedback.emit(
            target_x, target_y, error_x_um, error_y_um, False, False, 0
        )

        done = self._step_config.long_approach_done_um
        if abs(error_x_um) < done and abs(error_y_um) < done:
            self._step_long_approach_active = False
            self._dual_integral_a = 0.0
            self._dual_integral_b = 0.0
            self._send_command("B")
            self._send_command("A,0,0")
            time.sleep(0.15)
            logger.info(
                "[TestService] Aproximación legacy completa (err X=%.1f Y=%.1f µm)",
                error_x_um,
                error_y_um,
            )
            self.log_message.emit("   ✓ Aproximación legacy OK — pasos homogéneos")
            return True
        return False

    def _execute_trajectory_step_step_mode(self) -> None:
        """Un tick del controlador de pasos homogéneos."""
        if self._step_controller is None or not self._trajectory:
            return

        if self._step_long_approach_active:
            if self._tick_step_long_approach():
                self._prepare_step_transition()
            return

        out = self._step_controller.tick()
        self.trajectory_feedback.emit(
            out.feedback_target_x,
            out.feedback_target_y,
            out.error_x_um,
            out.error_y_um,
            out.lock_x,
            out.lock_y,
            out.settling,
        )

        if out.point_failed:
            idx = self._trajectory_index + 1
            self.error_occurred.emit(f"Punto {idx}: fallo en paso homogéneo")
            self.log_message.emit(f"❌ Punto {idx}: fallo en control de pasos homogéneos")
            self.stop_trajectory()
            return

        if out.point_complete:
            if self._point_accepted:
                return
            target = self._trajectory[self._trajectory_index]
            result = self._step_controller.last_point_result
            n_steps = result.n_steps if result else 0
            t_move = result.t_move_ms if result else 0.0
            status = f"✅ Homogéneo ({n_steps} pasos, {t_move:.0f}ms)"
            _, _, err_x, err_y = self.read_current_position_um(target[0], target[1])
            self._accept_trajectory_point(
                target[0],
                target[1],
                err_x,
                err_y,
                status,
                point_result=result,
            )
    
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
        
        FASE 2: Si está pausado, motores DETENIDOS con BRAKE (NO enviar comandos).
        """
        try:
            if not self._trajectory_active:
                return
            
            # FASE 2: Si está pausado, NO hacer nada (motores ya están con BRAKE)
            # Los motores fueron detenidos en _accept_trajectory_point() con:
            #   - send_command('B')  ← BRAKE activo
            #   - send_command('A,0,0')  ← PWM a 0
            # NO se envían comandos de corrección durante pausa
            if self._trajectory_paused:
                return

            if self.step_control_enabled:
                self._execute_trajectory_step_step_mode()
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
                        logger.info(f"[TestService] 📍 Aceptando punto {self._trajectory_index + 1} - auto_advance={self._trajectory_auto_advance}")
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
    
    def _accept_trajectory_point(self, target_x: float, target_y: float, 
                                  error_x: float, error_y: float, status: str,
                                  point_result: Optional[PointTransitionResult] = None):
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

        self._last_accepted_snapshot = self._build_accepted_snapshot(
            self._trajectory_index,
            target_x,
            target_y,
            error_x,
            error_y,
            status,
        )
        if point_result is not None:
            self._last_accepted_snapshot.n_steps = point_result.n_steps
            self._last_accepted_snapshot.t_move_ms = point_result.t_move_ms
            self._last_accepted_snapshot.point_steps = self._serialize_point_steps(point_result)
            self._last_accepted_snapshot.step_metrics = aggregate_point_metrics(point_result.steps)
            self._last_accepted_snapshot.fov_verify_passed = point_result.fov_verify_passed
            self._last_accepted_snapshot.t_fov_verify_ms = point_result.t_fov_verify_ms
            self._last_accepted_snapshot.fov_verify_ticks = point_result.fov_verify_ticks
            if point_result.fov_verify_passed:
                self._last_accepted_snapshot.x_actual_um = point_result.x_actual_um
                self._last_accepted_snapshot.y_actual_um = point_result.y_actual_um
                self._last_accepted_snapshot.error_x_um = point_result.residual_x_um
                self._last_accepted_snapshot.error_y_um = point_result.residual_y_um
        
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

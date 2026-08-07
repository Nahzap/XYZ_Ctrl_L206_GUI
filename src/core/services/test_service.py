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
    STITION_PWM_MAX,
    STITION_PWM_MIN,
    adc_to_um,
    host_slew_pwm,
    lsb_um,
    position_error_um,
)
from core.control.motor_antecedent import (
    AntecedentSample,
    MotorAntecedentResult,
    finalize_antecedent,
)
from core.control.sensor_buffer import SensorBuffer
from core.control.control_worker import ControlWorker
from core.control.dual_power_allocator import (
    DualPowerAllocator,
    DualPowerConfig,
    DualAxisState,
)
from core.control.host_approach import HostApproachController
from core.control.step_config import StepControlConfig, load_step_control_config
from core.control.step_controller import StepController
from core.control.step_metrics import aggregate_point_metrics
from core.control.step_types import PointTransitionResult, StepControllerPhase

logger = logging.getLogger('MotorControl_L206')

# Default si la UI no manda timeout; el valor vivo está en TrajectoryConfig.
DEFAULT_POINT_TIMEOUT_S = 6.0
FOV_COVER_LOG_INTERVAL_S = 1.0

from core.control.controller_config import ControllerConfig
class TrajectoryConfig:
    """Configuración para ejecución de trayectoria."""
    tolerance_um: float = 25.0
    pause_s: float = 2.0
    point_timeout_s: float = DEFAULT_POINT_TIMEOUT_S


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
    # Worker→GUI: programar auto-advance (QTimer.singleShot desde QThread no dispara).
    _schedule_auto_advance = pyqtSignal(int)
    
    # === SEÑALES GENERALES ===
    log_message = pyqtSignal(str)  # Mensaje para UI
    error_occurred = pyqtSignal(str)  # Error
    antecedent_probe_finished = pyqtSignal(object)  # MotorAntecedentResult
    
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
        # Reloj de control en QThread propio (Fase 1: fuera del hilo GUI).
        self._dual_worker: Optional[ControlWorker] = None
        self._trajectory_worker: Optional[ControlWorker] = None
        self._dual_ref_a_um = 0.0
        self._dual_ref_b_um = 0.0
        self._dual_integral_a = 0.0  # legacy (allocator lleva integral propia)
        self._dual_integral_b = 0.0
        self._dual_last_time = 0.0
        self._dual_position_reached = False
        self._dual_settling_counter = 0
        self._dual_log_counter = 0
        self._dual_power = DualPowerAllocator()
        self._host_approach = HostApproachController()
        self._last_dual_ui_mono = 0.0
        self._last_dual_term_mono = 0.0
        
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

        # Orquestación FOV: host approach único + MCU C(z)
        self._sensor_buffer: Optional[SensorBuffer] = None
        self._step_config: StepControlConfig = load_step_control_config()
        self._step_controller: Optional[StepController] = None
        self._step_long_approach_active = False
        self._fov_host_retries = 0
        self._last_traj_pwm: Tuple[int, int] = (0, 0)
        self._last_traj_pwm_mono = 0.0
        self._last_traj_fb_mono = 0.0
        self._last_traj_term_mono = 0.0
        self._traj_log_pwm: Tuple[int, int] = (0, 0)
        # Handoff no bloqueante: coast → pre_arm → prepare_mcu_fine
        self._handoff_phase: Optional[str] = None
        self._handoff_deadline_mono = 0.0
        self._xy_capture_paused = False
        # Watchdog por punto (approach + cobertura + cierre)
        self._fov_cover_t0_mono = 0.0
        self._fov_cover_last_log_mono = 0.0
        self._fov_cover_rearmed = False
        self._point_timeout_force_emitted = False

        # Sonda de antecedente (~2000 µm open-loop + sensado)
        self._antecedent_active = False
        self._antecedent_timer: Optional[QTimer] = None
        self._antecedent_result: Optional[MotorAntecedentResult] = None
        self._antecedent_pos0 = 0.0
        self._antecedent_t0 = 0.0
        self._antecedent_timeout_s = 10.0

        self._schedule_auto_advance.connect(self._on_schedule_auto_advance)
        
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

    # =========================================================================
    # ANTECEDENTE EMPÍRICO (~2000 µm)
    # =========================================================================
    def start_antecedent_probe(
        self,
        motor: str,
        *,
        delta_um: float = 2000.0,
        pwm: int = 120,
        direction: int = 1,
        sensor_key: Optional[str] = None,
        invert: Optional[bool] = None,
        timeout_s: float = 10.0,
    ) -> bool:
        """Open-loop: mueve un motor ~delta_um, sensa y guarda antecedente."""
        if self._send_command is None:
            self.error_occurred.emit("Sin conexión HW para sonda de antecedente")
            return False
        if self._antecedent_active:
            self.error_occurred.emit("Sonda de antecedente ya activa")
            return False

        motor_u = str(motor).strip().upper()
        if motor_u not in ("A", "B"):
            self.error_occurred.emit("Motor debe ser A o B")
            return False

        ctrl = self._controller_a if motor_u == "A" else self._controller_b
        axis = "x" if motor_u == "A" else "y"
        if sensor_key is None:
            if ctrl is not None:
                sensor_key = str(ctrl.sensor_key)
            else:
                sensor_key = "sensor_2" if motor_u == "A" else "sensor_1"
        if invert is None:
            invert = bool(getattr(ctrl, "invert", False)) if ctrl is not None else False

        pwm_mag = max(int(STITION_PWM_MIN), min(int(STITION_PWM_MAX), abs(int(pwm))))
        direction = 1 if int(direction) >= 0 else -1
        target = abs(float(delta_um))

        adc = self._read_sensor_adc(sensor_key)
        if adc is None:
            self.error_occurred.emit(f"No hay lectura de {sensor_key}")
            return False

        # Parar otras actuaciones
        self.halt_motion("antecedent_probe")
        self._motion_halted = False
        self._send_command("A,0,0")

        pos0 = float(adc_to_um(float(adc), axis=axis))
        self._antecedent_result = MotorAntecedentResult(
            motor=motor_u,
            axis=axis,
            sensor_key=sensor_key,
            host_invert=bool(invert),
            pwm_cmd=pwm_mag,
            target_delta_um=target,
            direction=direction,
            umin=int(STITION_PWM_MIN),
            umax=int(STITION_PWM_MAX),
        )
        self._antecedent_pos0 = pos0
        self._antecedent_t0 = time.perf_counter()
        self._antecedent_timeout_s = max(2.0, float(timeout_s))
        self._antecedent_active = True

        if self._antecedent_timer is None:
            self._antecedent_timer = QTimer(self)
            self._antecedent_timer.setInterval(10)  # 100 Hz
            self._antecedent_timer.timeout.connect(self._tick_antecedent_probe)
        self._antecedent_timer.start()

        self.log_message.emit(
            f"🔬 Antecedente Motor {motor_u}: Δ≈{target:.0f}µm PWM={pwm_mag} "
            f"dir={direction:+d} sensor={sensor_key} invert={bool(invert)}"
        )
        logger.info(
            "[TestService] Antecedent start %s Δ=%.0f pwm=%d dir=%+d sensor=%s",
            motor_u,
            target,
            pwm_mag,
            direction,
            sensor_key,
        )
        return True

    def stop_antecedent_probe(self, reason: str = "stop") -> None:
        if not self._antecedent_active:
            return
        self._finish_antecedent_probe(reason)

    def _tick_antecedent_probe(self) -> None:
        if not self._antecedent_active or self._antecedent_result is None:
            return
        res = self._antecedent_result
        adc = self._read_sensor_adc(res.sensor_key)
        if adc is None:
            return
        now = time.perf_counter()
        t = now - self._antecedent_t0
        pos = float(adc_to_um(float(adc), axis=res.axis))
        delta = pos - self._antecedent_pos0
        target_signed = float(res.direction) * float(res.target_delta_um)
        remaining = target_signed - delta

        pwm_axis = host_slew_pwm(
            remaining,
            host_invert=bool(res.host_invert),
            magnitude=int(res.pwm_cmd),
        )
        if res.motor == "A":
            self._send_command(f"A,{pwm_axis},0")
        else:
            self._send_command(f"A,0,{pwm_axis}")

        res.samples.append(
            AntecedentSample(
                t_s=t,
                adc=float(adc),
                pos_um=pos,
                pwm=int(pwm_axis),
                delta_um=delta,
            )
        )

        if abs(delta) >= 0.98 * abs(res.target_delta_um):
            self._finish_antecedent_probe("target_reached")
            return
        if t >= self._antecedent_timeout_s:
            self._finish_antecedent_probe("timeout")

    def _finish_antecedent_probe(self, reason: str) -> None:
        if self._antecedent_timer is not None:
            self._antecedent_timer.stop()
        if self._send_command is not None:
            self._send_command("A,0,0")
        self._antecedent_active = False
        res = self._antecedent_result
        self._antecedent_result = None
        if res is None:
            return
        if not res.reason:
            res.reason = reason
        res = finalize_antecedent(res)
        for line in res.summary_lines():
            self.log_message.emit(f"   {line}")
        logger.info(
            "[TestService] Antecedent done %s ok=%s Δ=%.1fµm K_eff=%.4f (%s)",
            res.motor,
            res.ok,
            res.delta_um,
            res.k_eff_um_s_per_pwm,
            res.reason,
        )
        self.antecedent_probe_finished.emit(res)

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
                    error_x = position_error_um(target_x, sensor_adc, 'x')

        if self._controller_b:
            sensor_adc = self._read_sensor_adc(self._controller_b.sensor_key)
            if sensor_adc is not None:
                y_actual = adc_to_um(sensor_adc, axis='y')
                if target_y is not None:
                    error_y = position_error_um(target_y, sensor_adc, 'y')

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

    def _infer_mesh_step_um(self) -> float:
        """Espaciado Chebyshev típico de la malla (primeros pasos)."""
        if not self._trajectory or len(self._trajectory) < 2:
            return 0.0
        steps: List[float] = []
        for i in range(1, min(len(self._trajectory), 12)):
            a = self._trajectory[i - 1]
            b = self._trajectory[i]
            steps.append(
                max(abs(float(b[0]) - float(a[0])), abs(float(b[1]) - float(a[1])))
            )
        return float(max(steps)) if steps else 0.0

    def _apply_fov_trajectory_policy(self, tolerance_um: float) -> None:
        """Política única de trayectoria FOV (valores desde StepControlConfig)."""
        tol = max(1.0, float(tolerance_um))
        mesh = self._infer_mesh_step_um()
        if mesh > 1.0:
            tol_safe = mesh / 10.0
            if tol > tol_safe + 1e-6:
                logger.warning(
                    "[TestService] Tol. trayectoria clamp: UI=%.1fµm → %.1fµm "
                    "(paso_malla/10=%.1f/10) — tol≥paso/2 hace imposible cubrir FOV",
                    tol,
                    tol_safe,
                    mesh,
                )
                self.log_message.emit(
                    f"   ⚠ Tol. {tol:.0f}µm > paso/10 ({tol_safe:.0f}µm, paso={mesh:.0f}) "
                    f"— usando {tol_safe:.0f}µm"
                )
                tol = tol_safe
        self._trajectory_config.tolerance_um = tol
        cfg = self._step_config
        cfg.tol_fov_um = tol
        cfg.tol_step_um = min(cfg.tol_step_um, tol)
        from config.constants import mcu_supports_cz
        cfg.use_mcu_cz_loop = bool(mcu_supports_cz())
        cfg.use_mcu_atom_pulse = False
        cfg.long_approach_done_um = max(
            float(cfg.handoff_done_min_um),
            tol * float(cfg.handoff_done_factor),
        )
        # Si la holgura es grande, el engage FINE debe crecer con ella
        # (si no, se queda cazando ±90µm aunque tol=500).
        # Usar piso fijo 90 — no el valor mutado de una corrida previa.
        cfg.fine_engage_um = max(90.0, cfg.long_approach_done_um + 20.0)
        # Cierre canónico = PI MCU continuo; no presupuesto de átomos.
        cfg.fov_cz_max_fires = 0
        logger.info(
            "[TestService] Política FOV: tol=%.1fµm done=%.1fµm engage=%.1fµm mesh=%.1fµm",
            tol,
            cfg.long_approach_done_um,
            cfg.fine_engage_um,
            mesh,
        )

    def _point_timeout_s(self) -> float:
        return max(
            0.5,
            float(
                getattr(
                    self._trajectory_config,
                    "point_timeout_s",
                    DEFAULT_POINT_TIMEOUT_S,
                )
                or DEFAULT_POINT_TIMEOUT_S
            ),
        )

    def _reset_fov_cover_watch(self) -> None:
        """Reinicia watchdog al entrar a un punto nuevo."""
        now = time.perf_counter()
        self._fov_cover_t0_mono = now
        self._fov_cover_last_log_mono = 0.0
        self._fov_cover_rearmed = False
        self._point_timeout_force_emitted = False

    def _fov_cover_timed_out(self) -> bool:
        """Alias: timeout de punto (approach / cobertura / cierre)."""
        if self._fov_cover_t0_mono <= 0.0:
            return False
        return (time.perf_counter() - self._fov_cover_t0_mono) >= self._point_timeout_s()

    def _emit_cover_deny_throttled(self, ui_msg: str, log_msg: str) -> None:
        now = time.perf_counter()
        if (now - self._fov_cover_last_log_mono) < FOV_COVER_LOG_INTERVAL_S:
            return
        self._fov_cover_last_log_mono = now
        tmo = self._point_timeout_s()
        elapsed = (
            (now - self._fov_cover_t0_mono) if self._fov_cover_t0_mono > 0 else 0.0
        )
        logger.warning("%s (t=%.1f/%.0fs)", log_msg, elapsed, tmo)
        self.log_message.emit(f"{ui_msg} (t={elapsed:.0f}/{tmo:.0f}s)")

    def _force_accept_point_timeout(self, phase: str) -> None:
        """Acepta el punto actual por timeout y avanza (con error en status)."""
        if self._point_accepted or not self._trajectory:
            return
        idx = self._trajectory_index
        if idx >= len(self._trajectory):
            return
        target = self._trajectory[idx]
        tmo = self._point_timeout_s()
        err_x = err_y = 0.0
        try:
            if self._step_controller is not None:
                actual = self._step_controller.read_current_xy_um(
                    self._controller_a, self._controller_b
                )
                err_x = float(target[0]) - float(actual[0])
                err_y = float(target[1]) - float(actual[1])
            else:
                _, _, err_x, err_y = self.read_current_position_um(
                    target[0], target[1]
                )
        except Exception:
            pass
        residual = max(abs(err_x), abs(err_y))
        status = (
            f"⚠️ point t/o {tmo:.0f}s ({phase}) "
            f"res={residual:.0f}µm err=({err_x:+.0f},{err_y:+.0f})"
        )
        if not self._point_timeout_force_emitted:
            self._point_timeout_force_emitted = True
            logger.error(
                "[TestService] POINT TIMEOUT P%d tras %.0fs phase=%s "
                "residual=%.1f err=(%+.1f,%+.1f) — avanzo con error",
                idx + 1,
                tmo,
                phase,
                residual,
                err_x,
                err_y,
            )
            self.log_message.emit(
                f"   ⚠ Punto {idx + 1}: timeout {tmo:.0f}s ({phase}) "
                f"res={residual:.0f}µm err=({err_x:+.0f},{err_y:+.0f})µm — avanzo"
            )
        self._step_long_approach_active = False
        self._handoff_phase = None
        try:
            self._send_command("A,0,0")
        except Exception:
            pass
        self._accept_trajectory_point(
            target[0],
            target[1],
            err_x,
            err_y,
            status,
            point_result=None,
            force_cover=True,
        )

    def _fov_step_coverage_ok(
        self,
        idx: int,
        actual_xy: Tuple[float, float],
        target_xy: Tuple[float, float],
        tol_um: float,
    ) -> Tuple[bool, dict]:
        """
        True si el XY actual cubrió el paso de malla hacia el target.

        Evita aceptar Pₙ₊₁ cuando aún estamos en la banda de Pₙ porque
        tol ≫ FOV/10 (p.ej. tol=100, FOV=122 → residual 22µm ≤ tol).
        """
        info = {
            "idx": int(idx) + 1,
            "target_xy": (float(target_xy[0]), float(target_xy[1])),
            "actual_xy": (float(actual_xy[0]), float(actual_xy[1])),
            "prev_nominal_xy": None,
            "delta_nominal_um": 0.0,
            "travel_um": 0.0,
            "tol_um": float(tol_um),
            "ok": True,
            "reason": "first_or_small_step",
        }
        if not self._trajectory or idx <= 0:
            return True, info
        prev = self._trajectory[idx - 1]
        info["prev_nominal_xy"] = (float(prev[0]), float(prev[1]))
        d_nom = max(
            abs(float(target_xy[0]) - float(prev[0])),
            abs(float(target_xy[1]) - float(prev[1])),
        )
        travel = max(
            abs(float(actual_xy[0]) - float(prev[0])),
            abs(float(actual_xy[1]) - float(prev[1])),
        )
        info["delta_nominal_um"] = float(d_nom)
        info["travel_um"] = float(travel)
        # Holgura efectiva ≤ paso/10: con tol=100 y FOV=162, la regla
        # antigua (Δ>2·tol) nunca disparaba y aceptaba Pₙ₊₁ sin cubrir FOV.
        cov_tol = float(tol_um)
        if d_nom > 1e-6:
            cov_tol = min(cov_tol, d_nom / 10.0)
        info["cov_tol_um"] = float(cov_tol)
        min_travel = max(0.0, d_nom - cov_tol)
        info["min_travel_um"] = float(min_travel)
        if d_nom > cov_tol + 1e-6 and travel + 1e-6 < min_travel:
            info["ok"] = False
            info["reason"] = "insufficient_travel"
            return False, info
        # Aún dentro de la bola del punto anterior → no es un FOV nuevo
        if d_nom > 2.0 * cov_tol + 1e-6 and travel <= float(tol_um) + 1e-6:
            info["ok"] = False
            info["reason"] = "still_in_prev_ball"
            return False, info
        info["reason"] = "coverage_ok"
        return True, info

    def _prepare_step_transition(self, *, reset_cover_watch: bool = True) -> None:
        """Descompone transición al punto FOV actual en cola de micro-pasos."""
        if not self.step_control_enabled or not self._trajectory or self._step_controller is None:
            return
        idx = self._trajectory_index
        if idx >= len(self._trajectory):
            return
        if reset_cover_watch:
            self._reset_fov_cover_watch()
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
        dist = max(abs(dx_actual), abs(dy_actual))
        done = float(self._step_config.long_approach_done_um)

        # Host sucesivo: PI TF + rampa PWM → handoff → MCU C(z).
        if dist > done:
            engage = float(getattr(self._step_config, "fine_engage_um", 90.0))
            self._arm_soft_approach(done)
            logger.info(
                "[TestService] Approach sucesivo punto %d "
                "(Δ=%.0fµm) rampa PI ±%.0f→±%.0fµm",
                idx + 1,
                dist,
                engage,
                done,
            )
            self.log_message.emit(
                f"   Approach sucesivo (Δ={dist:.0f}µm): "
                f"PI·rampa ±{engage:.0f}→±{done:.0f}µm (sin bang)…"
            )
            return

        self._step_long_approach_active = False
        move_dir_x, move_dir_y = self._movement_direction(idx)
        backlash_dx, backlash_dy = 0.0, 0.0
        backlash = getattr(self, "_backlash_correction", None)
        if backlash is not None:
            backlash_dx, backlash_dy = backlash.delta_for_direction(
                move_dir_x, move_dir_y
            )
        self._step_controller.prepare_mcu_fine(
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
            "[TestService] FOV MCU-fine (%d) actual (%.1f,%.1f)→(%.1f,%.1f) "
            "Δ=(%.1f,%.1f)µm",
            idx + 1,
            prev_actual[0],
            prev_actual[1],
            target[0],
            target[1],
            dx_actual,
            dy_actual,
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

        # Exclusión mutua: un solo productor A,* en el bus.
        if self._trajectory_active or self._dual_active:
            self.halt_motion("start_dual_preempt")
        self._motion_halted = False
        
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
        self._dual_power = DualPowerAllocator(config=DualPowerConfig.for_dual())
        self._dual_power.reset()
        
        # Activar control
        self._dual_active = True
        self._dual_paused = False
        self._xy_capture_paused = False
        self._handoff_phase = None

        # Reloj de control en QThread propio (Fase 1: reemplaza QTimer(10)).
        # Corre fuera del hilo GUI → sin jitter por repintado y a > 100 Hz.
        from config.constants import CONTROL_RATE_HZ
        if self._dual_worker is not None:
            self._dual_worker.stop()
        self._dual_worker = ControlWorker(
            tick=self._execute_dual_control_step,
            rate_hz=CONTROL_RATE_HZ,
            name="DualControlWorker",
        )
        self._dual_worker.start()

        self.dual_control_started.emit()
        self.log_message.emit("🎮 Control Dual ACTIVO")
        logger.info("TestService: Control dual iniciado")
        
        return True
    
    def halt_motion(self, reason: str = "") -> None:
        """Único corte de motores/MCU (idempotente).

        Emite N→B→A,0,0→M una sola vez hasta el próximo start_*. Evita carrera
        RX si stop_trajectory / stop_dual / stop_microscopy se encadenan.
        """
        if getattr(self, "_halt_in_progress", False):
            return
        self._halt_in_progress = True
        try:
            self._trajectory_active = False
            self._trajectory_paused = False
            self._trajectory_waiting = False
            self._dual_active = False
            self._dual_paused = False
            self._step_long_approach_active = False
            self._handoff_phase = None
            self._xy_capture_paused = False
            self._dual_position_reached = False
            if getattr(self, "_dual_power", None) is not None:
                self._dual_power.reset()
            if getattr(self, "_host_approach", None) is not None:
                self._host_approach.reset(
                    float(self._step_config.long_approach_done_um),
                    float(getattr(self._step_config, "fine_engage_um", 90.0)),
                )
            if self._step_controller is not None:
                try:
                    self._step_controller.reset_session()
                except Exception:
                    pass

            already = bool(getattr(self, "_motion_halted", False))
            if self._send_command and not already:
                from core.communication.motion_halt import send_full_halt
                if send_full_halt(self._send_command, reason=reason or "halt_motion"):
                    self._motion_halted = True
            elif already:
                logger.debug("[TestService] halt_motion (%s): ya haltado", reason or "?")

            if self._trajectory_worker is not None:
                self._trajectory_worker.stop(wait_ms=300)
                self._trajectory_worker = None
            if self._trajectory_timer:
                self._trajectory_timer.stop()
                self._trajectory_timer = None
            if self._dual_worker:
                self._dual_worker.stop(wait_ms=300)
                self._dual_worker = None
        finally:
            self._halt_in_progress = False

    def is_xy_motion_active(self) -> bool:
        """True si el lazo XY está actuando (no pausado para captura/punto)."""
        traj_busy = bool(self._trajectory_active and not self._trajectory_paused)
        dual_busy = bool(self._dual_active and not self._dual_paused)
        return traj_busy or dual_busy

    def pause_xy_for_capture(self, reason: str = "") -> None:
        """Pausa traj y/o dual para detección/AF/Z. Soft-park, sin sleep ni halt duro."""
        if self._xy_capture_paused:
            return
        paused_any = False
        if self._trajectory_active and not self._trajectory_paused:
            self._trajectory_paused = True
            paused_any = True
        if self._dual_active and not self._dual_paused:
            self._dual_paused = True
            if self._dual_worker is not None:
                self._dual_worker.pause(True)
            paused_any = True
        if self._send_command:
            sc = self._step_controller
            if sc is not None and getattr(sc, "_fov_cz_armed", False):
                try:
                    sc._cz_soft_off()
                except Exception:
                    self._send_command("N")
            self._send_command("A,0,0")
        self._xy_capture_paused = True
        logger.info(
            "[TestService] pause_xy_for_capture (%s) traj=%s dual=%s",
            reason or "?",
            self._trajectory_paused,
            self._dual_paused,
        )
        if not paused_any and (self._trajectory_active or self._dual_active):
            # Ya estaba pausado (p.ej. punto alcanzado); solo soft-park.
            logger.debug("[TestService] pause_xy: ya pausado, soft-park OK")

    def resume_xy_after_capture(self, reason: str = "") -> None:
        """Reanuda solo dual si estaba en captura. Trayectoria usa resume_trajectory()."""
        self._xy_capture_paused = False
        if self._dual_active and self._dual_paused:
            self._dual_paused = False
            if self._dual_worker is not None:
                self._dual_worker.pause(False)
            logger.info("[TestService] resume_xy_after_capture (%s): dual ON", reason or "?")
        else:
            logger.debug(
                "[TestService] resume_xy_after_capture (%s): dual no aplica",
                reason or "?",
            )

    def stop_dual_control(self):
        """Detiene el control dual vía halt_motion (método único)."""
        logger.info("=== TestService: DETENIENDO CONTROL DUAL ===")
        self.halt_motion("stop_dual_control")
        self.dual_control_stopped.emit()
        self.log_message.emit("⏹️ Control Dual DETENIDO (Freno Activo)")
        logger.info("TestService: Control dual detenido")
    
    def _execute_dual_control_step(self):
        """Control dual: potencia suave por eje (HOLD→0, FINE/COARSE con slew)."""
        try:
            if not self._dual_active or self._send_command is None or self._get_sensor_value is None:
                return
            
            # CRÍTICO: Si está pausado, NO enviar comandos (mantiene posición actual)
            if self._dual_paused:
                return
            
            current_time = time.time()
            Ts = current_time - self._dual_last_time
            self._dual_last_time = current_time
            now_m = time.perf_counter()

            error_a_um = 0.0
            error_b_um = 0.0
            if self._controller_a:
                sensor_adc = self._get_sensor_value(self._controller_a.sensor_key)
                if sensor_adc is not None:
                    error_a_um = position_error_um(self._dual_ref_a_um, sensor_adc, "x")
            if self._controller_b:
                sensor_adc = self._get_sensor_value(self._controller_b.sensor_key)
                if sensor_adc is not None:
                    error_b_um = position_error_um(self._dual_ref_b_um, sensor_adc, "y")

            pwm_a, st_a = self._dual_power.tick_axis(
                "a", error_a_um, Ts, self._controller_a, now_mono=now_m
            )
            pwm_b, st_b = self._dual_power.tick_axis(
                "b", error_b_um, Ts, self._controller_b, now_mono=now_m
            )

            # Potencia 0 por eje en HOLD; comando siempre (ceros parciales OK).
            self._send_command(f"A,{pwm_a},{pwm_b}")

            settled = self._dual_power.update_settle(now_m, ("a", "b"))
            if settled and not self._dual_position_reached:
                self._dual_position_reached = True
                self._send_command("A,0,0")
                self.dual_position_reached.emit(
                    self._dual_ref_a_um,
                    self._dual_ref_b_um,
                    error_a_um,
                    error_b_um,
                )
                self.log_message.emit(
                    f"✅ POSICIÓN ALCANZADA (HOLD {self._dual_power.config.settle_ms:.0f}ms): "
                    f"A={self._dual_ref_a_um:.0f}µm (err={error_a_um:.1f}), "
                    f"B={self._dual_ref_b_um:.0f}µm (err={error_b_um:.1f})"
                )
            elif self._dual_position_reached and not settled:
                if st_a != DualAxisState.HOLD or st_b != DualAxisState.HOLD:
                    self._dual_position_reached = False
                    self.dual_position_lost.emit()
                    self.log_message.emit(
                        f"🔄 Posición perdida ({st_a.value}/{st_b.value}) — corrigiendo suave…"
                    )

            if (now_m - self._last_dual_ui_mono) >= 0.05:
                self._last_dual_ui_mono = now_m
                self.dual_position_update.emit(error_a_um, error_b_um, pwm_a, pwm_b)
            if (now_m - self._last_dual_term_mono) >= 0.5:
                self._last_dual_term_mono = now_m
                status = (
                    "✅"
                    if self._dual_position_reached
                    else ("⏳" if st_a == DualAxisState.HOLD and st_b == DualAxisState.HOLD else "🔄")
                )
                self.log_message.emit(
                    f"{status} A:{error_a_um:+.1f}µm[{st_a.value}] "
                    f"B:{error_b_um:+.1f}µm[{st_b.value}] "
                    f"PWM:({pwm_a},{pwm_b})"
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
    
    def start_trajectory(
        self,
        trajectory: list,
        tolerance_um: float = 25.0,
        pause_s: float = 2.0,
        auto_advance: bool = False,
        start_index: int = 0,
        point_timeout_s: float = DEFAULT_POINT_TIMEOUT_S,
    ) -> bool:
        """
        Inicia la ejecución de una trayectoria con control PI dual.
        
        Args:
            trajectory: Lista de puntos (x, y) en µm
            tolerance_um: Tolerancia de posición en µm
            pause_s: Pausa en cada punto en segundos
            auto_advance: Si True, avanza automáticamente después de pausa (TestTab).
                         Si False, espera comando explícito resume_trajectory (MicroscopyService).
            start_index: Índice 0-based desde el que empezar (reanudación).
            point_timeout_s: Máx. segundos cazando un punto; luego accept+avance con error.
            
        Returns:
            True si se inició correctamente
        """
        logger.info(f"=== TestService: INICIANDO TRAYECTORIA ({len(trajectory)} puntos) ===")
        logger.info(f"    Modo: {'AUTO-ADVANCE' if auto_advance else 'MANUAL (espera resume_trajectory)'}")
        logger.info(f"    start_index (0-based): {start_index}")
        
        if not trajectory:
            self.error_occurred.emit("Trayectoria vacía")
            return False
        
        if not self._send_command:
            self.error_occurred.emit("Callbacks de hardware no configurados")
            return False
        
        if not self._controller_a and not self._controller_b:
            self.error_occurred.emit("No hay controladores cargados")
            return False
        
        # Exclusión mutua: cortar traj/dual previos (un solo worker TX).
        if self._trajectory_active or self._dual_active:
            logger.warning(
                "[TestService] Preempt motion antes de trayectoria "
                "(traj=%s dual=%s)",
                self._trajectory_active,
                self._dual_active,
            )
            self.halt_motion("start_trajectory_preempt")
        self._motion_halted = False  # permitir actuación tras halt previo
        self._handoff_phase = None
        self._xy_capture_paused = False
        
        # Guardar configuración
        self._trajectory = list(trajectory)
        self._trajectory_config.tolerance_um = tolerance_um
        self._trajectory_config.pause_s = pause_s
        self._trajectory_config.point_timeout_s = max(
            0.5, min(120.0, float(point_timeout_s))
        )
        self._trajectory_auto_advance = auto_advance  # NUEVO: modo auto-advance
        
        # LOGGING DETALLADO para diagnóstico
        logger.info(f"[TestService] ⚙️  auto_advance configurado: {auto_advance}")
        logger.info(f"[TestService] ⚙️  pause_s configurado: {pause_s}s")
        logger.info(f"[TestService] ⚙️  tolerance_um configurado: {tolerance_um}µm")
        logger.info(
            "[TestService] ⚙️  point_timeout_s configurado: %.1fs",
            self._trajectory_config.point_timeout_s,
        )
        
        # DEBUG: Mostrar primeros puntos de la trayectoria
        logger.info(f"[DEBUG] Primeros 5 puntos de trayectoria:")
        for i in range(min(5, len(self._trajectory))):
            p = self._trajectory[i]
            logger.info(f"  Punto {i}: ({p[0]:.1f}, {p[1]:.1f})µm")
        
        # Índice de arranque (reanudación desde Camera / microscopía)
        n_pts = len(self._trajectory)
        start_index = max(0, min(int(start_index), n_pts - 1)) if n_pts else 0
        self._trajectory_index = start_index
        self._trajectory_active = True
        self._trajectory_paused = False  # CORRECCIÓN: Iniciar NO pausado para ir al primer punto
        self._trajectory_waiting = False
        self._point_accepted = False  # NUEVO: Flag para evitar múltiples aceptaciones del mismo punto
        
        # Resetear integrales y contadores
        self._dual_integral_a = 0.0
        self._dual_integral_b = 0.0
        self._dual_last_time = time.time()
        self._traj_settling_counter = 0
        self._traj_near_attempts = 0
        
        # Activar modo automático
        self._send_command('A,0,0')
        # Trayectoria siempre usa orquestación FOV (host+MCU); no checkbox legacy.
        self._step_config.enabled = True
        self._ensure_step_controller()

        if self.step_control_enabled:
            self._apply_fov_trajectory_policy(float(tolerance_um))
            tolerance_um = float(self._trajectory_config.tolerance_um)
            self._step_controller.reset_session()
            self._step_long_approach_active = False
            self._fov_host_retries = 0
            # Aviso mapa canónico Lab: A/X→sensor_2, B/Y→sensor_1
            ca, cb = self._controller_a, self._controller_b
            if ca and cb:
                sa, sb = ca.sensor_key, cb.sensor_key
                if sa != "sensor_2" or sb != "sensor_1":
                    self.log_message.emit(
                        f"   ⚠ Mapa sensores A={sa} B={sb} "
                        f"(canónico Lab: A→sensor_2, B→sensor_1)"
                    )
            self._prepare_step_transition()
            engage = float(getattr(self._step_config, "fine_engage_um", 90.0))
            from config.constants import mcu_supports_cz
            if mcu_supports_cz():
                cierre = (
                    f"MCU K(z) @ 50 kHz (SETTLED) ±{float(tolerance_um):.0f}µm"
                )
            else:
                cierre = (
                    f"host residual ≤ ±{float(tolerance_um):.0f}µm "
                    f"(Arduino: sin C(z)/SETTLED)"
                )
            self.log_message.emit(
                f"   Cierre FOV: approach PI·rampa ±{engage:.0f}→"
                f"±{self._step_config.long_approach_done_um:.0f}µm; {cierre}"
            )

        # Reloj de trayectoria en QThread (igual que dual): el QTimer(10) en GUI
        # se quedaba mudo bajo flood de telemetría (log 13:03: prepare sin ningún A).
        from config.constants import CONTROL_RATE_HZ
        if self._trajectory_worker is not None:
            self._trajectory_worker.stop()
        self._trajectory_worker = ControlWorker(
            tick=self._execute_trajectory_step,
            rate_hz=CONTROL_RATE_HZ,
            name="TrajectoryControlWorker",
        )
        self._trajectory_worker.start()
        
        self.trajectory_started.emit(len(trajectory))
        if start_index > 0:
            self.log_message.emit(
                f"🚀 Reanudando trayectoria desde punto {start_index + 1}/{n_pts}"
            )
        else:
            self.log_message.emit(f"🚀 Ejecutando trayectoria: {n_pts} puntos")
        self.log_message.emit(
            f"   Tolerancia: {tolerance_um}µm, Pausa: {pause_s}s, "
            f"Timeout punto: {self._trajectory_config.point_timeout_s:.1f}s"
        )
        
        return True
    
    def stop_trajectory(self):
        """Detiene la trayectoria vía halt_motion (método único)."""
        logger.info("=== TestService: DETENIENDO TRAYECTORIA ===")
        self.halt_motion("stop_trajectory")
        total = len(self._trajectory) if self._trajectory else 0
        self.trajectory_stopped.emit(self._trajectory_index + 1, total)
        self.log_message.emit(
            f"⏹️ Trayectoria detenida en punto {self._trajectory_index + 1}/{total} (Freno Activo)"
        )
    
    def pause_trajectory(self):
        """Pausa la trayectoria (mantiene el timer activo).
        
        Timer continúa ejecutándose para mantener posición activamente.
        """
        if not self._trajectory_active:
            return
        
        self._trajectory_paused = True
        logger.info("[TestService] Trayectoria pausada - manteniendo posición")
    
    def pause_dual_control(self):
        """Alias → pause_xy_for_capture (cubre traj y dual sin sleep)."""
        self.pause_xy_for_capture("pause_dual_control")
    
    def resume_dual_control(self):
        """Alias → resume_xy_after_capture."""
        self.resume_xy_after_capture("resume_dual_control")
    
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
    
    def _on_schedule_auto_advance(self, pause_ms: int) -> None:
        """Slot en hilo Qt: QTimer solo funciona aquí (no desde ControlWorker)."""
        QTimer.singleShot(max(0, int(pause_ms)), self._auto_advance_to_next_point)

    def _auto_advance_to_next_point(self):
        """Avanza automáticamente al siguiente punto (modo auto_advance)."""
        if not self._trajectory_active:
            return
        
        logger.info("[TestService] ▶️  Auto-avanzando al siguiente punto")
        
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
    
    def _arm_soft_approach(self, done_um: float) -> None:
        """Activa approach sucesivo: PI TF + rampa PWM (sin bang-bang)."""
        self._step_long_approach_active = True
        self._dual_integral_a = 0.0
        self._dual_integral_b = 0.0
        engage = float(getattr(self._step_config, "fine_engage_um", 90.0))
        ca, cb = self._controller_a, self._controller_b
        # Techo = U_max de la TF cargada (no slew_pwm=150 legacy ni u_run+25)
        umax_a = int(getattr(ca, "U_max", STITION_PWM_MAX) or STITION_PWM_MAX) if ca else STITION_PWM_MAX
        umax_b = int(getattr(cb, "U_max", STITION_PWM_MAX) or STITION_PWM_MAX) if cb else STITION_PWM_MAX
        slew = max(umax_a, umax_b, int(STITION_PWM_MIN))
        slew = min(int(STITION_PWM_MAX), slew)
        self._host_approach.reset(
            done_um,
            engage,
            slew_pwm=slew,
            kp_x=float(getattr(ca, "Kp", 10.0) or 10.0) if ca else 10.0,
            ki_x=float(getattr(ca, "Ki", 8.0) or 8.0) if ca else 8.0,
            kp_y=float(getattr(cb, "Kp", 12.0) or 12.0) if cb else 12.0,
            ki_y=float(getattr(cb, "Ki", 11.0) or 11.0) if cb else 11.0,
        )
        logger.info(
            "[TestService] Approach armado U_max=%d (ctrl A=%d B=%d) "
            "done=±%.0f engage=±%.0f",
            slew,
            umax_a,
            umax_b,
            float(done_um),
            engage,
        )

    def _tick_step_long_approach(self) -> bool:
        """Approach sucesivo: PI TF + rampa PWM → HOLD → handoff MCU."""
        if not self._trajectory:
            return False

        target_x, target_y = self._trajectory[self._trajectory_index]
        Ts = max(1e-4, time.time() - self._dual_last_time)
        self._dual_last_time = time.time()
        now_m = time.perf_counter()

        error_x_um = error_y_um = 0.0
        if self._step_controller is not None:
            x_um, y_um = self._step_controller.read_current_xy_um(
                self._controller_a, self._controller_b
            )
            error_x_um = target_x - x_um
            error_y_um = target_y - y_um
        else:
            if self._controller_a:
                sensor_adc = self._get_sensor_value(self._controller_a.sensor_key)
                if sensor_adc is not None:
                    error_x_um = position_error_um(target_x, sensor_adc, "x")
            if self._controller_b:
                sensor_adc = self._get_sensor_value(self._controller_b.sensor_key)
                if sensor_adc is not None:
                    error_y_um = position_error_um(target_y, sensor_adc, "y")

        done = float(self._host_approach.config.done_um)
        engage = float(self._host_approach.config.engage_um)
        inv_a = bool(getattr(self._controller_a, "invert", False))
        inv_b = bool(getattr(self._controller_b, "invert", False))

        pwm_a, st_a_lbl = self._host_approach.tick_axis(
            "x", error_x_um, Ts, invert=inv_a
        )
        pwm_b, st_b_lbl = self._host_approach.tick_axis(
            "y", error_y_um, Ts, invert=inv_b
        )

        # Un solo mensaje al pasar a banda FINE (sin rearmes).
        if (
            not self._host_approach.entered_fine
            and st_a_lbl != "SLEW"
            and st_b_lbl != "SLEW"
        ):
            self._host_approach.entered_fine = True
            logger.info(
                "[TestService] Approach FINE decelerado (engage=±%.0f→done=±%.0f) "
                "err X=%.1f Y=%.1f",
                engage,
                done,
                error_x_um,
                error_y_um,
            )
            ax = self._host_approach._ax("x")
            ay = self._host_approach._ax("y")
            self.log_message.emit(
                f"   ✓ Approach FINE H∞ "
                f"Kp A={ax.kp:.3f}/B={ay.kp:.3f} "
                f"Ki A={ax.ki:.2f}/B={ay.ki:.2f} "
                f"U_max={self._host_approach.config.umax} "
                f"u_piso={ax.u_run}/{ay.u_run} "
                f"→ handoff ±{done:.0f}µm…"
            )

        residual = max(abs(error_x_um), abs(error_y_um))
        approach_done = self._host_approach.update_settle(
            (st_a_lbl, st_b_lbl), Ts, residual_um=residual
        )

        if approach_done:
            self._step_long_approach_active = False
            pwm_a, pwm_b = 0, 0
            logger.info(
                "[TestService] Aproximación host completa (err X=%.1f Y=%.1f µm)",
                error_x_um,
                error_y_um,
            )

        self._send_command(f"A,{pwm_a},{pwm_b}")
        self._last_traj_pwm = (pwm_a, pwm_b)
        self._last_traj_pwm_mono = now_m

        if (now_m - self._last_traj_fb_mono) >= 0.05:
            self._last_traj_fb_mono = now_m
            self.trajectory_feedback.emit(
                target_x, target_y, error_x_um, error_y_um, False, False, 0
            )

        if (now_m - self._last_traj_term_mono) >= 0.5:
            self._last_traj_term_mono = now_m
            self._traj_log_pwm = (pwm_a, pwm_b)
            idx = self._trajectory_index + 1
            phase = "slew" if (st_a_lbl == "SLEW" or st_b_lbl == "SLEW") else "fine"
            sens = ""
            try:
                sb = getattr(self, "_sensor_buffer", None)
                ca, cb = self._controller_a, self._controller_b
                if sb is not None and ca is not None and cb is not None:
                    ax = sb.get_adc(ca.sensor_key)
                    ay = sb.get_adc(cb.sensor_key)
                    age_a = sb.age_ms(ca.sensor_key)
                    age_b = sb.age_ms(cb.sensor_key)
                    ax_s = f"{ax:.0f}" if ax is not None else "?"
                    ay_s = f"{ay:.0f}" if ay is not None else "?"
                    sens = (
                        f" | ADC {ca.sensor_key}={ax_s}({age_a:.0f}ms) "
                        f"{cb.sensor_key}={ay_s}({age_b:.0f}ms)"
                    )
            except Exception:
                sens = ""
            self.log_message.emit(
                f"🔄 Trayectoria P{idx} approach/{phase} | "
                f"err X={error_x_um:+.1f}µm[{st_a_lbl}] "
                f"Y={error_y_um:+.1f}µm[{st_b_lbl}] | "
                f"PWM=({pwm_a},{pwm_b}) | "
                f"engage<{engage:.0f}µm handoff<{done:.0f}µm{sens}"
            )

        return approach_done

    def _begin_handoff_after_approach(self) -> None:
        """Inicia handoff no bloqueante (coast → pre_arm → prepare_mcu_fine)."""
        if (
            not self.step_control_enabled
            or not self._trajectory
            or self._step_controller is None
        ):
            return
        idx = self._trajectory_index
        if idx >= len(self._trajectory):
            return
        target = self._trajectory[idx]
        cfg = self._step_config
        done = float(cfg.long_approach_done_um)
        abort_lim = done * float(cfg.handoff_abort_factor)
        prev_actual = self._step_controller.read_current_xy_um(
            self._controller_a, self._controller_b
        )
        dx = target[0] - prev_actual[0]
        dy = target[1] - prev_actual[1]
        dist = max(abs(dx), abs(dy))
        if dist > abort_lim:
            self._handoff_phase = None
            self._arm_soft_approach(done)
            logger.warning(
                "[TestService] Handoff abortado (Δ=%.0fµm > %.0fµm) — sigue host",
                dist,
                abort_lim,
            )
            return
        self._step_long_approach_active = False
        self._send_command("A,0,0")
        self._handoff_phase = "coast"
        self._handoff_deadline_mono = time.perf_counter() + float(cfg.handoff_coast_s)
        logger.info(
            "[TestService] Handoff coast %.0fms punto %d residual=%.0fµm",
            float(cfg.handoff_coast_s) * 1000.0,
            idx + 1,
            dist,
        )

    def _tick_handoff(self) -> bool:
        """Avanza fases de handoff sin sleep. True si sigue en handoff."""
        if self._handoff_phase is None:
            return False
        if (
            not self._trajectory
            or self._step_controller is None
            or self._trajectory_index >= len(self._trajectory)
        ):
            self._handoff_phase = None
            return False
        now = time.perf_counter()
        if now < float(self._handoff_deadline_mono):
            return True

        cfg = self._step_config
        idx = self._trajectory_index
        target = self._trajectory[idx]
        done = float(cfg.long_approach_done_um)
        abort_lim = done * float(cfg.handoff_abort_factor)

        if self._handoff_phase == "coast":
            prev_actual = self._step_controller.read_current_xy_um(
                self._controller_a, self._controller_b
            )
            dx = target[0] - prev_actual[0]
            dy = target[1] - prev_actual[1]
            dist = max(abs(dx), abs(dy))
            if dist > abort_lim:
                self._handoff_phase = None
                self._arm_soft_approach(done)
                logger.warning(
                    "[TestService] Handoff post-coast abortado (Δ=%.0fµm) — sigue host",
                    dist,
                )
                return False
            self._send_command("A,0,0")
            self._handoff_phase = "pre_arm"
            self._handoff_deadline_mono = now + float(cfg.handoff_pre_arm_coast_s)
            return True

        if self._handoff_phase == "pre_arm":
            prev_actual = self._step_controller.read_current_xy_um(
                self._controller_a, self._controller_b
            )
            dx = target[0] - prev_actual[0]
            dy = target[1] - prev_actual[1]
            dist = max(abs(dx), abs(dy))
            if dist > abort_lim:
                self._handoff_phase = None
                self._arm_soft_approach(done)
                logger.warning(
                    "[TestService] Handoff pre-arm abortado (Δ=%.0fµm) — sigue host",
                    dist,
                )
                return False
            tol = float(cfg.tol_fov_um)
            # Si el host ya dejó residual ≤ tol: aceptar SIN MCU (evita spoil ±UMIN),
            # pero solo si el paso de malla ya se cubrió (anti multi-punto en 1 XY).
            if dist <= tol:
                cov_ok, cov = self._fov_step_coverage_ok(
                    idx, prev_actual, target, tol
                )
                if not cov_ok:
                    if self._fov_cover_timed_out():
                        tmo = self._point_timeout_s()
                        status = (
                            f"⚠️ cover t/o {tmo:.0f}s "
                            f"travel={cov['travel_um']:.0f}/"
                            f"{cov['delta_nominal_um']:.0f} "
                            f"res={dist:.0f}µm"
                        )
                        logger.error(
                            "[TestService] Host-stable FORCE ACCEPT P%d tras %.0fs: "
                            "travel=%.1f Δnom=%.1f residual=%.1f — avanzo con error",
                            idx + 1,
                            tmo,
                            cov["travel_um"],
                            cov["delta_nominal_um"],
                            dist,
                        )
                        self.log_message.emit(
                            f"   ⚠ Punto {idx + 1}: timeout cobertura "
                            f"({tmo:.0f}s) travel "
                            f"{cov['travel_um']:.0f}/{cov['delta_nominal_um']:.0f}µm "
                            f"— avanzo con error"
                        )
                        if not self._point_accepted:
                            self._accept_trajectory_point(
                                target[0],
                                target[1],
                                dx,
                                dy,
                                status,
                                point_result=None,
                                force_cover=True,
                            )
                            self._enrich_host_stable_snapshot(
                                target[0],
                                target[1],
                                float(dist),
                                float(dx),
                                float(dy),
                            )
                        return False
                    self._emit_cover_deny_throttled(
                        f"   ⚠ Punto {idx + 1}: residual {dist:.0f}µm ≤ tol pero "
                        f"travel {cov['travel_um']:.0f}≪paso "
                        f"{cov['delta_nominal_um']:.0f}µm — no aceptar",
                        f"[TestService] Host-stable BLOQUEADO punto {idx + 1}: "
                        f"residual={dist:.1f}µm travel={cov['travel_um']:.1f} "
                        f"Δnom={cov['delta_nominal_um']:.1f}",
                    )
                    # Forzar cierre MCU hacia el target (no aceptar en el XY anterior)
                    # cae al prepare_mcu_fine debajo
                else:
                    self._handoff_phase = None
                    self._send_command("A,0,0")
                    try:
                        self._send_command("N")
                    except Exception:
                        pass
                    logger.info(
                        "[TestService] Host-stable accept punto %d residual=%.1fµm "
                        "(≤tol %.1f) travel=%.1f Δnom=%.1f",
                        idx + 1,
                        dist,
                        tol,
                        cov["travel_um"],
                        cov["delta_nominal_um"],
                    )
                    self.log_message.emit(
                        f"   ✓ Approach host OK — residual {dist:.1f}µm ≤ tol {tol:.0f}µm "
                        f"(sin MCU)"
                    )
                    if not self._point_accepted:
                        self._accept_trajectory_point(
                            target[0],
                            target[1],
                            dx,
                            dy,
                            f"✅ Host OK ({dist:.0f}µm)",
                            point_result=None,
                        )
                        self._enrich_host_stable_snapshot(
                            target[0], target[1], float(dist), float(dx), float(dy)
                        )
                    return False
            nominal_prev = self._trajectory[idx - 1] if idx > 0 else prev_actual
            move_dir_x, move_dir_y = self._movement_direction(idx)
            backlash_dx, backlash_dy = 0.0, 0.0
            backlash = getattr(self, "_backlash_correction", None)
            if backlash is not None:
                backlash_dx, backlash_dy = backlash.delta_for_direction(
                    move_dir_x, move_dir_y
                )
            self._handoff_phase = None
            self._step_controller.prepare_mcu_fine(
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
                "[TestService] FOV MCU-fine post-approach (%d) Δ=(%.1f,%.1f)µm",
                idx + 1,
                dx,
                dy,
            )
            self.log_message.emit(
                f"   ✓ Approach host → MCU soft [{STITION_PWM_MIN},{STITION_PWM_MAX}] "
                f"(residual {dist:.0f}µm → tol ±{tol:.0f}µm)"
            )
            return False

        self._handoff_phase = None
        return False

    def _execute_trajectory_step_step_mode(self) -> None:
        """Un tick: host approach y/o FOV+MCU C(z)."""
        if self._step_controller is None or not self._trajectory:
            return

        # Timeout de punto: aplica también en approach (no solo cobertura FOV)
        if not self._point_accepted and self._fov_cover_timed_out():
            if self._step_long_approach_active:
                self._force_accept_point_timeout("approach")
                return
            if self._handoff_phase is not None:
                self._force_accept_point_timeout("handoff")
                return

        if self._handoff_phase is not None:
            self._tick_handoff()
            if not self._point_accepted and self._fov_cover_timed_out():
                self._force_accept_point_timeout("handoff")
            return

        if self._step_long_approach_active:
            if self._tick_step_long_approach():
                self._begin_handoff_after_approach()
            elif not self._point_accepted and self._fov_cover_timed_out():
                self._force_accept_point_timeout("approach")
            return

        out = self._step_controller.tick()
        if not self._point_accepted and self._fov_cover_timed_out():
            self._force_accept_point_timeout("fov_verify")
            return
        now_m = time.perf_counter()
        # Labels UI ~20 Hz; terminal ~2 Hz. El tick MCU/FOV sigue @ CONTROL_RATE.
        if (now_m - self._last_traj_fb_mono) >= 0.05:
            self._last_traj_fb_mono = now_m
            self.trajectory_feedback.emit(
                out.feedback_target_x,
                out.feedback_target_y,
                out.error_x_um,
                out.error_y_um,
                out.lock_x,
                out.lock_y,
                out.settling,
            )
        if (now_m - self._last_traj_term_mono) >= 0.5:
            self._last_traj_term_mono = now_m
            idx = self._trajectory_index + 1
            phase = getattr(out.phase, "value", str(out.phase))
            mcu_st = (getattr(out, "mcu_state", None) or "").strip() or "?"
            self.log_message.emit(
                f"🔄 Trayectoria P{idx} {phase} | "
                f"err X={out.error_x_um:+.1f} Y={out.error_y_um:+.1f}µm | "
                f"mcu={mcu_st} settle={out.settling} "
                f"t={getattr(out, 'settle_ms', 0):.0f}/"
                f"{float(self._step_config.fov_settle_ms):.0f}ms"
            )

        if out.point_failed:
            idx = self._trajectory_index + 1
            # Re-aproximación host (TF/PI) en vez de abortar con mensaje legacy.
            if self._fov_host_retries < 2:
                self._fov_host_retries += 1
                self._arm_soft_approach(float(self._step_config.long_approach_done_um))
                self.log_message.emit(
                    f"   ↻ FOV no cerró — re-aproximación host suave "
                    f"({self._fov_host_retries}/2) punto {idx}…"
                )
                logger.warning(
                    "[TestService] FOV fail → re-approach host punto %d (%d/2)",
                    idx,
                    self._fov_host_retries,
                )
                return
            self.error_occurred.emit(f"Punto {idx}: FOV no convergió")
            self.log_message.emit(f"❌ Punto {idx}: FOV no convergió tras reintentos")
            self.stop_trajectory()
            return

        if out.point_complete:
            if self._point_accepted:
                return
            self._fov_host_retries = 0
            target = self._trajectory[self._trajectory_index]
            result = self._step_controller.last_point_result
            t_move = result.t_move_ms if result else 0.0
            status = f"✅ FOV OK ({t_move:.0f}ms)"
            # Preferir residual del instante de aceptación (no una lectura posterior).
            accept_err = getattr(self._step_controller, "_last_fov_accept_err", None)
            if accept_err is not None:
                err_x, err_y = float(accept_err[0]), float(accept_err[1])
            else:
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
        """Techo PWM base agresivo (AUTO). MANUAL: U_max completo."""
        base_umax = self._controller_a.U_max if axis == "x" else self._controller_b.U_max
        if not self._trajectory_auto_advance:
            return float(base_umax)
        ae = abs(float(error_um))
        if ae > 150:
            return float(base_umax)
        if ae > 80:
            return max(150.0, float(base_umax) * 0.90)
        if ae > 40:
            return max(120.0, float(base_umax) * 0.75)
        return max(80.0, float(base_umax) * 0.55)
    
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

            # Sin step_control no hay camino canonico FOV: abortar (no fallback PI).
            logger.error(
                "[TestService] Trayectoria requiere step_control (FOV unico); abortando"
            )
            self.log_message.emit(
                "Trayectoria abortada: step_control obligatorio (sin fallback PI)"
            )
            self.halt_motion("trajectory_requires_step_control")
            return

        except Exception as e:
            logger.error(f"TestService: Error en trayectoria: {e}")
    
    def _enrich_host_stable_snapshot(
        self,
        target_x: float,
        target_y: float,
        dist_um: float,
        residual_x_um: float,
        residual_y_um: float,
    ) -> None:
        """Rellena n_steps/point_steps cuando el accept fue host-stable (sin MCU FOV)."""
        snap = self._last_accepted_snapshot
        if snap is None or snap.point_steps:
            return
        snap.n_steps = 1
        snap.fov_verify_passed = True
        snap.point_steps = [
            {
                "axis": "xy",
                "delta_um": round(float(dist_um), 3),
                "target_x_um": round(float(target_x), 3),
                "target_y_um": round(float(target_y), 3),
                "duration_ms": round(float(snap.t_move_ms), 1),
                "error_um": round(float(dist_um), 3),
                "residual_x_um": round(float(residual_x_um), 3),
                "residual_y_um": round(float(residual_y_um), 3),
                "status": "host_stable",
                "retries": 0,
                "pwm_max": 0,
            }
        ]
        logger.info(
            "[TestService] Snapshot host-stable enriquecido: dist=%.1fµm "
            "residual=(%+.1f,%+.1f)µm",
            dist_um,
            residual_x_um,
            residual_y_um,
        )

    def _accept_trajectory_point(
        self,
        target_x: float,
        target_y: float,
        error_x: float,
        error_y: float,
        status: str,
        point_result: Optional[PointTransitionResult] = None,
        force_cover: bool = False,
    ):
        """Acepta el punto actual y PAUSA o AVANZA según modo.
        
        Si auto_advance=True (TestTab): Pausa temporal y avanza automáticamente.
        Si auto_advance=False (MicroscopyService): Pausa indefinida esperando resume_trajectory().
        force_cover: salta chequeo de malla (timeout / accept con error).
        """
        # CRÍTICO: Evitar múltiples aceptaciones del mismo punto
        if self._point_accepted:
            logger.warning(f"[TestService] Punto {self._trajectory_index + 1} ya fue aceptado - ignorando llamada duplicada")
            return

        # No aceptar si aún no se cubrió el paso de malla (tol holgada vs FOV)
        if not force_cover:
            try:
                if self._step_controller is not None:
                    actual_xy = self._step_controller.read_current_xy_um(
                        self._controller_a, self._controller_b
                    )
                else:
                    actual_xy = (target_x - error_x, target_y - error_y)
                cov_ok, cov = self._fov_step_coverage_ok(
                    self._trajectory_index,
                    actual_xy,
                    (target_x, target_y),
                    float(getattr(self._step_config, "tol_fov_um", 25.0)),
                )
                if not cov_ok:
                    if self._fov_cover_timed_out():
                        tmo = self._point_timeout_s()
                        status = (
                            f"⚠️ cover t/o {tmo:.0f}s "
                            f"travel={cov['travel_um']:.0f}/"
                            f"{cov['delta_nominal_um']:.0f} "
                            f"err=({error_x:+.0f},{error_y:+.0f})"
                        )
                        logger.error(
                            "[TestService] ACCEPT FORCE P%d tras %.0fs: "
                            "travel=%.1f Δnom=%.1f tol=%.1f — avanzo con error",
                            self._trajectory_index + 1,
                            tmo,
                            cov["travel_um"],
                            cov["delta_nominal_um"],
                            cov["tol_um"],
                        )
                        self.log_message.emit(
                            f"   ⚠ Punto {self._trajectory_index + 1}: "
                            f"timeout cobertura ({tmo:.0f}s) "
                            f"travel {cov['travel_um']:.0f}/"
                            f"{cov['delta_nominal_um']:.0f}µm "
                            f"err=({error_x:+.0f},{error_y:+.0f})µm — avanzo"
                        )
                        force_cover = True
                    else:
                        self._emit_cover_deny_throttled(
                            f"   ✗ Punto {self._trajectory_index + 1}: "
                            f"sin cobertura FOV (travel {cov['travel_um']:.0f} / "
                            f"paso {cov['delta_nominal_um']:.0f}µm) — continúo",
                            f"[TestService] ACCEPT DENEGADO P"
                            f"{self._trajectory_index + 1}: travel="
                            f"{cov['travel_um']:.1f} Δnom="
                            f"{cov['delta_nominal_um']:.1f} tol={cov['tol_um']:.1f}",
                        )
                        self._point_accepted = False
                        self._trajectory_paused = False
                        # Un solo re-arm por punto (evitar reset settle cada 10ms)
                        if self.step_control_enabled and not self._fov_cover_rearmed:
                            self._fov_cover_rearmed = True
                            self._prepare_step_transition(reset_cover_watch=False)
                        return
            except Exception as e:
                logger.warning("[TestService] Coverage check skip: %s", e)
        
        # Marcar punto como aceptado
        self._point_accepted = True
        self._handoff_phase = None
        
        # Soft-park (sin sleep en el worker de control — no congelar el reloj).
        if self._send_command:
            sc = self._step_controller
            if sc is not None and getattr(sc, "_fov_cz_armed", False):
                try:
                    sc._cz_soft_off()
                except Exception:
                    self._send_command("N")
            self._send_command("A,0,0")
        
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
        
        # Auditoría: 1 punto = 1 FOV (detectar multi-aceptación en mismo XY)
        try:
            actual = (target_x - error_x, target_y - error_y)
            if self._step_controller is not None:
                actual = self._step_controller.read_current_xy_um(
                    self._controller_a, self._controller_b
                )
            cov_ok, cov = self._fov_step_coverage_ok(
                self._trajectory_index,
                actual,
                (target_x, target_y),
                float(self._step_config.tol_fov_um),
            )
            logger.info(
                "[TestService] AUDIT FOV accept P%d target=(%.1f,%.1f) "
                "prev=%s actual=(%.1f,%.1f) Δnom=%.1f travel=%.1f "
                "min_travel=%.1f tol=%.1f cov_tol=%.1f cov_ok=%s reason=%s status=%s",
                self._trajectory_index + 1,
                target_x,
                target_y,
                cov.get("prev_nominal_xy"),
                float(actual[0]),
                float(actual[1]),
                cov.get("delta_nominal_um", 0.0),
                cov.get("travel_um", 0.0),
                cov.get("min_travel_um", 0.0),
                cov.get("tol_um", 0.0),
                cov.get("cov_tol_um", cov.get("tol_um", 0.0)),
                cov_ok,
                cov.get("reason"),
                status,
            )
            if not cov_ok:
                logger.error(
                    "[TestService] AUDIT: aceptación con cobertura insuficiente "
                    "(revisar tol vs FOV)"
                )
        except Exception as e:
            logger.debug("[TestService] AUDIT FOV accept skip: %s", e)

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
            
            # Programar avance en hilo Qt (no QTimer desde ControlWorker — log 13:12 atasco).
            pause_ms = int(self._trajectory_config.pause_s * 1000)
            self._schedule_auto_advance.emit(pause_ms)
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
    
    @property
    def is_trajectory_active(self) -> bool:
        """Retorna si hay una trayectoria en ejecución."""
        return self._trajectory_active
    
    @property
    def trajectory_progress(self) -> Tuple[int, int]:
        """Retorna (punto_actual, total_puntos)."""
        total = len(self._trajectory) if self._trajectory else 0
        return (self._trajectory_index + 1, total)

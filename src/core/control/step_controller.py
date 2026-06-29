"""Orquestador de pasos homogéneos mono-eje (máquina de estados)."""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

from config.constants import CALIBRATION_X, CALIBRATION_Y, SETTLING_CYCLES, adc_to_um, um_to_adc
from core.control.controller_config import ControllerConfig
from core.control.pwm_crit_estimator import PwmCritEstimator
from core.control.discrete_hinf_kz import CONTROL_TS_S, DiscreteKzState, step_pi_kz
from core.control.hinf_actuator import HinfActuator, HinfActuatorConfig, HinfAxisState
from core.control.sensor_buffer import SensorBuffer
from core.control.step_config import StepControlConfig
from core.control.step_decomposer import decompose_transition
from core.control.step_metrics import StepSessionMetrics
from core.control.step_types import (
    MeasuredStep,
    PointTransitionResult,
    StepControllerPhase,
    StepExecutionResult,
)

logger = logging.getLogger("MotorControl_L206")


@dataclass
class StepTickOutput:
    phase: StepControllerPhase
    pwm_a: int = 0
    pwm_b: int = 0
    error_x_um: float = 0.0
    error_y_um: float = 0.0
    lock_x: bool = False
    lock_y: bool = False
    settling: int = 0
    point_complete: bool = False
    point_failed: bool = False
    feedback_target_x: float = 0.0
    feedback_target_y: float = 0.0


class StepController:
    """Ejecuta cola de MeasuredStep con verify + dwell homogéneo."""

    def __init__(
        self,
        config: StepControlConfig,
        sensor_buffer: SensorBuffer,
        get_controller_a: Callable[[], Optional[ControllerConfig]],
        get_controller_b: Callable[[], Optional[ControllerConfig]],
        send_command: Callable[[str], None],
    ):
        self.config = config
        self.sensor_buffer = sensor_buffer
        self._get_controller_a = get_controller_a
        self._get_controller_b = get_controller_b
        self._send_command = send_command

        self.phase = StepControllerPhase.IDLE
        self.metrics = StepSessionMetrics()
        self.last_point_result: Optional[PointTransitionResult] = None

        self._queue: List[MeasuredStep] = []
        self._queue_index = 0
        self._point_index = 0
        self._nominal_xy: Tuple[float, float] = (0.0, 0.0)
        self._nominal_prev_xy: Tuple[float, float] = (0.0, 0.0)
        self._prev_xy: Tuple[float, float] = (0.0, 0.0)

        self._step_started_mono = 0.0
        self._dwell_until_mono = 0.0
        self._settling_counter = 0
        self._step_retries = 0
        self._point_step_results: List[StepExecutionResult] = []
        self._transition_started_mono = 0.0
        self._step_pwm_max = 0

        self._integral_a = 0.0
        self._integral_b = 0.0
        self._last_axis: Optional[str] = None
        self._last_time = time.time()
        self._stall_log_counter = 0
        self._band_latched = False
        self._was_in_band = False
        self._last_hold_sent_mono = 0.0
        self._last_err_adc_x = 0.0
        self._last_err_adc_y = 0.0

        self._hinf_state_a = HinfAxisState()
        self._hinf_state_b = HinfAxisState()
        self._fov_settling_counter = 0
        self._fov_verify_started_mono = 0.0
        self._fov_verify_log_counter = 0
        self._fov_verify_ticks = 0
        self._fov_active_axis: Optional[str] = None
        self._fov_advance_axes: List[str] = []
        self._fov_advance_axis_idx = 0
        self._fov_priority_settling = 0
        self._fov_locked_axes: set[str] = set()
        self._move_dir_x = 0
        self._move_dir_y = 0
        self._backlash_dx_um = 0.0
        self._backlash_dy_um = 0.0
        self._kz_state_a = DiscreteKzState()
        self._kz_state_b = DiscreteKzState()
        self._fov_creep_cd_x = 0
        self._fov_creep_cd_y = 0
        self._pwm_crit = PwmCritEstimator(pwm_cap=config.step_hinf_pwm_min)
        self._last_control_telemetry_epoch = 0
        self._actuator = HinfActuator(
            HinfActuatorConfig(
                deadzone_um=config.deadzone_um(),
                pwm_min=0,
                use_integral=config.step_use_integral,
            )
        )

    @property
    def _hinf_native(self) -> bool:
        return self.config.is_hinf_native

    def _sync_actuator_config(self) -> None:
        self._actuator.config = HinfActuatorConfig(
            deadzone_um=self.config.deadzone_um(),
            pwm_min=0,
            use_integral=self.config.step_use_integral,
        )
        self._pwm_crit.pwm_cap = self.config.step_hinf_pwm_min

    def reset_session(self) -> None:
        self.phase = StepControllerPhase.IDLE
        self.metrics = StepSessionMetrics()
        self.last_point_result = None
        self._queue.clear()
        self._queue_index = 0
        self._point_step_results.clear()
        self._integral_a = 0.0
        self._integral_b = 0.0
        self._last_axis = None
        self._band_latched = False
        self._was_in_band = False
        self._last_err_adc_x = 0.0
        self._last_err_adc_y = 0.0
        self._hinf_state_a = HinfAxisState()
        self._hinf_state_b = HinfAxisState()
        self._fov_settling_counter = 0
        self._fov_verify_started_mono = 0.0
        self._fov_verify_log_counter = 0
        self._fov_verify_ticks = 0
        self._fov_active_axis = None
        self._fov_advance_axes = []
        self._fov_advance_axis_idx = 0
        self._fov_priority_settling = 0
        self._fov_locked_axes = set()
        self._move_dir_x = 0
        self._move_dir_y = 0
        self._backlash_dx_um = 0.0
        self._backlash_dy_um = 0.0
        self._kz_state_a = DiscreteKzState()
        self._kz_state_b = DiscreteKzState()
        self._fov_creep_cd_x = 0
        self._fov_creep_cd_y = 0
        self._pwm_crit.reset()
        self._last_control_telemetry_epoch = 0

    def _decay_fov_creep_cooldown(self) -> None:
        if self._fov_creep_cd_x > 0:
            self._fov_creep_cd_x -= 1
        if self._fov_creep_cd_y > 0:
            self._fov_creep_cd_y -= 1

    def _fov_creep_cooldown(self, axis: str) -> int:
        return self._fov_creep_cd_x if axis == "x" else self._fov_creep_cd_y

    def _set_fov_creep_cooldown(self, axis: str, ticks: int) -> None:
        if axis == "x":
            self._fov_creep_cd_x = ticks
        else:
            self._fov_creep_cd_y = ticks

    def _reset_control_telemetry_epoch(self) -> None:
        self._last_control_telemetry_epoch = 0

    @staticmethod
    def _sensor_lsb_um(axis: str) -> float:
        return CALIBRATION_X["slope"] if axis == "x" else CALIBRATION_Y["slope"]

    def _fov_tol_um_axis(self, axis: str) -> float:
        """Tolerancia de cierre por eje — respeta ``tol_fov_um`` de configuración."""
        return max(self.config.tol_fov_um, self._sensor_lsb_um(axis))

    def _fov_adc_tol_axis(self, axis: str) -> int:
        """Cuentas ADC equivalentes a ``tol_fov_um`` (mínimo 1 LSB)."""
        lsb = self._sensor_lsb_um(axis)
        return max(1, int(math.ceil(self._fov_tol_um_axis(axis) / lsb)))

    def _fov_adc_tol(self) -> int:
        return max(self._fov_adc_tol_axis("x"), self._fov_adc_tol_axis("y"))

    def _fov_actuator_deadzone_um(self) -> float:
        """Deadzone actuador FOV = resolución cuantizada del peor eje."""
        return max(self._sensor_lsb_um("x"), self._sensor_lsb_um("y"))

    def _consume_fresh_telemetry(self) -> bool:
        """
        True si hay telemetría nueva y fresca desde el último tick de control.

        Evita actuar varias veces sobre la misma muestra ADC @ 100 Hz.
        """
        if not self._hinf_native:
            return True
        epoch = self.sensor_buffer.update_count
        if epoch == self._last_control_telemetry_epoch:
            return False
        ctrl_a = self._get_controller_a()
        ctrl_b = self._get_controller_b()
        if ctrl_a is None or ctrl_b is None:
            return False
        max_age = self.config.sensor_control_max_age_ms
        if not self.sensor_buffer.is_fresh(ctrl_a.sensor_key, max_age):
            return False
        if not self.sensor_buffer.is_fresh(ctrl_b.sensor_key, max_age):
            return False
        self._last_control_telemetry_epoch = epoch
        return True

    def _observe_pwm_crit(self, pwm_a: int, pwm_b: int) -> None:
        ctrl_a = self._get_controller_a()
        ctrl_b = self._get_controller_b()
        if pwm_a != 0 and ctrl_a is not None:
            self._pwm_crit.observe(
                "x", pwm_a, self.sensor_buffer.get_adc(ctrl_a.sensor_key)
            )
        if pwm_b != 0 and ctrl_b is not None:
            self._pwm_crit.observe(
                "y", pwm_b, self.sensor_buffer.get_adc(ctrl_b.sensor_key)
            )

    def _apply_pwm_crit_floor(self, pwm_a: int, pwm_b: int) -> Tuple[int, int]:
        if not self._hinf_native:
            return pwm_a, pwm_b
        if pwm_a != 0:
            pwm_a = self._pwm_crit.apply_floor("x", pwm_a)
        if pwm_b != 0:
            pwm_b = self._pwm_crit.apply_floor("y", pwm_b)
        return pwm_a, pwm_b

    def _compute_advance_axis_order(self) -> List[str]:
        """
        Ejes con Δ **nominal FOV** en orden de avance (Y→X o X→Y).

        Usa grid nominal, no posición real previa, para priorizar el eje de avance
        (p. ej. solo X en filas).
        """
        x0, y0 = self._nominal_prev_xy
        x1, y1 = self._nominal_xy
        dx = x1 - x0
        dy = y1 - y0
        axes: List[str] = []
        if self.config.axis_order == "x_then_y":
            if abs(dx) > 1.0:
                axes.append("x")
            if abs(dy) > 1.0:
                axes.append("y")
        else:
            if abs(dy) > 1.0:
                axes.append("y")
            if abs(dx) > 1.0:
                axes.append("x")
        if not axes:
            axes = ["y", "x"] if self.config.axis_order.startswith("y") else ["x", "y"]
        return axes

    def _fov_secondary_axis(self) -> str:
        """Eje de ajuste fino tras completar avance (no alternar con eje principal)."""
        order = self._fov_advance_axes
        if len(order) == 1:
            return "y" if order[0] == "x" else "x"
        x0, y0 = self._nominal_prev_xy
        dx = abs(self._nominal_xy[0] - x0)
        dy = abs(self._nominal_xy[1] - y0)
        dominant = "x" if dx >= dy else "y"
        return "y" if dominant == "x" else "x"

    def _init_fov_advance_plan(self) -> None:
        self._fov_advance_axes = self._compute_advance_axis_order()
        self._fov_advance_axis_idx = 0
        self._fov_priority_settling = 0
        self._fov_locked_axes = set()
        self._fov_active_axis = None
        self._kz_state_a = DiscreteKzState()
        self._kz_state_b = DiscreteKzState()
        self._fov_creep_cd_x = 0
        self._fov_creep_cd_y = 0
        if self._fov_advance_axes:
            self._set_fov_active_axis(self._fov_advance_axes[0])

    def _fov_ref_um(self) -> Tuple[float, float]:
        """Setpoint FOV con corrección de backlash según dirección nominal."""
        nx, ny = self._nominal_xy
        return nx + self._backlash_dx_um, ny + self._backlash_dy_um

    def _read_fov_error_adc(self) -> Tuple[int, int, float, float, float, float]:
        """
        Errores ADC y µm.

        err_ctrl_um = (ref_adc − adc)·slope — ley K(z).
        err_traj_um = ref_um − actual_um — dirección de avance / lock.
        """
        ctrl_a = self._get_controller_a()
        ctrl_b = self._get_controller_b()
        ref_x, ref_y = self._fov_ref_um()
        ref_adc_x = int(round(um_to_adc(ref_x, axis="x")))
        ref_adc_y = int(round(um_to_adc(ref_y, axis="y")))
        adc_x = self.sensor_buffer.get_adc(ctrl_a.sensor_key) if ctrl_a else None
        adc_y = self.sensor_buffer.get_adc(ctrl_b.sensor_key) if ctrl_b else None
        err_adc_x = 0 if adc_x is None else ref_adc_x - adc_x
        err_adc_y = 0 if adc_y is None else ref_adc_y - adc_y
        err_ctrl_x = err_adc_x * CALIBRATION_X["slope"]
        err_ctrl_y = err_adc_y * CALIBRATION_Y["slope"]
        actual = self.read_current_xy_um(ctrl_a, ctrl_b)
        err_traj_x = ref_x - actual[0]
        err_traj_y = ref_y - actual[1]
        return err_adc_x, err_adc_y, err_ctrl_x, err_ctrl_y, err_traj_x, err_traj_y

    def _fov_move_dir(self, axis: str) -> int:
        return self._move_dir_x if axis == "x" else self._move_dir_y

    def _fov_approach_allows_pwm(self, axis: str, err_traj_um: float) -> bool:
        """Solo actuar desde el lado de avance; eje LOCKED nunca PWM."""
        if axis in self._fov_locked_axes:
            return False
        tol = self._fov_tol_um_axis(axis)
        move_dir = self._fov_move_dir(axis)
        if move_dir > 0:
            return err_traj_um > tol
        if move_dir < 0:
            return err_traj_um < -tol
        return abs(err_traj_um) > tol

    def _fov_can_lock_from_approach(self, axis: str, err_adc: int, err_traj_um: float) -> bool:
        """True si dentro de tol configurada y sin cruce de setpoint."""
        tol_um = self._fov_tol_um_axis(axis)
        if abs(err_traj_um) > tol_um:
            return False
        if abs(err_adc) > self._fov_adc_tol_axis(axis):
            return False
        move_dir = self._fov_move_dir(axis)
        if move_dir > 0 and err_traj_um < 0:
            return False
        if move_dir < 0 and err_traj_um > 0:
            return False
        return True

    def _lock_fov_axis(self, axis: str, err_adc: int) -> None:
        """Marca eje como LOCKED — PWM=0 permanente hasta POINT_COMPLETE."""
        if axis in self._fov_locked_axes:
            return
        self._fov_locked_axes.add(axis)
        logger.info(
            "[StepController] LOCK %s adc_err=%+d dir=%+d (sin re-actuación)",
            axis.upper(),
            err_adc,
            self._fov_move_dir(axis),
        )

    def _current_fov_priority_axis(self) -> Optional[str]:
        if self._fov_advance_axis_idx >= len(self._fov_advance_axes):
            return None
        return self._fov_advance_axes[self._fov_advance_axis_idx]

    def _update_fov_priority_handoff(
        self,
        err_adc_x: int,
        err_adc_y: int,
        err_traj_x: float,
        err_traj_y: float,
    ) -> None:
        """Avanza al siguiente eje cuando el actual cumple 1 LSB desde lado de avance."""
        axis = self._current_fov_priority_axis()
        if axis is None:
            return
        err_adc = err_adc_x if axis == "x" else err_adc_y
        err_traj = err_traj_x if axis == "x" else err_traj_y
        if self._fov_can_lock_from_approach(axis, err_adc, err_traj):
            self._fov_priority_settling += 1
            if self._fov_priority_settling >= SETTLING_CYCLES:
                self._lock_fov_axis(axis, err_adc)
                self._fov_advance_axis_idx += 1
                self._fov_priority_settling = 0
                nxt = self._current_fov_priority_axis()
                if nxt is not None:
                    self._set_fov_active_axis(nxt)
        else:
            self._fov_priority_settling = 0

    def _active_fov_correction_axis(
        self,
        err_adc_x: int,
        err_adc_y: int,
        err_traj_x: float,
        err_traj_y: float,
    ) -> str:
        """
        Avance nominal → LOCK por eje.

        Ejes LOCKED nunca vuelven a PWM. Sin recuperación de margen post-lock.
        """
        self._update_fov_priority_handoff(
            err_adc_x, err_adc_y, err_traj_x, err_traj_y
        )
        priority = self._current_fov_priority_axis()
        if priority is not None and priority not in self._fov_locked_axes:
            err_traj = err_traj_x if priority == "x" else err_traj_y
            if self._fov_approach_allows_pwm(priority, err_traj):
                return priority

        secondary = self._fov_secondary_axis()
        if secondary not in self._fov_locked_axes:
            err_traj = err_traj_x if secondary == "x" else err_traj_y
            if self._fov_approach_allows_pwm(secondary, err_traj):
                return secondary

        return secondary

    def prepare_transition(
        self,
        prev_xy: Tuple[float, float],
        next_xy: Tuple[float, float],
        point_index: int,
        backlash_x_um: float = 0.0,
        *,
        nominal_prev_xy: Optional[Tuple[float, float]] = None,
        backlash_dx_um: float = 0.0,
        backlash_dy_um: float = 0.0,
        move_dir_x: Optional[int] = None,
        move_dir_y: Optional[int] = None,
    ) -> None:
        self._sync_actuator_config()
        self._prev_xy = prev_xy
        self._nominal_xy = next_xy
        self._nominal_prev_xy = nominal_prev_xy if nominal_prev_xy is not None else prev_xy
        self._point_index = point_index
        if move_dir_x is not None and move_dir_y is not None:
            self._move_dir_x = move_dir_x
            self._move_dir_y = move_dir_y
        else:
            x0, y0 = self._nominal_prev_xy
            x1, y1 = self._nominal_xy
            dx_nom = x1 - x0
            dy_nom = y1 - y0
            self._move_dir_x = 0 if abs(dx_nom) < 1.0 else (1 if dx_nom > 0 else -1)
            self._move_dir_y = 0 if abs(dy_nom) < 1.0 else (1 if dy_nom > 0 else -1)
        self._backlash_dx_um = backlash_dx_um
        self._backlash_dy_um = backlash_dy_um
        self._reset_control_telemetry_epoch()
        self._queue = decompose_transition(prev_xy, next_xy, self.config, backlash_x_um)
        for i, step in enumerate(self._queue):
            step.transition_index = point_index
        self._queue_index = 0
        self._point_step_results.clear()
        self._transition_started_mono = time.perf_counter()
        self.last_point_result = None

        if not self._queue:
            if self._hinf_native:
                logger.info(
                    "[StepController] Punto %d: sin pasos (ya en objetivo) → FOV_VERIFY",
                    point_index + 1,
                )
                self._begin_fov_verify()
            else:
                self.phase = StepControllerPhase.POINT_COMPLETE
                logger.info(
                    "[StepController] Punto %d: sin pasos (ya en objetivo)",
                    point_index + 1,
                )
            return

        logger.info(
            "[StepController] Punto %d: %d pasos elementales hacia (%.1f, %.1f) "
            "[mode=%s tol_nom=%.1fµm tol_ef=%.1fµm deadzone≈%.1fµm pwm_cap=%d pwm_min=%d integral=%s] "
            "desde (%.1f, %.1f)",
            point_index + 1,
            len(self._queue),
            next_xy[0],
            next_xy[1],
            self.config.step_control_mode,
            self.config.tol_step_um,
            self._effective_tol_um(),
            self.config.deadzone_um(),
            self.config.step_pwm_cap,
            self.config.step_hinf_pwm_min if self._hinf_native else self.config.step_pwm_min,
            self.config.step_use_integral,
            prev_xy[0],
            prev_xy[1],
        )
        ctrl_a = self._get_controller_a()
        ctrl_b = self._get_controller_b()
        if self._hinf_native and ctrl_a and ctrl_b:
            logger.info(
                "[StepController] H∞ A: Kp=%.4f Ki=%.4f U_max=%.0f | "
                "B: Kp=%.4f Ki=%.4f U_max=%.0f | dual=%s brake=%s dwell=%dms",
                ctrl_a.Kp,
                ctrl_a.Ki,
                ctrl_a.U_max,
                ctrl_b.Kp,
                ctrl_b.Ki,
                ctrl_b.U_max,
                self._dual_axis_mode(),
                self.config.step_inter_step_brake,
                int(self.config.step_dwell_ms),
            )
        self._begin_current_step()

    def _current_step(self) -> Optional[MeasuredStep]:
        if self._queue_index >= len(self._queue):
            return None
        return self._queue[self._queue_index]

    def _begin_current_step(self, *, is_retry: bool = False) -> None:
        step = self._current_step()
        if step is None:
            if self._hinf_native:
                self._begin_fov_verify()
            else:
                self.phase = StepControllerPhase.POINT_COMPLETE
            return

        if not is_retry:
            if self._last_axis is not None and step.axis != self._last_axis:
                if step.axis == "x":
                    if self._hinf_native:
                        self._hinf_state_a = HinfAxisState(
                            integral=self._hinf_state_a.integral
                            * self.config.integral_carryover
                        )
                    else:
                        self._integral_a *= self.config.integral_carryover
                else:
                    if self._hinf_native:
                        self._hinf_state_b = HinfAxisState(
                            integral=self._hinf_state_b.integral
                            * self.config.integral_carryover
                        )
                    else:
                        self._integral_b *= self.config.integral_carryover
            self._step_retries = 0
            self._reset_control_telemetry_epoch()

        self.phase = StepControllerPhase.MOVING
        self._step_started_mono = time.perf_counter()
        self._settling_counter = 0
        self._step_pwm_max = 0
        self._last_time = time.time()
        self._stall_log_counter = 0
        self._band_latched = False
        self._was_in_band = False
        self._last_err_adc_x = 0.0
        self._last_err_adc_y = 0.0

    def _pwm_cap_for_step(self, step: MeasuredStep) -> float:
        ad = abs(step.delta_um)
        fine = float(self.config.step_pwm_cap)
        coarse = float(self.config.step_pwm_cap_coarse)
        if ad <= self.config.step_um * 1.5:
            return fine
        span = max(1.0, self.config.coarse_step_threshold_um * 4.0)
        t = min(1.0, (ad - self.config.step_um) / span)
        return fine + t * (coarse - fine)

    def _step_timeout_ms(self, step: MeasuredStep) -> float:
        ad = abs(step.delta_um)
        base = self.config.step_timeout_ms
        if ad <= self.config.step_um * 1.5:
            return base
        ratio = ad / max(1.0, self.config.step_um)
        return min(self.config.step_timeout_max_ms, base * max(1.0, ratio * 0.4))

    def _effective_tol_um(self) -> float:
        return self.config.effective_tol_step_um()

    def _in_acceptance_band(self, err_um: float) -> bool:
        tol = self._effective_tol_um()
        ae = abs(err_um)
        exit_tol = tol * self.config.tol_hysteresis_factor
        if self._band_latched:
            if ae >= exit_tol:
                self._band_latched = False
            return self._band_latched
        if ae < tol:
            self._band_latched = True
        return self._band_latched

    def _hold_position(self, force: bool = False) -> None:
        now = time.perf_counter()
        if not force and (now - self._last_hold_sent_mono) < (self.config.hold_resend_ms / 1000.0):
            return
        self._last_hold_sent_mono = now
        self._send_command("B")
        self._send_command("A,0,0")

    def _pi_gain_scale(self, error_um: float) -> Tuple[float, float]:
        """Escala Kp/Ki solo si step_use_full_hinf_gains=false (tuning legacy)."""
        if self.config.step_use_full_hinf_gains:
            return 1.0, 1.0
        ae = abs(error_um)
        span = max(self.config.step_um * 2.0, 1.0)
        if ae >= span:
            return 1.0, 1.0
        t = ae / span
        kp = max(self.config.approach_kp_scale, t)
        ki = max(self.config.approach_ki_scale, t * t)
        return kp, ki

    def _deadzone_for_error(self, error_um: float) -> int:
        if abs(error_um) <= self._effective_tol_um() * 2.0:
            return self.config.approach_deadzone_adc
        return self.config.deadzone_adc

    def _on_enter_acceptance_band(self, step: MeasuredStep) -> None:
        if step.axis == "x":
            self._integral_a = 0.0
            self._last_err_adc_x = 0.0
        else:
            self._integral_b = 0.0
            self._last_err_adc_y = 0.0

    def _pwm_limit(self, axis: str, error_um: float, step: Optional[MeasuredStep] = None) -> float:
        cap = self._pwm_cap_for_step(step) if step is not None else float(self.config.step_pwm_cap)
        ctrl = self._get_controller_a() if axis == "x" else self._get_controller_b()
        umax = float(ctrl.U_max) if ctrl else 150.0
        return min(cap, umax)

    @staticmethod
    def _enforce_pwm_floor(pwm: int, pwm_min: int) -> int:
        """Umbral mínimo físico del actuador (por debajo no hay movimiento útil)."""
        if pwm == 0:
            return 0
        if abs(pwm) < pwm_min:
            return pwm_min if pwm > 0 else -pwm_min
        return pwm

    def _sensor_has_reading(self, step: MeasuredStep) -> bool:
        ctrl = self._get_controller_a() if step.axis == "x" else self._get_controller_b()
        if ctrl is None:
            return False
        return self.sensor_buffer.get_adc(ctrl.sensor_key) is not None

    def _sensor_fresh(self, step: MeasuredStep) -> bool:
        ctrl = self._get_controller_a() if step.axis == "x" else self._get_controller_b()
        if ctrl is None:
            return False
        return self.sensor_buffer.is_fresh(ctrl.sensor_key, self.config.sensor_max_age_ms)

    def _ready_to_finalize_step(self, step: MeasuredStep) -> bool:
        """Condición final antes de cerrar el paso (más estricta que contar settling)."""
        if not self._sensor_has_reading(step):
            return False
        if not self._sensor_fresh(step):
            return False
        if self.config.use_arduino_settled and not self.sensor_buffer.is_settled():
            return False
        error_x, error_y, _, _ = self._read_error_um(step)
        err = error_x if step.axis == "x" else error_y
        return abs(err) <= self._effective_tol_um() * self.config.tol_hysteresis_factor

    def _log_stall_if_needed(self, step: MeasuredStep, err_um: float) -> None:
        self._stall_log_counter += 1
        if self._stall_log_counter % 100 != 0:
            return
        ctrl = self._get_controller_a() if step.axis == "x" else self._get_controller_b()
        age = self.sensor_buffer.age_ms(ctrl.sensor_key) if ctrl else float("inf")
        logger.warning(
            "[StepController] Paso %d eje %s: err=%.1fµm tol_ef=%.1f deadzone≈%.1fµm "
            "settling=%d/%d age=%.0fms latched=%s pwm_lim=%.0f",
            step.step_index + 1,
            step.axis.upper(),
            err_um,
            self._effective_tol_um(),
            self.config.deadzone_um(),
            self._settling_counter,
            SETTLING_CYCLES,
            age,
            self._band_latched,
            self._pwm_limit(step.axis, err_um, step),
        )

    def _read_error_um(self, step: MeasuredStep) -> Tuple[float, float, Optional[int], Optional[int]]:
        ctrl_a = self._get_controller_a()
        ctrl_b = self._get_controller_b()
        error_x = error_y = 0.0
        adc_x = adc_y = None

        ref_x = um_to_adc(step.target_x_um, axis="x")
        ref_y = um_to_adc(step.target_y_um, axis="y")

        if ctrl_a:
            adc_x = self.sensor_buffer.get_adc(ctrl_a.sensor_key)
            if adc_x is not None:
                error_x = (ref_x - adc_x) * CALIBRATION_X["slope"]
        if ctrl_b:
            adc_y = self.sensor_buffer.get_adc(ctrl_b.sensor_key)
            if adc_y is not None:
                error_y = (ref_y - adc_y) * CALIBRATION_Y["slope"]

        return error_x, error_y, adc_x, adc_y

    def _run_pi(self, step: MeasuredStep, Ts: float) -> Tuple[int, int, float, float]:
        ctrl_a = self._get_controller_a()
        ctrl_b = self._get_controller_b()
        pwm_a = pwm_b = 0
        error_x, error_y, _, _ = self._read_error_um(step)

        if step.axis == "x" and ctrl_a:
            ref_adc = um_to_adc(step.target_x_um, axis="x")
            adc = self.sensor_buffer.get_adc(ctrl_a.sensor_key)
            if adc is not None:
                err_adc = ref_adc - adc
                error_x = err_adc * CALIBRATION_X["slope"]
                deadzone = self._deadzone_for_error(error_x)
                kp_scale, ki_scale = self._pi_gain_scale(error_x)
                if not self.config.step_use_integral:
                    ki_scale = 0.0
                if self._last_err_adc_x * err_adc < 0:
                    self._integral_a = 0.0
                self._last_err_adc_x = err_adc
                if abs(err_adc) > deadzone:
                    self._integral_a += err_adc * Ts
                    pwm_base = (
                        ctrl_a.Kp * kp_scale * err_adc
                        + ctrl_a.Ki * ki_scale * self._integral_a
                    )
                    pwm_a = -int(pwm_base) if ctrl_a.invert else int(pwm_base)
                    umax = int(self._pwm_limit("x", error_x, step))
                    if abs(pwm_a) > umax:
                        self._integral_a -= err_adc * Ts
                        pwm_a = max(-umax, min(umax, pwm_a))
                    pwm_a = self._enforce_pwm_floor(pwm_a, self.config.step_pwm_min)
                    self._step_pwm_max = max(self._step_pwm_max, abs(pwm_a))
        elif step.axis == "y" and ctrl_b:
            ref_adc = um_to_adc(step.target_y_um, axis="y")
            adc = self.sensor_buffer.get_adc(ctrl_b.sensor_key)
            if adc is not None:
                err_adc = ref_adc - adc
                error_y = err_adc * CALIBRATION_Y["slope"]
                deadzone = self._deadzone_for_error(error_y)
                kp_scale, ki_scale = self._pi_gain_scale(error_y)
                if not self.config.step_use_integral:
                    ki_scale = 0.0
                if self._last_err_adc_y * err_adc < 0:
                    self._integral_b = 0.0
                self._last_err_adc_y = err_adc
                if abs(err_adc) > deadzone:
                    self._integral_b += err_adc * Ts
                    pwm_base = (
                        ctrl_b.Kp * kp_scale * err_adc
                        + ctrl_b.Ki * ki_scale * self._integral_b
                    )
                    pwm_b = -int(pwm_base) if ctrl_b.invert else int(pwm_base)
                    umax = int(self._pwm_limit("y", error_y, step))
                    if abs(pwm_b) > umax:
                        self._integral_b -= err_adc * Ts
                        pwm_b = max(-umax, min(umax, pwm_b))
                    pwm_b = self._enforce_pwm_floor(pwm_b, self.config.step_pwm_min)
                    self._step_pwm_max = max(self._step_pwm_max, abs(pwm_b))

        return pwm_a, pwm_b, error_x, error_y

    def _run_hinf_axis(
        self,
        active_axis: str,
        ref_x_um: float,
        ref_y_um: float,
        Ts: float,
    ) -> Tuple[int, int, float, float]:
        """H∞ sobre un solo eje; el otro PWM=0. Magnitud solo K(z) + piso aprendido."""
        ctrl_a = self._get_controller_a()
        ctrl_b = self._get_controller_b()
        pwm_a, pwm_b, error_x, error_y, self._hinf_state_a, self._hinf_state_b = (
            self._actuator.compute(
                ref_x_um,
                ref_y_um,
                ctrl_a,
                ctrl_b,
                self.sensor_buffer,
                self._hinf_state_a,
                self._hinf_state_b,
                Ts,
                dual_axis_mode="primary_only",
                primary_axis=active_axis,
            )
        )
        if active_axis == "x":
            pwm_b = 0
        else:
            pwm_a = 0
        pwm_a, pwm_b = self._apply_pwm_crit_floor(pwm_a, pwm_b)
        return pwm_a, pwm_b, error_x, error_y

    def _run_hinf(self, step: MeasuredStep, Ts: float) -> Tuple[int, int, float, float]:
        pwm_a, pwm_b, error_x, error_y = self._run_hinf_axis(
            step.axis,
            step.target_x_um,
            step.target_y_um,
            Ts,
        )
        self._step_pwm_max = max(self._step_pwm_max, abs(pwm_a), abs(pwm_b))
        return pwm_a, pwm_b, error_x, error_y

    def _set_fov_active_axis(self, axis: str) -> None:
        if axis == self._fov_active_axis:
            return
        if axis == "x":
            self._kz_state_a = DiscreteKzState()
            self._fov_creep_cd_x = 0
        else:
            self._kz_state_b = DiscreteKzState()
            self._fov_creep_cd_y = 0
        self._fov_active_axis = axis

    def _dual_axis_mode(self) -> str:
        """hinf_native: un solo eje activo; orchestrated: según config."""
        if self._hinf_native:
            return "primary_only"
        return self.config.step_dual_axis_mode

    def _fov_pwm_pulse(
        self, axis: str, err_ctrl_um: float, ctrl: ControllerConfig
    ) -> int:
        """Un pulso creep: signo de K(z), magnitud TF o piso crítico aprendido."""
        pwm, _, _ = step_pi_kz(
            err_ctrl_um,
            DiscreteKzState(),
            ctrl,
            deadzone_um=0.0,
            pwm_min=0,
            use_integral=False,
        )
        return self._pwm_crit.apply_floor(axis, pwm)

    def _run_kz_closure(
        self,
        active_axis: str,
        Ts: float = CONTROL_TS_S,
        *,
        use_integral: bool = False,
    ) -> Tuple[int, int, float, float]:
        """Cierre setpoint con K(z) @ 10 ms — magnitud solo TF (+ piso aprendido)."""
        ctrl_a = self._get_controller_a()
        ctrl_b = self._get_controller_b()
        deadzone = self._fov_actuator_deadzone_um()

        err_adc_x, err_adc_y, err_ctrl_x, err_ctrl_y, _, _ = self._read_fov_error_adc()
        err_adc = err_adc_x if active_axis == "x" else err_adc_y
        err_ctrl = err_ctrl_x if active_axis == "x" else err_ctrl_y

        pwm_a, pwm_b = 0, 0
        creep_adc = self.config.fov_creep_adc

        if abs(err_adc) <= self._fov_adc_tol():
            return 0, 0, err_ctrl_x, err_ctrl_y

        if self._fov_creep_cooldown(active_axis) > 0:
            return 0, 0, err_ctrl_x, err_ctrl_y

        if abs(err_adc) <= creep_adc:
            ctrl = ctrl_a if active_axis == "x" else ctrl_b
            if ctrl is None:
                return 0, 0, err_ctrl_x, err_ctrl_y
            pulse = self._fov_pwm_pulse(active_axis, err_ctrl, ctrl)
            if pulse != 0:
                self._set_fov_creep_cooldown(
                    active_axis, self.config.fov_creep_cooldown_ticks
                )
                if active_axis == "x":
                    pwm_a = pulse
                else:
                    pwm_b = pulse
            return pwm_a, pwm_b, err_ctrl_x, err_ctrl_y

        if active_axis == "x" and ctrl_a is not None:
            pwm_a, self._kz_state_a, err_ctrl_x = step_pi_kz(
                err_ctrl_x,
                self._kz_state_a,
                ctrl_a,
                deadzone_um=deadzone,
                pwm_min=0,
                use_integral=use_integral,
                Ts=Ts,
            )
            pwm_a = self._pwm_crit.apply_floor("x", pwm_a)
        elif active_axis == "y" and ctrl_b is not None:
            pwm_b, self._kz_state_b, err_ctrl_y = step_pi_kz(
                err_ctrl_y,
                self._kz_state_b,
                ctrl_b,
                deadzone_um=deadzone,
                pwm_min=0,
                use_integral=use_integral,
                Ts=Ts,
            )
            pwm_b = self._pwm_crit.apply_floor("y", pwm_b)

        return pwm_a, pwm_b, err_ctrl_x, err_ctrl_y

    def _run_hinf_to_nominal(self, Ts: float, *, active_axis: str) -> Tuple[int, int, float, float]:
        """FOV_VERIFY: K(z) + creep; sin Ki."""
        return self._run_kz_closure(active_axis, Ts=CONTROL_TS_S, use_integral=False)

    def _read_fov_residual_um(self) -> Tuple[float, float]:
        _, _, _, _, err_traj_x, err_traj_y = self._read_fov_error_adc()
        return err_traj_x, err_traj_y

    def _in_fov_band(self, err_adc_x: int, err_adc_y: int, err_traj_x: float, err_traj_y: float) -> bool:
        """Dentro de margen FOV — criterio único en µm (``tol_fov_um``)."""
        return (
            abs(err_traj_x) <= self._fov_tol_um_axis("x")
            and abs(err_traj_y) <= self._fov_tol_um_axis("y")
        )

    def _fov_sensors_ready(self) -> bool:
        ctrl_a = self._get_controller_a()
        ctrl_b = self._get_controller_b()
        if ctrl_a is None or ctrl_b is None:
            return False
        if self.sensor_buffer.get_adc(ctrl_a.sensor_key) is None:
            return False
        if self.sensor_buffer.get_adc(ctrl_b.sensor_key) is None:
            return False
        max_age = self.config.sensor_control_max_age_ms
        if not self.sensor_buffer.is_fresh(ctrl_a.sensor_key, max_age):
            return False
        if not self.sensor_buffer.is_fresh(ctrl_b.sensor_key, max_age):
            return False
        if self.config.use_arduino_settled and not self.sensor_buffer.is_settled():
            return False
        return True

    def _fov_tol_um(self) -> float:
        """Tolerancia de handoff (peor eje)."""
        return max(self._fov_tol_um_axis("x"), self._fov_tol_um_axis("y"))

    def _apply_fov_verify_actuator(self) -> None:
        """Deadzone FOV = 1 LSB; magnitud PWM solo K(z) + piso aprendido."""
        self._actuator.config = HinfActuatorConfig(
            deadzone_um=self._fov_actuator_deadzone_um(),
            pwm_min=0,
            use_integral=self.config.step_use_integral,
        )

    def _begin_fov_verify(self) -> None:
        self.phase = StepControllerPhase.FOV_VERIFY
        self._fov_settling_counter = 0
        self._fov_verify_ticks = 0
        self._fov_verify_log_counter = 0
        self._fov_verify_started_mono = time.perf_counter()
        self._last_time = time.time()
        self._reset_control_telemetry_epoch()
        self._apply_fov_verify_actuator()
        self._init_fov_advance_plan()
        err_adc_x, err_adc_y, err_ctrl_x, err_ctrl_y, err_traj_x, err_traj_y = (
            self._read_fov_error_adc()
        )
        order = "→".join(a.upper() for a in self._fov_advance_axes)
        ref_x, ref_y = self._fov_ref_um()
        logger.info(
            "[StepController] Punto %d FOV_VERIFY ref=(%.1f, %.1f) backlash=(%.1f, %.1f) "
            "residual_adc=(%+d, %+d) residual_traj=(%.1f, %.1f)µm tol_fov=%.1f avance=%s "
            "dir=(%+d,%+d) eje=%s",
            self._point_index + 1,
            ref_x,
            ref_y,
            self._backlash_dx_um,
            self._backlash_dy_um,
            err_adc_x,
            err_adc_y,
            err_traj_x,
            err_traj_y,
            self.config.tol_fov_um,
            order,
            self._move_dir_x,
            self._move_dir_y,
            (self._fov_active_axis or "?").upper(),
        )

    def _tick_fov_verify(self) -> StepTickOutput:
        out = StepTickOutput(phase=StepControllerPhase.FOV_VERIFY)
        out.feedback_target_x, out.feedback_target_y = self._fov_ref_um()
        self._fov_verify_ticks += 1
        self._decay_fov_creep_cooldown()

        err_adc_x, err_adc_y, err_ctrl_x, err_ctrl_y, err_traj_x, err_traj_y = (
            self._read_fov_error_adc()
        )
        out.error_x_um, out.error_y_um = err_traj_x, err_traj_y

        if self._in_fov_band(err_adc_x, err_adc_y, err_traj_x, err_traj_y):
            if self._fov_sensors_ready():
                self._fov_settling_counter += 1
                out.settling = self._fov_settling_counter
                out.pwm_a, out.pwm_b = 0, 0
                self._send_command("A,0,0")
                if self._fov_settling_counter >= SETTLING_CYCLES:
                    self._finish_point()
                    out.phase = StepControllerPhase.POINT_COMPLETE
                    out.point_complete = True
            else:
                out.settling = self._fov_settling_counter
            return out

        if not self._consume_fresh_telemetry():
            out.settling = self._fov_settling_counter
            return out

        self._last_time = time.time()

        self._fov_settling_counter = 0
        out.settling = 0
        active = self._active_fov_correction_axis(
            err_adc_x, err_adc_y, err_traj_x, err_traj_y
        )
        self._set_fov_active_axis(active)
        out.lock_x = active == "y" or "x" in self._fov_locked_axes
        out.lock_y = active == "x" or "y" in self._fov_locked_axes

        err_traj = err_traj_x if active == "x" else err_traj_y
        if active in self._fov_locked_axes or not self._fov_approach_allows_pwm(
            active, err_traj
        ):
            out.pwm_a, out.pwm_b = 0, 0
            self._send_command("A,0,0")
        else:
            pwm_a, pwm_b, error_x, error_y = self._run_hinf_to_nominal(
                CONTROL_TS_S, active_axis=active
            )
            out.pwm_a, out.pwm_b = pwm_a, pwm_b
            out.error_x_um, out.error_y_um = error_x, error_y
            self._send_command(f"A,{pwm_a},{pwm_b}")
            self._observe_pwm_crit(pwm_a, pwm_b)

        self._fov_verify_log_counter += 1
        if self._fov_verify_log_counter % 200 == 0:
            elapsed_ms = (time.perf_counter() - self._fov_verify_started_mono) * 1000.0
            logger.info(
                "[StepController] FOV_VERIFY punto %d eje %s: err_adc=(%+d,%+d) err_traj=(%.1f,%.1f)µm "
                "locked=%s crit_pwm=(%d,%d) creep_cd=(%d,%d) settling=%d/%d t=%.0fms",
                self._point_index + 1,
                active.upper(),
                err_adc_x,
                err_adc_y,
                err_traj_x,
                err_traj_y,
                sorted(self._fov_locked_axes),
                self._pwm_crit.effective_min("x"),
                self._pwm_crit.effective_min("y"),
                self._fov_creep_cd_x,
                self._fov_creep_cd_y,
                self._fov_settling_counter,
                SETTLING_CYCLES,
                elapsed_ms,
            )
        return out

    def _complete_current_step(self, status: str) -> None:
        step = self._current_step()
        if step is None:
            return
        error_x, error_y, adc_x, adc_y = self._read_error_um(step)
        err = error_x if step.axis == "x" else error_y
        adc = adc_x if step.axis == "x" else adc_y
        duration_ms = (time.perf_counter() - self._step_started_mono) * 1000.0

        result = StepExecutionResult(
            step=step,
            duration_ms=duration_ms,
            error_um=err,
            sensor_adc=adc,
            status=status,
            retries=self._step_retries,
            pwm_max=self._step_pwm_max,
        )
        self._point_step_results.append(result)
        self._last_axis = step.axis
        self._queue_index += 1

        dwell_ms = self.config.step_dwell_ms
        if self.config.use_temporal_padding:
            elapsed_ms = (time.perf_counter() - self._step_started_mono) * 1000.0
            pad = max(0.0, self.config.t_step_nominal_ms - elapsed_ms)
            dwell_ms = max(dwell_ms, pad)

        if self._hinf_native:
            if self.config.step_inter_step_brake:
                self._send_command("B")
                self._send_command("A,0,0")
        else:
            self._send_command("B")
            self._send_command("A,0,0")

        if dwell_ms <= 0:
            self._advance_after_step()
            return

        self._dwell_until_mono = time.perf_counter() + dwell_ms / 1000.0
        self.phase = StepControllerPhase.DWELL

        logger.info(
            "[StepController] Paso %d/%d eje %s status=%s err=%.1fµm %.0fms pwm_max=%d",
            step.step_index + 1,
            len(self._queue),
            step.axis.upper(),
            status,
            err,
            duration_ms,
            self._step_pwm_max,
        )

    def _advance_after_step(self) -> None:
        if self._queue_index >= len(self._queue):
            if self._hinf_native:
                self._begin_fov_verify()
            else:
                self._finish_point()
        else:
            self._begin_current_step()

    def _fail_point(self, reason: str) -> None:
        step = self._current_step()
        if step is not None:
            error_x, error_y, adc_x, adc_y = self._read_error_um(step)
            err = error_x if step.axis == "x" else error_y
            adc = adc_x if step.axis == "x" else adc_y
            duration_ms = (time.perf_counter() - self._step_started_mono) * 1000.0
            self._point_step_results.append(
                StepExecutionResult(
                    step=step,
                    duration_ms=duration_ms,
                    error_um=err,
                    sensor_adc=adc,
                    status="failed",
                    retries=self._step_retries,
                    pwm_max=self._step_pwm_max,
                )
            )
        t_move = (time.perf_counter() - self._transition_started_mono) * 1000.0
        self.last_point_result = PointTransitionResult(
            point_index=self._point_index,
            x_nominal_um=self._nominal_xy[0],
            y_nominal_um=self._nominal_xy[1],
            n_steps=len(self._point_step_results),
            t_move_ms=t_move,
            steps=list(self._point_step_results),
            status="failed",
            failed_step_index=step.step_index if step else None,
        )
        self.metrics.record_point(self.last_point_result)
        self.phase = StepControllerPhase.FAILED
        self._send_command("B")
        self._send_command("A,0,0")
        logger.warning("[StepController] Punto %d FALLÓ: %s", self._point_index + 1, reason)

    def _finish_point(self) -> PointTransitionResult:
        ctrl_a = self._get_controller_a()
        ctrl_b = self._get_controller_b()
        actual = self.read_current_xy_um(ctrl_a, ctrl_b)
        err_x = self._nominal_xy[0] - actual[0]
        err_y = self._nominal_xy[1] - actual[1]
        tol_fov = self._fov_tol_um() if self._hinf_native else self._effective_tol_um() * 2.0
        t_fov_verify_ms = 0.0
        if self._fov_verify_started_mono > 0:
            t_fov_verify_ms = (time.perf_counter() - self._fov_verify_started_mono) * 1000.0

        fov_passed = abs(err_x) <= tol_fov and abs(err_y) <= tol_fov
        if self._hinf_native and not fov_passed:
            logger.warning(
                "[StepController] Punto %d FOV_VERIFY residual inesperado: "
                "actual=(%.1f,%.1f) nominal=(%.1f,%.1f) err=(%.1f,%.1f)µm tol_fov=%.1f",
                self._point_index + 1,
                actual[0],
                actual[1],
                self._nominal_xy[0],
                self._nominal_xy[1],
                err_x,
                err_y,
                tol_fov,
            )
        elif self._hinf_native:
            logger.info(
                "[StepController] Punto %d FOV_VERIFY OK: err=(%.1f, %.1f)µm "
                "t_verify=%.0fms ticks=%d",
                self._point_index + 1,
                err_x,
                err_y,
                t_fov_verify_ms,
                self._fov_verify_ticks,
            )
        elif abs(err_x) > tol_fov or abs(err_y) > tol_fov:
            logger.warning(
                "[StepController] Punto %d residual vs FOV nominal: "
                "actual=(%.1f,%.1f) nominal=(%.1f,%.1f) err=(%.1f,%.1f)µm tol_fov=%.1f",
                self._point_index + 1,
                actual[0],
                actual[1],
                self._nominal_xy[0],
                self._nominal_xy[1],
                err_x,
                err_y,
                tol_fov,
            )

        t_move = (time.perf_counter() - self._transition_started_mono) * 1000.0
        result = PointTransitionResult(
            point_index=self._point_index,
            x_nominal_um=self._nominal_xy[0],
            y_nominal_um=self._nominal_xy[1],
            n_steps=len(self._point_step_results),
            t_move_ms=t_move,
            steps=list(self._point_step_results),
            status="ok",
            x_actual_um=actual[0],
            y_actual_um=actual[1],
            residual_x_um=err_x,
            residual_y_um=err_y,
            fov_verify_passed=fov_passed if self._hinf_native else True,
            t_fov_verify_ms=t_fov_verify_ms,
            fov_verify_ticks=self._fov_verify_ticks,
        )
        self.last_point_result = result
        self.metrics.record_point(result)
        self.phase = StepControllerPhase.POINT_COMPLETE
        self._fov_verify_started_mono = 0.0
        self._sync_actuator_config()
        return result

    def tick(self) -> StepTickOutput:
        self._sync_actuator_config()
        out = StepTickOutput(phase=self.phase)
        step = self._current_step()

        if self.phase == StepControllerPhase.IDLE:
            return out

        if self.phase == StepControllerPhase.POINT_COMPLETE:
            out.point_complete = True
            out.feedback_target_x, out.feedback_target_y = self._nominal_xy
            return out

        if self.phase == StepControllerPhase.FAILED:
            out.point_failed = True
            return out

        if self.phase == StepControllerPhase.FOV_VERIFY:
            return self._tick_fov_verify()

        if self.phase == StepControllerPhase.DWELL:
            if time.perf_counter() >= self._dwell_until_mono:
                if self._queue_index >= len(self._queue):
                    if self._hinf_native:
                        self._begin_fov_verify()
                        return self._tick_fov_verify()
                    self._finish_point()
                    out.phase = StepControllerPhase.POINT_COMPLETE
                    out.point_complete = True
                    out.feedback_target_x, out.feedback_target_y = self._nominal_xy
                else:
                    self._begin_current_step()
                    out.phase = StepControllerPhase.MOVING
            return out

        if step is None:
            if self._hinf_native:
                self._begin_fov_verify()
                return self._tick_fov_verify()
            self._finish_point()
            out.point_complete = True
            out.phase = StepControllerPhase.POINT_COMPLETE
            return out

        error_x, error_y, _, _ = self._read_error_um(step)
        err = error_x if step.axis == "x" else error_y
        out.error_x_um, out.error_y_um = error_x, error_y

        if self._hinf_native and not self._consume_fresh_telemetry():
            out.settling = self._settling_counter
            return out

        now = time.time()
        Ts = max(1e-4, now - self._last_time)
        self._last_time = now

        out.feedback_target_x = step.target_x_um
        out.feedback_target_y = step.target_y_um
        out.lock_x = step.axis == "y"
        out.lock_y = step.axis == "x"

        elapsed_ms = (time.perf_counter() - self._step_started_mono) * 1000.0
        step_timeout = self._step_timeout_ms(step)
        if elapsed_ms > step_timeout and not self._hinf_native:
            exit_tol = self._effective_tol_um() * self.config.tol_hysteresis_factor
            if abs(err) <= exit_tol and self._sensor_has_reading(step):
                logger.info(
                    "[StepController] Paso %d aceptado por proximidad (timeout %.0fms, err=%.1fµm)",
                    step.step_index + 1,
                    elapsed_ms,
                    err,
                )
                self._complete_current_step("timeout_ok")
                if self.phase == StepControllerPhase.DWELL:
                    out.phase = StepControllerPhase.DWELL
                elif self.phase == StepControllerPhase.POINT_COMPLETE:
                    out.phase = StepControllerPhase.POINT_COMPLETE
                    out.point_complete = True
                elif self.phase == StepControllerPhase.FOV_VERIFY:
                    out.phase = StepControllerPhase.FOV_VERIFY
                else:
                    out.phase = StepControllerPhase.MOVING
                return out
            self._step_retries += 1
            if self._step_retries <= self.config.max_step_retries:
                logger.warning(
                    "[StepController] Timeout paso %d (%.0fms, límite %.0fms, |Δ|=%.0fµm) — reintento %d/%d",
                    step.step_index + 1,
                    elapsed_ms,
                    step_timeout,
                    abs(step.delta_um),
                    self._step_retries,
                    self.config.max_step_retries,
                )
                self._begin_current_step(is_retry=True)
                return out
            self._fail_point("timeout tras reintentos")
            out.point_failed = True
            out.phase = StepControllerPhase.FAILED
            return out
        elif elapsed_ms > step_timeout and self._hinf_native:
            self._stall_log_counter += 1
            if self._stall_log_counter % 500 == 0:
                crit = self._pwm_crit.effective_min(step.axis)
                logger.info(
                    "[StepController] Paso %d continúa sin límite (%.0fms, err=%.1fµm, crit_pwm=%d)",
                    step.step_index + 1,
                    elapsed_ms,
                    err,
                    crit,
                )

        in_band = self._in_acceptance_band(err)

        if in_band:
            if not self._was_in_band and not self._hinf_native:
                self._on_enter_acceptance_band(step)
            if not self._hinf_native:
                self._hold_position(force=not self._was_in_band)
            elif self._hinf_native:
                pwm_a, pwm_b, error_x, error_y = self._run_hinf(step, Ts)
                out.pwm_a, out.pwm_b = pwm_a, pwm_b
                out.error_x_um, out.error_y_um = error_x, error_y
                self._send_command(f"A,{pwm_a},{pwm_b}")
                self._observe_pwm_crit(pwm_a, pwm_b)
            self._was_in_band = True
            if self._sensor_has_reading(step):
                self._settling_counter += 1
            out.settling = self._settling_counter
            if self._settling_counter >= SETTLING_CYCLES and self._ready_to_finalize_step(step):
                self._complete_current_step("ok")
                if self.phase == StepControllerPhase.DWELL:
                    out.phase = StepControllerPhase.DWELL
                elif self.phase == StepControllerPhase.POINT_COMPLETE:
                    out.phase = StepControllerPhase.POINT_COMPLETE
                    out.point_complete = True
                elif self.phase == StepControllerPhase.FOV_VERIFY:
                    out.phase = StepControllerPhase.FOV_VERIFY
                else:
                    out.phase = StepControllerPhase.MOVING
        else:
            self._was_in_band = False
            if self._hinf_native:
                pwm_a, pwm_b, error_x, error_y = self._run_hinf(step, Ts)
            else:
                pwm_a, pwm_b, error_x, error_y = self._run_pi(step, Ts)
            out.pwm_a, out.pwm_b = pwm_a, pwm_b
            out.error_x_um, out.error_y_um = error_x, error_y
            self._settling_counter = 0
            out.settling = 0
            self._send_command(f"A,{pwm_a},{pwm_b}")
            self._observe_pwm_crit(pwm_a, pwm_b)
            self._log_stall_if_needed(step, err)

        return out

    def read_current_xy_um(
        self,
        ctrl_a: Optional[ControllerConfig],
        ctrl_b: Optional[ControllerConfig],
    ) -> Tuple[float, float]:
        x_um = self._prev_xy[0]
        y_um = self._prev_xy[1]
        if ctrl_a:
            adc = self.sensor_buffer.get_adc(ctrl_a.sensor_key)
            if adc is not None:
                x_um = adc_to_um(adc, axis="x")
        if ctrl_b:
            adc = self.sensor_buffer.get_adc(ctrl_b.sensor_key)
            if adc is not None:
                y_um = adc_to_um(adc, axis="y")
        return x_um, y_um

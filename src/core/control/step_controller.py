"""Orquestador de pasos homogéneos mono-eje (máquina de estados).

Fine FOV:
  STM32: host approach → I+F (MCU C(z)) → SETTLED/HOST_STABLE ∧ residual≤tol
  Arduino: host approach → residual≤tol sostenido (sin C(z)/SETTLED; no ARM)

Coarse: H∞ MOVING → A,pwm. Parking vía _park_motors.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

from config.constants import (
    SETTLING_CYCLES,
    adc_to_um,
    lsb_um,
    mcu_cz_invert,
    mcu_supports_cz,
    position_error_um,
    um_to_adc,
)
from core.control.controller_config import ControllerConfig
from core.control.pwm_crit_estimator import PwmCritEstimator
from core.control.hinf_actuator import HinfActuator, HinfActuatorConfig, HinfAxisState
from core.control.sensor_buffer import SensorBuffer
from core.communication.protocol import MotorProtocol
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
    settle_ms: float = 0.0
    point_complete: bool = False
    point_failed: bool = False
    feedback_target_x: float = 0.0
    feedback_target_y: float = 0.0
    mcu_state: str = ""


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
        self._fov_verify_started_mono = 0.0
        self._fov_verify_ticks = 0
        self._fov_in_band_since_mono = 0.0
        self._fov_mcu_settled_since_mono = 0.0
        self._fov_retries = 0
        self._fov_active_axis: Optional[str] = None
        self._fov_advance_axes: List[str] = []
        self._move_dir_x = 0
        self._move_dir_y = 0
        self._backlash_dx_um = 0.0
        self._backlash_dy_um = 0.0
        self._fov_pulse = self._new_fov_pulse_state()
        self._fov_brake_pending = False
        self._fov_best_max_um: Optional[float] = None
        self._fov_frozen: bool = False  # latch: media≤tol sostenida → no más C(z)
        self._fov_freeze_since_mono: float = 0.0
        self._fov_filt_good_since_mono: float = 0.0
        self._fov_spoil_since_mono: float = 0.0
        self._fov_hyst_exit_since: float = 0.0
        self._fov_hold_cooldown_until: float = 0.0
        self._fov_pulse_count = 0
        self._fov_pulses_since_improve = 0
        self._fov_locked = {"x": False, "y": False}
        self._fov_hard_lock = {"x": False, "y": False}
        self._fov_axis_in_band_since = {"x": 0.0, "y": 0.0}
        self._fov_gate_out_since = {"x": 0.0, "y": 0.0}
        self._fov_gate_observe_since = 0.0
        self._fov_cz_last_arm_mono = 0.0
        self._fov_cz_armed = False
        self._fov_cz_ref_key: Optional[Tuple[int, ...]] = None
        self._fov_cz_rearms = 0
        self._fov_atom_um = {
            "x": float(config.fov_atom_um_per_idx0),
            "y": float(config.fov_atom_um_per_idx0),
        }
        self._pwm_crit = PwmCritEstimator(pwm_cap=config.step_hinf_pwm_min)
        self._last_control_telemetry_epoch = 0
        self._actuator = HinfActuator(
            HinfActuatorConfig(
                deadzone_um=config.deadzone_um(),
                pwm_min=max(0, int(config.step_hinf_pwm_min)),
                use_integral=config.step_use_integral,
            )
        )

    @property
    def _hinf_native(self) -> bool:
        return self.config.is_hinf_native

    def _sync_actuator_config(self) -> None:
        self._actuator.config = HinfActuatorConfig(
            deadzone_um=self.config.deadzone_um(),
            pwm_min=max(0, int(self.config.step_hinf_pwm_min)),
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
        self._fov_verify_started_mono = 0.0
        self._fov_verify_ticks = 0
        self._fov_in_band_since_mono = 0.0
        self._fov_mcu_settled_since_mono = 0.0
        self._fov_retries = 0
        self._fov_active_axis = None
        self._fov_advance_axes = []
        self._move_dir_x = 0
        self._move_dir_y = 0
        self._backlash_dx_um = 0.0
        self._backlash_dy_um = 0.0
        self._fov_pulse = self._new_fov_pulse_state()
        self._fov_brake_pending = False
        self._fov_best_max_um = None
        self._fov_frozen = False
        self._fov_freeze_since_mono = 0.0
        self._fov_filt_good_since_mono = 0.0
        self._fov_spoil_since_mono = 0.0
        self._fov_hyst_exit_since = 0.0
        self._fov_hold_cooldown_until = 0.0
        self._fov_pulse_count = 0
        self._fov_pulses_since_improve = 0
        self._fov_locked = {"x": False, "y": False}
        self._fov_hard_lock = {"x": False, "y": False}
        self._fov_axis_in_band_since = {"x": 0.0, "y": 0.0}
        self._fov_gate_out_since = {"x": 0.0, "y": 0.0}
        self._fov_gate_observe_since = 0.0
        self._fov_cz_last_arm_mono = 0.0
        self._fov_cz_armed = False
        self._fov_cz_ref_key = None
        self._fov_cz_rearms = 0
        self._fov_atom_um = {
            "x": float(self.config.fov_atom_um_per_idx0),
            "y": float(self.config.fov_atom_um_per_idx0),
        }
        self._pwm_crit.reset()
        self._last_control_telemetry_epoch = 0

    def _new_fov_pulse_state(self) -> dict:
        """Estado por eje: magnitud, signo aprendido, ganancia y reposo."""
        return {
            "x": self._fresh_fov_axis_pulse(),
            "y": self._fresh_fov_axis_pulse(),
        }

    def _fresh_fov_axis_pulse(self) -> dict:
        # Timing del pulso en TIEMPO DE PARED (Fase 2.1): pstate ∈ idle|on|rest.
        return {
            "pstate": "idle",
            "pulse_on_until": 0.0,
            "rest_until": 0.0,
            "pwm": self.config.fov_pwm_min,
            "atom_idx": int(self.config.fov_atom_idx_min),
            "pre_adc": None,
            "pre_err": 0.0,
            "pre_pwm": 0,
            "sign": 0,
            "gain": 0.0,
            "overshoot_at_min": 0,
            "hold_gate": False,
        }

    def _reset_fov_pulse_axis(self, axis: str) -> None:
        # Conserva ganancia/hold/atom_idx; reinicia el ciclo de pulso (pstate→idle).
        prev = self._fov_pulse.get(axis, {})
        st = self._fresh_fov_axis_pulse()
        st["gain"] = float(prev.get("gain", 0.0) or 0.0)
        st["hold_gate"] = bool(prev.get("hold_gate"))
        st["pwm"] = max(self.config.fov_pwm_min, int(prev.get("pwm", self.config.fov_pwm_min)))
        st["atom_idx"] = int(prev.get("atom_idx", self.config.fov_atom_idx_min))
        self._fov_pulse[axis] = st

    def _reset_control_telemetry_epoch(self) -> None:
        self._last_control_telemetry_epoch = 0

    @staticmethod
    def _sensor_lsb_um(axis: str) -> float:
        return lsb_um(axis)

    def _fov_tol_um_axis(self, axis: str) -> float:
        """Tolerancia de cierre por eje — respeta ``tol_fov_um`` de configuración."""
        return max(self.config.tol_fov_um, self._sensor_lsb_um(axis))

    def _consume_fresh_telemetry(self) -> bool:
        """
        True si hay telemetría usable para un tick de control.

        Requiere sensores frescos. No exige epoch nuevo cada vez: el lazo host
        @ ~100 Hz debe poder rearmar A,pwm aunque el ADC no haya cambiado
        (si no, la trayectoria queda muda — log 13:03 sin ningún A tras prepare).
        """
        if not self._hinf_native:
            return True
        ctrl_a = self._get_controller_a()
        ctrl_b = self._get_controller_b()
        if ctrl_a is None or ctrl_b is None:
            return False
        max_age = max(
            float(self.config.sensor_control_max_age_ms),
            float(self.config.approach_sensor_max_age_ms),
        )
        if not self.sensor_buffer.is_fresh(ctrl_a.sensor_key, max_age):
            return False
        if not self.sensor_buffer.is_fresh(ctrl_b.sensor_key, max_age):
            return False
        self._last_control_telemetry_epoch = self.sensor_buffer.update_count
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

    def _init_fov_advance_plan(self) -> None:
        # Ambos ejes siempre disponibles: el residual de acoplamiento no respeta Δ nominal.
        prefer = self._compute_advance_axis_order()
        both = ["x", "y"]
        ordered = [a for a in prefer if a in both]
        for a in both:
            if a not in ordered:
                ordered.append(a)
        self._fov_advance_axes = ordered
        self._fov_active_axis = None
        self._fov_pulse = self._new_fov_pulse_state()
        self._fov_brake_pending = False
        self._fov_best_max_um = None
        self._fov_frozen = False
        self._fov_freeze_since_mono = 0.0
        self._fov_filt_good_since_mono = 0.0
        self._fov_spoil_since_mono = 0.0
        self._fov_hyst_exit_since = 0.0
        self._fov_hold_cooldown_until = 0.0
        self._fov_pulse_count = 0
        self._fov_pulses_since_improve = 0
        self._fov_locked = {"x": False, "y": False}
        self._fov_hard_lock = {"x": False, "y": False}
        self._fov_axis_in_band_since = {"x": 0.0, "y": 0.0}
        self._fov_gate_out_since = {"x": 0.0, "y": 0.0}
        self._fov_gate_observe_since = 0.0
        self._fov_atom_um = {
            "x": float(self.config.fov_atom_um_per_idx0),
            "y": float(self.config.fov_atom_um_per_idx0),
        }
        if self._fov_advance_axes:
            self._set_fov_active_axis(self._fov_advance_axes[0])

    def _fov_ref_um(self) -> Tuple[float, float]:
        """Setpoint FOV con corrección de backlash según dirección nominal."""
        nx, ny = self._nominal_xy
        return nx + self._backlash_dx_um, ny + self._backlash_dy_um

    def _arm_mcu_cz(self, force: bool = False) -> None:
        """Activa C(z) una vez por punto (solo STM32; Arduino no tiene F/I/P)."""
        if not mcu_supports_cz():
            self._fov_cz_armed = False
            return
        ref_x, ref_y = self._fov_ref_um()
        ref_adc_x = int(round(um_to_adc(ref_x, axis="x")))
        ref_adc_y = int(round(um_to_adc(ref_y, axis="y")))
        tol = float(self.config.tol_fov_um)
        lsb = max(float(lsb_um("x")), float(lsb_um("y")), 1e-6)
        # LSB≈12µm: gate=1 ADC (~tol). NO forzar min=3 (~37µm) — freeze prematuro.
        gate_adc = max(1, min(40, int(math.ceil(tol / lsb))))
        key = (ref_adc_x, ref_adc_y, gate_adc)
        if (
            not force
            and self._fov_cz_armed
            and getattr(self, "_fov_cz_ref_key", None) == key
        ):
            return
        ctrl_a = self._get_controller_a()
        ctrl_b = self._get_controller_b()
        inv_x = mcu_cz_invert("x", bool(getattr(ctrl_a, "invert", False)))
        inv_y = mcu_cz_invert("y", bool(getattr(ctrl_b, "invert", False)))
        self._send_command(MotorProtocol.format_cz_invert(inv_x, inv_y))
        self._send_command(
            MotorProtocol.format_cz_fine(ref_adc_x, ref_adc_y, gate_adc=gate_adc)
        )
        self._fov_cz_last_arm_mono = time.perf_counter()
        self._fov_cz_armed = True
        self._fov_cz_ref_key = key
        self._fov_mcu_settled_since_mono = 0.0
        logger.info(
            "[StepController] Punto %d FOV_CZ_ARM ref_um=(%.1f,%.1f) ref_adc=(%d,%d) "
            "gate_adc=%d tol=%.1fµm inv=(%d,%d) host_inv=(%s,%s)",
            self._point_index + 1,
            ref_x,
            ref_y,
            ref_adc_x,
            ref_adc_y,
            gate_adc,
            tol,
            int(inv_x),
            int(inv_y),
            bool(getattr(ctrl_a, "invert", False)),
            bool(getattr(ctrl_b, "invert", False)),
        )

    def _read_fov_error_adc(self) -> Tuple[int, int, float, float, float, float]:
        """
        Errores ADC y µm sobre estimación filtrada (media ~40 ms).

        No usar 1 muestra de telemetría: ruido ~50–100 ADC pp (UI) hace
        best=1.8 µm fantasma y spoil de C(z).
        """
        ctrl_a = self._get_controller_a()
        ctrl_b = self._get_controller_b()
        ref_x, ref_y = self._fov_ref_um()
        ref_adc_x = int(round(um_to_adc(ref_x, axis="x")))
        ref_adc_y = int(round(um_to_adc(ref_y, axis="y")))
        win = float(getattr(self.config, "sensor_estimate_window_ms", 40.0))
        adc_x = (
            self.sensor_buffer.get_adc_mean(ctrl_a.sensor_key, win) if ctrl_a else None
        )
        adc_y = (
            self.sensor_buffer.get_adc_mean(ctrl_b.sensor_key, win) if ctrl_b else None
        )
        err_adc_x = 0 if adc_x is None else int(round(ref_adc_x - adc_x))
        err_adc_y = 0 if adc_y is None else int(round(ref_adc_y - adc_y))
        err_x = 0.0 if adc_x is None else position_error_um(ref_x, adc_x, "x")
        err_y = 0.0 if adc_y is None else position_error_um(ref_y, adc_y, "y")
        return err_adc_x, err_adc_y, err_x, err_y, err_x, err_y

    def _active_fov_correction_axis(
        self,
        err_adc_x: int,
        err_adc_y: int,
        err_traj_x: float,
        err_traj_y: float,
    ) -> Optional[str]:
        """Un eje activo; el otro queda bloqueado (PWM=0).

        Se mantiene el eje activo hasta encerrarlo en banda (settle) → lock.
        Si |err| ≤ gate (paso mínimo > residual útil) → hard-lock sin pulsar.
        """
        now = time.perf_counter()
        errs = {"x": err_traj_x, "y": err_traj_y}
        for axis in ("x", "y"):
            err = errs[axis]
            gate = self._fov_gate_um(axis)
            in_tol = abs(err) <= self._fov_tol_um_axis(axis)
            in_gate = abs(err) <= gate

            # Bajo el gate: soft-lock. Reapertura con histéresis temporal (acoplamiento).
            if in_gate:
                self._fov_gate_out_since[axis] = 0.0
                if not self._fov_locked[axis]:
                    self._fov_locked[axis] = True
                    self._fov_pulse[axis]["hold_gate"] = True
                    logger.info(
                        "[StepController] FOV_GATE_LOCK punto %d eje %s err=%.1fµm gate=%.1fµm",
                        self._point_index + 1,
                        axis.upper(),
                        err,
                        gate,
                    )
            elif self._fov_locked[axis] and not self._fov_hard_lock[axis]:
                if self._fov_gate_out_since[axis] <= 0.0:
                    self._fov_gate_out_since[axis] = now
                out_ms = (now - self._fov_gate_out_since[axis]) * 1000.0
                if out_ms >= self.config.fov_gate_unlock_hold_ms:
                    self._fov_locked[axis] = False
                    self._fov_pulse[axis]["hold_gate"] = False
                    self._fov_gate_out_since[axis] = 0.0
                    logger.info(
                        "[StepController] FOV_GATE_UNLOCK punto %d eje %s err=%.1fµm "
                        "gate=%.1fµm held_out=%.0fms",
                        self._point_index + 1,
                        axis.upper(),
                        err,
                        gate,
                        out_ms,
                    )

            if in_tol:
                if self._fov_axis_in_band_since[axis] <= 0.0:
                    self._fov_axis_in_band_since[axis] = now
                held_ms = (now - self._fov_axis_in_band_since[axis]) * 1000.0
                if held_ms >= self.config.fov_axis_settle_ms and not self._fov_locked[axis]:
                    self._fov_locked[axis] = True
                    logger.info(
                        "[StepController] FOV_LOCK punto %d eje %s err=%.1fµm held=%.0fms",
                        self._point_index + 1,
                        axis.upper(),
                        err,
                        held_ms,
                    )
            else:
                self._fov_axis_in_band_since[axis] = 0.0
                if self._fov_locked[axis] and not self._fov_hard_lock[axis] and not in_gate:
                    # Ya manejado por GATE_UNLOCK arriba si salió del gate.
                    pass

        cur = self._fov_active_axis
        if cur is not None and not self._fov_locked.get(cur, False):
            if abs(errs[cur]) > self._fov_gate_um(cur):
                return cur

        candidates = [
            a
            for a in self._fov_advance_axes
            if not self._fov_locked[a] and abs(errs[a]) > self._fov_gate_um(a)
        ]
        if not candidates:
            return None
        candidates.sort(key=lambda a: abs(errs[a]), reverse=True)
        return candidates[0]

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
        self._fov_retries = 0
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

    def prepare_fov_settle(
        self,
        prev_xy: Tuple[float, float],
        next_xy: Tuple[float, float],
        point_index: int,
        *,
        nominal_prev_xy: Optional[Tuple[float, float]] = None,
        backlash_dx_um: float = 0.0,
        backlash_dy_um: float = 0.0,
        move_dir_x: Optional[int] = None,
        move_dir_y: Optional[int] = None,
    ) -> None:
        """Alias del único cierre FOV (MCU C(z)). No hay settle_only paralelo."""
        logger.info(
            "[StepController] prepare_fov_settle → prepare_mcu_fine (método único)"
        )
        self.prepare_mcu_fine(
            prev_xy,
            next_xy,
            point_index,
            nominal_prev_xy=nominal_prev_xy,
            backlash_dx_um=backlash_dx_um,
            backlash_dy_um=backlash_dy_um,
            move_dir_x=move_dir_x,
            move_dir_y=move_dir_y,
        )

    def prepare_mcu_fine(
        self,
        prev_xy: Tuple[float, float],
        next_xy: Tuple[float, float],
        point_index: int,
        *,
        nominal_prev_xy: Optional[Tuple[float, float]] = None,
        backlash_dx_um: float = 0.0,
        backlash_dy_um: float = 0.0,
        move_dir_x: Optional[int] = None,
        move_dir_y: Optional[int] = None,
    ) -> None:
        """Tras approach host: MCU C(z) fine (presupuesto ≤fov_cz_max_fires)."""
        self.config.use_mcu_cz_loop = bool(mcu_supports_cz())
        self.config.use_mcu_atom_pulse = False
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
        self._fov_retries = 0
        self._fov_cz_rearms = 0
        self._queue.clear()
        self._queue_index = 0
        self._point_step_results.clear()
        self._transition_started_mono = time.perf_counter()
        self.last_point_result = None
        dx = next_xy[0] - prev_xy[0]
        dy = next_xy[1] - prev_xy[1]
        n_max = int(getattr(self.config, "fov_cz_max_fires", 15))
        logger.info(
            "[StepController] Punto %d: FOV MCU-fine Δ=(%.1f,%.1f)µm max_fires=%d",
            point_index + 1,
            dx,
            dy,
            n_max,
        )
        self._begin_fov_verify()

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

    def _park_motors(self, *, soft: bool = False) -> None:
        """Detiene actuación.

        soft=True (FOV settle/fail interno): solo A,0,0 — B provoca jerk (log 14:29).
        soft=False: legado con B si no hay C(z).
        """
        self._cz_soft_off()
        if not soft and not self.config.use_mcu_cz_loop:
            self._send_command("B")
        self._send_command("A,0,0")

    def _hold_position(self, force: bool = False) -> None:
        now = time.perf_counter()
        if not force and (now - self._last_hold_sent_mono) < (self.config.hold_resend_ms / 1000.0):
            return
        self._last_hold_sent_mono = now
        self._park_motors()

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

        if ctrl_a:
            adc_x = self.sensor_buffer.get_adc(ctrl_a.sensor_key)
            if adc_x is not None:
                error_x = position_error_um(step.target_x_um, adc_x, "x")
        if ctrl_b:
            adc_y = self.sensor_buffer.get_adc(ctrl_b.sensor_key)
            if adc_y is not None:
                error_y = position_error_um(step.target_y_um, adc_y, "y")

        return error_x, error_y, adc_x, adc_y

    def _run_pi(self, step: MeasuredStep, Ts: float) -> Tuple[int, int, float, float]:
        """PI legacy en µm (misma ley que HinfActuator)."""
        ctrl_a = self._get_controller_a()
        ctrl_b = self._get_controller_b()
        pwm_a = pwm_b = 0
        error_x, error_y, adc_x, adc_y = self._read_error_um(step)

        if step.axis == "x" and ctrl_a and adc_x is not None:
            deadzone_um = self._deadzone_for_error(error_x) * lsb_um("x")
            kp_scale, ki_scale = self._pi_gain_scale(error_x)
            if not self.config.step_use_integral:
                ki_scale = 0.0
            if self._last_err_adc_x * error_x < 0:
                self._integral_a = 0.0
            self._last_err_adc_x = error_x
            if abs(error_x) > deadzone_um:
                self._integral_a += error_x * Ts
                pwm_base = (
                    ctrl_a.Kp * kp_scale * error_x
                    + ctrl_a.Ki * ki_scale * self._integral_a
                )
                pwm_a = -int(pwm_base) if ctrl_a.invert else int(pwm_base)
                umax = int(self._pwm_limit("x", error_x, step))
                if abs(pwm_a) > umax:
                    self._integral_a -= error_x * Ts
                    pwm_a = max(-umax, min(umax, pwm_a))
                pwm_a = self._enforce_pwm_floor(pwm_a, self.config.step_pwm_min)
                self._step_pwm_max = max(self._step_pwm_max, abs(pwm_a))
        elif step.axis == "y" and ctrl_b and adc_y is not None:
            deadzone_um = self._deadzone_for_error(error_y) * lsb_um("y")
            kp_scale, ki_scale = self._pi_gain_scale(error_y)
            if not self.config.step_use_integral:
                ki_scale = 0.0
            if self._last_err_adc_y * error_y < 0:
                self._integral_b = 0.0
            self._last_err_adc_y = error_y
            if abs(error_y) > deadzone_um:
                self._integral_b += error_y * Ts
                pwm_base = (
                    ctrl_b.Kp * kp_scale * error_y
                    + ctrl_b.Ki * ki_scale * self._integral_b
                )
                pwm_b = -int(pwm_base) if ctrl_b.invert else int(pwm_base)
                umax = int(self._pwm_limit("y", error_y, step))
                if abs(pwm_b) > umax:
                    self._integral_b -= error_y * Ts
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
        self._reset_fov_pulse_axis(axis)
        self._fov_brake_pending = False
        self._fov_pulses_since_improve = 0
        self._fov_active_axis = axis

    def _dual_axis_mode(self) -> str:
        """hinf_native: un solo eje activo; orchestrated: según config."""
        if self._hinf_native:
            return "primary_only"
        return self.config.step_dual_axis_mode

    def _fov_nominal_pwm_sign(self, err_ctrl_um: float, ctrl: ControllerConfig) -> int:
        """Signo nominal H∞ (respeta ``invert``). Se sobreescribe si la planta aprende lo contrario."""
        if err_ctrl_um == 0:
            return 0
        d = 1 if err_ctrl_um > 0 else -1
        return -d if ctrl.invert else d

    def _fov_pulse_direction(self, axis: str, err_ctrl_um: float, ctrl: ControllerConfig) -> int:
        """Solo polaridad H∞ (invert). SIGNFLIP desactivado: el log mostró flips espurios por costa/backlash."""
        return self._fov_nominal_pwm_sign(err_ctrl_um, ctrl)

    def _fov_gate_um(self, axis: str) -> float:
        """Umbral bajo el cual un pulso mínimo empeora más de lo que corrige."""
        return max(self.config.fov_pulse_gate_um, self._fov_tol_um_axis(axis))

    def _measure_and_adapt_pulse(
        self, axis: str, adc_now: Optional[int], err_traj_now: float
    ) -> None:
        """Evalúa el pulso: adapta magnitud/idx; congela eje si ya está dentro del gate."""
        st = self._fov_pulse[axis]
        pre_adc = st["pre_adc"]
        if pre_adc is None or adc_now is None:
            return
        moved = int(adc_now) - int(pre_adc)
        moved_abs = abs(moved)
        pre_err = float(st["pre_err"])
        pre_pwm = int(st.get("pre_pwm", 0) or 0)
        pre_idx = int(st.get("atom_idx", self.config.fov_atom_idx_min))
        overshoot = (pre_err * err_traj_now < 0) and (
            abs(err_traj_now) > self._fov_tol_um_axis(axis)
        )
        worsened = abs(err_traj_now) > abs(pre_err) + 0.5 and (pre_err * err_traj_now > 0)
        improved = abs(err_traj_now) < abs(pre_err) - 0.5
        gate = self._fov_gate_um(axis)
        use_atom = bool(self.config.use_mcu_atom_pulse)
        imin = max(0, int(self.config.fov_atom_idx_min))
        imax = max(imin, int(self.config.fov_atom_idx_max))

        if moved_abs > 0 and abs(pre_pwm) > 0:
            g = moved_abs / float(abs(pre_pwm))
            st["gain"] = g if st["gain"] <= 0 else (0.7 * st["gain"] + 0.3 * g)

        # Aprende µm por átomo idx0 (escala ~1.4^idx) — puente H∞-µm ↔ LUT.
        if use_atom and moved_abs > 0:
            slope = self._sensor_lsb_um(axis)
            scale = 1.4 ** max(0, pre_idx)
            um_eq = (moved_abs * slope) / scale
            prev = float(self._fov_atom_um.get(axis, self.config.fov_atom_um_per_idx0))
            self._fov_atom_um[axis] = 0.7 * prev + 0.3 * max(2.0, min(80.0, um_eq))

        target = self.config.fov_target_step_adc
        max_step = self.config.fov_max_step_adc
        adapted = False

        if use_atom:
            # Adapta índice LUT (no la escalera pwm 22..45 que dejaba idx fijo=2).
            idx = pre_idx
            if moved_abs == 0 or (not improved and not overshoot and moved_abs < 2):
                idx = min(imax, idx + 1)
                adapted = True
                st["overshoot_at_min"] = 0
            elif overshoot or moved_abs > max_step:
                idx = max(imin, idx - 1)
                adapted = True
                if overshoot and idx <= imin:
                    st["overshoot_at_min"] = int(st.get("overshoot_at_min", 0) or 0) + 1
                else:
                    st["overshoot_at_min"] = 0
            else:
                st["overshoot_at_min"] = 0
            st["atom_idx"] = idx
        elif moved_abs == 0:
            st["pwm"] = min(self.config.fov_pwm_cap, st["pwm"] + self.config.fov_pwm_step)
            adapted = True
            st["overshoot_at_min"] = 0
        elif overshoot or moved_abs > max_step:
            scale = target / max(moved_abs, 1)
            new_pwm = int(round(st["pwm"] * scale))
            st["pwm"] = max(self.config.fov_pwm_min, min(self.config.fov_pwm_cap, new_pwm))
            adapted = True
            if overshoot and st["pwm"] <= self.config.fov_pwm_min:
                st["overshoot_at_min"] = int(st.get("overshoot_at_min", 0) or 0) + 1
            else:
                st["overshoot_at_min"] = 0
        elif st["gain"] > 0:
            desired = max(
                1,
                min(max_step, int(round(abs(err_traj_now) / self._sensor_lsb_um(axis)))),
            )
            new_pwm = int(round(desired / st["gain"]))
            st["pwm"] = max(self.config.fov_pwm_min, min(self.config.fov_pwm_cap, new_pwm))
            adapted = True
            st["overshoot_at_min"] = 0
        else:
            st["overshoot_at_min"] = 0

        # Dentro del gate → no tiene sentido otro pulso (paso mín. >> residual).
        if abs(err_traj_now) <= gate:
            st["hold_gate"] = True
            st["overshoot_at_min"] = 0

        # Spoil tras haber alcanzado tol_fov: congelar eje (log 15:04 best=1–3 µm luego spoil).
        tol = self._fov_tol_um_axis(axis)
        if (
            self._fov_best_max_um is not None
            and self._fov_best_max_um <= tol
            and abs(err_traj_now) > tol
            and (worsened or overshoot)
        ):
            st["hold_gate"] = True
        elif (
            self._fov_best_max_um is not None
            and self._fov_best_max_um <= gate
            and max(abs(err_traj_now), abs(pre_err)) > self._fov_best_max_um * 2.0
            and worsened
        ):
            st["hold_gate"] = True

        if improved or adapted:
            self._fov_pulses_since_improve = 0
        else:
            self._fov_pulses_since_improve += 1

        logger.info(
            "[StepController] FOV_PULSE punto %d eje %s pwm=%+d idx=%d moved_adc=%+d "
            "err=%.1f→%.1fµm overshoot=%s worsened=%s stall=%d pwm_next=%d idx_next=%d "
            "gain=%.3f ovr_min=%d hold=%d gate=%.1f",
            self._point_index + 1,
            axis.upper(),
            pre_pwm,
            pre_idx,
            moved,
            pre_err,
            err_traj_now,
            overshoot,
            worsened,
            self._fov_pulses_since_improve,
            st["pwm"],
            int(st.get("atom_idx", pre_idx)),
            float(st.get("gain", 0.0) or 0.0),
            int(st.get("overshoot_at_min", 0) or 0),
            int(bool(st.get("hold_gate"))),
            gate,
        )
        st["pre_adc"] = None
        st["pre_pwm"] = 0

    def _bump_fov_pulse_pwm(self, axis: str) -> None:
        st = self._fov_pulse[axis]
        st["pwm"] = min(self.config.fov_pwm_cap, st["pwm"] + self.config.fov_pwm_step)
        st["atom_idx"] = min(
            int(self.config.fov_atom_idx_max),
            int(st.get("atom_idx", self.config.fov_atom_idx_min)) + 1,
        )

    def _fov_atom_idx_for_error(self, axis: str, err_um: float) -> int:
        """Índice LUT desde Δµm deseado (paradigma H∞ en µm → átomo MCU).

        H∞ ya expresa el error en µm; aquí no se re-sintetiza con W1 (Ms/wb),
        se cuantiza la corrección pedida a 1..N átomos calibrados.
        """
        e = abs(float(err_um))
        imin = max(0, int(self.config.fov_atom_idx_min))
        imax = max(imin, int(self.config.fov_atom_idx_max))
        um0 = max(2.0, float(self._fov_atom_um.get(axis, self.config.fov_atom_um_per_idx0)))
        # nº de átomos idx0 equivalentes; cada +1 idx ≈ ×1.4 energía.
        n = e / um0
        if n <= 1.15:
            sug = imin
        elif n <= 2.2:
            sug = imin + 1
        else:
            sug = imax
        return max(imin, min(imax, sug))

    def _fov_atom_idx(self, st: dict, err_um: float = 0.0, axis: str = "x") -> int:
        """Combina Δµm→idx + bump en stall; respeta freeze-after-best."""
        imin = max(0, int(self.config.fov_atom_idx_min))
        imax = max(imin, int(self.config.fov_atom_idx_max))
        if (
            self.config.fov_freeze_after_best
            and self._fov_best_max_um is not None
            and self._fov_best_max_um <= self.config.tol_fov_um
        ):
            return imin
        sug = self._fov_atom_idx_for_error(axis, err_um)
        learned = max(imin, min(imax, int(st.get("atom_idx", sug))))
        if self._fov_pulses_since_improve >= 2:
            idx = max(sug, learned)
        elif int(st.get("overshoot_at_min", 0) or 0) > 0:
            idx = min(sug, learned)
        else:
            idx = sug
        return max(imin, min(imax, idx))

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

    def _begin_fov_verify(self) -> None:
        self.phase = StepControllerPhase.FOV_VERIFY
        self._fov_in_band_since_mono = 0.0
        self._fov_mcu_settled_since_mono = 0.0
        self._fov_verify_ticks = 0
        self._fov_verify_started_mono = time.perf_counter()
        self._last_time = time.time()
        self._reset_control_telemetry_epoch()
        self._init_fov_advance_plan()
        err_adc_x, err_adc_y, err_ctrl_x, err_ctrl_y, err_traj_x, err_traj_y = (
            self._read_fov_error_adc()
        )
        order = "→".join(a.upper() for a in self._fov_advance_axes)
        ref_x, ref_y = self._fov_ref_um()
        # Arduino: sin C(z). STM32: lazo MCU. Nunca forzar ARM sobre UNO.
        self.config.use_mcu_cz_loop = bool(mcu_supports_cz())
        self.config.use_mcu_atom_pulse = False
        logger.info(
            "[StepController] Punto %d FOV_VERIFY ref=(%.1f, %.1f) backlash=(%.1f, %.1f) "
            "residual_adc=(%+d, %+d) residual_traj=(%.1f, %.1f)µm tol_fov=%.1f avance=%s "
            "dir=(%+d,%+d) eje=%s cz=%s",
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
            bool(self.config.use_mcu_cz_loop),
        )
        self._fov_cz_armed = False
        self._fov_cz_ref_key = None
        self._fov_cz_rearms = 0
        self._fov_frozen = False
        self._fov_pos_ok_streak = 0
        self._fov_stale_since_mono = 0.0
        self._fov_active_ms = 0.0
        self._fov_last_active_mono = 0.0
        cur0 = max(abs(err_traj_x), abs(err_traj_y))
        if self.config.use_mcu_cz_loop:
            self._arm_mcu_cz(force=True)
            logger.info(
                "[StepController] Punto %d FOV_CZ_ARM residual=%.1fµm "
                "(MCU PI @ 50 kHz)",
                self._point_index + 1,
                cur0,
            )
        else:
            logger.info(
                "[StepController] Punto %d FOV_HOST_ONLY residual=%.1fµm "
                "tol=%.1fµm (Arduino: sin C(z)/SETTLED)",
                self._point_index + 1,
                cur0,
                float(self.config.tol_fov_um),
            )

    def _fov_settle_progress(self, settle_ms: float) -> int:
        """Progreso de asentamiento 0..SETTLING_CYCLES para feedback de UI."""
        if self.config.fov_settle_ms <= 0:
            return SETTLING_CYCLES
        frac = settle_ms / self.config.fov_settle_ms
        return int(max(0, min(SETTLING_CYCLES, round(frac * SETTLING_CYCLES))))

    def _update_fov_best_residual(self, err_traj_x: float, err_traj_y: float) -> None:
        """Actualiza best y FREEZE solo con media filtrada sostenida (no 1 spike)."""
        cur = max(abs(err_traj_x), abs(err_traj_y))
        if self._fov_best_max_um is None or cur < self._fov_best_max_um:
            self._fov_best_max_um = cur
        if not self.config.fov_freeze_after_best:
            return
        tol = float(self.config.tol_fov_um)
        hold_ms = float(getattr(self.config, "fov_freeze_hold_ms", 100.0))
        now = time.perf_counter()
        if cur <= tol:
            if self._fov_filt_good_since_mono <= 0.0:
                self._fov_filt_good_since_mono = now
            good_ms = (now - self._fov_filt_good_since_mono) * 1000.0
            if good_ms >= hold_ms and not self._fov_frozen:
                self._fov_freeze_since_mono = now
                self._fov_gate_observe_since = 0.0
                self._fov_frozen = True
                logger.info(
                    "[StepController] Punto %d FOV_FREEZE_LATCH best=%.1fµm "
                    "(media≤tol %.0fms) → observar sin cortar C(z)",
                    self._point_index + 1,
                    self._fov_best_max_um if self._fov_best_max_um is not None else cur,
                    good_ms,
                )
                # MCU posee el fine: no apagar C(z) al ver best (spoil histórico).
                # Solo el MCU HOLD/SETTLED o timeout host corta F.
        else:
            self._fov_filt_good_since_mono = 0.0

    def _cz_soft_off(self) -> None:
        """Apaga C(z) sin freno B (evita jerk/spoil)."""
        if self._fov_cz_armed:
            self._send_command(MotorProtocol.format_cz_off())
            self._fov_cz_armed = False

    def _tick_fov_verify_cz(
        self,
        out: StepTickOutput,
        err_adc_x: int,
        err_adc_y: int,
        err_traj_x: float,
        err_traj_y: float,
    ) -> StepTickOutput:
        """Cierre FOV canónico (dos criterios de aceptación, un solo productor).

        Cadena:
          host approach → armar F → MCU C(z) → aceptar si sensores frescos y:
            (A) SETTLED ∧ residual≤tol  (ruta rápida latch MCU), o
            (B) MCU activo (FINE/HOLD/SETTLED/PULSE) ∧ residual≤tol
                sostenido fov_settle_ms  (HOST_STABLE medido)

        No best-effort por timeout. Stale → pausa watchdog. REARM acotado.
        """
        now = time.perf_counter()
        cur = max(abs(err_traj_x), abs(err_traj_y))
        tol = float(self.config.tol_fov_um)
        out.pwm_a, out.pwm_b = 0, 0
        out.lock_x = False
        out.lock_y = False

        ctrl_a = self._get_controller_a()
        ctrl_b = self._get_controller_b()
        frame = self.sensor_buffer.last_frame()
        frame_age_ms = (
            max(0.0, (now - frame.t_monotonic) * 1000.0)
            if frame is not None
            else float("inf")
        )
        max_age = max(
            float(self.config.sensor_control_max_age_ms),
            float(self.config.fov_verify_sensor_max_age_ms),
        )
        sensors_ok = bool(
            ctrl_a
            and ctrl_b
            and self.sensor_buffer.is_fresh(ctrl_a.sensor_key, max_age)
            and self.sensor_buffer.is_fresh(ctrl_b.sensor_key, max_age)
        )
        self._update_fov_best_residual(err_traj_x, err_traj_y)

        mcu_state = (frame.state if frame else "").upper()
        out.mcu_state = mcu_state
        mcu_active = mcu_state in ("FINE", "HOLD", "SETTLED", "PULSE")

        # --- Pausa unificada si no hay telemetría usable ---
        if not sensors_ok:
            if float(getattr(self, "_fov_stale_since_mono", 0.0) or 0.0) <= 0.0:
                self._fov_stale_since_mono = now
                logger.warning(
                    "[StepController] Punto %d FOV_TLM_STALE age=%.0fms — "
                    "pausa watchdog (MCU sigue en C(z))",
                    self._point_index + 1,
                    frame_age_ms,
                )
            # No avanzar reloj activo ni REARM con datos congelados.
            self._fov_last_active_mono = 0.0
            self._fov_in_band_since_mono = 0.0
            out.settling = 0
            out.settle_ms = 0.0
            return out
        if float(getattr(self, "_fov_stale_since_mono", 0.0) or 0.0) > 0.0:
            logger.info(
                "[StepController] Punto %d FOV_TLM_OK tras stale %.0fms",
                self._point_index + 1,
                (now - float(self._fov_stale_since_mono)) * 1000.0,
            )
            self._fov_stale_since_mono = 0.0

        # Reloj activo solo con sensores frescos (método único de timeout).
        last_act = float(getattr(self, "_fov_last_active_mono", 0.0) or 0.0)
        if last_act > 0.0:
            self._fov_active_ms = float(getattr(self, "_fov_active_ms", 0.0) or 0.0) + (
                (now - last_act) * 1000.0
            )
        self._fov_last_active_mono = now

        def _accept_fov(reason: str) -> StepTickOutput:
            out.lock_x = True
            out.lock_y = True
            out.settling = 100
            settle_ms = float(self._fov_active_ms)
            if self._fov_in_band_since_mono > 0.0:
                settle_ms = max(
                    settle_ms, (now - float(self._fov_in_band_since_mono)) * 1000.0
                )
            out.settle_ms = settle_ms
            self._last_fov_accept_err = (err_traj_x, err_traj_y)
            self._cz_soft_off()
            self._finish_point()
            out.phase = StepControllerPhase.POINT_COMPLETE
            out.point_complete = True
            logger.info(
                "[StepController] Punto %d %s residual=(%.1f,%.1f)µm "
                "best=%.1fµm mcu=%s t_active=%.0fms",
                self._point_index + 1,
                reason,
                err_traj_x,
                err_traj_y,
                self._fov_best_max_um if self._fov_best_max_um is not None else -1.0,
                mcu_state or "?",
                float(self._fov_active_ms),
            )
            return out

        # --- (A) Ruta rápida: SETTLED + residual≤tol ---
        if mcu_state == "SETTLED" and cur <= tol:
            return _accept_fov("FOV_OK")

        # --- (B) HOST_STABLE: MCU activo + residual≤tol sostenido ---
        if mcu_active and cur <= tol:
            if self._fov_in_band_since_mono <= 0.0:
                self._fov_in_band_since_mono = now
            stable_ms = (now - float(self._fov_in_band_since_mono)) * 1000.0
            out.lock_x = True
            out.lock_y = True
            out.settling = self._fov_settle_progress(stable_ms)
            out.settle_ms = stable_ms
            if stable_ms >= float(self.config.fov_settle_ms):
                return _accept_fov("FOV_HOST_STABLE_OK")
        else:
            self._fov_in_band_since_mono = 0.0

        # REARM solo con C(z) real (STM32). Arduino nunca rearma F.
        if self.config.use_mcu_cz_loop and mcu_supports_cz():
            rearm_grace_s = float(self.config.fov_cz_rearm_grace_s)
            max_rearms = max(0, int(self.config.fov_cz_max_rearms))
            arm_age = now - float(getattr(self, "_fov_cz_last_arm_mono", 0.0) or 0.0)
            need_rearm = False
            rearm_why = ""
            if mcu_state == "SETTLED" and cur > tol:
                need_rearm = True
                rearm_why = "SETTLED_OUT"
            elif self._fov_cz_armed and not mcu_active and arm_age >= rearm_grace_s:
                need_rearm = True
                rearm_why = f"LOST_{mcu_state or '?'}"
            rearms = int(getattr(self, "_fov_cz_rearms", 0) or 0)
            if need_rearm and rearms < max_rearms:
                self._fov_cz_rearms = rearms + 1
                self._fov_cz_last_rearm_mono = now
                self._arm_mcu_cz(force=True)
                logger.warning(
                    "[StepController] Punto %d FOV_CZ_REARM why=%s residual=%.1fµm "
                    "rearm=%d/%d",
                    self._point_index + 1,
                    rearm_why,
                    cur,
                    self._fov_cz_rearms,
                    max_rearms,
                )

        prog_interval = float(self.config.fov_prog_log_interval_s)
        if not hasattr(self, "_fov_cz_last_prog_log") or (
            now - float(getattr(self, "_fov_cz_last_prog_log", 0.0))
        ) >= prog_interval:
            self._fov_cz_last_prog_log = now
            logger.info(
                "[StepController] Punto %d FOV_PROG residual=(%.1f,%.1f)µm "
                "best=%.1fµm mcu=%s pwm=(%d,%d) tlm_age=%.0fms active=%.0fms",
                self._point_index + 1,
                err_traj_x,
                err_traj_y,
                self._fov_best_max_um if self._fov_best_max_um is not None else -1.0,
                mcu_state or "?",
                frame.pot_a if frame else 0,
                frame.pot_b if frame else 0,
                frame_age_ms,
                float(getattr(self, "_fov_active_ms", 0.0) or 0.0),
            )

        out.settling = 50 if (mcu_active and cur <= tol) else 0
        out.settle_ms = float(getattr(self, "_fov_active_ms", 0.0) or 0.0)
        # Timeout único sobre tiempo con telemetría fresca.
        if float(self._fov_active_ms) >= float(self.config.fov_verify_timeout_ms):
            self._cz_soft_off()
            logger.warning(
                "[StepController] Punto %d FOV_TIMEOUT residual=(%.1f,%.1f)µm "
                "best=%.1fµm mcu=%s t_active=%.0fms",
                self._point_index + 1,
                err_traj_x,
                err_traj_y,
                self._fov_best_max_um if self._fov_best_max_um is not None else -1.0,
                mcu_state or "?",
                float(self._fov_active_ms),
            )
            self._park_motors(soft=True)
            self.phase = StepControllerPhase.FAILED
            out.phase = StepControllerPhase.FAILED
            out.point_failed = True
            return out
        return out

    def _tick_fov_verify_host_only(
        self,
        out: StepTickOutput,
        err_traj_x: float,
        err_traj_y: float,
    ) -> StepTickOutput:
        """Arduino / sin C(z): aceptar residual≤tol; no esperar SETTLED de ARM."""
        now = time.perf_counter()
        cur = max(abs(err_traj_x), abs(err_traj_y))
        tol = float(self.config.tol_fov_um)
        out.pwm_a, out.pwm_b = 0, 0
        out.lock_x = False
        out.lock_y = False
        out.mcu_state = "HOST"

        ctrl_a = self._get_controller_a()
        ctrl_b = self._get_controller_b()
        frame = self.sensor_buffer.last_frame()
        max_age = max(
            float(self.config.sensor_control_max_age_ms),
            float(self.config.fov_verify_sensor_max_age_ms),
        )
        sensors_ok = bool(
            ctrl_a
            and ctrl_b
            and self.sensor_buffer.is_fresh(ctrl_a.sensor_key, max_age)
            and self.sensor_buffer.is_fresh(ctrl_b.sensor_key, max_age)
        )
        self._update_fov_best_residual(err_traj_x, err_traj_y)
        if frame is not None:
            out.mcu_state = (frame.state or "LEGACY").upper()

        if not sensors_ok:
            self._fov_last_active_mono = 0.0
            self._fov_in_band_since_mono = 0.0
            out.settling = 0
            out.settle_ms = 0.0
            return out

        last_act = float(getattr(self, "_fov_last_active_mono", 0.0) or 0.0)
        if last_act > 0.0:
            self._fov_active_ms = float(getattr(self, "_fov_active_ms", 0.0) or 0.0) + (
                (now - last_act) * 1000.0
            )
        self._fov_last_active_mono = now

        def _accept(reason: str) -> StepTickOutput:
            out.lock_x = True
            out.lock_y = True
            out.settling = 100
            settle_ms = float(self._fov_active_ms)
            if self._fov_in_band_since_mono > 0.0:
                settle_ms = max(
                    settle_ms, (now - float(self._fov_in_band_since_mono)) * 1000.0
                )
            out.settle_ms = settle_ms
            self._last_fov_accept_err = (err_traj_x, err_traj_y)
            self._finish_point()
            out.phase = StepControllerPhase.POINT_COMPLETE
            out.point_complete = True
            logger.info(
                "[StepController] Punto %d %s residual=(%.1f,%.1f)µm "
                "best=%.1fµm t_active=%.0fms",
                self._point_index + 1,
                reason,
                err_traj_x,
                err_traj_y,
                self._fov_best_max_um if self._fov_best_max_um is not None else -1.0,
                float(self._fov_active_ms),
            )
            return out

        if cur <= tol:
            if self._fov_in_band_since_mono <= 0.0:
                self._fov_in_band_since_mono = now
            stable_ms = (now - float(self._fov_in_band_since_mono)) * 1000.0
            out.lock_x = True
            out.lock_y = True
            out.settling = self._fov_settle_progress(stable_ms)
            out.settle_ms = stable_ms
            if stable_ms >= float(self.config.fov_settle_ms):
                return _accept("FOV_HOST_ONLY_OK")
        else:
            self._fov_in_band_since_mono = 0.0
            out.settling = 0
            out.settle_ms = float(self._fov_active_ms)

        # Arduino no corrige en FOV: fallar pronto → re-approach host (no 25 s).
        host_timeout = min(float(self.config.fov_verify_timeout_ms), 2500.0)
        if float(self._fov_active_ms) >= host_timeout:
            if cur <= tol:
                return _accept("FOV_HOST_ONLY_TIMEOUT_IN_BAND")
            logger.warning(
                "[StepController] Punto %d FOV_HOST_ONLY_TIMEOUT "
                "residual=(%.1f,%.1f)µm tol=%.1fµm t_active=%.0fms",
                self._point_index + 1,
                err_traj_x,
                err_traj_y,
                tol,
                float(self._fov_active_ms),
            )
            self._park_motors(soft=True)
            self.phase = StepControllerPhase.FAILED
            out.phase = StepControllerPhase.FAILED
            out.point_failed = True
            return out
        return out

    def _tick_fov_verify(self) -> StepTickOutput:
        """FOV: STM32→C(z); Arduino→host residual≤tol (sin SETTLED)."""
        self.config.use_mcu_cz_loop = bool(mcu_supports_cz())
        self.config.use_mcu_atom_pulse = False

        out = StepTickOutput(phase=StepControllerPhase.FOV_VERIFY)
        out.feedback_target_x, out.feedback_target_y = self._fov_ref_um()
        self._fov_verify_ticks += 1

        err_adc_x, err_adc_y, err_ctrl_x, err_ctrl_y, err_traj_x, err_traj_y = (
            self._read_fov_error_adc()
        )
        out.error_x_um, out.error_y_um = err_traj_x, err_traj_y
        self._update_fov_best_residual(err_traj_x, err_traj_y)
        if not self.config.use_mcu_cz_loop:
            return self._tick_fov_verify_host_only(out, err_traj_x, err_traj_y)
        return self._tick_fov_verify_cz(
            out, err_adc_x, err_adc_y, err_traj_x, err_traj_y
        )

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

        if self._hinf_native:
            accepted_valid = int(
                abs(err_x) <= self.config.tol_fov_um
                and abs(err_y) <= self.config.tol_fov_um
            )
            logger.info(
                "[StepController] POINT_METRICS idx=%d residual=(%.1f,%.1f)µm tol=%.1f "
                "best=%.1fµm pulses=%d t_verify=%.0fms retries=%d settle_ms=%.0f "
                "locks=(X=%d,Y=%d) accepted_valid=%d",
                self._point_index + 1,
                err_x,
                err_y,
                self.config.tol_fov_um,
                self._fov_best_max_um if self._fov_best_max_um is not None else -1.0,
                self._fov_pulse_count,
                t_fov_verify_ms,
                self._fov_retries,
                self.config.fov_settle_ms,
                int(self._fov_locked["x"]),
                int(self._fov_locked["y"]),
                accepted_valid,
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
        """Posición XY desde media de telemetría (no 1 sample ruidoso)."""
        x_um = self._prev_xy[0]
        y_um = self._prev_xy[1]
        win = float(getattr(self.config, "sensor_estimate_window_ms", 40.0))
        if ctrl_a:
            adc = self.sensor_buffer.get_adc_mean(ctrl_a.sensor_key, win)
            if adc is not None:
                x_um = adc_to_um(adc, axis="x")
        if ctrl_b:
            adc = self.sensor_buffer.get_adc_mean(ctrl_b.sensor_key, win)
            if adc is not None:
                y_um = adc_to_um(adc, axis="y")
        return x_um, y_um

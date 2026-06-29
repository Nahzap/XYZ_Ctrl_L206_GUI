"""Actuador H∞ — ley u(k) coherente con síntesis W2 / U_max (Fase 8)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Optional, Tuple

from config.constants import CALIBRATION_X, CALIBRATION_Y, um_to_adc
from core.control.controller_config import ControllerConfig
from core.control.sensor_buffer import SensorBuffer

DualAxisMode = Literal["full_dual", "primary_only"]


@dataclass
class HinfAxisState:
    """Estado PI — error posicional en µm (Ki = ωb/K_planta)."""

    integral: float = 0.0
    last_err_um: float = 0.0


@dataclass
class HinfActuatorConfig:
    deadzone_um: float = 0.5
    pwm_min: int = 80
    use_integral: bool = True


class HinfActuator:
    """
    u(k) = sat( Kp·e_um + Ki·∫e_um , ±U_max ).

    e_um = error posicional en µm (misma convención que HInfTab live y Ki=ωb/K).
    U_max = W2 de la síntesis — única saturación de amplitud.
    """

    def __init__(self, config: HinfActuatorConfig):
        self.config = config

    @staticmethod
    def _enforce_pwm_floor(pwm: int, pwm_min: int) -> int:
        if pwm == 0:
            return 0
        if pwm_min > 0 and abs(pwm) < pwm_min:
            return pwm_min if pwm > 0 else -pwm_min
        return pwm

    def _compute_axis(
        self,
        ref_um: float,
        axis: str,
        ctrl: ControllerConfig,
        sensor_buffer: SensorBuffer,
        state: HinfAxisState,
        Ts: float,
    ) -> Tuple[int, float, HinfAxisState]:
        slope = CALIBRATION_X["slope"] if axis == "x" else CALIBRATION_Y["slope"]
        ref_adc = um_to_adc(ref_um, axis=axis)
        adc = sensor_buffer.get_adc(ctrl.sensor_key)
        if adc is None:
            return 0, 0.0, state

        error_um = (ref_adc - adc) * slope

        if state.last_err_um * error_um < 0:
            state = HinfAxisState(integral=0.0, last_err_um=error_um)
        else:
            state = HinfAxisState(integral=state.integral, last_err_um=error_um)

        if abs(error_um) <= self.config.deadzone_um:
            return 0, error_um, state

        integral = state.integral
        if self.config.use_integral:
            integral += error_um * Ts

        pwm_base = ctrl.Kp * error_um + (
            ctrl.Ki * integral if self.config.use_integral else 0.0
        )
        pwm = -int(pwm_base) if ctrl.invert else int(pwm_base)
        umax = int(ctrl.U_max)
        if abs(pwm) > umax:
            if self.config.use_integral:
                integral -= error_um * Ts
            pwm = max(-umax, min(umax, pwm))

        pwm = self._enforce_pwm_floor(pwm, self.config.pwm_min)
        if abs(pwm) > umax:
            pwm = max(-umax, min(umax, pwm))

        state = HinfAxisState(integral=integral, last_err_um=error_um)
        return pwm, error_um, state

    def compute(
        self,
        ref_x_um: float,
        ref_y_um: float,
        ctrl_a: Optional[ControllerConfig],
        ctrl_b: Optional[ControllerConfig],
        sensor_buffer: SensorBuffer,
        state_a: HinfAxisState,
        state_b: HinfAxisState,
        Ts: float,
        *,
        dual_axis_mode: DualAxisMode = "primary_only",
        primary_axis: Optional[str] = None,
    ) -> Tuple[int, int, float, float, HinfAxisState, HinfAxisState]:
        pwm_a = pwm_b = 0
        error_x = error_y = 0.0

        active_x = dual_axis_mode == "full_dual" or primary_axis == "x"
        active_y = dual_axis_mode == "full_dual" or primary_axis == "y"

        if active_x and ctrl_a:
            pwm_a, error_x, state_a = self._compute_axis(
                ref_x_um, "x", ctrl_a, sensor_buffer, state_a, Ts
            )
        elif ctrl_a:
            _, error_x, _ = self._compute_axis(
                ref_x_um, "x", ctrl_a, sensor_buffer, state_a, 0.0
            )

        if active_y and ctrl_b:
            pwm_b, error_y, state_b = self._compute_axis(
                ref_y_um, "y", ctrl_b, sensor_buffer, state_b, Ts
            )
        elif ctrl_b:
            _, error_y, _ = self._compute_axis(
                ref_y_um, "y", ctrl_b, sensor_buffer, state_b, 0.0
            )

        return pwm_a, pwm_b, error_x, error_y, state_a, state_b

    def reset_axis(self, state: HinfAxisState) -> HinfAxisState:
        return HinfAxisState()

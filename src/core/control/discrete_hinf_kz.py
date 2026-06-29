"""Evaluador K(z) discreto @ Ts=10 ms — PI equivalente a la síntesis HInfTab."""

from __future__ import annotations

from dataclasses import dataclass

from core.control.controller_config import ControllerConfig

# Periodo de muestreo del lazo de cierre (100 Hz en TestService).
CONTROL_TS_S = 0.010


@dataclass
class DiscreteKzState:
    """Estado del integrador discreto: x[k+1] = x[k] + Ts·e[k]."""

    integral_um: float = 0.0
    last_err_um: float = 0.0


def step_pi_kz(
    err_um: float,
    state: DiscreteKzState,
    ctrl: ControllerConfig,
    *,
    deadzone_um: float,
    pwm_min: int,
    use_integral: bool,
    Ts: float = CONTROL_TS_S,
) -> tuple[int, DiscreteKzState, float]:
    """
    K(z) = Kp + Ki·Ts/(z−1) en dominio µm.

    Misma ley que ``HinfActuator._compute_axis`` (invert, anti-windup, sat).
    """
    if state.last_err_um * err_um < 0:
        state = DiscreteKzState(integral_um=0.0, last_err_um=err_um)
    else:
        state = DiscreteKzState(integral_um=state.integral_um, last_err_um=err_um)

    if abs(err_um) <= deadzone_um:
        return 0, state, err_um

    integral = state.integral_um
    if use_integral:
        integral += err_um * Ts

    pwm_base = ctrl.Kp * err_um + (
        ctrl.Ki * integral if use_integral else 0.0
    )
    pwm = -int(pwm_base) if ctrl.invert else int(pwm_base)
    umax = int(ctrl.U_max)
    if abs(pwm) > umax:
        if use_integral:
            integral -= err_um * Ts
        pwm = max(-umax, min(umax, pwm))

    if pwm != 0 and pwm_min > 0 and abs(pwm) < pwm_min:
        pwm = pwm_min if pwm > 0 else -pwm_min
    if abs(pwm) > umax:
        pwm = max(-umax, min(umax, pwm))

    state = DiscreteKzState(integral_um=integral, last_err_um=err_um)
    return pwm, state, err_um

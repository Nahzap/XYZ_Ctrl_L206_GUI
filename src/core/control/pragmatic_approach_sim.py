"""Sim pragmática: approach [umin,umax] + stiction de banco."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

from config.constants import STITION_PWM_MAX, STITION_PWM_MIN
from core.control.host_approach import HostApproachController
from core.control.plant_simulator import PlantAxis, StabilityVerdict, evaluate_stability


@dataclass
class StictionPlant(PlantAxis):
    stiction_pwm: float = 80.0

    def step(self, u_pwm: float, dt: float) -> float:
        u_eff = float(u_pwm) if abs(float(u_pwm)) >= float(self.stiction_pwm) else 0.0
        return PlantAxis.step(self, u_eff, dt)


@dataclass
class PragmaticSimResult:
    t: List[float]
    x: List[float]
    e: List[float]
    u: List[float]
    phase: List[str]
    verdict: StabilityVerdict
    max_abs_u_below_stiction_while_far: float


def simulate_pragmatic_approach(
    *,
    x0_um: float = 0.0,
    target_um: float = 600.0,
    done_um: float = 20.0,
    engage_um: float = 90.0,
    K: float = 0.9429,
    tau: float = 0.059,
    invert: bool = False,
    slew_mag: int = STITION_PWM_MAX,
    stiction: float = 80.0,
    dt: float = 0.0025,
    t_max_s: float = 4.0,
) -> PragmaticSimResult:
    plant = StictionPlant(K=K, tau=tau, stiction_pwm=stiction)
    plant.reset(x0_um)
    ap = HostApproachController()
    ap.reset(done_um, engage_um, slew_pwm=slew_mag, kp=10.0, ki=8.0)

    t_list: List[float] = []
    x_list: List[float] = []
    e_list: List[float] = []
    u_list: List[float] = []
    ph_list: List[str] = []
    ghost_far = 0.0
    t = 0.0

    while t <= t_max_s + 1e-12:
        e = target_um - plant.x_um
        ae = abs(e)
        u, phase = ap.tick_axis("x", e, dt, invert=invert)
        if ae > engage_um and 0 < abs(u) < stiction:
            ghost_far = max(ghost_far, abs(u))
        plant.step(float(u), dt)
        t_list.append(t)
        x_list.append(plant.x_um)
        e_list.append(e)
        u_list.append(float(u))
        ph_list.append(phase)
        if phase == "HOLD" and ap._ax("x").hold_ms >= ap.config.settle_ms:
            break
        t += dt

    verdict = evaluate_stability(
        t_list,
        e_list,
        tol_um=done_um,
        axis="pragmatic",
        t_max_s=max(6.0, 0.85 * t_max_s),
        hold_s=0.05,
    )
    return PragmaticSimResult(
        t=t_list,
        x=x_list,
        e=e_list,
        u=u_list,
        phase=ph_list,
        verdict=verdict,
        max_abs_u_below_stiction_while_far=ghost_far,
    )


def prove_ghost_pwm_does_not_move(
    *,
    K: float = 0.94,
    tau: float = 0.05,
    pwm: float = 70.0,
    stiction: float = 80.0,
    t_s: float = 1.0,
    dt: float = 0.0025,
) -> Tuple[float, float]:
    p1 = StictionPlant(K=K, tau=tau, stiction_pwm=stiction)
    p1.reset(0.0)
    n = int(t_s / dt)
    for _ in range(n):
        p1.step(pwm, dt)
    p2 = StictionPlant(K=K, tau=tau, stiction_pwm=stiction)
    p2.reset(0.0)
    for _ in range(n):
        p2.step(float(STITION_PWM_MAX), dt)
    return p1.x_um, p2.x_um

"""Simulación de planta XY con las TF identificadas: G_vel = K/(τs+1), pos = ∫v.

Unidades (sesión H∞):
  K  → µm/s/PWM
  τ  → s
  x  → µm
  u  → PWM (mismo signo que envía el allocator tras invert)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from core.control.controller_config import ControllerConfig
from core.control.dual_power_allocator import (
    DualPowerAllocator,
    DualPowerConfig,
)


@dataclass
class PlantAxis:
    """Planta de velocidad de 1er orden + integrador de posición."""

    K: float  # µm/s/PWM
    tau: float  # s
    x_um: float = 0.0
    v_ums: float = 0.0

    def reset(self, x_um: float = 0.0) -> None:
        self.x_um = float(x_um)
        self.v_ums = 0.0

    def step(self, u_pwm: float, dt: float) -> float:
        dt = max(1e-6, float(dt))
        tau = max(1e-4, float(self.tau))
        a = math.exp(-dt / tau)
        v_inf = float(self.K) * float(u_pwm)
        self.v_ums = a * self.v_ums + (1.0 - a) * v_inf
        self.x_um += self.v_ums * dt
        return self.x_um


@dataclass
class StabilityVerdict:
    """Veredicto explícito de estabilización (para UI/tests/logs)."""

    stable: bool
    reason: str
    settle_time_s: float
    max_abs_error_um: float
    max_overshoot_um: float
    sign_flips: int
    final_error_um: float
    axis: str = "?"

    def as_label(self) -> str:
        return "STABLE" if self.stable else "NOT_STABLE"


@dataclass
class AxisSimResult:
    t: List[float] = field(default_factory=list)
    x: List[float] = field(default_factory=list)
    e: List[float] = field(default_factory=list)
    u: List[float] = field(default_factory=list)
    state: List[str] = field(default_factory=list)
    verdict: Optional[StabilityVerdict] = None


def load_session_plants(
    session_path: Optional[Path] = None,
) -> Dict[str, Dict[str, Any]]:
    """Carga planta+PI desde hinf_session.json (test.controllers + plant)."""
    import json

    if session_path is None:
        session_path = (
            Path(__file__).resolve().parents[1] / "config" / "hinf_session.json"
        )
    data = json.loads(Path(session_path).read_text(encoding="utf-8"))
    controllers = (data.get("test") or {}).get("controllers") or {}
    invert_map = (data.get("test") or {}).get("invert_map") or {}
    sensor_map = (data.get("test") or {}).get("sensor_map") or {}
    out: Dict[str, Dict[str, Any]] = {}
    for axis in ("A", "B"):
        c = controllers.get(axis) or {}
        if not c:
            continue
        K = float(c.get("K", 1.0))
        tau = float(c.get("tau", 0.05))
        if tau <= 0.0:
            tau = 1e-3
        # Signo de planta: K_sign o invert_map
        k_sign = float(c.get("K_sign", 1.0) or 1.0)
        invert = bool(invert_map.get(axis, False))
        if k_sign < 0:
            invert = not invert
        out[axis] = {
            "plant": PlantAxis(K=abs(K), tau=tau),
            "ctrl": ControllerConfig(
                Kp=float(c.get("Kp", 1.0)),
                Ki=float(c.get("Ki", 0.0)),
                U_max=float(c.get("U_max", 105.0)),
                invert=invert,
                sensor_key=str(sensor_map.get(axis, "sensor_1")),
                K_plant=abs(K),
            ),
            "K": abs(K),
            "tau": tau,
        }
    return out


def evaluate_stability(
    t: List[float],
    e: List[float],
    *,
    tol_um: float,
    axis: str = "?",
    t_max_s: float = 2.0,
    hold_s: float = 0.05,
    max_sign_flips: int = 8,
    overshoot_factor: float = 2.5,
) -> StabilityVerdict:
    """Criterio STABLE: entra en ±tol, permanece hold_s, pocas oscilaciones."""
    if not t or not e or len(t) != len(e):
        return StabilityVerdict(
            False, "serie vacía o inconsistente", 0.0, 0.0, 0.0, 0, 0.0, axis
        )
    tol = float(tol_um)
    max_abs = max(abs(x) for x in e)
    # Overshoot: pico tras primer cruce de banda hacia el lado opuesto al error inicial
    e0 = e[0]
    overshoot = 0.0
    entered = False
    enter_t = None
    for ti, ei in zip(t, e):
        if abs(ei) <= tol:
            if not entered:
                entered = True
                enter_t = ti
        if entered and e0 != 0.0 and (ei * e0) < 0.0:
            overshoot = max(overshoot, abs(ei))

    # Sign flips (después de 20% del tiempo)
    t_end = t[-1]
    flips = 0
    prev = None
    for ti, ei in zip(t, e):
        if ti < 0.2 * t_end:
            prev = ei
            continue
        if prev is not None and prev != 0.0 and ei != 0.0 and (prev * ei) < 0.0:
            flips += 1
        prev = ei

    if enter_t is None:
        return StabilityVerdict(
            False,
            f"no entra en ±{tol:.1f}µm en {t_end:.2f}s",
            t_end,
            max_abs,
            overshoot,
            flips,
            e[-1],
            axis,
        )
    if enter_t > t_max_s:
        return StabilityVerdict(
            False,
            f"settle lento ({enter_t:.2f}s > {t_max_s:.2f}s)",
            enter_t,
            max_abs,
            overshoot,
            flips,
            e[-1],
            axis,
        )
    # Permanencia final hold_s
    hold_ok = True
    for ti, ei in zip(t, e):
        if ti >= t_end - hold_s and abs(ei) > tol * 1.25:
            hold_ok = False
            break
    if not hold_ok:
        return StabilityVerdict(
            False,
            f"no sostiene ±{tol:.1f}µm los últimos {hold_s*1000:.0f}ms",
            enter_t,
            max_abs,
            overshoot,
            flips,
            e[-1],
            axis,
        )
    if flips > max_sign_flips:
        return StabilityVerdict(
            False,
            f"oscilación excesiva ({flips} flips > {max_sign_flips})",
            enter_t,
            max_abs,
            overshoot,
            flips,
            e[-1],
            axis,
        )
    if overshoot > overshoot_factor * max(tol, abs(e0) * 0.15):
        return StabilityVerdict(
            False,
            f"overshoot {overshoot:.1f}µm excesivo",
            enter_t,
            max_abs,
            overshoot,
            flips,
            e[-1],
            axis,
        )
    return StabilityVerdict(
        True,
        f"estable en {enter_t*1000:.0f}ms (flips={flips}, OS={overshoot:.1f}µm)",
        enter_t,
        max_abs,
        overshoot,
        flips,
        e[-1],
        axis,
    )


def simulate_axis_approach(
    plant: PlantAxis,
    ctrl: ControllerConfig,
    *,
    x0_um: float,
    target_um: float,
    done_um: float,
    dt: float = 0.0025,
    t_max_s: float = 3.0,
    power_cfg: Optional[DualPowerConfig] = None,
) -> AxisSimResult:
    """Lazo cerrado: DualPowerAllocator (perfil approach) + planta FO."""
    cfg = power_cfg or DualPowerConfig.for_approach(done_um)
    alloc = DualPowerAllocator(config=cfg)
    plant.reset(x0_um)
    res = AxisSimResult()
    t = 0.0
    while t <= t_max_s + 1e-12:
        e = target_um - plant.x_um
        u, st = alloc.tick_axis("a", e, dt, ctrl, now_mono=t)
        plant.step(float(u), dt)
        res.t.append(t)
        res.x.append(plant.x_um)
        res.e.append(e)
        res.u.append(float(u))
        res.state.append(st.value)
        if alloc.update_settle(t, ("a",)):
            # Unos ticks más para evaluar hold
            for _ in range(int(0.08 / dt)):
                t += dt
                e = target_um - plant.x_um
                u, st = alloc.tick_axis("a", e, dt, ctrl, now_mono=t)
                plant.step(float(u), dt)
                res.t.append(t)
                res.x.append(plant.x_um)
                res.e.append(e)
                res.u.append(float(u))
                res.state.append(st.value)
            break
        t += dt
    res.verdict = evaluate_stability(
        res.t,
        res.e,
        tol_um=done_um,
        axis="a",
        t_max_s=min(2.0, t_max_s),
        hold_s=max(0.04, cfg.settle_ms / 1000.0 * 0.5),
    )
    return res


def simulate_dual_approach_from_session(
    *,
    dx_um: float = 50.0,
    dy_um: float = 40.0,
    done_um: float = 20.0,
    session_path: Optional[Path] = None,
    dt: float = 0.0025,
    t_max_s: float = 3.0,
) -> Tuple[AxisSimResult, AxisSimResult, bool, str]:
    """Simula A y B con TF de sesión; retorna (ra, rb, both_stable, summary)."""
    plants = load_session_plants(session_path)
    if "A" not in plants or "B" not in plants:
        raise RuntimeError("hinf_session.json sin test.controllers A/B")
    cfg = DualPowerConfig.for_approach(done_um)
    ra = simulate_axis_approach(
        plants["A"]["plant"],
        plants["A"]["ctrl"],
        x0_um=0.0,
        target_um=float(dx_um),
        done_um=done_um,
        dt=dt,
        t_max_s=t_max_s,
        power_cfg=cfg,
    )
    rb = simulate_axis_approach(
        plants["B"]["plant"],
        plants["B"]["ctrl"],
        x0_um=0.0,
        target_um=float(dy_um),
        done_um=done_um,
        dt=dt,
        t_max_s=t_max_s,
        power_cfg=cfg,
    )
    ra.verdict.axis = "A"
    rb.verdict.axis = "B"
    both = bool(ra.verdict and ra.verdict.stable and rb.verdict and rb.verdict.stable)
    summary = (
        f"A={ra.verdict.as_label()} ({ra.verdict.reason}); "
        f"B={rb.verdict.as_label()} ({rb.verdict.reason})"
    )
    return ra, rb, both, summary

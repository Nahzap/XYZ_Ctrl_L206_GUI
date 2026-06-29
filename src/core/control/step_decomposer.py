"""Descomposición de transiciones FOV en cola de pasos mono-eje."""

from __future__ import annotations

import math
from typing import List, Tuple

from core.control.step_config import StepControlConfig
from core.control.step_types import MeasuredStep


def _signum(value: float) -> int:
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0


def _effective_step_um(abs_delta_um: float, config: StepControlConfig) -> float:
    """Amplía el paso en trayectos largos para evitar cientos de micro-pasos."""
    if abs_delta_um <= config.coarse_step_threshold_um:
        return config.step_um
    n_at_fine = max(1, int(round(abs_delta_um / config.step_um)))
    if n_at_fine <= config.max_steps_per_axis:
        return config.step_um
    return abs_delta_um / float(config.max_steps_per_axis)


def _split_axis_delta(delta_um: float, step_um: float) -> List[float]:
    if abs(delta_um) < 1e-6:
        return []
    n = max(1, int(round(abs(delta_um) / step_um)))
    steps: List[float] = []
    sign = _signum(delta_um)
    remaining = abs(delta_um)
    for i in range(n):
        if i == n - 1:
            chunk = remaining
        else:
            chunk = step_um
        steps.append(sign * chunk)
        remaining = max(0.0, remaining - step_um)
    return steps


def decompose_transition(
    prev_xy: Tuple[float, float],
    next_xy: Tuple[float, float],
    config: StepControlConfig,
    backlash_x_um: float = 0.0,
) -> List[MeasuredStep]:
    """
    Descompone (prev → next) en pasos elementales Y→X o X→Y.

    backlash_x_um se aplica al target X cuando move_dir > 0 (convención placement).
    """
    x0, y0 = prev_xy
    x1, y1 = next_xy
    dx = x1 - x0
    dy = y1 - y0

    y_step = _effective_step_um(abs(dy), config)
    x_step = _effective_step_um(abs(dx), config)
    y_chunks = _split_axis_delta(dy, y_step)
    x_chunks = _split_axis_delta(dx, x_step)

    ordered: List[Tuple[str, float]] = []
    if config.axis_order == "x_then_y":
        ordered.extend(("x", c) for c in x_chunks)
        ordered.extend(("y", c) for c in y_chunks)
    else:
        ordered.extend(("y", c) for c in y_chunks)
        ordered.extend(("x", c) for c in x_chunks)

    cx, cy = x0, y0
    steps: List[MeasuredStep] = []
    for idx, (axis, delta) in enumerate(ordered):
        if axis == "x":
            cx += delta
            move_dir = _signum(delta)
            tx = cx + (backlash_x_um if move_dir > 0 else 0.0)
            ty = cy
        else:
            cy += delta
            move_dir = _signum(delta)
            tx = cx
            ty = cy
        steps.append(
            MeasuredStep(
                axis=axis,
                delta_um=delta,
                target_x_um=tx,
                target_y_um=ty,
                move_dir=move_dir if axis == "x" else _signum(delta),
                step_index=idx,
            )
        )
    return steps


def estimate_step_count(prev_xy: Tuple[float, float], next_xy: Tuple[float, float], step_um: float) -> int:
    dx = abs(next_xy[0] - prev_xy[0])
    dy = abs(next_xy[1] - prev_xy[1])
    nx = 0 if dx < 1e-6 else max(1, int(round(dx / step_um)))
    ny = 0 if dy < 1e-6 else max(1, int(round(dy / step_um)))
    return nx + ny

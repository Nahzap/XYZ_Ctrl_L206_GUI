"""Tipos de datos para control de pasos homogéneos."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Literal, Optional


class StepControllerPhase(str, Enum):
    IDLE = "idle"
    DECOMPOSE = "decompose"
    MOVING = "moving"
    DWELL = "dwell"
    FOV_VERIFY = "fov_verify"
    POINT_COMPLETE = "point_complete"
    FAILED = "failed"


@dataclass
class MeasuredStep:
    axis: Literal["x", "y"]
    delta_um: float
    target_x_um: float
    target_y_um: float
    move_dir: int
    step_index: int
    transition_index: int = 0


@dataclass
class StepExecutionResult:
    step: MeasuredStep
    duration_ms: float
    error_um: float
    sensor_adc: Optional[int]
    status: str
    retries: int = 0
    pwm_max: int = 0


@dataclass
class PointTransitionResult:
    point_index: int
    x_nominal_um: float
    y_nominal_um: float
    n_steps: int = 0
    t_move_ms: float = 0.0
    steps: List[StepExecutionResult] = field(default_factory=list)
    status: str = "ok"
    failed_step_index: Optional[int] = None
    x_actual_um: float = 0.0
    y_actual_um: float = 0.0
    residual_x_um: float = 0.0
    residual_y_um: float = 0.0
    fov_verify_passed: bool = False
    t_fov_verify_ms: float = 0.0
    fov_verify_ticks: int = 0

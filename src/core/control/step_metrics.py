"""Agregación de métricas de sesión de pasos."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

from core.control.step_types import PointTransitionResult, StepExecutionResult


@dataclass
class StepSessionMetrics:
    points_completed: int = 0
    points_failed: int = 0
    total_steps: int = 0
    steps_ok: int = 0
    steps_failed: int = 0
    total_move_ms: float = 0.0
    step_durations_ms: List[float] = field(default_factory=list)
    step_errors_um: List[float] = field(default_factory=list)

    def record_point(self, result: PointTransitionResult) -> None:
        if result.status == "ok":
            self.points_completed += 1
        else:
            self.points_failed += 1
        self.total_move_ms += result.t_move_ms
        for step in result.steps:
            self.total_steps += 1
            self.step_durations_ms.append(step.duration_ms)
            self.step_errors_um.append(abs(step.error_um))
            if step.status == "ok":
                self.steps_ok += 1
            else:
                self.steps_failed += 1

    def to_dict(self) -> Dict[str, object]:
        durations = self.step_durations_ms
        errors = self.step_errors_um
        mean_d = sum(durations) / len(durations) if durations else 0.0
        std_d = (
            (sum((d - mean_d) ** 2 for d in durations) / len(durations)) ** 0.5
            if len(durations) > 1
            else 0.0
        )
        cv = (std_d / mean_d) if mean_d > 1e-6 else 0.0
        return {
            "points_completed": self.points_completed,
            "points_failed": self.points_failed,
            "total_steps": self.total_steps,
            "steps_ok": self.steps_ok,
            "steps_failed": self.steps_failed,
            "pct_step_ok": round(100.0 * self.steps_ok / max(1, self.total_steps), 2),
            "total_move_ms": round(self.total_move_ms, 1),
            "mean_step_duration_ms": round(mean_d, 1),
            "std_step_duration_ms": round(std_d, 1),
            "step_duration_cv": round(cv, 4),
            "mean_abs_error_um": round(sum(errors) / len(errors), 3) if errors else 0.0,
        }


def aggregate_point_metrics(steps: List[StepExecutionResult]) -> Dict[str, float]:
    if not steps:
        return {"n_steps": 0, "t_move_ms": 0.0, "mean_error_um": 0.0}
    return {
        "n_steps": len(steps),
        "t_move_ms": round(sum(s.duration_ms for s in steps), 1),
        "mean_error_um": round(sum(abs(s.error_um) for s in steps) / len(steps), 3),
    }

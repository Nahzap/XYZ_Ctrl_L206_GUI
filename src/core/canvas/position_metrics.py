"""Métricas de calidad de posicionamiento para inventario de canvas."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from core.canvas.capture_position import CapturePositionMetadata


@dataclass
class PositionQualityMetrics:
    """Resumen estadístico de errores de posición en un lote de capturas."""

    tile_count: int = 0
    with_actual_position: int = 0
    legacy_nominal_only: int = 0
    mean_error_x_um: float = 0.0
    mean_error_y_um: float = 0.0
    std_error_x_um: float = 0.0
    std_error_y_um: float = 0.0
    max_abs_error_x_um: float = 0.0
    max_abs_error_y_um: float = 0.0
    rmse_um: float = 0.0
    backlash_sample_counts: Dict[str, int] = field(default_factory=dict)
    estimated_backlash: Dict[str, float] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, object]:
        return {
            "tile_count": self.tile_count,
            "with_actual_position": self.with_actual_position,
            "legacy_nominal_only": self.legacy_nominal_only,
            "mean_error_x_um": round(self.mean_error_x_um, 3),
            "mean_error_y_um": round(self.mean_error_y_um, 3),
            "std_error_x_um": round(self.std_error_x_um, 3),
            "std_error_y_um": round(self.std_error_y_um, 3),
            "max_abs_error_x_um": round(self.max_abs_error_x_um, 3),
            "max_abs_error_y_um": round(self.max_abs_error_y_um, 3),
            "rmse_um": round(self.rmse_um, 3),
            "backlash_sample_counts": self.backlash_sample_counts,
            "estimated_backlash": {k: round(v, 3) for k, v in self.estimated_backlash.items()},
        }


def compute_position_metrics(
    positions: List[Optional[CapturePositionMetadata]],
    estimate_backlash: bool = True,
) -> PositionQualityMetrics:
    """Calcula métricas a partir de metadatos de posición por tile."""
    from core.canvas.backlash_model import BacklashEstimator

    metrics = PositionQualityMetrics(tile_count=len(positions))
    errors_x: List[float] = []
    errors_y: List[float] = []
    estimator = BacklashEstimator()

    for pos in positions:
        if pos is None:
            metrics.legacy_nominal_only += 1
            continue
        if pos.source == "filename":
            metrics.legacy_nominal_only += 1
            errors_x.append(pos.error_x_um)
            errors_y.append(pos.error_y_um)
            continue

        metrics.with_actual_position += 1
        errors_x.append(pos.error_x_um)
        errors_y.append(pos.error_y_um)
        if estimate_backlash:
            estimator.add_sample(pos)

    if errors_x:
        ex = np.array(errors_x, dtype=np.float64)
        ey = np.array(errors_y, dtype=np.float64)
        metrics.mean_error_x_um = float(np.mean(ex))
        metrics.mean_error_y_um = float(np.mean(ey))
        metrics.std_error_x_um = float(np.std(ex))
        metrics.std_error_y_um = float(np.std(ey))
        metrics.max_abs_error_x_um = float(np.max(np.abs(ex)))
        metrics.max_abs_error_y_um = float(np.max(np.abs(ey)))
        metrics.rmse_um = float(np.sqrt(np.mean(ex ** 2 + ey ** 2)))

    if estimate_backlash:
        metrics.backlash_sample_counts = estimator.sample_counts()
        bl = estimator.estimate()
        metrics.estimated_backlash = bl.to_dict()

    return metrics

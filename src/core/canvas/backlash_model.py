"""Modelo simple de corrección de holgura (backlash) por dirección de movimiento."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from core.canvas.capture_position import CapturePositionMetadata


@dataclass
class BacklashCorrection:
    """Correcciones empíricas en µm según dirección de avance."""

    correction_x_pos: float = 0.0
    correction_x_neg: float = 0.0
    correction_y_pos: float = 0.0
    correction_y_neg: float = 0.0

    def delta_for_direction(self, move_dir_x: int, move_dir_y: int) -> Tuple[float, float]:
        dx = 0.0
        dy = 0.0
        if move_dir_x > 0:
            dx = self.correction_x_pos
        elif move_dir_x < 0:
            dx = self.correction_x_neg
        if move_dir_y > 0:
            dy = self.correction_y_pos
        elif move_dir_y < 0:
            dy = self.correction_y_neg
        return dx, dy

    def to_dict(self) -> Dict[str, float]:
        return {
            "correction_x_pos": self.correction_x_pos,
            "correction_x_neg": self.correction_x_neg,
            "correction_y_pos": self.correction_y_pos,
            "correction_y_neg": self.correction_y_neg,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, float]) -> BacklashCorrection:
        return cls(
            correction_x_pos=float(data.get("correction_x_pos", 0.0)),
            correction_x_neg=float(data.get("correction_x_neg", 0.0)),
            correction_y_pos=float(data.get("correction_y_pos", 0.0)),
            correction_y_neg=float(data.get("correction_y_neg", 0.0)),
        )


@dataclass
class BacklashEstimator:
    """Estima corrección de backlash a partir de errores medidos por dirección."""

    min_samples: int = 3
    _errors_x_pos: List[float] = field(default_factory=list)
    _errors_x_neg: List[float] = field(default_factory=list)
    _errors_y_pos: List[float] = field(default_factory=list)
    _errors_y_neg: List[float] = field(default_factory=list)

    def add_sample(self, metadata: CapturePositionMetadata) -> None:
        if metadata.move_dir_x > 0:
            self._errors_x_pos.append(metadata.error_x_um)
        elif metadata.move_dir_x < 0:
            self._errors_x_neg.append(metadata.error_x_um)
        if metadata.move_dir_y > 0:
            self._errors_y_pos.append(metadata.error_y_um)
        elif metadata.move_dir_y < 0:
            self._errors_y_neg.append(metadata.error_y_um)

    @staticmethod
    def _median(values: List[float]) -> float:
        if not values:
            return 0.0
        ordered = sorted(values)
        mid = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[mid]
        return 0.5 * (ordered[mid - 1] + ordered[mid])

    def estimate(self) -> BacklashCorrection:
        return BacklashCorrection(
            correction_x_pos=-self._median(self._errors_x_pos) if len(self._errors_x_pos) >= self.min_samples else 0.0,
            correction_x_neg=-self._median(self._errors_x_neg) if len(self._errors_x_neg) >= self.min_samples else 0.0,
            correction_y_pos=-self._median(self._errors_y_pos) if len(self._errors_y_pos) >= self.min_samples else 0.0,
            correction_y_neg=-self._median(self._errors_y_neg) if len(self._errors_y_neg) >= self.min_samples else 0.0,
        )

    def sample_counts(self) -> Dict[str, int]:
        return {
            "x_pos": len(self._errors_x_pos),
            "x_neg": len(self._errors_x_neg),
            "y_pos": len(self._errors_y_pos),
            "y_neg": len(self._errors_y_neg),
        }

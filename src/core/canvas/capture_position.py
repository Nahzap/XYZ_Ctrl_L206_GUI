"""Metadatos de posición real en captura de microscopía."""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Any, Dict, Optional, Tuple


@dataclass
class CapturePositionMetadata:
    """Posición nominal vs real en el momento de captura."""

    x_nominal_um: float
    y_nominal_um: float
    x_actual_um: float
    y_actual_um: float
    error_x_um: float
    error_y_um: float
    move_dir_x: int = 0
    move_dir_y: int = 0
    status: str = ""
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat(timespec="seconds"))
    source: str = "sensor"

    @property
    def has_actual(self) -> bool:
        return self.x_actual_um != 0.0 or self.y_actual_um != 0.0 or abs(self.error_x_um) > 1e-6

    def placement_x_um(self, backlash_dx: float = 0.0) -> float:
        return self.x_actual_um + backlash_dx

    def placement_y_um(self, backlash_dy: float = 0.0) -> float:
        return self.y_actual_um + backlash_dy

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> CapturePositionMetadata:
        return cls(
            x_nominal_um=float(data.get("x_nominal_um", data.get("x_um", 0.0))),
            y_nominal_um=float(data.get("y_nominal_um", data.get("y_um", 0.0))),
            x_actual_um=float(data.get("x_actual_um", data.get("x_um", 0.0))),
            y_actual_um=float(data.get("y_actual_um", data.get("y_um", 0.0))),
            error_x_um=float(data.get("error_x_um", 0.0)),
            error_y_um=float(data.get("error_y_um", 0.0)),
            move_dir_x=int(data.get("move_dir_x", 0)),
            move_dir_y=int(data.get("move_dir_y", 0)),
            status=str(data.get("status", "")),
            timestamp=str(data.get("timestamp", "")),
            source=str(data.get("source", "unknown")),
        )

    @classmethod
    def from_nominal_only(cls, x_um: float, y_um: float) -> CapturePositionMetadata:
        return cls(
            x_nominal_um=x_um,
            y_nominal_um=y_um,
            x_actual_um=x_um,
            y_actual_um=y_um,
            error_x_um=0.0,
            error_y_um=0.0,
            source="filename",
        )

    @classmethod
    def from_acceptance(
        cls,
        x_nominal: float,
        y_nominal: float,
        x_actual: float,
        y_actual: float,
        move_dir_x: int = 0,
        move_dir_y: int = 0,
        status: str = "",
        source: str = "sensor",
    ) -> CapturePositionMetadata:
        return cls(
            x_nominal_um=x_nominal,
            y_nominal_um=y_nominal,
            x_actual_um=x_actual,
            y_actual_um=y_actual,
            error_x_um=x_nominal - x_actual,
            error_y_um=y_nominal - y_actual,
            move_dir_x=move_dir_x,
            move_dir_y=move_dir_y,
            status=status,
            source=source,
        )


def position_sidecar_path(folder: str, point_base: str) -> str:
    return os.path.join(folder, f"{point_base}_position.json")


def save_position_sidecar(folder: str, point_base: str, metadata: CapturePositionMetadata) -> str:
    os.makedirs(folder, exist_ok=True)
    path = position_sidecar_path(folder, point_base)
    payload = metadata.to_dict()
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
    return path


def load_position_metadata(
    folder: str,
    point_base: str,
    focus_json_path: Optional[str] = None,
) -> Optional[CapturePositionMetadata]:
    """Carga metadatos desde sidecar o campos extendidos de focus.json."""
    sidecar = position_sidecar_path(folder, point_base)
    if os.path.isfile(sidecar):
        try:
            with open(sidecar, encoding="utf-8") as handle:
                return CapturePositionMetadata.from_dict(json.load(handle))
        except (OSError, json.JSONDecodeError, TypeError, ValueError):
            pass

    focus_path = focus_json_path or os.path.join(folder, f"{point_base}_focus.json")
    if os.path.isfile(focus_path):
        try:
            with open(focus_path, encoding="utf-8") as handle:
                data = json.load(handle)
            if "x_actual_um" in data or "x_nominal_um" in data:
                return CapturePositionMetadata.from_dict(data)
        except (OSError, json.JSONDecodeError, TypeError, ValueError):
            pass
    return None


def merge_position_into_focus_dict(
    focus_meta: Dict[str, Any],
    position: CapturePositionMetadata,
) -> Dict[str, Any]:
    """Añade campos de posición a metadatos de enfoque existentes."""
    merged = dict(focus_meta)
    merged.update(position.to_dict())
    merged["x_um"] = round(position.x_nominal_um, 3)
    merged["y_um"] = round(position.y_nominal_um, 3)
    return merged

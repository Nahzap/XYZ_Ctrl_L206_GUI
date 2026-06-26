"""Inventario de capturas de microscopía para montaje de canvas."""

from __future__ import annotations

import glob
import json
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2

from core.canvas.backlash_model import BacklashCorrection
from core.canvas.capture_position import CapturePositionMetadata, load_position_metadata
from core.canvas.grid_config import GridConfig
from core.canvas.position_metrics import PositionQualityMetrics, compute_position_metrics
from utils.microscopy_filename import parse_microscopy_filename


@dataclass
class TileRecord:
    filepath: str
    x_um: float
    y_um: float
    col: int
    row: int
    f_index: int
    seq: int
    score: Optional[float] = None
    is_bpof: bool = False
    x_actual_um: Optional[float] = None
    y_actual_um: Optional[float] = None
    error_x_um: float = 0.0
    error_y_um: float = 0.0
    move_dir_x: int = 0
    move_dir_y: int = 0
    placement_x_um: Optional[float] = None
    placement_y_um: Optional[float] = None
    offset_px_x: float = 0.0
    offset_px_y: float = 0.0
    position_source: str = "filename"
    reg_offset_px_x: float = 0.0
    reg_offset_px_y: float = 0.0

    @property
    def placement_px_x(self) -> float:
        return self.offset_px_x + self.reg_offset_px_x

    @property
    def placement_px_y(self) -> float:
        return self.offset_px_y + self.reg_offset_px_y

    def effective_x_um(self) -> float:
        return self.placement_x_um if self.placement_x_um is not None else self.x_um

    def effective_y_um(self) -> float:
        return self.placement_y_um if self.placement_y_um is not None else self.y_um


@dataclass
class CanvasInventory:
    folder: str
    grid: GridConfig
    tiles: List[TileRecord] = field(default_factory=list)
    tile_width: int = 0
    tile_height: int = 0
    class_name: Optional[str] = None
    focal_layer: str = "f1"
    position_metrics: Optional[PositionQualityMetrics] = None
    backlash_correction: Optional[BacklashCorrection] = None

    @property
    def captured_cells(self) -> int:
        return len({(t.col, t.row) for t in self.tiles})

    @property
    def total_cells(self) -> int:
        return self.grid.total_cells

    @property
    def coverage_percent(self) -> float:
        if self.total_cells == 0:
            return 0.0
        return 100.0 * self.captured_cells / self.total_cells

    def coverage_map(self) -> List[List[bool]]:
        """Matriz [row][col] con True si hay captura."""
        covered = [[False] * self.grid.n_cols for _ in range(self.grid.n_rows)]
        for tile in self.tiles:
            covered[tile.row][tile.col] = True
        return covered


def _load_focus_score(folder: str, base_name: str) -> Tuple[Optional[float], bool, int]:
    """Lee score y mejor f_index desde *_focus.json si existe."""
    json_path = os.path.join(folder, f"{base_name}_focus.json")
    if not os.path.isfile(json_path):
        return None, False, 1
    try:
        with open(json_path, encoding="utf-8") as handle:
            data = json.load(handle)
        captures = data.get("captures") or []
        best_idx = 1
        best_score = -1.0
        for cap in captures:
            score = float(cap.get("S", 0))
            f_idx = int(cap.get("f_index", 0))
            if score > best_score:
                best_score = score
                best_idx = f_idx
        bpof_score = float(data.get("S_bpof", best_score))
        return bpof_score, True, best_idx
    except (OSError, json.JSONDecodeError, TypeError, ValueError):
        return None, False, 1


def _point_basename_from_parsed(parsed) -> str:
    """Base sin sufijo _fN para localizar focus.json."""
    from utils.microscopy_filename import build_point_basename

    return build_point_basename(
        parsed.class_name,
        parsed.point_index_0based,
        parsed.x_um,
        parsed.y_um,
    )


def _apply_position_to_tile(
    record: TileRecord,
    grid: GridConfig,
    tile_w: int,
    tile_h: int,
    backlash: Optional[BacklashCorrection] = None,
    position_meta: Optional[CapturePositionMetadata] = None,
) -> None:
    """Calcula posición de colocación y offsets en píxeles."""
    nominal_x = record.x_um
    nominal_y = record.y_um

    if position_meta is not None:
        record.x_actual_um = position_meta.x_actual_um
        record.y_actual_um = position_meta.y_actual_um
        record.error_x_um = position_meta.error_x_um
        record.error_y_um = position_meta.error_y_um
        record.move_dir_x = position_meta.move_dir_x
        record.move_dir_y = position_meta.move_dir_y
        record.position_source = position_meta.source

        bl_dx, bl_dy = (0.0, 0.0)
        if backlash is not None:
            bl_dx, bl_dy = backlash.delta_for_direction(
                position_meta.move_dir_x, position_meta.move_dir_y
            )
        record.placement_x_um = position_meta.placement_x_um(bl_dx)
        record.placement_y_um = position_meta.placement_y_um(bl_dy)
    else:
        record.x_actual_um = nominal_x
        record.y_actual_um = nominal_y
        record.placement_x_um = nominal_x
        record.placement_y_um = nominal_y
        record.position_source = "filename"

    px, py = grid.um_to_pixel(record.placement_x_um, record.placement_y_um, tile_w, tile_h)
    px_nom, py_nom = grid.um_to_pixel(nominal_x, nominal_y, tile_w, tile_h)
    record.offset_px_x = px - px_nom
    record.offset_px_y = py - py_nom

    col, row = grid.snap_cell_from_xy(record.placement_x_um, record.placement_y_um)
    record.col = col
    record.row = row


def scan_capture_folder(
    folder: str,
    grid: GridConfig,
    focal_layer: str = "f1",
    apply_backlash: bool = True,
) -> CanvasInventory:
    """
    Escanea carpeta de capturas y asigna cada tile a una celda de rejilla.

    focal_layer: 'f1' (BPoF), 'f0', 'f2', o 'best' (mayor S del JSON).
    """
    if not os.path.isdir(folder):
        raise FileNotFoundError(f"Carpeta no encontrada: {folder}")

    pattern = os.path.join(folder, "*.png")
    candidates: Dict[int, TileRecord] = {}
    class_names: Dict[str, int] = {}
    position_metas: List[Optional[CapturePositionMetadata]] = []
    tile_w = tile_h = 0

    raw_records: List[Tuple[TileRecord, Optional[CapturePositionMetadata]]] = []

    for filepath in glob.glob(pattern):
        parsed = parse_microscopy_filename(os.path.basename(filepath))
        if parsed is None or parsed.f_index is None:
            continue

        use_f_index = parsed.f_index
        score = None
        is_bpof = parsed.f_index == 1

        if focal_layer == "best":
            base = _point_basename_from_parsed(parsed)
            score, is_bpof, use_f_index = _load_focus_score(folder, base)
            if parsed.f_index != use_f_index:
                continue
        elif focal_layer.startswith("f"):
            want = int(focal_layer[1:])
            if parsed.f_index != want:
                continue
            base = _point_basename_from_parsed(parsed)
            score, is_bpof, _ = _load_focus_score(folder, base)
        else:
            base = _point_basename_from_parsed(parsed)
            score, is_bpof, _ = _load_focus_score(folder, base)

        if tile_w == 0:
            img = cv2.imread(filepath, cv2.IMREAD_UNCHANGED)
            if img is not None:
                tile_h, tile_w = img.shape[:2]

        col, row = grid.cell_from_xy(parsed.x_um, parsed.y_um)
        class_names[parsed.class_name] = class_names.get(parsed.class_name, 0) + 1

        record = TileRecord(
            filepath=filepath,
            x_um=parsed.x_um,
            y_um=parsed.y_um,
            col=col,
            row=row,
            f_index=parsed.f_index,
            seq=parsed.point_index_0based + 1,
            score=score,
            is_bpof=is_bpof,
        )

        position_meta = load_position_metadata(folder, base)
        position_metas.append(position_meta)

        key = parsed.point_index_0based
        raw_records.append((record, position_meta))
        existing = candidates.get(key)
        if existing is None:
            candidates[key] = record
        else:
            prev_s = existing.score if existing.score is not None else -1.0
            new_s = score if score is not None else -1.0
            if new_s > prev_s:
                candidates[key] = record

    dominant_class = max(class_names, key=class_names.get) if class_names else None

    tiles = sorted(candidates.values(), key=lambda t: (t.row, t.col))

    metrics = compute_position_metrics(position_metas, estimate_backlash=apply_backlash)
    backlash = None
    if apply_backlash and metrics.estimated_backlash:
        backlash = BacklashCorrection.from_dict(metrics.estimated_backlash)

    if tile_w > 0 and tile_h > 0:
        tile_positions: Dict[int, Optional[CapturePositionMetadata]] = {}
        for record, pos_meta in raw_records:
            parsed = parse_microscopy_filename(os.path.basename(record.filepath))
            if parsed is not None:
                tile_positions[parsed.point_index_0based] = pos_meta

        for tile in tiles:
            parsed = parse_microscopy_filename(os.path.basename(tile.filepath))
            pos_meta = tile_positions.get(parsed.point_index_0based) if parsed else None
            _apply_position_to_tile(tile, grid, tile_w, tile_h, backlash, pos_meta)

    return CanvasInventory(
        folder=folder,
        grid=grid,
        tiles=tiles,
        tile_width=tile_w,
        tile_height=tile_h,
        class_name=dominant_class,
        focal_layer=focal_layer,
        position_metrics=metrics,
        backlash_correction=backlash,
    )

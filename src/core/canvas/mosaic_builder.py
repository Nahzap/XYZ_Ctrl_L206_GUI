"""Montaje de mosaico por tiles con colocación física y registro visual."""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Callable, Dict, List, Optional, Tuple

import cv2
import numpy as np

from core.canvas.background_estimator import estimate_background_bgr
from core.canvas.canvas_inventory import CanvasInventory, TileRecord
from core.canvas.tile_registration import TilePlacement, refine_placements


@dataclass
class MosaicBuildOptions:
    """Opciones de montaje del canvas."""

    use_actual_position: bool = True
    apply_backlash: bool = True
    visual_registration: bool = True
    overlap_blend: bool = True
    canvas_padding_px: int = 32
    registration_max_shift_px: int = 48
    registration_min_response: float = 0.08
    preview_max_side: int = 4096


@dataclass
class MosaicBuildResult:
    canvas_path: str
    preview_path: str
    metadata_path: str
    width_px: int
    height_px: int
    background_bgr: Tuple[int, int, int]
    build_time_s: float
    tiles_placed: int
    coverage_percent: float
    registration_metrics: Dict[str, float] = field(default_factory=dict)
    canvas_offset_px: Tuple[int, int] = (0, 0)


def _image_row_for_grid_row(grid_row: int, n_rows: int, tile_h: int) -> int:
    """Fila en imagen nominal: y_min (grid row 0) queda abajo."""
    return (n_rows - 1 - grid_row) * tile_h


def _nominal_pixel_origin(tile: TileRecord, grid, n_rows: int, tw: int, th: int) -> Tuple[float, float]:
    px = tile.col * tw + tile.offset_px_x
    py = _image_row_for_grid_row(tile.row, n_rows, th) + tile.offset_px_y
    return px, py


def _compute_canvas_bounds(
    tiles: List[TileRecord],
    grid,
    tw: int,
    th: int,
    padding: int,
) -> Tuple[int, int, int, int, List[TilePlacement]]:
    """Calcula bounding box del canvas y colocaciones base."""
    n_rows = grid.n_rows
    placements: List[TilePlacement] = []

    for idx, tile in enumerate(tiles):
        px, py = _nominal_pixel_origin(tile, grid, n_rows, tw, th)
        placements.append(
            TilePlacement(
                tile_id=idx,
                px=px + tile.reg_offset_px_x,
                py=py + tile.reg_offset_px_y,
            )
        )

    if not placements:
        return 0, 0, tw + 2 * padding, th + 2 * padding, placements

    min_x = min(p.final_px for p in placements)
    min_y = min(p.final_py for p in placements)
    max_x = max(p.final_px + tw for p in placements)
    max_y = max(p.final_py + th for p in placements)

    offset_x = int(np.floor(min_x)) - padding
    offset_y = int(np.floor(min_y)) - padding
    width = int(np.ceil(max_x)) - offset_x + padding
    height = int(np.ceil(max_y)) - offset_y + padding

    for p in placements:
        p.px -= offset_x
        p.py -= offset_y

    return offset_x, offset_y, width, height, placements


def _fill_memmap_background(
    canvas: np.memmap,
    color_bgr: Tuple[int, int, int],
    band_rows: int = 4,
) -> None:
    h, w = canvas.shape[:2]
    band = np.full((band_rows, w, 3), color_bgr, dtype=np.uint8)
    for y in range(0, h, band_rows):
        y_end = min(y + band_rows, h)
        canvas[y:y_end] = band[: y_end - y]


def _make_preview(canvas: np.memmap, max_side: int = 4096) -> np.ndarray:
    h, w = canvas.shape[:2]
    step = max(1, int(np.ceil(max(h, w) / max_side)))
    preview = canvas[::step, ::step].copy()
    if preview.ndim == 2:
        preview = cv2.cvtColor(preview, cv2.COLOR_GRAY2BGR)
    return np.ascontiguousarray(preview)


def _paste_tile_simple(
    canvas: np.ndarray,
    img: np.ndarray,
    px: int,
    py: int,
) -> None:
    th, tw = img.shape[:2]
    h, w = canvas.shape[:2]
    x1 = max(0, px)
    y1 = max(0, py)
    x2 = min(w, px + tw)
    y2 = min(h, py + th)
    if x1 >= x2 or y1 >= y2:
        return
    sx1 = x1 - px
    sy1 = y1 - py
    canvas[y1:y2, x1:x2] = img[sy1 : sy1 + (y2 - y1), sx1 : sx1 + (x2 - x1)]


def _paste_tile_weighted(
    acc: np.ndarray,
    weight: np.ndarray,
    img: np.ndarray,
    px: int,
    py: int,
    tile_weight: float,
    feather_px: int = 12,
) -> None:
    th, tw = img.shape[:2]
    h, w = acc.shape[:2]
    x1 = max(0, px)
    y1 = max(0, py)
    x2 = min(w, px + tw)
    y2 = min(h, py + th)
    if x1 >= x2 or y1 >= y2:
        return

    sx1 = x1 - px
    sy1 = y1 - py
    patch = img[sy1 : sy1 + (y2 - y1), sx1 : sx1 + (x2 - x1)].astype(np.float32)

    w_mask = np.full((y2 - y1, x2 - x1), tile_weight, dtype=np.float32)
    if feather_px > 0:
        fw = min(feather_px, (x2 - x1) // 2, (y2 - y1) // 2)
        if fw >= 2:
            ramp = np.linspace(0.2, 1.0, fw, dtype=np.float32)
            w_mask[:fw, :] *= ramp[:, None]
            w_mask[-fw:, :] *= ramp[::-1][:, None]
            w_mask[:, :fw] *= ramp[None, :]
            w_mask[:, -fw:] *= ramp[::-1][None, :]

    acc[y1:y2, x1:x2] += patch * w_mask[:, :, None]
    weight[y1:y2, x1:x2] += w_mask


def _finalize_weighted_canvas(
    acc: np.ndarray,
    weight: np.ndarray,
    background_bgr: Tuple[int, int, int],
) -> np.ndarray:
    bg = np.array(background_bgr, dtype=np.float32)
    mask = weight > 1e-6
    out = np.broadcast_to(bg, acc.shape).copy()
    out[mask] = acc[mask] / weight[mask, None]
    return np.clip(out, 0, 255).astype(np.uint8)


def build_mosaic_to_memmap(
    inventory: CanvasInventory,
    output_dir: str,
    background_bgr: Optional[Tuple[int, int, int]] = None,
    progress_callback: Optional[Callable[[int, int, str], None]] = None,
    options: Optional[MosaicBuildOptions] = None,
) -> MosaicBuildResult:
    """
    Construye canvas completo con colocación por posición real y registro visual opcional.

    Solo un tile en RAM a la vez durante lectura; composición con blending acumulado.
    """
    opts = options or MosaicBuildOptions()
    if inventory.tile_width <= 0 or inventory.tile_height <= 0:
        raise ValueError("Inventario sin dimensiones de tile — escanee una carpeta con PNG válidos")

    os.makedirs(output_dir, exist_ok=True)
    grid = inventory.grid
    tw, th = inventory.tile_width, inventory.tile_height
    tiles = inventory.tiles

    if not opts.use_actual_position:
        for tile in tiles:
            tile.placement_x_um = tile.x_um
            tile.placement_y_um = tile.y_um
            tile.offset_px_x = 0.0
            tile.offset_px_y = 0.0
            col, row = grid.cell_from_xy(tile.x_um, tile.y_um)
            tile.col, tile.row = col, row

    if background_bgr is None:
        paths = [t.filepath for t in tiles]
        background_bgr, _ = estimate_background_bgr(paths)

    registration_metrics: Dict[str, float] = {}
    images_for_reg: List[np.ndarray] = []
    placements: List[TilePlacement] = []

    if opts.visual_registration and tiles:
        if progress_callback:
            progress_callback(0, len(tiles) + 2, "Cargando tiles para registro visual...")
        for tile in tiles:
            img = cv2.imread(tile.filepath, cv2.IMREAD_COLOR)
            if img is None:
                img = np.full((th, tw, 3), background_bgr, dtype=np.uint8)
            elif img.shape[0] != th or img.shape[1] != tw:
                img = cv2.resize(img, (tw, th), interpolation=cv2.INTER_LINEAR)
            images_for_reg.append(img)

        _, _, _, _, placements = _compute_canvas_bounds(
            tiles, grid, tw, th, opts.canvas_padding_px
        )
        placements, registration_metrics = refine_placements(
            images_for_reg,
            placements,
            tw,
            th,
            max_shift_px=opts.registration_max_shift_px,
            min_response=opts.registration_min_response,
        )
        for tile, placement in zip(tiles, placements):
            tile.reg_offset_px_x = placement.reg_dx
            tile.reg_offset_px_y = placement.reg_dy

    offset_x, offset_y, canvas_w, canvas_h, placements = _compute_canvas_bounds(
        tiles, grid, tw, th, opts.canvas_padding_px
    )

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    class_part = inventory.class_name or "canvas"
    canvas_path = os.path.join(output_dir, f"{class_part}_mosaic_{stamp}.dat")
    preview_path = os.path.join(output_dir, f"{class_part}_mosaic_{stamp}_preview.png")
    metadata_path = os.path.join(output_dir, f"{class_part}_mosaic_{stamp}_meta.json")

    t0 = time.perf_counter()
    canvas = np.memmap(
        canvas_path,
        dtype=np.uint8,
        mode="w+",
        shape=(canvas_h, canvas_w, 3),
    )

    total_steps = len(tiles) + 2
    step_i = 0

    if progress_callback:
        progress_callback(step_i, total_steps, "Rellenando fondo por bandas...")
    _fill_memmap_background(canvas, background_bgr)
    step_i += 1

    if opts.overlap_blend:
        acc = np.zeros((canvas_h, canvas_w, 3), dtype=np.float32)
        weight = np.zeros((canvas_h, canvas_w), dtype=np.float32)

    tiles_placed = 0
    for idx, tile in enumerate(tiles):
        if images_for_reg and idx < len(images_for_reg):
            img = images_for_reg[idx]
        else:
            img = cv2.imread(tile.filepath, cv2.IMREAD_COLOR)
            if img is None:
                if progress_callback:
                    progress_callback(
                        step_i,
                        total_steps,
                        f"⚠ No se pudo leer {os.path.basename(tile.filepath)}",
                    )
                continue
            if img.shape[0] != th or img.shape[1] != tw:
                img = cv2.resize(img, (tw, th), interpolation=cv2.INTER_LINEAR)

        if placements:
            px = int(round(placements[idx].final_px))
            py = int(round(placements[idx].final_py))
        else:
            px, py = _nominal_pixel_origin(tile, grid, grid.n_rows, tw, th)
            px, py = int(round(px)), int(round(py))

        tile_weight = float(tile.score) if tile.score is not None and tile.score > 0 else 1.0

        if opts.overlap_blend:
            _paste_tile_weighted(acc, weight, img, px, py, tile_weight)
        else:
            _paste_tile_simple(canvas, img, px, py)

        tiles_placed += 1
        step_i += 1

        if progress_callback:
            progress_callback(
                step_i,
                total_steps,
                f"Tile {tiles_placed}/{len(tiles)} → ({px},{py}) px "
                f"[err {tile.error_x_um:+.1f},{tile.error_y_um:+.1f} µm]",
            )

    if opts.overlap_blend:
        if progress_callback:
            progress_callback(total_steps - 1, total_steps, "Fusionando solapamientos...")
        composed = _finalize_weighted_canvas(acc, weight, background_bgr)
        canvas[:] = composed

    canvas.flush()

    if progress_callback:
        progress_callback(total_steps - 1, total_steps, "Generando preview...")
    preview = _make_preview(canvas, opts.preview_max_side)
    cv2.imwrite(preview_path, preview, [cv2.IMWRITE_PNG_COMPRESSION, 3])

    png_path = canvas_path.replace(".dat", ".png")
    exported_full = _export_memmap_png_banded(canvas, png_path, preview_path)

    build_time = time.perf_counter() - t0
    pos_metrics = inventory.position_metrics.to_dict() if inventory.position_metrics else {}

    meta = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "class_name": inventory.class_name,
        "folder": inventory.folder,
        "focal_layer": inventory.focal_layer,
        "build_options": {
            "use_actual_position": opts.use_actual_position,
            "apply_backlash": opts.apply_backlash,
            "visual_registration": opts.visual_registration,
            "overlap_blend": opts.overlap_blend,
            "canvas_padding_px": opts.canvas_padding_px,
        },
        "grid": {
            "x_min": grid.x_min,
            "x_max": grid.x_max,
            "y_min": grid.y_min,
            "y_max": grid.y_max,
            "fov_x": grid.fov_x,
            "fov_y": grid.fov_y,
            "n_cols": grid.n_cols,
            "n_rows": grid.n_rows,
        },
        "canvas_offset_px": [offset_x, offset_y],
        "tile_size_px": [tw, th],
        "canvas_size_px": [canvas_w, canvas_h],
        "background_bgr": list(background_bgr),
        "tiles_placed": tiles_placed,
        "captured_cells": inventory.captured_cells,
        "total_cells": inventory.total_cells,
        "coverage_percent": round(inventory.coverage_percent, 2),
        "build_time_s": round(build_time, 2),
        "position_metrics": pos_metrics,
        "registration_metrics": registration_metrics,
        "files": {
            "memmap": canvas_path,
            "preview_png": preview_path,
            "full_png": png_path if exported_full else None,
        },
        "tiles": [
            {
                "seq": t.seq,
                "col": t.col,
                "row": t.row,
                "x_um": t.x_um,
                "y_um": t.y_um,
                "x_actual_um": t.x_actual_um,
                "y_actual_um": t.y_actual_um,
                "placement_x_um": t.placement_x_um,
                "placement_y_um": t.placement_y_um,
                "error_x_um": round(t.error_x_um, 3),
                "error_y_um": round(t.error_y_um, 3),
                "offset_px": [round(t.offset_px_x, 2), round(t.offset_px_y, 2)],
                "reg_offset_px": [round(t.reg_offset_px_x, 2), round(t.reg_offset_px_y, 2)],
                "position_source": t.position_source,
                "file": os.path.basename(t.filepath),
                "score": t.score,
            }
            for t in tiles
        ],
    }
    with open(metadata_path, "w", encoding="utf-8") as handle:
        json.dump(meta, handle, indent=2, ensure_ascii=False)

    del canvas

    return MosaicBuildResult(
        canvas_path=png_path if exported_full else preview_path,
        preview_path=preview_path,
        metadata_path=metadata_path,
        width_px=canvas_w,
        height_px=canvas_h,
        background_bgr=background_bgr,
        build_time_s=build_time,
        tiles_placed=tiles_placed,
        coverage_percent=inventory.coverage_percent,
        registration_metrics=registration_metrics,
        canvas_offset_px=(offset_x, offset_y),
    )


def _export_memmap_png_banded(
    canvas: np.memmap,
    png_path: str,
    preview_path: str,
    max_pixels: int = 120_000_000,
) -> bool:
    """Exporta PNG completo solo si cabe en RAM; si no, deja memmap + preview."""
    h, w = canvas.shape[:2]
    if h * w > max_pixels:
        return False
    try:
        cv2.imwrite(png_path, np.asarray(canvas), [cv2.IMWRITE_PNG_COMPRESSION, 1])
        return os.path.isfile(png_path)
    except (MemoryError, cv2.error):
        return False

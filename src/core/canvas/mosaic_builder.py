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
    feather_px: int = 12
    preview_from_tiles_threshold_px: int = 25_000_000
    output_mode: str = "hdf5"  # "hdf5" (sin canvas denso) | "memmap"
    swap_stage_axes: bool = False  # cámara 90° respecto al stage: Y→horizontal canvas


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
    hdf5_path: Optional[str] = None


def _physical_pixel_origin(tile: TileRecord, grid, tw: int, th: int, swap_axes: bool = False) -> Tuple[float, float]:
    """Posición en px desde coordenadas físicas (µm), sin cuantizar a índice de celda."""
    return grid.um_to_pixel(
        tile.effective_x_um(), tile.effective_y_um(), tw, th, swap_axes=swap_axes
    )


def _compute_canvas_bounds(
    tiles: List[TileRecord],
    grid,
    tw: int,
    th: int,
    padding: int,
    swap_axes: bool = False,
) -> Tuple[int, int, int, int, List[TilePlacement]]:
    """Bounding box compacto del canvas y colocaciones base."""
    placements: List[TilePlacement] = []

    for idx, tile in enumerate(tiles):
        px, py = _physical_pixel_origin(tile, grid, tw, th, swap_axes=swap_axes)
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


def estimate_canvas_size_px(
    inventory: CanvasInventory,
    padding: int = 32,
    swap_axes: bool = False,
) -> Tuple[int, int]:
    """Estima tamaño del canvas (ancho, alto) sin montarlo."""
    tw, th = inventory.tile_width, inventory.tile_height
    if tw <= 0 or th <= 0 or not inventory.tiles:
        return tw + 2 * padding, th + 2 * padding
    _, _, w, h, _ = _compute_canvas_bounds(
        inventory.tiles, inventory.grid, tw, th, padding, swap_axes=swap_axes
    )
    return w, h


def cleanup_orphan_canvas_temp_files(output_dir: str) -> list[str]:
    """Elimina .dat / _acc.dat / _weight.dat de montajes memmap anteriores."""
    removed: list[str] = []
    if not os.path.isdir(output_dir):
        return removed
    for name in os.listdir(output_dir):
        if not name.endswith(".dat"):
            continue
        if "_mosaic_" not in name:
            continue
        path = os.path.join(output_dir, name)
        try:
            os.remove(path)
            removed.append(name)
        except OSError:
            pass
    return removed


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


def _close_memmap(arr: Optional[np.memmap]) -> None:
    if arr is None:
        return
    arr.flush()
    if hasattr(arr, "_mmap"):
        arr._mmap.close()


def _load_tile_image(
    tile: TileRecord,
    tw: int,
    th: int,
    background_bgr: Tuple[int, int, int],
) -> np.ndarray:
    img = cv2.imread(tile.filepath, cv2.IMREAD_COLOR)
    if img is None:
        return np.full((th, tw, 3), background_bgr, dtype=np.uint8)
    if img.shape[0] != th or img.shape[1] != tw:
        img = cv2.resize(img, (tw, th), interpolation=cv2.INTER_LINEAR)
    return img


def _feather_alpha_mask(h: int, w: int, feather_px: int) -> np.ndarray:
    alpha = np.ones((h, w), dtype=np.float32)
    if feather_px < 2:
        return alpha
    fw = min(feather_px, w // 2, h // 2)
    if fw < 2:
        return alpha
    ramp = np.linspace(0.15, 1.0, fw, dtype=np.float32)
    alpha[:fw, :] *= ramp[:, None]
    alpha[-fw:, :] *= ramp[::-1][:, None]
    alpha[:, :fw] *= ramp[None, :]
    alpha[:, -fw:] *= ramp[::-1][None, :]
    return alpha


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


def _paste_tile_feather_blend(
    canvas: np.ndarray,
    img: np.ndarray,
    px: int,
    py: int,
    background_bgr: Tuple[int, int, int],
    feather_px: int = 12,
    bg_threshold: float = 12.0,
) -> None:
    """
    Blend local en la región intersectada — solo esa parche en RAM.

    Sin acumuladores globales acc/weight; operaciones matriciales tile-sized.
    """
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
    rh, rw = y2 - y1, x2 - x1
    patch = img[sy1 : sy1 + rh, sx1 : sx1 + rw].astype(np.float32)
    existing = np.array(canvas[y1:y2, x1:x2], dtype=np.float32)

    alpha = _feather_alpha_mask(rh, rw, feather_px)
    bg = np.array(background_bgr, dtype=np.float32)
    bg_dist = np.linalg.norm(existing - bg, axis=2)
    is_bg = (bg_dist < bg_threshold) | (np.max(existing, axis=2) < 1.0)

    alpha3 = np.where(is_bg[:, :, None], 1.0, alpha[:, :, None])
    blended = existing * (1.0 - alpha3) + patch * alpha3
    canvas[y1:y2, x1:x2] = np.clip(blended, 0, 255).astype(np.uint8)


def _make_preview_from_tiles(
    tiles: List[TileRecord],
    placements: List[TilePlacement],
    tw: int,
    th: int,
    canvas_w: int,
    canvas_h: int,
    background_bgr: Tuple[int, int, int],
    max_side: int,
    overlap_blend: bool,
    feather_px: int,
) -> np.ndarray:
    """Preview reducido componiendo tiles — no lee el canvas memmap completo."""
    scale = max(canvas_w, canvas_h) / max(max_side, 1)
    pw = max(1, int(np.ceil(canvas_w / scale)))
    ph = max(1, int(np.ceil(canvas_h / scale)))
    preview = np.full((ph, pw, 3), background_bgr, dtype=np.uint8)
    tile_w_s = max(1, int(round(tw / scale)))
    tile_h_s = max(1, int(round(th / scale)))

    for idx, tile in enumerate(tiles):
        if idx >= len(placements):
            break
        img = _load_tile_image(tile, tw, th, background_bgr)
        img_small = cv2.resize(img, (tile_w_s, tile_h_s), interpolation=cv2.INTER_AREA)
        px = int(round(placements[idx].final_px / scale))
        py = int(round(placements[idx].final_py / scale))
        if overlap_blend:
            _paste_tile_feather_blend(
                preview, img_small, px, py, background_bgr, feather_px=max(2, feather_px // 2)
            )
        else:
            _paste_tile_simple(preview, img_small, px, py)

    return np.ascontiguousarray(preview)


def _make_preview_from_memmap(canvas: np.memmap, max_side: int = 4096) -> np.ndarray:
    h, w = canvas.shape[:2]
    step = max(1, int(np.ceil(max(h, w) / max_side)))
    preview = canvas[::step, ::step].copy()
    if preview.ndim == 2:
        preview = cv2.cvtColor(preview, cv2.COLOR_GRAY2BGR)
    return np.ascontiguousarray(preview)


@dataclass
class PreparedMosaic:
    """Resultado de registro + colocación, compartido por HDF5 y memmap."""

    background_bgr: Tuple[int, int, int]
    placements: List[TilePlacement]
    offset_x: int
    offset_y: int
    canvas_w: int
    canvas_h: int
    registration_metrics: Dict[str, float]


def prepare_mosaic_build(
    inventory: CanvasInventory,
    opts: MosaicBuildOptions,
    background_bgr: Optional[Tuple[int, int, int]] = None,
    progress_callback: Optional[Callable[[int, int, str], None]] = None,
) -> PreparedMosaic:
    """Registro visual y bounding box — sin materializar canvas."""
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
    placements: List[TilePlacement] = []

    def _tile_loader(idx: int) -> np.ndarray:
        return _load_tile_image(tiles[idx], tw, th, background_bgr)

    if opts.visual_registration and tiles:
        if progress_callback:
            progress_callback(0, len(tiles) + 2, "Registro visual (tiles bajo demanda)...")
        _, _, _, _, placements = _compute_canvas_bounds(
            tiles, grid, tw, th, opts.canvas_padding_px, swap_axes=opts.swap_stage_axes
        )
        placements, registration_metrics = refine_placements(
            _tile_loader,
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
        tiles, grid, tw, th, opts.canvas_padding_px, swap_axes=opts.swap_stage_axes
    )
    return PreparedMosaic(
        background_bgr=background_bgr,
        placements=placements,
        offset_x=offset_x,
        offset_y=offset_y,
        canvas_w=canvas_w,
        canvas_h=canvas_h,
        registration_metrics=registration_metrics,
    )


def _build_metadata_dict(
    inventory: CanvasInventory,
    opts: MosaicBuildOptions,
    prepared: PreparedMosaic,
    tiles_placed: int,
    build_time: float,
    stamp: str,
    output_files: Dict[str, Optional[str]],
) -> dict:
    grid = inventory.grid
    tw, th = inventory.tile_width, inventory.tile_height
    tiles = inventory.tiles
    pos_metrics = inventory.position_metrics.to_dict() if inventory.position_metrics else {}
    return {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "class_name": inventory.class_name,
        "folder": inventory.folder,
        "focal_layer": inventory.focal_layer,
        "build_options": {
            "use_actual_position": opts.use_actual_position,
            "apply_backlash": opts.apply_backlash,
            "visual_registration": opts.visual_registration,
            "overlap_blend": opts.overlap_blend,
            "overlap_blend_mode": "local_feather",
            "output_mode": opts.output_mode,
            "swap_stage_axes": opts.swap_stage_axes,
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
        "canvas_offset_px": [prepared.offset_x, prepared.offset_y],
        "tile_size_px": [tw, th],
        "canvas_size_px": [prepared.canvas_w, prepared.canvas_h],
        "background_bgr": list(prepared.background_bgr),
        "tiles_placed": tiles_placed,
        "captured_cells": inventory.captured_cells,
        "total_cells": inventory.total_cells,
        "coverage_percent": round(inventory.coverage_percent, 2),
        "build_time_s": round(build_time, 2),
        "position_metrics": pos_metrics,
        "registration_metrics": prepared.registration_metrics,
        "files": output_files,
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


def build_mosaic(
    inventory: CanvasInventory,
    output_dir: str,
    background_bgr: Optional[Tuple[int, int, int]] = None,
    progress_callback: Optional[Callable[[int, int, str], None]] = None,
    options: Optional[MosaicBuildOptions] = None,
) -> MosaicBuildResult:
    """Punto de entrada: HDF5 (por defecto) o memmap según ``output_mode``."""
    opts = options or MosaicBuildOptions()
    if opts.output_mode == "memmap":
        return build_mosaic_to_memmap(
            inventory, output_dir, background_bgr, progress_callback, opts
        )
    from core.canvas.hdf5_mosaic_store import build_mosaic_to_hdf5

    return build_mosaic_to_hdf5(
        inventory, output_dir, background_bgr, progress_callback, opts
    )


def build_mosaic_to_memmap(
    inventory: CanvasInventory,
    output_dir: str,
    background_bgr: Optional[Tuple[int, int, int]] = None,
    progress_callback: Optional[Callable[[int, int, str], None]] = None,
    options: Optional[MosaicBuildOptions] = None,
) -> MosaicBuildResult:
    """
    Construye canvas denso en memmap (modo legacy — usa mucho disco).

    Preferir ``build_mosaic`` con ``output_mode='hdf5'`` para evitar canvas denso.
    """
    opts = options or MosaicBuildOptions()
    if inventory.tile_width <= 0 or inventory.tile_height <= 0:
        raise ValueError("Inventario sin dimensiones de tile — escanee una carpeta con PNG válidos")

    os.makedirs(output_dir, exist_ok=True)
    removed = cleanup_orphan_canvas_temp_files(output_dir)
    if removed and progress_callback:
        progress_callback(0, len(inventory.tiles) + 2, f"Temporales eliminados: {', '.join(removed)}")

    prepared = prepare_mosaic_build(inventory, opts, background_bgr, progress_callback)
    background_bgr = prepared.background_bgr
    placements = prepared.placements
    offset_x, offset_y = prepared.offset_x, prepared.offset_y
    canvas_w, canvas_h = prepared.canvas_w, prepared.canvas_h
    grid = inventory.grid
    tw, th = inventory.tile_width, inventory.tile_height
    tiles = inventory.tiles

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
        progress_callback(
            step_i,
            total_steps,
            f"Rellenando fondo ({canvas_w}×{canvas_h} px, por bandas)...",
        )
    _fill_memmap_background(canvas, background_bgr)
    step_i += 1

    tiles_placed = 0
    for idx, tile in enumerate(tiles):
        img = _load_tile_image(tile, tw, th, background_bgr)

        if placements:
            px = int(round(placements[idx].final_px))
            py = int(round(placements[idx].final_py))
        else:
            px, py = _physical_pixel_origin(tile, grid, tw, th, swap_axes=opts.swap_stage_axes)
            px, py = int(round(px)), int(round(py))

        if opts.overlap_blend:
            _paste_tile_feather_blend(
                canvas, img, px, py, background_bgr, feather_px=opts.feather_px
            )
        else:
            _paste_tile_simple(canvas, img, px, py)

        tiles_placed += 1
        step_i += 1
        del img

        if progress_callback:
            progress_callback(
                step_i,
                total_steps,
                f"Tile {tiles_placed}/{len(tiles)} → ({px},{py}) px "
                f"[err {tile.error_x_um:+.1f},{tile.error_y_um:+.1f} µm]",
            )

    canvas.flush()

    if progress_callback:
        progress_callback(total_steps - 1, total_steps, "Generando preview...")
    if canvas_w * canvas_h > opts.preview_from_tiles_threshold_px:
        preview = _make_preview_from_tiles(
            tiles,
            placements,
            tw,
            th,
            canvas_w,
            canvas_h,
            background_bgr,
            opts.preview_max_side,
            opts.overlap_blend,
            opts.feather_px,
        )
    else:
        preview = _make_preview_from_memmap(canvas, opts.preview_max_side)
    cv2.imwrite(preview_path, preview, [cv2.IMWRITE_PNG_COMPRESSION, 3])
    del preview

    png_path = canvas_path.replace(".dat", ".png")
    exported_full = _export_memmap_png_banded(canvas, png_path, preview_path)
    build_time = time.perf_counter() - t0

    meta = _build_metadata_dict(
        inventory,
        opts,
        prepared,
        tiles_placed,
        build_time,
        stamp,
        {
            "memmap": canvas_path,
            "preview_png": preview_path,
            "full_png": png_path if exported_full else None,
            "hdf5": None,
        },
    )
    with open(metadata_path, "w", encoding="utf-8") as handle:
        json.dump(meta, handle, indent=2, ensure_ascii=False)

    _close_memmap(canvas)

    if progress_callback:
        progress_callback(total_steps, total_steps, "✅ Completado — canvas memmap")

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
        registration_metrics=prepared.registration_metrics,
        canvas_offset_px=(offset_x, offset_y),
    )


def _export_memmap_png_banded(
    canvas: np.memmap,
    png_path: str,
    preview_path: str,
    max_pixels: int = 120_000_000,
    band_rows: int = 512,
) -> bool:
    """Exporta PNG por bandas si cabe en disco; evita cargar canvas completo en RAM."""
    h, w = canvas.shape[:2]
    if h * w > max_pixels:
        return False
    try:
        out = np.empty((h, w, 3), dtype=np.uint8)
        for y in range(0, h, band_rows):
            y_end = min(y + band_rows, h)
            out[y:y_end] = canvas[y:y_end]
        cv2.imwrite(png_path, out, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        del out
        return os.path.isfile(png_path)
    except (MemoryError, cv2.error):
        return False

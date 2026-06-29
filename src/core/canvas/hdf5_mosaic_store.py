"""Almacén HDF5 tile-first — mosaico sin canvas denso en RAM ni en disco."""

from __future__ import annotations

import json
import os
import time
from datetime import datetime
from typing import Callable, Optional, Tuple

import cv2
import h5py
import numpy as np

from core.canvas.canvas_inventory import CanvasInventory
from core.canvas.mosaic_builder import (
    MosaicBuildOptions,
    MosaicBuildResult,
    _build_metadata_dict,
    _load_tile_image,
    _make_preview_from_tiles,
    cleanup_orphan_canvas_temp_files,
    prepare_mosaic_build,
)

HDF5_FORMAT_VERSION = 1


def build_mosaic_to_hdf5(
    inventory: CanvasInventory,
    output_dir: str,
    background_bgr: Optional[Tuple[int, int, int]] = None,
    progress_callback: Optional[Callable[[int, int, str], None]] = None,
    options: Optional[MosaicBuildOptions] = None,
) -> MosaicBuildResult:
    """
    Empaqueta tiles + metadatos en un .h5 — **sin** crear canvas denso.

    RAM: un tile a la vez + preview reducido (~4096 px).
    """
    opts = options or MosaicBuildOptions()
    if inventory.tile_width <= 0 or inventory.tile_height <= 0:
        raise ValueError("Inventario sin dimensiones de tile — escanee una carpeta con PNG válidos")

    os.makedirs(output_dir, exist_ok=True)
    tw, th = inventory.tile_width, inventory.tile_height
    tiles = inventory.tiles
    removed = cleanup_orphan_canvas_temp_files(output_dir)
    if removed and progress_callback:
        progress_callback(0, len(tiles) + 2, f"Temporales eliminados: {', '.join(removed)}")

    prepared = prepare_mosaic_build(inventory, opts, background_bgr, progress_callback)
    background_bgr = prepared.background_bgr
    placements = prepared.placements
    offset_x, offset_y = prepared.offset_x, prepared.offset_y
    canvas_w, canvas_h = prepared.canvas_w, prepared.canvas_h

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    class_part = inventory.class_name or "canvas"
    hdf5_path = os.path.join(output_dir, f"{class_part}_mosaic_{stamp}.h5")
    preview_path = os.path.join(output_dir, f"{class_part}_mosaic_{stamp}_preview.png")
    metadata_path = os.path.join(output_dir, f"{class_part}_mosaic_{stamp}_meta.json")

    t0 = time.perf_counter()
    total_steps = len(tiles) + 2
    step_i = 1

    with h5py.File(hdf5_path, "w") as h5:
        meta_grp = h5.create_group("meta")
        meta_grp.attrs["format_version"] = HDF5_FORMAT_VERSION
        meta_grp.attrs["timestamp"] = datetime.now().isoformat(timespec="seconds")
        meta_grp.attrs["class_name"] = inventory.class_name or ""
        meta_grp.attrs["folder"] = inventory.folder
        meta_grp.attrs["focal_layer"] = inventory.focal_layer
        meta_grp.attrs["canvas_width_px"] = canvas_w
        meta_grp.attrs["canvas_height_px"] = canvas_h
        meta_grp.attrs["canvas_offset_x_px"] = offset_x
        meta_grp.attrs["canvas_offset_y_px"] = offset_y
        meta_grp.attrs["tile_width_px"] = tw
        meta_grp.attrs["tile_height_px"] = th
        meta_grp.attrs["background_bgr"] = background_bgr

        grid = inventory.grid
        meta_grp.attrs["grid_x_min"] = grid.x_min
        meta_grp.attrs["grid_x_max"] = grid.x_max
        meta_grp.attrs["grid_y_min"] = grid.y_min
        meta_grp.attrs["grid_y_max"] = grid.y_max
        meta_grp.attrs["grid_fov_x"] = grid.fov_x
        meta_grp.attrs["grid_fov_y"] = grid.fov_y
        meta_grp.attrs["swap_stage_axes"] = opts.swap_stage_axes

        if placements:
            px_arr = np.array([p.final_px for p in placements], dtype=np.float32)
            py_arr = np.array([p.final_py for p in placements], dtype=np.float32)
            meta_grp.create_dataset("placement_px", data=px_arr)
            meta_grp.create_dataset("placement_py", data=py_arr)

        tiles_grp = h5.create_group("tiles")
        tiles_placed = 0

        for idx, tile in enumerate(tiles):
            img = _load_tile_image(tile, tw, th, background_bgr)
            tile_name = f"tile_{tile.seq:05d}"
            ds = tiles_grp.create_dataset(
                tile_name,
                data=img,
                compression="lzf",
                chunks=(min(th, 256), min(tw, 256), 3),
            )
            ds.attrs["seq"] = tile.seq
            ds.attrs["col"] = tile.col
            ds.attrs["row"] = tile.row
            ds.attrs["x_um"] = tile.x_um
            ds.attrs["y_um"] = tile.y_um
            ds.attrs["source_file"] = os.path.basename(tile.filepath)
            if tile.score is not None:
                ds.attrs["score"] = float(tile.score)
            if placements and idx < len(placements):
                ds.attrs["px"] = float(placements[idx].final_px)
                ds.attrs["py"] = float(placements[idx].final_py)
            if tile.placement_x_um is not None:
                ds.attrs["placement_x_um"] = float(tile.placement_x_um)
            if tile.placement_y_um is not None:
                ds.attrs["placement_y_um"] = float(tile.placement_y_um)

            tiles_placed += 1
            step_i += 1
            del img

            if progress_callback:
                px = int(round(placements[idx].final_px)) if placements else 0
                py = int(round(placements[idx].final_py)) if placements else 0
                progress_callback(
                    step_i,
                    total_steps,
                    f"HDF5 tile {tiles_placed}/{len(tiles)} → ({px},{py}) px",
                )

    build_time = time.perf_counter() - t0

    if progress_callback:
        progress_callback(total_steps - 1, total_steps, "Generando preview desde tiles...")
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
    cv2.imwrite(preview_path, preview, [cv2.IMWRITE_PNG_COMPRESSION, 3])
    del preview

    meta = _build_metadata_dict(
        inventory,
        opts,
        prepared,
        tiles_placed,
        build_time,
        stamp,
        {
            "hdf5": hdf5_path,
            "preview_png": preview_path,
            "memmap": None,
            "full_png": None,
        },
    )
    with open(metadata_path, "w", encoding="utf-8") as handle:
        json.dump(meta, handle, indent=2, ensure_ascii=False)

    if progress_callback:
        progress_callback(
            total_steps,
            total_steps,
            f"✅ Completado — HDF5 {os.path.getsize(hdf5_path) / (1024 * 1024):.1f} MiB",
        )

    return MosaicBuildResult(
        canvas_path=hdf5_path,
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
        hdf5_path=hdf5_path,
    )


def read_preview_from_hdf5(hdf5_path: str, max_side: int = 4096) -> Optional[np.ndarray]:
    """Regenera preview desde HDF5 sin canvas denso (solo lectura de tiles)."""
    if not os.path.isfile(hdf5_path):
        return None
    with h5py.File(hdf5_path, "r") as h5:
        meta = h5["meta"]
        canvas_w = int(meta.attrs["canvas_width_px"])
        canvas_h = int(meta.attrs["canvas_height_px"])
        tw = int(meta.attrs["tile_width_px"])
        th = int(meta.attrs["tile_height_px"])
        bg = tuple(int(v) for v in meta.attrs["background_bgr"])

        scale = max(canvas_w, canvas_h) / max(max_side, 1)
        pw = max(1, int(np.ceil(canvas_w / scale)))
        ph = max(1, int(np.ceil(canvas_h / scale)))
        preview = np.full((ph, pw, 3), bg, dtype=np.uint8)
        tile_w_s = max(1, int(round(tw / scale)))
        tile_h_s = max(1, int(round(th / scale)))

        tiles_grp = h5["tiles"]
        for name in sorted(tiles_grp.keys()):
            ds = tiles_grp[name]
            img = ds[()]
            px = int(round(float(ds.attrs.get("px", 0)) / scale))
            py = int(round(float(ds.attrs.get("py", 0)) / scale))
            img_small = cv2.resize(img, (tile_w_s, tile_h_s), interpolation=cv2.INTER_AREA)
            th_s, tw_s = img_small.shape[:2]
            x1, y1 = max(0, px), max(0, py)
            x2, y2 = min(pw, px + tw_s), min(ph, py + th_s)
            if x1 >= x2 or y1 >= y2:
                continue
            preview[y1:y2, x1:x2] = img_small[: y2 - y1, : x2 - x1]

        return np.ascontiguousarray(preview)

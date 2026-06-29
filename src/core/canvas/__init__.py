"""Mosaico de microscopía: inventario, fondo, registro y montaje por tiles."""

from .grid_config import GridConfig
from .canvas_inventory import CanvasInventory, TileRecord, scan_capture_folder
from .background_estimator import estimate_background_bgr
from .capture_position import CapturePositionMetadata, save_position_sidecar, load_position_metadata
from .backlash_model import BacklashCorrection, BacklashEstimator
from .position_metrics import PositionQualityMetrics, compute_position_metrics
from .tile_registration import RegistrationOffset, refine_placements, phase_correlate_pair
from .mosaic_builder import (
    MosaicBuildOptions,
    MosaicBuildResult,
    build_mosaic,
    build_mosaic_to_memmap,
    estimate_canvas_size_px,
)
from .hdf5_mosaic_store import build_mosaic_to_hdf5, read_preview_from_hdf5

__all__ = [
    "GridConfig",
    "CanvasInventory",
    "TileRecord",
    "scan_capture_folder",
    "estimate_background_bgr",
    "CapturePositionMetadata",
    "save_position_sidecar",
    "load_position_metadata",
    "BacklashCorrection",
    "BacklashEstimator",
    "PositionQualityMetrics",
    "compute_position_metrics",
    "RegistrationOffset",
    "refine_placements",
    "phase_correlate_pair",
    "MosaicBuildOptions",
    "MosaicBuildResult",
    "build_mosaic",
    "build_mosaic_to_memmap",
    "build_mosaic_to_hdf5",
    "read_preview_from_hdf5",
    "estimate_canvas_size_px",
]

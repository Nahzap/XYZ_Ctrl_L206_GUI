"""Worker en background para escaneo y montaje de canvas."""

import logging
import traceback
from typing import Optional

import cv2
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal

from core.canvas.canvas_inventory import CanvasInventory, scan_capture_folder
from core.canvas.grid_config import GridConfig
from core.canvas.mosaic_builder import MosaicBuildOptions, MosaicBuildResult, build_mosaic_to_memmap

logger = logging.getLogger("MotorControl_L206")


class CanvasBuildWorker(QThread):
    """Ejecuta inventario o montaje sin bloquear la UI."""

    progress_changed = pyqtSignal(int, int, str)
    inventory_ready = pyqtSignal(object)
    build_finished = pyqtSignal(object, object)
    failed = pyqtSignal(str)

    MODE_SCAN = "scan"
    MODE_BUILD = "build"

    def __init__(self, parent=None):
        super().__init__(parent)
        self._mode = self.MODE_SCAN
        self._folder = ""
        self._grid: Optional[GridConfig] = None
        self._focal_layer = "f1"
        self._output_dir = ""
        self._inventory: Optional[CanvasInventory] = None
        self._build_options = MosaicBuildOptions()

    def configure_scan(
        self,
        folder: str,
        grid: GridConfig,
        focal_layer: str,
        apply_backlash: bool = True,
    ):
        self._mode = self.MODE_SCAN
        self._folder = folder
        self._grid = grid
        self._focal_layer = focal_layer
        self._build_options.apply_backlash = apply_backlash

    def configure_build(
        self,
        inventory: CanvasInventory,
        output_dir: str,
        build_options: Optional[MosaicBuildOptions] = None,
    ):
        self._mode = self.MODE_BUILD
        self._inventory = inventory
        self._output_dir = output_dir
        if build_options is not None:
            self._build_options = build_options

    def run(self):
        try:
            if self._mode == self.MODE_SCAN:
                self._run_scan()
            else:
                self._run_build()
        except Exception as exc:
            logger.error("[CanvasBuildWorker] %s", exc)
            logger.debug(traceback.format_exc())
            self.failed.emit(str(exc))

    def _run_scan(self):
        self.progress_changed.emit(0, 1, "Escaneando carpeta...")
        inventory = scan_capture_folder(
            self._folder,
            self._grid,
            focal_layer=self._focal_layer,
            apply_backlash=self._build_options.apply_backlash,
        )
        self.progress_changed.emit(1, 1, f"{inventory.captured_cells} tiles encontrados")
        self.inventory_ready.emit(inventory)

    def _run_build(self):
        inv = self._inventory

        def on_progress(current: int, total: int, msg: str):
            self.progress_changed.emit(current, total, msg)

        result = build_mosaic_to_memmap(
            inv,
            self._output_dir,
            progress_callback=on_progress,
            options=self._build_options,
        )
        preview = cv2.imread(result.preview_path, cv2.IMREAD_COLOR)
        if preview is None:
            preview = np.zeros((100, 100, 3), dtype=np.uint8)
        self.build_finished.emit(result, preview)

"""Pestaña CanvasGen — mosaico de microscopía con fondo sintético."""

import logging
import os

import cv2
import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
)

from config.constants import DEFAULT_FOV_X_UM, DEFAULT_FOV_Y_UM
from core.canvas.grid_config import GridConfig
from core.canvas.mosaic_builder import MosaicBuildOptions
from core.services.canvas_build_worker import CanvasBuildWorker
from gui.tabs.base_tab import BaseTab
from gui.tabs.img_analysis_tab import ZoomableImageView
from utils.parameter_manager import get_parameter_manager

logger = logging.getLogger("MotorControl_L206")


class CanvasGenTab(BaseTab):
    """Generación de canvas a partir de capturas con coordenadas XY en µm."""

    def __init__(self, parent=None, test_tab=None):
        self._test_tab = test_tab
        self._worker: CanvasBuildWorker | None = None
        self._inventory = None
        self._last_result = None
        super().__init__(parent)

    def setup_ui(self):
        self.layout.setContentsMargins(8, 8, 8, 8)

        title = QLabel("CanvasGen — Mosaico por tiles (memoria optimizada)")
        title.setStyleSheet("font-size: 14px; font-weight: bold; color: #5DADE2;")
        self.layout.addWidget(title)

        self.layout.addWidget(self._build_input_group())
        self.layout.addWidget(self._build_options_group())
        self.layout.addWidget(self._build_actions_group())

        metrics_row = QHBoxLayout()
        self.metrics_label = QLabel("Sin escanear")
        self.metrics_label.setWordWrap(True)
        metrics_row.addWidget(self.metrics_label, stretch=2)

        self.coverage_label = QLabel("")
        metrics_row.addWidget(self.coverage_label, stretch=1)
        self.layout.addLayout(metrics_row)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.layout.addWidget(self.progress_bar)

        self.image_view = ZoomableImageView()
        self.layout.addWidget(self.image_view, stretch=1)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(120)
        self.layout.addWidget(self.log_text)

        self._load_defaults()

    def _build_input_group(self) -> QGroupBox:
        group = QGroupBox("Entrada y rejilla FOV")
        grid = QGridLayout(group)

        grid.addWidget(QLabel("Carpeta:"), 0, 0)
        self.folder_input = QLineEdit()
        grid.addWidget(self.folder_input, 0, 1, 1, 3)
        browse_btn = QPushButton("Examinar…")
        browse_btn.clicked.connect(self._browse_folder)
        grid.addWidget(browse_btn, 0, 4)

        grid.addWidget(QLabel("Capa focal:"), 1, 0)
        self.focal_combo = QComboBox()
        self.focal_combo.addItems(["f1 (BPoF)", "best (mayor S)", "f0", "f2"])
        grid.addWidget(self.focal_combo, 1, 1)

        grid.addWidget(QLabel("FOV X (µm):"), 1, 2)
        self.fov_x_spin = self._um_spin(DEFAULT_FOV_X_UM)
        grid.addWidget(self.fov_x_spin, 1, 3)

        grid.addWidget(QLabel("FOV Y (µm):"), 1, 4)
        self.fov_y_spin = self._um_spin(DEFAULT_FOV_Y_UM)
        grid.addWidget(self.fov_y_spin, 1, 5)

        for idx, (label, attr) in enumerate(
            [
                ("X min", "x_min_spin"),
                ("X max", "x_max_spin"),
                ("Y min", "y_min_spin"),
                ("Y max", "y_max_spin"),
            ]
        ):
            grid.addWidget(QLabel(f"{label} (µm):"), 2 + idx // 2, (idx % 2) * 2)
            spin = self._um_spin(12000.0)
            setattr(self, attr, spin)
            grid.addWidget(spin, 2 + idx // 2, (idx % 2) * 2 + 1)

        load_traj_btn = QPushButton("Cargar desde TestTab")
        load_traj_btn.clicked.connect(self._load_from_test_tab)
        grid.addWidget(load_traj_btn, 4, 0, 1, 2)

        return group

    def _build_options_group(self) -> QGroupBox:
        group = QGroupBox("Alineación de canvas")
        row = QHBoxLayout(group)

        self.cb_actual_position = QCheckBox("Posición real (sensor)")
        self.cb_actual_position.setChecked(True)
        self.cb_actual_position.setToolTip(
            "Coloca tiles según x_actual_um / y_actual_um del sidecar JSON"
        )
        row.addWidget(self.cb_actual_position)

        self.cb_backlash = QCheckBox("Corrección backlash")
        self.cb_backlash.setChecked(True)
        row.addWidget(self.cb_backlash)

        self.cb_visual_reg = QCheckBox("Registro visual")
        self.cb_visual_reg.setChecked(True)
        row.addWidget(self.cb_visual_reg)

        self.cb_overlap_blend = QCheckBox("Blend solapamiento")
        self.cb_overlap_blend.setChecked(True)
        row.addWidget(self.cb_overlap_blend)

        return group

    def _build_actions_group(self) -> QGroupBox:
        group = QGroupBox("Acciones")
        row = QHBoxLayout(group)

        self.scan_btn = QPushButton("1. Escanear carpeta")
        self.scan_btn.clicked.connect(self._start_scan)
        row.addWidget(self.scan_btn)

        self.build_btn = QPushButton("2. Generar canvas")
        self.build_btn.setEnabled(False)
        self.build_btn.clicked.connect(self._start_build)
        row.addWidget(self.build_btn)

        self.export_btn = QPushButton("Abrir carpeta de salida")
        self.export_btn.setEnabled(False)
        self.export_btn.clicked.connect(self._open_output_folder)
        row.addWidget(self.export_btn)

        return group

    @staticmethod
    def _um_spin(value: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(0, 100000)
        spin.setDecimals(1)
        spin.setValue(value)
        spin.setFixedWidth(100)
        return spin

    def _load_defaults(self):
        try:
            params = get_parameter_manager().get_trajectory_defaults()
            traj = params.get("x_range", {})
            tray = params.get("y_range", {})
            fov = params.get("fov", {})
            self.fov_x_spin.setValue(float(fov.get("x", DEFAULT_FOV_X_UM)))
            self.fov_y_spin.setValue(float(fov.get("y", DEFAULT_FOV_Y_UM)))
            self.x_min_spin.setValue(float(traj.get("min", 12000)))
            self.x_max_spin.setValue(float(traj.get("max", 15000)))
            self.y_min_spin.setValue(float(tray.get("min", 12400)))
            self.y_max_spin.setValue(float(tray.get("max", 15000)))
        except Exception as exc:
            logger.warning("[CanvasGenTab] Defaults: %s", exc)

    def _load_from_test_tab(self):
        if self._test_tab is None:
            self._log("TestTab no disponible")
            return
        try:
            if getattr(self._test_tab, "fov_x_input", None):
                self.fov_x_spin.setValue(float(self._test_tab.fov_x_input.text()))
            if getattr(self._test_tab, "fov_y_input", None):
                self.fov_y_spin.setValue(float(self._test_tab.fov_y_input.text()))
            if getattr(self._test_tab, "x_start_input", None):
                self.x_min_spin.setValue(float(self._test_tab.x_start_input.text()))
                self.x_max_spin.setValue(float(self._test_tab.x_end_input.text()))
            if getattr(self._test_tab, "y_start_input", None):
                self.y_min_spin.setValue(float(self._test_tab.y_start_input.text()))
                self.y_max_spin.setValue(float(self._test_tab.y_end_input.text()))
            self._log("Parámetros cargados desde TestTab")
        except (ValueError, AttributeError) as exc:
            self._log(f"Error cargando TestTab: {exc}")

    def _browse_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Carpeta de capturas")
        if folder:
            self.folder_input.setText(folder)

    def _grid_config(self) -> GridConfig:
        return GridConfig(
            x_min=self.x_min_spin.value(),
            x_max=self.x_max_spin.value(),
            y_min=self.y_min_spin.value(),
            y_max=self.y_max_spin.value(),
            fov_x=self.fov_x_spin.value(),
            fov_y=self.fov_y_spin.value(),
        )

    def _focal_layer(self) -> str:
        text = self.focal_combo.currentText()
        if text.startswith("best"):
            return "best"
        if text.startswith("f0"):
            return "f0"
        if text.startswith("f2"):
            return "f2"
        return "f1"

    def _set_busy(self, busy: bool):
        self.scan_btn.setEnabled(not busy)
        self.build_btn.setEnabled(not busy and self._inventory is not None)

    def _start_scan(self):
        folder = self.folder_input.text().strip()
        if not folder or not os.path.isdir(folder):
            QMessageBox.warning(self, "CanvasGen", "Seleccione una carpeta válida.")
            return
        self._set_busy(True)
        self._log(f"Escaneando {folder}…")
        self._worker = CanvasBuildWorker(self)
        self._worker.progress_changed.connect(self._on_progress)
        self._worker.inventory_ready.connect(self._on_inventory_ready)
        self._worker.failed.connect(self._on_failed)
        self._worker.finished.connect(lambda: self._set_busy(False))
        self._worker.configure_scan(
            folder,
            self._grid_config(),
            self._focal_layer(),
            apply_backlash=self.cb_backlash.isChecked(),
        )
        self._worker.start()

    def _start_build(self):
        if self._inventory is None:
            return
        folder = self.folder_input.text().strip()
        output_dir = os.path.join(folder, "_canvas_output")
        self._set_busy(True)
        self._log("Montando canvas por tiles (memmap)…")
        self._worker = CanvasBuildWorker(self)
        self._worker.progress_changed.connect(self._on_progress)
        self._worker.build_finished.connect(self._on_build_finished)
        self._worker.failed.connect(self._on_failed)
        self._worker.finished.connect(lambda: self._set_busy(False))
        self._worker.configure_build(
            self._inventory,
            output_dir,
            build_options=self._build_options(),
        )
        self._worker.start()

    def _on_progress(self, current: int, total: int, message: str):
        if total > 0:
            self.progress_bar.setValue(int(100 * current / total))
        self._log(message)

    def _build_options(self) -> MosaicBuildOptions:
        return MosaicBuildOptions(
            use_actual_position=self.cb_actual_position.isChecked(),
            apply_backlash=self.cb_backlash.isChecked(),
            visual_registration=self.cb_visual_reg.isChecked(),
            overlap_blend=self.cb_overlap_blend.isChecked(),
        )

    def _on_inventory_ready(self, inventory):
        self._inventory = inventory
        self.build_btn.setEnabled(True)
        g = inventory.grid
        metrics_text = ""
        if inventory.position_metrics is not None:
            m = inventory.position_metrics
            metrics_text = (
                f" | Err σ X/Y: {m.std_error_x_um:.1f}/{m.std_error_y_um:.1f} µm"
                f" | RMSE: {m.rmse_um:.1f} µm"
                f" | Con sensor: {m.with_actual_position}/{m.tile_count}"
            )
        self.metrics_label.setText(
            f"Tiles: {len(inventory.tiles)} | "
            f"Celdas: {inventory.captured_cells}/{inventory.total_cells} | "
            f"Rejilla: {g.n_cols}×{g.n_rows} | "
            f"Tile: {inventory.tile_width}×{inventory.tile_height} px | "
            f"Clase: {inventory.class_name or '—'}"
            f"{metrics_text}"
        )
        self.coverage_label.setText(f"Cobertura: {inventory.coverage_percent:.1f} %")
        self._log(
            f"Inventario listo — {inventory.captured_cells} celdas con datos "
            f"({inventory.coverage_percent:.1f} %)"
        )
        if inventory.position_metrics is not None:
            pm = inventory.position_metrics.to_dict()
            self._log(
                f"Métricas posición: RMSE={pm['rmse_um']} µm, "
                f"max|err| X/Y={pm['max_abs_error_x_um']}/{pm['max_abs_error_y_um']} µm, "
                f"legacy={pm['legacy_nominal_only']}"
            )

    def _on_build_finished(self, result, preview: np.ndarray):
        self._last_result = result
        self.export_btn.setEnabled(True)
        self.image_view.set_image(preview)
        self.image_view.reset_view()
        self.metrics_label.setText(
            self.metrics_label.text()
            + f" | Canvas: {result.width_px}×{result.height_px} px"
            + f" | Fondo BGR{result.background_bgr}"
            + f" | {result.build_time_s:.1f}s"
        )
        if result.registration_metrics:
            rm = result.registration_metrics
            self._log(
                f"Registro visual: {rm.get('registrations_applied', 0)}/"
                f"{rm.get('neighbor_pairs', 0)} pares, "
                f"resp media={rm.get('mean_response', 0):.3f}"
            )
        self._log(f"Canvas generado: {result.preview_path}")
        self._log(f"Metadatos: {result.metadata_path}")
        if result.canvas_path != result.preview_path:
            self._log(f"PNG completo: {result.canvas_path}")

    def _on_failed(self, message: str):
        self._set_busy(False)
        self._log(f"ERROR: {message}")
        QMessageBox.critical(self, "CanvasGen", message)

    def _open_output_folder(self):
        if self._last_result is None:
            return
        folder = os.path.dirname(self._last_result.preview_path)
        os.startfile(folder)

    def _log(self, message: str):
        self.log_text.append(message)
        logger.info("[CanvasGenTab] %s", message)

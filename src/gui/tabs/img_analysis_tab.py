"""
Pestaña de Análisis de Imagen - U2-Net Salient Object Detection.

Detecta objetos salientes usando U2-Net y calcula su score de enfoque.
"""

import os
import logging
import cv2
import numpy as np
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QGroupBox, QFileDialog, QListWidget, QListWidgetItem, QSplitter,
    QCheckBox, QSpinBox, QDoubleSpinBox, QSizePolicy, QTableWidget,
    QTableWidgetItem
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap, QFont

from img_analysis.smart_focus_scorer import SmartFocusScorer, FocusResult

logger = logging.getLogger('MotorControl_L206')


class ZoomableImageView(QLabel):
    """Vista de imagen con zoom (rueda) y pan (arrastre)."""
    
    def __init__(self):
        super().__init__()
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background-color: #1e1e1e; border: 1px solid #3c3c3c;")
        self.setMinimumSize(400, 300)
        self._image = None
        self._zoom = 1.0
        self._center_x = 0.5
        self._center_y = 0.5
        self._dragging = False
        self._last_pos = None
    
    def set_image(self, img_bgr: np.ndarray):
        """Establece la imagen a mostrar."""
        self._image = img_bgr.copy()
        self._update_display()
    
    def reset_view(self):
        """Resetea zoom y pan."""
        self._zoom = 1.0
        self._center_x = 0.5
        self._center_y = 0.5
        self._update_display()
    
    def _update_display(self):
        """Actualiza la visualización con zoom real (recorte)."""
        if self._image is None:
            return
        
        img = self._image
        h, w = img.shape[:2]
        
        # Calcular región visible
        view_w = max(1, int(w / self._zoom))
        view_h = max(1, int(h / self._zoom))
        
        cx = int(self._center_x * w)
        cy = int(self._center_y * h)
        
        x1 = max(0, min(w - view_w, cx - view_w // 2))
        y1 = max(0, min(h - view_h, cy - view_h // 2))
        x2 = min(w, x1 + view_w)
        y2 = min(h, y1 + view_h)
        
        cropped = img[y1:y2, x1:x2]
        
        # Indicador de zoom
        if self._zoom > 1.0:
            cv2.putText(cropped, f"x{self._zoom:.1f}", (cropped.shape[1] - 50, 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Convertir a QPixmap
        img_rgb = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
        img_rgb = np.ascontiguousarray(img_rgb)
        qimg = QImage(img_rgb.data, img_rgb.shape[1], img_rgb.shape[0],
                     img_rgb.shape[1] * 3, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        self.setPixmap(pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
    
    def wheelEvent(self, event):
        """Zoom con rueda."""
        if self._image is None:
            return
        delta = event.angleDelta().y()
        factor = 1.2 if delta > 0 else 0.83
        self._zoom = max(1.0, min(8.0, self._zoom * factor))
        self._update_display()
    
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self._zoom > 1.0:
            self._dragging = True
            self._last_pos = event.pos()
    
    def mouseMoveEvent(self, event):
        if self._dragging and self._last_pos:
            dx = event.x() - self._last_pos.x()
            dy = event.y() - self._last_pos.y()
            sens = 0.003 / self._zoom
            self._center_x = max(0, min(1, self._center_x - dx * sens))
            self._center_y = max(0, min(1, self._center_y - dy * sens))
            self._last_pos = event.pos()
            self._update_display()
    
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._dragging = False
    
    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_display()


class ImgAnalysisTab(QWidget):
    """Pestaña de análisis de imagen con U2-Net."""
    
    analysis_completed = pyqtSignal(str, float)  # filename, score
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Scorer U2-Net
        self.scorer = SmartFocusScorer(
            model_type='u2netp',
            threshold=0.5,
            min_area=500,
            min_prob=0.3,
            focus_threshold=20.0,
            min_circularity=0.45,
            min_aspect_ratio=0.4
        )
        
        self._current_result = None
        self._current_image = None
        self._layers = {}
        
        self._setup_ui()
        logger.debug("ImgAnalysisTab (U2-Net) inicializado")
    
    def _setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)
        
        # === Fila 1: Controles ===
        controls = QHBoxLayout()
        
        self.btn_load = QPushButton("📁 Cargar Imágenes")
        self.btn_load.clicked.connect(self._load_images)
        controls.addWidget(self.btn_load)
        
        self.label_folder = QLabel("Sin carpeta")
        self.label_folder.setStyleSheet("color: #888;")
        controls.addWidget(self.label_folder, 1)
        
        controls.addWidget(QLabel("Saliencia:"))
        self.spin_threshold = QDoubleSpinBox()
        self.spin_threshold.setRange(0.1, 0.9)
        self.spin_threshold.setValue(0.5)
        self.spin_threshold.setSingleStep(0.05)
        self.spin_threshold.valueChanged.connect(self._on_params_changed)
        controls.addWidget(self.spin_threshold)
        
        controls.addWidget(QLabel("Área Mín:"))
        self.spin_min_area = QSpinBox()
        self.spin_min_area.setRange(100, 50000)
        self.spin_min_area.setValue(500)
        self.spin_min_area.valueChanged.connect(self._on_params_changed)
        controls.addWidget(self.spin_min_area)
        
        controls.addWidget(QLabel("Focus:"))
        self.spin_focus = QDoubleSpinBox()
        self.spin_focus.setRange(1, 100)
        self.spin_focus.setValue(20)
        self.spin_focus.valueChanged.connect(self._on_params_changed)
        controls.addWidget(self.spin_focus)
        
        main_layout.addLayout(controls)
        
        # === Fila 2: Splitter principal ===
        splitter = QSplitter(Qt.Horizontal)
        
        # Panel izquierdo: Lista de imágenes
        self.image_list = QListWidget()
        self.image_list.setMaximumWidth(220)
        self.image_list.setStyleSheet("""
            QListWidget { background: #1e1e1e; color: #fff; font-size: 11px; }
            QListWidget::item { padding: 4px; }
            QListWidget::item:selected { background: #0078d4; }
        """)
        self.image_list.currentItemChanged.connect(self._on_image_selected)
        splitter.addWidget(self.image_list)
        
        # Panel central: Visor con overlays
        viewer_panel = QWidget()
        viewer_layout = QVBoxLayout(viewer_panel)
        viewer_layout.setContentsMargins(0, 0, 0, 0)
        
        # Controles de overlay
        overlay_row = QHBoxLayout()
        overlay_row.addWidget(QLabel("Capas:"))
        
        self.cb_prob = QCheckBox("Prob")
        self.cb_prob.stateChanged.connect(self._refresh_view)
        overlay_row.addWidget(self.cb_prob)
        
        self.cb_mask = QCheckBox("Mask")
        self.cb_mask.stateChanged.connect(self._refresh_view)
        overlay_row.addWidget(self.cb_mask)
        
        self.cb_focus = QCheckBox("Focus")
        self.cb_focus.stateChanged.connect(self._refresh_view)
        overlay_row.addWidget(self.cb_focus)
        
        self.cb_boxes = QCheckBox("Boxes")
        self.cb_boxes.setChecked(True)
        self.cb_boxes.stateChanged.connect(self._refresh_view)
        overlay_row.addWidget(self.cb_boxes)
        
        overlay_row.addWidget(QLabel("|"))
        self.cb_lock = QCheckBox("🔒")
        self.cb_lock.setToolTip("Bloquear capas al cambiar imagen")
        overlay_row.addWidget(self.cb_lock)
        
        self.btn_reset_zoom = QPushButton("🔍")
        self.btn_reset_zoom.setFixedWidth(30)
        self.btn_reset_zoom.clicked.connect(lambda: self.image_view.reset_view())
        overlay_row.addWidget(self.btn_reset_zoom)
        
        overlay_row.addStretch()
        viewer_layout.addLayout(overlay_row)
        
        # Visor
        self.image_view = ZoomableImageView()
        viewer_layout.addWidget(self.image_view, 1)
        
        splitter.addWidget(viewer_panel)
        
        # Panel derecho: Tabla de objetos
        objects_panel = QWidget()
        objects_layout = QVBoxLayout(objects_panel)
        objects_layout.setContentsMargins(0, 0, 0, 0)
        
        objects_layout.addWidget(QLabel("📋 Objetos"))
        
        self.objects_table = QTableWidget()
        self.objects_table.setColumnCount(3)
        self.objects_table.setHorizontalHeaderLabels(["#", "Score", "Estado"])
        self.objects_table.setMaximumWidth(180)
        self.objects_table.setStyleSheet("""
            QTableWidget { background: #1e1e1e; color: #fff; font-size: 11px; }
            QHeaderView::section { background: #2d2d2d; color: #fff; }
        """)
        objects_layout.addWidget(self.objects_table)
        
        self.label_summary = QLabel("Total: 0")
        self.label_summary.setStyleSheet("color: #3498DB; font-weight: bold;")
        objects_layout.addWidget(self.label_summary)
        
        splitter.addWidget(objects_panel)
        
        splitter.setSizes([200, 500, 150])
        main_layout.addWidget(splitter, 1)
    
    def _load_images(self):
        """Carga imágenes de una carpeta."""
        folder = QFileDialog.getExistingDirectory(self, "Seleccionar carpeta de imágenes")
        if not folder:
            return
        
        self.label_folder.setText(os.path.basename(folder))
        self.image_list.clear()
        
        extensions = ('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff')
        files = sorted([f for f in os.listdir(folder) if f.lower().endswith(extensions)])
        
        for f in files:
            item = QListWidgetItem(f)
            item.setData(Qt.UserRole, os.path.join(folder, f))
            self.image_list.addItem(item)
        
        logger.info(f"Cargadas {len(files)} imágenes de {folder}")
    
    def _on_params_changed(self):
        """Actualiza parámetros del scorer."""
        self.scorer.set_parameters(
            threshold=self.spin_threshold.value(),
            min_area=self.spin_min_area.value(),
            focus_threshold=self.spin_focus.value()
        )
        # Re-analizar imagen actual
        if self._current_image is not None:
            self._analyze_current()
    
    def _on_image_selected(self, current, previous):
        """Analiza la imagen seleccionada."""
        if current is None:
            return
        
        filepath = current.data(Qt.UserRole)
        img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
        if img is None:
            return
        
        self._current_image = img
        self._analyze_current()
    
    def _analyze_current(self):
        """Analiza la imagen actual."""
        if self._current_image is None:
            return
        
        img = self._current_image
        result = self.scorer.assess_image(img)
        self._current_result = result
        
        # Pre-calcular capas
        self._calculate_layers(img, result)
        
        # Actualizar vista
        self._refresh_view()
        
        # Actualizar tabla
        self._update_objects_table(result)
        
        # Emitir señal
        self.analysis_completed.emit("", result.focus_score)
    
    def _calculate_layers(self, img: np.ndarray, result: FocusResult):
        """Pre-calcula capas de overlay."""
        h, w = img.shape[:2]
        
        self._layers['base'] = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)
        
        if result.probability_map is not None:
            prob = (result.probability_map * 255).astype(np.uint8)
            self._layers['prob'] = cv2.applyColorMap(prob, cv2.COLORMAP_JET)
        else:
            self._layers['prob'] = None
        
        self._layers['mask'] = result.binary_mask
        
        # Focus map
        lap = np.abs(cv2.Laplacian(img, cv2.CV_64F))
        lap_max = lap.max()
        if lap_max > 0:
            lap_norm = (lap / lap_max * 255).astype(np.uint8)
        else:
            lap_norm = np.zeros_like(img, dtype=np.uint8)
        if result.binary_mask is not None:
            lap_norm = cv2.bitwise_and(lap_norm, result.binary_mask)
        self._layers['focus'] = cv2.applyColorMap(lap_norm, cv2.COLORMAP_HOT)
    
    def _refresh_view(self):
        """Compone imagen con overlays seleccionados."""
        if 'base' not in self._layers:
            return
        
        img = self._layers['base'].copy()
        
        if self.cb_prob.isChecked() and self._layers.get('prob') is not None:
            img = cv2.addWeighted(img, 0.5, self._layers['prob'], 0.5, 0)
        
        if self.cb_focus.isChecked() and self._layers.get('focus') is not None:
            focus_gray = cv2.cvtColor(self._layers['focus'], cv2.COLOR_BGR2GRAY)
            mask = focus_gray > 10
            img[mask] = cv2.addWeighted(img, 0.4, self._layers['focus'], 0.6, 0)[mask]
        
        if self.cb_mask.isChecked() and self._layers.get('mask') is not None:
            contours, _ = cv2.findContours(self._layers['mask'], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(img, contours, -1, (255, 0, 255), 1)
        
        if self.cb_boxes.isChecked() and self._current_result:
            for i, obj in enumerate(self._current_result.objects):
                color = (0, 255, 0) if obj.is_focused else (0, 0, 255)
                x, y, bw, bh = obj.bounding_box
                cv2.rectangle(img, (x, y), (x + bw, y + bh), color, 2)
                text = f"#{i+1}: {obj.focus_score:.1f}"
                cv2.putText(img, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        self.image_view.set_image(img)
    
    def _update_objects_table(self, result: FocusResult):
        """Actualiza tabla de objetos."""
        objects = result.objects
        self.objects_table.setRowCount(len(objects))
        
        n_focused = 0
        for i, obj in enumerate(objects):
            self.objects_table.setItem(i, 0, QTableWidgetItem(f"{i+1}"))
            
            score_item = QTableWidgetItem(f"{obj.focus_score:.1f}")
            score_item.setForeground(Qt.green if obj.is_focused else Qt.red)
            self.objects_table.setItem(i, 1, score_item)
            
            self.objects_table.setItem(i, 2, QTableWidgetItem("✓" if obj.is_focused else "✗"))
            
            if obj.is_focused:
                n_focused += 1
        
        self.objects_table.resizeColumnsToContents()
        self.label_summary.setText(f"Total: {len(objects)} | ✓: {n_focused}")

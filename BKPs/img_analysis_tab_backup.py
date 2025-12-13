"""
Pestaña de Análisis de Imagen - Evaluación de Enfoque.

Soporta dos métodos:
1. SMART (Blind Assessment): Entropía + Brenner - Sin calibración
2. LEGACY (Z-Score): Modelo de fondo + Laplaciano - Requiere calibración

Incluye scroll vertical para extensibilidad futura.
"""

import os
import logging
import cv2
import numpy as np
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QGroupBox, QFileDialog, QProgressBar, QListWidget, QListWidgetItem,
    QSplitter, QCheckBox, QSpinBox, QDoubleSpinBox, QSlider, QComboBox,
    QSizePolicy, QScrollArea, QRadioButton, QTableWidget, QTableWidgetItem,
    QStackedWidget, QTabWidget, QMessageBox, QFrame
)
from PyQt5.QtCore import Qt, pyqtSignal, QThread, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap, QFont

from img_analysis.background_model import (
    train_background_model, load_background_model, list_images_in_folder
)
from img_analysis.sharpness_detector import (
    SharpnessDetector, SharpnessResult, create_debug_composite
)
from img_analysis.smart_focus_scorer import (
    SmartFocusScorer, FocusResult, ObjectInfo
)

logger = logging.getLogger('MotorControl_L206')


class CalibrationWorker(QThread):
    """Worker thread para calibración en background."""
    
    progress = pyqtSignal(int, int, str)  # current, total, message
    finished = pyqtSignal(bool, str)  # success, message
    
    def __init__(self, folder_path, output_dir=None):
        super().__init__()
        self.folder_path = folder_path
        self.output_dir = output_dir
    
    def run(self):
        def progress_callback(current, total, msg):
            self.progress.emit(current, total, msg)
        
        success, message, _, _ = train_background_model(
            self.folder_path,
            self.output_dir,
            progress_callback
        )
        self.finished.emit(success, message)


class ZoomableLabel(QLabel):
    """QLabel con zoom real (recorta región) y pan con arrastre."""
    
    zoom_changed = pyqtSignal(float, float, float)  # zoom, center_x_ratio, center_y_ratio
    
    def __init__(self, title: str = ""):
        super().__init__()
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background-color: #1e1e1e; border: 1px solid #3c3c3c;")
        self._title = title
        self._zoom = 1.0
        self._center_x = 0.5  # Centro de vista (0-1)
        self._center_y = 0.5
        self._full_image = None
        self._dragging = False
        self._last_pos = None
        self.setMouseTracking(True)
    
    def set_image(self, img_bgr: np.ndarray, title: str = None):
        """Establece la imagen a mostrar."""
        if title:
            self._title = title
        self._full_image = img_bgr.copy()
        self._update_display()
    
    def set_zoom(self, zoom: float, cx: float = None, cy: float = None):
        """Establece el nivel de zoom y centro."""
        self._zoom = max(1.0, min(10.0, zoom))
        if cx is not None:
            self._center_x = max(0.0, min(1.0, cx))
        if cy is not None:
            self._center_y = max(0.0, min(1.0, cy))
        self._update_display()
    
    def reset_view(self):
        """Resetea zoom y pan."""
        self._zoom = 1.0
        self._center_x = 0.5
        self._center_y = 0.5
        self._update_display()
    
    def _update_display(self):
        """Actualiza la visualización con zoom real (recorte de región)."""
        if self._full_image is None:
            return
        
        img = self._full_image
        h, w = img.shape[:2]
        
        # Calcular región visible según zoom
        view_w = int(w / self._zoom)
        view_h = int(h / self._zoom)
        
        # Centro de la vista
        cx = int(self._center_x * w)
        cy = int(self._center_y * h)
        
        # Límites de la región
        x1 = max(0, cx - view_w // 2)
        y1 = max(0, cy - view_h // 2)
        x2 = min(w, x1 + view_w)
        y2 = min(h, y1 + view_h)
        
        # Ajustar si toca bordes
        if x2 - x1 < view_w:
            x1 = max(0, x2 - view_w)
        if y2 - y1 < view_h:
            y1 = max(0, y2 - view_h)
        
        # Recortar región
        cropped = img[y1:y2, x1:x2].copy()
        
        # Dibujar título en la esquina
        cv2.putText(cropped, self._title, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.45, (0, 0, 0), 2)
        cv2.putText(cropped, self._title, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.45, (255, 255, 255), 1)
        
        # Indicador de zoom
        if self._zoom > 1.0:
            zoom_text = f"x{self._zoom:.1f}"
            cv2.putText(cropped, zoom_text, (cropped.shape[1] - 45, 16), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
        # Convertir a QPixmap
        if len(cropped.shape) == 2:
            img_rgb = cv2.cvtColor(cropped, cv2.COLOR_GRAY2RGB)
        else:
            img_rgb = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
        
        img_rgb = np.ascontiguousarray(img_rgb)
        qimg = QImage(img_rgb.data, img_rgb.shape[1], img_rgb.shape[0], 
                     img_rgb.shape[1] * 3, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        
        # Escalar al tamaño del widget
        self.setPixmap(pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
    
    def wheelEvent(self, event):
        """Zoom con rueda del mouse centrado en cursor."""
        if self._full_image is None:
            return
        
        # Calcular posición relativa del cursor
        widget_w, widget_h = self.width(), self.height()
        mouse_x = event.x() / widget_w
        mouse_y = event.y() / widget_h
        
        # Zoom
        delta = event.angleDelta().y()
        factor = 1.2 if delta > 0 else 0.83
        old_zoom = self._zoom
        self._zoom = max(1.0, min(10.0, self._zoom * factor))
        
        # Ajustar centro hacia el cursor al hacer zoom in
        if self._zoom > old_zoom:
            self._center_x += (mouse_x - 0.5) * 0.1
            self._center_y += (mouse_y - 0.5) * 0.1
            self._center_x = max(0.0, min(1.0, self._center_x))
            self._center_y = max(0.0, min(1.0, self._center_y))
        
        self._update_display()
        self.zoom_changed.emit(self._zoom, self._center_x, self._center_y)
    
    def mousePressEvent(self, event):
        """Inicia arrastre para pan."""
        if event.button() == Qt.LeftButton and self._zoom > 1.0:
            self._dragging = True
            self._last_pos = event.pos()
    
    def mouseMoveEvent(self, event):
        """Pan con arrastre."""
        if self._dragging and self._last_pos:
            dx = event.x() - self._last_pos.x()
            dy = event.y() - self._last_pos.y()
            
            # Mover centro (invertido para sensación natural)
            sensitivity = 0.002 / self._zoom
            self._center_x -= dx * sensitivity * 2
            self._center_y -= dy * sensitivity * 2
            self._center_x = max(0.0, min(1.0, self._center_x))
            self._center_y = max(0.0, min(1.0, self._center_y))
            
            self._last_pos = event.pos()
            self._update_display()
            self.zoom_changed.emit(self._zoom, self._center_x, self._center_y)
    
    def mouseReleaseEvent(self, event):
        """Termina arrastre."""
        if event.button() == Qt.LeftButton:
            self._dragging = False
            self._last_pos = None
    
    def resizeEvent(self, event):
        """Actualiza al redimensionar."""
        super().resizeEvent(event)
        self._update_display()


class SingleViewWithOverlays(QWidget):
    """Vista única con overlays seleccionables y lista de objetos detectados."""
    
    def __init__(self):
        super().__init__()
        self._setup_ui()
        self._focus_result = None
        self._image = None
        self._layers = {}  # Capas pre-calculadas
    
    def _setup_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(4)
        
        # === Panel izquierdo: Visor con zoom ===
        viewer_panel = QVBoxLayout()
        
        # Controles de overlay
        overlay_row = QHBoxLayout()
        overlay_row.addWidget(QLabel("Capas:"))
        
        self.cb_prob = QCheckBox("Prob")
        self.cb_prob.setToolTip("Mapa de probabilidad U2-Net")
        self.cb_prob.stateChanged.connect(self._refresh_composite)
        overlay_row.addWidget(self.cb_prob)
        
        self.cb_mask = QCheckBox("Mask")
        self.cb_mask.setToolTip("Contornos de máscara binaria")
        self.cb_mask.stateChanged.connect(self._refresh_composite)
        overlay_row.addWidget(self.cb_mask)
        
        self.cb_focus = QCheckBox("Focus")
        self.cb_focus.setToolTip("Mapa de enfoque (Laplaciano)")
        self.cb_focus.stateChanged.connect(self._refresh_composite)
        overlay_row.addWidget(self.cb_focus)
        
        self.cb_boxes = QCheckBox("Boxes")
        self.cb_boxes.setChecked(True)
        self.cb_boxes.setToolTip("Bounding boxes y scores")
        self.cb_boxes.stateChanged.connect(self._refresh_composite)
        overlay_row.addWidget(self.cb_boxes)
        
        overlay_row.addWidget(QLabel("|"))
        
        self.cb_lock = QCheckBox("🔒 Lock")
        self.cb_lock.setToolTip("Bloquear capas al cambiar de imagen")
        overlay_row.addWidget(self.cb_lock)
        
        overlay_row.addStretch()
        
        self.btn_reset_zoom = QPushButton("🔍 Reset")
        self.btn_reset_zoom.setFixedWidth(60)
        self.btn_reset_zoom.clicked.connect(self._reset_zoom)
        overlay_row.addWidget(self.btn_reset_zoom)
        
        viewer_panel.addLayout(overlay_row)
        
        # Visor principal
        self.main_view = ZoomableLabel("Vista Principal")
        self.main_view.setMinimumSize(400, 300)
        viewer_panel.addWidget(self.main_view, stretch=1)
        
        main_layout.addLayout(viewer_panel, stretch=3)
        
        # === Panel derecho: Lista de objetos ===
        objects_panel = QVBoxLayout()
        objects_panel.addWidget(QLabel("📋 Objetos Detectados"))
        
        self.objects_table = QTableWidget()
        self.objects_table.setColumnCount(4)
        self.objects_table.setHorizontalHeaderLabels(["#", "Score", "Estado", "Área"])
        self.objects_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.objects_table.setSelectionMode(QTableWidget.SingleSelection)
        self.objects_table.setMinimumWidth(200)
        self.objects_table.setMaximumWidth(250)
        self.objects_table.horizontalHeader().setStretchLastSection(True)
        self.objects_table.setStyleSheet("""
            QTableWidget { background-color: #1e1e1e; color: #fff; font-size: 11px; }
            QTableWidget::item:selected { background-color: #0078d4; }
            QHeaderView::section { background-color: #2d2d2d; color: #fff; padding: 4px; }
        """)
        self.objects_table.itemSelectionChanged.connect(self._on_object_selected)
        objects_panel.addWidget(self.objects_table)
        
        # Resumen
        self.label_summary = QLabel("Total: 0 | Focused: 0")
        self.label_summary.setStyleSheet("color: #3498DB; font-weight: bold;")
        objects_panel.addWidget(self.label_summary)
        
        main_layout.addLayout(objects_panel, stretch=1)
    
    def _reset_zoom(self):
        self.main_view.reset_view()
    
    def reset_zoom(self):
        self.main_view.reset_view()
    
    def display_result(self, img_gray: np.ndarray, focus_result: FocusResult):
        """Muestra imagen con overlays según checkboxes."""
        self._image = img_gray
        self._focus_result = focus_result
        
        # Pre-calcular capas
        self._calculate_layers()
        
        # Actualizar tabla de objetos
        self._update_objects_table()
        
        # Refrescar vista compuesta
        self._refresh_composite()
    
    def _calculate_layers(self):
        """Pre-calcula todas las capas de overlay."""
        if self._image is None or self._focus_result is None:
            return
        
        img = self._image
        fr = self._focus_result
        h, w = img.shape[:2]
        
        # Capa base (original en BGR)
        self._layers['base'] = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)
        
        # Capa de probabilidad (heatmap semi-transparente)
        if fr.probability_map is not None:
            prob_uint8 = (fr.probability_map * 255).astype(np.uint8)
            self._layers['prob'] = cv2.applyColorMap(prob_uint8, cv2.COLORMAP_JET)
        else:
            self._layers['prob'] = None
        
        # Capa de máscara (contornos)
        self._layers['mask'] = fr.binary_mask if fr.binary_mask is not None else None
        
        # Capa de focus (Laplaciano)
        laplacian = cv2.Laplacian(img, cv2.CV_64F)
        lap_abs = np.abs(laplacian)
        lap_max = lap_abs.max()
        if lap_max > 0:
            lap_norm = (lap_abs / lap_max * 255).astype(np.uint8)
        else:
            lap_norm = np.zeros_like(img, dtype=np.uint8)
        
        if fr.binary_mask is not None:
            lap_norm = cv2.bitwise_and(lap_norm, fr.binary_mask)
        
        self._layers['focus'] = cv2.applyColorMap(lap_norm, cv2.COLORMAP_HOT)
    
    def _refresh_composite(self):
        """Compone la imagen final según checkboxes activos."""
        if self._image is None or 'base' not in self._layers:
            return
        
        # Empezar con imagen base
        composite = self._layers['base'].copy()
        h, w = composite.shape[:2]
        
        # Superponer probabilidad
        if self.cb_prob.isChecked() and self._layers.get('prob') is not None:
            composite = cv2.addWeighted(composite, 0.5, self._layers['prob'], 0.5, 0)
        
        # Superponer focus map
        if self.cb_focus.isChecked() and self._layers.get('focus') is not None:
            # Solo donde hay focus (no negro)
            focus_gray = cv2.cvtColor(self._layers['focus'], cv2.COLOR_BGR2GRAY)
            mask = focus_gray > 10
            composite[mask] = cv2.addWeighted(
                composite, 0.4, self._layers['focus'], 0.6, 0
            )[mask]
        
        # Dibujar contornos de máscara
        if self.cb_mask.isChecked() and self._layers.get('mask') is not None:
            contours, _ = cv2.findContours(
                self._layers['mask'], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(composite, contours, -1, (255, 0, 255), 1)
        
        # Dibujar boxes y scores
        if self.cb_boxes.isChecked() and self._focus_result:
            for i, obj in enumerate(self._focus_result.objects):
                color = (0, 255, 0) if obj.is_focused else (0, 0, 255)
                x, y, bw, bh = obj.bounding_box
                
                # Bounding box
                cv2.rectangle(composite, (x, y), (x + bw, y + bh), color, 2)
                
                # Score arriba a la derecha del bbox
                score_text = f"#{i+1}: {obj.focus_score:.1f}"
                text_x = x + bw - 5
                text_y = y - 8 if y > 25 else y + bh + 15
                (tw, th), _ = cv2.getTextSize(score_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                cv2.rectangle(composite, (text_x - tw - 4, text_y - th - 4), 
                             (text_x + 4, text_y + 4), (0, 0, 0), -1)
                cv2.putText(composite, score_text, (text_x - tw, text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        
        self.main_view.set_image(composite)
    
    def _update_objects_table(self):
        """Actualiza la tabla de objetos detectados."""
        if self._focus_result is None:
            self.objects_table.setRowCount(0)
            self.label_summary.setText("Total: 0 | Focused: 0")
            return
        
        objects = self._focus_result.objects
        self.objects_table.setRowCount(len(objects))
        
        n_focused = 0
        for i, obj in enumerate(objects):
            # Número
            item_num = QTableWidgetItem(f"{i+1}")
            item_num.setTextAlignment(Qt.AlignCenter)
            self.objects_table.setItem(i, 0, item_num)
            
            # Score
            item_score = QTableWidgetItem(f"{obj.focus_score:.1f}")
            item_score.setTextAlignment(Qt.AlignCenter)
            if obj.is_focused:
                item_score.setForeground(Qt.green)
                n_focused += 1
            else:
                item_score.setForeground(Qt.red)
            self.objects_table.setItem(i, 1, item_score)
            
            # Estado
            status = "✓" if obj.is_focused else "✗"
            item_status = QTableWidgetItem(status)
            item_status.setTextAlignment(Qt.AlignCenter)
            self.objects_table.setItem(i, 2, item_status)
            
            # Área
            item_area = QTableWidgetItem(f"{int(obj.area)}")
            item_area.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.objects_table.setItem(i, 3, item_area)
        
        self.objects_table.resizeColumnsToContents()
        self.label_summary.setText(f"Total: {len(objects)} | Focused: {n_focused}")
        
        if n_focused > 0:
            self.label_summary.setStyleSheet("color: #2ECC71; font-weight: bold;")
        elif len(objects) > 0:
            self.label_summary.setStyleSheet("color: #E67E22; font-weight: bold;")
        else:
            self.label_summary.setStyleSheet("color: #95A5A6; font-weight: bold;")
    
    def _on_object_selected(self):
        """Resalta el objeto seleccionado en la vista."""
        # Por ahora solo refresca, podría hacer zoom al objeto
        pass
    
    def is_locked(self) -> bool:
        """Retorna si las capas están bloqueadas."""
        return self.cb_lock.isChecked()


class RobustImageViewer(QLabel):
    """Widget para mostrar imágenes con análisis robusto."""
    
    def __init__(self):
        super().__init__()
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(300, 300)
        self.setStyleSheet("background-color: #1e1e1e; border: 1px solid #3c3c3c;")
        self.setText("Calibre el modelo y seleccione imágenes")
        self.setFont(QFont("Segoe UI", 11))
        self.current_image = None
        self.current_result = None
        self._original_pixmap = None
        self._display_mode = "normal"  # "normal", "zscore", "mask", "debug"
    
    def set_display_mode(self, mode: str):
        """Cambia el modo de visualización."""
        self._display_mode = mode
        if self.current_image is not None and self.current_result is not None:
            self._refresh_display()
    
    def display_result(self, img_gray: np.ndarray, result: SharpnessResult):
        """Muestra imagen con resultado de análisis robusto."""
        self.current_image = img_gray
        self.current_result = result
        self._refresh_display()
    
    def _refresh_display(self):
        """Refresca la visualización según el modo actual."""
        if self.current_image is None:
            return
        
        img = self.current_image
        result = self.current_result
        
        if self._display_mode == "debug" and result:
            # Modo debug: usar laplacian_map si es imagen BGR de debug (Smart)
            if result.laplacian_map is not None and len(result.laplacian_map.shape) == 3:
                # Es la visualización de debug del SmartFocusScorer (BGR)
                img_display = result.laplacian_map.copy()
                self._draw_score_overlay(img_display, result)
            elif result.z_score_map is not None:
                # Modo legacy: visualización compuesta con Multi-Otsu info
                img_display = create_debug_composite(
                    img, result.z_score_map, result.mask, result.sharpness,
                    optimal_threshold=result.optimal_z_threshold,
                    bias=result.bias_correction,
                    hysteresis_low=result.hysteresis_low
                )
            else:
                # Fallback
                img_display = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)
                if result:
                    self._draw_score_overlay(img_display, result)
        elif self._display_mode == "zscore" and result and result.z_score_map is not None:
            # Solo Z-Score map / edges
            z_clipped = np.clip(result.z_score_map, 0, 255)
            z_norm = z_clipped.astype(np.uint8)
            img_display = cv2.applyColorMap(z_norm, cv2.COLORMAP_JET)
        elif self._display_mode == "mask" and result and result.mask is not None:
            # Solo máscara
            img_display = cv2.cvtColor(result.mask, cv2.COLOR_GRAY2BGR)
        else:
            # Normal: imagen con overlay de score
            if len(img.shape) == 2:
                img_display = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)
            else:
                img_display = img.copy()
            
            if result:
                self._draw_score_overlay(img_display, result)
        
        self._show_image(img_display)
    
    def _draw_score_overlay(self, img_display, result: SharpnessResult):
        """Dibuja overlay con información del resultado."""
        h, w = img_display.shape[:2]
        
        # Color según estado
        color_map = {
            "Muy Enfocada": (0, 255, 0),
            "Enfocada": (0, 200, 255),
            "Desenfocada": (0, 0, 255),
            "Sin Objeto": (128, 128, 128)
        }
        color = color_map.get(result.focus_state, (255, 255, 255))
        
        # Texto principal
        text = f"S={result.sharpness:.1f}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        # Fondo semi-transparente
        (tw, th), _ = cv2.getTextSize(text, font, 1.0, 2)
        overlay = img_display.copy()
        cv2.rectangle(overlay, (w-tw-20, 5), (w-5, th+15), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, img_display, 0.4, 0, img_display)
        
        # Texto
        cv2.putText(img_display, text, (w-tw-15, th+10), font, 1.0, color, 2)
        
        # Estado
        cv2.putText(img_display, result.focus_state, (10, h-10), font, 0.6, color, 1)
    
    def _show_image(self, img_display):
        """Convierte y muestra la imagen en el widget."""
        if len(img_display.shape) == 2:
            img_rgb = cv2.cvtColor(img_display, cv2.COLOR_GRAY2RGB)
        else:
            img_rgb = cv2.cvtColor(img_display, cv2.COLOR_BGR2RGB)
        
        h, w = img_rgb.shape[:2]
        ch = 3
        bytes_per_line = ch * w
        
        # Asegurar que los datos son contiguos
        img_rgb = np.ascontiguousarray(img_rgb)
        
        q_img = QImage(img_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self._original_pixmap = QPixmap.fromImage(q_img)
        
        self._update_scaled_pixmap()
    
    def _update_scaled_pixmap(self):
        """Actualiza el pixmap escalado al tamaño actual."""
        if self._original_pixmap and not self._original_pixmap.isNull():
            scaled_pixmap = self._original_pixmap.scaled(
                self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            self.setPixmap(scaled_pixmap)
    
    def resizeEvent(self, event):
        """Redibuja la imagen al redimensionar."""
        super().resizeEvent(event)
        self._update_scaled_pixmap()


class ImgAnalysisTab(QWidget):
    """
    Pestaña para análisis de nitidez robusto de imágenes.
    
    Basado en Control Robusto (Zhou/Doyle):
    - Modo Calibración: Entrena modelo de fondo (μ, σ)
    - Modo Análisis: Detecta objetos y mide sharpness
    
    Signals:
        analysis_completed: Emitido cuando se completa un análisis
        calibration_completed: Emitido cuando se completa calibración
    """
    
    # Señales para comunicación
    analysis_completed = pyqtSignal(str, float)  # filename, score
    calibration_completed = pyqtSignal(bool, str)  # success, message
    
    def __init__(self, parent=None):
        """
        Inicializa la pestaña de análisis de imagen.
        
        Args:
            parent: Widget padre (ArduinoGUI)
        """
        super().__init__(parent)
        self.parent_gui = parent
        
        # Estado
        self.image_files = []
        self.current_folder = ""
        self.background_folder = ""
        self.results_cache = {}  # {filename: SharpnessResult or FocusResult}
        
        # Método de análisis: "smart" (nuevo) o "legacy" (Z-Score)
        self.analysis_method = "smart"  # Default: nuevo método sin calibración
        
        # Detector de sharpness robusto (legacy)
        self.sharpness_detector = SharpnessDetector()
        
        # Smart Focus Scorer (U2-Net Salient Object Detection)
        self.smart_scorer = SmartFocusScorer(
            model_type='u2netp',   # Modelo pequeño/rápido
            threshold=0.5,         # Umbral de binarización de saliencia
            min_area=500,          # Área mínima para considerar objeto (filtra ruido)
            min_prob=0.3,          # Probabilidad mínima para considerar objeto real
            focus_threshold=20.0,  # Threshold para clasificar como enfocado
            use_laplacian=True
        )
        
        self.calibration_worker = None
        
        self._setup_ui()
        self._try_load_existing_model()
        logger.debug("ImgAnalysisTab (Robusto) inicializado")
    
    def _try_load_existing_model(self):
        """Intenta cargar modelo existente al iniciar."""
        success, msg = self.sharpness_detector.load_model()
        if success:
            self.label_model_status.setText(f"✅ {msg}")
            self.label_model_status.setStyleSheet("color: #2ECC71;")
            self.btn_load_test.setEnabled(True)
        else:
            self.label_model_status.setText("⚠️ Sin modelo - Calibre primero")
            self.label_model_status.setStyleSheet("color: #E67E22;")
    
    def _setup_ui(self):
        """Configura la interfaz de usuario con scroll vertical."""
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        # Scroll Area
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll_area.setStyleSheet("""
            QScrollArea { border: none; background-color: transparent; }
            QScrollBar:vertical {
                background-color: #2E2E2E; width: 12px; border-radius: 6px;
            }
            QScrollBar::handle:vertical {
                background-color: #505050; border-radius: 6px; min-height: 30px;
            }
            QScrollBar::handle:vertical:hover { background-color: #606060; }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }
        """)
        
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setSpacing(10)
        
        # === Sección 0: Selector de Método ===
        method_group = QGroupBox("🔬 Método de Análisis")
        method_layout = QHBoxLayout()
        
        method_layout.addWidget(QLabel("Método:"))
        self.combo_method = QComboBox()
        self.combo_method.addItem("🚀 SMART (Entropía + Brenner) - Sin calibración", "smart")
        self.combo_method.addItem("📊 LEGACY (Z-Score + Laplaciano) - Requiere calibración", "legacy")
        self.combo_method.setCurrentIndex(0)  # Smart por defecto
        self.combo_method.currentIndexChanged.connect(self._on_method_changed)
        self.combo_method.setStyleSheet("""
            QComboBox { 
                padding: 5px; font-size: 12px; 
                background-color: #2E2E2E; color: #F0F0F0;
                border: 1px solid #505050; border-radius: 4px;
            }
            QComboBox:hover { border-color: #2E86C1; }
            QComboBox::drop-down { border: none; }
        """)
        method_layout.addWidget(self.combo_method, 1)
        
        self.label_method_info = QLabel("✅ Listo para analizar (sin calibración)")
        self.label_method_info.setStyleSheet("color: #2ECC71; font-weight: bold;")
        method_layout.addWidget(self.label_method_info)
        
        method_group.setLayout(method_layout)
        scroll_layout.addWidget(method_group)
        
        # === Sección SMART: Parámetros U2-Net ===
        self.smart_params_group = QGroupBox("🧠 Parámetros SMART (U2-Net AI)")
        smart_main_layout = QVBoxLayout()
        
        # --- Fila 1: Parámetros principales ---
        params_row = QHBoxLayout()
        
        params_row.addWidget(QLabel("Saliencia:"))
        self.spin_saliency_thresh = QDoubleSpinBox()
        self.spin_saliency_thresh.setRange(0.1, 0.9)
        self.spin_saliency_thresh.setValue(0.5)
        self.spin_saliency_thresh.setSingleStep(0.05)
        self.spin_saliency_thresh.setToolTip("Umbral para binarizar la máscara.\nMenor = detecta más, Mayor = más estricto.")
        self.spin_saliency_thresh.valueChanged.connect(self._on_smart_params_changed)
        params_row.addWidget(self.spin_saliency_thresh)
        
        params_row.addWidget(QLabel("Prob Mín:"))
        self.spin_min_prob = QDoubleSpinBox()
        self.spin_min_prob.setRange(0.1, 0.9)
        self.spin_min_prob.setValue(0.3)
        self.spin_min_prob.setSingleStep(0.05)
        self.spin_min_prob.setToolTip("Probabilidad mínima promedio para considerar objeto REAL.\nFiltra ruido de fondo que tiene baja probabilidad.")
        self.spin_min_prob.valueChanged.connect(self._on_smart_params_changed)
        params_row.addWidget(self.spin_min_prob)
        
        params_row.addWidget(QLabel("Área Mín:"))
        self.spin_min_area = QSpinBox()
        self.spin_min_area.setRange(100, 50000)
        self.spin_min_area.setValue(500)
        self.spin_min_area.setSingleStep(100)
        self.spin_min_area.setToolTip("Área mínima en px² para considerar objeto válido.\nFiltra objetos pequeños (ruido).")
        self.spin_min_area.valueChanged.connect(self._on_smart_params_changed)
        params_row.addWidget(self.spin_min_area)
        
        params_row.addWidget(QLabel("Focus:"))
        self.spin_focus_thresh = QDoubleSpinBox()
        self.spin_focus_thresh.setRange(1.0, 500.0)
        self.spin_focus_thresh.setValue(20.0)
        self.spin_focus_thresh.setSingleStep(5.0)
        self.spin_focus_thresh.setToolTip("Score mínimo para clasificar como ENFOCADO.")
        self.spin_focus_thresh.valueChanged.connect(self._on_smart_params_changed)
        params_row.addWidget(self.spin_focus_thresh)
        
        self.cb_use_laplacian = QCheckBox("Lap")
        self.cb_use_laplacian.setChecked(True)
        self.cb_use_laplacian.setToolTip("Laplaciano vs Brenner")
        self.cb_use_laplacian.stateChanged.connect(self._on_smart_params_changed)
        params_row.addWidget(self.cb_use_laplacian)
        
        smart_main_layout.addLayout(params_row)
        
        # --- Fila 2: Estado y Reset ---
        status_row = QHBoxLayout()
        
        # Estado del modelo
        status_row.addWidget(QLabel("Modelo:"))
        self.label_model_status_smart = QLabel("⏳ No cargado")
        self.label_model_status_smart.setStyleSheet("color: #E67E22; font-weight: bold;")
        status_row.addWidget(self.label_model_status_smart)
        
        status_row.addStretch()
        
        self.btn_reset_smart = QPushButton("🔄 Reset")
        self.btn_reset_smart.setStyleSheet(self._button_style("#E67E22"))
        self.btn_reset_smart.setFixedWidth(70)
        self.btn_reset_smart.clicked.connect(self._reset_smart_parameters)
        status_row.addWidget(self.btn_reset_smart)
        
        smart_main_layout.addLayout(status_row)
        
        # --- Fila 3: Estadísticas en tiempo real ---
        stats_row = QHBoxLayout()
        
        stats_row.addWidget(QLabel("📊"))
        self.label_smart_prob_stats = QLabel("Prob: --")
        self.label_smart_prob_stats.setStyleSheet("color: #3498DB;")
        self.label_smart_prob_stats.setToolTip("Probabilidad media/max en el objeto")
        stats_row.addWidget(self.label_smart_prob_stats)
        
        self.label_smart_area_stats = QLabel("Área: --")
        self.label_smart_area_stats.setStyleSheet("color: #9B59B6;")
        self.label_smart_area_stats.setToolTip("Área del objeto detectado")
        stats_row.addWidget(self.label_smart_area_stats)
        
        self.label_smart_focus_stats = QLabel("Focus: --")
        self.label_smart_focus_stats.setStyleSheet("color: #E67E22;")
        self.label_smart_focus_stats.setToolTip("Score de enfoque (raw / normalizado)")
        stats_row.addWidget(self.label_smart_focus_stats)
        
        self.label_smart_centroid = QLabel("Centro: --")
        self.label_smart_centroid.setStyleSheet("color: #1ABC9C;")
        self.label_smart_centroid.setToolTip("Centroide del objeto")
        stats_row.addWidget(self.label_smart_centroid)
        
        stats_row.addStretch()
        
        smart_main_layout.addLayout(stats_row)
        
        self.smart_params_group.setLayout(smart_main_layout)
        scroll_layout.addWidget(self.smart_params_group)
        
        # Variable para modo de visualización SMART
        self._smart_view_mode = "overlay"
        
        # === Sección 1: Calibración (Solo para método Legacy) ===
        self.calib_group = QGroupBox("🎯 Calibración - Modelo de Fondo (Solo para método Legacy)")
        calib_layout = QVBoxLayout()
        
        # Fila 1: Selección de carpeta de fondo
        calib_row1 = QHBoxLayout()
        self.btn_select_bg = QPushButton("📁 Carpeta Fondo")
        self.btn_select_bg.setStyleSheet(self._button_style("#8E44AD"))
        self.btn_select_bg.clicked.connect(self._select_background_folder)
        calib_row1.addWidget(self.btn_select_bg)
        
        self.label_bg_folder = QLabel("No seleccionada")
        self.label_bg_folder.setStyleSheet("color: #888888;")
        calib_row1.addWidget(self.label_bg_folder, 1)
        
        self.btn_calibrate = QPushButton("⚙️ Calibrar")
        self.btn_calibrate.setStyleSheet(self._button_style("#E74C3C"))
        self.btn_calibrate.clicked.connect(self._start_calibration)
        self.btn_calibrate.setEnabled(False)
        calib_row1.addWidget(self.btn_calibrate)
        
        calib_layout.addLayout(calib_row1)
        
        # Fila 2: Progreso y estado
        calib_row2 = QHBoxLayout()
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        self.progress_bar.setStyleSheet("""
            QProgressBar { border: 1px solid #505050; border-radius: 3px; text-align: center; }
            QProgressBar::chunk { background-color: #2E86C1; }
        """)
        calib_row2.addWidget(self.progress_bar)
        
        self.label_model_status = QLabel("⚠️ Sin modelo")
        self.label_model_status.setStyleSheet("color: #E67E22; font-weight: bold;")
        calib_row2.addWidget(self.label_model_status)
        
        calib_layout.addLayout(calib_row2)
        self.calib_group.setLayout(calib_layout)
        self.calib_group.setVisible(False)  # Oculto por defecto (método Smart)
        scroll_layout.addWidget(self.calib_group)
        
        # === Sección 2: Panel de Parámetros Avanzados (Solo Legacy) ===
        self.params_group = QGroupBox("🔧 Parámetros del Detector Legacy (Z-Score)")
        self.params_group.setVisible(False)  # Oculto por defecto (método Smart)
        params_main_layout = QVBoxLayout()
        
        # --- Fila 1: Detección de Objeto (Z-Score) ---
        detection_frame = QGroupBox("Detección de Objeto (Blanqueo Z-Score)")
        detection_layout = QGridLayout()
        
        # Modo Automático (Otsu + Histéresis)
        self.cb_auto_threshold = QCheckBox("🤖 Umbral Automático (Otsu + Histéresis)")
        self.cb_auto_threshold.setChecked(True)
        self.cb_auto_threshold.setToolTip(
            "RECOMENDADO: Calcula umbral óptimo automáticamente.\n"
            "Basado en Zhou & Doyle (Control Robusto):\n"
            "• Otsu: Minimiza varianza intra-clase (H₂ óptimo)\n"
            "• Histéresis: Reconstruye objeto por conectividad espacial\n"
            "• Bias Correction: Rechaza perturbaciones de iluminación"
        )
        self.cb_auto_threshold.stateChanged.connect(self._on_auto_threshold_changed)
        detection_layout.addWidget(self.cb_auto_threshold, 0, 0, 1, 3)
        
        # Z-Threshold con slider (deshabilitado en modo auto)
        self.label_z_threshold = QLabel("Umbral Z (manual):")
        detection_layout.addWidget(self.label_z_threshold, 1, 0)
        self.slider_z_threshold = QSlider(Qt.Horizontal)
        self.slider_z_threshold.setRange(10, 100)  # 1.0 - 10.0
        self.slider_z_threshold.setValue(30)  # 3.0
        self.slider_z_threshold.setTickPosition(QSlider.TicksBelow)
        self.slider_z_threshold.setTickInterval(10)
        self.slider_z_threshold.setEnabled(False)  # Deshabilitado por defecto
        self.slider_z_threshold.valueChanged.connect(self._on_z_slider_changed)
        detection_layout.addWidget(self.slider_z_threshold, 1, 1)
        
        self.spin_z_threshold = QDoubleSpinBox()
        self.spin_z_threshold.setRange(0.5, 10.0)
        self.spin_z_threshold.setValue(3.0)
        self.spin_z_threshold.setSingleStep(0.1)
        self.spin_z_threshold.setDecimals(1)
        self.spin_z_threshold.setEnabled(False)  # Deshabilitado por defecto
        self.spin_z_threshold.setToolTip("Umbral Z-Score manual (solo si modo automático está desactivado)")
        self.spin_z_threshold.valueChanged.connect(self._on_z_spin_changed)
        detection_layout.addWidget(self.spin_z_threshold, 1, 2)
        
        # Min Object Ratio
        detection_layout.addWidget(QLabel("% Mín Objeto:"), 2, 0)
        self.slider_min_object = QSlider(Qt.Horizontal)
        self.slider_min_object.setRange(1, 100)  # 0.01% - 1.0%
        self.slider_min_object.setValue(10)  # 0.1%
        self.slider_min_object.valueChanged.connect(self._on_min_object_slider_changed)
        detection_layout.addWidget(self.slider_min_object, 2, 1)
        
        self.spin_min_object = QDoubleSpinBox()
        self.spin_min_object.setRange(0.001, 5.0)
        self.spin_min_object.setValue(0.1)
        self.spin_min_object.setSingleStep(0.01)
        self.spin_min_object.setDecimals(3)
        self.spin_min_object.setSuffix(" %")
        self.spin_min_object.setToolTip("Porcentaje mínimo de píxeles para considerar que hay objeto.\nMuy bajo = detecta ruido, Muy alto = ignora objetos pequeños")
        self.spin_min_object.valueChanged.connect(self._on_min_object_spin_changed)
        detection_layout.addWidget(self.spin_min_object, 2, 2)
        
        detection_frame.setLayout(detection_layout)
        params_main_layout.addWidget(detection_frame)
        
        # --- Fila 2: Clasificación de Sharpness ---
        sharpness_frame = QGroupBox("Clasificación de Nitidez (Sharpness)")
        sharpness_layout = QGridLayout()
        
        # Umbral Enfocada
        sharpness_layout.addWidget(QLabel("S Enfocada:"), 0, 0)
        self.slider_s_threshold = QSlider(Qt.Horizontal)
        self.slider_s_threshold.setRange(10, 500)
        self.slider_s_threshold.setValue(50)
        self.slider_s_threshold.setTickPosition(QSlider.TicksBelow)
        self.slider_s_threshold.setTickInterval(50)
        self.slider_s_threshold.valueChanged.connect(self._on_s_slider_changed)
        sharpness_layout.addWidget(self.slider_s_threshold, 0, 1)
        
        self.spin_s_threshold = QDoubleSpinBox()
        self.spin_s_threshold.setRange(1.0, 1000.0)
        self.spin_s_threshold.setValue(50.0)
        self.spin_s_threshold.setSingleStep(5.0)
        self.spin_s_threshold.setDecimals(1)
        self.spin_s_threshold.setToolTip("Umbral S para clasificar como 'Enfocada'.\nS > umbral×10 = Muy Enfocada\nS > umbral = Enfocada\nS < umbral = Desenfocada")
        self.spin_s_threshold.valueChanged.connect(self._on_s_spin_changed)
        sharpness_layout.addWidget(self.spin_s_threshold, 0, 2)
        
        # Multiplicador para "Muy Enfocada"
        sharpness_layout.addWidget(QLabel("Factor Muy Enf:"), 1, 0)
        self.spin_very_focused_mult = QDoubleSpinBox()
        self.spin_very_focused_mult.setRange(2.0, 20.0)
        self.spin_very_focused_mult.setValue(10.0)
        self.spin_very_focused_mult.setSingleStep(1.0)
        self.spin_very_focused_mult.setDecimals(1)
        self.spin_very_focused_mult.setSuffix(" ×")
        self.spin_very_focused_mult.setToolTip("Multiplicador del umbral para 'Muy Enfocada'.\nS > umbral × factor = Muy Enfocada")
        self.spin_very_focused_mult.valueChanged.connect(self._on_params_changed)
        sharpness_layout.addWidget(self.spin_very_focused_mult, 1, 1, 1, 2)
        
        sharpness_frame.setLayout(sharpness_layout)
        params_main_layout.addWidget(sharpness_frame)
        
        # --- Fila 3: Procesamiento Morfológico ---
        morph_frame = QGroupBox("Procesamiento Morfológico (Limpieza de Máscara)")
        morph_layout = QGridLayout()
        
        # Habilitar morfología
        self.cb_enable_morph = QCheckBox("Habilitar")
        self.cb_enable_morph.setChecked(True)
        self.cb_enable_morph.setToolTip("Activa operaciones morfológicas para limpiar la máscara")
        self.cb_enable_morph.stateChanged.connect(self._on_params_changed)
        morph_layout.addWidget(self.cb_enable_morph, 0, 0)
        
        # Kernel size
        morph_layout.addWidget(QLabel("Kernel:"), 0, 1)
        self.slider_morph = QSlider(Qt.Horizontal)
        self.slider_morph.setRange(1, 21)
        self.slider_morph.setValue(5)
        self.slider_morph.setSingleStep(2)
        self.slider_morph.valueChanged.connect(self._on_morph_slider_changed)
        morph_layout.addWidget(self.slider_morph, 0, 2)
        
        self.spin_morph = QSpinBox()
        self.spin_morph.setRange(1, 31)
        self.spin_morph.setValue(5)
        self.spin_morph.setSingleStep(2)
        self.spin_morph.setSuffix(" px")
        self.spin_morph.setToolTip("Tamaño del kernel morfológico.\nMayor = elimina más ruido pero puede perder detalles")
        self.spin_morph.valueChanged.connect(self._on_morph_spin_changed)
        morph_layout.addWidget(self.spin_morph, 0, 3)
        
        # Operaciones morfológicas
        morph_layout.addWidget(QLabel("Opening:"), 1, 0)
        self.cb_morph_open = QCheckBox()
        self.cb_morph_open.setChecked(True)
        self.cb_morph_open.setToolTip("Opening: Elimina ruido pequeño (erosión + dilatación)")
        self.cb_morph_open.stateChanged.connect(self._on_params_changed)
        morph_layout.addWidget(self.cb_morph_open, 1, 1)
        
        morph_layout.addWidget(QLabel("Closing:"), 1, 2)
        self.cb_morph_close = QCheckBox()
        self.cb_morph_close.setChecked(True)
        self.cb_morph_close.setToolTip("Closing: Rellena huecos pequeños (dilatación + erosión)")
        self.cb_morph_close.stateChanged.connect(self._on_params_changed)
        morph_layout.addWidget(self.cb_morph_close, 1, 3)
        
        morph_frame.setLayout(morph_layout)
        params_main_layout.addWidget(morph_frame)
        
        # --- Fila 4: ROI Dilatada para Sharpness (Captura de Bordes) ---
        roi_frame = QGroupBox("Cálculo de Sharpness (ROI Dilatada)")
        roi_layout = QGridLayout()
        
        # Habilitar dilatación
        self.cb_dilate_roi = QCheckBox("Dilatar ROI")
        self.cb_dilate_roi.setChecked(True)
        self.cb_dilate_roi.setToolTip("Expande la máscara para incluir los bordes del objeto.\nLos bordes contienen la información de enfoque (alta frecuencia).")
        self.cb_dilate_roi.stateChanged.connect(self._on_params_changed)
        roi_layout.addWidget(self.cb_dilate_roi, 0, 0)
        
        # Kernel de dilatación
        roi_layout.addWidget(QLabel("Dilatación:"), 0, 1)
        self.slider_dilation = QSlider(Qt.Horizontal)
        self.slider_dilation.setRange(1, 51)
        self.slider_dilation.setValue(15)
        self.slider_dilation.setSingleStep(2)
        self.slider_dilation.valueChanged.connect(self._on_dilation_slider_changed)
        roi_layout.addWidget(self.slider_dilation, 0, 2)
        
        self.spin_dilation = QSpinBox()
        self.spin_dilation.setRange(1, 51)
        self.spin_dilation.setValue(15)
        self.spin_dilation.setSingleStep(2)
        self.spin_dilation.setSuffix(" px")
        self.spin_dilation.setToolTip("Tamaño del kernel de dilatación.\nMayor = captura más borde (halo) del objeto")
        self.spin_dilation.valueChanged.connect(self._on_dilation_spin_changed)
        roi_layout.addWidget(self.spin_dilation, 0, 3)
        
        # Método de cálculo
        roi_layout.addWidget(QLabel("Método S:"), 1, 0)
        self.cb_use_mse = QCheckBox("MSE (√mean(L²))")
        self.cb_use_mse.setChecked(True)
        self.cb_use_mse.setToolTip("Mean Square Energy: Penaliza bordes suaves, premia nítidos.\nDesactivado: usa mean(|L|) tradicional.")
        self.cb_use_mse.stateChanged.connect(self._on_params_changed)
        roi_layout.addWidget(self.cb_use_mse, 1, 1, 1, 3)
        
        roi_frame.setLayout(roi_layout)
        params_main_layout.addWidget(roi_frame)
        
        # --- Fila 5: Visualización y Acciones ---
        viz_frame = QHBoxLayout()
        
        # Modo de visualización
        viz_frame.addWidget(QLabel("Vista:"))
        self.cb_view_normal = QCheckBox("Normal")
        self.cb_view_normal.setChecked(True)
        self.cb_view_normal.stateChanged.connect(lambda: self._set_view_mode("normal"))
        viz_frame.addWidget(self.cb_view_normal)
        
        self.cb_view_zscore = QCheckBox("Z-Score")
        self.cb_view_zscore.stateChanged.connect(lambda: self._set_view_mode("zscore"))
        viz_frame.addWidget(self.cb_view_zscore)
        
        self.cb_view_mask = QCheckBox("Máscara")
        self.cb_view_mask.stateChanged.connect(lambda: self._set_view_mode("mask"))
        viz_frame.addWidget(self.cb_view_mask)
        
        self.cb_debug = QCheckBox("Debug (3 paneles)")
        self.cb_debug.setToolTip("Muestra: Original | Z-Score | Máscara")
        self.cb_debug.stateChanged.connect(lambda: self._set_view_mode("debug"))
        viz_frame.addWidget(self.cb_debug)
        
        viz_frame.addStretch()
        
        # Botón reset
        self.btn_reset_params = QPushButton("🔄 Reset")
        self.btn_reset_params.setStyleSheet(self._button_style("#E67E22"))
        self.btn_reset_params.setFixedWidth(80)
        self.btn_reset_params.clicked.connect(self._reset_parameters)
        viz_frame.addWidget(self.btn_reset_params)
        
        params_main_layout.addLayout(viz_frame)
        
        self.params_group.setLayout(params_main_layout)
        scroll_layout.addWidget(self.params_group)
        
        # === Sección 3: Análisis de Imágenes ===
        analysis_group = QGroupBox("📊 Análisis de Imágenes (Medición Robusta)")
        analysis_layout = QVBoxLayout()
        
        # Barra de herramientas
        toolbar = QHBoxLayout()
        
        self.btn_load_test = QPushButton("📁 Cargar Imágenes")
        self.btn_load_test.setStyleSheet(self._button_style("#0078d4"))
        self.btn_load_test.clicked.connect(self._load_test_images)
        self.btn_load_test.setEnabled(False)
        toolbar.addWidget(self.btn_load_test)
        
        self.btn_export = QPushButton("📊 Exportar CSV")
        self.btn_export.setStyleSheet(self._button_style("#27AE60"))
        self.btn_export.clicked.connect(self._export_results)
        toolbar.addWidget(self.btn_export)
        
        toolbar.addStretch()
        
        self.label_folder = QLabel("Sin carpeta")
        self.label_folder.setStyleSheet("color: #888888;")
        toolbar.addWidget(self.label_folder)
        
        analysis_layout.addLayout(toolbar)
        
        # Contador
        self.label_count = QLabel("Imágenes: 0")
        self.label_count.setStyleSheet("font-weight: bold; color: #F0F0F0;")
        analysis_layout.addWidget(self.label_count)
        
        # Splitter: Lista | Visor
        splitter = QSplitter(Qt.Horizontal)
        splitter.setStyleSheet("""
            QSplitter::handle { background-color: #505050; width: 3px; }
            QSplitter::handle:hover { background-color: #2E86C1; }
        """)
        
        # Lista de imágenes
        self.list_images = QListWidget()
        self.list_images.setStyleSheet("""
            QListWidget { background-color: #1e1e1e; color: #fff; border: 1px solid #3c3c3c; font-size: 11px; }
            QListWidget::item { padding: 6px; border-bottom: 1px solid #3c3c3c; }
            QListWidget::item:selected { background-color: #0078d4; }
            QListWidget::item:hover { background-color: #3c3c3c; }
        """)
        self.list_images.currentItemChanged.connect(self._on_image_selected)
        self.list_images.setMinimumWidth(200)
        splitter.addWidget(self.list_images)
        
        # Container para visores (single o grid)
        viewer_container = QWidget()
        viewer_layout = QVBoxLayout(viewer_container)
        viewer_layout.setContentsMargins(0, 0, 0, 0)
        viewer_layout.setSpacing(2)
        
        # Toggle para cambiar entre Legacy y SMART view
        view_toggle = QHBoxLayout()
        self.rb_legacy_view = QRadioButton("Legacy")
        self.rb_smart_view = QRadioButton("SMART")
        self.rb_smart_view.setChecked(True)  # Default: SMART
        self.rb_legacy_view.toggled.connect(self._toggle_view_mode)
        view_toggle.addWidget(self.rb_legacy_view)
        view_toggle.addWidget(self.rb_smart_view)
        view_toggle.addStretch()
        viewer_layout.addLayout(view_toggle)
        
        # Stack para alternar entre visores
        self.viewer_stack = QStackedWidget()
        
        # Visor único legacy (para método Legacy)
        self.image_viewer = RobustImageViewer()
        self.image_viewer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.viewer_stack.addWidget(self.image_viewer)
        
        # Vista única con overlays (para método SMART)
        self.single_view = SingleViewWithOverlays()
        self.viewer_stack.addWidget(self.single_view)
        
        # Default: mostrar vista SMART
        self.viewer_stack.setCurrentIndex(1)
        
        viewer_layout.addWidget(self.viewer_stack)
        splitter.addWidget(viewer_container)
        
        splitter.setSizes([200, 600])
        splitter.setMinimumHeight(350)
        analysis_layout.addWidget(splitter)
        
        analysis_group.setLayout(analysis_layout)
        scroll_layout.addWidget(analysis_group)
        
        # === Sección 4: Estadísticas Detalladas ===
        stats_group = QGroupBox("📈 Resultado Actual")
        stats_layout = QGridLayout()
        
        stats_layout.addWidget(QLabel("Archivo:"), 0, 0)
        self.label_current_file = QLabel("-")
        self.label_current_file.setStyleSheet("color: #2E86C1; font-weight: bold;")
        stats_layout.addWidget(self.label_current_file, 0, 1)
        
        stats_layout.addWidget(QLabel("Sharpness (S):"), 0, 2)
        self.label_sharpness = QLabel("-")
        self.label_sharpness.setStyleSheet("font-weight: bold; font-size: 14px;")
        stats_layout.addWidget(self.label_sharpness, 0, 3)
        
        stats_layout.addWidget(QLabel("Estado:"), 1, 0)
        self.label_focus_state = QLabel("-")
        stats_layout.addWidget(self.label_focus_state, 1, 1)
        
        stats_layout.addWidget(QLabel("Objeto:"), 1, 2)
        self.label_object_info = QLabel("-")
        self.label_object_info.setStyleSheet("color: #888888;")
        stats_layout.addWidget(self.label_object_info, 1, 3)
        
        stats_layout.addWidget(QLabel("Z-Score máx:"), 2, 0)
        self.label_zscore = QLabel("-")
        self.label_zscore.setStyleSheet("color: #888888;")
        stats_layout.addWidget(self.label_zscore, 2, 1)
        
        stats_layout.addWidget(QLabel("Umbrales (hi/lo):"), 2, 2)
        self.label_thresholds = QLabel("-")
        self.label_thresholds.setStyleSheet("color: #888888;")
        stats_layout.addWidget(self.label_thresholds, 2, 3)
        
        stats_layout.addWidget(QLabel("Bias:"), 3, 0)
        self.label_bias = QLabel("-")
        self.label_bias.setStyleSheet("color: #888888;")
        stats_layout.addWidget(self.label_bias, 3, 1)
        
        stats_layout.addWidget(QLabel("Status:"), 3, 2)
        self.label_status = QLabel("Listo")
        self.label_status.setStyleSheet("color: #E67E22;")
        stats_layout.addWidget(self.label_status, 3, 3)
        
        stats_group.setLayout(stats_layout)
        scroll_layout.addWidget(stats_group)
        
        # Espacio para futuras características
        scroll_layout.addStretch()
        
        scroll_area.setWidget(scroll_content)
        main_layout.addWidget(scroll_area)
    
    def _button_style(self, color):
        """Genera estilo para botones."""
        return f"""
            QPushButton {{
                background-color: {color}; color: white; border: none;
                padding: 8px 16px; font-size: 12px; font-weight: bold; border-radius: 4px;
            }}
            QPushButton:hover {{ background-color: {color}dd; }}
            QPushButton:pressed {{ background-color: {color}aa; }}
            QPushButton:disabled {{ background-color: #505050; color: #808080; }}
        """
    
    # === Calibración ===
    
    def _select_background_folder(self):
        """Selecciona carpeta con imágenes de fondo."""
        folder = QFileDialog.getExistingDirectory(
            self, "Seleccionar Carpeta de Fondo (100-300 imágenes vacías)", ""
        )
        if folder:
            self.background_folder = folder
            n_images = len(list_images_in_folder(folder))
            self.label_bg_folder.setText(f"{self._truncate_path(folder, 30)} ({n_images} imgs)")
            self.label_bg_folder.setToolTip(folder)
            self.btn_calibrate.setEnabled(n_images >= 10)
            
            if n_images < 10:
                self.label_status.setText(f"Se requieren al menos 10 imágenes de fondo")
                self.label_status.setStyleSheet("color: #E74C3C;")
    
    def _start_calibration(self):
        """Inicia calibración en thread separado."""
        if not self.background_folder:
            return
        
        self.btn_calibrate.setEnabled(False)
        self.btn_select_bg.setEnabled(False)
        self.progress_bar.setVisible(True)
        self.progress_bar.setValue(0)
        self.label_status.setText("Calibrando...")
        self.label_status.setStyleSheet("color: #3498DB;")
        
        self.calibration_worker = CalibrationWorker(self.background_folder)
        self.calibration_worker.progress.connect(self._on_calibration_progress)
        self.calibration_worker.finished.connect(self._on_calibration_finished)
        self.calibration_worker.start()
    
    @pyqtSlot(int, int, str)
    def _on_calibration_progress(self, current, total, message):
        """Actualiza progreso de calibración."""
        self.progress_bar.setMaximum(total)
        self.progress_bar.setValue(current)
    
    @pyqtSlot(bool, str)
    def _on_calibration_finished(self, success, message):
        """Maneja fin de calibración."""
        self.progress_bar.setVisible(False)
        self.btn_calibrate.setEnabled(True)
        self.btn_select_bg.setEnabled(True)
        
        if success:
            self.label_model_status.setText(f"✅ {message}")
            self.label_model_status.setStyleSheet("color: #2ECC71;")
            self.label_status.setText("Calibración completada")
            self.label_status.setStyleSheet("color: #2ECC71;")
            
            # Recargar modelo
            self.sharpness_detector.load_model()
            self.btn_load_test.setEnabled(True)
            
            self.calibration_completed.emit(True, message)
        else:
            self.label_model_status.setText(f"❌ Error")
            self.label_model_status.setStyleSheet("color: #E74C3C;")
            self.label_status.setText(message)
            self.label_status.setStyleSheet("color: #E74C3C;")
            
            self.calibration_completed.emit(False, message)
    
    # === Visor Grid/Single ===
    
    def _toggle_view_mode(self, single_checked: bool):
        """Alterna entre vista única y grid."""
        if single_checked:
            self.viewer_stack.setCurrentIndex(0)  # Single view
        else:
            self.viewer_stack.setCurrentIndex(1)  # SMART view
            # Refrescar vista si hay resultado
            if hasattr(self, '_last_focus_result') and self._last_focus_result:
                self.single_view.display_result(
                    self.image_viewer.current_image, 
                    self._last_focus_result
                )
    
    # === Método de Análisis ===
    
    def _on_method_changed(self, index):
        """Cambia entre método Smart y Legacy."""
        self.analysis_method = self.combo_method.currentData()
        
        if self.analysis_method == "smart":
            # Método Smart: Sin calibración necesaria
            self.smart_params_group.setVisible(True)
            self.calib_group.setVisible(False)
            self.params_group.setVisible(False)
            self.label_method_info.setText("✅ Listo para analizar (sin calibración)")
            self.label_method_info.setStyleSheet("color: #2ECC71; font-weight: bold;")
            self.btn_load_test.setEnabled(True)
            logger.info("[ImgAnalysisTab] Método cambiado a SMART (ROI-Based)")
        else:
            # Método Legacy: Requiere calibración
            self.smart_params_group.setVisible(False)
            self.calib_group.setVisible(True)
            self.params_group.setVisible(True)
            if self.sharpness_detector.is_ready:
                self.label_method_info.setText("✅ Modelo cargado")
                self.label_method_info.setStyleSheet("color: #2ECC71; font-weight: bold;")
                self.btn_load_test.setEnabled(True)
            else:
                self.label_method_info.setText("⚠️ Requiere calibración")
                self.label_method_info.setStyleSheet("color: #E67E22; font-weight: bold;")
                self.btn_load_test.setEnabled(False)
            logger.info("[ImgAnalysisTab] Método cambiado a LEGACY (Z-Score)")
        
        # Limpiar cache y re-analizar si hay imagen seleccionada
        self.results_cache.clear()
        current = self.list_images.currentItem()
        if current:
            self._analyze_current_image(current)
    
    def _on_smart_params_changed(self):
        """Actualiza parámetros del SmartFocusScorer cuando cambian en la UI."""
        self.smart_scorer.set_parameters(
            threshold=self.spin_saliency_thresh.value(),
            min_prob=self.spin_min_prob.value(),
            min_area=self.spin_min_area.value(),
            focus_threshold=self.spin_focus_thresh.value(),
            use_laplacian=self.cb_use_laplacian.isChecked()
        )
        
        # Actualizar estado del modelo
        if self.smart_scorer.is_model_loaded():
            self.label_model_status_smart.setText("✅ U2-Net")
            self.label_model_status_smart.setStyleSheet("color: #2ECC71; font-weight: bold;")
        else:
            error = self.smart_scorer.get_load_error()
            if error:
                self.label_model_status_smart.setText(f"❌ {error[:20]}...")
                self.label_model_status_smart.setStyleSheet("color: #E74C3C;")
            else:
                self.label_model_status_smart.setText("⏳ Cargando...")
                self.label_model_status_smart.setStyleSheet("color: #E67E22;")
        
        # Re-analizar imagen actual si hay una
        self.results_cache.clear()
        current = self.list_images.currentItem()
        if current and self.analysis_method == "smart":
            self._analyze_current_image(current)
    
    def _update_smart_stats(self, focus_result: FocusResult):
        """Actualiza las estadísticas del panel SMART."""
        num_objects = focus_result.num_objects
        objects = focus_result.objects
        
        # Contar enfocados/desenfocados
        focused_count = sum(1 for o in objects if o.is_focused)
        blurry_count = num_objects - focused_count
        
        # Probabilidad del objeto principal
        if focus_result.mean_probability > 0:
            self.label_smart_prob_stats.setText(f"Prob: {focus_result.mean_probability:.2f}")
            if focus_result.mean_probability >= self.smart_scorer.min_prob:
                self.label_smart_prob_stats.setStyleSheet("color: #2ECC71;")
            else:
                self.label_smart_prob_stats.setStyleSheet("color: #E74C3C;")
        else:
            self.label_smart_prob_stats.setText("Prob: --")
            self.label_smart_prob_stats.setStyleSheet("color: #95A5A6;")
        
        # Objetos: total (enfocados/desenfocados)
        if num_objects > 0:
            obj_str = f"Obj: {num_objects} ({focused_count}✓/{blurry_count}✗)"
            self.label_smart_area_stats.setText(obj_str)
            if focused_count > 0:
                self.label_smart_area_stats.setStyleSheet("color: #2ECC71;")
            else:
                self.label_smart_area_stats.setStyleSheet("color: #E67E22;")
        else:
            self.label_smart_area_stats.setText("Obj: 0")
            self.label_smart_area_stats.setStyleSheet("color: #95A5A6;")
        
        # Focus score del principal
        if focus_result.focus_score > 0:
            self.label_smart_focus_stats.setText(f"Main: {focus_result.focus_score:.1f}")
            if focus_result.status in ["FOCUSED_OBJECT", "MULTIPLE_OBJECTS"] and focused_count > 0:
                self.label_smart_focus_stats.setStyleSheet("color: #2ECC71; font-weight: bold;")
            else:
                self.label_smart_focus_stats.setStyleSheet("color: #E67E22;")
        else:
            self.label_smart_focus_stats.setText("Main: 0")
            self.label_smart_focus_stats.setStyleSheet("color: #95A5A6;")
        
        # Área total
        total_area = sum(o.area for o in objects)
        if total_area > 0:
            self.label_smart_centroid.setText(f"Área: {int(total_area)} px²")
            self.label_smart_centroid.setStyleSheet("color: #1ABC9C;")
        else:
            self.label_smart_centroid.setText("Área: --")
            self.label_smart_centroid.setStyleSheet("color: #95A5A6;")
    
    def _reset_smart_parameters(self):
        """Resetea parámetros SMART a valores por defecto."""
        self.spin_saliency_thresh.blockSignals(True)
        self.spin_min_prob.blockSignals(True)
        self.spin_min_area.blockSignals(True)
        self.spin_focus_thresh.blockSignals(True)
        self.cb_use_laplacian.blockSignals(True)
        
        self.spin_saliency_thresh.setValue(0.5)
        self.spin_min_prob.setValue(0.3)
        self.spin_min_area.setValue(500)
        self.spin_focus_thresh.setValue(20.0)
        self.cb_use_laplacian.setChecked(True)
        
        self.spin_saliency_thresh.blockSignals(False)
        self.spin_min_prob.blockSignals(False)
        self.spin_min_area.blockSignals(False)
        self.spin_focus_thresh.blockSignals(False)
        self.cb_use_laplacian.blockSignals(False)
        
        self._on_smart_params_changed()
        self.label_status.setText("Parámetros SMART reseteados")
        self.label_status.setStyleSheet("color: #3498DB;")
    
    # === Parámetros Legacy - Callbacks de Sliders/Spins ===
    
    def _on_auto_threshold_changed(self, state):
        """Cambia entre modo automático y manual de umbral Z."""
        is_auto = state == Qt.Checked
        
        # Habilitar/deshabilitar controles manuales
        self.slider_z_threshold.setEnabled(not is_auto)
        self.spin_z_threshold.setEnabled(not is_auto)
        self.label_z_threshold.setEnabled(not is_auto)
        
        # Actualizar detector
        self.sharpness_detector.use_automatic_threshold = is_auto
        
        # Re-analizar
        self._on_params_changed()
        
        # Feedback
        mode = "Automático (Otsu + Histéresis)" if is_auto else "Manual"
        self.label_status.setText(f"Modo de umbral: {mode}")
        self.label_status.setStyleSheet("color: #3498DB;")
    
    def _on_z_slider_changed(self, value):
        """Slider Z-Score cambió."""
        self.spin_z_threshold.blockSignals(True)
        self.spin_z_threshold.setValue(value / 10.0)
        self.spin_z_threshold.blockSignals(False)
        self._on_params_changed()
    
    def _on_z_spin_changed(self, value):
        """Spin Z-Score cambió."""
        self.slider_z_threshold.blockSignals(True)
        self.slider_z_threshold.setValue(int(value * 10))
        self.slider_z_threshold.blockSignals(False)
        self._on_params_changed()
    
    def _on_min_object_slider_changed(self, value):
        """Slider Min Object cambió."""
        self.spin_min_object.blockSignals(True)
        self.spin_min_object.setValue(value / 100.0)
        self.spin_min_object.blockSignals(False)
        self._on_params_changed()
    
    def _on_min_object_spin_changed(self, value):
        """Spin Min Object cambió."""
        self.slider_min_object.blockSignals(True)
        self.slider_min_object.setValue(int(value * 100))
        self.slider_min_object.blockSignals(False)
        self._on_params_changed()
    
    def _on_s_slider_changed(self, value):
        """Slider Sharpness cambió."""
        self.spin_s_threshold.blockSignals(True)
        self.spin_s_threshold.setValue(float(value))
        self.spin_s_threshold.blockSignals(False)
        self._on_params_changed()
    
    def _on_s_spin_changed(self, value):
        """Spin Sharpness cambió."""
        self.slider_s_threshold.blockSignals(True)
        self.slider_s_threshold.setValue(int(value))
        self.slider_s_threshold.blockSignals(False)
        self._on_params_changed()
    
    def _on_morph_slider_changed(self, value):
        """Slider Morph cambió."""
        # Asegurar valor impar
        if value % 2 == 0:
            value += 1
        self.spin_morph.blockSignals(True)
        self.spin_morph.setValue(value)
        self.spin_morph.blockSignals(False)
        self._on_params_changed()
    
    def _on_morph_spin_changed(self, value):
        """Spin Morph cambió."""
        # Asegurar valor impar
        if value % 2 == 0:
            value += 1
            self.spin_morph.blockSignals(True)
            self.spin_morph.setValue(value)
            self.spin_morph.blockSignals(False)
        self.slider_morph.blockSignals(True)
        self.slider_morph.setValue(value)
        self.slider_morph.blockSignals(False)
        self._on_params_changed()
    
    def _on_dilation_slider_changed(self, value):
        """Slider Dilatación cambió."""
        if value % 2 == 0:
            value += 1
        self.spin_dilation.blockSignals(True)
        self.spin_dilation.setValue(value)
        self.spin_dilation.blockSignals(False)
        self._on_params_changed()
    
    def _on_dilation_spin_changed(self, value):
        """Spin Dilatación cambió."""
        if value % 2 == 0:
            value += 1
            self.spin_dilation.blockSignals(True)
            self.spin_dilation.setValue(value)
            self.spin_dilation.blockSignals(False)
        self.slider_dilation.blockSignals(True)
        self.slider_dilation.setValue(value)
        self.slider_dilation.blockSignals(False)
        self._on_params_changed()
    
    def _on_params_changed(self):
        """Actualiza parámetros del detector con todos los valores."""
        # Calcular kernel size (0 si morfología deshabilitada)
        morph_enabled = self.cb_enable_morph.isChecked()
        morph_kernel = self.spin_morph.value() if morph_enabled else 0
        
        # Actualizar detector
        self.sharpness_detector.set_parameters(
            z_threshold=self.spin_z_threshold.value(),
            sharpness_threshold=self.spin_s_threshold.value(),
            morph_kernel_size=morph_kernel,
            min_object_ratio=self.spin_min_object.value() / 100.0
        )
        
        # Modo automático vs manual
        self.sharpness_detector.use_automatic_threshold = self.cb_auto_threshold.isChecked()
        
        # Actualizar factor de "Muy Enfocada"
        self.sharpness_detector.very_focused_multiplier = self.spin_very_focused_mult.value()
        
        # Configurar operaciones morfológicas (solo aplica en modo manual)
        self.sharpness_detector.morph_open = self.cb_morph_open.isChecked()
        self.sharpness_detector.morph_close = self.cb_morph_close.isChecked()
        
        # Configurar ROI Dilatada para Sharpness
        self.sharpness_detector.dilate_mask_for_sharpness = self.cb_dilate_roi.isChecked()
        self.sharpness_detector.dilation_kernel_size = self.spin_dilation.value()
        self.sharpness_detector.use_mse_sharpness = self.cb_use_mse.isChecked()
        
        # Configurar closing kernel (usa mismo valor que morph)
        self.sharpness_detector.closing_kernel_size = morph_kernel
        
        # Re-analizar imagen actual si hay una
        current = self.list_images.currentItem()
        if current and self.sharpness_detector.is_ready:
            self._analyze_current_image(current)
    
    def _set_view_mode(self, mode):
        """Cambia modo de visualización con radio-button behavior."""
        # Desmarcar otros checkboxes
        self.cb_view_normal.blockSignals(True)
        self.cb_view_zscore.blockSignals(True)
        self.cb_view_mask.blockSignals(True)
        self.cb_debug.blockSignals(True)
        
        self.cb_view_normal.setChecked(mode == "normal")
        self.cb_view_zscore.setChecked(mode == "zscore")
        self.cb_view_mask.setChecked(mode == "mask")
        self.cb_debug.setChecked(mode == "debug")
        
        self.cb_view_normal.blockSignals(False)
        self.cb_view_zscore.blockSignals(False)
        self.cb_view_mask.blockSignals(False)
        self.cb_debug.blockSignals(False)
        
        self.image_viewer.set_display_mode(mode)
    
    def _reset_parameters(self):
        """Resetea todos los parámetros a valores por defecto."""
        # Bloquear señales durante reset
        self.cb_auto_threshold.blockSignals(True)
        self.spin_z_threshold.blockSignals(True)
        self.slider_z_threshold.blockSignals(True)
        self.spin_min_object.blockSignals(True)
        self.slider_min_object.blockSignals(True)
        self.spin_s_threshold.blockSignals(True)
        self.slider_s_threshold.blockSignals(True)
        self.spin_very_focused_mult.blockSignals(True)
        self.spin_morph.blockSignals(True)
        self.slider_morph.blockSignals(True)
        self.cb_enable_morph.blockSignals(True)
        self.cb_morph_open.blockSignals(True)
        self.cb_morph_close.blockSignals(True)
        self.cb_dilate_roi.blockSignals(True)
        self.spin_dilation.blockSignals(True)
        self.slider_dilation.blockSignals(True)
        self.cb_use_mse.blockSignals(True)
        
        # Valores por defecto
        self.cb_auto_threshold.setChecked(True)  # Modo automático por defecto
        self.spin_z_threshold.setValue(3.0)
        self.slider_z_threshold.setValue(30)
        self.spin_z_threshold.setEnabled(False)
        self.slider_z_threshold.setEnabled(False)
        self.spin_min_object.setValue(0.1)
        self.slider_min_object.setValue(10)
        self.spin_s_threshold.setValue(50.0)
        self.slider_s_threshold.setValue(50)
        self.spin_very_focused_mult.setValue(5.0)  # Ajustado para MSE
        self.spin_morph.setValue(5)
        self.slider_morph.setValue(5)
        self.cb_enable_morph.setChecked(True)
        self.cb_morph_open.setChecked(True)
        self.cb_morph_close.setChecked(True)
        # ROI Dilatada
        self.cb_dilate_roi.setChecked(True)
        self.spin_dilation.setValue(15)
        self.slider_dilation.setValue(15)
        self.cb_use_mse.setChecked(True)
        
        # Desbloquear señales
        self.cb_auto_threshold.blockSignals(False)
        self.spin_z_threshold.blockSignals(False)
        self.slider_z_threshold.blockSignals(False)
        self.spin_min_object.blockSignals(False)
        self.slider_min_object.blockSignals(False)
        self.spin_s_threshold.blockSignals(False)
        self.slider_s_threshold.blockSignals(False)
        self.spin_very_focused_mult.blockSignals(False)
        self.spin_morph.blockSignals(False)
        self.slider_morph.blockSignals(False)
        self.cb_enable_morph.blockSignals(False)
        self.cb_morph_open.blockSignals(False)
        self.cb_morph_close.blockSignals(False)
        self.cb_dilate_roi.blockSignals(False)
        self.spin_dilation.blockSignals(False)
        self.slider_dilation.blockSignals(False)
        self.cb_use_mse.blockSignals(False)
        
        # Vista normal
        self._set_view_mode("normal")
        
        # Aplicar cambios
        self._on_params_changed()
        
        self.label_status.setText("Parámetros reseteados a valores por defecto")
        self.label_status.setStyleSheet("color: #3498DB;")
    
    # === Análisis ===
    
    def _load_test_images(self):
        """Carga carpeta de imágenes para analizar."""
        if not self.sharpness_detector.is_ready:
            QMessageBox.warning(self, "Modelo no cargado", 
                               "Debe calibrar el modelo primero.")
            return
        
        folder = QFileDialog.getExistingDirectory(
            self, "Seleccionar Carpeta de Imágenes a Analizar", ""
        )
        if folder:
            self.current_folder = folder
            self._load_images(folder)
    
    def _load_images(self, folder):
        """Carga imágenes de la carpeta."""
        self.list_images.clear()
        self.results_cache.clear()
        self.image_files = list_images_in_folder(folder)
        
        if not self.image_files:
            self.label_status.setText("No se encontraron imágenes")
            self.label_status.setStyleSheet("color: #E74C3C;")
            self.label_count.setText("Imágenes: 0")
            return
        
        for filepath in self.image_files:
            filename = os.path.basename(filepath)
            item = QListWidgetItem(f"⚪ {filename}")
            item.setData(Qt.UserRole, filepath)
            self.list_images.addItem(item)
        
        self.label_folder.setText(self._truncate_path(folder, 35))
        self.label_folder.setToolTip(folder)
        self.label_count.setText(f"Imágenes: {len(self.image_files)}")
        self.label_status.setText(f"Cargadas {len(self.image_files)} imágenes")
        self.label_status.setStyleSheet("color: #2ECC71;")
        
        logger.info(f"Cargadas {len(self.image_files)} imágenes de {folder}")
        
        if self.list_images.count() > 0:
            self.list_images.setCurrentRow(0)
    
    def _on_image_selected(self, current, previous):
        """Maneja selección de imagen."""
        if current is None:
            return
        self._analyze_current_image(current)
    
    def _analyze_current_image(self, item):
        """Analiza la imagen seleccionada usando el método activo."""
        filepath = item.data(Qt.UserRole)
        filename = os.path.basename(filepath)
        
        # Cargar imagen
        img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.label_status.setText(f"Error cargando {filename}")
            self.label_status.setStyleSheet("color: #E74C3C;")
            return
        
        if self.analysis_method == "smart":
            # === MÉTODO SMART (U2-Net Salient Object Detection) ===
            focus_result = self.smart_scorer.assess_image(img)
            
            # Guardar FocusResult original para visualización
            self._last_focus_result = focus_result
            
            # Actualizar estadísticas SMART
            self._update_smart_stats(focus_result)
            
            # Actualizar estado del modelo
            if self.smart_scorer.is_model_loaded():
                self.label_model_status_smart.setText("✅ U2-Net")
                self.label_model_status_smart.setStyleSheet("color: #2ECC71; font-weight: bold;")
            
            # Convertir FocusResult a formato compatible con el visor
            result = self._convert_focus_result(focus_result, img)
            
        else:
            # === MÉTODO LEGACY (Z-Score + Laplaciano) ===
            if not self.sharpness_detector.is_ready:
                self.label_status.setText("⚠️ Modelo no calibrado")
                self.label_status.setStyleSheet("color: #E74C3C;")
                return
            
            result = self.sharpness_detector.analyze_array(img, return_maps=True)
        
        if result is None:
            self.label_status.setText("Error en análisis")
            self.label_status.setStyleSheet("color: #E74C3C;")
            return
        
        # Guardar en cache
        self.results_cache[filename] = result
        
        # Actualizar visor único
        self.image_viewer.display_result(img, result)
        
        # Actualizar vista SMART si está visible
        if self.analysis_method == "smart" and self.viewer_stack.currentIndex() == 1:
            self.single_view.display_result(img, focus_result)
        
        # Actualizar lista
        self._update_list_item(item, filename, result)
        
        # Actualizar estadísticas
        self._update_stats(filename, result)
        
        # Emitir señal
        score = result.sharpness if hasattr(result, 'sharpness') else result.focus_score
        self.analysis_completed.emit(filename, score)
    
    def _convert_focus_result(self, focus_result: FocusResult, img: np.ndarray) -> SharpnessResult:
        """Convierte FocusResult (Smart ROI-Based) a SharpnessResult para compatibilidad."""
        # Mapear estados ROI-Based
        state_map = {
            "FOCUSED_OBJECT": "Muy Enfocada",
            "BLURRY_OBJECT": "Desenfocada",
            "EMPTY": "Sin Objeto",
            "ERROR": "Sin Objeto"
        }
        focus_state = state_map.get(focus_result.status, "Sin Objeto")
        is_focused = focus_result.status == "FOCUSED_OBJECT"
        
        # Crear máscara binaria con bounding box si hay objeto
        mask = np.zeros(img.shape, dtype=np.uint8)
        if focus_result.bounding_box is not None:
            x, y, w, h = focus_result.bounding_box
            mask[y:y+h, x:x+w] = 255
        
        # Calcular ratio de objeto
        object_pixels = int(focus_result.contour_area) if focus_result.contour_area else 0
        object_ratio = object_pixels / img.size if img.size > 0 else 0.0
        
        # Usar debug_mask como z_score_map para visualización
        # La debug_mask ya es BGR con bordes, contorno y bbox dibujados
        z_score_map = None
        if focus_result.debug_mask is not None:
            # Convertir BGR a grayscale para compatibilidad con el visor
            if len(focus_result.debug_mask.shape) == 3:
                z_score_map = cv2.cvtColor(focus_result.debug_mask, cv2.COLOR_BGR2GRAY).astype(np.float32)
            else:
                z_score_map = focus_result.debug_mask.astype(np.float32)
        
        return SharpnessResult(
            sharpness=focus_result.focus_score,
            is_focused=is_focused,
            focus_state=focus_state,
            object_pixels=object_pixels,
            object_ratio=object_ratio,
            z_score_max=focus_result.focus_score,
            z_score_mean_object=focus_result.raw_score,
            bias_correction=0.0,
            optimal_z_threshold=focus_result.raw_score,
            hysteresis_low=self.smart_scorer.focus_threshold,
            z_score_map=z_score_map,
            mask=mask,
            laplacian_map=focus_result.debug_mask  # Guardar la imagen de debug BGR aquí
        )
    
    def _update_list_item(self, item, filename, result: SharpnessResult):
        """Actualiza item en la lista."""
        emoji_map = {
            "Muy Enfocada": "🟢",
            "Enfocada": "🟡",
            "Desenfocada": "🔴",
            "Sin Objeto": "⚫"
        }
        emoji = emoji_map.get(result.focus_state, "⚪")
        item.setText(f"{emoji} {filename}\n    S={result.sharpness:.1f}")
    
    def _update_stats(self, filename, result: SharpnessResult):
        """Actualiza panel de estadísticas."""
        self.label_current_file.setText(filename)
        self.label_sharpness.setText(f"{result.sharpness:.2f}")
        
        # Color según estado
        color_map = {
            "Muy Enfocada": "#2ECC71",
            "Enfocada": "#F1C40F",
            "Desenfocada": "#E74C3C",
            "Sin Objeto": "#888888"
        }
        color = color_map.get(result.focus_state, "#FFFFFF")
        
        self.label_sharpness.setStyleSheet(f"font-weight: bold; font-size: 14px; color: {color};")
        self.label_focus_state.setText(result.focus_state)
        self.label_focus_state.setStyleSheet(f"color: {color}; font-weight: bold;")
        
        self.label_object_info.setText(f"{result.object_ratio*100:.2f}% ({result.object_pixels} px)")
        
        # Mostrar info según método
        if self.analysis_method == "smart":
            self.label_zscore.setText(f"Score={result.z_score_max:.1f}")
            self.label_thresholds.setText(f"Raw={result.optimal_z_threshold:.1f}")
            self.label_bias.setText(f"Thresh={result.hysteresis_low:.0f}")
        else:
            self.label_zscore.setText(f"{result.z_score_max:.2f}")
            self.label_thresholds.setText(f"{result.optimal_z_threshold:.2f} / {result.hysteresis_low:.2f}")
            self.label_bias.setText(f"{result.bias_correction:.2f}")
        
        method_name = "SMART" if self.analysis_method == "smart" else "LEGACY"
        self.label_status.setText(f"Análisis completado ({method_name})")
        self.label_status.setStyleSheet("color: #2ECC71;")
    
    def _export_results(self):
        """Exporta resultados a CSV."""
        if not self.results_cache:
            self.label_status.setText("No hay resultados para exportar")
            self.label_status.setStyleSheet("color: #E74C3C;")
            return
        
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Guardar Resultados", "resultados_robusto.csv", "CSV (*.csv)"
        )
        
        if filepath:
            try:
                with open(filepath, 'w', encoding='utf-8') as f:
                    f.write("Archivo,Sharpness,Estado,Objeto_Ratio,Z_Score_Max\n")
                    for filename, result in sorted(self.results_cache.items()):
                        f.write(f"{filename},{result.sharpness:.2f},{result.focus_state},"
                               f"{result.object_ratio:.4f},{result.z_score_max:.2f}\n")
                
                self.label_status.setText(f"Exportado: {os.path.basename(filepath)}")
                self.label_status.setStyleSheet("color: #2ECC71;")
                logger.info(f"Resultados exportados a {filepath}")
            except Exception as e:
                self.label_status.setText(f"Error: {e}")
                self.label_status.setStyleSheet("color: #E74C3C;")
    
    def _truncate_path(self, path, max_len):
        """Trunca path largo."""
        if len(path) <= max_len:
            return path
        return "..." + path[-(max_len - 3):]
    
    # === API Pública ===
    
    def get_current_result(self) -> SharpnessResult:
        """Retorna resultado actual."""
        return self.image_viewer.current_result
    
    def get_sharpness_detector(self) -> SharpnessDetector:
        """Retorna el detector de sharpness para uso externo."""
        return self.sharpness_detector

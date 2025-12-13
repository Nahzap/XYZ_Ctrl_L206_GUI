"""
Saliency Overlay Widget - Visualización con Overlays en Tiempo Real
===================================================================

Widget que superpone información de detección sobre la imagen de cámara:
- Mapa de saliencia (semi-transparente)
- Bounding boxes de objetos detectados
- Scores y probabilidades
- Indicador de Z durante autofoco

Autor: Sistema de Control L206
Fecha: 2025-12-12
"""

import numpy as np
import cv2
from typing import List, Optional, Tuple

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QCheckBox, QLabel
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor, QPen, QFont

from core.detection.u2net_detector import DetectedObject


class SaliencyOverlayWidget(QWidget):
    """
    Widget de visualización con overlays de detección.
    
    Muestra:
    - Frame de cámara base
    - Mapa de saliencia superpuesto (opcional)
    - Bounding boxes de objetos detectados
    - Scores de cada objeto
    - Indicador de Z y score durante autofoco
    
    Signals:
        object_clicked: Emitido cuando se hace clic en un objeto (index)
    """
    
    object_clicked = pyqtSignal(int)  # índice del objeto clickeado
    
    # Colores para objetos (ciclo de colores)
    OBJECT_COLORS = [
        (0, 255, 0),    # Verde
        (255, 0, 0),    # Azul (BGR)
        (0, 255, 255),  # Amarillo
        (255, 0, 255),  # Magenta
        (255, 255, 0),  # Cyan
        (128, 0, 255),  # Naranja
    ]
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Datos de visualización
        self.current_frame: Optional[np.ndarray] = None
        self.saliency_map: Optional[np.ndarray] = None
        self.detected_objects: List[DetectedObject] = []
        
        # Estado de autofoco
        self.autofocus_active = False
        self.current_z = 0.0
        self.current_score = 0.0
        self.active_object_index = -1
        
        # Opciones de visualización
        self.show_saliency = True
        self.show_boxes = True
        self.show_scores = True
        self.show_centroids = True
        self.saliency_alpha = 0.4  # Transparencia del mapa de saliencia
        
        # Imagen renderizada
        self._rendered_pixmap: Optional[QPixmap] = None
        
        # Label para mostrar imagen
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(320, 240)
        self.image_label.setStyleSheet("background-color: #1a1a1a;")
        
        # Layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.image_label)
        
        # Info label
        self.info_label = QLabel("Sin imagen")
        self.info_label.setStyleSheet("color: #888; font-size: 10px;")
        self.info_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.info_label)
    
    def update_frame(self, frame: np.ndarray):
        """Actualiza el frame base de la cámara."""
        if frame is None:
            return
        self.current_frame = frame.copy()
        self._render()
    
    def update_detection(self, saliency_map: np.ndarray, objects: List[DetectedObject]):
        """Actualiza los resultados de detección."""
        self.saliency_map = saliency_map
        self.detected_objects = objects
        self._render()
    
    def update_autofocus_state(self, z: float, score: float, active_obj_index: int = -1):
        """Actualiza el estado del autofoco."""
        self.autofocus_active = (active_obj_index >= 0)
        self.current_z = z
        self.current_score = score
        self.active_object_index = active_obj_index
        self._render()
    
    def clear_autofocus_state(self):
        """Limpia el estado de autofoco."""
        self.autofocus_active = False
        self.active_object_index = -1
        self._render()
    
    def set_visualization_options(self, show_saliency: bool = None, show_boxes: bool = None,
                                   show_scores: bool = None, show_centroids: bool = None,
                                   saliency_alpha: float = None):
        """Configura opciones de visualización."""
        if show_saliency is not None:
            self.show_saliency = show_saliency
        if show_boxes is not None:
            self.show_boxes = show_boxes
        if show_scores is not None:
            self.show_scores = show_scores
        if show_centroids is not None:
            self.show_centroids = show_centroids
        if saliency_alpha is not None:
            self.saliency_alpha = saliency_alpha
        self._render()
    
    def _render(self):
        """Renderiza el frame con todos los overlays."""
        if self.current_frame is None:
            self.info_label.setText("Sin imagen")
            return
        
        # Copiar frame base
        frame = self.current_frame.copy()
        
        # Asegurar que es BGR para dibujar
        if len(frame.shape) == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        
        # Normalizar si es uint16
        if frame.dtype == np.uint16:
            frame = (frame / 256).astype(np.uint8)
        
        h, w = frame.shape[:2]
        
        # 1. Overlay de saliencia
        if self.show_saliency and self.saliency_map is not None:
            frame = self._draw_saliency_overlay(frame)
        
        # 2. Bounding boxes y scores
        if self.show_boxes and self.detected_objects:
            frame = self._draw_bounding_boxes(frame)
        
        # 3. Indicador de autofoco
        if self.autofocus_active:
            frame = self._draw_autofocus_indicator(frame)
        
        # Convertir a QPixmap
        self._rendered_pixmap = self._numpy_to_pixmap(frame)
        
        # Escalar para ajustar al widget
        scaled_pixmap = self._rendered_pixmap.scaled(
            self.image_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.image_label.setPixmap(scaled_pixmap)
        
        # Actualizar info
        n_objects = len(self.detected_objects)
        info_text = f"{w}x{h} | {n_objects} objetos"
        if self.autofocus_active:
            info_text += f" | Z={self.current_z:.1f}µm S={self.current_score:.1f}"
        self.info_label.setText(info_text)
    
    def _draw_saliency_overlay(self, frame: np.ndarray) -> np.ndarray:
        """Dibuja el mapa de saliencia como overlay semi-transparente."""
        h, w = frame.shape[:2]
        
        # Redimensionar saliencia si es necesario
        saliency = self.saliency_map
        if saliency.shape[:2] != (h, w):
            saliency = cv2.resize(saliency, (w, h))
        
        # Crear mapa de color (rojo para alta saliencia)
        saliency_uint8 = (saliency * 255).astype(np.uint8)
        saliency_color = cv2.applyColorMap(saliency_uint8, cv2.COLORMAP_JET)
        
        # Mezclar con frame original
        frame = cv2.addWeighted(frame, 1 - self.saliency_alpha, saliency_color, self.saliency_alpha, 0)
        
        return frame
    
    def _draw_bounding_boxes(self, frame: np.ndarray) -> np.ndarray:
        """Dibuja bounding boxes y scores de objetos detectados."""
        for obj in self.detected_objects:
            x, y, w, h = obj.bbox
            color = self.OBJECT_COLORS[obj.index % len(self.OBJECT_COLORS)]
            
            # Grosor mayor si es el objeto activo en autofoco
            thickness = 3 if obj.index == self.active_object_index else 2
            
            # Dibujar rectángulo
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, thickness)
            
            # Dibujar centroide
            if self.show_centroids:
                cx, cy = obj.centroid
                cv2.circle(frame, (cx, cy), 5, color, -1)
            
            # Dibujar score
            if self.show_scores:
                label = f"#{obj.index} P={obj.probability:.2f}"
                
                # Fondo para texto
                (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(frame, (x, y - text_h - 8), (x + text_w + 4, y), color, -1)
                
                # Texto
                cv2.putText(frame, label, (x + 2, y - 4), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return frame
    
    def _draw_autofocus_indicator(self, frame: np.ndarray) -> np.ndarray:
        """Dibuja indicador de estado de autofoco."""
        h, w = frame.shape[:2]
        
        # Panel de información en esquina superior derecha
        panel_w, panel_h = 180, 60
        panel_x = w - panel_w - 10
        panel_y = 10
        
        # Fondo semi-transparente
        overlay = frame.copy()
        cv2.rectangle(overlay, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), 
                     (0, 0, 0), -1)
        frame = cv2.addWeighted(frame, 0.7, overlay, 0.3, 0)
        
        # Texto
        cv2.putText(frame, "AUTOFOCO ACTIVO", (panel_x + 5, panel_y + 18),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame, f"Z: {self.current_z:.2f} um", (panel_x + 5, panel_y + 35),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Score: {self.current_score:.1f}", (panel_x + 5, panel_y + 52),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Resaltar objeto activo
        if self.active_object_index >= 0 and self.active_object_index < len(self.detected_objects):
            obj = self.detected_objects[self.active_object_index]
            x, y, bw, bh = obj.bbox
            
            # Borde pulsante (más grueso)
            cv2.rectangle(frame, (x-2, y-2), (x + bw + 2, y + bh + 2), (0, 255, 0), 4)
        
        return frame
    
    def _numpy_to_pixmap(self, frame: np.ndarray) -> QPixmap:
        """Convierte numpy array BGR a QPixmap."""
        h, w = frame.shape[:2]
        
        if len(frame.shape) == 2:
            # Grayscale
            q_image = QImage(frame.data, w, h, w, QImage.Format_Grayscale8)
        else:
            # BGR -> RGB
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            q_image = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
        
        return QPixmap.fromImage(q_image)
    
    def get_object_at_position(self, x: int, y: int) -> Optional[int]:
        """Retorna el índice del objeto en la posición dada, o None."""
        for obj in self.detected_objects:
            ox, oy, ow, oh = obj.bbox
            if ox <= x <= ox + ow and oy <= y <= oy + oh:
                return obj.index
        return None
    
    def mousePressEvent(self, event):
        """Maneja clics en objetos."""
        if event.button() == Qt.LeftButton:
            # Convertir posición del widget a posición de imagen
            # (simplificado - asume que la imagen llena el widget)
            pos = event.pos()
            obj_idx = self.get_object_at_position(pos.x(), pos.y())
            if obj_idx is not None:
                self.object_clicked.emit(obj_idx)
        super().mousePressEvent(event)


class SaliencyControlPanel(QWidget):
    """Panel de controles para opciones de visualización."""
    
    options_changed = pyqtSignal(dict)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 2, 5, 2)
        
        # Checkboxes
        self.cb_saliency = QCheckBox("Saliencia")
        self.cb_saliency.setChecked(True)
        self.cb_saliency.stateChanged.connect(self._emit_options)
        layout.addWidget(self.cb_saliency)
        
        self.cb_boxes = QCheckBox("Boxes")
        self.cb_boxes.setChecked(True)
        self.cb_boxes.stateChanged.connect(self._emit_options)
        layout.addWidget(self.cb_boxes)
        
        self.cb_scores = QCheckBox("Scores")
        self.cb_scores.setChecked(True)
        self.cb_scores.stateChanged.connect(self._emit_options)
        layout.addWidget(self.cb_scores)
        
        layout.addStretch()
    
    def _emit_options(self):
        """Emite las opciones actuales."""
        self.options_changed.emit({
            'show_saliency': self.cb_saliency.isChecked(),
            'show_boxes': self.cb_boxes.isChecked(),
            'show_scores': self.cb_scores.isChecked()
        })
    
    def get_options(self) -> dict:
        """Retorna las opciones actuales."""
        return {
            'show_saliency': self.cb_saliency.isChecked(),
            'show_boxes': self.cb_boxes.isChecked(),
            'show_scores': self.cb_scores.isChecked()
        }

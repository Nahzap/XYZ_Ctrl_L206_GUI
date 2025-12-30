"""
Diálogo de confirmación para sistema de aprendizaje de ROIs.
Permite al usuario confirmar si el ROI detectado es válido o no.
"""

import logging
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, 
                             QPushButton, QProgressBar, QWidget)
from PyQt5.QtCore import Qt, QTimer, QPoint, QRect
from PyQt5.QtGui import QPixmap, QImage, QCursor, QPainter, QPen, QColor
import numpy as np
import cv2

logger = logging.getLogger(__name__)


class InteractiveImageLabel(QLabel):
    """QLabel interactivo para dibujar rectángulos (ROIs manuales)."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)
        self._drawing_enabled = False
        self._start_pos = None
        self._current_rect = None
        self._scaled_pixmap = None
        self._base_pixmap = None
        self._scale_x = 1.0
        self._scale_y = 1.0
        self._offset_x = 0
        self._offset_y = 0
        self._image_w = 0
        self._image_h = 0
        # ROIs en coordenadas ORIGINALES (x, y, w, h)
        self._custom_rois = []

    def set_image_geometry(self, image_w: int, image_h: int, scaled_w: int, scaled_h: int, offset_x: int, offset_y: int):
        self._image_w = image_w
        self._image_h = image_h
        self._scale_x = float(scaled_w) / float(image_w) if image_w > 0 else 1.0
        self._scale_y = float(scaled_h) / float(image_h) if image_h > 0 else 1.0
        self._offset_x = offset_x
        self._offset_y = offset_y

    def setPixmap(self, pixmap: QPixmap):
        super().setPixmap(pixmap)
        # Guardar copias para overlay
        self._scaled_pixmap = pixmap
        self._base_pixmap = QPixmap(pixmap)
        # Redibujar overlays existentes
        self._redraw_overlays()

    def set_drawing_enabled(self, enabled: bool):
        self._drawing_enabled = enabled

    def clear_rois(self):
        self._custom_rois = []
        self._redraw_overlays()

    def get_custom_rois(self):
        return list(self._custom_rois)

    def mousePressEvent(self, event):
        if not self._drawing_enabled:
            return super().mousePressEvent(event)
        if event.button() == 1:  # Left button
            pos = event.pos()
            if not self._point_in_image_area(pos):
                return
            self._start_pos = pos
            self._current_rect = None

    def mouseMoveEvent(self, event):
        if not self._drawing_enabled or self._start_pos is None:
            return super().mouseMoveEvent(event)
        end_pos = event.pos()
        self._current_rect = self._make_rect(self._start_pos, end_pos)
        self._redraw_overlays(temp_rect=self._current_rect)

    def mouseReleaseEvent(self, event):
        if not self._drawing_enabled or self._start_pos is None:
            return super().mouseReleaseEvent(event)
        end_pos = event.pos()
        rect = self._make_rect(self._start_pos, end_pos)
        self._start_pos = None
        self._current_rect = None
        # Convertir a coordenadas ORIGINALES
        if rect is not None:
            x = max(0, int((rect.x() - self._offset_x) / self._scale_x))
            y = max(0, int((rect.y() - self._offset_y) / self._scale_y))
            w = int(rect.width() / self._scale_x)
            h = int(rect.height() / self._scale_y)
            # Clampear a imagen
            if x + w > self._image_w:
                w = self._image_w - x
            if y + h > self._image_h:
                h = self._image_h - y
            if w > 2 and h > 2:
                self._custom_rois.append((x, y, w, h))
        self._redraw_overlays()

    def _point_in_image_area(self, pt: QPoint) -> bool:
        if self._scaled_pixmap is None:
            return False
        rect = self.contentsRect()
        img_rect = rect.adjusted(self._offset_x, self._offset_y, -self._offset_x, -self._offset_y)
        return img_rect.contains(pt)

    def _make_rect(self, p1: QPoint, p2: QPoint):
        x1, y1 = p1.x(), p1.y()
        x2, y2 = p2.x(), p2.y()
        x = min(x1, x2)
        y = min(y1, y2)
        w = abs(x2 - x1)
        h = abs(y2 - y1)
        # Restringir a área visible de la imagen
        x = max(self._offset_x, min(x, self.width() - self._offset_x))
        y = max(self._offset_y, min(y, self.height() - self._offset_y))
        return QRect(x, y, w, h)

    def _redraw_overlays(self, temp_rect=None):
        if self._base_pixmap is None:
            return
        pix = QPixmap(self._base_pixmap)
        painter = QPainter(pix)
        pen_saved = QPen(QColor(255, 255, 0))  # Amarillo para existentes
        pen_saved.setWidth(3)
        painter.setPen(pen_saved)
        # Dibujar ROIs existentes (convertir a coords escaladas)
        for (x, y, w, h) in self._custom_rois:
            sx = int(x * self._scale_x) + self._offset_x
            sy = int(y * self._scale_y) + self._offset_y
            sw = int(w * self._scale_x)
            sh = int(h * self._scale_y)
            painter.drawRect(sx, sy, sw, sh)

        # Rectángulo temporal (naranja)
        if temp_rect is not None:
            pen_temp = QPen(QColor(255, 165, 0))
            pen_temp.setWidth(2)
            painter.setPen(pen_temp)
            painter.drawRect(temp_rect)
        painter.end()
        super().setPixmap(pix)


class LearningConfirmationDialog(QDialog):
    """
    Diálogo para confirmar si un ROI es válido durante el aprendizaje.
    
    Muestra:
    - Imagen con ROI/máscara resaltada
    - Contador de imágenes aprendidas (X/50)
    - Botones: "✓ Sí, es válido" y "✗ No, descartar"
    - Cuenta regresiva de 10 segundos (auto-acepta si no responde)
    """
    
    def __init__(self, parent=None):
        super().__init__(parent, Qt.Window | Qt.WindowStaysOnTopHint)
        self.setWindowTitle("📚 Aprendizaje de ROI")
        self.setMinimumSize(800, 700)
        self.setStyleSheet("""
            QDialog {
                background-color: #2C2C2C;
                color: white;
            }
            QLabel {
                color: white;
            }
        """)
        
        self.user_response = None  # True=válido, False=descartar, None=timeout
        self.countdown_seconds = 10
        self.countdown_timer = QTimer()
        self.countdown_timer.timeout.connect(self._update_countdown)
        
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        
        # Título y progreso
        header = QHBoxLayout()
        
        self.title_label = QLabel("📚 ¿Este ROI es válido para aprendizaje?")
        self.title_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #3498DB;")
        header.addWidget(self.title_label)
        
        header.addStretch()
        
        self.progress_label = QLabel("Progreso: 0/50")
        self.progress_label.setStyleSheet("font-size: 14px; font-weight: bold; color: #27AE60;")
        header.addWidget(self.progress_label)
        
        layout.addLayout(header)
        
        # Barra de progreso
        self.progress_bar = QProgressBar()
        self.progress_bar.setMaximum(50)
        self.progress_bar.setValue(0)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid #555;
                border-radius: 5px;
                text-align: center;
                background-color: #1E1E1E;
                color: white;
                font-weight: bold;
            }
            QProgressBar::chunk {
                background-color: #27AE60;
                border-radius: 3px;
            }
        """)
        layout.addWidget(self.progress_bar)
        
        # Imagen con ROI
        self.image_label = InteractiveImageLabel("Cargando imagen...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("background-color: #000; border: 2px solid #505050;")
        self.image_label.setMinimumSize(640, 480)
        layout.addWidget(self.image_label)
        
        # Info del ROI
        self.roi_info_label = QLabel("ROI: Área=0 px, Score=0.0")
        self.roi_info_label.setStyleSheet("font-size: 12px; color: #95A5A6;")
        self.roi_info_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.roi_info_label)
        
        # Cuenta regresiva
        self.countdown_label = QLabel("⏱️ Auto-aceptar en: 10s")
        self.countdown_label.setStyleSheet("font-size: 14px; font-weight: bold; color: #E67E22;")
        self.countdown_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.countdown_label)
        
        # Botones de decisión
        buttons_layout = QHBoxLayout()
        
        self.reject_btn = QPushButton("✗ No, descartar")
        self.reject_btn.setStyleSheet("""
            QPushButton {
                background-color: #E74C3C;
                color: white;
                font-size: 16px;
                font-weight: bold;
                padding: 15px 30px;
                border-radius: 8px;
            }
            QPushButton:hover { background-color: #C0392B; }
        """)
        self.reject_btn.clicked.connect(self._on_reject)
        buttons_layout.addWidget(self.reject_btn)
        
        self.accept_btn = QPushButton("✓ Sí, es válido")
        self.accept_btn.setStyleSheet("""
            QPushButton {
                background-color: #27AE60;
                color: white;
                font-size: 16px;
                font-weight: bold;
                padding: 15px 30px;
                border-radius: 8px;
            }
            QPushButton:hover { background-color: #229954; }
        """)
        self.accept_btn.clicked.connect(self._on_accept)
        buttons_layout.addWidget(self.accept_btn)

        # Botones para segmentación manual
        self.add_roi_btn = QPushButton("✎ Añadir ROI manual")
        self.add_roi_btn.setToolTip("Activa modo dibujo: arrastra sobre la imagen para añadir un ROI")
        self.add_roi_btn.clicked.connect(self._toggle_draw_mode)
        buttons_layout.addWidget(self.add_roi_btn)

        self.clear_rois_btn = QPushButton("🧹 Limpiar ROIs")
        self.clear_rois_btn.setToolTip("Elimina todos los ROIs manuales dibujados")
        self.clear_rois_btn.clicked.connect(self._on_clear_rois)
        buttons_layout.addWidget(self.clear_rois_btn)

        self.use_manual_btn = QPushButton("✅ Usar ROIs manuales")
        self.use_manual_btn.setToolTip("Reemplaza la segmentación detectada por tus ROIs manuales")
        self.use_manual_btn.clicked.connect(self._on_use_manual_rois)
        buttons_layout.addWidget(self.use_manual_btn)
        
        layout.addLayout(buttons_layout)
        
        # Instrucciones
        instructions = QLabel(
            "💡 Confirma si el objeto detectado (resaltado en verde) es válido.\n"
            "El sistema aprenderá de tus respuestas para mejorar la detección."
        )
        instructions.setStyleSheet("font-size: 11px; color: #7F8C8D; padding: 5px;")
        instructions.setAlignment(Qt.AlignCenter)
        instructions.setWordWrap(True)
        layout.addWidget(instructions)
    
    def show_roi_for_confirmation(self, frame, roi_bbox, roi_mask, area, score, 
                                   current_count, total_count=50):
        """
        Muestra el ROI para confirmación del usuario.
        
        Args:
            frame: Frame BGR de la cámara
            roi_bbox: (x, y, w, h) del ROI
            roi_mask: Máscara binaria del objeto
            area: Área del objeto en píxeles
            score: Score de enfoque
            current_count: Número actual de imágenes aprendidas
            total_count: Total de imágenes objetivo (default 50)
        
        Returns:
            True si válido, False si descartado, None si timeout
        """
        # Actualizar progreso
        self.progress_label.setText(f"Progreso: {current_count}/{total_count}")
        self.progress_bar.setValue(current_count)
        self.progress_bar.setMaximum(total_count)
        
        # Actualizar info del ROI
        self.roi_info_label.setText(f"ROI: Área={area:.0f} px, Score={score:.2f}")
        
        # Dibujar ROI y máscara en el frame
        frame_with_roi = self._draw_roi_on_frame(frame.copy(), roi_bbox, roi_mask)
        
        # Convertir a QImage y mostrar
        q_image = self._numpy_to_qimage(frame_with_roi)
        pixmap = QPixmap.fromImage(q_image)
        scaled = pixmap.scaled(self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.image_label.setPixmap(scaled)
        # Configurar geometría para convertir coord. dibujo -> imagen original
        lw, lh = self.image_label.width(), self.image_label.height()
        sw, sh = scaled.width(), scaled.height()
        off_x = int((lw - sw) / 2)
        off_y = int((lh - sh) / 2)
        self.image_label.set_image_geometry(frame.shape[1], frame.shape[0], sw, sh, off_x, off_y)
        
        # Reiniciar cuenta regresiva
        self.countdown_seconds = 10
        self.user_response = None
        self.countdown_timer.start(1000)  # 1 segundo
        
        # Mover el cursor automáticamente al botón de Aceptar para acelerar el flujo
        try:
            QTimer.singleShot(150, self._move_cursor_to_accept)
        except Exception:
            pass
        
        # Mostrar diálogo modal
        self.exec_()
        
        return self.user_response

    def _move_cursor_to_accept(self):
        """Mueve el cursor al centro del botón de Aceptar (no hace click)."""
        try:
            center = self.accept_btn.rect().center()
            global_pos = self.accept_btn.mapToGlobal(center)
            QCursor.setPos(global_pos)
        except Exception:
            pass
    
    def _toggle_draw_mode(self):
        """Activa/desactiva modo de dibujo de ROIs manuales."""
        enabled = not getattr(self, '_draw_mode', False)
        self._draw_mode = enabled
        self.image_label.set_drawing_enabled(enabled)
        self.add_roi_btn.setText("✎ Añadir ROI (ON)" if enabled else "✎ Añadir ROI manual")
    
    def _on_clear_rois(self):
        self.image_label.clear_rois()
    
    def _on_use_manual_rois(self):
        """Devuelve respuesta con ROIs manuales para reemplazar segmentación detectada."""
        rois = self.image_label.get_custom_rois()
        if not rois:
            # No hay ROIs; no hacemos nada
            return
        # Construir respuesta enriquecida
        self.countdown_timer.stop()
        self.user_response = {
            'accepted': True,
            'replace': True,
            'custom_rois': rois,
        }
        logger.info(f"Usuario definió {len(rois)} ROIs manuales (reemplazar segmentación)")
        self.accept()
    
    def _draw_roi_on_frame(self, frame, bbox, mask):
        """Dibuja el ROI y la máscara sobre el frame."""
        x, y, w, h = bbox
        
        # Dibujar máscara semi-transparente (verde)
        if mask is not None:
            # Redimensionar máscara al tamaño del frame si es necesario
            if mask.shape[:2] != frame.shape[:2]:
                mask_resized = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
            else:
                mask_resized = mask
            
            # Crear overlay verde
            overlay = frame.copy()
            overlay[mask_resized > 0] = [0, 255, 0]  # Verde
            
            # Mezclar con transparencia
            cv2.addWeighted(overlay, 0.3, frame, 0.7, 0, frame)
        
        # Dibujar bounding box (verde brillante)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
        
        # Etiqueta
        label = f"ROI: {w}x{h} px"
        cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.7, (0, 255, 0), 2)
        
        return frame
    
    def _numpy_to_qimage(self, frame):
        """Convierte numpy BGR a QImage."""
        if frame is None:
            return QImage()
        
        frame = np.ascontiguousarray(frame)
        h, w = frame.shape[:2]
        
        if len(frame.shape) == 2:
            return QImage(frame.data, w, h, w, QImage.Format_Grayscale8).copy()
        else:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb = np.ascontiguousarray(rgb)
            return QImage(rgb.data, w, h, 3*w, QImage.Format_RGB888).copy()
    
    def _update_countdown(self):
        """Actualiza la cuenta regresiva."""
        self.countdown_seconds -= 1
        self.countdown_label.setText(f"⏱️ Auto-aceptar en: {self.countdown_seconds}s")
        
        if self.countdown_seconds <= 0:
            self.countdown_timer.stop()
            logger.info("Timeout en confirmación de aprendizaje - auto-aceptando")
            self.user_response = True  # Auto-aceptar por timeout
            self.accept()
    
    def _on_accept(self):
        """Usuario confirmó que el ROI es válido."""
        self.countdown_timer.stop()
        self.user_response = True
        logger.info("Usuario confirmó ROI como válido")
        self.accept()
    
    def _on_reject(self):
        """Usuario rechazó el ROI."""
        self.countdown_timer.stop()
        self.user_response = False
        logger.info("Usuario rechazó ROI")
        self.accept()
    
    def closeEvent(self, event):
        """Detener timer al cerrar."""
        self.countdown_timer.stop()
        super().closeEvent(event)

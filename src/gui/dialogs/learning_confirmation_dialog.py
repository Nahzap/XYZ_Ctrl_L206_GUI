"""
Diálogo de confirmación para sistema de aprendizaje de ROIs.
Permite al usuario confirmar si el ROI detectado es válido o no.
"""

import logging
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, 
                             QPushButton, QProgressBar, QWidget)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QImage
import numpy as np
import cv2

logger = logging.getLogger(__name__)


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
        self.image_label = QLabel("Cargando imagen...")
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
        
        # Reiniciar cuenta regresiva
        self.countdown_seconds = 10
        self.user_response = None
        self.countdown_timer.start(1000)  # 1 segundo
        
        # Mostrar diálogo modal
        self.exec_()
        
        return self.user_response
    
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

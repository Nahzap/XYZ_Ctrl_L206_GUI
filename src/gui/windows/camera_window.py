"""
Ventana para visualización de cámara Thorlabs.

Esta ventana muestra los frames capturados por la cámara en tiempo real
con información de resolución y contador de frames.
"""

import logging
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
from gui.styles.dark_theme import DARK_STYLESHEET

logger = logging.getLogger(__name__)


class CameraViewWindow(QWidget):
    """Ventana independiente redimensionable para visualizar la cámara."""
    
    def __init__(self, parent=None):
        """
        Inicializa la ventana de vista de cámara.
        
        Args:
            parent: Widget padre (opcional)
        """
        super().__init__(parent, Qt.Window)
        self.setWindowTitle('🎥 Vista de Cámara Thorlabs - Tiempo Real')
        self.setGeometry(200, 200, 800, 600)
        self.setStyleSheet(DARK_STYLESHEET)
        
        layout = QVBoxLayout(self)
        
        # Label para mostrar video
        self.video_label = QLabel("Esperando frames de la cámara...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("""
            QLabel {
                background-color: #000000;
                color: #00FF00;
                border: 2px solid #505050;
                font-size: 14px;
                font-weight: bold;
            }
        """)
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setScaledContents(False)
        
        layout.addWidget(self.video_label)
        
        # Información de frame
        info_layout = QHBoxLayout()
        self.frame_info_label = QLabel("Frame: 0 | Resolución: --- | FPS: ---")
        self.frame_info_label.setStyleSheet("color: #95A5A6; font-size: 10px;")
        info_layout.addWidget(self.frame_info_label)
        info_layout.addStretch()
        layout.addLayout(info_layout)
        
        self.frame_count = 0
        self.current_pixmap = None
        
        logger.debug("CameraViewWindow creada exitosamente")
        
    def update_frame(self, q_image):
        """
        Actualiza el frame mostrado en la ventana.
        
        Args:
            q_image: QImage con el frame capturado
        """
        try:
            self.frame_count += 1
            
            # Convertir QImage a QPixmap
            pixmap = QPixmap.fromImage(q_image)
            self.current_pixmap = pixmap
            
            # Escalar manteniendo aspect ratio
            scaled_pixmap = pixmap.scaled(
                self.video_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            
            self.video_label.setPixmap(scaled_pixmap)
            
            # Actualizar info
            self.frame_info_label.setText(
                f"Frame: {self.frame_count} | Resolución: {q_image.width()}x{q_image.height()}"
            )
            
        except Exception as e:
            logger.error(f"Error actualizando frame: {e}")
    
    def resizeEvent(self, event):
        """Reescalar imagen cuando se redimensiona la ventana."""
        super().resizeEvent(event)
        if self.current_pixmap:
            scaled_pixmap = self.current_pixmap.scaled(
                self.video_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            self.video_label.setPixmap(scaled_pixmap)

"""
Pestaña de Grabación de Datos.

Encapsula la UI y lógica de grabación de experimentos.
Usa DataRecorder para la lógica de archivos CSV.
"""

import logging
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QGridLayout, QGroupBox,
                             QLabel, QLineEdit, QPushButton)
from PyQt5.QtCore import pyqtSignal

logger = logging.getLogger('MotorControl_L206')


class RecordingTab(QWidget):
    """
    Pestaña para grabación de experimentos.
    
    Signals:
        recording_started: Emitido cuando se inicia grabación exitosamente
        recording_stopped: Emitido cuando se detiene grabación
    """
    
    # Señales para comunicación con ArduinoGUI
    recording_started = pyqtSignal(str)  # filename
    recording_stopped = pyqtSignal()
    
    def __init__(self, data_recorder, parent=None):
        """
        Inicializa la pestaña de grabación.
        
        Args:
            data_recorder: Instancia de DataRecorder para manejar archivos
            parent: Widget padre (ArduinoGUI)
        """
        super().__init__(parent)
        self.data_recorder = data_recorder
        self.parent_gui = parent
        self._setup_ui()
        logger.debug("RecordingTab inicializado")
    
    def _setup_ui(self):
        """Configura la interfaz de usuario."""
        layout = QVBoxLayout(self)
        
        # Grupo principal de grabación
        group_box = QGroupBox("Registro de Experimento (Respuesta al Escalón)")
        grid_layout = QGridLayout()
        
        # Nombre de archivo
        grid_layout.addWidget(QLabel("Nombre Archivo:"), 0, 0)
        self.filename_input = QLineEdit("experimento_escalon.csv")
        self.filename_input.setPlaceholderText("Nombre del archivo CSV...")
        grid_layout.addWidget(self.filename_input, 0, 1)
        
        # Botón iniciar
        self.start_btn = QPushButton("▶️ Iniciar Grabación")
        self.start_btn.setStyleSheet("""
            QPushButton {
                font-size: 12px;
                font-weight: bold;
                padding: 8px;
                background-color: #27AE60;
            }
            QPushButton:hover { background-color: #2ECC71; }
            QPushButton:pressed { background-color: #1E8449; }
            QPushButton:disabled { background-color: #505050; color: #808080; }
        """)
        self.start_btn.clicked.connect(self.start_recording)
        grid_layout.addWidget(self.start_btn, 1, 0)
        
        # Botón detener
        self.stop_btn = QPushButton("⏹️ Detener Grabación")
        self.stop_btn.setStyleSheet("""
            QPushButton {
                font-size: 12px;
                font-weight: bold;
                padding: 8px;
                background-color: #E74C3C;
            }
            QPushButton:hover { background-color: #EC7063; }
            QPushButton:pressed { background-color: #C0392B; }
            QPushButton:disabled { background-color: #505050; color: #808080; }
        """)
        self.stop_btn.clicked.connect(self.stop_recording)
        self.stop_btn.setEnabled(False)
        grid_layout.addWidget(self.stop_btn, 1, 1)
        
        # Estado
        self.status_label = QLabel("Estado: Detenido")
        self.status_label.setStyleSheet("color: #E67E22; font-weight: bold;")
        grid_layout.addWidget(self.status_label, 2, 0, 1, 2)
        
        group_box.setLayout(grid_layout)
        layout.addWidget(group_box)
        layout.addStretch()  # Empujar todo hacia arriba
    
    def start_recording(self):
        """Inicia la grabación de datos usando DataRecorder."""
        logger.info("=== BOTÓN: Iniciar Grabación presionado ===")
        filename = self.filename_input.text()
        
        success, message = self.data_recorder.start_recording(filename)
        
        if success:
            self.status_label.setText(message)
            self.status_label.setStyleSheet("color: #2ECC71; font-weight: bold;")
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.filename_input.setEnabled(False)
            self.recording_started.emit(filename)
            logger.debug("Grabación iniciada exitosamente")
        else:
            self.status_label.setText(message)
            self.status_label.setStyleSheet("color: #E74C3C; font-weight: bold;")
            logger.error(f"Error al iniciar grabación: {message}")
    
    def stop_recording(self):
        """Detiene la grabación de datos usando DataRecorder."""
        logger.info("=== BOTÓN: Detener Grabación presionado ===")
        
        message = self.data_recorder.stop_recording()
        
        self.status_label.setText(message)
        self.status_label.setStyleSheet("color: #E67E22; font-weight: bold;")
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.filename_input.setEnabled(True)
        self.recording_stopped.emit()
        logger.debug("Grabación detenida")
    
    def get_filename(self):
        """Retorna el nombre del archivo actual."""
        return self.filename_input.text()
    
    def is_recording(self):
        """Retorna True si está grabando actualmente."""
        return self.data_recorder.is_recording

"""
Pestaña de Control de Cámara Thorlabs.

Encapsula la UI para control de cámara y microscopía automatizada.
La lógica de cámara está en hardware/camera/camera_worker.py
"""

import os
import logging
from datetime import datetime

import numpy as np
import cv2

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QGroupBox, QLabel, QLineEdit, QPushButton,
                             QCheckBox, QFileDialog, QTextEdit, QMessageBox,
                             QComboBox, QSpinBox, QDoubleSpinBox, QScrollArea)
from PyQt5.QtCore import pyqtSignal, Qt

# Imports para lógica de cámara
try:
    import pylablib as pll
    pll.par["devices/dlls/thorlabs_tlcam"] = r"C:\Program Files\Thorlabs\ThorImageCAM\Bin"
    from pylablib.devices import Thorlabs
    THORLABS_AVAILABLE = True
except:
    THORLABS_AVAILABLE = False

from hardware.camera.camera_worker import CameraWorker
from gui.windows.camera_window import CameraViewWindow

logger = logging.getLogger('MotorControl_L206')


class CameraTab(QWidget):
    """
    Pestaña para control de cámara Thorlabs y microscopía automatizada.
    
    Signals:
        connect_requested: Solicita conexión de cámara
        disconnect_requested: Solicita desconexión
        view_requested: Solicita abrir vista de cámara
        start_live_requested: Solicita iniciar vista en vivo
        stop_live_requested: Solicita detener vista en vivo
        capture_requested: Solicita capturar imagen
        exposure_changed: Nuevo valor de exposición (float)
        microscopy_start_requested: Solicita iniciar microscopía automatizada
        microscopy_stop_requested: Solicita detener microscopía
    """
    
    connect_requested = pyqtSignal()
    disconnect_requested = pyqtSignal()
    view_requested = pyqtSignal()
    start_live_requested = pyqtSignal()
    stop_live_requested = pyqtSignal()
    capture_requested = pyqtSignal(str)  # folder path
    exposure_changed = pyqtSignal(float)
    fps_changed = pyqtSignal(int)
    buffer_changed = pyqtSignal(int)
    microscopy_start_requested = pyqtSignal(dict)  # config dict
    microscopy_stop_requested = pyqtSignal()
    
    def __init__(self, thorlabs_available=False, parent=None):
        """
        Inicializa la pestaña de cámara.
        
        Args:
            thorlabs_available: Si pylablib está disponible
            parent: Widget padre (ArduinoGUI)
        """
        super().__init__(parent)
        self.thorlabs_available = THORLABS_AVAILABLE  # Usar detección automática
        self.parent_gui = parent
        
        # Variables de cámara
        self.camera_worker = None
        self.camera_view_window = None
        self.camera_log = []
        
        # Referencia a TestTab para obtener trayectoria
        self.test_tab = None
        
        self._setup_ui()
        logger.debug("CameraTab inicializado")
    
    def set_test_tab_reference(self, test_tab):
        """
        Configura la referencia a TestTab para sincronizar trayectoria.
        
        Args:
            test_tab: Instancia de TestTab
        """
        self.test_tab = test_tab
        logger.debug("TestTab reference configurada en CameraTab")
        
        # Conectar señal de cambio de trayectoria si existe
        if hasattr(test_tab, 'trajectory_changed'):
            test_tab.trajectory_changed.connect(self._on_trajectory_changed)
    
    def _on_trajectory_changed(self, n_points):
        """Callback cuando cambia la trayectoria en TestTab."""
        self.set_trajectory_status(n_points > 0, n_points)
    
    def refresh_trajectory_from_test_tab(self):
        """Actualiza el estado de trayectoria desde TestTab."""
        if self.test_tab and hasattr(self.test_tab, 'current_trajectory'):
            trajectory = self.test_tab.current_trajectory
            if trajectory is not None and len(trajectory) > 0:
                self.set_trajectory_status(True, len(trajectory))
                self.log_message(f"📍 Trayectoria sincronizada: {len(trajectory)} puntos")
                return True
        self.set_trajectory_status(False, 0)
        return False
    
    def _setup_ui(self):
        """Configura la interfaz de usuario."""
        # Crear scroll area para permitir navegación vertical
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        
        # Widget contenedor para el contenido
        content_widget = QWidget()
        main_layout = QVBoxLayout(content_widget)
        
        # Sección 1: Conexión
        connection_group = QGroupBox("1️⃣ Conexión")
        conn_layout = QVBoxLayout()
        
        conn_buttons = QHBoxLayout()
        self.connect_btn = QPushButton("🔌 Conectar Cámara")
        self.connect_btn.setStyleSheet("""
            QPushButton { font-size: 13px; font-weight: bold; padding: 8px; background-color: #27AE60; }
            QPushButton:hover { background-color: #2ECC71; }
            QPushButton:disabled { background-color: #505050; color: #808080; }
        """)
        self.connect_btn.clicked.connect(self.connect_camera)
        
        self.disconnect_btn = QPushButton("🔌 Desconectar")
        self.disconnect_btn.setEnabled(False)
        self.disconnect_btn.clicked.connect(self.disconnect_camera)
        
        self.detect_btn = QPushButton("🔍 Detectar Cámaras")
        self.detect_btn.clicked.connect(self.detect_thorlabs_camera)
        
        if not self.thorlabs_available:
            self.connect_btn.setEnabled(False)
            self.connect_btn.setText("⚠️ pylablib no instalado")
            self.detect_btn.setEnabled(False)
        
        conn_buttons.addWidget(self.connect_btn)
        conn_buttons.addWidget(self.disconnect_btn)
        conn_buttons.addWidget(self.detect_btn)
        conn_buttons.addStretch()
        conn_layout.addLayout(conn_buttons)
        
        self.camera_info_label = QLabel("Estado: Desconectada")
        self.camera_info_label.setStyleSheet("color: #E74C3C; font-weight: bold;")
        conn_layout.addWidget(self.camera_info_label)
        
        connection_group.setLayout(conn_layout)
        main_layout.addWidget(connection_group)
        
        # Sección 2: Vista en Vivo
        view_group = QGroupBox("2️⃣ Vista en Vivo")
        view_layout = QVBoxLayout()
        
        view_buttons = QHBoxLayout()
        self.view_btn = QPushButton("📹 Ver Cámara")
        self.view_btn.setStyleSheet("""
            QPushButton { font-size: 13px; font-weight: bold; padding: 8px; background-color: #2E86C1; }
            QPushButton:hover { background-color: #3498DB; }
            QPushButton:disabled { background-color: #505050; color: #808080; }
        """)
        self.view_btn.setEnabled(False)
        self.view_btn.clicked.connect(self.open_camera_view)
        
        self.start_live_btn = QPushButton("▶️ Iniciar")
        self.start_live_btn.setEnabled(False)
        self.start_live_btn.clicked.connect(self.start_camera_live_view)
        
        self.stop_live_btn = QPushButton("⏹️ Detener")
        self.stop_live_btn.setEnabled(False)
        self.stop_live_btn.clicked.connect(self.stop_camera_live_view)
        
        view_buttons.addWidget(self.view_btn)
        view_buttons.addWidget(self.start_live_btn)
        view_buttons.addWidget(self.stop_live_btn)
        view_buttons.addStretch()
        view_layout.addLayout(view_buttons)
        
        view_group.setLayout(view_layout)
        main_layout.addWidget(view_group)
        
        # Sección 3: Configuración
        config_group = QGroupBox("3️⃣ Configuración")
        config_layout = QGridLayout()
        
        # Exposición
        config_layout.addWidget(QLabel("Exposición (s):"), 0, 0)
        self.exposure_input = QLineEdit("0.01")
        self.exposure_input.setFixedWidth(100)
        config_layout.addWidget(self.exposure_input, 0, 1)
        
        self.apply_exposure_btn = QPushButton("✓ Aplicar")
        self.apply_exposure_btn.setEnabled(False)
        self.apply_exposure_btn.setFixedWidth(80)
        self.apply_exposure_btn.clicked.connect(self._apply_exposure)
        config_layout.addWidget(self.apply_exposure_btn, 0, 2)
        
        # FPS
        config_layout.addWidget(QLabel("FPS:"), 1, 0)
        self.fps_input = QLineEdit("60")
        self.fps_input.setFixedWidth(100)
        config_layout.addWidget(self.fps_input, 1, 1)
        
        self.apply_fps_btn = QPushButton("✓ Aplicar")
        self.apply_fps_btn.setEnabled(False)
        self.apply_fps_btn.setFixedWidth(80)
        self.apply_fps_btn.clicked.connect(self._apply_fps)
        config_layout.addWidget(self.apply_fps_btn, 1, 2)
        
        # Buffer de imágenes
        config_layout.addWidget(QLabel("Buffer (frames):"), 2, 0)
        self.buffer_input = QLineEdit("2")  # Predeterminado: 2 frames
        self.buffer_input.setFixedWidth(100)
        self.buffer_input.setToolTip("Número de frames en buffer (1-10). Usar 2 para estabilidad.")
        config_layout.addWidget(self.buffer_input, 2, 1)
        
        self.apply_buffer_btn = QPushButton("✓ Aplicar")
        self.apply_buffer_btn.setEnabled(False)
        self.apply_buffer_btn.setFixedWidth(80)
        self.apply_buffer_btn.clicked.connect(self._apply_buffer)
        config_layout.addWidget(self.apply_buffer_btn, 2, 2)
        
        # Info de buffer
        buffer_info = QLabel("ℹ️ Buffer=2 recomendado: visualiza frame actual, guarda el anterior")
        buffer_info.setStyleSheet("color: #888888; font-size: 10px;")
        config_layout.addWidget(buffer_info, 3, 0, 1, 3)
        
        config_group.setLayout(config_layout)
        main_layout.addWidget(config_group)
        
        # Sección 4: Captura
        capture_group = QGroupBox("4️⃣ Captura de Imágenes")
        capture_layout = QVBoxLayout()
        
        folder_layout = QHBoxLayout()
        folder_layout.addWidget(QLabel("Carpeta:"))
        self.save_folder_input = QLineEdit(r"C:\CapturasCamara")
        folder_layout.addWidget(self.save_folder_input)
        
        browse_btn = QPushButton("📁 Explorar")
        browse_btn.clicked.connect(self._browse_folder)
        folder_layout.addWidget(browse_btn)
        capture_layout.addLayout(folder_layout)
        
        # Fila de formato de imagen
        format_layout = QHBoxLayout()
        format_layout.addWidget(QLabel("Formato:"))
        self.image_format_combo = QComboBox()
        self.image_format_combo.addItems(["PNG", "TIFF", "JPG"])
        self.image_format_combo.setCurrentText("PNG")  # Default PNG
        self.image_format_combo.setFixedWidth(80)
        self.image_format_combo.setToolTip("Formato de imagen para capturas")
        format_layout.addWidget(self.image_format_combo)
        format_layout.addStretch()
        capture_layout.addLayout(format_layout)
        
        capture_btn_layout = QHBoxLayout()
        self.capture_btn = QPushButton("📸 Capturar Imagen")
        self.capture_btn.setStyleSheet("""
            QPushButton { font-size: 14px; font-weight: bold; padding: 10px; background-color: #E67E22; }
            QPushButton:hover { background-color: #F39C12; }
            QPushButton:disabled { background-color: #505050; color: #808080; }
        """)
        self.capture_btn.setEnabled(False)
        self.capture_btn.clicked.connect(self.capture_single_image)
        capture_btn_layout.addWidget(self.capture_btn)
        capture_btn_layout.addStretch()
        capture_layout.addLayout(capture_btn_layout)
        
        capture_group.setLayout(capture_layout)
        main_layout.addWidget(capture_group)
        
        # Sección 5: Microscopía Automatizada
        microscopy_group = QGroupBox("🔬 Microscopía Automatizada")
        microscopy_layout = QVBoxLayout()
        
        # Info
        info_label = QLabel(
            "ℹ️ <b>Ejecuta la trayectoria zig-zag con captura automática de imágenes</b><br>"
            "Usa la trayectoria generada en la pestaña 'Prueba' y captura una imagen en cada punto."
        )
        info_label.setWordWrap(True)
        info_label.setStyleSheet("padding: 8px; background-color: #34495E; border-radius: 5px;")
        microscopy_layout.addWidget(info_label)
        
        # Estado de trayectoria
        traj_layout = QHBoxLayout()
        traj_layout.addWidget(QLabel("<b>Estado:</b>"))
        self.trajectory_status = QLabel("⚪ Sin trayectoria")
        self.trajectory_status.setStyleSheet("color: #95A5A6; font-weight: bold;")
        traj_layout.addWidget(self.trajectory_status)
        
        # Botón para actualizar trayectoria desde TestTab
        refresh_traj_btn = QPushButton("🔄 Actualizar")
        refresh_traj_btn.setFixedWidth(100)
        refresh_traj_btn.setToolTip("Sincronizar trayectoria desde pestaña Prueba")
        refresh_traj_btn.clicked.connect(self.refresh_trajectory_from_test_tab)
        traj_layout.addWidget(refresh_traj_btn)
        
        traj_layout.addStretch()
        microscopy_layout.addLayout(traj_layout)
        
        # Fila 1: Nombre de clase + Tamaño imagen
        row1_layout = QHBoxLayout()
        row1_layout.addWidget(QLabel("Nombre clase:"))
        self.class_name_input = QLineEdit("Especie_001")
        self.class_name_input.setFixedWidth(150)
        self.class_name_input.setPlaceholderText("Ej: Rosa_Canina")
        self.class_name_input.textChanged.connect(self._update_storage_estimate)
        row1_layout.addWidget(self.class_name_input)
        
        row1_layout.addSpacing(20)
        row1_layout.addWidget(QLabel("Tamaño imagen (px):"))
        self.img_width_input = QLineEdit("1920")
        self.img_width_input.setFixedWidth(60)
        self.img_width_input.textChanged.connect(self._update_storage_estimate)
        row1_layout.addWidget(self.img_width_input)
        row1_layout.addWidget(QLabel("×"))
        self.img_height_input = QLineEdit("1080")
        self.img_height_input.setFixedWidth(60)
        self.img_height_input.textChanged.connect(self._update_storage_estimate)
        row1_layout.addWidget(self.img_height_input)
        row1_layout.addStretch()
        microscopy_layout.addLayout(row1_layout)
        
        # Fila 2: Canales RGB + Estimación
        row2_layout = QHBoxLayout()
        row2_layout.addWidget(QLabel("Canales RGB:"))
        self.channel_r_check = QCheckBox("R")
        self.channel_r_check.setStyleSheet("color: #E74C3C; font-weight: bold;")
        self.channel_r_check.stateChanged.connect(self._update_storage_estimate)
        row2_layout.addWidget(self.channel_r_check)
        
        self.channel_g_check = QCheckBox("G")
        self.channel_g_check.setStyleSheet("color: #27AE60; font-weight: bold;")
        self.channel_g_check.setChecked(True)  # Por defecto G para cámara mono
        self.channel_g_check.stateChanged.connect(self._update_storage_estimate)
        row2_layout.addWidget(self.channel_g_check)
        
        self.channel_b_check = QCheckBox("B")
        self.channel_b_check.setStyleSheet("color: #3498DB; font-weight: bold;")
        self.channel_b_check.stateChanged.connect(self._update_storage_estimate)
        row2_layout.addWidget(self.channel_b_check)
        
        row2_layout.addSpacing(30)
        row2_layout.addWidget(QLabel("Estimación:"))
        self.storage_estimate_label = QLabel("~0 MB")
        self.storage_estimate_label.setStyleSheet("font-weight: bold; color: #F39C12;")
        row2_layout.addWidget(self.storage_estimate_label)
        row2_layout.addStretch()
        microscopy_layout.addLayout(row2_layout)
        
        # Fila 3: Carpeta de destino para microscopia
        folder_micro_layout = QHBoxLayout()
        folder_micro_layout.addWidget(QLabel("Carpeta destino:"))
        self.microscopy_folder_input = QLineEdit("C:\\MicroscopyData")
        self.microscopy_folder_input.setMinimumWidth(300)
        self.microscopy_folder_input.setToolTip("Carpeta donde se guardaran las imagenes")
        folder_micro_layout.addWidget(self.microscopy_folder_input)
        
        browse_micro_btn = QPushButton("Explorar")
        browse_micro_btn.clicked.connect(self._browse_microscopy_folder)
        folder_micro_layout.addWidget(browse_micro_btn)
        folder_micro_layout.addStretch()
        microscopy_layout.addLayout(folder_micro_layout)
        
        # Fila 4: Demoras antes y despues
        row3_layout = QHBoxLayout()
        row3_layout.addWidget(QLabel("Demora antes (s):"))
        self.delay_before_input = QLineEdit("2.0")
        self.delay_before_input.setFixedWidth(60)
        self.delay_before_input.setToolTip("Tiempo de espera antes de capturar (estabilizacion)")
        row3_layout.addWidget(self.delay_before_input)
        
        row3_layout.addSpacing(30)
        row3_layout.addWidget(QLabel("Demora despues (s):"))
        self.delay_after_input = QLineEdit("0.2")
        self.delay_after_input.setFixedWidth(60)
        self.delay_after_input.setToolTip("Tiempo de espera despues de capturar")
        row3_layout.addWidget(self.delay_after_input)
        row3_layout.addStretch()
        microscopy_layout.addLayout(row3_layout)
        
        # Botones de microscopía
        micro_btn_layout = QHBoxLayout()
        self.microscopy_start_btn = QPushButton("🚀 Iniciar Microscopía")
        self.microscopy_start_btn.setStyleSheet("""
            QPushButton { font-size: 13px; font-weight: bold; padding: 10px; background-color: #27AE60; }
            QPushButton:hover { background-color: #2ECC71; }
            QPushButton:disabled { background-color: #505050; color: #808080; }
        """)
        self.microscopy_start_btn.setEnabled(False)
        self.microscopy_start_btn.clicked.connect(self._start_microscopy)
        
        self.microscopy_stop_btn = QPushButton("⏹️ Detener")
        self.microscopy_stop_btn.setStyleSheet("background-color: #E74C3C; font-weight: bold; padding: 10px;")
        self.microscopy_stop_btn.setEnabled(False)
        self.microscopy_stop_btn.clicked.connect(self._stop_microscopy)
        
        micro_btn_layout.addWidget(self.microscopy_start_btn)
        micro_btn_layout.addWidget(self.microscopy_stop_btn)
        micro_btn_layout.addStretch()
        microscopy_layout.addLayout(micro_btn_layout)
        
        # Progreso
        self.microscopy_progress_label = QLabel("Progreso: 0 / 0 imágenes capturadas")
        self.microscopy_progress_label.setStyleSheet("font-weight: bold; color: #3498DB;")
        microscopy_layout.addWidget(self.microscopy_progress_label)
        
        microscopy_group.setLayout(microscopy_layout)
        main_layout.addWidget(microscopy_group)
        
        # Sección 5.5: Autofoco C-Focus Multi-Objeto
        autofocus_group = QGroupBox("🔍 Autofoco Multi-Objeto (C-Focus)")
        autofocus_layout = QVBoxLayout()
        
        # Checkbox para habilitar autofoco
        self.autofocus_enabled_cb = QCheckBox("Habilitar autofoco por objeto")
        self.autofocus_enabled_cb.setToolTip(
            "Pre-detecta objetos con U2-Net y captura una imagen enfocada por cada uno.\n"
            "Genera N imágenes por punto, donde N = objetos detectados."
        )
        autofocus_layout.addWidget(self.autofocus_enabled_cb)
        
        # Botones de conexión C-Focus
        cfocus_btn_layout = QHBoxLayout()
        self.cfocus_connect_btn = QPushButton("🔌 Conectar C-Focus")
        self.cfocus_connect_btn.setStyleSheet("""
            QPushButton { font-size: 12px; font-weight: bold; padding: 6px; background-color: #8E44AD; }
            QPushButton:hover { background-color: #9B59B6; }
            QPushButton:disabled { background-color: #505050; color: #808080; }
        """)
        self.cfocus_connect_btn.clicked.connect(self._connect_cfocus)
        cfocus_btn_layout.addWidget(self.cfocus_connect_btn)
        
        self.cfocus_disconnect_btn = QPushButton("⏹️ Desconectar")
        self.cfocus_disconnect_btn.setEnabled(False)
        self.cfocus_disconnect_btn.clicked.connect(self._disconnect_cfocus)
        cfocus_btn_layout.addWidget(self.cfocus_disconnect_btn)
        
        # Botón para test de detección
        self.test_detection_btn = QPushButton("🔍 Test Detección")
        self.test_detection_btn.setToolTip("Muestra visualización de detección de objetos en tiempo real")
        self.test_detection_btn.clicked.connect(self._test_detection)
        cfocus_btn_layout.addWidget(self.test_detection_btn)
        
        cfocus_btn_layout.addStretch()
        autofocus_layout.addLayout(cfocus_btn_layout)
        
        # Modo de autofoco
        autofocus_mode_layout = QHBoxLayout()
        autofocus_mode_layout.addWidget(QLabel("Modo Z-scan:"))
        
        self.full_scan_cb = QCheckBox("Escaneo Completo (0-100µm)")
        self.full_scan_cb.setChecked(True)
        self.full_scan_cb.setToolTip(
            "Escanea todo el rango Z evaluando índice S para encontrar BPoF.\n"
            "Más lento pero más preciso. Desmarcar para Golden Section Search."
        )
        autofocus_mode_layout.addWidget(self.full_scan_cb)
        autofocus_mode_layout.addStretch()
        autofocus_layout.addLayout(autofocus_mode_layout)
        
        # Parámetros de detección (UMBRAL DE PÍXELES)
        detection_form = QGridLayout()
        
        detection_form.addWidget(QLabel("Área mínima:"), 0, 0)
        self.min_pixels_spin = QSpinBox()
        self.min_pixels_spin.setRange(10, 100000)
        self.min_pixels_spin.setValue(100)
        self.min_pixels_spin.setSuffix(" px")
        self.min_pixels_spin.setToolTip("Área mínima del objeto en píxeles")
        self.min_pixels_spin.setFixedWidth(100)
        detection_form.addWidget(self.min_pixels_spin, 0, 1)
        
        detection_form.addWidget(QLabel("Área máxima:"), 0, 2)
        self.max_pixels_spin = QSpinBox()
        self.max_pixels_spin.setRange(100, 500000)
        self.max_pixels_spin.setValue(50000)
        self.max_pixels_spin.setSuffix(" px")
        self.max_pixels_spin.setToolTip("Área máxima del objeto en píxeles")
        self.max_pixels_spin.setFixedWidth(100)
        detection_form.addWidget(self.max_pixels_spin, 0, 3)
        
        # Parámetros de búsqueda Z
        detection_form.addWidget(QLabel("Rango Z:"), 1, 0)
        self.z_range_spin = QDoubleSpinBox()
        self.z_range_spin.setRange(5.0, 200.0)
        self.z_range_spin.setValue(50.0)
        self.z_range_spin.setSuffix(" µm")
        self.z_range_spin.setToolTip("Rango total de búsqueda de foco")
        self.z_range_spin.setFixedWidth(100)
        detection_form.addWidget(self.z_range_spin, 1, 1)
        
        detection_form.addWidget(QLabel("Tolerancia:"), 1, 2)
        self.z_tolerance_spin = QDoubleSpinBox()
        self.z_tolerance_spin.setRange(0.1, 5.0)
        self.z_tolerance_spin.setValue(0.5)
        self.z_tolerance_spin.setSuffix(" µm")
        self.z_tolerance_spin.setDecimals(2)
        self.z_tolerance_spin.setToolTip("Tolerancia de convergencia")
        self.z_tolerance_spin.setFixedWidth(100)
        detection_form.addWidget(self.z_tolerance_spin, 1, 3)
        
        autofocus_layout.addLayout(detection_form)
        
        # Label de estado C-Focus
        self.cfocus_status_label = QLabel("C-Focus: No conectado")
        self.cfocus_status_label.setStyleSheet("color: #888; font-style: italic;")
        autofocus_layout.addWidget(self.cfocus_status_label)
        
        autofocus_group.setLayout(autofocus_layout)
        main_layout.addWidget(autofocus_group)
        
        # Sección 6: Terminal de Log
        log_group = QGroupBox("📋 Log de Cámara")
        log_layout = QVBoxLayout()
        
        self.camera_terminal = QTextEdit()
        self.camera_terminal.setReadOnly(True)
        self.camera_terminal.setMaximumHeight(150)
        self.camera_terminal.setStyleSheet("""
            QTextEdit {
                background-color: #1a1a1a;
                color: #00FF00;
                font-family: 'Courier New', monospace;
                font-size: 11px;
                border: 1px solid #444444;
            }
        """)
        self.camera_terminal.setPlaceholderText("Eventos de cámara aparecerán aquí...")
        log_layout.addWidget(self.camera_terminal)
        
        # Botón para limpiar log
        clear_log_btn = QPushButton("🗑️ Limpiar Log")
        clear_log_btn.setFixedWidth(120)
        clear_log_btn.clicked.connect(self.camera_terminal.clear)
        log_layout.addWidget(clear_log_btn)
        
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)
        
        main_layout.addStretch()
        
        # Configurar scroll area
        scroll_area.setWidget(content_widget)
        
        # Layout principal del tab
        tab_layout = QVBoxLayout(self)
        tab_layout.setContentsMargins(0, 0, 0, 0)
        tab_layout.addWidget(scroll_area)
    
    def _apply_exposure(self):
        """Aplica el valor de exposición y muestra confirmación."""
        try:
            exposure = float(self.exposure_input.text())
            self.exposure_changed.emit(exposure)
            
            # Aplicar directamente al worker si existe
            if self.camera_worker:
                self.camera_worker.change_exposure(exposure)
            
            # Mostrar en ms para mejor legibilidad
            exposure_ms = exposure * 1000
            self.log_message(f"✅ Exposición configurada: {exposure}s ({exposure_ms:.1f}ms)")
        except ValueError:
            self.log_message("❌ Error: Valor de exposición inválido")
            logger.error("Valor de exposición inválido")
    
    def _apply_fps(self):
        """Aplica el valor de FPS."""
        try:
            fps = int(self.fps_input.text())
            self.fps_changed.emit(fps)
            self.log_message(f"FPS configurado: {fps}")
        except ValueError:
            self.log_message("❌ Error: Valor de FPS inválido")
            logger.error("Valor de FPS inválido")
    
    def _apply_buffer(self):
        """Aplica el valor de buffer."""
        try:
            buffer_size = int(self.buffer_input.text())
            if buffer_size < 1 or buffer_size > 10:
                self.log_message("❌ Error: Buffer debe estar entre 1 y 10")
                return
            
            self.buffer_changed.emit(buffer_size)
            
            # Aplicar directamente al worker si existe
            if self.camera_worker:
                self.camera_worker.change_buffer_size(buffer_size)
            
            self.log_message(f"✅ Buffer configurado: {buffer_size} frames")
        except ValueError:
            self.log_message("❌ Error: Valor de buffer inválido")
            logger.error("Valor de buffer inválido")
    
    def log_message(self, message: str):
        """Escribe un mensaje en la terminal de log."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.camera_terminal.append(f"[{timestamp}] {message}")
    
    def _browse_folder(self):
        """Abre dialogo para seleccionar carpeta."""
        folder = QFileDialog.getExistingDirectory(self, "Seleccionar carpeta de guardado")
        if folder:
            self.save_folder_input.setText(folder)
    
    def _browse_microscopy_folder(self):
        """Abre dialogo para seleccionar carpeta de microscopia."""
        folder = QFileDialog.getExistingDirectory(self, "Seleccionar carpeta para microscopia")
        if folder:
            self.microscopy_folder_input.setText(folder)
    
    def _update_storage_estimate(self):
        """Calcula y actualiza la estimacion de almacenamiento."""
        try:
            width = int(self.img_width_input.text()) if self.img_width_input.text() else 1920
            height = int(self.img_height_input.text()) if self.img_height_input.text() else 1080
            
            # Contar canales seleccionados
            n_channels = 0
            if self.channel_r_check.isChecked():
                n_channels += 1
            if self.channel_g_check.isChecked():
                n_channels += 1
            if self.channel_b_check.isChecked():
                n_channels += 1
            
            if n_channels == 0:
                n_channels = 1  # Minimo 1 canal
            
            # Logica: 1 canal = grayscale (1 byte), 2-3 canales = BGR (3 bytes)
            if n_channels == 1:
                bytes_per_pixel = 1  # Grayscale
            else:
                bytes_per_pixel = 3  # BGR
            
            # Obtener numero de puntos de trayectoria
            n_points = self._trajectory_n_points if hasattr(self, '_trajectory_n_points') else 0
            
            # Calcular tamano (PNG comprimido ~50% del raw)
            bytes_per_image = width * height * bytes_per_pixel * 0.5  # Factor compresion PNG
            total_bytes = bytes_per_image * max(1, n_points)
            total_mb = total_bytes / (1024 * 1024)
            
            if total_mb < 1:
                self.storage_estimate_label.setText(f"~{total_bytes/1024:.1f} KB")
            elif total_mb < 1024:
                self.storage_estimate_label.setText(f"~{total_mb:.1f} MB")
            else:
                self.storage_estimate_label.setText(f"~{total_mb/1024:.2f} GB")
                
        except ValueError:
            self.storage_estimate_label.setText("~0 MB")
    
    def _start_microscopy(self):
        """Inicia microscopía con la configuración actual."""
        # Verificar que hay trayectoria
        if not hasattr(self, '_trajectory_n_points') or self._trajectory_n_points == 0:
            self.log_message("❌ Error: No hay trayectoria generada")
            return
        
        # Verificar que hay al menos un canal seleccionado
        if not (self.channel_r_check.isChecked() or 
                self.channel_g_check.isChecked() or 
                self.channel_b_check.isChecked()):
            self.log_message("❌ Error: Selecciona al menos un canal RGB")
            return
        
        # Obtener configuracion
        try:
            config = {
                'class_name': self.class_name_input.text().strip().replace(' ', '_'),
                'save_folder': self.microscopy_folder_input.text(),
                'img_width': int(self.img_width_input.text()),
                'img_height': int(self.img_height_input.text()),
                'channels': {
                    'R': self.channel_r_check.isChecked(),
                    'G': self.channel_g_check.isChecked(),
                    'B': self.channel_b_check.isChecked()
                },
                'delay_before': float(self.delay_before_input.text()),
                'delay_after': float(self.delay_after_input.text()),
                'n_points': self._trajectory_n_points,
                'autofocus_enabled': self.autofocus_enabled_cb.isChecked(),
                'min_pixels': self.min_pixels_spin.value(),
                'max_pixels': self.max_pixels_spin.value(),
                'z_range': self.z_range_spin.value(),
                'z_tolerance': self.z_tolerance_spin.value()
            }
        except ValueError as e:
            self.log_message(f"Error en parametros: {e}")
            return
        
        # Validar carpeta
        if not config['save_folder']:
            self.log_message("Error: Selecciona una carpeta de destino")
            return
        
        os.makedirs(config['save_folder'], exist_ok=True)
        
        # Log de inicio
        self.log_message("=" * 40)
        self.log_message("INICIANDO MICROSCOPIA AUTOMATIZADA")
        self.log_message(f"   Clase: {config['class_name']}")
        self.log_message(f"   Puntos: {config['n_points']}")
        self.log_message(f"   Tamano: {config['img_width']}x{config['img_height']} px")
        channels_str = ''.join([c for c in ['R', 'G', 'B'] if config['channels'][c]])
        self.log_message(f"   Canales: {channels_str}")
        self.log_message(f"   Demoras: {config['delay_before']}s antes, {config['delay_after']}s despues")
        self.log_message(f"   Carpeta: {config['save_folder']}")
        
        # Log de autofoco si está habilitado
        if config['autofocus_enabled']:
            self.log_message(f"   🔍 Autofoco: HABILITADO")
            self.log_message(f"      Área: [{config['min_pixels']}, {config['max_pixels']}] px")
            self.log_message(f"      Rango Z: {config['z_range']} µm, Tol: {config['z_tolerance']} µm")
            
            # Actualizar parámetros en autofocus controller
            if self.parent_gui and self.parent_gui.autofocus_controller:
                self.parent_gui.autofocus_controller.set_pixel_threshold(
                    config['min_pixels'],
                    config['max_pixels']
                )
                self.parent_gui.autofocus_controller.z_search_range = config['z_range']
                self.parent_gui.autofocus_controller.z_tolerance = config['z_tolerance']
        else:
            self.log_message(f"   📸 Autofoco: DESHABILITADO (captura normal)")
        
        self.log_message("=" * 40)
        
        # Actualizar UI
        self.microscopy_start_btn.setEnabled(False)
        self.microscopy_stop_btn.setEnabled(True)
        self._microscopy_image_counter = 0
        self.set_microscopy_progress(0, config['n_points'])
        
        # Emitir señal con configuración
        self.microscopy_start_requested.emit(config)
    
    def _stop_microscopy(self):
        """Detiene la microscopía automatizada."""
        self.log_message("⏹️ DETENIENDO MICROSCOPÍA...")
        self.microscopy_start_btn.setEnabled(True)
        self.microscopy_stop_btn.setEnabled(False)
        self.microscopy_stop_requested.emit()
    
    # === Métodos para actualizar estado desde el padre ===
    
    def set_connected(self, connected: bool, info: str = ""):
        """Actualiza UI cuando cambia estado de conexión."""
        if connected:
            self.camera_info_label.setText(f"Estado: Conectada - {info}")
            self.camera_info_label.setStyleSheet("color: #27AE60; font-weight: bold;")
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.view_btn.setEnabled(True)
            self.start_live_btn.setEnabled(True)
            self.apply_exposure_btn.setEnabled(True)
            self.apply_fps_btn.setEnabled(True)
            self.apply_buffer_btn.setEnabled(True)
            self.capture_btn.setEnabled(True)
            self.log_message(f"✅ Cámara conectada: {info}")
        else:
            self.camera_info_label.setText("Estado: Desconectada")
            self.camera_info_label.setStyleSheet("color: #E74C3C; font-weight: bold;")
            self.connect_btn.setEnabled(self.thorlabs_available)
            self.disconnect_btn.setEnabled(False)
            self.view_btn.setEnabled(False)
            self.start_live_btn.setEnabled(False)
            self.stop_live_btn.setEnabled(False)
            self.apply_exposure_btn.setEnabled(False)
            self.apply_fps_btn.setEnabled(False)
            self.apply_buffer_btn.setEnabled(False)
            self.capture_btn.setEnabled(False)
            self.log_message("🔌 Cámara desconectada")
    
    def set_trajectory_status(self, has_trajectory: bool, n_points: int = 0):
        """Actualiza estado de trayectoria."""
        self._trajectory_n_points = n_points if has_trajectory else 0
        
        if has_trajectory:
            self.trajectory_status.setText(f"✅ Trayectoria lista: {n_points} puntos")
            self.trajectory_status.setStyleSheet("color: #27AE60; font-weight: bold;")
            self.microscopy_start_btn.setEnabled(True)
            self.log_message(f"📍 Trayectoria cargada: {n_points} puntos")
        else:
            self.trajectory_status.setText("⚪ Sin trayectoria")
            self.trajectory_status.setStyleSheet("color: #95A5A6; font-weight: bold;")
            self.microscopy_start_btn.setEnabled(False)
        
        # Actualizar estimación de almacenamiento
        self._update_storage_estimate()
    
    def set_microscopy_progress(self, current: int, total: int):
        """Actualiza progreso de microscopía."""
        self.microscopy_progress_label.setText(f"Progreso: {current} / {total} imágenes capturadas")
        
        # Cambiar color según progreso
        if current == 0:
            self.microscopy_progress_label.setStyleSheet("font-weight: bold; color: #3498DB;")
        elif current < total:
            self.microscopy_progress_label.setStyleSheet("font-weight: bold; color: #F39C12;")
        else:
            self.microscopy_progress_label.setStyleSheet("font-weight: bold; color: #27AE60;")
    
    # ============================================================
    # MÉTODOS DE LÓGICA DE CÁMARA
    # ============================================================
    
    def detect_thorlabs_camera(self):
        """Detecta cámaras Thorlabs conectadas."""
        if not self.thorlabs_available:
            self.log_message("❌ Error: pylablib no está instalado")
            QMessageBox.warning(self.parent_gui, "Error", "pylablib no está instalado")
            logger.warning("Intento de detectar cámara sin pylablib")
            return
        
        self.log_message("🔍 Buscando cámaras Thorlabs...")
        logger.info("Detectando cámaras Thorlabs...")
        self.detect_btn.setEnabled(False)
        
        try:
            cameras = Thorlabs.list_cameras_tlcam()
            
            if not cameras:
                self.log_message("⚠️ No se encontraron cámaras")
                QMessageBox.information(self.parent_gui, "Detección",
                                       "No se encontraron cámaras Thorlabs.\n\n"
                                       "Verificar:\n"
                                       "1. Conexión USB\n"
                                       "2. Drivers instalados\n"
                                       "3. Alimentación de cámara")
                logger.warning("No se encontraron cámaras")
            else:
                self.log_message(f"✅ Encontradas {len(cameras)} cámara(s)")
                for i, cam in enumerate(cameras, 1):
                    self.log_message(f"   Cámara {i}: {cam}")
                msg = f"¡Cámaras encontradas! Total: {len(cameras)}\n\n"
                for i, cam in enumerate(cameras, 1):
                    msg += f"Cámara {i}: {cam}\n"
                QMessageBox.information(self.parent_gui, "Detección Exitosa", msg)
                logger.info(f"Detectadas {len(cameras)} cámaras Thorlabs")
                
        except Exception as e:
            self.log_message(f"❌ Error detectando: {e}")
            QMessageBox.critical(self.parent_gui, "Error", f"Error detectando cámaras:\n{e}")
            logger.error(f"Error en detección: {e}")
        finally:
            self.detect_btn.setEnabled(True)
    
    def connect_camera(self):
        """Conecta con la cámara Thorlabs."""
        if not self.thorlabs_available:
            self.log_message("❌ Error: pylablib no está disponible")
            QMessageBox.warning(self.parent_gui, "Error", "pylablib no está disponible")
            return
        
        self.log_message("🔌 Conectando cámara Thorlabs...")
        logger.info("=== CONECTANDO CÁMARA THORLABS ===")
        
        # Crear worker si no existe
        if self.camera_worker is None:
            self.camera_worker = CameraWorker()
            self.camera_worker.connection_success.connect(self._on_camera_connected)
            self.camera_worker.new_frame_ready.connect(self.on_camera_frame)
            self.camera_worker.status_update.connect(self.log_message)  # Conectar status a log
            
            # Configurar buffer inicial
            try:
                buffer_size = int(self.buffer_input.text())
                self.camera_worker.buffer_size = buffer_size
                self.log_message(f"   Buffer inicial: {buffer_size} frames")
            except:
                self.camera_worker.buffer_size = 2
        
        # Conectar
        self.camera_worker.connect_camera()
    
    def _on_camera_connected(self, success: bool, info: str):
        """Callback cuando la cámara se conecta."""
        if success:
            self.set_connected(True, info)
            logger.info(f"Cámara conectada: {info}")
        else:
            self.log_message(f"❌ Fallo al conectar: {info}")
            QMessageBox.critical(self.parent_gui, "Error", f"Fallo al conectar:\n{info}")
            self.set_connected(False)
            logger.error(f"Fallo conexión: {info}")
    
    def disconnect_camera(self):
        """Desconecta la cámara."""
        self.log_message("🔌 Desconectando cámara...")
        logger.info("=== DESCONECTANDO CÁMARA ===")
        
        if self.camera_worker:
            self.camera_worker.disconnect_camera()
            self.camera_worker = None
        
        if self.camera_view_window:
            self.camera_view_window.close()
            self.camera_view_window = None
        
        self.set_connected(False)
        logger.info("Cámara desconectada")
    
    def open_camera_view(self):
        """Abre ventana de visualización de cámara."""
        if not self.camera_worker:
            self.log_message("❌ Error: Conecta la cámara primero")
            QMessageBox.warning(self.parent_gui, "Error", "Conecta la cámara primero")
            return
        
        if self.camera_view_window is None:
            self.camera_view_window = CameraViewWindow(self.parent_gui)
        
        self.camera_view_window.show()
        self.camera_view_window.raise_()
        self.camera_view_window.activateWindow()
        self.log_message("📹 Ventana de cámara abierta")
        logger.info("Ventana de cámara abierta")
    
    def start_camera_live_view(self):
        """Inicia vista en vivo."""
        if not self.camera_worker:
            self.log_message("❌ Error: Conecta la cámara primero")
            QMessageBox.warning(self.parent_gui, "Error", "Conecta la cámara primero")
            return
        
        # Leer parámetros
        try:
            exposure_s = float(self.exposure_input.text())
            fps = int(self.fps_input.text())
            buffer_size = int(self.buffer_input.text())
        except:
            exposure_s = 0.01
            fps = 60
            buffer_size = 2
        
        # Configurar cámara
        self.camera_worker.exposure = exposure_s
        self.camera_worker.fps = fps
        self.camera_worker.buffer_size = buffer_size
        
        self.log_message(f"▶️ Iniciando vista en vivo...")
        self.log_message(f"   Exposición: {exposure_s}s, FPS: {fps}, Buffer: {buffer_size}")
        
        # Iniciar vista en vivo (en thread)
        self.camera_worker.start()
        
        # Actualizar UI
        self.start_live_btn.setEnabled(False)
        self.stop_live_btn.setEnabled(True)
        self.capture_btn.setEnabled(True)
        
        logger.info(f"Vista en vivo iniciada: {exposure_s}s, {fps}fps, buffer={buffer_size}")
    
    def stop_camera_live_view(self):
        """Detiene vista en vivo."""
        self.log_message("⏹️ Deteniendo vista en vivo...")
        
        if self.camera_worker:
            self.camera_worker.stop_live_view()
        
        self.start_live_btn.setEnabled(True)
        self.stop_live_btn.setEnabled(False)
        self.capture_btn.setEnabled(False)
        
        self.log_message("⏹️ Vista en vivo detenida")
        logger.info("Vista en vivo detenida")
    
    def on_camera_frame(self, q_image):
        """Callback cuando llega un frame de cámara."""
        if self.camera_view_window and self.camera_view_window.isVisible():
            self.camera_view_window.update_frame(q_image)
    
    def capture_single_image(self):
        """Captura una imagen única en el formato seleccionado."""
        if not self.camera_worker:
            self.log_message("❌ Error: Cámara no conectada")
            QMessageBox.warning(self.parent_gui, "Error", "Cámara no conectada")
            return
        
        # Usar carpeta configurada
        folder = self.save_folder_input.text()
        if not folder:
            folder = QFileDialog.getExistingDirectory(self.parent_gui, "Seleccionar Carpeta")
            if folder:
                self.save_folder_input.setText(folder)
        
        if folder:
            # Crear carpeta si no existe
            os.makedirs(folder, exist_ok=True)
            
            # Capturar usando el frame actual del buffer
            if self.camera_worker.current_frame is not None:
                # Obtener formato seleccionado
                img_format = self.image_format_combo.currentText().lower()
                
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(folder, f"captura_{timestamp}.{img_format}")
                
                frame = self.camera_worker.current_frame.copy()
                frame_info = f"Original: {frame.shape}, dtype={frame.dtype}"
                
                # Normalizar frame uint16 para visualización correcta
                if frame.dtype == np.uint16:
                    frame_min, frame_max = frame.min(), frame.max()
                    
                    if img_format == 'tiff':
                        # TIFF: mantener 16 bits original
                        cv2.imwrite(filename, frame)
                        self.log_message(f"   16-bit TIFF: rango [{frame_min}, {frame_max}]")
                    else:
                        # PNG/JPG: normalizar a 8 bits
                        if frame_max > frame_min:
                            frame_norm = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                        else:
                            frame_norm = np.zeros_like(frame, dtype=np.uint8)
                        
                        if img_format == 'jpg':
                            cv2.imwrite(filename, frame_norm, [cv2.IMWRITE_JPEG_QUALITY, 95])
                        else:  # png
                            cv2.imwrite(filename, frame_norm, [cv2.IMWRITE_PNG_COMPRESSION, 6])
                        
                        self.log_message(f"   Normalizado: [{frame_min}, {frame_max}] → 8-bit")
                else:
                    # Frame ya es uint8
                    if img_format == 'jpg':
                        cv2.imwrite(filename, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    elif img_format == 'png':
                        cv2.imwrite(filename, frame, [cv2.IMWRITE_PNG_COMPRESSION, 6])
                    else:
                        cv2.imwrite(filename, frame)
                
                self.log_message(f"📸 Imagen guardada: {filename}")
                self.log_message(f"   {frame_info}")
                logger.info(f"Captura guardada: {filename}")
            else:
                self.log_message("❌ Error: No hay frame disponible en buffer")
                logger.warning("No hay frame en buffer para capturar")
    
    def capture_microscopy_image(self, config: dict, image_index: int) -> bool:
        """
        Captura una imagen para microscopia automatizada.
        
        Logica de canales:
        - 1 canal seleccionado: Guarda como GRAYSCALE puro (1 canal, pequeno)
        - 2-3 canales seleccionados: Guarda como BGR (3 canales)
        
        Args:
            config: Configuracion de microscopia (class_name, save_folder, img_width, img_height, channels)
            image_index: Indice de la imagen (0 a n_points-1)
            
        Returns:
            bool: True si la captura fue exitosa
        """
        if not self.camera_worker or self.camera_worker.current_frame is None:
            self.log_message(f"Error: No hay frame disponible para imagen {image_index}")
            return False
        
        try:
            # Obtener frame actual
            frame = self.camera_worker.current_frame.copy()
            h_orig, w_orig = frame.shape[:2]
            
            # Normalizar uint16 a uint8 para PNG
            if frame.dtype == np.uint16:
                if frame.max() > 0:
                    frame = (frame / frame.max() * 255).astype(np.uint8)
                else:
                    frame = frame.astype(np.uint8)
            
            # Redimensionar si es necesario
            target_width = config.get('img_width', 1920)
            target_height = config.get('img_height', 1080)
            
            if w_orig != target_width or h_orig != target_height:
                frame = cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_LINEAR)
            
            # Procesar canales segun seleccion del usuario
            channels = config.get('channels', {'R': False, 'G': True, 'B': False})
            selected_channels = [c for c in ['R', 'G', 'B'] if channels.get(c, False)]
            n_selected = len(selected_channels)
            
            # Logica de canales (igual que backup)
            if len(frame.shape) == 2:  # Frame grayscale original
                if n_selected == 1:
                    # MONOBANDA: Guardar directamente en grayscale puro
                    pass  # frame ya esta en grayscale
                elif n_selected >= 2:
                    # Duobanda o RGB: convertir a BGR y aplicar mascara
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    
                    # Crear mascara de canales (poner en 0 los NO seleccionados)
                    if n_selected < 3:
                        new_frame = np.zeros_like(frame)
                        if channels.get('B', False):
                            new_frame[:, :, 0] = frame[:, :, 0]
                        if channels.get('G', False):
                            new_frame[:, :, 1] = frame[:, :, 1]
                        if channels.get('R', False):
                            new_frame[:, :, 2] = frame[:, :, 2]
                        frame = new_frame
            
            elif len(frame.shape) == 3:  # Frame ya es color (BGR)
                if n_selected == 1:
                    # MONOBANDA desde imagen color: extraer solo ese canal
                    channel_map = {'B': 0, 'G': 1, 'R': 2}
                    channel_idx = channel_map[selected_channels[0]]
                    frame = frame[:, :, channel_idx]  # Convertir a grayscale
                else:
                    # Duobanda o RGB: aplicar mascara
                    if n_selected < 3:
                        new_frame = np.zeros_like(frame)
                        if channels.get('B', False):
                            new_frame[:, :, 0] = frame[:, :, 0]
                        if channels.get('G', False):
                            new_frame[:, :, 1] = frame[:, :, 1]
                        if channels.get('R', False):
                            new_frame[:, :, 2] = frame[:, :, 2]
                        frame = new_frame
            
            # Generar nombre de archivo: NombreClase_XXXXX.png
            class_name = config.get('class_name', 'Imagen')
            save_folder = config.get('save_folder', '.')
            
            # Formato PNG por defecto (5 digitos: 00000-99999)
            filename = f"{class_name}_{image_index:05d}.png"
            filepath = os.path.join(save_folder, filename)
            
            # Guardar imagen
            success = cv2.imwrite(filepath, frame, [cv2.IMWRITE_PNG_COMPRESSION, 6])
            
            if not success:
                self.log_message(f"Error: cv2.imwrite fallo para {filename}")
                return False
            
            # Calcular tamano del archivo
            file_size_kb = os.path.getsize(filepath) / 1024
            channels_str = ''.join(selected_channels)
            self.log_message(f"[{image_index+1}] {filename} ({channels_str}, {file_size_kb:.0f} KB)")
            logger.info(f"Microscopía: {filepath}")
            
            return True
            
        except Exception as e:
            self.log_message(f"❌ Error capturando imagen {image_index}: {e}")
            logger.error(f"Error en capture_microscopy_image: {e}")
            return False
    
    # === Métodos de Autofoco C-Focus ===
    
    def _test_detection(self):
        """Muestra visualización de detección de objetos en tiempo real."""
        if not self.camera_worker or self.camera_worker.current_frame is None:
            self.log_message("⚠️ No hay frame disponible. Inicia vista en vivo primero.")
            return
        
        if not self.parent_gui or not hasattr(self.parent_gui, 'focus_scorer'):
            self.log_message("⚠️ SmartFocusScorer no inicializado")
            return
        
        try:
            # Obtener frame actual
            frame = self.camera_worker.current_frame.copy()
            
            # Actualizar parámetros del scorer
            scorer = self.parent_gui.focus_scorer
            scorer.min_object_area = self.min_pixels_spin.value()
            scorer.min_probability = 0.3
            
            # Detectar objetos con visualización
            objects, vis_image = scorer.detect_objects_with_visualization(frame)
            
            # Mostrar en ventana separada
            cv2.imshow("Deteccion de Objetos - Debug", vis_image)
            cv2.waitKey(1)
            
            # Log resultados
            self.log_message(f"🔍 Test Detección: {len(objects)} objetos detectados")
            for i, obj in enumerate(objects[:3]):
                self.log_message(f"  Obj{i}: {obj['area']}px, prob={obj['probability']:.2f}")
            
            if len(objects) == 0:
                self.log_message("⚠️ Sin objetos detectados. Ajusta 'Área mínima' o mejora iluminación/contraste")
            
        except Exception as e:
            self.log_message(f"❌ Error en test detección: {e}")
            logger.error(f"Error test detección: {e}", exc_info=True)
    
    def _connect_cfocus(self):
        """Conecta el piezo C-Focus."""
        if self.parent_gui:
            success = self.parent_gui.connect_cfocus()
            if success:
                self.cfocus_connect_btn.setEnabled(False)
                self.cfocus_disconnect_btn.setEnabled(True)
                self.cfocus_status_label.setText("C-Focus: ✅ Conectado")
                self.cfocus_status_label.setStyleSheet("color: #27AE60; font-weight: bold;")
                
                self.parent_gui.initialize_autofocus()
    
    def _disconnect_cfocus(self):
        """Desconecta el piezo C-Focus."""
        if self.parent_gui and self.parent_gui.cfocus_controller:
            self.parent_gui.disconnect_cfocus()
            self.cfocus_connect_btn.setEnabled(True)
            self.cfocus_disconnect_btn.setEnabled(False)
            self.cfocus_status_label.setText("C-Focus: No conectado")
            self.cfocus_status_label.setStyleSheet("color: #888; font-style: italic;")
            self.log_message("C-Focus desconectado")

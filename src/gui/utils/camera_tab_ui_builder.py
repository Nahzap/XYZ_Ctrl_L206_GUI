"""
Builder de UI para CameraTab.

Módulo separado que contiene los métodos de creación de secciones de UI,
reduciendo significativamente el tamaño de CameraTab.

Cada función retorna un QGroupBox configurado con todos sus widgets.
Los widgets que necesitan ser accedidos desde CameraTab se almacenan
en un diccionario 'widgets' que se pasa como parámetro.
"""

import logging
from PyQt5.QtWidgets import (QGroupBox, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QLabel, QLineEdit, QPushButton, QTextEdit, QWidget,
                             QCheckBox, QComboBox, QSpinBox, QDoubleSpinBox, QRadioButton)
from PyQt5.QtCore import Qt

logger = logging.getLogger('MotorControl_L206')


def create_connection_section(widgets: dict, thorlabs_available: bool,
                               connect_cb, disconnect_cb, detect_cb) -> QGroupBox:
    """
    Crea la sección de conexión de cámara.
    
    Args:
        widgets: Dict donde almacenar referencias a widgets
        thorlabs_available: Si pylablib está disponible
        connect_cb: Callback para conectar
        disconnect_cb: Callback para desconectar
        detect_cb: Callback para detectar cámaras
        
    Returns:
        QGroupBox configurado
    """
    group = QGroupBox("1️⃣ Conexión")
    layout = QVBoxLayout()
    
    btn_layout = QHBoxLayout()
    widgets['connect_btn'] = QPushButton("🔌 Conectar Cámara")
    widgets['connect_btn'].setStyleSheet("""
        QPushButton { font-size: 13px; font-weight: bold; padding: 8px; background-color: #27AE60; }
        QPushButton:hover { background-color: #2ECC71; }
        QPushButton:disabled { background-color: #505050; color: #808080; }
    """)
    widgets['connect_btn'].clicked.connect(connect_cb)
    
    widgets['disconnect_btn'] = QPushButton("🔌 Desconectar")
    widgets['disconnect_btn'].setEnabled(False)
    widgets['disconnect_btn'].clicked.connect(disconnect_cb)
    
    widgets['detect_btn'] = QPushButton("🔍 Detectar Cámaras")
    widgets['detect_btn'].clicked.connect(detect_cb)
    
    if not thorlabs_available:
        widgets['connect_btn'].setEnabled(False)
        widgets['connect_btn'].setText("⚠️ pylablib no instalado")
        widgets['detect_btn'].setEnabled(False)
    
    btn_layout.addWidget(widgets['connect_btn'])
    btn_layout.addWidget(widgets['disconnect_btn'])
    btn_layout.addWidget(widgets['detect_btn'])
    btn_layout.addStretch()
    layout.addLayout(btn_layout)
    
    widgets['camera_info_label'] = QLabel("Estado: Desconectada")
    widgets['camera_info_label'].setStyleSheet("color: #E74C3C; font-weight: bold;")
    layout.addWidget(widgets['camera_info_label'])
    
    group.setLayout(layout)
    return group


def create_live_view_section(widgets: dict, view_cb, start_cb, stop_cb) -> QGroupBox:
    """
    Crea la sección de vista en vivo.
    
    Args:
        widgets: Dict donde almacenar referencias a widgets
        view_cb: Callback para abrir vista
        start_cb: Callback para iniciar live
        stop_cb: Callback para detener live
        
    Returns:
        QGroupBox configurado
    """
    group = QGroupBox("2️⃣ Vista en Vivo")
    layout = QVBoxLayout()
    
    btn_layout = QHBoxLayout()
    widgets['view_btn'] = QPushButton("📹 Ver Cámara")
    widgets['view_btn'].setStyleSheet("""
        QPushButton { font-size: 13px; font-weight: bold; padding: 8px; background-color: #2E86C1; }
        QPushButton:hover { background-color: #3498DB; }
        QPushButton:disabled { background-color: #505050; color: #808080; }
    """)
    widgets['view_btn'].setEnabled(False)
    widgets['view_btn'].clicked.connect(view_cb)
    
    widgets['start_live_btn'] = QPushButton("▶️ Iniciar")
    widgets['start_live_btn'].setEnabled(False)
    widgets['start_live_btn'].clicked.connect(start_cb)
    
    widgets['stop_live_btn'] = QPushButton("⏹️ Detener")
    widgets['stop_live_btn'].setEnabled(False)
    widgets['stop_live_btn'].clicked.connect(stop_cb)
    
    btn_layout.addWidget(widgets['view_btn'])
    btn_layout.addWidget(widgets['start_live_btn'])
    btn_layout.addWidget(widgets['stop_live_btn'])
    btn_layout.addStretch()
    layout.addLayout(btn_layout)
    
    group.setLayout(layout)
    return group


def create_config_section(widgets: dict, apply_exposure_cb, apply_fps_cb, apply_buffer_cb) -> QGroupBox:
    """
    Crea la sección de configuración de cámara.
    
    Args:
        widgets: Dict donde almacenar referencias a widgets
        apply_exposure_cb: Callback para aplicar exposición
        apply_fps_cb: Callback para aplicar FPS
        apply_buffer_cb: Callback para aplicar buffer
        
    Returns:
        QGroupBox configurado
    """
    group = QGroupBox("3️⃣ Configuración")
    layout = QGridLayout()
    
    # Exposición
    layout.addWidget(QLabel("Exposición (s):"), 0, 0)
    widgets['exposure_input'] = QLineEdit("0.015")
    widgets['exposure_input'].setFixedWidth(100)
    layout.addWidget(widgets['exposure_input'], 0, 1)
    
    widgets['apply_exposure_btn'] = QPushButton("✓ Aplicar")
    widgets['apply_exposure_btn'].setEnabled(False)
    widgets['apply_exposure_btn'].setFixedWidth(80)
    widgets['apply_exposure_btn'].clicked.connect(apply_exposure_cb)
    layout.addWidget(widgets['apply_exposure_btn'], 0, 2)
    
    # FPS
    layout.addWidget(QLabel("FPS:"), 1, 0)
    widgets['fps_input'] = QLineEdit("30")
    widgets['fps_input'].setFixedWidth(100)
    layout.addWidget(widgets['fps_input'], 1, 1)
    
    widgets['apply_fps_btn'] = QPushButton("✓ Aplicar")
    widgets['apply_fps_btn'].setEnabled(False)
    widgets['apply_fps_btn'].setFixedWidth(80)
    widgets['apply_fps_btn'].clicked.connect(apply_fps_cb)
    layout.addWidget(widgets['apply_fps_btn'], 1, 2)
    
    # Buffer de imágenes
    layout.addWidget(QLabel("Buffer (frames):"), 2, 0)
    widgets['buffer_input'] = QLineEdit("1")
    widgets['buffer_input'].setFixedWidth(100)
    widgets['buffer_input'].setToolTip("Número de frames en buffer (1-10). Usar 2 para estabilidad.")
    layout.addWidget(widgets['buffer_input'], 2, 1)
    
    widgets['apply_buffer_btn'] = QPushButton("✓ Aplicar")
    widgets['apply_buffer_btn'].setEnabled(False)
    widgets['apply_buffer_btn'].setFixedWidth(80)
    widgets['apply_buffer_btn'].clicked.connect(apply_buffer_cb)
    layout.addWidget(widgets['apply_buffer_btn'], 2, 2)
    
    # Info de buffer
    buffer_info = QLabel("ℹ️ Buffer=2 recomendado: visualiza frame actual, guarda el anterior")
    buffer_info.setStyleSheet("color: #888888; font-size: 10px;")
    layout.addWidget(buffer_info, 3, 0, 1, 3)
    
    group.setLayout(layout)
    return group


def create_capture_section(widgets: dict, browse_cb, capture_cb, focus_cb) -> QGroupBox:
    """
    Crea la sección de captura de imágenes.
    
    Args:
        widgets: Dict donde almacenar referencias a widgets
        browse_cb: Callback para explorar carpeta
        capture_cb: Callback para capturar imagen
        focus_cb: Callback para enfocar objetos
        
    Returns:
        QGroupBox configurado
    """
    group = QGroupBox("4️⃣ Captura de Imágenes")
    layout = QVBoxLayout()
    
    # Carpeta
    folder_layout = QHBoxLayout()
    folder_layout.addWidget(QLabel("Carpeta:"))
    widgets['save_folder_input'] = QLineEdit(r"C:\CapturasCamara")
    folder_layout.addWidget(widgets['save_folder_input'])
    
    browse_btn = QPushButton("📁 Explorar")
    browse_btn.clicked.connect(browse_cb)
    folder_layout.addWidget(browse_btn)
    layout.addLayout(folder_layout)
    
    # Formato de imagen y profundidad de bits
    format_layout = QHBoxLayout()
    format_layout.addWidget(QLabel("Formato:"))
    widgets['image_format_combo'] = QComboBox()
    widgets['image_format_combo'].addItems(["PNG", "TIFF", "JPG"])
    widgets['image_format_combo'].setCurrentText("PNG")
    widgets['image_format_combo'].setFixedWidth(80)
    widgets['image_format_combo'].setToolTip("Formato de imagen para capturas")
    format_layout.addWidget(widgets['image_format_combo'])
    
    # Checkbox para 16-bit
    widgets['use_16bit_check'] = QCheckBox("16-bit")
    widgets['use_16bit_check'].setChecked(True)  # Por defecto 16-bit para máxima calidad
    widgets['use_16bit_check'].setToolTip("Activar para guardar imágenes en 16-bit (máxima resolución).\nDesactivar para 8-bit (archivos más pequeños).\nNota: JPG solo soporta 8-bit.")
    format_layout.addWidget(widgets['use_16bit_check'])
    
    format_layout.addStretch()
    layout.addLayout(format_layout)
    
    # === Método de Captura (Volumetría) ===
    method_group = QGroupBox("Método de Captura")
    method_group.setStyleSheet("QGroupBox { font-weight: bold; }")
    method_layout = QVBoxLayout()
    
    # Radio buttons para selección de método
    widgets['capture_simple_radio'] = QRadioButton("Captura Simple (1 imagen)")
    widgets['capture_simple_radio'].setChecked(True)
    widgets['capture_simple_radio'].setToolTip("Captura una sola imagen del frame actual")
    method_layout.addWidget(widgets['capture_simple_radio'])
    
    widgets['capture_volumetry_radio'] = QRadioButton("Volumetría (múltiples planos Z)")
    widgets['capture_volumetry_radio'].setToolTip("Detecta objeto, encuentra BPoF, y captura X imágenes\nen diferentes planos Z para análisis volumétrico")
    method_layout.addWidget(widgets['capture_volumetry_radio'])
    
    # Parámetros de volumetría (inicialmente ocultos)
    volumetry_params = QWidget()
    volumetry_params_layout = QGridLayout()
    volumetry_params_layout.setContentsMargins(20, 5, 5, 5)
    
    # Número de imágenes
    volumetry_params_layout.addWidget(QLabel("Imágenes:"), 0, 0)
    widgets['volumetry_n_images_spin'] = QSpinBox()
    widgets['volumetry_n_images_spin'].setRange(3, 50)
    widgets['volumetry_n_images_spin'].setValue(10)
    widgets['volumetry_n_images_spin'].setToolTip("Número total de imágenes a capturar en el rango Z")
    widgets['volumetry_n_images_spin'].setFixedWidth(70)
    volumetry_params_layout.addWidget(widgets['volumetry_n_images_spin'], 0, 1)
    
    # Paso Z para el scan
    volumetry_params_layout.addWidget(QLabel("Paso Z (µm):"), 0, 2)
    widgets['volumetry_z_step_spin'] = QDoubleSpinBox()
    widgets['volumetry_z_step_spin'].setRange(0.01, 10.0)
    widgets['volumetry_z_step_spin'].setValue(1.0)
    widgets['volumetry_z_step_spin'].setDecimals(2)
    widgets['volumetry_z_step_spin'].setSingleStep(0.1)
    widgets['volumetry_z_step_spin'].setToolTip("Resolución del Z-scan (C-Focus soporta hasta 0.01µm)")
    widgets['volumetry_z_step_spin'].setFixedWidth(70)
    volumetry_params_layout.addWidget(widgets['volumetry_z_step_spin'], 0, 3)
    
    # Distribución (segunda fila)
    volumetry_params_layout.addWidget(QLabel("Distribución:"), 1, 0)
    widgets['volumetry_distribution_combo'] = QComboBox()
    widgets['volumetry_distribution_combo'].addItems(["Uniforme", "Centrada (más cerca BPoF)"])
    widgets['volumetry_distribution_combo'].setToolTip("Uniforme: espaciado igual entre imágenes\nCentrada: más imágenes cerca del BPoF")
    widgets['volumetry_distribution_combo'].setFixedWidth(150)
    volumetry_params_layout.addWidget(widgets['volumetry_distribution_combo'], 1, 1, 1, 2)
    
    # Checkboxes (tercera fila)
    widgets['volumetry_include_bpof_check'] = QCheckBox("Incluir BPoF exacto")
    widgets['volumetry_include_bpof_check'].setChecked(True)
    widgets['volumetry_include_bpof_check'].setToolTip("Asegura que una imagen se capture exactamente en el BPoF")
    volumetry_params_layout.addWidget(widgets['volumetry_include_bpof_check'], 2, 0, 1, 2)
    
    widgets['volumetry_save_json_check'] = QCheckBox("Guardar JSON con metadatos")
    widgets['volumetry_save_json_check'].setChecked(True)
    widgets['volumetry_save_json_check'].setToolTip("Guarda archivo JSON con información de Z, scores y parámetros")
    volumetry_params_layout.addWidget(widgets['volumetry_save_json_check'], 2, 2, 1, 2)
    
    volumetry_params.setLayout(volumetry_params_layout)
    widgets['volumetry_params_widget'] = volumetry_params
    volumetry_params.setVisible(False)  # Oculto por defecto
    method_layout.addWidget(volumetry_params)
    
    # Conectar radio button para mostrar/ocultar parámetros
    widgets['capture_volumetry_radio'].toggled.connect(
        lambda checked: volumetry_params.setVisible(checked)
    )
    
    method_group.setLayout(method_layout)
    layout.addWidget(method_group)
    
    # Botones de captura
    btn_layout = QHBoxLayout()
    widgets['capture_btn'] = QPushButton("📸 Capturar Imagen")
    widgets['capture_btn'].setStyleSheet("""
        QPushButton { font-size: 14px; font-weight: bold; padding: 10px; background-color: #E67E22; }
        QPushButton:hover { background-color: #F39C12; }
        QPushButton:disabled { background-color: #505050; color: #808080; }
    """)
    widgets['capture_btn'].setEnabled(False)
    widgets['capture_btn'].clicked.connect(capture_cb)
    btn_layout.addWidget(widgets['capture_btn'])
    
    widgets['focus_btn'] = QPushButton("🎯 Enfocar Objs")
    widgets['focus_btn'].setStyleSheet("""
        QPushButton { font-size: 14px; font-weight: bold; padding: 10px; background-color: #9B59B6; }
        QPushButton:hover { background-color: #8E44AD; }
        QPushButton:disabled { background-color: #505050; color: #808080; }
    """)
    widgets['focus_btn'].setEnabled(False)
    widgets['focus_btn'].clicked.connect(focus_cb)
    btn_layout.addWidget(widgets['focus_btn'])
    btn_layout.addStretch()
    layout.addLayout(btn_layout)
    
    group.setLayout(layout)
    return group


def create_microscopy_section(widgets: dict, refresh_traj_cb, start_cb, stop_cb, 
                               browse_folder_cb, update_estimate_cb) -> QGroupBox:
    """
    Crea la sección de microscopía automatizada.
    
    Args:
        widgets: Dict donde almacenar referencias a widgets
        refresh_traj_cb: Callback para actualizar trayectoria
        start_cb: Callback para iniciar microscopía
        stop_cb: Callback para detener microscopía
        browse_folder_cb: Callback para explorar carpeta
        update_estimate_cb: Callback para actualizar estimación de almacenamiento
        
    Returns:
        QGroupBox configurado
    """
    group = QGroupBox("🔬 Microscopía Automatizada")
    layout = QVBoxLayout()
    
    # Info
    info_label = QLabel(
        "ℹ️ <b>Ejecuta la trayectoria zig-zag con captura automática de imágenes</b><br>"
        "Usa la trayectoria generada en la pestaña 'Prueba' y captura una imagen en cada punto."
    )
    info_label.setWordWrap(True)
    info_label.setStyleSheet("padding: 8px; background-color: #34495E; border-radius: 5px;")
    layout.addWidget(info_label)
    
    # Estado de trayectoria
    traj_layout = QHBoxLayout()
    traj_layout.addWidget(QLabel("<b>Estado:</b>"))
    widgets['trajectory_status'] = QLabel("⚪ Sin trayectoria")
    widgets['trajectory_status'].setStyleSheet("color: #95A5A6; font-weight: bold;")
    traj_layout.addWidget(widgets['trajectory_status'])
    
    refresh_traj_btn = QPushButton("🔄 Actualizar")
    refresh_traj_btn.setFixedWidth(100)
    refresh_traj_btn.setToolTip("Sincronizar trayectoria desde pestaña Prueba")
    refresh_traj_btn.clicked.connect(refresh_traj_cb)
    traj_layout.addWidget(refresh_traj_btn)
    traj_layout.addStretch()
    layout.addLayout(traj_layout)
    
    # Fila 1: Nombre de clase + Tamaño imagen
    row1_layout = QHBoxLayout()
    row1_layout.addWidget(QLabel("Nombre clase:"))
    widgets['class_name_input'] = QLineEdit("Especie_001")
    widgets['class_name_input'].setFixedWidth(150)
    widgets['class_name_input'].setPlaceholderText("Ej: Rosa_Canina")
    widgets['class_name_input'].textChanged.connect(update_estimate_cb)
    row1_layout.addWidget(widgets['class_name_input'])
    
    row1_layout.addSpacing(20)
    row1_layout.addWidget(QLabel("Tamaño imagen (px):"))
    widgets['img_width_input'] = QLineEdit("1920")
    widgets['img_width_input'].setFixedWidth(60)
    widgets['img_width_input'].textChanged.connect(update_estimate_cb)
    row1_layout.addWidget(widgets['img_width_input'])
    row1_layout.addWidget(QLabel("×"))
    widgets['img_height_input'] = QLineEdit("1080")
    widgets['img_height_input'].setFixedWidth(60)
    widgets['img_height_input'].textChanged.connect(update_estimate_cb)
    row1_layout.addWidget(widgets['img_height_input'])
    row1_layout.addStretch()
    layout.addLayout(row1_layout)
    
    # Fila 2: Canales RGB + Estimación
    row2_layout = QHBoxLayout()
    row2_layout.addWidget(QLabel("Canales RGB:"))
    widgets['channel_r_check'] = QCheckBox("R")
    widgets['channel_r_check'].setStyleSheet("color: #E74C3C; font-weight: bold;")
    widgets['channel_r_check'].stateChanged.connect(update_estimate_cb)
    row2_layout.addWidget(widgets['channel_r_check'])
    
    widgets['channel_g_check'] = QCheckBox("G")
    widgets['channel_g_check'].setStyleSheet("color: #27AE60; font-weight: bold;")
    widgets['channel_g_check'].setChecked(True)
    widgets['channel_g_check'].stateChanged.connect(update_estimate_cb)
    row2_layout.addWidget(widgets['channel_g_check'])
    
    widgets['channel_b_check'] = QCheckBox("B")
    widgets['channel_b_check'].setStyleSheet("color: #3498DB; font-weight: bold;")
    widgets['channel_b_check'].stateChanged.connect(update_estimate_cb)
    row2_layout.addWidget(widgets['channel_b_check'])
    
    row2_layout.addSpacing(30)
    row2_layout.addWidget(QLabel("Estimación:"))
    widgets['storage_estimate_label'] = QLabel("~0 MB")
    widgets['storage_estimate_label'].setStyleSheet("font-weight: bold; color: #F39C12;")
    row2_layout.addWidget(widgets['storage_estimate_label'])
    row2_layout.addStretch()
    layout.addLayout(row2_layout)
    
    # Fila 3: Carpeta de destino
    folder_layout = QHBoxLayout()
    folder_layout.addWidget(QLabel("Carpeta destino:"))
    widgets['microscopy_folder_input'] = QLineEdit("C:\\MicroscopyData")
    widgets['microscopy_folder_input'].setMinimumWidth(300)
    widgets['microscopy_folder_input'].setToolTip("Carpeta donde se guardarán las imágenes")
    folder_layout.addWidget(widgets['microscopy_folder_input'])
    
    browse_btn = QPushButton("Explorar")
    browse_btn.clicked.connect(browse_folder_cb)
    folder_layout.addWidget(browse_btn)
    folder_layout.addStretch()
    layout.addLayout(folder_layout)
    
    # Fila 4: Demoras
    row3_layout = QHBoxLayout()
    row3_layout.addWidget(QLabel("Demora antes (s):"))
    widgets['delay_before_input'] = QLineEdit("2.0")
    widgets['delay_before_input'].setFixedWidth(60)
    widgets['delay_before_input'].setToolTip("Tiempo de espera antes de capturar (estabilización)")
    row3_layout.addWidget(widgets['delay_before_input'])
    
    row3_layout.addSpacing(30)
    row3_layout.addWidget(QLabel("Demora después (s):"))
    widgets['delay_after_input'] = QLineEdit("0.2")
    widgets['delay_after_input'].setFixedWidth(60)
    widgets['delay_after_input'].setToolTip("Tiempo de espera después de capturar")
    row3_layout.addWidget(widgets['delay_after_input'])
    row3_layout.addStretch()
    layout.addLayout(row3_layout)
    
    # Botones de microscopía
    btn_layout = QHBoxLayout()
    widgets['microscopy_start_btn'] = QPushButton("🚀 Iniciar Microscopía")
    widgets['microscopy_start_btn'].setStyleSheet("""
        QPushButton { font-size: 13px; font-weight: bold; padding: 10px; background-color: #27AE60; }
        QPushButton:hover { background-color: #2ECC71; }
        QPushButton:disabled { background-color: #505050; color: #808080; }
    """)
    widgets['microscopy_start_btn'].setEnabled(False)
    widgets['microscopy_start_btn'].clicked.connect(start_cb)
    
    widgets['microscopy_stop_btn'] = QPushButton("⏹️ Detener")
    widgets['microscopy_stop_btn'].setStyleSheet("background-color: #E74C3C; font-weight: bold; padding: 10px;")
    widgets['microscopy_stop_btn'].setEnabled(False)
    widgets['microscopy_stop_btn'].clicked.connect(stop_cb)
    
    btn_layout.addWidget(widgets['microscopy_start_btn'])
    btn_layout.addWidget(widgets['microscopy_stop_btn'])
    btn_layout.addStretch()
    layout.addLayout(btn_layout)
    
    # Progreso
    widgets['microscopy_progress_label'] = QLabel("Progreso: 0 / 0 imágenes capturadas")
    widgets['microscopy_progress_label'].setStyleSheet("font-weight: bold; color: #3498DB;")
    layout.addWidget(widgets['microscopy_progress_label'])
    
    group.setLayout(layout)
    return group


def create_autofocus_section(widgets: dict, connect_cb, disconnect_cb, 
                              test_detection_cb, update_params_cb) -> QGroupBox:
    """
    Crea la sección de autofoco multi-objeto (C-Focus).
    
    Args:
        widgets: Dict donde almacenar referencias a widgets
        connect_cb: Callback para conectar C-Focus
        disconnect_cb: Callback para desconectar C-Focus
        test_detection_cb: Callback para test de detección
        update_params_cb: Callback para actualizar parámetros
        
    Returns:
        QGroupBox configurado
    """
    group = QGroupBox("🔍 Autofoco Multi-Objeto (C-Focus)")
    layout = QVBoxLayout()
    
    # Checkbox para habilitar autofoco
    widgets['autofocus_enabled_cb'] = QCheckBox("Habilitar autofoco por objeto")
    widgets['autofocus_enabled_cb'].setToolTip(
        "Pre-detecta objetos con U2-Net y captura una imagen enfocada por cada uno.\n"
        "Genera N imágenes por punto, donde N = objetos detectados."
    )
    layout.addWidget(widgets['autofocus_enabled_cb'])
    
    # Botones de conexión C-Focus
    btn_layout = QHBoxLayout()
    widgets['cfocus_connect_btn'] = QPushButton("🔌 Conectar C-Focus")
    widgets['cfocus_connect_btn'].setStyleSheet("""
        QPushButton { font-size: 12px; font-weight: bold; padding: 6px; background-color: #8E44AD; }
        QPushButton:hover { background-color: #9B59B6; }
        QPushButton:disabled { background-color: #505050; color: #808080; }
    """)
    widgets['cfocus_connect_btn'].clicked.connect(connect_cb)
    btn_layout.addWidget(widgets['cfocus_connect_btn'])
    
    widgets['cfocus_disconnect_btn'] = QPushButton("⏹️ Desconectar")
    widgets['cfocus_disconnect_btn'].setEnabled(False)
    widgets['cfocus_disconnect_btn'].clicked.connect(disconnect_cb)
    btn_layout.addWidget(widgets['cfocus_disconnect_btn'])
    
    widgets['test_detection_btn'] = QPushButton("🔍 Test Detección")
    widgets['test_detection_btn'].setToolTip("Muestra visualización de detección de objetos en tiempo real")
    widgets['test_detection_btn'].clicked.connect(test_detection_cb)
    btn_layout.addWidget(widgets['test_detection_btn'])
    
    btn_layout.addStretch()
    layout.addLayout(btn_layout)
    
    # Modo de autofoco
    mode_layout = QHBoxLayout()
    mode_layout.addWidget(QLabel("Modo Z-scan:"))
    
    widgets['full_scan_cb'] = QCheckBox("Escaneo Completo (0-100µm)")
    widgets['full_scan_cb'].setChecked(True)
    widgets['full_scan_cb'].setToolTip(
        "Escanea todo el rango Z evaluando índice S para encontrar BPoF.\n"
        "Más lento pero más preciso. Desmarcar para Golden Section Search."
    )
    mode_layout.addWidget(widgets['full_scan_cb'])
    mode_layout.addStretch()
    layout.addLayout(mode_layout)
    
    # Parámetros de detección
    detection_form = QGridLayout()
    
    detection_form.addWidget(QLabel("Área mínima:"), 0, 0)
    widgets['min_pixels_spin'] = QSpinBox()
    widgets['min_pixels_spin'].setRange(10, 100000)
    widgets['min_pixels_spin'].setValue(100)
    widgets['min_pixels_spin'].setSuffix(" px")
    widgets['min_pixels_spin'].setToolTip("Área mínima del objeto en píxeles")
    widgets['min_pixels_spin'].setFixedWidth(100)
    widgets['min_pixels_spin'].valueChanged.connect(update_params_cb)
    detection_form.addWidget(widgets['min_pixels_spin'], 0, 1)
    
    detection_form.addWidget(QLabel("Área máxima:"), 0, 2)
    widgets['max_pixels_spin'] = QSpinBox()
    widgets['max_pixels_spin'].setRange(100, 500000)
    widgets['max_pixels_spin'].setValue(50000)
    widgets['max_pixels_spin'].setSuffix(" px")
    widgets['max_pixels_spin'].setToolTip("Área máxima del objeto en píxeles")
    widgets['max_pixels_spin'].setFixedWidth(100)
    widgets['max_pixels_spin'].valueChanged.connect(update_params_cb)
    detection_form.addWidget(widgets['max_pixels_spin'], 0, 3)
    
    # Circularidad mínima
    detection_form.addWidget(QLabel("Circularidad mín:"), 1, 0)
    widgets['circularity_spin'] = QDoubleSpinBox()
    widgets['circularity_spin'].setRange(0.0, 1.0)
    widgets['circularity_spin'].setSingleStep(0.05)
    widgets['circularity_spin'].setValue(0.35)
    widgets['circularity_spin'].setDecimals(2)
    widgets['circularity_spin'].setToolTip("Circularidad mínima (0-1). 1=círculo perfecto. REDUCIR para muestras borrosas.")
    widgets['circularity_spin'].setFixedWidth(100)
    widgets['circularity_spin'].valueChanged.connect(update_params_cb)
    detection_form.addWidget(widgets['circularity_spin'], 1, 1)
    
    # Aspect ratio mínimo
    detection_form.addWidget(QLabel("Aspect ratio mín:"), 1, 2)
    widgets['aspect_ratio_spin'] = QDoubleSpinBox()
    widgets['aspect_ratio_spin'].setRange(0.0, 1.0)
    widgets['aspect_ratio_spin'].setSingleStep(0.05)
    widgets['aspect_ratio_spin'].setValue(0.40)
    widgets['aspect_ratio_spin'].setDecimals(2)
    widgets['aspect_ratio_spin'].setToolTip("Aspect ratio mínimo (0-1). Rechaza objetos muy alargados.")
    widgets['aspect_ratio_spin'].setFixedWidth(100)
    widgets['aspect_ratio_spin'].valueChanged.connect(update_params_cb)
    detection_form.addWidget(widgets['aspect_ratio_spin'], 1, 3)
    
    # Parámetros de búsqueda Z
    detection_form.addWidget(QLabel("Rango Z:"), 2, 0)
    widgets['z_range_spin'] = QDoubleSpinBox()
    widgets['z_range_spin'].setRange(5.0, 200.0)
    widgets['z_range_spin'].setValue(50.0)
    widgets['z_range_spin'].setSuffix(" µm")
    widgets['z_range_spin'].setToolTip("Rango total de búsqueda de foco")
    widgets['z_range_spin'].setFixedWidth(100)
    detection_form.addWidget(widgets['z_range_spin'], 2, 1)
    
    detection_form.addWidget(QLabel("Tolerancia:"), 2, 2)
    widgets['z_tolerance_spin'] = QDoubleSpinBox()
    widgets['z_tolerance_spin'].setRange(0.1, 5.0)
    widgets['z_tolerance_spin'].setValue(0.5)
    widgets['z_tolerance_spin'].setSuffix(" µm")
    widgets['z_tolerance_spin'].setDecimals(2)
    widgets['z_tolerance_spin'].setToolTip("Tolerancia de convergencia")
    widgets['z_tolerance_spin'].setFixedWidth(100)
    detection_form.addWidget(widgets['z_tolerance_spin'], 2, 3)
    
    layout.addLayout(detection_form)
    
    # Label de estado C-Focus
    widgets['cfocus_status_label'] = QLabel("C-Focus: No conectado")
    widgets['cfocus_status_label'].setStyleSheet("color: #888; font-style: italic;")
    layout.addWidget(widgets['cfocus_status_label'])
    
    group.setLayout(layout)
    return group


def create_log_section(widgets: dict, clear_cb) -> QGroupBox:
    """
    Crea la sección de terminal de log.
    
    Args:
        widgets: Dict donde almacenar referencias a widgets
        clear_cb: Callback para limpiar log
        
    Returns:
        QGroupBox configurado
    """
    group = QGroupBox("📋 Log de Cámara")
    layout = QVBoxLayout()
    
    widgets['camera_terminal'] = QTextEdit()
    widgets['camera_terminal'].setReadOnly(True)
    widgets['camera_terminal'].setMaximumHeight(150)
    widgets['camera_terminal'].setStyleSheet("""
        QTextEdit {
            background-color: #1a1a1a;
            color: #00FF00;
            font-family: 'Courier New', monospace;
            font-size: 11px;
            border: 1px solid #444444;
        }
    """)
    widgets['camera_terminal'].setPlaceholderText("Eventos de cámara aparecerán aquí...")
    layout.addWidget(widgets['camera_terminal'])
    
    clear_btn = QPushButton("🗑️ Limpiar Log")
    clear_btn.setFixedWidth(120)
    clear_btn.clicked.connect(clear_cb)
    layout.addWidget(clear_btn)
    
    group.setLayout(layout)
    return group

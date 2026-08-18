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
from PyQt5.QtCore import Qt, QLocale

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
    # acA2500-14uc: techo ~14 fps @ full frame
    widgets['fps_input'] = QLineEdit("14")
    widgets['fps_input'].setFixedWidth(100)
    layout.addWidget(widgets['fps_input'], 1, 1)
    
    widgets['apply_fps_btn'] = QPushButton("✓ Aplicar")
    widgets['apply_fps_btn'].setEnabled(False)
    widgets['apply_fps_btn'].setFixedWidth(80)
    widgets['apply_fps_btn'].clicked.connect(apply_fps_cb)
    layout.addWidget(widgets['apply_fps_btn'], 1, 2)
    
    # Buffer de imágenes
    layout.addWidget(QLabel("Buffer (frames):"), 2, 0)
    widgets['buffer_input'] = QLineEdit("2")
    widgets['buffer_input'].setFixedWidth(100)
    widgets['buffer_input'].setToolTip(
        "Frames en buffer (2 recomendado con LatestImageOnly)."
    )
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
    
    # === Método de Captura (Z-Stack) ===
    method_group = QGroupBox("Método de Captura")
    method_group.setStyleSheet("QGroupBox { font-weight: bold; }")
    method_layout = QVBoxLayout()
    
    # Radio buttons para selección de método
    widgets['capture_simple_radio'] = QRadioButton("Captura Simple (1 imagen)")
    widgets['capture_simple_radio'].setChecked(True)
    widgets['capture_simple_radio'].setToolTip("Captura una sola imagen del frame actual")
    method_layout.addWidget(widgets['capture_simple_radio'])
    
    widgets['capture_zstack_radio'] = QRadioButton("Capturar Z-Stack (múltiples planos Z)")
    widgets['capture_zstack_radio'].setToolTip("Captura múltiples imágenes en diferentes planos Z\ncontrolado por el Paso Z que tú defines")
    method_layout.addWidget(widgets['capture_zstack_radio'])
    
    # Parámetros de Z-Stack (inicialmente ocultos)
    zstack_params = QWidget()
    zstack_params_layout = QGridLayout()
    zstack_params_layout.setContentsMargins(20, 5, 5, 5)
    
    # FILA 1: Z Mínimo y Z Máximo (solo lectura desde hardware calibrado)
    zstack_params_layout.addWidget(QLabel("Z Min (µm):"), 0, 0)
    widgets['zstack_z_min_spin'] = QDoubleSpinBox()
    widgets['zstack_z_min_spin'].setRange(0.0, 200.0)
    widgets['zstack_z_min_spin'].setValue(0.0)
    widgets['zstack_z_min_spin'].setDecimals(2)
    widgets['zstack_z_min_spin'].setSingleStep(0.5)
    widgets['zstack_z_min_spin'].setToolTip("Límite mínimo del hardware (solo lectura, definido por calibración)")
    widgets['zstack_z_min_spin'].setFixedWidth(80)
    widgets['zstack_z_min_spin'].setEnabled(False)
    zstack_params_layout.addWidget(widgets['zstack_z_min_spin'], 0, 1)
    
    zstack_params_layout.addWidget(QLabel("Z Max (µm):"), 0, 2)
    widgets['zstack_z_max_spin'] = QDoubleSpinBox()
    widgets['zstack_z_max_spin'].setRange(0.0, 200.0)
    widgets['zstack_z_max_spin'].setValue(76.0)
    widgets['zstack_z_max_spin'].setDecimals(2)
    widgets['zstack_z_max_spin'].setSingleStep(0.5)
    widgets['zstack_z_max_spin'].setToolTip("Límite máximo del hardware (solo lectura, definido por calibración)")
    widgets['zstack_z_max_spin'].setFixedWidth(80)
    widgets['zstack_z_max_spin'].setEnabled(False)
    zstack_params_layout.addWidget(widgets['zstack_z_max_spin'], 0, 3)
    
    # FILA 2: Paso Z y Número de Imágenes (calculado)
    zstack_params_layout.addWidget(QLabel("Paso Z (µm):"), 1, 0)
    widgets['zstack_z_step_spin'] = QDoubleSpinBox()
    widgets['zstack_z_step_spin'].setRange(0.001, 10.0)
    widgets['zstack_z_step_spin'].setValue(0.05)
    widgets['zstack_z_step_spin'].setDecimals(3)
    widgets['zstack_z_step_spin'].setSingleStep(0.01)
    widgets['zstack_z_step_spin'].setToolTip("Paso Z entre imágenes consecutivas")
    widgets['zstack_z_step_spin'].setFixedWidth(80)
    widgets['zstack_z_step_spin'].setEnabled(True)
    zstack_params_layout.addWidget(widgets['zstack_z_step_spin'], 1, 1)
    
    zstack_params_layout.addWidget(QLabel("N° Imágenes:"), 1, 2)
    widgets['zstack_n_images_spin'] = QSpinBox()
    widgets['zstack_n_images_spin'].setRange(3, 10000)
    widgets['zstack_n_images_spin'].setValue(200)
    widgets['zstack_n_images_spin'].setToolTip("Número calculado: (Z_max - Z_min) / Paso_Z + 1")
    widgets['zstack_n_images_spin'].setFixedWidth(80)
    widgets['zstack_n_images_spin'].setEnabled(False)  # READONLY - calculado automáticamente
    widgets['zstack_n_images_spin'].setStyleSheet("QSpinBox { background-color: #2a2a2a; }")
    zstack_params_layout.addWidget(widgets['zstack_n_images_spin'], 1, 3)
    
    # Función para calcular N imágenes automáticamente
    def update_n_images():
        z_min = widgets['zstack_z_min_spin'].value()
        z_max = widgets['zstack_z_max_spin'].value()
        z_step = widgets['zstack_z_step_spin'].value()
        if z_step > 0 and z_max >= z_min:
            n_images = int((z_max - z_min) / z_step) + 1
            widgets['zstack_n_images_spin'].setValue(n_images)
    
    widgets['zstack_z_min_spin'].valueChanged.connect(update_n_images)
    widgets['zstack_z_max_spin'].valueChanged.connect(update_n_images)
    widgets['zstack_z_step_spin'].valueChanged.connect(update_n_images)
    
    # FILA 3: Indicador de rango calibrado C-Focus
    zstack_params_layout.addWidget(QLabel("Rango C-Focus:"), 2, 0)
    widgets['zstack_cfocus_range_label'] = QLabel("0.0 - 0.0 µm")
    widgets['zstack_cfocus_range_label'].setStyleSheet("color: #888; font-style: italic;")
    widgets['zstack_cfocus_range_label'].setToolTip("Rango calibrado del C-Focus (solo lectura)")
    zstack_params_layout.addWidget(widgets['zstack_cfocus_range_label'], 2, 1, 1, 3)
    
    # FILA 4: Carpeta de destino (MOSTRAR DÓNDE SE GUARDAN LOS DATOS)
    zstack_params_layout.addWidget(QLabel("Guardar en:"), 3, 0)
    widgets['zstack_save_folder_label'] = QLabel("(usar carpeta principal)")
    widgets['zstack_save_folder_label'].setStyleSheet("color: #3498db; font-style: italic;")
    widgets['zstack_save_folder_label'].setToolTip("Las imágenes Z-Stack se guardarán en la carpeta seleccionada arriba")
    zstack_params_layout.addWidget(widgets['zstack_save_folder_label'], 3, 1, 1, 3)
    
    # FILA 5: Canal monobanda (R/G/B)
    zstack_params_layout.addWidget(QLabel("Canal (mono):"), 4, 0)
    channel_layout = QHBoxLayout()
    widgets['zstack_channel_r_check'] = QCheckBox("R")
    widgets['zstack_channel_r_check'].setStyleSheet("color: #E74C3C; font-weight: bold;")
    channel_layout.addWidget(widgets['zstack_channel_r_check'])
    widgets['zstack_channel_g_check'] = QCheckBox("G")
    widgets['zstack_channel_g_check'].setStyleSheet("color: #27AE60; font-weight: bold;")
    widgets['zstack_channel_g_check'].setChecked(True)
    channel_layout.addWidget(widgets['zstack_channel_g_check'])
    widgets['zstack_channel_b_check'] = QCheckBox("B")
    widgets['zstack_channel_b_check'].setStyleSheet("color: #3498DB; font-weight: bold;")
    channel_layout.addWidget(widgets['zstack_channel_b_check'])
    channel_layout.addStretch()
    zstack_params_layout.addLayout(channel_layout, 4, 1, 1, 3)

    # FILA 6: Checkboxes
    widgets['zstack_save_json_check'] = QCheckBox("Guardar JSON con metadatos")
    widgets['zstack_save_json_check'].setChecked(True)
    widgets['zstack_save_json_check'].setToolTip("Guarda archivo JSON con información de Z, scores y parámetros")
    zstack_params_layout.addWidget(widgets['zstack_save_json_check'], 5, 0, 1, 2)

    # FILA 7: Estimación de tamaño
    zstack_params_layout.addWidget(QLabel("Tamaño aprox.:"), 6, 0)
    widgets['zstack_storage_estimate_label'] = QLabel("~0 MB")
    widgets['zstack_storage_estimate_label'].setStyleSheet("font-weight: bold; color: #F39C12;")
    widgets['zstack_storage_estimate_label'].setToolTip("Estimación sin compresión para stack monobanda 16-bit")
    zstack_params_layout.addWidget(widgets['zstack_storage_estimate_label'], 6, 1, 1, 3)
    
    zstack_params.setLayout(zstack_params_layout)
    widgets['zstack_params_widget'] = zstack_params
    zstack_params.setVisible(False)  # Oculto por defecto
    method_layout.addWidget(zstack_params)
    
    # Conectar radio button para mostrar/ocultar parámetros
    def toggle_zstack_params(checked: bool):
        zstack_params.setVisible(checked)

    widgets['capture_zstack_radio'].toggled.connect(toggle_zstack_params)
    # Estado inicial coherente con selección por defecto
    toggle_zstack_params(widgets['capture_zstack_radio'].isChecked())
    
    method_group.setLayout(method_layout)
    layout.addWidget(method_group)
    
    # Botones de captura
    btn_layout = QHBoxLayout()
    widgets['capture_btn'] = QPushButton("📸 Captura Simple")
    widgets['capture_btn'].setStyleSheet("""
        QPushButton { font-size: 14px; font-weight: bold; padding: 10px; background-color: #E67E22; }
        QPushButton:hover { background-color: #F39C12; }
        QPushButton:disabled { background-color: #505050; color: #808080; }
    """)
    widgets['capture_btn'].setEnabled(False)
    widgets['capture_btn'].clicked.connect(capture_cb)
    btn_layout.addWidget(widgets['capture_btn'])
    
    # Función para actualizar texto del botón según modo de captura
    def update_capture_btn_text(checked):
        if checked:  # Z-Stack seleccionado
            widgets['capture_btn'].setText("📸 Capturar Z-Stack")
        else:  # Captura simple
            widgets['capture_btn'].setText("📸 Captura Simple")
    
    # Conectar cambio de radio button
    widgets['capture_zstack_radio'].toggled.connect(update_capture_btn_text)
    update_capture_btn_text(widgets['capture_zstack_radio'].isChecked())
    
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
                               browse_folder_cb, update_estimate_cb,
                               open_folder_cb=None) -> QGroupBox:
    """
    Crea la sección de microscopía automatizada.
    
    Args:
        widgets: Dict donde almacenar referencias a widgets
        refresh_traj_cb: Callback para actualizar trayectoria
        start_cb: Callback para iniciar microscopía
        stop_cb: Callback para detener microscopía
        browse_folder_cb: Callback para explorar carpeta
        update_estimate_cb: Callback para actualizar estimación de almacenamiento
        open_folder_cb: Callback opcional para abrir la carpeta destino en el explorador
        
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
    
    # Modo de ejecución (XY solo vs XY + AF Z)
    mode_layout = QHBoxLayout()
    widgets['xy_only_cb'] = QCheckBox("Trayectoria XY + Auto foco Z")
    widgets['xy_only_cb'].setToolTip(
        "Activado: Sólo trayectoria XY (captura sin segmentación / sin AF Z)\n"
        "Desactivado: Trayectoria XY + Auto foco Z (detección y enfoque por objeto)"
    )
    widgets['xy_only_cb'].setChecked(False)  # Por defecto: XY + AF Z
    def _update_xy_label(checked: bool):
        widgets['xy_only_cb'].setText("Sólo trayectoria XY" if checked else "Trayectoria XY + Auto foco Z")
    widgets['xy_only_cb'].toggled.connect(_update_xy_label)
    # Inicializar texto acorde al estado inicial
    _update_xy_label(widgets['xy_only_cb'].isChecked())
    mode_layout.addWidget(widgets['xy_only_cb'])
    mode_layout.addStretch()
    layout.addLayout(mode_layout)

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

    # Continuar desde un punto (reanudación tras FOV / stop)
    resume_layout = QHBoxLayout()
    resume_layout.addWidget(QLabel("Continuar desde punto:"))
    widgets['resume_point_spin'] = QSpinBox()
    widgets['resume_point_spin'].setRange(1, 1)
    widgets['resume_point_spin'].setValue(1)
    widgets['resume_point_spin'].setFixedWidth(80)
    widgets['resume_point_spin'].setToolTip(
        "Punto 1-based de la trayectoria zig-zag.\n"
        "• 1 = empezar desde el inicio\n"
        "• Si falló el P36, pon 36 para reintentar ese punto\n"
        "• Las capturas ya guardadas (índices anteriores) no se borran"
    )
    resume_layout.addWidget(widgets['resume_point_spin'])
    widgets['resume_hint_label'] = QLabel("(1 = inicio)")
    widgets['resume_hint_label'].setStyleSheet("color: #95A5A6; font-style: italic;")
    resume_layout.addWidget(widgets['resume_hint_label'])
    resume_layout.addStretch()
    layout.addLayout(resume_layout)
    
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
    # Defaults Basler acA2500 nativo (se sobrescriben al conectar con ROI real)
    widgets['img_width_input'] = QLineEdit("2590")
    widgets['img_width_input'].setFixedWidth(60)
    widgets['img_width_input'].setToolTip(
        "Se autocompleta con la resolución real de la Basler al conectar"
    )
    widgets['img_width_input'].textChanged.connect(update_estimate_cb)
    row1_layout.addWidget(widgets['img_width_input'])
    row1_layout.addWidget(QLabel("×"))
    widgets['img_height_input'] = QLineEdit("1942")
    widgets['img_height_input'].setFixedWidth(60)
    widgets['img_height_input'].setToolTip(
        "Se autocompleta con la resolución real de la Basler al conectar"
    )
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
    widgets['microscopy_folder_input'] = QLineEdit(
        "F:/MICROSCOPIA/MIELES/APICOLA QUINCHAO/SAMPLE 001"
    )
    widgets['microscopy_folder_input'].setMinimumWidth(300)
    widgets['microscopy_folder_input'].setToolTip(
        "Carpeta exacta donde se escriben las PNG/JSON de microscopía"
    )
    folder_layout.addWidget(widgets['microscopy_folder_input'])
    
    browse_btn = QPushButton("Explorar")
    browse_btn.clicked.connect(browse_folder_cb)
    folder_layout.addWidget(browse_btn)
    open_btn = QPushButton("Abrir")
    open_btn.setToolTip("Abrir esta carpeta en el Explorador de Windows")
    if open_folder_cb is not None:
        open_btn.clicked.connect(open_folder_cb)
    else:
        open_btn.setEnabled(False)
    folder_layout.addWidget(open_btn)
    folder_layout.addStretch()
    layout.addLayout(folder_layout)
    
    # Fila 4: Demoras
    row3_layout = QHBoxLayout()
    row3_layout.addWidget(QLabel("Demora antes (s):"))
    widgets['delay_before_input'] = QLineEdit("0.3")
    widgets['delay_before_input'].setFixedWidth(60)
    widgets['delay_before_input'].setToolTip(
        "Tiempo de espera antes de capturar (estabilización XY).\n"
        "Se paga en TODOS los puntos, con objeto o sin él: con 5292 puntos,\n"
        "cada segundo aquí son 1.5 h de sesión. La quietud del piezo la\n"
        "impone el autofoco por lectura de Z, no este temporizador."
    )
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
    widgets['microscopy_start_btn'] = QPushButton("🚀 Iniciar / Continuar Microscopía")
    widgets['microscopy_start_btn'].setStyleSheet("""
        QPushButton { font-size: 13px; font-weight: bold; padding: 10px; background-color: #27AE60; }
        QPushButton:hover { background-color: #2ECC71; }
        QPushButton:disabled { background-color: #505050; color: #808080; }
    """)
    widgets['microscopy_start_btn'].setEnabled(False)
    widgets['microscopy_start_btn'].setToolTip(
        "Inicia desde el punto indicado arriba. "
        "Tras un fallo FOV, el spin se rellena solo con el punto detenido."
    )
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
        "Solo para microscopía automatizada: si hay objeto válido en el punto,\n"
        "el algoritmo dispara autofoco Z (requiere cámara en vivo + C-Focus).\n"
        "El botón 'Enfocar' ejecuta autofoco manual en cualquier momento."
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
    
    # Botón de calibración C-Focus
    widgets['cfocus_calibrate_btn'] = QPushButton("🔧 Calibrar")
    widgets['cfocus_calibrate_btn'].setEnabled(False)
    widgets['cfocus_calibrate_btn'].setToolTip("Calibra límites del C-Focus (detecta min/max/centro)")
    widgets['cfocus_calibrate_btn'].setStyleSheet("""
        QPushButton { font-size: 12px; font-weight: bold; padding: 6px; background-color: #E67E22; }
        QPushButton:hover { background-color: #D35400; }
        QPushButton:disabled { background-color: #505050; color: #808080; }
    """)
    btn_layout.addWidget(widgets['cfocus_calibrate_btn'])
    
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
    # Sin tope artificial útil: hasta ~1e9 px (límite práctico de QSpinBox int)
    widgets['min_pixels_spin'].setRange(1, 999_999_999)
    widgets['min_pixels_spin'].setValue(100)
    widgets['min_pixels_spin'].setSuffix(" px")
    widgets['min_pixels_spin'].setToolTip("Área mínima del objeto en píxeles (sin límite artificial)")
    widgets['min_pixels_spin'].setFixedWidth(130)
    widgets['min_pixels_spin'].valueChanged.connect(update_params_cb)
    detection_form.addWidget(widgets['min_pixels_spin'], 0, 1)
    
    detection_form.addWidget(QLabel("Área máxima:"), 0, 2)
    widgets['max_pixels_spin'] = QSpinBox()
    widgets['max_pixels_spin'].setRange(1, 999_999_999)
    widgets['max_pixels_spin'].setValue(50000)
    widgets['max_pixels_spin'].setSuffix(" px")
    widgets['max_pixels_spin'].setToolTip("Área máxima del objeto en píxeles (sin límite artificial; escribe el valor que quieras)")
    widgets['max_pixels_spin'].setFixedWidth(130)
    widgets['max_pixels_spin'].valueChanged.connect(update_params_cb)
    detection_form.addWidget(widgets['max_pixels_spin'], 0, 3)
    
    # Circularidad mínima
    detection_form.addWidget(QLabel("Circularidad mín:"), 1, 0)
    widgets['circularity_spin'] = QDoubleSpinBox()
    widgets['circularity_spin'].setRange(0.0, 1.0)
    widgets['circularity_spin'].setSingleStep(0.05)
    widgets['circularity_spin'].setValue(0.25)
    widgets['circularity_spin'].setDecimals(2)
    widgets['circularity_spin'].setToolTip(
        "Circularidad mínima (0-1). 1=círculo perfecto.\n"
        "BAJAR (0.10-0.25) para polen/manchas irregulares.\n"
        "0 = desactivar filtro."
    )
    widgets['circularity_spin'].setFixedWidth(100)
    widgets['circularity_spin'].valueChanged.connect(update_params_cb)
    detection_form.addWidget(widgets['circularity_spin'], 1, 1)
    
    # Aspect ratio mínimo
    detection_form.addWidget(QLabel("Aspect ratio mín:"), 1, 2)
    widgets['aspect_ratio_spin'] = QDoubleSpinBox()
    widgets['aspect_ratio_spin'].setRange(0.0, 1.0)
    widgets['aspect_ratio_spin'].setSingleStep(0.05)
    widgets['aspect_ratio_spin'].setValue(0.25)
    widgets['aspect_ratio_spin'].setDecimals(2)
    widgets['aspect_ratio_spin'].setToolTip(
        "Aspect ratio mínimo (0-1). Rechaza objetos muy alargados.\n"
        "BAJAR (0.10-0.25) para aceptar más formas.\n"
        "0 = desactivar filtro."
    )
    widgets['aspect_ratio_spin'].setFixedWidth(100)
    widgets['aspect_ratio_spin'].valueChanged.connect(update_params_cb)
    detection_form.addWidget(widgets['aspect_ratio_spin'], 1, 3)
    
    # Parámetros de búsqueda Z
    # Fila 2: Distancia total de escaneo
    detection_form.addWidget(QLabel("Distancia fine ±:"), 2, 0)
    widgets['z_scan_range_spin'] = QDoubleSpinBox()
    widgets['z_scan_range_spin'].setRange(0.1, 500.0)
    widgets['z_scan_range_spin'].setValue(6.0)
    widgets['z_scan_range_spin'].setSuffix(" µm")
    widgets['z_scan_range_spin'].setDecimals(1)
    widgets['z_scan_range_spin'].setSingleStep(1.0)
    widgets['z_scan_range_spin'].setToolTip(
        "Límite máximo ±µm alrededor del plano COARSE con mayor S.\n"
        "El recorrido real FINE usa Paso fino × N° capas, sin exceder este Δ.\n"
        "FINE refina el plano que eligió el COARSE: con Δ mayor que 2–3 pasos\n"
        "gruesos vuelve a ser un segundo barrido completo."
    )
    widgets['z_scan_range_spin'].setFixedWidth(100)
    widgets['z_scan_range_spin'].valueChanged.connect(update_params_cb)
    detection_form.addWidget(widgets['z_scan_range_spin'], 2, 1)
    
    # Label de rango de búsqueda
    detection_form.addWidget(QLabel("Rango búsqueda:"), 2, 2)
    widgets['estimated_images_label'] = QLabel("±6.0µm")
    widgets['estimated_images_label'].setStyleSheet("color: #3498DB; font-weight: bold;")
    widgets['estimated_images_label'].setToolTip(
        "Distancia de búsqueda desde centro\n"
        "Autofoco busca BPoF con algoritmo adaptativo,\n"
        "NO captura volumen (usar Z-Stack para eso)"
    )
    detection_form.addWidget(widgets['estimated_images_label'], 2, 3)
    
    # Fila 3: Pasos Z
    detection_form.addWidget(QLabel("Paso grueso:"), 3, 0)
    widgets['z_step_coarse_spin'] = QDoubleSpinBox()
    widgets['z_step_coarse_spin'].setRange(0.001, 100.0)
    widgets['z_step_coarse_spin'].setValue(2.0)
    widgets['z_step_coarse_spin'].setSuffix(" µm")
    widgets['z_step_coarse_spin'].setDecimals(3)
    widgets['z_step_coarse_spin'].setSingleStep(0.1)
    widgets['z_step_coarse_spin'].setToolTip(
        "Paso grueso del escaneo Z (debe ser > Paso fino).\n"
        "Recomendado: 1–3 µm para no saltarse el pico."
    )
    widgets['z_step_coarse_spin'].setFixedWidth(100)
    widgets['z_step_coarse_spin'].valueChanged.connect(update_params_cb)
    detection_form.addWidget(widgets['z_step_coarse_spin'], 3, 1)
    
    detection_form.addWidget(QLabel("Paso fino:"), 3, 2)
    widgets['z_step_fine_spin'] = QDoubleSpinBox()
    widgets['z_step_fine_spin'].setRange(0.001, 100.0)
    widgets['z_step_fine_spin'].setValue(0.5)
    widgets['z_step_fine_spin'].setSuffix(" µm")
    widgets['z_step_fine_spin'].setDecimals(3)
    widgets['z_step_fine_spin'].setSingleStep(0.05)
    widgets['z_step_fine_spin'].setToolTip(
        "Paso real entre candidatos FINE alrededor de Z_coarse*.\n"
        "Junto con N° capas define el recorrido simétrico. "
        "Recomendado: 0.2–0.5 µm."
    )
    widgets['z_step_fine_spin'].setFixedWidth(100)
    widgets['z_step_fine_spin'].valueChanged.connect(update_params_cb)
    detection_form.addWidget(widgets['z_step_fine_spin'], 3, 3)
    
    # Fila 4: N° capturas multi-focales y tolerancia de llegada Z
    detection_form.addWidget(QLabel("N° capturas:"), 4, 0)
    widgets['n_captures_spin'] = QSpinBox()
    widgets['n_captures_spin'].setRange(3, 11)
    widgets['n_captures_spin'].setValue(3)
    widgets['n_captures_spin'].setSingleStep(2)  # Solo impares
    widgets['n_captures_spin'].setToolTip(
        "Capturas multi-focales (impar ≥3).\n"
        "Con n=3 y paso 10µm: BPoF-10, BPoF, BPoF+10 (f0,f1,f2)."
    )
    widgets['n_captures_spin'].setFixedWidth(100)
    widgets['n_captures_spin'].valueChanged.connect(update_params_cb)
    detection_form.addWidget(widgets['n_captures_spin'], 4, 1)
    
    detection_form.addWidget(QLabel("Tol. Z llegada:"), 4, 2)
    widgets['z_arrive_tol_spin'] = QDoubleSpinBox()
    widgets['z_arrive_tol_spin'].setRange(0.05, 5.0)
    widgets['z_arrive_tol_spin'].setValue(0.25)
    widgets['z_arrive_tol_spin'].setSuffix(" µm")
    widgets['z_arrive_tol_spin'].setDecimals(2)
    widgets['z_arrive_tol_spin'].setSingleStep(0.05)
    widgets['z_arrive_tol_spin'].setLocale(QLocale(QLocale.C))
    widgets['z_arrive_tol_spin'].setToolTip(
        "Condición de cumplimiento: |Z_read − Z_cmd| ≤ tol en lecturas\n"
        "consecutivas. No es un tiempo de asentamiento fijo.\n"
        "Debe ser menor que la mitad del paso fino: con tol ≥ paso, dos\n"
        "candidatos FINE distintos pueden medirse en la misma Z real."
    )
    widgets['z_arrive_tol_spin'].setFixedWidth(100)
    widgets['z_arrive_tol_spin'].valueChanged.connect(update_params_cb)
    detection_form.addWidget(widgets['z_arrive_tol_spin'], 4, 3)
    # Compat: alias antiguo (tests / código que aún busque z_settle_spin)
    widgets['z_settle_spin'] = widgets['z_arrive_tol_spin']

    detection_form.addWidget(QLabel("N° capas fine:"), 5, 2)
    widgets['n_fine_planes_spin'] = QSpinBox()
    widgets['n_fine_planes_spin'].setRange(3, 101)
    widgets['n_fine_planes_spin'].setValue(9)
    widgets['n_fine_planes_spin'].setSingleStep(2)
    widgets['n_fine_planes_spin'].setToolTip(
        "N candidatos FINE (impar), centrados exactamente en Z_coarse*.\n"
        "Semirango solicitado = Paso fino × (N−1)/2; Δ fine es el máximo.\n"
        "Cada capa cuesta ~0.9 s medidos (mover, esperar quietud, flush\n"
        "óptico, RAW completo y métrica). El recorrido se visita del centro\n"
        "hacia afuera y se corta cuando el pico queda atrás."
    )
    widgets['n_fine_planes_spin'].setFixedWidth(100)
    widgets['n_fine_planes_spin'].valueChanged.connect(update_params_cb)
    detection_form.addWidget(widgets['n_fine_planes_spin'], 5, 3)

    detection_form.addWidget(QLabel("Variación S:"), 5, 0)
    widgets['z_step_capture_spin'] = QDoubleSpinBox()
    widgets['z_step_capture_spin'].setRange(0.1, 90.0)
    widgets['z_step_capture_spin'].setValue(10.0)
    widgets['z_step_capture_spin'].setSuffix(" %")
    widgets['z_step_capture_spin'].setDecimals(1)
    widgets['z_step_capture_spin'].setLocale(QLocale(QLocale.C))
    widgets['z_step_capture_spin'].setSingleStep(0.1)
    widgets['z_step_capture_spin'].setToolTip(
        "Caída óptica objetivo del índice S respecto al BPoF.\n"
        "Los planos se seleccionan desde la curva COARSE+FINE ya medida;\n"
        "no se ejecuta un segundo barrido Z después de encontrar el BPoF."
    )
    widgets['z_step_capture_spin'].setFixedWidth(100)
    widgets['z_step_capture_spin'].valueChanged.connect(update_params_cb)
    detection_form.addWidget(widgets['z_step_capture_spin'], 5, 1)

    # Fila 6: ROI margin
    detection_form.addWidget(QLabel("ROI Margin:"), 6, 0)
    widgets['roi_margin_spin'] = QSpinBox()
    widgets['roi_margin_spin'].setRange(0, 10000)
    widgets['roi_margin_spin'].setValue(200)
    widgets['roi_margin_spin'].setSuffix(" px")
    widgets['roi_margin_spin'].setToolTip(
        "Margen solicitado alrededor del bbox/ROI (px).\n"
        "El índice S usa la máscara del objeto y un contexto máximo de 16 px\n"
        "para no procesar fondo descartado ni ralentizar COARSE/FINE."
    )
    widgets['roi_margin_spin'].setFixedWidth(100)
    widgets['roi_margin_spin'].valueChanged.connect(update_params_cb)
    detection_form.addWidget(widgets['roi_margin_spin'], 6, 1)
    
    layout.addLayout(detection_form)
    
    # Label de estado C-Focus
    widgets['cfocus_status_label'] = QLabel("C-Focus: No conectado")
    widgets['cfocus_status_label'].setStyleSheet("color: #888; font-style: italic;")
    layout.addWidget(widgets['cfocus_status_label'])
    
    group.setLayout(layout)
    return group


def create_u2net_config_section(widgets: dict, mode_change_cb, update_params_cb) -> QGroupBox:
    """
    Crea la sección de configuración del detector U2NET.
    
    Args:
        widgets: Dict donde almacenar referencias a widgets
        update_params_cb: Callback para actualizar parámetros del detector
        
    Returns:
        QGroupBox configurado
    """
    group = QGroupBox("🎛️ Opciones Detector U2NET")
    layout = QVBoxLayout()
    
    # Modo de detección (presets)
    mode_layout = QHBoxLayout()
    mode_layout.addWidget(QLabel("Modo de Detección:"))
    
    widgets['detection_mode_combo'] = QComboBox()
    widgets['detection_mode_combo'].addItems(["Normal", "Sensible (Polen)", "Robusto"])
    widgets['detection_mode_combo'].setToolTip(
        "Presets optimizados:\n"
        "• Normal: Objetos medianos-grandes, buen contraste\n"
        "• Sensible: Polen/objetos pequeños, bajo contraste\n"
        "• Robusto: Objetos grandes con ruido de fondo"
    )
    widgets['detection_mode_combo'].setFixedWidth(180)
    widgets['detection_mode_combo'].currentIndexChanged.connect(mode_change_cb)
    mode_layout.addWidget(widgets['detection_mode_combo'])
    mode_layout.addStretch()
    layout.addLayout(mode_layout)
    
    # Parámetros avanzados
    advanced_form = QGridLayout()
    
    # Fila 0: Umbral de Saliencia
    advanced_form.addWidget(QLabel("Umbral Saliencia:"), 0, 0)
    widgets['saliency_threshold_spin'] = QDoubleSpinBox()
    widgets['saliency_threshold_spin'].setRange(0.05, 0.60)
    widgets['saliency_threshold_spin'].setSingleStep(0.05)
    widgets['saliency_threshold_spin'].setValue(0.30)
    widgets['saliency_threshold_spin'].setDecimals(2)
    widgets['saliency_threshold_spin'].setToolTip(
        "Sensibilidad de detección (0.05-0.60)\n"
        "• Menor valor = más objetos detectados\n"
        "• Mayor valor = solo objetos muy salientes\n"
        "Default: 0.30 (Normal), 0.15 (Sensible)"
    )
    widgets['saliency_threshold_spin'].setFixedWidth(100)
    widgets['saliency_threshold_spin'].valueChanged.connect(lambda: update_params_cb())
    advanced_form.addWidget(widgets['saliency_threshold_spin'], 0, 1)
    
    # Fila 0 col 2: Factor Adaptativo K
    advanced_form.addWidget(QLabel("Factor Adaptativo K:"), 0, 2)
    widgets['adaptive_k_spin'] = QDoubleSpinBox()
    widgets['adaptive_k_spin'].setRange(0.1, 1.0)
    widgets['adaptive_k_spin'].setSingleStep(0.1)
    widgets['adaptive_k_spin'].setValue(0.5)
    widgets['adaptive_k_spin'].setDecimals(1)
    widgets['adaptive_k_spin'].setToolTip(
        "Agresividad del umbral adaptativo (0.1-1.0)\n"
        "Fórmula: threshold = mean + k×std\n"
        "• Menor k = más sensible a variaciones\n"
        "• Mayor k = más conservador\n"
        "Default: 0.5 (Normal), 0.3 (Sensible)"
    )
    widgets['adaptive_k_spin'].setFixedWidth(100)
    widgets['adaptive_k_spin'].valueChanged.connect(lambda: update_params_cb())
    advanced_form.addWidget(widgets['adaptive_k_spin'], 0, 3)
    
    # Fila 1: Kernel Morfológico
    advanced_form.addWidget(QLabel("Kernel Morfológico:"), 1, 0)
    widgets['morph_kernel_combo'] = QComboBox()
    widgets['morph_kernel_combo'].addItems(["3×3 (Sensible)", "5×5 (Normal)", "7×7 (Robusto)"])
    widgets['morph_kernel_combo'].setCurrentIndex(1)  # 5×5 por defecto
    widgets['morph_kernel_combo'].setToolTip(
        "Tamaño del filtro morfológico\n"
        "• 3×3: Preserva objetos pequeños (polen)\n"
        "• 5×5: Balance general\n"
        "• 7×7: Elimina ruido agresivamente"
    )
    widgets['morph_kernel_combo'].setFixedWidth(150)
    widgets['morph_kernel_combo'].currentIndexChanged.connect(lambda: update_params_cb())
    advanced_form.addWidget(widgets['morph_kernel_combo'], 1, 1)
    
    # Fila 1 col 2: CLAHE Clip Limit
    advanced_form.addWidget(QLabel("CLAHE Clip Limit:"), 1, 2)
    widgets['clahe_clip_spin'] = QDoubleSpinBox()
    widgets['clahe_clip_spin'].setRange(1.0, 5.0)
    widgets['clahe_clip_spin'].setSingleStep(0.5)
    widgets['clahe_clip_spin'].setValue(2.0)
    widgets['clahe_clip_spin'].setDecimals(1)
    widgets['clahe_clip_spin'].setToolTip(
        "Intensidad de mejora de contraste (1.0-5.0)\n"
        "• Mayor valor = más contraste local\n"
        "• Útil para objetos con bajo contraste\n"
        "Default: 2.0 (Normal), 3.5 (Sensible)"
    )
    widgets['clahe_clip_spin'].setFixedWidth(100)
    widgets['clahe_clip_spin'].valueChanged.connect(lambda: update_params_cb())
    advanced_form.addWidget(widgets['clahe_clip_spin'], 1, 3)
    
    # Fila 2: CLAHE Tile Size
    advanced_form.addWidget(QLabel("CLAHE Tile Size:"), 2, 0)
    widgets['clahe_tile_combo'] = QComboBox()
    widgets['clahe_tile_combo'].addItems(["4×4 (Detalle)", "8×8 (Normal)", "16×16 (Global)"])
    widgets['clahe_tile_combo'].setCurrentIndex(1)  # 8×8 por defecto
    widgets['clahe_tile_combo'].setToolTip(
        "Tamaño de región para ecualización\n"
        "• 4×4: Más detalle local (polen pequeño)\n"
        "• 8×8: Balance general\n"
        "• 16×16: Ecualización más global"
    )
    widgets['clahe_tile_combo'].setFixedWidth(150)
    widgets['clahe_tile_combo'].currentIndexChanged.connect(lambda: update_params_cb())
    advanced_form.addWidget(widgets['clahe_tile_combo'], 2, 1)
    
    # Botón restaurar defaults
    restore_btn = QPushButton("↺ Restaurar Defaults")
    restore_btn.setToolTip("Restaura valores por defecto según el modo seleccionado")
    restore_btn.setFixedWidth(150)
    restore_btn.clicked.connect(lambda: update_params_cb(restore_defaults=True))
    advanced_form.addWidget(restore_btn, 2, 2, 1, 2)
    
    layout.addLayout(advanced_form)
    
    # Información de estado
    widgets['u2net_status_label'] = QLabel("Modelo: U2NETP | Device: CPU")
    widgets['u2net_status_label'].setStyleSheet("color: #3498DB; font-style: italic; font-size: 10px;")
    layout.addWidget(widgets['u2net_status_label'])
    
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

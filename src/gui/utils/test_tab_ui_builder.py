"""
Builder de UI para TestTab.

Módulo separado que contiene los métodos de creación de secciones de UI,
reduciendo significativamente el tamaño de TestTab.

Cada función retorna un QGroupBox configurado con todos sus widgets.
Los widgets que necesitan ser accedidos desde TestTab se almacenan
en un diccionario 'widgets' que se pasa como parámetro.
"""

import logging
from PyQt5.QtWidgets import (QGroupBox, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QLabel, QLineEdit, QPushButton, QTextEdit,
                             QCheckBox, QRadioButton, QFrame, QButtonGroup)
from PyQt5.QtCore import Qt

logger = logging.getLogger('MotorControl_L206')


def create_controllers_section(widgets: dict, clear_callback) -> QGroupBox:
    """
    Crea sección de controladores H∞ transferidos.
    
    Args:
        widgets: Dict donde almacenar referencias a widgets
        clear_callback: Función a llamar cuando se presiona limpiar
        
    Returns:
        QGroupBox configurado
    """
    group = QGroupBox("📦 Controladores H∞ Transferidos")
    layout = QVBoxLayout()
    
    # Motor A
    motor_a_frame = QFrame()
    motor_a_frame.setFrameStyle(QFrame.Box | QFrame.Raised)
    motor_a_layout = QVBoxLayout()
    
    header_a = QHBoxLayout()
    widgets['motor_a_label'] = QLabel("<b>Motor A (X)</b>")
    header_a.addWidget(widgets['motor_a_label'])
    header_a.addStretch()
    widgets['motor_a_status'] = QLabel("⚪ Sin controlador")
    widgets['motor_a_status'].setStyleSheet("color: #95A5A6;")
    header_a.addWidget(widgets['motor_a_status'])
    motor_a_layout.addLayout(header_a)
    
    widgets['motor_a_info'] = QTextEdit()
    widgets['motor_a_info'].setReadOnly(True)
    widgets['motor_a_info'].setMaximumHeight(70)
    widgets['motor_a_info'].setStyleSheet("font-family: monospace; font-size: 11px; background: white; color: black;")
    widgets['motor_a_info'].setPlaceholderText("Transfiere un controlador desde 'H∞ Synthesis'...")
    motor_a_layout.addWidget(widgets['motor_a_info'])
    
    btn_a = QHBoxLayout()
    widgets['clear_a_btn'] = QPushButton("🗑️ Limpiar")
    widgets['clear_a_btn'].clicked.connect(lambda: clear_callback('A'))
    widgets['clear_a_btn'].setEnabled(False)
    btn_a.addWidget(widgets['clear_a_btn'])
    btn_a.addStretch()
    motor_a_layout.addLayout(btn_a)
    
    motor_a_frame.setLayout(motor_a_layout)
    layout.addWidget(motor_a_frame)
    
    # Motor B
    motor_b_frame = QFrame()
    motor_b_frame.setFrameStyle(QFrame.Box | QFrame.Raised)
    motor_b_layout = QVBoxLayout()
    
    header_b = QHBoxLayout()
    widgets['motor_b_label'] = QLabel("<b>Motor B (Y)</b>")
    header_b.addWidget(widgets['motor_b_label'])
    header_b.addStretch()
    widgets['motor_b_status'] = QLabel("⚪ Sin controlador")
    widgets['motor_b_status'].setStyleSheet("color: #95A5A6;")
    header_b.addWidget(widgets['motor_b_status'])
    motor_b_layout.addLayout(header_b)
    
    widgets['motor_b_info'] = QTextEdit()
    widgets['motor_b_info'].setReadOnly(True)
    widgets['motor_b_info'].setMaximumHeight(70)
    widgets['motor_b_info'].setStyleSheet("font-family: monospace; font-size: 11px; background: white; color: black;")
    widgets['motor_b_info'].setPlaceholderText("Transfiere un controlador desde 'H∞ Synthesis'...")
    motor_b_layout.addWidget(widgets['motor_b_info'])
    
    btn_b = QHBoxLayout()
    widgets['clear_b_btn'] = QPushButton("🗑️ Limpiar")
    widgets['clear_b_btn'].clicked.connect(lambda: clear_callback('B'))
    widgets['clear_b_btn'].setEnabled(False)
    btn_b.addWidget(widgets['clear_b_btn'])
    btn_b.addStretch()
    motor_b_layout.addLayout(btn_b)
    
    motor_b_frame.setLayout(motor_b_layout)
    layout.addWidget(motor_b_frame)
    
    group.setLayout(layout)
    return group


def create_motor_sensor_section(widgets: dict) -> QGroupBox:
    """
    Crea sección de asignación motor-sensor.
    
    Permite seleccionar qué sensor lee cada motor y si se invierte el PWM.
    Cada motor tiene su propio grupo de radio buttons independiente.
    
    Args:
        widgets: Dict donde almacenar referencias a widgets
        
    Returns:
        QGroupBox configurado
    """
    group = QGroupBox("🔧 Asignación Motor ↔ Sensor")
    layout = QVBoxLayout()
    
    # === Motor A ===
    row_a = QHBoxLayout()
    row_a.addWidget(QLabel("<b>Motor A lee:</b>"))
    
    # Grupo de botones para Motor A (independiente de Motor B)
    widgets['motor_a_sensor1'] = QCheckBox("Sensor 1")
    widgets['motor_a_sensor2'] = QCheckBox("Sensor 2")
    
    # Exclusión mutua manual para Motor A
    widgets['motor_a_sensor1'].toggled.connect(
        lambda checked: widgets['motor_a_sensor2'].setChecked(False) if checked else None
    )
    widgets['motor_a_sensor2'].toggled.connect(
        lambda checked: widgets['motor_a_sensor1'].setChecked(False) if checked else None
    )
    
    row_a.addWidget(widgets['motor_a_sensor1'])
    row_a.addWidget(widgets['motor_a_sensor2'])
    
    widgets['motor_a_invert'] = QCheckBox("⇄ Invertir PWM")
    row_a.addWidget(widgets['motor_a_invert'])
    row_a.addStretch()
    layout.addLayout(row_a)
    
    # === Motor B ===
    row_b = QHBoxLayout()
    row_b.addWidget(QLabel("<b>Motor B lee:</b>"))
    
    # Grupo de botones para Motor B (independiente de Motor A)
    widgets['motor_b_sensor1'] = QCheckBox("Sensor 1")
    widgets['motor_b_sensor2'] = QCheckBox("Sensor 2")
    
    # Exclusión mutua manual para Motor B
    widgets['motor_b_sensor1'].toggled.connect(
        lambda checked: widgets['motor_b_sensor2'].setChecked(False) if checked else None
    )
    widgets['motor_b_sensor2'].toggled.connect(
        lambda checked: widgets['motor_b_sensor1'].setChecked(False) if checked else None
    )
    
    row_b.addWidget(widgets['motor_b_sensor1'])
    row_b.addWidget(widgets['motor_b_sensor2'])
    
    widgets['motor_b_invert'] = QCheckBox("⇄ Invertir PWM")
    row_b.addWidget(widgets['motor_b_invert'])
    row_b.addStretch()
    layout.addLayout(row_b)
    
    # Nota informativa
    info = QLabel("⚠️ Configura sensor e inversión ANTES de iniciar control.")
    info.setStyleSheet("padding: 5px; background: #FFF3CD; border: 1px solid #FFC107; border-radius: 3px;")
    layout.addWidget(info)
    
    group.setLayout(layout)
    return group


def create_calibration_section(widgets: dict, reload_callback) -> QGroupBox:
    """
    Crea sección de calibración.
    
    Args:
        widgets: Dict donde almacenar referencias a widgets
        reload_callback: Función a llamar cuando se presiona recargar
        
    Returns:
        QGroupBox configurado
    """
    group = QGroupBox("📐 Calibración del Sistema")
    layout = QVBoxLayout()
    
    # Status
    widgets['calibration_status'] = QLabel("⚪ Cargando calibración...")
    widgets['calibration_status'].setStyleSheet("font-size: 14px; font-weight: bold; color: #95A5A6;")
    layout.addWidget(widgets['calibration_status'])
    
    # Detalles
    widgets['calibration_details'] = QTextEdit()
    widgets['calibration_details'].setReadOnly(True)
    widgets['calibration_details'].setMaximumHeight(80)
    widgets['calibration_details'].setStyleSheet("font-family: monospace; font-size: 11px; background: white; color: black;")
    layout.addWidget(widgets['calibration_details'])
    
    # Botón recargar
    reload_btn = QPushButton("🔄 Recargar Calibración")
    reload_btn.clicked.connect(reload_callback)
    layout.addWidget(reload_btn)
    
    group.setLayout(layout)
    return group


def create_position_control_section(widgets: dict, start_callback, stop_callback) -> QGroupBox:
    """
    Crea sección de control por posición.
    
    Args:
        widgets: Dict donde almacenar referencias a widgets
        start_callback: Función para iniciar control dual
        stop_callback: Función para detener control dual
        
    Returns:
        QGroupBox configurado
    """
    group = QGroupBox("🎯 Control por Posición (µm)")
    layout = QVBoxLayout()
    
    # Referencias
    ref_layout = QGridLayout()
    ref_layout.addWidget(QLabel("Ref. Motor A (X):"), 0, 0)
    widgets['ref_a_input'] = QLineEdit("15000")
    widgets['ref_a_input'].setStyleSheet("background: white; color: black;")
    ref_layout.addWidget(widgets['ref_a_input'], 0, 1)
    ref_layout.addWidget(QLabel("µm"), 0, 2)
    
    ref_layout.addWidget(QLabel("Ref. Motor B (Y):"), 1, 0)
    widgets['ref_b_input'] = QLineEdit("15000")
    widgets['ref_b_input'].setStyleSheet("background: white; color: black;")
    ref_layout.addWidget(widgets['ref_b_input'], 1, 1)
    ref_layout.addWidget(QLabel("µm"), 1, 2)
    layout.addLayout(ref_layout)
    
    # Botones
    btn_layout = QHBoxLayout()
    widgets['start_dual_btn'] = QPushButton("▶️ Iniciar Control Dual")
    widgets['start_dual_btn'].setStyleSheet("font-weight: bold; padding: 8px; background: #27AE60;")
    widgets['start_dual_btn'].setEnabled(False)
    widgets['start_dual_btn'].clicked.connect(start_callback)
    btn_layout.addWidget(widgets['start_dual_btn'])
    
    widgets['stop_dual_btn'] = QPushButton("⏹️ Detener")
    widgets['stop_dual_btn'].setStyleSheet("font-weight: bold; padding: 8px; background: #E74C3C;")
    widgets['stop_dual_btn'].setEnabled(False)
    widgets['stop_dual_btn'].clicked.connect(stop_callback)
    btn_layout.addWidget(widgets['stop_dual_btn'])
    layout.addLayout(btn_layout)
    
    group.setLayout(layout)
    return group


def create_trajectory_section(widgets: dict, generate_callback, preview_callback, 
                              export_callback, import_callback) -> QGroupBox:
    """
    Crea sección de generación de trayectorias.
    
    Args:
        widgets: Dict donde almacenar referencias a widgets
        generate_callback: Función para generar trayectoria
        preview_callback: Función para vista previa
        export_callback: Función para exportar CSV
        import_callback: Función para importar CSV
        
    Returns:
        QGroupBox configurado
    """
    group = QGroupBox("📍 Generador de Trayectorias Zig-Zag")
    layout = QVBoxLayout()
    
    # Parámetros
    params_layout = QGridLayout()
    
    params_layout.addWidget(QLabel("Puntos:"), 0, 0)
    widgets['points_input'] = QLineEdit("100")
    widgets['points_input'].setStyleSheet("background: white; color: black;")
    params_layout.addWidget(widgets['points_input'], 0, 1)
    
    params_layout.addWidget(QLabel("X inicio (µm):"), 0, 2)
    widgets['x_start_input'] = QLineEdit("10000")
    widgets['x_start_input'].setStyleSheet("background: white; color: black;")
    params_layout.addWidget(widgets['x_start_input'], 0, 3)
    
    params_layout.addWidget(QLabel("X fin (µm):"), 0, 4)
    widgets['x_end_input'] = QLineEdit("20000")
    widgets['x_end_input'].setStyleSheet("background: white; color: black;")
    params_layout.addWidget(widgets['x_end_input'], 0, 5)
    
    params_layout.addWidget(QLabel("Y inicio (µm):"), 1, 2)
    widgets['y_start_input'] = QLineEdit("10000")
    widgets['y_start_input'].setStyleSheet("background: white; color: black;")
    params_layout.addWidget(widgets['y_start_input'], 1, 3)
    
    params_layout.addWidget(QLabel("Y fin (µm):"), 1, 4)
    widgets['y_end_input'] = QLineEdit("20000")
    widgets['y_end_input'].setStyleSheet("background: white; color: black;")
    params_layout.addWidget(widgets['y_end_input'], 1, 5)
    
    params_layout.addWidget(QLabel("Delay (s):"), 1, 0)
    widgets['delay_input'] = QLineEdit("0.5")
    widgets['delay_input'].setStyleSheet("background: white; color: black;")
    params_layout.addWidget(widgets['delay_input'], 1, 1)
    
    layout.addLayout(params_layout)
    
    # Botones
    btn_layout = QHBoxLayout()
    
    generate_btn = QPushButton("🔄 Generar")
    generate_btn.setStyleSheet("font-weight: bold; padding: 6px;")
    generate_btn.clicked.connect(generate_callback)
    btn_layout.addWidget(generate_btn)
    
    preview_btn = QPushButton("👁️ Vista Previa")
    preview_btn.clicked.connect(preview_callback)
    btn_layout.addWidget(preview_btn)
    
    export_btn = QPushButton("💾 Exportar CSV")
    export_btn.clicked.connect(export_callback)
    btn_layout.addWidget(export_btn)
    
    import_btn = QPushButton("📂 Importar CSV")
    import_btn.clicked.connect(import_callback)
    btn_layout.addWidget(import_btn)
    
    layout.addLayout(btn_layout)
    
    group.setLayout(layout)
    return group


def create_zigzag_section(widgets: dict, start_callback, stop_callback) -> QGroupBox:
    """
    Crea sección de ejecución zig-zag.
    
    Args:
        widgets: Dict donde almacenar referencias a widgets
        start_callback: Función para iniciar ejecución
        stop_callback: Función para detener ejecución
        
    Returns:
        QGroupBox configurado
    """
    group = QGroupBox("🚀 Ejecución de Trayectoria")
    layout = QVBoxLayout()
    
    # Status
    widgets['trajectory_status'] = QLabel("⚪ Sin trayectoria")
    widgets['trajectory_status'].setStyleSheet("font-size: 14px; font-weight: bold; color: #95A5A6;")
    layout.addWidget(widgets['trajectory_status'])
    
    # Parámetros de ejecución
    exec_layout = QGridLayout()
    
    exec_layout.addWidget(QLabel("Tolerancia (µm):"), 0, 0)
    widgets['tolerance_input'] = QLineEdit("25")
    widgets['tolerance_input'].setStyleSheet("background: white; color: black;")
    exec_layout.addWidget(widgets['tolerance_input'], 0, 1)
    
    exec_layout.addWidget(QLabel("Pausa (s):"), 0, 2)
    widgets['pause_input'] = QLineEdit("2.0")
    widgets['pause_input'].setStyleSheet("background: white; color: black;")
    exec_layout.addWidget(widgets['pause_input'], 0, 3)
    
    layout.addLayout(exec_layout)
    
    # Feedback visual
    feedback_layout = QGridLayout()
    
    feedback_layout.addWidget(QLabel("Progreso:"), 0, 0)
    widgets['trajectory_progress_label'] = QLabel("-- / --")
    widgets['trajectory_progress_label'].setStyleSheet("font-family: monospace; font-weight: bold;")
    feedback_layout.addWidget(widgets['trajectory_progress_label'], 0, 1)
    
    feedback_layout.addWidget(QLabel("Punto actual:"), 0, 2)
    widgets['current_point_label'] = QLabel("(---, ---) µm")
    widgets['current_point_label'].setStyleSheet("font-family: monospace;")
    feedback_layout.addWidget(widgets['current_point_label'], 0, 3)
    
    feedback_layout.addWidget(QLabel("Error:"), 1, 0)
    widgets['error_x_label'] = QLabel("X: --- µm")
    widgets['error_x_label'].setStyleSheet("font-family: monospace; color: #E74C3C;")
    feedback_layout.addWidget(widgets['error_x_label'], 1, 1)
    
    widgets['error_y_label'] = QLabel("Y: --- µm")
    widgets['error_y_label'].setStyleSheet("font-family: monospace; color: #E74C3C;")
    feedback_layout.addWidget(widgets['error_y_label'], 1, 2)
    
    widgets['settling_label'] = QLabel("Settling: --/--")
    widgets['settling_label'].setStyleSheet("font-family: monospace; color: #F39C12;")
    feedback_layout.addWidget(widgets['settling_label'], 1, 3)
    
    layout.addLayout(feedback_layout)
    
    # Botones
    btn_layout = QHBoxLayout()
    
    widgets['zigzag_start_btn'] = QPushButton("▶️ Ejecutar Trayectoria")
    widgets['zigzag_start_btn'].setStyleSheet("font-weight: bold; padding: 8px; background: #3498DB;")
    widgets['zigzag_start_btn'].setEnabled(False)
    widgets['zigzag_start_btn'].clicked.connect(start_callback)
    btn_layout.addWidget(widgets['zigzag_start_btn'])
    
    widgets['zigzag_stop_btn'] = QPushButton("⏹️ Detener")
    widgets['zigzag_stop_btn'].setStyleSheet("font-weight: bold; padding: 8px; background: #E74C3C;")
    widgets['zigzag_stop_btn'].setEnabled(False)
    widgets['zigzag_stop_btn'].clicked.connect(stop_callback)
    btn_layout.addWidget(widgets['zigzag_stop_btn'])
    
    layout.addLayout(btn_layout)
    
    group.setLayout(layout)
    return group

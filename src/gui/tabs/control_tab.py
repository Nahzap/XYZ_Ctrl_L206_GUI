"""
Pestaña de Control de Motores.

Encapsula la UI para control manual/automático de motores y visualización de sensores.
"""

import logging
import serial.tools.list_ports
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QGroupBox, QLabel, QLineEdit, QPushButton, QComboBox,
                             QCheckBox)
from PyQt5.QtCore import pyqtSignal

from config.constants import BAUD_RATE, FACTORY_UI, MCU_TYPE
from config.mcu_profiles import MCU_FPGA, MCU_PROFILES, list_mcu_ids
from core.communication.protocol import FPGA_FINAL, FPGA_RESET, FPGA_ZERO, MotorProtocol

logger = logging.getLogger('MotorControl_L206')

_DEFAULT_TEXTS = "default"
# Con la FPGA las filas de sensores se ocultan: la tabla Pedido/Cuenta/Error las reemplaza.
_SENSOR_TITLES = {
    _DEFAULT_TEXTS: "Lectura de Sensores Análogos",
    MCU_FPGA: "Cuenta de Encoders (FPGA)",
}
# (valor inicial, placeholder, tooltip)
_POWER_TEXTS = {
    _DEFAULT_TEXTS: (
        "128,0",
        "Ej: 128,-128 (Arduino ≥110)",
        "Potencia Motor A y B (-255..255). Arranque útil: |pwm|≥110 (Arduino) / ≥95 (STM32).",
    ),
    MCU_FPGA: (
        "40,0",
        "Ej: 40,-40 (%)",
        "Potencia X y Y en % (-80..80). La FPGA recorta a ±80 %.",
    ),
}
_COMMAND_TEXTS = {
    _DEFAULT_TEXTS: (
        "Comandos: M | A,<pwm_a>,<pwm_b> | B | N. "
        "STM32: +F/I/P. Arduino: PWM≥110; F/I/P ignorados."
    ),
    MCU_FPGA: (
        "FPGA: calibrar 1 · Zero, 2 · Máximo, 3 · Manual (M: los potes mandan) | "
        "A,<%x>,<%y> hasta ±80 | B freno | N motores sueltos. F/I no se usan."
    ),
}
_MANUAL_TIPS = {
    _DEFAULT_TEXTS: "",
    MCU_FPGA: "En la FPGA, M hace que los potes manden la posición: la platina va a donde estén.",
}
_MOTOR_TEXTS = {
    _DEFAULT_TEXTS: ("Potencia Motor A:", "Potencia Motor B:"),
    MCU_FPGA: ("Potencia FPGA X (%):", "Potencia FPGA Y (%):"),
}

_STATE_COLORS = {
    'MANUAL': '#3498DB',
    'AUTO': '#9B59B6',
    'HOLD': '#27AE60',
    'BRAKE': '#E74C3C',
    'SETTLING': '#F39C12',
    'UNKNOWN': '#95A5A6',
    'LEGACY': '#F39C12',
    'RESET': '#E67E22',
    'PULSE': '#1ABC9C',
    'PC': '#27AE60',
}

# Calibración FPGA, igual que por COM: lo que hace cada grupo de marcas.
_CALIB_TIPS = {
    FPGA_ZERO: "la cuenta de X y de Y pasa a 0 donde está la platina.",
    FPGA_FINAL: "el máximo de cada eje queda en su cuenta actual; con los dos, la FPGA sale de RESET.",
    FPGA_RESET: "borra cero, máximo y cuentas; la FPGA vuelve a RESET.",
}
_FPGA_COUNTS_PER_EDGE = MCU_PROFILES[MCU_FPGA]["counts_per_edge"]


def _texts_for(table: dict, mcu_id: str):
    return table.get(mcu_id, table[_DEFAULT_TEXTS])


class ControlTab(QWidget):
    """Pestaña para control de motores y visualización de sensores."""

    serial_reconnect_requested = pyqtSignal(str, int)  # puerto, baudrate
    mcu_profile_changed = pyqtSignal(str)  # STM32 | ARDUINO | FPGA
    frame_log_requested = pyqtSignal(bool)  # grabar tramas FPGA a CSV: iniciar / detener
    brake_requested = pyqtSignal()
    
    def __init__(self, serial_handler=None, parent=None):
        """
        Inicializa la pestaña de control.
        
        Args:
            serial_handler: Instancia de SerialHandler para comunicación
            parent: Widget padre (CTRL_GUI)
        """
        super().__init__(parent)
        self.serial_handler = serial_handler
        self.value_labels = {}
        self._power_max = 255
        self._mcu_state = None          # Estado de la última trama; None = sin trama
        self._last_settled = None
        self._last_xy = (0, 0)
        self._error_colors = {}
        self._setup_ui()
        self._apply_profile_ui(self.get_selected_mcu())
        logger.debug("ControlTab inicializado")
    
    def _setup_ui(self):
        """Configura la interfaz de usuario."""
        layout = QVBoxLayout(self)
        
        # Configuración Serial
        serial_group = self._create_serial_config_group()
        layout.addWidget(serial_group)
        
        # Panel de Control
        control_group = self._create_control_group()
        layout.addWidget(control_group)
        
        # Estado de Motores
        motors_group = self._create_motors_group()
        layout.addWidget(motors_group)
        
        # Lectura de Sensores
        sensors_group = self._create_sensors_group()
        layout.addWidget(sensors_group)
        
        layout.addWidget(self._create_mcu_status_group())
        
        layout.addStretch()
    
    def _create_serial_config_group(self):
        """Crea el panel de configuración serial."""
        group_box = QGroupBox("⚙️ Configuración Serial")
        layout = QGridLayout()

        layout.addWidget(QLabel("MCU:"), 0, 0)
        self.mcu_combo = QComboBox()
        for mcu_id in list_mcu_ids():
            prof = MCU_PROFILES[mcu_id]
            self.mcu_combo.addItem(prof["label"], mcu_id)
        idx = self.mcu_combo.findData(MCU_TYPE)
        if idx < 0:
            idx = self.mcu_combo.findData("ARDUINO")
        self.mcu_combo.setCurrentIndex(max(0, idx))
        self.mcu_combo.setToolTip(
            "STM32F767ZI = MycoViT (C(z) F/I/P). "
            "Arduino UNO = emergencia DRV8871 (host-only; PWM≥110). "
            "FPGA Tang Nano 9K = Motor_CTRL (encoders, 115200, potencia en %)."
        )
        self.mcu_combo.currentIndexChanged.connect(self._on_mcu_changed)
        layout.addWidget(self.mcu_combo, 0, 1, 1, 2)
        
        # Puerto COM con detección automática
        layout.addWidget(QLabel("Puerto:"), 1, 0)
        self.port_combo = QComboBox()
        self.port_combo.setToolTip("Puerto serial: ST-Link VCP (STM32) o Arduino UNO")
        layout.addWidget(self.port_combo, 1, 1)
        
        # Botón escanear puertos
        scan_btn = QPushButton("🔄")
        scan_btn.setFixedWidth(40)
        scan_btn.setToolTip("Escanear puertos disponibles")
        scan_btn.clicked.connect(self._scan_ports)
        layout.addWidget(scan_btn, 1, 2)
        
        # Escanear puertos al inicializar
        self._scan_ports()
        
        # Baudrate seleccionable; default de diseño = BAUD_RATE (1 Mbps).
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(['9600', '19200', '38400', '57600', '115200', '230400', '1000000'])
        self.baudrate_combo.setCurrentText(str(BAUD_RATE))
        self.baudrate_combo.setToolTip(
            "Velocidad serial. Firmware STM32/Arduino emergencia: 1000000 bps. "
            "FPGA Motor_CTRL: 115200 bps. Se ajusta al elegir el MCU."
        )
        if FACTORY_UI:
            self.baudrate_combo.setVisible(False)
            self.baudrate_combo.setCurrentText(str(BAUD_RATE))
            baud_lbl = QLabel(f"Enlace: {BAUD_RATE // 1000} kbps (fijo)")
            baud_lbl.setStyleSheet("color: #7F8C8D;")
            layout.addWidget(baud_lbl, 2, 0, 1, 3)
        else:
            layout.addWidget(QLabel("Baudrate:"), 2, 0)
            layout.addWidget(self.baudrate_combo, 2, 1, 1, 2)

        # Estado de conexión
        layout.addWidget(QLabel("Estado:"), 3, 0)
        self.connection_status = QLabel("❌ Desconectado")
        self.connection_status.setStyleSheet("font-weight: bold; color: #E74C3C;")
        layout.addWidget(self.connection_status, 3, 1, 1, 2)
        
        # Botón reconectar
        reconnect_btn = QPushButton("🔌 Conectar / Reconectar")
        reconnect_btn.setStyleSheet("""
            QPushButton { font-size: 12px; font-weight: bold; padding: 8px; background-color: #3498DB; }
            QPushButton:hover { background-color: #5DADE2; }
        """)
        reconnect_btn.clicked.connect(self._request_reconnect)
        layout.addWidget(reconnect_btn, 4, 0, 1, 3)
        
        group_box.setLayout(layout)
        return group_box

    def _on_mcu_changed(self, _index: int = 0):
        mcu_id = self.mcu_combo.currentData()
        if not mcu_id:
            return
        logger.info("ControlTab: perfil MCU -> %s", mcu_id)
        self.mcu_profile_changed.emit(str(mcu_id))
        self._apply_profile_ui(str(mcu_id))

    def _apply_profile_ui(self, mcu_id: str):
        """Baudios, rango de potencia y textos del perfil elegido."""
        prof = MCU_PROFILES.get(mcu_id)
        if prof is None:
            return
        self._power_max = int(prof["power_max"])
        self.baudrate_combo.setCurrentText(str(prof["baud"]))

        self.sensors_group.setTitle(_texts_for(_SENSOR_TITLES, mcu_id))

        value, placeholder, tip = _texts_for(_POWER_TEXTS, mcu_id)
        self.power_input.setText(value)
        self.power_input.setPlaceholderText(placeholder)
        self.power_input.setToolTip(tip)

        self.commands_info_label.setText(_texts_for(_COMMAND_TEXTS, mcu_id))
        self.manual_btn.setToolTip(_texts_for(_MANUAL_TIPS, mcu_id))

        power_a_text, power_b_text = _texts_for(_MOTOR_TEXTS, mcu_id)
        self.power_a_name_label.setText(power_a_text)
        self.power_b_name_label.setText(power_b_text)

        is_fpga = mcu_id == MCU_FPGA
        self.calib_box.setVisible(is_fpga)
        self.arrival_box.setVisible(is_fpga)
        for widget in (self.sensor1_name_label, self.value_labels['sensor_1'],
                       self.sensor2_name_label, self.value_labels['sensor_2']):
            widget.setVisible(not is_fpga)
        if not is_fpga:
            self._show_mode("MANUAL", "#E67E22")
        self._forget_state()

    def _forget_state(self):
        """Sin trama todavía: con la FPGA, Modo Actual espera el Estado de la próxima."""
        self._mcu_state = None
        self._last_settled = None
        if self.get_selected_mcu() == MCU_FPGA:
            self._show_mode("SIN TRAMA", _STATE_COLORS['UNKNOWN'])

    def get_selected_mcu(self) -> str:
        return str(self.mcu_combo.currentData() or MCU_TYPE)
    
    def _create_control_group(self):
        """Crea el panel de control de modos."""
        group_box = QGroupBox("Panel de Control")
        layout = QGridLayout()
        
        # Modo actual
        layout.addWidget(QLabel("Modo Actual:"), 0, 0)
        self.value_labels['mode'] = QLabel("MANUAL")
        self.value_labels['mode'].setStyleSheet("font-weight: bold; color: #E67E22; font-size: 14px;")
        layout.addWidget(self.value_labels['mode'], 0, 1)
        
        # Botón modo manual
        self.manual_btn = QPushButton("🔧 Activar MODO MANUAL")
        self.manual_btn.setStyleSheet("""
            QPushButton { font-size: 12px; font-weight: bold; padding: 8px; background-color: #E67E22; }
            QPushButton:hover { background-color: #F39C12; }
        """)
        self.manual_btn.clicked.connect(self.set_manual_mode)
        layout.addWidget(self.manual_btn, 1, 0, 1, 2)
        
        # Botón modo auto
        auto_btn = QPushButton("🤖 Activar MODO AUTO")
        auto_btn.setStyleSheet("""
            QPushButton { font-size: 12px; font-weight: bold; padding: 8px; background-color: #27AE60; }
            QPushButton:hover { background-color: #2ECC71; }
        """)
        auto_btn.clicked.connect(self.set_auto_mode)
        layout.addWidget(auto_btn, 2, 0, 1, 2)
        
        # Entrada de potencia
        layout.addWidget(QLabel("Potencia (A, B):"), 3, 0)
        self.power_input = QLineEdit()
        layout.addWidget(self.power_input, 3, 1)
        
        # Botón enviar potencia
        self.send_power_btn = QPushButton("⚡ Enviar Potencia (en modo AUTO)")
        self.send_power_btn.setStyleSheet("""
            QPushButton { font-size: 11px; font-weight: bold; padding: 6px; background-color: #3498DB; }
            QPushButton:hover { background-color: #5DADE2; }
        """)
        self.send_power_btn.clicked.connect(self._send_power_command)
        layout.addWidget(self.send_power_btn, 4, 0, 1, 2)

        self.calib_box = self._create_calib_box()
        layout.addWidget(self.calib_box, 5, 0, 1, 2)
        
        group_box.setLayout(layout)
        return group_box

    def _create_calib_box(self):
        """FPGA: 1 · Zero, 2 · Máximo, 3 · Manual y Reset mandan lo mismo que se escribe por COM."""
        box = QWidget()
        col = QVBoxLayout(box)
        col.setContentsMargins(0, 4, 0, 0)

        self.edit_points_check = QCheckBox("Editar puntos del recorrido")
        self.edit_points_check.setToolTip("Habilita Zero, Máximo y Reset.")
        col.addWidget(self.edit_points_check)

        button_style = """
            QPushButton { font-size: 11px; font-weight: bold; padding: 6px; background-color: %s; }
            QPushButton:hover { background-color: %s; }
            QPushButton:disabled { background-color: #555555; color: #AAAAAA; }
        """
        calib_colors = ("#16A085", "#1ABC9C")
        self.zero_btn = QPushButton("1 · Zero")
        self.final_btn = QPushButton("2 · Máximo")
        self.calib_manual_btn = QPushButton("3 · Manual")
        self.reset_btn = QPushButton("⟲ Reset")
        for btn, commands, colors in (
            (self.zero_btn, FPGA_ZERO, calib_colors),
            (self.final_btn, FPGA_FINAL, calib_colors),
            (self.reset_btn, FPGA_RESET, ("#C0392B", "#E74C3C")),
        ):
            btn.setStyleSheet(button_style % colors)
            btn.setToolTip(f"{', '.join(commands)}: {_CALIB_TIPS[commands]}")
            btn.setEnabled(False)
            btn.clicked.connect(lambda _checked=False, b=btn, c=commands: self._send_calib(b, c))
            self.edit_points_check.toggled.connect(btn.setEnabled)
        self.calib_manual_btn.setStyleSheet(button_style % calib_colors)
        self.calib_manual_btn.setToolTip(_MANUAL_TIPS[MCU_FPGA])
        self.calib_manual_btn.clicked.connect(self.set_manual_mode)

        row = QHBoxLayout()
        for btn in (self.zero_btn, self.final_btn, self.calib_manual_btn, self.reset_btn):
            row.addWidget(btn)
        col.addLayout(row)
        return box
    
    def _create_motors_group(self):
        """Crea el panel de estado de motores."""
        group_box = QGroupBox("Estado de Motores")
        layout = QGridLayout()
        value_style = "font-size: 18px; font-weight: bold; color: #5DADE2;"
        
        self.power_a_name_label = QLabel()
        layout.addWidget(self.power_a_name_label, 0, 0)
        self.value_labels['power_a'] = QLabel("0")
        self.value_labels['power_a'].setStyleSheet(value_style)
        layout.addWidget(self.value_labels['power_a'], 0, 1)
        
        self.power_b_name_label = QLabel()
        layout.addWidget(self.power_b_name_label, 1, 0)
        self.value_labels['power_b'] = QLabel("0")
        self.value_labels['power_b'].setStyleSheet(value_style)
        layout.addWidget(self.value_labels['power_b'], 1, 1)
        
        group_box.setLayout(layout)
        return group_box
    
    def _create_sensors_group(self):
        """Crea el panel de lectura de sensores."""
        group_box = QGroupBox()
        self.sensors_group = group_box
        layout = QGridLayout()
        value_style = "font-size: 18px; color: #58D68D;"
        
        self.sensor1_name_label = QLabel("Valor Sensor 1 (Y / PC3):")
        layout.addWidget(self.sensor1_name_label, 0, 0)
        self.value_labels['sensor_1'] = QLabel("---")
        self.value_labels['sensor_1'].setStyleSheet(value_style)
        layout.addWidget(self.value_labels['sensor_1'], 0, 1)
        
        self.sensor2_name_label = QLabel("Valor Sensor 2 (X / PA3):")
        layout.addWidget(self.sensor2_name_label, 1, 0)
        self.value_labels['sensor_2'] = QLabel("---")
        self.value_labels['sensor_2'].setStyleSheet(value_style)
        layout.addWidget(self.value_labels['sensor_2'], 1, 1)

        self.arrival_box = self._create_arrival_box()
        layout.addWidget(self.arrival_box, 2, 0, 1, 2)
        
        group_box.setLayout(layout)
        return group_box

    def _create_arrival_box(self):
        """FPGA: pedido (PotA/PotB), cuenta y error por eje, y grabación de tramas a CSV."""
        box = QWidget()
        grid = QGridLayout(box)
        grid.setContentsMargins(0, 0, 0, 0)
        head_style = "color: #95A5A6; font-weight: bold;"
        for col, text in enumerate(("Eje", "Pedido", "Cuenta", "Error (pedido − cuenta)")):
            head = QLabel(text)
            head.setStyleSheet(head_style)
            grid.addWidget(head, 0, col)

        self.arrival_labels = {}
        error_tip = (
            f"En MANUAL la FPGA lleva la cuenta al pedido: verde = a 1 paso o menos "
            f"({_FPGA_COUNTS_PER_EDGE} cuentas). En gris la FPGA no está posicionando "
            f"(RESET, AUTO, BRAKE)."
        )
        for row, axis in enumerate(("x", "y"), start=1):
            name = QLabel(axis.upper())
            name.setStyleSheet("font-size: 16px; font-weight: bold;")
            grid.addWidget(name, row, 0)
            for col, (field, style) in enumerate((
                ("pedido", "font-size: 16px; color: #5DADE2;"),
                ("cuenta", "font-size: 16px; color: #58D68D;"),
                ("error", "font-size: 16px; font-weight: bold; color: #95A5A6;"),
            ), start=1):
                lab = QLabel("---")
                lab.setStyleSheet(style)
                grid.addWidget(lab, row, col)
                self.arrival_labels[axis, field] = lab
            self.arrival_labels[axis, "error"].setToolTip(error_tip)

        self.frame_log_btn = QPushButton("⏺ Grabar tramas (CSV)")
        self.frame_log_btn.setCheckable(True)
        self.frame_log_btn.setToolTip(
            "Guarda cada trama de la FPGA (~41 por segundo) en CSVs/tramas_fpga: "
            "estado, potencia, pedido y cuenta de X e Y."
        )
        self.frame_log_btn.clicked.connect(self.frame_log_requested.emit)
        grid.addWidget(self.frame_log_btn, 3, 0, 1, 2)
        self.frame_log_label = QLabel("Sin grabar")
        self.frame_log_label.setStyleSheet("color: #95A5A6;")
        self.frame_log_label.setWordWrap(True)
        grid.addWidget(self.frame_log_label, 3, 2, 1, 2)
        return box

    def update_targets(self, target_x: int, target_y: int):
        """Pedido de la FPGA contra la cuenta medida, por eje."""
        in_manual = self._mcu_state == "MANUAL"
        for axis, target, count in (("x", target_x, self._last_xy[0]),
                                    ("y", target_y, self._last_xy[1])):
            err = target - count
            steps = f"{err / _FPGA_COUNTS_PER_EDGE:+.1f}".replace(".", ",")
            self.arrival_labels[axis, "pedido"].setText(str(target))
            self.arrival_labels[axis, "cuenta"].setText(str(count))
            error_label = self.arrival_labels[axis, "error"]
            error_label.setText(f"{err:+d} ({steps} pasos)")
            if not in_manual:
                color = "#95A5A6"
            elif abs(err) <= _FPGA_COUNTS_PER_EDGE:
                color = "#27AE60"
            else:
                color = "#E67E22"
            if self._error_colors.get(axis) != color:
                self._error_colors[axis] = color
                error_label.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {color};")

    def show_frame_log(self, active: bool, path: str, lines: int):
        self.frame_log_btn.setChecked(active)
        if active:
            self.frame_log_btn.setText("⏹ Detener grabación")
            self.frame_log_label.setText(f"Grabando: {path}")
            self.frame_log_label.setStyleSheet("color: #E74C3C; font-weight: bold;")
        else:
            self.frame_log_btn.setText("⏺ Grabar tramas (CSV)")
            self.frame_log_label.setText(
                f"Guardado: {path} ({lines} tramas)" if path else "Sin grabar"
            )
            self.frame_log_label.setStyleSheet("color: #95A5A6;")
    
    def _scan_ports(self):
        """Escanea puertos seriales disponibles y actualiza el combo."""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        
        if not ports:
            self.port_combo.addItem("No hay puertos disponibles")
            logger.warning("No se encontraron puertos seriales disponibles")
            return
        
        ctrl_index = -1
        keywords = (
            'stlink', 'st-link', 'stm', 'stmicroelectronics', 'virtual com',
            'arduino', 'ch340', 'ch341', 'ftdi', 'usb serial',
        )
        for i, port in enumerate(ports):
            # Mostrar puerto con descripción
            display = f"{port.device} - {port.description[:30]}"
            self.port_combo.addItem(display, port.device)
            
            desc_lower = port.description.lower()
            mfg = (port.manufacturer or '').lower()
            haystack = f"{desc_lower} {mfg}"
            if any(x in haystack for x in keywords):
                ctrl_index = i
        
        if ctrl_index >= 0:
            self.port_combo.setCurrentIndex(ctrl_index)
            logger.info(f"Controlador XY detectado en: {ports[ctrl_index].device}")
        
        logger.info(f"Puertos escaneados: {[p.device for p in ports]}")
    
    def _get_selected_port(self):
        """Obtiene el puerto seleccionado (solo el nombre del dispositivo)."""
        # El combo puede tener formato "COM3 - Arduino Mega" o solo "COM3"
        current_data = self.port_combo.currentData()
        if current_data:
            return current_data
        # Fallback: extraer del texto
        text = self.port_combo.currentText()
        if " - " in text:
            return text.split(" - ")[0]
        return text
    
    def _request_reconnect(self):
        """Solicita reconexión serial con los parámetros seleccionados."""
        port = self._get_selected_port()
        baudrate = int(self.baudrate_combo.currentText())
        
        logger.info(f"Solicitando reconexión serial: {port} @ {baudrate}")
        self.connection_status.setText("🔄 Conectando...")
        self.connection_status.setStyleSheet("font-weight: bold; color: #F39C12;")
        
        self.serial_reconnect_requested.emit(port, baudrate)
    
    def _send_power_command(self):
        """Envía comando de potencia DIRECTAMENTE al Arduino."""
        try:
            power_text = self.power_input.text()
            parts = power_text.split(',')
            if len(parts) != 2:
                logger.error("Formato inválido. Use: potencia_a,potencia_b")
                return
            
            power_a = int(parts[0].strip())
            power_b = int(parts[1].strip())
            
            # Validar rango del perfil: ±255 PWM o ±80 % (FPGA)
            lim = self._power_max
            power_a = max(-lim, min(lim, power_a))
            power_b = max(-lim, min(lim, power_b))
            
            self.send_power(power_a, power_b)
        except ValueError as e:
            logger.error(f"Error al parsear potencia: {e}")
    
    # === Métodos para actualizar estado desde el padre ===
    
    def set_mode(self, mode: str):
        """Actualiza el modo mostrado. Con la FPGA lo pone la trama."""
        if self.get_selected_mcu() == MCU_FPGA:
            return
        self._show_mode(mode, "#E67E22" if mode == "MANUAL" else "#27AE60")

    def _show_mode(self, text: str, color: str):
        self.value_labels['mode'].setText(text)
        self.value_labels['mode'].setStyleSheet(f"font-weight: bold; color: {color}; font-size: 14px;")
    
    def update_motor_values(self, power_a: int, power_b: int):
        """Actualiza los valores de potencia de motores."""
        self.value_labels['power_a'].setText(str(power_a))
        self.value_labels['power_b'].setText(str(power_b))
    
    def update_sensor_values(self, sensor_1: int, sensor_2: int):
        """Actualiza los valores de sensores."""
        self.value_labels['sensor_1'].setText(str(sensor_1))
        self.value_labels['sensor_2'].setText(str(sensor_2))
        self._last_xy = (sensor_2, sensor_1)    # X = Sensor1 de la FPGA = sensor_2
    
    def set_connection_status(self, connected: bool, port: str = ""):
        """
        Actualiza el estado de conexión serial.
        
        Args:
            connected: True si está conectado, False si no
            port: Puerto al que está conectado (opcional)
        """
        if connected:
            self.connection_status.setText(f"✅ Conectado ({port})")
            self.connection_status.setStyleSheet("font-weight: bold; color: #27AE60;")
            logger.info(f"Estado serial actualizado: Conectado a {port}")
        else:
            self.connection_status.setText("❌ Desconectado")
            self.connection_status.setStyleSheet("font-weight: bold; color: #E74C3C;")
            logger.info("Estado serial actualizado: Desconectado")
        self._forget_state()
    
    # ================================================================
    # LÓGICA DE CONTROL (movida desde main.py)
    # ================================================================
    
    def send_command(self, command: str):
        """Manda una orden al MCU. El SerialHandler la registra (TX) o avisa si el puerto está cerrado."""
        if self.serial_handler is None:
            logger.error("Sin SerialHandler: orden no enviada: %s", command)
            return
        self.serial_handler.send_command(command)

    def _send_calib(self, button: QPushButton, commands):
        """Una línea por orden, igual que escribirlas por COM."""
        x, y = self._last_xy
        logger.info("Calib: %s (X=%d, Y=%d)", button.text(), x, y)
        for command in commands:
            self.send_command(command)

    def set_manual_mode(self):
        """M: modo MANUAL (con la FPGA, los potes mandan la posición)."""
        logger.info("ControlTab: Activar MODO MANUAL")
        self.send_command(MotorProtocol.format_manual_mode())
        self.set_mode("MANUAL")

    def set_auto_mode(self):
        """A,0,0: modo AUTO con potencia 0."""
        logger.info("ControlTab: Activar MODO AUTO")
        self.send_command(MotorProtocol.format_power_command(0, 0))
        self.set_mode("AUTOMÁTICO")
    
    def send_power(self, power_a: int, power_b: int):
        """
        Envía comando de potencia a los motores.
        
        Args:
            power_a: Potencia motor A (-255 a 255)
            power_b: Potencia motor B (-255 a 255)
        """
        logger.info(f"ControlTab: Enviar Potencia - A={power_a}, B={power_b}")
        self.send_command(MotorProtocol.format_power_command(power_a, power_b))
        self.update_motor_values(power_a, power_b)

    def _create_mcu_status_group(self):
        """Estado del MCU según la trama, Settled y freno activo."""
        group_box = QGroupBox("Estado MCU / Freno")
        layout = QGridLayout()

        brake_btn = QPushButton("Freno Activo")
        brake_btn.setStyleSheet("""
            QPushButton { font-size: 12px; font-weight: bold; padding: 8px; background-color: #E74C3C; }
            QPushButton:hover { background-color: #C0392B; }
        """)
        brake_btn.clicked.connect(self._request_brake)
        layout.addWidget(brake_btn, 0, 0, 1, 4)

        layout.addWidget(QLabel("Estado MCU:"), 1, 0)
        self.arduino_state_label = QLabel("DESCONOCIDO")
        self.arduino_state_label.setStyleSheet("font-weight: bold; color: #95A5A6;")
        layout.addWidget(self.arduino_state_label, 1, 1)

        layout.addWidget(QLabel("Settled (info):"), 1, 2)
        self.settled_status_label = QLabel("NO")
        self.settled_status_label.setStyleSheet("font-weight: bold; color: #E74C3C;")
        layout.addWidget(self.settled_status_label, 1, 3)

        self.commands_info_label = QLabel()
        self.commands_info_label.setStyleSheet("color: #7F8C8D; font-size: 10px;")
        layout.addWidget(self.commands_info_label, 2, 0, 1, 4)

        self.firmware_status_label = QLabel("Firmware: Esperando telemetría...")
        self.firmware_status_label.setStyleSheet("color: #F39C12; font-size: 10px; font-weight: bold;")
        layout.addWidget(self.firmware_status_label, 3, 0, 1, 4)

        group_box.setLayout(layout)
        return group_box

    def _request_brake(self):
        """Solicita freno activo."""
        logger.info("ControlTab: Solicitar Freno Activo")
        self.brake_requested.emit()

    def update_arduino_status(self, state: str, settled: bool):
        """Estado del MCU y Settled. Con la FPGA, Modo Actual es el Estado de la trama."""
        state = state.upper()
        state_changed = state != self._mcu_state
        # La FPGA cambia Settled sin cambiar de estado.
        if not state_changed and settled == self._last_settled:
            return
        self._mcu_state = state
        self._last_settled = settled

        color = _STATE_COLORS.get(state, _STATE_COLORS['UNKNOWN'])
        self.arduino_state_label.setText(state)
        self.arduino_state_label.setStyleSheet(f"font-weight: bold; color: {color};")
        if settled:
            self.settled_status_label.setText("SI")
            self.settled_status_label.setStyleSheet("font-weight: bold; color: #27AE60;")
        else:
            self.settled_status_label.setText("NO")
            self.settled_status_label.setStyleSheet("font-weight: bold; color: #E74C3C;")

        if not state_changed:
            return
        logger.info(f"ControlTab: Estado MCU cambiado a {state}, Settled={settled}")
        if self.get_selected_mcu() == MCU_FPGA:
            self._show_mode(state, color)

        profile_label = MCU_PROFILES.get(self.get_selected_mcu(), {}).get("label", "MCU")
        if state == 'LEGACY':
            self.firmware_status_label.setText("Firmware LEGACY 4 campos - preferir STM32 6 campos")
            self.firmware_status_label.setStyleSheet("color: #E74C3C; font-size: 10px; font-weight: bold;")
        elif state == 'RESET':
            self.firmware_status_label.setText(
                f"{profile_label} - RESET: sin calibrar; los potes no mueven X ni Y "
                f"(1 · Zero, 2 · Máximo, 3 · Manual)"
            )
            self.firmware_status_label.setStyleSheet("color: #E67E22; font-size: 10px; font-weight: bold;")
        elif state in ('MANUAL', 'AUTO', 'BRAKE', 'PULSE', 'PC'):
            self.firmware_status_label.setText(f"{profile_label} - estado {state} OK")
            self.firmware_status_label.setStyleSheet("color: #27AE60; font-size: 10px; font-weight: bold;")
        else:
            self.firmware_status_label.setText(f"Firmware: estado {state}")
            self.firmware_status_label.setStyleSheet("color: #F39C12; font-size: 10px; font-weight: bold;")

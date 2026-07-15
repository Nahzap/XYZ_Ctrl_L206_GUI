"""
Pestaña de Control de Motores.

Encapsula la UI para control manual/automático de motores y visualización de sensores.
"""

import logging
import serial.tools.list_ports
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QGroupBox, QLabel, QLineEdit, QPushButton, QComboBox)
from PyQt5.QtCore import pyqtSignal

from config.constants import BAUD_RATE, FACTORY_UI

logger = logging.getLogger('MotorControl_L206')


class ControlTab(QWidget):
    """
    Pestaña para control de motores y visualización de sensores.
    
    Signals:
        manual_mode_requested: Solicita cambio a modo manual
        auto_mode_requested: Solicita cambio a modo automático
        power_command_requested: Solicita envío de potencia (power_a, power_b)
    """
    
    manual_mode_requested = pyqtSignal()
    auto_mode_requested = pyqtSignal()
    power_command_requested = pyqtSignal(int, int)  # power_a, power_b
    serial_reconnect_requested = pyqtSignal(str, int)  # puerto, baudrate
    
    # --- NUEVAS SEÑALES PARA POSITION HOLD ---
    position_hold_requested = pyqtSignal(int, int)  # sensor1_target, sensor2_target
    brake_requested = pyqtSignal()
    settling_config_requested = pyqtSignal(int)  # threshold
    
    def __init__(self, serial_handler=None, parent=None):
        """
        Inicializa la pestaña de control.
        
        Args:
            serial_handler: Instancia de SerialHandler para comunicación
            parent: Widget padre (CTRL_GUI)
        """
        super().__init__(parent)
        self.parent_gui = parent
        self.serial_handler = serial_handler
        self.value_labels = {}
        self._setup_ui()
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
        
        # --- NUEVO: Position Hold para Testing ---
        position_hold_group = self._create_position_hold_group()
        layout.addWidget(position_hold_group)
        
        layout.addStretch()
    
    def _create_serial_config_group(self):
        """Crea el panel de configuración serial."""
        group_box = QGroupBox("⚙️ Configuración Serial")
        layout = QGridLayout()
        
        # Puerto COM con detección automática
        layout.addWidget(QLabel("Puerto:"), 0, 0)
        self.port_combo = QComboBox()
        self.port_combo.setToolTip("Selecciona el puerto serial del controlador XY (STM32 ST-Link VCP)")
        layout.addWidget(self.port_combo, 0, 1)
        
        # Botón escanear puertos
        scan_btn = QPushButton("🔄")
        scan_btn.setFixedWidth(40)
        scan_btn.setToolTip("Escanear puertos disponibles")
        scan_btn.clicked.connect(self._scan_ports)
        layout.addWidget(scan_btn, 0, 2)
        
        # Escanear puertos al inicializar
        self._scan_ports()
        
        # Baudrate: fábrica fija 1 Mbps (Fase 5.3); lab puede cambiar.
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(['9600', '19200', '38400', '57600', '115200', '230400', '1000000'])
        self.baudrate_combo.setCurrentText(str(BAUD_RATE))
        self.baudrate_combo.setToolTip("Velocidad de comunicación serial")
        if FACTORY_UI:
            self.baudrate_combo.setVisible(False)
            self.baudrate_combo.setCurrentText(str(BAUD_RATE))
            baud_lbl = QLabel(f"Enlace: {BAUD_RATE // 1000} kbps (fijo)")
            baud_lbl.setStyleSheet("color: #7F8C8D;")
            layout.addWidget(baud_lbl, 1, 0, 1, 3)
        else:
            layout.addWidget(QLabel("Baudrate:"), 1, 0)
            layout.addWidget(self.baudrate_combo, 1, 1, 1, 2)

        # Estado de conexión
        layout.addWidget(QLabel("Estado:"), 2, 0)
        self.connection_status = QLabel("❌ Desconectado")
        self.connection_status.setStyleSheet("font-weight: bold; color: #E74C3C;")
        layout.addWidget(self.connection_status, 2, 1, 1, 2)
        
        # Botón reconectar
        reconnect_btn = QPushButton("🔌 Conectar / Reconectar")
        reconnect_btn.setStyleSheet("""
            QPushButton { font-size: 12px; font-weight: bold; padding: 8px; background-color: #3498DB; }
            QPushButton:hover { background-color: #5DADE2; }
        """)
        reconnect_btn.clicked.connect(self._request_reconnect)
        layout.addWidget(reconnect_btn, 3, 0, 1, 3)
        
        group_box.setLayout(layout)
        return group_box
    
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
        manual_btn = QPushButton("🔧 Activar MODO MANUAL")
        manual_btn.setStyleSheet("""
            QPushButton { font-size: 12px; font-weight: bold; padding: 8px; background-color: #E67E22; }
            QPushButton:hover { background-color: #F39C12; }
        """)
        manual_btn.clicked.connect(self._request_manual_mode)
        layout.addWidget(manual_btn, 1, 0, 1, 2)
        
        # Botón modo auto
        auto_btn = QPushButton("🤖 Activar MODO AUTO")
        auto_btn.setStyleSheet("""
            QPushButton { font-size: 12px; font-weight: bold; padding: 8px; background-color: #27AE60; }
            QPushButton:hover { background-color: #2ECC71; }
        """)
        auto_btn.clicked.connect(self._request_auto_mode)
        layout.addWidget(auto_btn, 2, 0, 1, 2)
        
        # Entrada de potencia
        layout.addWidget(QLabel("Potencia (A, B):"), 3, 0)
        self.power_input = QLineEdit("100,-100")
        self.power_input.setPlaceholderText("Ej: 100,-100")
        self.power_input.setToolTip("Valores de potencia para Motor A y Motor B (-255 a 255)")
        layout.addWidget(self.power_input, 3, 1)
        
        # Botón enviar potencia
        send_power_btn = QPushButton("⚡ Enviar Potencia (en modo AUTO)")
        send_power_btn.setStyleSheet("""
            QPushButton { font-size: 11px; font-weight: bold; padding: 6px; background-color: #3498DB; }
            QPushButton:hover { background-color: #5DADE2; }
        """)
        send_power_btn.clicked.connect(self._send_power_command)
        layout.addWidget(send_power_btn, 4, 0, 1, 2)
        
        group_box.setLayout(layout)
        return group_box
    
    def _create_motors_group(self):
        """Crea el panel de estado de motores."""
        group_box = QGroupBox("Estado de Motores")
        layout = QGridLayout()
        value_style = "font-size: 18px; font-weight: bold; color: #5DADE2;"
        
        layout.addWidget(QLabel("Potencia Motor A:"), 0, 0)
        self.value_labels['power_a'] = QLabel("0")
        self.value_labels['power_a'].setStyleSheet(value_style)
        layout.addWidget(self.value_labels['power_a'], 0, 1)
        
        layout.addWidget(QLabel("Potencia Motor B:"), 1, 0)
        self.value_labels['power_b'] = QLabel("0")
        self.value_labels['power_b'].setStyleSheet(value_style)
        layout.addWidget(self.value_labels['power_b'], 1, 1)
        
        group_box.setLayout(layout)
        return group_box
    
    def _create_sensors_group(self):
        """Crea el panel de lectura de sensores."""
        group_box = QGroupBox("Lectura de Sensores Análogos")
        layout = QGridLayout()
        value_style = "font-size: 18px; color: #58D68D;"
        
        layout.addWidget(QLabel("Valor Sensor 1 (Y / PC3):"), 0, 0)
        self.value_labels['sensor_1'] = QLabel("---")
        self.value_labels['sensor_1'].setStyleSheet(value_style)
        layout.addWidget(self.value_labels['sensor_1'], 0, 1)
        
        layout.addWidget(QLabel("Valor Sensor 2 (X / PA3):"), 1, 0)
        self.value_labels['sensor_2'] = QLabel("---")
        self.value_labels['sensor_2'].setStyleSheet(value_style)
        layout.addWidget(self.value_labels['sensor_2'], 1, 1)
        
        group_box.setLayout(layout)
        return group_box
    
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
    
    def _request_manual_mode(self):
        """Cambia a modo manual."""
        self.set_manual_mode()
    
    def _request_auto_mode(self):
        """Cambia a modo automático."""
        self.set_auto_mode()
    
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
            
            # Validar rango
            power_a = max(-255, min(255, power_a))
            power_b = max(-255, min(255, power_b))
            
            # ENVIAR DIRECTAMENTE AL ARDUINO (formato: A,potA,potB)
            self.send_power(power_a, power_b)
            logger.debug(f"Comando de potencia ENVIADO: A={power_a}, B={power_b}")
        except ValueError as e:
            logger.error(f"Error al parsear potencia: {e}")
    
    # === Métodos para actualizar estado desde el padre ===
    
    def set_mode(self, mode: str):
        """Actualiza el modo mostrado."""
        self.value_labels['mode'].setText(mode)
        if mode == "MANUAL":
            self.value_labels['mode'].setStyleSheet("font-weight: bold; color: #E67E22; font-size: 14px;")
        else:
            self.value_labels['mode'].setStyleSheet("font-weight: bold; color: #27AE60; font-size: 14px;")
    
    def update_motor_values(self, power_a: int, power_b: int):
        """Actualiza los valores de potencia de motores."""
        self.value_labels['power_a'].setText(str(power_a))
        self.value_labels['power_b'].setText(str(power_b))
    
    def update_sensor_values(self, sensor_1: int, sensor_2: int):
        """Actualiza los valores de sensores."""
        self.value_labels['sensor_1'].setText(str(sensor_1))
        self.value_labels['sensor_2'].setText(str(sensor_2))
    
    def get_value_labels(self):
        """Retorna el diccionario de labels para compatibilidad."""
        return self.value_labels
    
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
    
    # ================================================================
    # LÓGICA DE CONTROL (movida desde main.py)
    # ================================================================
    
    def send_command(self, command: str):
        """
        Envía comando al Arduino vía serial.
        
        Args:
            command: Comando a enviar
        """
        if self.serial_handler and self.serial_handler.ser and self.serial_handler.ser.is_open:
            try:
                self.serial_handler.send_command(command)
                logger.info(f"Comando enviado: {command}")
            except Exception as e:
                logger.error(f"Error al enviar comando: {e}")
        else:
            logger.error("Puerto serial no está abierto. Comando no enviado.")
    
    def set_manual_mode(self):
        """Activa modo MANUAL en el Arduino."""
        logger.info("ControlTab: Activar MODO MANUAL")
        self.send_command('M')
        self.set_mode("MANUAL")
        logger.debug("Modo MANUAL activado")
    
    def set_auto_mode(self):
        """Activa modo AUTOMÁTICO en el Arduino con potencia inicial 0,0."""
        logger.info("ControlTab: Activar MODO AUTO")
        # Arduino espera formato: A,potA,potB
        # Enviamos A,0,0 para activar modo AUTO con potencia 0
        self.send_command('A,0,0')
        self.set_mode("AUTOMÁTICO")
        logger.debug("Modo AUTOMÁTICO activado")
    
    def send_power(self, power_a: int, power_b: int):
        """
        Envía comando de potencia a los motores.
        
        Args:
            power_a: Potencia motor A (-255 a 255)
            power_b: Potencia motor B (-255 a 255)
        """
        logger.info(f"ControlTab: Enviar Potencia - A={power_a}, B={power_b}")
        command_string = f"A,{power_a},{power_b}"
        self.send_command(command_string)
        self.update_motor_values(power_a, power_b)
    
    def _create_position_hold_group(self):
        """Panel de estado MCU + freno. Hold/S deshabilitados (STM32 no implementa H/S)."""
        group_box = QGroupBox("Estado MCU STM32 / Freno")
        layout = QGridLayout()
        
        layout.addWidget(QLabel("Target Sensor 1 (ADC):"), 0, 0)
        self.sensor1_target_input = QLineEdit("2048")
        self.sensor1_target_input.setEnabled(False)
        self.sensor1_target_input.setToolTip("Hold no disponible en firmware STM32")
        layout.addWidget(self.sensor1_target_input, 0, 1)
        
        layout.addWidget(QLabel("Target Sensor 2 (ADC):"), 0, 2)
        self.sensor2_target_input = QLineEdit("2048")
        self.sensor2_target_input.setEnabled(False)
        self.sensor2_target_input.setToolTip("Hold no disponible en firmware STM32")
        layout.addWidget(self.sensor2_target_input, 0, 3)
        
        hold_btn = QPushButton("Position Hold (N/A)")
        hold_btn.setEnabled(False)
        hold_btn.setToolTip("Firmware STM32 no soporta H,<s1>,<s2>. Usar control PC vía A,<pwm>.")
        hold_btn.setStyleSheet("""
            QPushButton { font-size: 12px; font-weight: bold; padding: 8px; background-color: #555555; color: #AAAAAA; }
        """)
        layout.addWidget(hold_btn, 1, 0)
        
        brake_btn = QPushButton("Freno Activo")
        brake_btn.setStyleSheet("""
            QPushButton { font-size: 12px; font-weight: bold; padding: 8px; background-color: #E74C3C; }
            QPushButton:hover { background-color: #C0392B; }
        """)
        brake_btn.clicked.connect(self._request_brake)
        layout.addWidget(brake_btn, 1, 1)
        
        layout.addWidget(QLabel("Umbral Asentamiento:"), 1, 2)
        self.settling_threshold_input = QLineEdit("32")
        self.settling_threshold_input.setEnabled(False)
        self.settling_threshold_input.setToolTip("Comando S no soportado; settling en PC")
        layout.addWidget(self.settling_threshold_input, 1, 3)
        
        config_btn = QPushButton("Configurar (N/A)")
        config_btn.setEnabled(False)
        config_btn.setToolTip("Firmware STM32 no soporta S,<threshold>")
        config_btn.setStyleSheet("""
            QPushButton { font-size: 11px; padding: 6px; background-color: #555555; color: #AAAAAA; }
        """)
        layout.addWidget(config_btn, 1, 4)
        
        layout.addWidget(QLabel("Estado MCU:"), 2, 0)
        self.arduino_state_label = QLabel("DESCONOCIDO")
        self.arduino_state_label.setStyleSheet("font-weight: bold; color: #95A5A6;")
        layout.addWidget(self.arduino_state_label, 2, 1)
        
        layout.addWidget(QLabel("Settled (info):"), 2, 2)
        self.settled_status_label = QLabel("NO")
        self.settled_status_label.setStyleSheet("font-weight: bold; color: #E74C3C;")
        layout.addWidget(self.settled_status_label, 2, 3)
        
        info_label = QLabel("Comandos MCU: M | A,<pwm_a>,<pwm_b> | B. Hold/S deshabilitados.")
        info_label.setStyleSheet("color: #7F8C8D; font-size: 10px;")
        layout.addWidget(info_label, 3, 0, 1, 5)
        
        self.firmware_status_label = QLabel("Firmware: Esperando telemetría STM32...")
        self.firmware_status_label.setStyleSheet("color: #F39C12; font-size: 10px; font-weight: bold;")
        layout.addWidget(self.firmware_status_label, 4, 0, 1, 5)
        
        group_box.setLayout(layout)
        return group_box
    
    def _request_position_hold(self):
        """No-op: Hold no soportado en STM32."""
        logger.warning("Position Hold solicitado pero deshabilitado (STM32 sin comando H)")
    
    def _request_brake(self):
        """Solicita freno activo."""
        logger.info("ControlTab: Solicitar Freno Activo")
        self.brake_requested.emit()
    
    def _request_settling_config(self):
        """No-op: S no soportado en STM32."""
        logger.warning("Settling config solicitado pero deshabilitado (STM32 sin comando S)")
    
    def update_arduino_status(self, state: str, settled: bool):
        """Actualiza el estado del MCU y flag settled (informativo)."""
        current_state = self.arduino_state_label.text()
        if current_state == state.upper():
            return
        
        self.arduino_state_label.setText(state.upper())
        
        state_colors = {
            'MANUAL': '#3498DB',
            'AUTO': '#9B59B6', 
            'HOLD': '#27AE60',
            'BRAKE': '#E74C3C',
            'SETTLING': '#F39C12',
            'UNKNOWN': '#95A5A6',
            'LEGACY': '#F39C12'
        }
        color = state_colors.get(state.upper(), '#95A5A6')
        self.arduino_state_label.setStyleSheet(f"font-weight: bold; color: {color};")
        
        if settled:
            self.settled_status_label.setText("SI")
            self.settled_status_label.setStyleSheet("font-weight: bold; color: #27AE60;")
        else:
            self.settled_status_label.setText("NO")
            self.settled_status_label.setStyleSheet("font-weight: bold; color: #E74C3C;")
        
        logger.info(f"ControlTab: Estado MCU cambiado a {state}, Settled={settled}")
        
        if hasattr(self, 'firmware_status_label'):
            if state.upper() == 'LEGACY':
                self.firmware_status_label.setText("Firmware LEGACY 4 campos - preferir STM32 6 campos")
                self.firmware_status_label.setStyleSheet("color: #E74C3C; font-size: 10px; font-weight: bold;")
            elif state.upper() in ['MANUAL', 'AUTO', 'BRAKE']:
                self.firmware_status_label.setText("STM32F767ZI - M/A/B OK (Hold/S N/A)")
                self.firmware_status_label.setStyleSheet("color: #27AE60; font-size: 10px; font-weight: bold;")
            else:
                self.firmware_status_label.setText(f"Firmware: estado {state}")
                self.firmware_status_label.setStyleSheet("color: #F39C12; font-size: 10px; font-weight: bold;")

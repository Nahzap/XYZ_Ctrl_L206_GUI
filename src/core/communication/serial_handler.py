"""Manejo de comunicación serial asíncrona."""
import serial
import time
import logging
import traceback
from PyQt5.QtCore import QThread, pyqtSignal

logger = logging.getLogger(__name__)


class SerialHandler(QThread):
    """
    Thread para lectura serial asíncrona con BUFFER CIRCULAR.
    
    Reconstruye líneas completas antes de emitirlas para evitar
    datos corruptos a alta velocidad (1 Mbps).
    
    Signals:
        data_received (str): Emite cada línea COMPLETA recibida del Arduino
    """
    data_received = pyqtSignal(str)

    def __init__(self, port, baudrate):
        """
        Inicializa el handler de comunicación serial.
        
        Args:
            port (str): Puerto serial (ej: 'COM3', '/dev/ttyUSB0')
            baudrate (int): Velocidad de comunicación (ej: 115200)
        """
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.ser = None
        # Buffer circular para reconstruir líneas completas
        self._buffer = ""
        logger.info(f"SerialHandler inicializado: Puerto={port}, Baudrate={baudrate}")

    def run(self):
        """
        LOOP DE LECTURA SERIAL CON BUFFER CIRCULAR.
        Reconstruye líneas completas para evitar datos corruptos.
        """
        logger.debug(f"Iniciando thread de lectura serial en {self.port}")
        try:
            # Puerto serial - timeout pequeño para lectura eficiente
            self.ser = serial.Serial(
                port=self.port, 
                baudrate=self.baudrate, 
                timeout=0.001,  # 1ms - permite leer chunks
                write_timeout=0
            )
            logger.info(f"Puerto {self.port} @ {self.baudrate} bps - Buffer circular activo")
            
            # Espera inicial para Arduino
            time.sleep(0.1)
            self._buffer = ""  # Limpiar buffer
            self.data_received.emit("INFO: Conectado exitosamente.")
            
            # ================================================================
            # LOOP PRINCIPAL CON BUFFER CIRCULAR
            # ================================================================
            while self.running:
                try:
                    # Leer bytes disponibles
                    if self.ser.in_waiting:
                        # Leer chunk de datos (más eficiente que readline)
                        chunk = self.ser.read(self.ser.in_waiting)
                        if chunk:
                            # Agregar al buffer
                            self._buffer += chunk.decode('utf-8', errors='ignore')
                            
                            # Procesar líneas completas (terminan en \n)
                            while '\n' in self._buffer:
                                line, self._buffer = self._buffer.split('\n', 1)
                                line = line.strip()
                                if line:
                                    self.data_received.emit(line)
                            
                            # Evitar que el buffer crezca indefinidamente
                            if len(self._buffer) > 200:
                                self._buffer = ""
                except:
                    if not self.running:
                        break
            
            # Cierre limpio
            if self.ser and self.ser.is_open:
                self.ser.close()
                logger.info("Puerto serial cerrado")
        except serial.SerialException as e:
            logger.error(f"Error al abrir puerto {self.port}: {e}")
            self.data_received.emit(f"ERROR: Puerto {self.port} no encontrado.")
        except Exception as e:
            logger.critical(f"Error inesperado en SerialHandler: {e}\n{traceback.format_exc()}")

    def stop(self):
        """Detiene el thread de lectura serial de forma segura."""
        logger.debug("Deteniendo SerialHandler")
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("Puerto serial cerrado en stop()")
        self.wait()
    
    def write(self, data):
        """
        Envía datos al puerto serial (bytes).
        
        Args:
            data (bytes): Datos a enviar
            
        Returns:
            bool: True si se envió exitosamente, False si no
        """
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data)
                return True
            except Exception as e:
                logger.error(f"Error escribiendo al serial: {e}")
                return False
        return False
    
    def send_command(self, command):
        """
        Envía un comando string al Arduino.
        
        Args:
            command (str): Comando a enviar (ej: 'A,100,0' o 'M')
            
        Returns:
            bool: True si se envió exitosamente
        """
        if self.ser and self.ser.is_open:
            try:
                full_command = command + '\n'
                self.ser.write(full_command.encode('utf-8'))
                logger.debug(f"Comando enviado: {command}")
                return True
            except Exception as e:
                logger.error(f"Error enviando comando: {e}")
                return False
        else:
            logger.warning("Puerto no abierto, comando no enviado")
            return False

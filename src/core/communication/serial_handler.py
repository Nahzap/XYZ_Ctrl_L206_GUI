"""Manejo de comunicación serial asíncrona."""
import serial
import time
import logging
import threading
import traceback
from PyQt5.QtCore import QThread, pyqtSignal

from core.communication.protocol import MotorProtocol
from core.communication.serial_tx_queue import SerialTxQueue

logger = logging.getLogger(__name__)


class SerialHandler(QThread):
    """
    Thread RX serial + cola TX gestionada.

    Regla dura: el hilo RX prioriza SIEMPRE la lectura. Escribir TX desde este
    hilo antes de leer ahoga la telemetría (síntoma: err X/Y clavados con PWM
    subiendo). El drain TX en este hilo solo corre cuando no hay bytes RX.

    send_command() encola (coalesce A,*) y drena desde el hilo del llamador
    bajo lock — así I/F del handoff salen al instante sin tocar el RX.
    """
    data_received = pyqtSignal(str)

    def __init__(self, port, baudrate):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.ser = None
        self._buffer = ""
        self.sensor_buffer = None
        self._tx_queue = SerialTxQueue()
        self._tx_lock = threading.Lock()
        self._last_tx_stats_log = 0.0
        logger.info(f"SerialHandler inicializado: Puerto={port}, Baudrate={baudrate}")

    def set_sensor_buffer(self, sensor_buffer):
        """Conecta el SensorBuffer que se actualizará en el hilo RX (plano máquina)."""
        self.sensor_buffer = sensor_buffer

    def _update_sensor_buffer(self, line):
        """Parsea telemetría y actualiza el buffer en el hilo RX (tasa completa)."""
        if self.sensor_buffer is None:
            return
        if not line or line[0] not in '-0123456789':
            return
        try:
            parts = line.split(',')
            if len(parts) >= 6:
                parsed = MotorProtocol.parse_sensor_data_with_status(line)
                if not parsed:
                    return
                self.sensor_buffer.update(
                    parsed['sens_1'], parsed['sens_2'],
                    pot_a=parsed['pot_a'], pot_b=parsed['pot_b'],
                    settled=bool(parsed.get('settled', False)),
                    state=str(parsed.get('state', '')),
                )
            elif len(parts) == 4:
                pot_a, pot_b, sens_1, sens_2 = map(int, parts)
                self.sensor_buffer.update(
                    sens_1, sens_2, pot_a=pot_a, pot_b=pot_b,
                    settled=False, state='LEGACY',
                )
        except (ValueError, IndexError):
            return

    def _drain_tx_queue(self, max_n: int = 8) -> int:
        """Escribe hasta max_n comandos pendientes. Retorna cuántos envió."""
        if not self.ser or not self.ser.is_open:
            return 0
        batch = self._tx_queue.pop_batch(max_n=max_n)
        if not batch:
            return 0
        sent = 0
        try:
            with self._tx_lock:
                payload = "".join(cmd + "\n" for cmd in batch).encode("utf-8")
                self.ser.write(payload)
                sent = len(batch)
        except Exception as e:
            logger.error(f"Error drenando TX serial: {e}")
            return sent
        for cmd in batch:
            if cmd.startswith("A,") or cmd.startswith("a,"):
                logger.debug("TX auto: %s", cmd)
            else:
                logger.info("TX control: %s", cmd)
        now = time.perf_counter()
        if (now - self._last_tx_stats_log) >= 5.0:
            self._last_tx_stats_log = now
            st = self._tx_queue.stats()
            if st["dropped_a"] or st["pending_ctrl"]:
                logger.info(
                    "SerialTX stats enq=%d drain=%d dropA=%d pendCtrl=%d pendA=%s",
                    st["enqueued"],
                    st["drained"],
                    st["dropped_a"],
                    st["pending_ctrl"],
                    st["pending_a"],
                )
        return sent

    def _ingest_rx(self) -> int:
        """Lee todo lo pendiente en el puerto. Retorna líneas completas vistas."""
        if not self.ser or not self.ser.is_open:
            return 0
        waiting = self.ser.in_waiting
        if not waiting:
            return 0
        chunk = self.ser.read(waiting)
        if not chunk:
            return 0
        self._buffer += chunk.decode("utf-8", errors="ignore")
        lines = 0
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            lines += 1
            self._update_sensor_buffer(line)
            self.data_received.emit(line)
        # Fragmento incompleto: no borrar a ciegas (antes >200 vaciaba y perdía CSV).
        if len(self._buffer) > 4096:
            logger.warning("RX buffer overflow (%d B) — reset parcial", len(self._buffer))
            self._buffer = ""
        return lines

    def run(self):
        """Loop RX prioritario + drain TX solo en idle."""
        logger.debug(f"Iniciando thread de lectura serial en {self.port}")
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.001,
                write_timeout=0.05,
            )
            logger.info(
                f"Puerto {self.port} @ {self.baudrate} bps — RX-first + TX queue"
            )
            time.sleep(0.1)
            self._buffer = ""
            self._tx_queue.clear()
            self.data_received.emit("INFO: Conectado exitosamente.")

            while self.running:
                try:
                    # 1) RX primero (nunca sacrificar sensores por TX).
                    n = self._ingest_rx()
                    # 2) TX solo si el cable está quieto (o backlog de control).
                    if n == 0:
                        pending = self._tx_queue.pending_count()
                        if pending:
                            self._drain_tx_queue(max_n=4)
                        else:
                            time.sleep(0.0005)
                except Exception as e:
                    if not self.running:
                        break
                    logger.error("SerialHandler loop: %s", e)
                    time.sleep(0.01)

            if self.ser and self.ser.is_open:
                self.ser.close()
                logger.info("Puerto serial cerrado")
        except serial.SerialException as e:
            logger.error(f"Error al abrir puerto {self.port}: {e}")
            self.data_received.emit(f"ERROR: Puerto {self.port} no encontrado.")
        except Exception as e:
            logger.critical(
                f"Error inesperado en SerialHandler: {e}\n{traceback.format_exc()}"
            )

    def stop(self):
        """Detiene el thread de lectura serial de forma segura."""
        logger.debug("Deteniendo SerialHandler")
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("Puerto serial cerrado en stop()")
        self.wait()

    def write(self, data):
        """Envía bytes crudos (bypass cola). Preferir send_command()."""
        if self.ser and self.ser.is_open:
            try:
                with self._tx_lock:
                    self.ser.write(data)
                return True
            except Exception as e:
                logger.error(f"Error escribiendo al serial: {e}")
                return False
        return False

    def send_command(self, command):
        """
        Encola comando (A,* coalesce; F/I/N/B prioritarios) y drena ya.

        El drain corre en el hilo del llamador (control/GUI), no en el RX,
        para no bloquear la telemetría.
        """
        if not (self.ser and self.ser.is_open):
            logger.warning("Puerto no abierto, comando no encolado: %s", command)
            return False
        self._tx_queue.enqueue(command)
        self._drain_tx_queue(max_n=16)
        return True

    def tx_queue_stats(self) -> dict:
        return self._tx_queue.stats()

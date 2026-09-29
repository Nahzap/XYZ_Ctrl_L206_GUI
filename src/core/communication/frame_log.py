"""CSV con cada trama de la FPGA a tasa completa: estado, potencia, pedido y cuenta por eje."""

import os
import time
from datetime import datetime

HEADER = "t_s,estado,settled,potencia_x,potencia_y,pedido_x,pedido_y,cuenta_x,cuenta_y"


class FrameLog:
    def __init__(self, directory: str):
        self.directory = directory
        self.path = ""
        self.lines = 0
        self._f = None
        self._t0 = 0.0

    @property
    def active(self) -> bool:
        return self._f is not None

    def start(self) -> str:
        os.makedirs(self.directory, exist_ok=True)
        name = f"tramas_fpga_{datetime.now():%Y%m%d_%H%M%S}.csv"
        self.path = os.path.join(self.directory, name)
        self._f = open(self.path, "w", encoding="utf-8", newline="")
        self._f.write(HEADER + "\n")
        self._t0 = time.perf_counter()
        self.lines = 0
        return self.path

    def write(self, parsed: dict) -> None:
        """parsed = MotorProtocol.parse_sensor_data_with_status (X = sens_2, Y = sens_1)."""
        if self._f is None:
            return
        t = time.perf_counter() - self._t0
        self._f.write(
            f"{t:.4f},{parsed['state']},{int(bool(parsed['settled']))},"
            f"{parsed['pot_a']},{parsed['pot_b']},"
            f"{parsed['target_x']},{parsed['target_y']},"
            f"{parsed['sens_2']},{parsed['sens_1']}\n"
        )
        self.lines += 1

    def stop(self):
        """Cierra el archivo. Retorna (ruta, tramas escritas)."""
        if self._f is not None:
            self._f.close()
            self._f = None
        return self.path, self.lines

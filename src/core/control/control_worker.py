"""Reloj de control desacoplado del hilo GUI (Fase 1, sub-pasos 1.1/1.3).

`ControlWorker` ejecuta un callback `tick` a una tasa objetivo en su PROPIO
`QThread`, con temporización de alta resolución (`perf_counter` + `sleep`).
Sustituye al `QTimer(10)` que corría sobre el hilo de la interfaz: así el
repintado de la UI no introduce jitter en el lazo, y se puede subir la tasa
por encima de 100 Hz (necesario para los pulsos fine en ms de la Fase 2).

Contrato del callback:
- Debe ser thread-safe: solo puede leer el `SensorBuffer`, enviar comandos por
  la vía TX con lock y **emitir señales Qt** (que se encolan al hilo GUI).
- NO debe tocar widgets directamente.

Ver: Docs/20260714_0032_Plan_Implementacion_Control_Micrometrico_Rapido.md
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Optional

from PyQt5.QtCore import QThread

logger = logging.getLogger('MotorControl_L206')


class ControlWorker(QThread):
    """Hilo de reloj para un lazo de control periódico y determinista."""

    def __init__(
        self,
        tick: Callable[[], None],
        rate_hz: float = 200.0,
        name: str = "ControlWorker",
        parent=None,
    ):
        super().__init__(parent)
        self._tick = tick
        self._period = 1.0 / max(1e-6, float(rate_hz))
        self._running = False
        self._paused = False
        self._lock = threading.Lock()
        self.setObjectName(name)

    # --- API pública (thread-safe) ---------------------------------------
    def set_rate(self, rate_hz: float) -> None:
        """Cambia la tasa objetivo del lazo en caliente."""
        with self._lock:
            self._period = 1.0 / max(1e-6, float(rate_hz))

    @property
    def rate_hz(self) -> float:
        with self._lock:
            return 1.0 / self._period

    def pause(self, paused: bool = True) -> None:
        """Suspende/reanuda las llamadas al tick sin matar el hilo."""
        with self._lock:
            self._paused = paused

    def is_paused(self) -> bool:
        with self._lock:
            return self._paused

    def stop(self, wait_ms: int = 2000) -> None:
        """Detiene el lazo y espera a que el hilo termine."""
        with self._lock:
            self._running = False
        if self.isRunning():
            self.wait(wait_ms)

    # --- Bucle del hilo ---------------------------------------------------
    def run(self) -> None:  # noqa: D401 - QThread entrypoint
        with self._lock:
            self._running = True
        logger.info(f"[{self.objectName()}] iniciado @ {self.rate_hz:.0f} Hz")
        next_t = time.perf_counter()
        while True:
            with self._lock:
                if not self._running:
                    break
                period = self._period
                paused = self._paused

            if not paused:
                try:
                    self._tick()
                except Exception:  # el lazo nunca debe morir por una excepción
                    logger.exception(f"[{self.objectName()}] error en tick")

            # Programación de próximo disparo con corrección de deriva.
            next_t += period
            now = time.perf_counter()
            sleep_s = next_t - now
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                # Atrasados respecto al plan: reancla la base para no encadenar
                # una ráfaga de ticks de recuperación.
                next_t = now
        logger.info(f"[{self.objectName()}] detenido")

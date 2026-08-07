"""Cola TX serial: prioridad de control + coalescing de A,*.

El approach manda A,* @ ~400 Hz. Sin gestión, satura el USART del MCU y
se pierden I/F/N/B del handoff FOV (síntoma: mcu=AUTO fijo).

Política:
  - A,* / a,*: solo se conserva el ÚLTIMO (coalesce).
  - Control (F,I,N,B,M,P,…): cola FIFO, nunca se coalescen entre sí.
  - Al drenar: primero TODO el control, luego el A,* pendiente.
"""

from __future__ import annotations

import threading
from collections import deque
from typing import Deque, List, Optional


def _is_auto_pwm(cmd: str) -> bool:
    c = cmd.lstrip()
    return len(c) >= 2 and (c[0] in "Aa") and c[1] == ","


class SerialTxQueue:
    """Cola thread-safe de comandos hacia el MCU."""

    def __init__(self, max_control: int = 64) -> None:
        self._lock = threading.Lock()
        self._control: Deque[str] = deque(maxlen=max_control)
        self._pending_a: Optional[str] = None
        self._dropped_a = 0
        self._dropped_ctrl = 0
        self._enqueued = 0
        self._drained = 0

    def enqueue(self, command: str) -> None:
        cmd = (command or "").strip()
        if not cmd:
            return
        with self._lock:
            self._enqueued += 1
            if _is_auto_pwm(cmd):
                if self._pending_a is not None:
                    self._dropped_a += 1
                self._pending_a = cmd
                return
            # Control: si la deque está a tope, maxlen descarta el más viejo.
            if (
                self._control.maxlen is not None
                and len(self._control) >= self._control.maxlen
            ):
                self._dropped_ctrl += 1
            self._control.append(cmd)

    def pop_batch(self, max_n: int = 16) -> List[str]:
        """Saca hasta max_n comandos: control primero, luego un A,*."""
        out: List[str] = []
        with self._lock:
            while self._control and len(out) < max_n:
                out.append(self._control.popleft())
            if self._pending_a is not None and len(out) < max_n:
                out.append(self._pending_a)
                self._pending_a = None
            self._drained += len(out)
        return out

    def pending_count(self) -> int:
        with self._lock:
            return len(self._control) + (1 if self._pending_a else 0)

    def stats(self) -> dict:
        with self._lock:
            return {
                "enqueued": self._enqueued,
                "drained": self._drained,
                "dropped_a": self._dropped_a,
                "dropped_ctrl": self._dropped_ctrl,
                "pending_ctrl": len(self._control),
                "pending_a": self._pending_a is not None,
            }

    def clear(self) -> None:
        with self._lock:
            self._control.clear()
            self._pending_a = None

"""Parada de movimiento XY/MCU — único módulo productor de la secuencia dura.

Evita que MicroscopyService / TestService / StepController emitan N/B/A,0,0/M
en paralelo (carrera en cola TX / RX MCU).
"""

from __future__ import annotations

import logging
from typing import Callable, Optional

from core.communication.protocol import MotorProtocol

logger = logging.getLogger("MotorControl_L206")


def send_full_halt(
    send_command: Optional[Callable[[str], None]],
    *,
    reason: str = "",
) -> bool:
    """Emite N → B → A,0,0 → M. True si se envió."""
    if send_command is None:
        logger.error("[MotionHalt] send_command no disponible (%s)", reason or "?")
        return False
    for cmd in MotorProtocol.full_halt_commands():
        send_command(cmd)
    logger.info("[MotionHalt] full halt (%s): %s", reason or "?", "/".join(MotorProtocol.full_halt_commands()))
    return True

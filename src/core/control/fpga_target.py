"""AUTO de la FPGA v3.0: el PC pide la posición en cuentas y la FPGA cierra el lazo.

Una orden por movimiento, ``T,cx,cy,px,py``; después solo se mira la trama.
Llegada: en la misma trama y sostenidas ``settle_s``:
  A1 Estado AUTO · A2 PotA/PotB = pedido · A3 Settled 1 · A4 cuentas sin cambio.
Sin avance (Settled 0 y cuentas quietas ``settle_s``) o vencimiento
(``timeout_s``): manda B. Nunca se da por llegado sin A1..A4.
Si la trama deja AUTO o muestra otro pedido, alguien más tomó el mando:
se detiene sin mandar nada.
Sin Qt: la pestaña Control le entrega cada trama FPGA ya parseada.
"""
from __future__ import annotations

import enum
import logging
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

from core.communication.protocol import MotorProtocol

logger = logging.getLogger("MotorControl_L206")

Pair = Tuple[int, int]


class Outcome(str, enum.Enum):
    ARRIVED = "llegó"
    BLOCKED = "sin avance"
    TIMEOUT = "vencido"
    TAKEN_OVER = "otro tomó el mando"
    NOT_TAKEN = "la FPGA no tomó el pedido"
    DISCONNECTED = "sin conexión"


# Salidas que mandan B: la FPGA sigue empujando y el PC la detiene.
BRAKE_OUTCOMES = (Outcome.BLOCKED, Outcome.TIMEOUT)


@dataclass(frozen=True)
class TargetSettings:
    """Valores del perfil activo, leídos al empezar cada movimiento."""
    settle_s: float        # fov_settle_ms / 1000
    timeout_s: float       # techo por movimiento (point_timeout_s)
    power_min: int         # stiction_pwm_min: bajo esto el motor no vence el roce
    power_max: int         # power_max del perfil (CFG_POWER)
    margin: int            # travel_margin_counts: distancia al cero y al máximo
    counts_per_edge: int   # 1 flanco del Hall


@dataclass(frozen=True)
class AxisRange:
    lo: int
    hi: int

    def contains(self, count: int) -> bool:
        return self.lo <= count <= self.hi


@dataclass(frozen=True)
class TargetResult:
    outcome: Outcome
    target: Pair
    counts: Pair
    elapsed_s: float
    take_frames: Optional[int]   # tramas hasta A1 + A2; None si no lo tomó
    state: str                   # Estado de la última trama
    commands: int                # órdenes mandadas en este movimiento (T y B)

    @property
    def residual(self) -> Pair:
        return (self.target[0] - self.counts[0], self.target[1] - self.counts[1])


class FpgaTarget:
    """Un movimiento AUTO a la vez; uno nuevo reemplaza al anterior."""

    def __init__(self, send: Callable[[str], None], settings: TargetSettings,
                 clock: Callable[[], float] = time.monotonic):
        self._send = send
        self.settings = settings
        self.clock = clock
        self._max: Optional[Pair] = None
        self._active = False
        self._target: Pair = (0, 0)
        self._power: Pair = (0, 0)
        self._t0 = 0.0
        self._frames = 0
        self._take_frames: Optional[int] = None
        self._commands = 0
        self._still_counts: Optional[Pair] = None
        self._still_since = 0.0
        self._ok_since: Optional[float] = None
        self._last_counts: Pair = (0, 0)
        self._last_state = ""

    # ---- rango de la sesión ----
    def set_max(self, x_max: int, y_max: int) -> None:
        """Cuenta de cada eje al marcar 2 · Máximo; el cero es la cuenta 0."""
        self._max = (int(x_max), int(y_max))

    def clear_max(self) -> None:
        self._max = None

    @property
    def has_max(self) -> bool:
        return self._max is not None

    def ranges(self) -> Optional[Tuple[AxisRange, AxisRange]]:
        if self._max is None:
            return None
        m = self.settings.margin
        return (AxisRange(m, self._max[0] - m), AxisRange(m, self._max[1] - m))

    @property
    def active(self) -> bool:
        return self._active

    @property
    def target(self) -> Pair:
        """Último pedido mandado (cuentas)."""
        return self._target

    @property
    def power(self) -> Pair:
        """Potencia del último pedido (%)."""
        return self._power

    # ---- validación ----
    def _power_error(self, name: str, power: int) -> Optional[str]:
        s = self.settings
        if power == 0 or s.power_min <= power <= s.power_max:
            return None
        return f"Potencia {name} {power} %: debe ser 0 o de {s.power_min} a {s.power_max} %"

    def _validate(self, target: Pair, power: Pair, moving: Tuple[bool, bool]) -> Optional[str]:
        """``moving[i]``: el eje i va a un pedido nuevo y debe caer en su rango."""
        if self._max is None:
            return "Falta 2 · Máximo en esta sesión"
        for name, p in zip("XY", power):
            err = self._power_error(name, p)
            if err:
                return err
        for i, rng in enumerate(self.ranges()):
            if not moving[i]:
                continue
            name = "XY"[i]
            if rng.hi < rng.lo:
                return (f"Recorrido de {name} ({self._max[i]} cuentas) más corto "
                        f"que dos márgenes de {self.settings.margin}")
            if not rng.contains(target[i]):
                return f"{name} fuera de rango: {target[i]} (va de {rng.lo} a {rng.hi})"
        return None

    def _here(self, counts: Pair) -> Pair:
        return (max(0, min(self._max[0], int(counts[0]))),
                max(0, min(self._max[1], int(counts[1]))))

    # ---- órdenes ----
    def go(self, target: Pair, power: Pair, counts: Pair) -> Optional[str]:
        """Pide ``target`` (cuentas) con ``power`` (% por eje).

        Un eje con potencia 0 pide su cuenta actual: queda frenado donde está.
        Devuelve el motivo si no se manda, o None.
        """
        power = (abs(int(power[0])), abs(int(power[1])))
        if power == (0, 0):
            return "Potencia 0 en los dos ejes: ninguno se movería"
        moving = (power[0] != 0, power[1] != 0)
        err = self._validate(target, power, moving)
        if err:
            return err
        here = self._here(counts)
        target = tuple(int(target[i]) if moving[i] else here[i] for i in range(2))
        self._start(target, power, counts)
        return None

    def hold(self, power: Pair, counts: Pair) -> Optional[str]:
        """Mantener aquí: pide la cuenta actual de los dos ejes."""
        power = (abs(int(power[0])), abs(int(power[1])))
        if power == (0, 0):
            return "Potencia 0 en los dos ejes: no sostendría la posición"
        err = self._validate(counts, power, (False, False))
        if err:
            return err
        self._start(self._here(counts), power, counts)
        return None

    def _start(self, target: Pair, power: Pair, counts: Pair) -> None:
        self._target = (int(target[0]), int(target[1]))
        self._power = power
        self._active = True
        self._t0 = self.clock()
        self._frames = 0
        self._take_frames = None
        self._commands = 0
        self._still_counts = None
        self._ok_since = None
        self._last_counts = (int(counts[0]), int(counts[1]))
        self._last_state = ""
        logger.info("AUTO T: pedido X=%d Y=%d a %d/%d %% desde X=%d Y=%d",
                    self._target[0], self._target[1], power[0], power[1],
                    counts[0], counts[1])
        self._command(MotorProtocol.format_target_command(
            self._target[0], self._target[1], power[0], power[1]))

    def _command(self, command: str) -> None:
        self._commands += 1
        self._send(command)

    def cancel(self, outcome: Outcome = Outcome.DISCONNECTED) -> Optional[TargetResult]:
        """Termina sin mandar nada (conexión caída, cambio de perfil)."""
        if not self._active:
            return None
        return self._finish(outcome)

    # ---- trama ----
    def on_frame(self, frame: dict) -> Optional[TargetResult]:
        """Cada trama FPGA; devuelve el resultado cuando el movimiento termina."""
        if not self._active:
            return None
        now = self.clock()
        self._frames += 1
        counts = (int(frame["sens_2"]), int(frame["sens_1"]))    # X = Sensor1 = sens_2
        state = str(frame["state"])
        settled = bool(frame["settled"])
        asked = (int(frame["target_x"]), int(frame["target_y"]))
        self._last_counts = counts
        self._last_state = state
        taken = state == "AUTO" and asked == self._target

        if self._take_frames is None:
            if not taken:
                if now - self._t0 >= self.settings.settle_s:
                    return self._finish(Outcome.NOT_TAKEN)
                return None
            self._take_frames = self._frames
        elif not taken:
            return self._finish(Outcome.TAKEN_OVER)

        if counts != self._still_counts:
            self._still_counts = counts
            self._still_since = now
            self._ok_since = now if settled else None
        elif not settled:
            self._ok_since = None
        elif self._ok_since is None:
            self._ok_since = now

        settle = self.settings.settle_s
        if self._ok_since is not None and now - self._ok_since >= settle:
            return self._finish(Outcome.ARRIVED)
        if not settled and now - self._still_since >= settle:
            self._command(MotorProtocol.format_brake_command())
            return self._finish(Outcome.BLOCKED)
        if now - self._t0 >= self.settings.timeout_s:
            self._command(MotorProtocol.format_brake_command())
            return self._finish(Outcome.TIMEOUT)
        return None

    def _finish(self, outcome: Outcome) -> TargetResult:
        self._active = False
        result = TargetResult(
            outcome=outcome, target=self._target, counts=self._last_counts,
            elapsed_s=self.clock() - self._t0, take_frames=self._take_frames,
            state=self._last_state, commands=self._commands,
        )
        cpe = float(self.settings.counts_per_edge)
        rx, ry = result.residual
        logger.info(
            "AUTO T: %s en %.2f s; tomado en %s tramas; residual X %+d (%+.1f fl) "
            "Y %+d (%+.1f fl); estado %s; órdenes %d",
            outcome.value, result.elapsed_s,
            "—" if result.take_frames is None else result.take_frames,
            rx, rx / cpe, ry, ry / cpe, result.state or "—", result.commands,
        )
        return result

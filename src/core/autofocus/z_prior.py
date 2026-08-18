"""Prior de posición del BPoF para acotar el barrido COARSE.

En una muestra plana los granos comparten plano focal: los BPoF de la misma
sesión se agrupan en pocos µm. Aun así el COARSE arrancaba en el extremo del
recorrido calibrado (Z≈0 µm) aunque el origen estuviera en 43 µm y el pico
apareciera siempre pasado el origen, así que cada semilla pagaba una decena de
planos de meseta antes de encontrar señal.

El prior resuelve eso sin apostar: acumula los BPoF ya confirmados y sólo
acota la ventana cuando hay suficientes muestras coherentes. La dispersión se
mide con MAD (desviación absoluta mediana) y no con la desviación típica,
porque un único BPoF equivocado —justo lo que este sistema produce cuando S se
descontrola— no debe ensanchar la ventana de todos los puntos siguientes.
"""

from __future__ import annotations

from statistics import median
from typing import List, Optional, Tuple


def clamp_window(
    z_lo: float,
    z_hi: float,
    z_min_hw: float,
    z_max_hw: float,
) -> Tuple[float, float]:
    """Recorta una ventana al recorrido calibrado conservando su ancho.

    Si la ventana se sale por un extremo se desplaza hacia dentro en vez de
    encogerse: un pico cercano al límite necesita el mismo número de planos
    que uno centrado.
    """
    lo_hw = float(min(z_min_hw, z_max_hw))
    hi_hw = float(max(z_min_hw, z_max_hw))
    lo = float(min(z_lo, z_hi))
    hi = float(max(z_lo, z_hi))

    span = min(hi - lo, hi_hw - lo_hw)
    if span <= 0.0:
        return lo_hw, hi_hw

    if lo < lo_hw:
        lo = lo_hw
        hi = lo + span
    if hi > hi_hw:
        hi = hi_hw
        lo = hi - span
    return max(lo_hw, lo), min(hi_hw, hi)


def bootstrap_window(
    z_origin: float,
    *,
    below_um: float,
    above_um: float,
    z_min_hw: float,
    z_max_hw: float,
) -> Tuple[float, float]:
    """Ventana COARSE alrededor del origen calibrado, sin historial todavía."""
    return clamp_window(
        float(z_origin) - abs(float(below_um)),
        float(z_origin) + abs(float(above_um)),
        z_min_hw,
        z_max_hw,
    )


class BpofPrior:
    """Historial de BPoF de la sesión y ventana COARSE derivada.

    Parameters
    ----------
    min_samples : int
        BPoF necesarios antes de acotar. Los primeros puntos de la muestra
        siguen barriendo el rango pedido por la interfaz: pagar unos pocos
        ciclos completos es más barato que fijar la ventana sobre un plano
        focal que todavía no se conoce.
    min_half_span_um : float
        Semiancho mínimo de la ventana, aunque todos los BPoF coincidan.
        Absorbe la inclinación de la muestra entre puntos vecinos.
    mad_k : float
        Multiplicador de la MAD para el semiancho.
    """

    def __init__(
        self,
        *,
        min_samples: int = 5,
        min_half_span_um: float = 4.0,
        mad_k: float = 2.0,
        max_history: int = 64,
    ):
        self.min_samples = max(1, int(min_samples))
        self.min_half_span_um = max(0.0, float(min_half_span_um))
        self.mad_k = max(0.0, float(mad_k))
        self._max_history = max(self.min_samples, int(max_history))
        self._history: List[float] = []

    def add(self, z_um: float) -> None:
        """Registra un BPoF confirmado."""
        value = float(z_um)
        if value != value:  # NaN
            return
        self._history.append(value)
        if len(self._history) > self._max_history:
            del self._history[: len(self._history) - self._max_history]

    def clear(self) -> None:
        self._history.clear()

    def __len__(self) -> int:
        return len(self._history)

    @property
    def history(self) -> List[float]:
        return list(self._history)

    @property
    def ready(self) -> bool:
        return len(self._history) >= self.min_samples

    @property
    def center(self) -> Optional[float]:
        """Mediana de los BPoF acumulados."""
        if not self._history:
            return None
        return float(median(self._history))

    @property
    def mad_um(self) -> Optional[float]:
        """Desviación absoluta mediana del historial."""
        center = self.center
        if center is None:
            return None
        return float(median([abs(z - center) for z in self._history]))

    @property
    def half_span_um(self) -> Optional[float]:
        if not self.ready:
            return None
        mad = self.mad_um or 0.0
        return max(self.min_half_span_um, self.mad_k * mad)

    def window(
        self,
        z_min_hw: float,
        z_max_hw: float,
    ) -> Optional[Tuple[float, float]]:
        """Ventana COARSE acotada, o None si el historial aún no alcanza."""
        center = self.center
        half = self.half_span_um
        if center is None or half is None or half <= 0.0:
            return None
        return clamp_window(center - half, center + half, z_min_hw, z_max_hw)

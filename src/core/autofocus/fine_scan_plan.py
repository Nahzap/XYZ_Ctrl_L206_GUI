"""Orden de visita y corte temprano de la fase FINE.

Dos decisiones separan un refinamiento de un segundo barrido completo:

1. **Recorrer desde el centro hacia afuera.** Visitar la ventana en orden
   creciente de Z obliga a saltar del plano COARSE ganador al extremo inferior
   (en el log de referencia, 63 → 51 → 32 µm) y a medir 38 µm de fondo antes de
   llegar al pico. El primer plano medido queda además justo después del salto
   más largo del ciclo, que es donde el frame tiene más probabilidad de venir
   contaminado por el movimiento.
2. **Cortar cuando el pico ya quedó atrás.** El COARSE tiene corte temprano; el
   FINE recorría siempre las N capas. Al avanzar por anillos simétricos, un
   descenso sostenido en ambos lados es evidencia suficiente de que el máximo
   está dentro de lo ya medido.

Un anillo es el par de planos a la misma distancia del centro. Sólo se decide
al cerrar el anillo (ambos lados medidos) para que el corte no dependa de qué
lado se visitó primero.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple


def center_out_sequence(
    planes: Sequence[float],
    z_center: float,
) -> List[Tuple[float, int]]:
    """Planos FINE en orden centro→afuera.

    Returns
    -------
    list[(z_um, anillo)]
        El anillo 0 es el plano más cercano a ``z_center`` (la salida del
        COARSE). Cada anillo k contiene el plano a +k pasos y el de −k pasos,
        en ese orden.
    """
    zs = [float(z) for z in planes]
    if not zs:
        return []

    center_i = min(
        range(len(zs)), key=lambda i: abs(zs[i] - float(z_center))
    )
    order: List[Tuple[float, int]] = [(zs[center_i], 0)]
    max_ring = max(center_i, len(zs) - 1 - center_i)
    for ring in range(1, max_ring + 1):
        above = center_i + ring
        below = center_i - ring
        if above < len(zs):
            order.append((zs[above], ring))
        if below >= 0:
            order.append((zs[below], ring))
    return order


def ring_counts(order: Sequence[Tuple[float, int]]) -> Dict[int, int]:
    """Cuántos planos tiene cada anillo (1 en los bordes, 2 en el interior)."""
    counts: Dict[int, int] = {}
    for _z, ring in order:
        counts[int(ring)] = counts.get(int(ring), 0) + 1
    return counts


class RingDeclineStop:
    """Corte temprano del FINE por descenso sostenido de S en ambos lados.

    Parameters
    ----------
    expected_per_ring : dict[int, int]
        Planos planificados por anillo (``ring_counts``). Un anillo sólo se
        evalúa cuando está completo.
    patience_rings : int
        Anillos consecutivos que deben quedar por detrás del máximo antes de
        cortar. Con 3 y paso 0.5 µm se exigen 1.5 µm de descenso por lado.
    drop_rel : float
        Caída relativa mínima respecto al máximo en esos anillos. Evita cortar
        sobre una meseta donde el pico todavía puede estar más afuera.
    """

    def __init__(
        self,
        expected_per_ring: Dict[int, int],
        *,
        patience_rings: int = 3,
        drop_rel: float = 0.05,
    ):
        self._expected = {int(k): int(v) for k, v in dict(expected_per_ring).items()}
        self.patience_rings = max(1, int(patience_rings))
        self.drop_rel = max(0.0, float(drop_rel))

        self._ring_max: Dict[int, float] = {}
        self._ring_seen: Dict[int, int] = {}
        self._best_s = 0.0
        self._best_ring = 0
        self._max_ring_seen = -1
        self._reason = ""

    def observe(self, ring: int, s: float) -> None:
        """Registra una medición S del anillo indicado."""
        ring = int(ring)
        value = float(s)
        self._ring_seen[ring] = self._ring_seen.get(ring, 0) + 1
        self._max_ring_seen = max(self._max_ring_seen, ring)
        if value <= 0.0:
            # Un plano inválido no puede sostener ni desmentir el descenso.
            return
        if ring not in self._ring_max or value > self._ring_max[ring]:
            self._ring_max[ring] = value
        if value > self._best_s:
            self._best_s = value
            self._best_ring = ring

    def _ring_closed(self, ring: int) -> bool:
        expected = self._expected.get(int(ring))
        if not expected:
            return False
        return self._ring_seen.get(int(ring), 0) >= expected

    def should_stop(self) -> bool:
        """True si el máximo ya quedó atrás y los últimos anillos descienden."""
        current = self._max_ring_seen
        if current < 0 or self._best_s <= 0.0:
            return False
        if not self._ring_closed(current):
            return False
        if current - self._best_ring < self.patience_rings:
            return False

        window = range(current - self.patience_rings + 1, current + 1)
        limit = self._best_s * (1.0 - self.drop_rel)
        worst_ring = current
        worst_value = 0.0
        for ring in window:
            if ring <= self._best_ring:
                return False
            if not self._ring_closed(ring):
                return False
            value = self._ring_max.get(ring)
            if value is None or value > limit:
                return False
            if value > worst_value:
                worst_value, worst_ring = value, ring

        drop_pct = 100.0 * (self._best_s - worst_value) / self._best_s
        self._reason = (
            f"pico en anillo {self._best_ring} superado por "
            f"{current - self._best_ring} anillos y S cayó ≥{drop_pct:.1f}% "
            f"(anillo {worst_ring})"
        )
        return True

    @property
    def reason(self) -> str:
        return self._reason

    @property
    def best_ring(self) -> int:
        return self._best_ring

    @property
    def best_score(self) -> float:
        return self._best_s


def fine_span_um(planes: Sequence[float]) -> Optional[float]:
    """Recorrido Z total de la ventana FINE (para el KPI spanF)."""
    zs = [float(z) for z in planes]
    if len(zs) < 2:
        return 0.0 if zs else None
    return max(zs) - min(zs)

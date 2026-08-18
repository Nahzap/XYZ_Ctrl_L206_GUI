"""Indicadores del ciclo de autofoco (una línea AF_KPI por semilla).

Sin medición no hay forma de saber si un cambio de algoritmo redujo el tiempo
o sólo lo movió de fase: el barrido COARSE puede acortarse mientras el FINE
crece, y el total queda igual. Por eso cada ciclo publica los mismos campos en
una línea parseable, y la sesión acumula medianas para proyectar el ETA.

Los tres bloques responden a preguntas distintas:

- tiempos y trabajo (T_*, N_*): ¿cuánto cuesta decidir un Z?
- calidad de Z (dZ, borde, agujeros): ¿el BPoF es creíble?
- repetibilidad de S (e_*): ¿la escala S sirve para planificar el stack?

El tercer bloque es el que delata que un ciclo rápido no es necesariamente
fiable: si S no se repite en la misma Z, el stack fotográfico se construye
sobre una escala que no existe.
"""

from __future__ import annotations

from dataclasses import dataclass
from statistics import median
from typing import List, Optional

# Coste por plano observado en el log de referencia (2026-08-13): 62.43 s para
# 68 mediciones. Sirve como semilla del ETA hasta que la sesión acumule datos.
DEFAULT_S_PER_PLANE_S = 0.9


def _fmt(value: Optional[float], digits: int = 3) -> str:
    """Número con ancho fijo, o ``na`` si la fase no se ejecutó."""
    if value is None:
        return "na"
    return f"{float(value):.{digits}f}"


def _fmt_flag(value: Optional[bool]) -> str:
    if value is None:
        return "na"
    return "1" if value else "0"


@dataclass
class AfCycleKpi:
    """Métricas de un ciclo completo de autofoco (una semilla)."""

    # --- Tiempos (s) ---
    t_total: Optional[float] = None
    t_coarse: Optional[float] = None
    t_fine: Optional[float] = None
    t_confirm: Optional[float] = None
    t_photos: Optional[float] = None
    t_save: Optional[float] = None

    # --- Trabajo de búsqueda ---
    n_coarse_measured: int = 0
    n_coarse_planned: int = 0
    n_fine_measured: int = 0
    n_fine_planned: int = 0
    # Mediciones S totales del ciclo, contadas en el único embudo de medida.
    # Es el número que manda: las tablas guardan un valor por plano, pero el
    # tiempo lo paga cada toma (mediana del ancla, re-medida de agujeros,
    # confirmación, seek óptico y fotografías incluidas).
    n_s_total: int = 0
    # Contador manual de respaldo cuando el embudo está intervenido (tests).
    n_extra_measurements: int = 0
    n_detections: Optional[int] = None

    # --- Calidad de Z ---
    z_coarse_star: Optional[float] = None
    z_bpof: Optional[float] = None
    fine_span_um: Optional[float] = None
    coarse_span_um: Optional[float] = None
    coarse_window_source: str = "full"
    peak_at_edge: Optional[bool] = None
    span_rel: Optional[float] = None
    prominence_rel: Optional[float] = None
    n_holes: int = 0
    n_holes_invalidated: int = 0
    fine_early_stop: bool = False
    coarse_early_stop: bool = False

    # --- Repetibilidad de S ---
    eps_anchor: Optional[float] = None
    eps_confirm: Optional[float] = None
    eps_photo: Optional[float] = None
    delta_s_stack: Optional[float] = None
    stack_asymmetry: Optional[float] = None

    @property
    def n_s_measurements(self) -> int:
        """Total de mediciones S del ciclo (lo que realmente cuesta tiempo)."""
        if self.n_s_total > 0:
            return int(self.n_s_total)
        return int(
            self.n_coarse_measured
            + self.n_fine_measured
            + self.n_extra_measurements
        )

    @property
    def n_extra_effective(self) -> int:
        """Mediciones que no aparecen como plano nuevo en las tablas."""
        if self.n_s_total > 0:
            return max(
                0,
                int(self.n_s_total)
                - int(self.n_coarse_measured)
                - int(self.n_fine_measured),
            )
        return int(self.n_extra_measurements)

    @property
    def dz_coarse_fine(self) -> Optional[float]:
        """|BPoF − Z_c*|: cuánto corrigió FINE al COARSE."""
        if self.z_bpof is None or self.z_coarse_star is None:
            return None
        return abs(float(self.z_bpof) - float(self.z_coarse_star))

    @property
    def s_per_plane(self) -> Optional[float]:
        """Coste medio por medición S; separa 'menos planos' de 'planos más rápidos'."""
        n = self.n_s_measurements
        if self.t_total is None or n <= 0:
            return None
        return float(self.t_total) / float(n)

    def format_line(self) -> str:
        """Línea única parseable, comparable entre commits."""
        return (
            "AF_KPI "
            f"T_AF={_fmt(self.t_total, 2)} "
            f"T_c={_fmt(self.t_coarse, 2)} "
            f"T_f={_fmt(self.t_fine, 2)} "
            f"T_k={_fmt(self.t_confirm, 2)} "
            f"T_ph={_fmt(self.t_photos, 2)} "
            f"N_c={self.n_coarse_measured}/{self.n_coarse_planned} "
            f"N_f={self.n_fine_measured}/{self.n_fine_planned} "
            f"N_S={self.n_s_measurements} "
            f"N_x={self.n_extra_effective} "
            f"T_plano={_fmt(self.s_per_plane, 3)} "
            f"u2net={self.n_detections if self.n_detections is not None else 'na'} "
            f"ventana_c={self.coarse_window_source}:{_fmt(self.coarse_span_um, 2)} "
            f"Zc={_fmt(self.z_coarse_star, 2)} "
            f"Z*={_fmt(self.z_bpof, 2)} "
            f"dZ={_fmt(self.dz_coarse_fine, 2)} "
            f"spanF={_fmt(self.fine_span_um, 2)} "
            f"borde={_fmt_flag(self.peak_at_edge)} "
            f"span_rel={_fmt(self.span_rel, 4)} "
            f"prom_rel={_fmt(self.prominence_rel, 4)} "
            f"agujero={self.n_holes}/{self.n_holes_invalidated} "
            f"early_c={_fmt_flag(self.coarse_early_stop)} "
            f"early_f={_fmt_flag(self.fine_early_stop)} "
            f"e_ancla={_fmt(self.eps_anchor, 3)} "
            f"e_conf={_fmt(self.eps_confirm, 3)} "
            f"e_foto={_fmt(self.eps_photo, 3)} "
            f"dS_stack={_fmt(self.delta_s_stack, 3)} "
            f"asim={_fmt(self.stack_asymmetry, 2)}"
        )


class AfSessionKpi:
    """Acumulador de ciclos: medianas, hit-rate y ETA de la sesión.

    El ETA no se puede sacar del número de puntos: sólo los puntos con objeto
    pagan autofoco. Por eso se cuentan por separado los puntos visitados y los
    ciclos ejecutados, y el hit-rate resultante pondera el coste.
    """

    def __init__(self, max_history: int = 500):
        self._max_history = max(1, int(max_history))
        self._cycles: List[AfCycleKpi] = []
        self._n_points = 0
        self._n_cycles_total = 0

    def note_point_visited(self, n: int = 1) -> None:
        self._n_points += max(0, int(n))

    def add_cycle(self, kpi: AfCycleKpi) -> None:
        self._n_cycles_total += 1
        self._cycles.append(kpi)
        if len(self._cycles) > self._max_history:
            del self._cycles[: len(self._cycles) - self._max_history]

    def clear(self) -> None:
        self._cycles.clear()
        self._n_points = 0
        self._n_cycles_total = 0

    @property
    def n_points(self) -> int:
        return self._n_points

    @property
    def n_cycles(self) -> int:
        return self._n_cycles_total

    @property
    def hit_rate(self) -> Optional[float]:
        """Fracción de puntos que disparan autofoco (p_hat)."""
        if self._n_points <= 0:
            return None
        return min(1.0, float(self._n_cycles_total) / float(self._n_points))

    def median_of(self, field: str) -> Optional[float]:
        """Mediana de un campo/propiedad entre los ciclos con dato válido."""
        values = []
        for cycle in self._cycles:
            value = getattr(cycle, field, None)
            if value is None or isinstance(value, str):
                continue
            values.append(float(value))
        if not values:
            return None
        return float(median(values))

    def count_where(self, field: str) -> int:
        """Ciclos con el campo booleano activo (p.ej. picos en borde)."""
        return sum(1 for cycle in self._cycles if bool(getattr(cycle, field, False)))

    def eta_s(
        self,
        points_remaining: int,
        *,
        t_point_overhead_s: float = 0.0,
        t_point_without_af_s: float = 5.0,
    ) -> Optional[float]:
        """Segundos restantes según T_AF mediano y el hit-rate observado.

        ``t_point_overhead_s`` es lo que cuesta un punto con objeto aparte del
        autofoco (viaje XY, delay, guardado): no depende del algoritmo pero sí
        del ETA, así que el llamador lo aporta desde su propia medición.
        """
        remaining = max(0, int(points_remaining))
        if remaining == 0:
            return 0.0
        p = self.hit_rate
        t_af = self.median_of("t_total")
        if p is None or t_af is None:
            return None
        t_with_af = float(t_af) + max(0.0, float(t_point_overhead_s))
        t_point = p * t_with_af + (1.0 - p) * max(0.0, float(t_point_without_af_s))
        return float(remaining) * t_point

    def summary_line(
        self,
        points_remaining: Optional[int] = None,
        *,
        t_point_overhead_s: float = 0.0,
        t_point_without_af_s: float = 5.0,
    ) -> str:
        """Línea de progreso de sesión (para UI y log)."""
        p = self.hit_rate
        eta = (
            self.eta_s(
                points_remaining,
                t_point_overhead_s=t_point_overhead_s,
                t_point_without_af_s=t_point_without_af_s,
            )
            if points_remaining is not None
            else None
        )
        return (
            "AF_SESION "
            f"n_AF={self.n_cycles} "
            f"n_puntos={self.n_points} "
            f"p_hat={_fmt(p, 3)} "
            f"T_AF_med={_fmt(self.median_of('t_total'), 2)} "
            f"N_S_med={_fmt(self.median_of('n_s_measurements'), 1)} "
            f"dZ_med={_fmt(self.median_of('dz_coarse_fine'), 2)} "
            f"e_conf_med={_fmt(self.median_of('eps_confirm'), 3)} "
            f"bordes={self.count_where('peak_at_edge')} "
            f"ETA_h={_fmt(eta / 3600.0 if eta is not None else None, 2)}"
        )

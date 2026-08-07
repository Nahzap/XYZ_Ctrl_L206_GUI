"""Tablas PC de candidatos BPoF (coarse y fine, separadas).

Algoritmo:
  1) Tabla COARSE (Z, S) → Z_c* = argmax S
  2) Zona FINE centrada en Z_c*: paso_fine × N, limitada por ±Δ
  3) Tabla FINE (Z, S) → BPoF = argmax S
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


@dataclass(frozen=True)
class FocusCandidate:
    """Medición real: plano Z e índice S."""

    z_um: float
    s: float
    phase: str = "coarse"  # coarse | fine


def min_candidates_for_planes(n_planes: int) -> int:
    n = max(0, int(n_planes))
    if n <= 0:
        return 0
    return min(n, max(5, (n + 3) // 4))


def unique_z_key(z_um: float, decimals: int = 3) -> float:
    return round(float(z_um), int(decimals))


def symmetric_fine_window(
    z_center_um: float,
    delta_um: float,
    z_min_um: float,
    z_max_um: float,
) -> Tuple[float, float, float]:
    """Intervalo fine simétrico. Returns (z_lo, z_hi, delta_eff)."""
    zc = float(z_center_um)
    z_lo_hw = float(z_min_um)
    z_hi_hw = float(z_max_um)
    if z_hi_hw < z_lo_hw:
        z_lo_hw, z_hi_hw = z_hi_hw, z_lo_hw

    zc = max(z_lo_hw, min(z_hi_hw, zc))
    delta = max(0.0, float(delta_um))
    margin_below = zc - z_lo_hw
    margin_above = z_hi_hw - zc
    delta_eff = min(delta, margin_below, margin_above)
    return zc - delta_eff, zc + delta_eff, delta_eff


def build_fine_z_planes(
    z_center_um: float,
    delta_um: float,
    n_planes: int,
    z_min_um: float,
    z_max_um: float,
    z_step_um: Optional[float] = None,
) -> Tuple[List[float], float]:
    """N planos FINE derivados de Z_coarse*, siempre distintos si hay rango.

    ``z_step_um`` (GUI Paso fino) y N determinan el span solicitado. ``delta_um``
    es el límite máximo ±Δ. Cerca de un límite hardware la ventana completa se
    desplaza hacia el interior: nunca se colapsa silenciosamente a un solo plano.
    Si el hardware no admite el span, se reduce el paso conservando N.
    """
    n = int(n_planes)
    if n < 3:
        n = 3
    if n % 2 == 0:
        n += 1

    half = n // 2
    delta_req = max(0.0, float(delta_um))
    if z_step_um is not None:
        step_req = max(0.0, float(z_step_um))
        if step_req > 0.0:
            delta_req = min(delta_req, step_req * float(half))

    z_lo_hw = float(min(z_min_um, z_max_um))
    z_hi_hw = float(max(z_min_um, z_max_um))
    hw_span = max(0.0, z_hi_hw - z_lo_hw)
    if hw_span <= 1e-12:
        return [z_lo_hw], 0.0

    span_req = 2.0 * delta_req
    span_eff = min(span_req, hw_span)
    step = span_eff / float(n - 1)
    zc = max(z_lo_hw, min(z_hi_hw, float(z_center_um)))
    lo = zc - span_eff * 0.5
    hi = lo + span_eff
    if lo < z_lo_hw:
        lo = z_lo_hw
        hi = lo + span_eff
    if hi > z_hi_hw:
        hi = z_hi_hw
        lo = hi - span_eff

    planes = [lo + i * step for i in range(n)]
    return planes, span_eff * 0.5


class BpofCandidateTable:
    """Tabla host (Z, S) de UNA fase. Coarse y fine son instancias distintas."""

    __slots__ = ("_rows", "_n_planned", "phase")

    def __init__(self, n_planned_planes: int = 0, phase: str = "coarse"):
        self._rows: List[FocusCandidate] = []
        self._n_planned = max(0, int(n_planned_planes))
        self.phase = str(phase)

    @property
    def n_planned(self) -> int:
        return self._n_planned

    @property
    def min_required(self) -> int:
        return min_candidates_for_planes(self._n_planned or len(self._rows))

    def __len__(self) -> int:
        return len(self._rows)

    def add(self, z_um: float, s: float, phase: Optional[str] = None) -> None:
        ph = self.phase if phase is None else str(phase)
        self._rows.append(
            FocusCandidate(z_um=float(z_um), s=float(s), phase=ph)
        )

    def clear(self) -> None:
        self._rows.clear()
        self._n_planned = 0

    def set_planned(self, n_planes: int) -> None:
        self._n_planned = max(0, int(n_planes))

    def sorted_by_z(self) -> List[FocusCandidate]:
        return sorted(self._rows, key=lambda r: r.z_um)

    def valid_rows(self) -> List[FocusCandidate]:
        return [r for r in self._rows if r.s > 0.0]

    def meets_minimum(self) -> bool:
        return len(self.valid_rows()) >= self.min_required

    def latest_per_z(
        self, rows: Optional[List[FocusCandidate]] = None
    ) -> List[FocusCandidate]:
        src = self.valid_rows() if rows is None else rows
        by_z: Dict[float, FocusCandidate] = {}
        for r in src:
            by_z[unique_z_key(r.z_um)] = r
        return list(by_z.values())

    def select_argmax(self) -> Tuple[float, float, Dict]:
        """Plano con MAYOR S en ESTA tabla."""
        pool = self.latest_per_z()
        info: Dict = {
            "n_total": len(self._rows),
            "n_valid": len(self.valid_rows()),
            "n_pool": len(pool),
            "n_planned": self._n_planned,
            "min_required": self.min_required,
            "meets_minimum": self.meets_minimum(),
            "method": "none",
            "phase": self.phase,
        }
        if not pool:
            info["method"] = "empty"
            return 0.0, 0.0, info
        best = max(pool, key=lambda r: (r.s, -abs(r.z_um)))
        info["method"] = f"argmax_{self.phase}"
        info["z_bpof"] = best.z_um
        info["s_bpof"] = best.s
        return best.z_um, best.s, info

    def select_coarse_max_s(self) -> Tuple[float, float, Dict]:
        return self.select_argmax()

    def select_near_max(
        self,
        reference_z: float,
        *,
        relative_tie: float = 0.005,
        absolute_tie: float = 0.0,
    ) -> Tuple[float, float, Dict]:
        """Desempata una meseta S por cercanía a la referencia calibrada."""
        pool = self.latest_per_z()
        if not pool:
            return self.select_argmax()

        raw_best = max(pool, key=lambda r: r.s)
        tie_band = max(
            float(absolute_tie),
            abs(float(raw_best.s)) * max(0.0, float(relative_tie)),
        )
        near_max = [
            row for row in pool if float(row.s) >= float(raw_best.s) - tie_band
        ]
        selected = min(
            near_max,
            key=lambda row: (
                abs(float(row.z_um) - float(reference_z)),
                -float(row.s),
            ),
        )
        info = {
            "method": "near_max_reference",
            "phase": self.phase,
            "reference_z": float(reference_z),
            "raw_argmax_z": float(raw_best.z_um),
            "raw_argmax_s": float(raw_best.s),
            "tie_band": float(tie_band),
            "n_tied": len(near_max),
            "z_bpof": float(selected.z_um),
            "s_bpof": float(selected.s),
        }
        return float(selected.z_um), float(selected.s), info

    def select_bpof(self) -> Tuple[float, float, Dict]:
        return self.select_argmax()

    def assess_peak(
        self,
        *,
        min_relative_span: float = 0.015,
        min_prominence_rel: float = 0.005,
    ) -> Dict:
        """Valida que el argmax sea un pico interior y distinguible del ruido."""
        rows = sorted(self.latest_per_z(), key=lambda r: r.z_um)
        if len(rows) < 3:
            return {
                "valid": False,
                "reason": f"solo {len(rows)} plano(s) FINE válido(s)",
                "n": len(rows),
            }

        best = max(rows, key=lambda r: (r.s, -abs(r.z_um)))
        best_i = rows.index(best)
        scores = [float(r.s) for r in rows]
        scale = max(abs(sorted(scores)[len(scores) // 2]), 1e-9)
        relative_span = (max(scores) - min(scores)) / scale
        prominence_rel = (float(best.s) - float(sorted(scores)[len(scores) // 2])) / scale
        at_edge = best_i == 0 or best_i == len(rows) - 1

        reason = "ok"
        valid = True
        if at_edge:
            valid = False
            reason = "argmax FINE en borde: el pico queda fuera de la ventana"
        elif relative_span < float(min_relative_span):
            valid = False
            reason = (
                f"curva S plana: span_rel={relative_span:.4f} "
                f"< {float(min_relative_span):.4f}"
            )
        elif prominence_rel < float(min_prominence_rel):
            valid = False
            reason = (
                f"pico sin prominencia: prom_rel={prominence_rel:.4f} "
                f"< {float(min_prominence_rel):.4f}"
            )

        return {
            "valid": valid,
            "reason": reason,
            "n": len(rows),
            "best_index": best_i,
            "best_z": float(best.z_um),
            "best_s": float(best.s),
            "relative_span": float(relative_span),
            "prominence_rel": float(prominence_rel),
            "at_edge": at_edge,
        }

    select_coarse_approach = select_coarse_max_s
    select_argmax_phase = select_argmax

    def summary_top(self, k: int = 5) -> List[FocusCandidate]:
        pool = self.latest_per_z()
        return sorted(pool, key=lambda r: r.s, reverse=True)[: max(0, k)]

    def format_dump(self, title: str = "Candidatos") -> str:
        # Una fila por Z (sin duplicados de re-medición)
        rows = sorted(self.latest_per_z(self.sorted_by_z()), key=lambda r: r.z_um)
        z_b, s_b, info = self.select_argmax()
        method = info.get("method", "none")
        tag = "CENTRO_FINE (max S)" if self.phase == "coarse" else "BPoF (max S)"
        lines = [
            f"=== {title} | fase={self.phase} | n={len(rows)} | "
            f"method={method} ===",
            f"{'idx':>4}  {'Z_um':>8}  {'S':>8}  mark",
        ]
        best_z_key = unique_z_key(z_b)
        marked = False
        for i, r in enumerate(rows):
            mark = ""
            if (
                not marked
                and unique_z_key(r.z_um) == best_z_key
                and abs(r.s - s_b) < 1e-6
                and r.s > 0.0
            ):
                mark = f" <-- {tag}"
                marked = True
            lines.append(f"{i:4d}  {r.z_um:8.2f}  {r.s:8.1f}{mark}")
        if method == "empty":
            lines.append(f"=== SIN mediciones válidas ({self.phase}) ===")
        else:
            lines.append(
                f"=== {tag}: Z={z_b:.2f}µm  S={s_b:.1f} "
                f"(tabla {self.phase}) ==="
            )
        return "\n".join(lines)

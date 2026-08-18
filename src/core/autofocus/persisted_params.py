"""Reparación del formulario de autofoco restaurado desde JSON.

``camera_tab`` guarda el último formulario y lo restaura tal cual en el arranque,
así que pisa cualquier default nuevo del builder. El ciclo auditado el
2026-08-13 venía de ahí: Δ=22µm con 39 planos de 1µm da una ventana FINE de
±19µm, es decir FINE repitiendo el barrido COARSE a mayor resolución (61
mediciones, ~60s por punto), y tol=1.0µm con paso 1.0µm permite que dos planos
FINE distintos se midan en la misma Z real.

Estas dos combinaciones no son una preferencia del usuario: hacen que la curva
S(z) mienta. Se corrigen al restaurar y se informa qué se cambió y por qué.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

MIN_FINE_PLANES = 3
MIN_ARRIVE_TOL_UM = 0.05


def _as_float(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if out > 0.0 else None


def _as_int(value: Any) -> Optional[int]:
    try:
        out = int(value)
    except (TypeError, ValueError):
        return None
    return out if out > 0 else None


def max_fine_planes(step_fine_um: float, step_coarse_um: float) -> int:
    """Planos FINE que caben en ±1 paso grueso, siempre impar.

    El ganador COARSE acota el BPoF a ±1 paso grueso: fuera de esa ventana FINE
    ya no refina, vuelve a buscar.
    """
    half = int(step_coarse_um / step_fine_um)
    n = 2 * half + 1
    return max(MIN_FINE_PLANES, n)


def max_arrive_tol_um(step_fine_um: float) -> float:
    """Tolerancia máxima para que dos planos FINE no colapsen en la misma Z."""
    return max(MIN_ARRIVE_TOL_UM, round(step_fine_um / 2.0, 3))


def sanitize_autofocus_form(
    values: Dict[str, Any],
) -> Tuple[Dict[str, Any], List[str]]:
    """Devuelve el formulario corregido y las notas de lo ajustado.

    No inventa valores ausentes ni toca los que ya son coherentes: si el JSON
    guardado es sano, sale idéntico y sin notas.
    """
    fixed = dict(values)
    notes: List[str] = []

    step_fine = _as_float(fixed.get("z_step_fine_um"))
    step_coarse = _as_float(fixed.get("z_step_coarse_um"))
    n_fine = _as_int(fixed.get("n_fine_planes"))
    tol = _as_float(fixed.get("z_arrive_tol_um"))

    if step_fine and step_coarse and n_fine:
        n_max = max_fine_planes(step_fine, step_coarse)
        if n_fine > n_max:
            fixed["n_fine_planes"] = n_max
            notes.append(
                f"N_fine {n_fine}→{n_max}: ±{(n_fine - 1) / 2 * step_fine:.1f}µm "
                f"con paso grueso {step_coarse:.1f}µm no refinaba el plano "
                f"COARSE ganador, repetía el barrido "
                f"(ventana FINE ±{(n_max - 1) / 2 * step_fine:.1f}µm)"
            )

    if step_fine and tol:
        tol_max = max_arrive_tol_um(step_fine)
        if tol > tol_max:
            fixed["z_arrive_tol_um"] = tol_max
            notes.append(
                f"tol_Z {tol:.2f}→{tol_max:.2f}µm: con tol ≥ paso fino "
                f"{step_fine:.2f}µm dos planos FINE distintos podían medirse "
                f"en la misma Z real"
            )

    return fixed, notes

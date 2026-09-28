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

    hyst_um = _as_float(fixed.get("center_hysteresis_um"))
    if hyst_um is not None:
        clamped = max(1.0, min(200.0, hyst_um))
        if clamped != hyst_um:
            fixed["center_hysteresis_um"] = clamped
            notes.append(
                f"center_hysteresis_um {hyst_um:.1f}→{clamped:.1f}µm"
            )

    hyst_px = _as_float(fixed.get("center_hysteresis_px"))
    if hyst_px is not None:
        clamped_px = max(1.0, min(2000.0, hyst_px))
        if clamped_px != hyst_px:
            fixed["center_hysteresis_px"] = clamped_px
            notes.append(
                f"center_hysteresis_px {hyst_px:.1f}→{clamped_px:.1f}px"
            )

    retries = fixed.get("center_max_retries")
    if retries is not None:
        try:
            n_ret = int(retries)
        except (TypeError, ValueError):
            n_ret = 8
        if n_ret < 6:
            notes.append(
                f"center_max_retries {n_ret}→8 (pasos del lazo XY, no reintentos de signo)"
            )
            n_ret = 8
        n_ret = max(6, min(8, n_ret))
        if n_ret != retries:
            fixed["center_max_retries"] = n_ret

    max_d = _as_float(fixed.get("center_max_delta_um"))
    if max_d is not None:
        # 0 = sin cap (interpolación completa al centro). >0 = techo explícito.
        if max_d <= 0.0:
            clamped_d = 0.0
        else:
            clamped_d = max(5.0, min(2000.0, max_d))
        if clamped_d != max_d:
            fixed["center_max_delta_um"] = clamped_d

    for sign_key in ("center_sign_x", "center_sign_y"):
        raw_sign = fixed.get(sign_key)
        if raw_sign is None:
            continue
        try:
            sign = 1 if int(raw_sign) >= 0 else -1
        except (TypeError, ValueError):
            sign = 1
        if sign != raw_sign:
            fixed[sign_key] = sign

    # Δpx = centro_imagen − centroide. JSON viejo usaba e=cx−W/2 con sign=−1;
    # invertir signos una vez para no flip-flop el comando de stage.
    if not fixed.get("center_delta_toward_image"):
        flipped = []
        for sign_key in ("center_sign_x", "center_sign_y"):
            if sign_key not in fixed:
                continue
            try:
                old = 1 if int(fixed[sign_key]) >= 0 else -1
            except (TypeError, ValueError):
                old = -1
            fixed[sign_key] = -old
            flipped.append(f"{sign_key} {old:+d}→{-old:+d}")
        hyst = _as_float(fixed.get("center_hysteresis_um"))
        if hyst is not None and hyst >= 40.0:
            fixed["center_hysteresis_um"] = 12.0
            notes.append(
                f"center_hysteresis_um {hyst:.1f}→12.0µm (default; 50µm saltaba el goto)"
            )
        fixed["center_delta_toward_image"] = True
        if flipped:
            notes.append(
                "Δpx hacia centro de imagen: " + ", ".join(flipped)
            )

    return fixed, notes

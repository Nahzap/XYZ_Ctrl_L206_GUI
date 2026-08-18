"""Equilibrado del stack fotográfico alrededor del BPoF.

Los planos del stack se eligen por caída relativa de S, no por µm arbitrarios.
Cuando la curva medida es limpia eso produce un bracket casi simétrico. Cuando
no lo es, produce stacks como el del log de referencia: BPoF en 52.04 µm con
planos en −5.00 y +1.00 µm. Las tres fotografías salieron con S = 282 / 298 /
299, es decir indistinguibles, y una de ellas 5 µm fuera de foco por un lado de
la curva que estaba inflado.

La causa no se arregla aquí (es la repetibilidad de S), pero el síntoma sí se
puede acotar: si un lado exige cinco veces más recorrido que el otro para la
misma caída de S, la curva no es de fiar como escala y el bracket geométrico es
la opción conservadora. La distancia que se conserva es la **menor** de las
dos, la del lado que sí responde ópticamente: mantiene las tres tomas cerca del
foco en vez de alejar una de ellas.

Sólo se reflejan planos que **ya fueron medidos** en el barrido. Así el stack
sigue costando cero sondeos Z adicionales y nunca se fotografía un plano del
que no se sabe nada.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple


def stack_asymmetry_ratio(offsets: Sequence[float]) -> Optional[float]:
    """max|offset| / min|offset| entre planos laterales; 1.0 = bracket simétrico.

    Devuelve None si no hay al menos dos planos laterales que comparar.
    """
    distances = [abs(float(offset)) for offset in offsets if abs(float(offset)) > 1e-9]
    if len(distances) < 2:
        return None
    smallest = min(distances)
    if smallest <= 1e-9:
        return None
    return max(distances) / smallest


def _measured_at(
    pool: Sequence[Dict],
    z_target: float,
    tol_um: float,
) -> Optional[Dict]:
    """Plano ya medido más cercano a ``z_target`` dentro de la tolerancia."""
    best: Optional[Dict] = None
    best_error = None
    for row in pool:
        error = abs(float(row["z_um"]) - float(z_target))
        if error > tol_um:
            continue
        if best_error is None or error < best_error:
            best, best_error = row, error
    return best


def rebalance_symmetric(
    lower: Sequence[Dict],
    upper: Sequence[Dict],
    *,
    pool: Sequence[Dict],
    z_bpof: float,
    baseline: float,
    max_ratio: float = 3.0,
    tol_um: float = 0.05,
) -> Optional[Tuple[List[float], List[Dict]]]:
    """Refleja las distancias del stack cuando la curva sale desequilibrada.

    Parameters
    ----------
    lower, upper : list[dict]
        Planos seleccionados por caída de S a cada lado del BPoF.
    pool : list[dict]
        Todos los planos medidos disponibles (``z_um``, ``score``).
    baseline : float
        S del BPoF en la misma curva; sirve para recalcular las caídas.
    max_ratio : float
        Asimetría tolerada antes de intervenir.

    Returns
    -------
    (z_positions, items) o None
        None significa "dejar el plan como está": o ya es simétrico, o no hay
        planos medidos en el lado reflejado, o el stack es unilateral por
        límite de hardware (ahí la asimetría es física, no un artefacto).
    """
    if not lower or not upper or len(lower) != len(upper):
        return None
    if float(baseline) <= 0.0:
        return None

    z0 = float(z_bpof)
    distances_lower = sorted(abs(float(item["z_um"]) - z0) for item in lower)
    distances_upper = sorted(abs(float(item["z_um"]) - z0) for item in upper)
    offsets = [float(item["z_um"]) - z0 for item in list(lower) + list(upper)]

    ratio = stack_asymmetry_ratio(offsets)
    if ratio is None or ratio <= float(max_ratio):
        return None

    rebuilt: List[Dict] = []
    previous_distance = 0.0
    for level, (d_low, d_up) in enumerate(
        zip(distances_lower, distances_upper), start=1
    ):
        distance = min(d_low, d_up)
        if distance <= previous_distance + 1e-9:
            # Niveles que colapsarían en el mismo plano: sin stack válido.
            return None
        previous_distance = distance

        pair = []
        for sign in (-1, 1):
            row = _measured_at(pool, z0 + sign * distance, tol_um)
            if row is None:
                return None
            score = float(row["score"])
            if score <= 0.0:
                return None
            pair.append(
                {
                    **row,
                    "drop_rel": max(0.0, (float(baseline) - score) / float(baseline)),
                    "distance_um": abs(float(row["z_um"]) - z0),
                    "curve_reused": True,
                    "symmetrized": True,
                    "symmetry_level": level,
                }
            )
        rebuilt.extend(pair)

    # Conservar el objetivo ΔS por nivel que traía el plan original: informa de
    # cuánto se sacrificó al simetrizar y queda registrado en el JSON del stack.
    targets = {}
    for item in list(lower) + list(upper):
        level_distance = abs(float(item["z_um"]) - z0)
        targets[round(level_distance, 6)] = float(
            item.get("target_drop_rel", 0.0)
        )
    ordered_targets = [targets[key] for key in sorted(targets)]
    for item in rebuilt:
        level = int(item["symmetry_level"])
        target = (
            ordered_targets[level - 1]
            if level - 1 < len(ordered_targets)
            else 0.0
        )
        item["target_drop_rel"] = target
        item["target_met"] = bool(float(item["drop_rel"]) + 1e-12 >= target)
        item["requested_target_drop_rel"] = target
        item["requested_target_met"] = item["target_met"]

    rebuilt_lower = sorted(
        [item for item in rebuilt if float(item["z_um"]) < z0],
        key=lambda item: float(item["z_um"]),
    )
    rebuilt_upper = sorted(
        [item for item in rebuilt if float(item["z_um"]) > z0],
        key=lambda item: float(item["z_um"]),
    )
    if len(rebuilt_lower) != len(lower) or len(rebuilt_upper) != len(upper):
        return None

    z_positions = (
        [float(item["z_um"]) for item in rebuilt_lower]
        + [z0]
        + [float(item["z_um"]) for item in rebuilt_upper]
    )
    if len({round(z, 6) for z in z_positions}) != len(z_positions):
        return None
    return z_positions, rebuilt_lower + rebuilt_upper

"""Registro visual fino entre tiles adyacentes (phase correlation)."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple, Union

import cv2
import numpy as np

logger = logging.getLogger("MotorControl_L206")


@dataclass
class RegistrationOffset:
    """Offset fino en píxeles tras registro visual."""

    dx_px: float = 0.0
    dy_px: float = 0.0
    response: float = 0.0
    method: str = "none"

    @property
    def applied(self) -> bool:
        return abs(self.dx_px) > 0.01 or abs(self.dy_px) > 0.01


def _to_gray(img: np.ndarray) -> np.ndarray:
    if img.ndim == 2:
        gray = img.astype(np.float32)
    else:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    if gray.max() > 255:
        gray = gray / max(gray.max(), 1.0) * 255.0
    return gray


def _overlap_strip(
    img_a: np.ndarray,
    img_b: np.ndarray,
    direction: str,
    strip_frac: float = 0.35,
) -> Tuple[np.ndarray, np.ndarray]:
    """Extrae bandas solapadas esperadas entre dos tiles vecinos."""
    ha, wa = img_a.shape[:2]
    hb, wb = img_b.shape[:2]
    sx = max(8, int(min(wa, wb) * strip_frac))
    sy = max(8, int(min(ha, hb) * strip_frac))

    if direction == "right":
        return img_a[:, wa - sx :], img_b[:, :sx]
    if direction == "left":
        return img_a[:, :sx], img_b[:, wb - sx :]
    if direction == "down":
        return img_a[ha - sy :, :], img_b[:sy, :]
    if direction == "up":
        return img_a[:sy, :], img_b[hb - sy :, :]
    raise ValueError(f"Dirección de overlap desconocida: {direction}")


def phase_correlate_pair(
    img_a: np.ndarray,
    img_b: np.ndarray,
    direction: str,
    max_shift_px: int = 64,
) -> RegistrationOffset:
    """
    Estima desplazamiento fino entre dos tiles usando phase correlation en banda solapada.

    direction indica dónde está img_b respecto a img_a (right = b está a la derecha de a).
    """
    try:
        strip_a, strip_b = _overlap_strip(img_a, img_b, direction)
        g_a = _to_gray(strip_a)
        g_b = _to_gray(strip_b)

        if g_a.size < 64 or g_b.size < 64:
            return RegistrationOffset(method="too_small")

        h = min(g_a.shape[0], g_b.shape[0])
        w = min(g_a.shape[1], g_b.shape[1])
        g_a = g_a[:h, :w]
        g_b = g_b[:h, :w]

        win = cv2.createHanningWindow((w, h), cv2.CV_32F)
        shift, response = cv2.phaseCorrelate(g_a * win, g_b * win)
        dx, dy = float(shift[0]), float(shift[1])

        if abs(dx) > max_shift_px or abs(dy) > max_shift_px:
            return RegistrationOffset(dx_px=0.0, dy_px=0.0, response=float(response), method="rejected")

        if direction == "right":
            fine_dx = dx
            fine_dy = dy
        elif direction == "left":
            fine_dx = -dx
            fine_dy = dy
        elif direction == "down":
            fine_dx = dx
            fine_dy = dy
        else:  # up
            fine_dx = dx
            fine_dy = -dy

        return RegistrationOffset(dx_px=fine_dx, dy_px=fine_dy, response=float(response), method="phase_correlate")
    except cv2.error as exc:
        logger.debug("[tile_registration] phaseCorrelate falló: %s", exc)
        return RegistrationOffset(method="cv2_error")


@dataclass
class TilePlacement:
    """Tile con posición base y refinamiento."""

    tile_id: int
    px: float
    py: float
    reg_dx: float = 0.0
    reg_dy: float = 0.0

    @property
    def final_px(self) -> float:
        return self.px + self.reg_dx

    @property
    def final_py(self) -> float:
        return self.py + self.reg_dy


def find_spatial_neighbors(
    placements: Sequence[TilePlacement],
    tile_w: int,
    tile_h: int,
    max_gap_px: float,
) -> List[Tuple[int, int, str]]:
    """
    Encuentra pares de tiles vecinos probables.

    Returns:
        Lista de (id_a, id_b, direction) donde direction es la posición de b respecto a a.
    """
    pairs: List[Tuple[int, int, str]] = []
    n = len(placements)
    for i in range(n):
        for j in range(i + 1, n):
            pi, pj = placements[i], placements[j]
            dx = pj.px - pi.px
            dy = pj.py - pi.py
            adx, ady = abs(dx), abs(dy)

            if ady < tile_h * 0.35 and abs(adx - tile_w) <= max_gap_px:
                pairs.append((i, j, "right" if dx > 0 else "left"))
            elif adx < tile_w * 0.35 and abs(ady - tile_h) <= max_gap_px:
                pairs.append((i, j, "down" if dy > 0 else "up"))
    return pairs


def refine_placements(
    images: Union[Sequence[np.ndarray], Callable[[int], np.ndarray]],
    placements: List[TilePlacement],
    tile_w: int,
    tile_h: int,
    max_shift_px: int = 48,
    min_response: float = 0.08,
    max_gap_px: Optional[float] = None,
) -> Tuple[List[TilePlacement], Dict[str, float]]:
    """
    Refina offsets de colocación usando pares de tiles vecinos.

    Aplica correcciones acumuladas sobre el tile "b" de cada par con buena correlación.
    ``images`` puede ser una secuencia en RAM o un loader ``(idx) -> ndarray`` bajo demanda.
    """
    if max_gap_px is None:
        max_gap_px = max(tile_w, tile_h) * 0.45

    def _load(idx: int) -> np.ndarray:
        if callable(images):
            return images(idx)
        return images[idx]

    neighbors = find_spatial_neighbors(placements, tile_w, tile_h, max_gap_px)
    responses: List[float] = []
    applied = 0

    for idx_a, idx_b, direction in neighbors:
        reg = phase_correlate_pair(
            _load(idx_a), _load(idx_b), direction, max_shift_px=max_shift_px
        )
        if reg.response < min_response:
            continue
        responses.append(reg.response)
        placements[idx_b].reg_dx += reg.dx_px * 0.5
        placements[idx_b].reg_dy += reg.dy_px * 0.5
        applied += 1

    metrics = {
        "neighbor_pairs": len(neighbors),
        "registrations_applied": applied,
        "mean_response": float(np.mean(responses)) if responses else 0.0,
        "max_response": float(np.max(responses)) if responses else 0.0,
    }
    return placements, metrics

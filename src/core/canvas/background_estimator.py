"""Estimación de color de fondo a partir de bordes de tiles."""

from __future__ import annotations

from typing import Iterable, List, Sequence, Tuple

import cv2
import numpy as np


def _border_samples(img: np.ndarray, margin_frac: float = 0.08) -> np.ndarray:
    """Muestras RGB de franjas perimetrales (zonas sin objeto típico)."""
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    h, w = img.shape[:2]
    mx = max(2, int(w * margin_frac))
    my = max(2, int(h * margin_frac))

    strips = [
        img[:my, :, :],
        img[h - my :, :, :],
        img[:, :mx, :],
        img[:, w - mx :, :],
    ]
    return np.concatenate([s.reshape(-1, 3) for s in strips], axis=0)


def estimate_background_bgr(
    image_paths: Sequence[str],
    margin_frac: float = 0.08,
    max_tiles: int = 40,
) -> Tuple[Tuple[int, int, int], float]:
    """
    Estima color BGR de fondo claro a partir de bordes de tiles.

    Returns:
        ((B, G, R), std_mean) — std media de los canales tras filtrar outliers.
    """
    samples: List[np.ndarray] = []
    paths = list(image_paths)[:max_tiles]

    for path in paths:
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            continue
        samples.append(_border_samples(img, margin_frac))

    if not samples:
        return (200, 210, 200), 0.0

    all_px = np.concatenate(samples, axis=0).astype(np.float32)
    lo = np.percentile(all_px, 5, axis=0)
    hi = np.percentile(all_px, 95, axis=0)
    mask = np.all((all_px >= lo) & (all_px <= hi), axis=1)
    filtered = all_px[mask] if mask.any() else all_px

    mean = np.median(filtered, axis=0)
    std = float(np.mean(np.std(filtered, axis=0)))
    bgr = tuple(int(round(v)) for v in mean)
    return bgr, std

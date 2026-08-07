"""Preview live de baja latencia: downscale + estimación de memoria.

El path de UI no debe pintar 2590×1942; AF/captura usan el full-res aparte.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

from PyQt5.QtGui import QImage


# Ancho máx. del QImage de preview (mantiene aspecto)
PREVIEW_MAX_WIDTH = 1280


@dataclass
class LivePipelineMetrics:
    """Indicadores del path live (worker → UI)."""

    frames_grabbed: int = 0
    preview_builds: int = 0
    preview_builds_skipped: int = 0
    frames_dropped_coalesce: int = 0
    frames_emitted_ui: int = 0
    last_grab_ms: float = 0.0
    last_preview_ms: float = 0.0
    last_full_w: int = 0
    last_full_h: int = 0
    last_preview_w: int = 0
    last_preview_h: int = 0

    def est_full_bytes(self) -> int:
        return int(self.last_full_w * self.last_full_h * 3)

    def est_preview_bytes(self) -> int:
        return int(self.last_preview_w * self.last_preview_h * 3)

    def snapshot(self) -> dict:
        return {
            "frames_grabbed": self.frames_grabbed,
            "preview_builds": self.preview_builds,
            "preview_builds_skipped": self.preview_builds_skipped,
            "frames_dropped_coalesce": self.frames_dropped_coalesce,
            "frames_emitted_ui": self.frames_emitted_ui,
            "last_grab_ms": round(self.last_grab_ms, 2),
            "last_preview_ms": round(self.last_preview_ms, 2),
            "full_wh": (self.last_full_w, self.last_full_h),
            "preview_wh": (self.last_preview_w, self.last_preview_h),
            "est_full_bytes": self.est_full_bytes(),
            "est_preview_bytes": self.est_preview_bytes(),
        }


def preview_target_size(
    width: int, height: int, max_width: int = PREVIEW_MAX_WIDTH
) -> Tuple[int, int]:
    """Tamaño de preview manteniendo aspecto."""
    w = int(width)
    h = int(height)
    mw = max(1, int(max_width))
    if w <= mw:
        return w, h
    nh = max(1, int(round(h * (mw / float(w)))))
    return mw, nh


def make_preview_bgr(
    frame_bgr: np.ndarray, max_width: int = PREVIEW_MAX_WIDTH
) -> np.ndarray:
    """BGR downscaled para UI. Puede devolver la misma vista si ya cabe."""
    if frame_bgr is None or getattr(frame_bgr, "size", 0) == 0:
        raise ValueError("frame_bgr vacío")
    h, w = frame_bgr.shape[:2]
    tw, th = preview_target_size(w, h, max_width)
    if tw == w and th == h:
        return frame_bgr
    if cv2 is None:
        # Fallback sin OpenCV: subsample simple
        ys = np.linspace(0, h - 1, th).astype(np.int32)
        xs = np.linspace(0, w - 1, tw).astype(np.int32)
        return np.ascontiguousarray(frame_bgr[ys][:, xs])
    return cv2.resize(frame_bgr, (tw, th), interpolation=cv2.INTER_AREA)


def bgr8_to_qimage_copy(frame_bgr: np.ndarray) -> QImage:
    """QImage RGB888 owned (copia) a partir de BGR8 contiguo."""
    frame = np.ascontiguousarray(frame_bgr)
    h, w = frame.shape[:2]
    bytes_per_line = 3 * w
    return QImage(
        frame.data, w, h, bytes_per_line, QImage.Format_RGB888
    ).rgbSwapped().copy()


def estimate_bgr_bytes(width: int, height: int) -> int:
    return max(0, int(width) * int(height) * 3)

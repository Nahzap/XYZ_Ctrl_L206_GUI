"""
Pipeline único de imagen científica (SRP).

ÚNICA vía de adquisición CMOS: ``worker.acquire_scientific_frame()``.

Basler (WYSIWYG): mismo BGR8 pylon+WB del preview → uint16 MSB
(``scientific_frame_from_preview_bgr8``). El PNG debe verse igual que la UI.

Otras cámaras / raw: ``prepare_scientific_bgr16`` (demosaic OpenCV + WB).

AF, detección de captura y guardado consumen solo ``ScientificFrame.image16``.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple, Union

import numpy as np

from core.utils.image_io import safe_imwrite
from hardware.camera.scientific_config import (
    align_12bit_to_uint16_msb,
    apply_channel_gains,
    bayer_opencv_code,
    estimate_brightfield_wb_gains,
)

logger = logging.getLogger("MotorControl_L206")

PathLike = Union[str, "os.PathLike[str]"]
WbGains = Tuple[float, float, float]

# Identidad del pipeline (metadatos / logs)
PIPELINE_ID = "scientific_bgr16_v1"
# Basler WYSIWYG: mismos píxeles que el preview pylon (BGR8→MSB16).
PIPELINE_ID_WYSIWYG = "scientific_bgr16_wysiwyg_pylon_v1"


@dataclass(frozen=True)
class ScientificFrame:
    """Frame científico producido solo por acquire → prepare."""

    image16: np.ndarray
    pixel_format: str
    wb_gains: WbGains
    frame_id: int
    timestamp_s: float
    raw: Optional[np.ndarray] = None
    pipeline_id: str = PIPELINE_ID

    @property
    def shape(self):
        return self.image16.shape

    @property
    def dtype(self):
        return self.image16.dtype


def prepare_scientific_bgr16(
    frame: np.ndarray,
    *,
    pixel_format: str = "BayerGB12",
    wb_gains: Optional[WbGains] = None,
) -> np.ndarray:
    """
    Única transformación a BGR16 científico.

    Args:
        frame: Bayer uint16 2D, Mono uint16 2D, o BGR uint16 3D.
        pixel_format: PixelFormat Basler (BayerGB12, Mono12, …).
        wb_gains: ganancias BGR ya estimadas (p. ej. del live). Si None y el
            frame es color, se estiman aquí una sola vez sobre este frame.

    Returns:
        uint16 HxW (mono) o HxWx3 BGR, empaquetado MSB para visores 16-bit.
    """
    import cv2

    arr = np.asarray(frame)
    if arr.dtype != np.uint16:
        raise ValueError(
            f"[{PIPELINE_ID}] frame debe ser uint16, recibido {arr.dtype}"
        )
    if arr.size == 0:
        raise ValueError(f"[{PIPELINE_ID}] frame vacío")

    pf = str(pixel_format or "")

    if arr.ndim == 2:
        if pf.startswith("Mono"):
            return align_12bit_to_uint16_msb(arr)

        code = bayer_opencv_code(pf)
        if code is None:
            raise ValueError(
                f"[{PIPELINE_ID}] PixelFormat no soportado: {pf or '?'}"
            )
        bgr = np.asarray(cv2.cvtColor(arr, code), dtype=np.uint16)
    elif arr.ndim == 3 and arr.shape[2] == 3:
        bgr = np.asarray(arr, dtype=np.uint16)
    else:
        raise ValueError(
            f"[{PIPELINE_ID}] shape inválida {getattr(arr, 'shape', None)}"
        )

    bgr = align_12bit_to_uint16_msb(bgr)
    gains = wb_gains
    if gains is None:
        gains = estimate_brightfield_wb_gains(bgr)
    return apply_channel_gains(bgr, gains)


def save_scientific_image(
    filepath: PathLike,
    frame: np.ndarray,
    *,
    pixel_format: str = "BayerGB12",
    wb_gains: Optional[WbGains] = None,
    already_prepared: bool = False,
    params: Optional[Sequence[int]] = None,
) -> bool:
    """
    Único punto de guardado de capturas científicas.

    Si ``already_prepared`` es True, ``frame`` ya es salida de
    ``prepare_scientific_bgr16`` (solo MSB-seguro + escritura).
    """
    import os

    if frame is None or getattr(frame, "size", 0) == 0:
        logger.error("[%s] save: frame vacío (%s)", PIPELINE_ID, filepath)
        return False

    try:
        if already_prepared:
            out = np.asarray(frame)
            if out.dtype != np.uint16:
                raise ValueError(
                    f"already_prepared exige uint16, recibido {out.dtype}"
                )
            # Defensa: nunca persistir LSB12 “casi negro”
            out = align_12bit_to_uint16_msb(out)
        else:
            out = prepare_scientific_bgr16(
                frame, pixel_format=pixel_format, wb_gains=wb_gains
            )
    except Exception as exc:
        logger.error("[%s] prepare falló (%s): %s", PIPELINE_ID, filepath, exc)
        return False

    encode_params = list(params) if params is not None else None
    ext = os.path.splitext(os.fspath(filepath))[1].lower()
    if encode_params is None and ext == ".png":
        import cv2

        encode_params = [cv2.IMWRITE_PNG_COMPRESSION, 6]

    ok = safe_imwrite(filepath, out, encode_params)
    if ok:
        logger.debug(
            "[%s] guardado %s shape=%s dtype=%s",
            PIPELINE_ID,
            filepath,
            out.shape,
            out.dtype,
        )
    return ok


def scientific_frame_from_preview_bgr8(
    bgr8: np.ndarray,
    *,
    frame_id: int,
    wb_gains: Optional[WbGains] = None,
    raw: Optional[np.ndarray] = None,
    pixel_format: str = "BGR8packed",
) -> ScientificFrame:
    """
    WYSIWYG Basler: el PNG es el preview pylon empaquetado a uint16 MSB.

    ``bgr8`` debe ser exactamente el frame mostrado (demosaic pylon + WB).
    ``image16 >> 8`` reproduce el preview píxel a píxel.
    """
    arr = np.asarray(bgr8)
    if arr.dtype != np.uint8 or arr.ndim != 3 or arr.shape[2] != 3:
        raise ValueError(
            f"[{PIPELINE_ID_WYSIWYG}] preview exige HxWx3 uint8, "
            f"recibido shape={getattr(arr, 'shape', None)} dtype={arr.dtype}"
        )
    if arr.size == 0:
        raise ValueError(f"[{PIPELINE_ID_WYSIWYG}] preview vacío")

    image16 = np.left_shift(arr.astype(np.uint16, copy=False), 8)
    gains = wb_gains if wb_gains is not None else (1.0, 1.0, 1.0)
    return ScientificFrame(
        image16=image16,
        raw=np.asarray(raw).copy() if raw is not None else None,
        pixel_format=str(pixel_format or "BGR8packed"),
        wb_gains=(float(gains[0]), float(gains[1]), float(gains[2])),
        frame_id=int(frame_id),
        timestamp_s=float(time.time()),
        pipeline_id=PIPELINE_ID_WYSIWYG,
    )


def scientific_frame_from_raw(
    raw: np.ndarray,
    *,
    pixel_format: str,
    wb_gains: Optional[WbGains],
    frame_id: int,
) -> ScientificFrame:
    """Construye ScientificFrame vía demosaic OpenCV (p. ej. Thorlabs / raw)."""
    # Estimar WB sobre demosaic+MSB *antes* de aplicar ganancias. Si se estima
    # sobre la imagen ya balanceada, sci.wb_gains≈(1,1,1) y el preview se
    # desincroniza del PNG.
    if wb_gains is None:
        neutral = prepare_scientific_bgr16(
            raw, pixel_format=pixel_format, wb_gains=(1.0, 1.0, 1.0)
        )
        if neutral.ndim == 3 and neutral.shape[2] == 3:
            gains = estimate_brightfield_wb_gains(neutral)
            image16 = apply_channel_gains(neutral, gains)
        else:
            gains = (1.0, 1.0, 1.0)
            image16 = neutral
    else:
        gains = wb_gains
        image16 = prepare_scientific_bgr16(
            raw, pixel_format=pixel_format, wb_gains=wb_gains
        )
    return ScientificFrame(
        image16=image16,
        raw=np.asarray(raw).copy(),
        pixel_format=str(pixel_format or ""),
        wb_gains=(float(gains[0]), float(gains[1]), float(gains[2])),
        frame_id=int(frame_id),
        timestamp_s=float(time.time()),
    )


def image16_to_u8_preview(image16: np.ndarray) -> np.ndarray:
    """Derivado de visualización/detección; NO es una adquisición alternativa."""
    arr = np.asarray(image16)
    if arr.dtype == np.uint8:
        return arr.copy()
    if arr.dtype != np.uint16:
        arr = arr.astype(np.uint16, copy=False)
    # MSB12/16 → uint8
    return np.right_shift(arr, 8).astype(np.uint8)

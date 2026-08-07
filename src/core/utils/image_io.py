"""
Escritura de imágenes compatible con rutas Unicode en Windows.

OpenCV ``cv2.imwrite`` / ``cv2.imread`` fallan en Windows cuando la ruta
contiene caracteres no-ASCII (p. ej. ANTÁRSEEDS). Python ``open()`` sí
soporta Unicode; por eso se usa ``imencode`` + escritura binaria.
"""

from __future__ import annotations

import logging
import os
from typing import Optional, Sequence, Union

import cv2
import numpy as np

logger = logging.getLogger("MotorControl_L206")

PathLike = Union[str, os.PathLike]


def _extension_for_imencode(path: str) -> str:
    ext = os.path.splitext(path)[1].lower()
    if not ext:
        return ".png"
    # OpenCV espera extensión con punto; .jpg y .jpeg son equivalentes.
    if ext == ".jpeg":
        return ".jpg"
    return ext


def safe_imwrite(
    filepath: PathLike,
    image: np.ndarray,
    params: Optional[Sequence[int]] = None,
) -> bool:
    """Guarda una imagen en cualquier ruta (incluye Unicode en Windows).

    Returns:
        True si el archivo se escribió correctamente.
    """
    if image is None:
        logger.error("[safe_imwrite] image es None")
        return False

    path = os.fspath(filepath)
    parent = os.path.dirname(path)
    if parent:
        try:
            os.makedirs(parent, exist_ok=True)
        except OSError as e:
            logger.error("[safe_imwrite] No se pudo crear carpeta %s: %s", parent, e)
            return False

    ext = _extension_for_imencode(path)
    try:
        encode_params = list(params) if params is not None else []
        ok, buf = cv2.imencode(ext, image, encode_params)
        if not ok or buf is None:
            logger.error(
                "[safe_imwrite] cv2.imencode falló (%s, shape=%s, dtype=%s)",
                ext,
                getattr(image, "shape", None),
                getattr(image, "dtype", None),
            )
            return False
        with open(path, "wb") as f:
            f.write(buf.tobytes())
        return True
    except Exception as e:
        logger.error("[safe_imwrite] Error escribiendo %s: %s", path, e)
        return False

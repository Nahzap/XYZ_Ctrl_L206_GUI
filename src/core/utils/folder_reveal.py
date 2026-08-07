"""Abrir carpeta en el explorador del SO."""

from __future__ import annotations

import logging
import os
import sys
from typing import Union

logger = logging.getLogger("MotorControl_L206")

PathLike = Union[str, os.PathLike]


def reveal_folder(path: PathLike, *, create: bool = True) -> bool:
    """Abre ``path`` en el explorador. Crea la carpeta si ``create`` y no existe."""
    folder = os.path.abspath(os.fspath(path))
    if not folder:
        return False
    if create and not os.path.isdir(folder):
        try:
            os.makedirs(folder, exist_ok=True)
        except OSError as exc:
            logger.error("[reveal_folder] No se pudo crear %s: %s", folder, exc)
            return False
    if not os.path.isdir(folder):
        logger.error("[reveal_folder] No es carpeta: %s", folder)
        return False
    try:
        if sys.platform == "win32":
            os.startfile(folder)  # type: ignore[attr-defined]
        elif sys.platform == "darwin":
            import subprocess

            subprocess.Popen(["open", folder])
        else:
            import subprocess

            subprocess.Popen(["xdg-open", folder])
        logger.info("[reveal_folder] Abierto: %s", folder)
        return True
    except Exception as exc:
        logger.error("[reveal_folder] Error abriendo %s: %s", folder, exc)
        return False

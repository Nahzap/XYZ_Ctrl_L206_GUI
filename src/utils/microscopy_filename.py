"""Convenciones de nombre de archivo para capturas de microscopía (XY en µm)."""

import os
import re
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class ParsedMicroscopyFilename:
    """Resultado de parsear un nombre de captura de microscopía."""

    class_name: str
    point_index_0based: int
    x_um: float
    y_um: float
    f_index: Optional[int]
    ext: str


_MICROSCOPY_FILENAME_RE = re.compile(
    r"^(.+)_(\d{4})_X([\d.]+)um_Y([\d.]+)um(?:_f(\d+))?\.(\w+)$"
)


def format_um_axis(um: float, axis: str) -> str:
    """Formatea coordenada en micrometros para nombre de archivo, p.ej. X12500um."""
    um = float(um)
    if abs(um - round(um)) < 0.01:
        return f"{axis}{int(round(um))}um"
    return f"{axis}{um:.1f}um"


def build_point_basename(
    class_name: str,
    point_index_0based: int,
    x_um: float,
    y_um: float,
) -> str:
    """
    Base del nombre: Class_0043_X12500um_Y15200um

    point_index_0based: indice de trayectoria (0-based); en el nombre se usa +1.
    """
    seq = point_index_0based + 1
    x_tag = format_um_axis(x_um, "X")
    y_tag = format_um_axis(y_um, "Y")
    return f"{class_name}_{seq:04d}_{x_tag}_{y_tag}"


def build_multifocal_filename(
    class_name: str,
    point_index_0based: int,
    x_um: float,
    y_um: float,
    f_index: int,
    ext: str = "png",
) -> str:
    """Ej.: Escalonia_pulverulenta_0043_X12500um_Y15200um_f1.png"""
    base = build_point_basename(class_name, point_index_0based, x_um, y_um)
    return f"{base}_f{f_index}.{ext}"


def build_single_capture_filename(
    class_name: str,
    point_index_0based: int,
    x_um: float,
    y_um: float,
    ext: str = "png",
) -> str:
    """Captura simple sin sufijo focal."""
    base = build_point_basename(class_name, point_index_0based, x_um, y_um)
    return f"{base}.{ext}"


def parse_microscopy_filename(path_or_name: str) -> Optional[ParsedMicroscopyFilename]:
    """
    Extrae clase, secuencia, XY (µm) y opcionalmente índice focal de un nombre de archivo.

    Ej.: Escalonia_pulverulenta_0043_X12500um_Y15200um_f1.png
    """
    basename = os.path.basename(path_or_name)
    match = _MICROSCOPY_FILENAME_RE.match(basename)
    if not match:
        return None

    class_name, seq, x_um, y_um, f_index, ext = match.groups()
    return ParsedMicroscopyFilename(
        class_name=class_name,
        point_index_0based=int(seq) - 1,
        x_um=float(x_um),
        y_um=float(y_um),
        f_index=int(f_index) if f_index is not None else None,
        ext=ext,
    )


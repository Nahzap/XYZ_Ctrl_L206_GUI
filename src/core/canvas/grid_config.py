"""Parámetros de rejilla FOV para montaje de canvas."""

import math
from dataclasses import dataclass


@dataclass
class GridConfig:
    """Rejilla zig-zag FOV-a-FOV (misma convención que TrajectoryGenerator)."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    fov_x: float
    fov_y: float
    snap_tolerance_um: float = 80.0

    @property
    def n_cols(self) -> int:
        return max(1, math.ceil((self.x_max - self.x_min) / self.fov_x))

    @property
    def n_rows(self) -> int:
        return max(1, math.ceil((self.y_max - self.y_min) / self.fov_y))

    @property
    def total_cells(self) -> int:
        return self.n_cols * self.n_rows

    def cell_from_xy(self, x_um: float, y_um: float) -> tuple[int, int]:
        """Índice de celda (col, row) con row=0 en y_min."""
        col = round((x_um - self.x_min) / self.fov_x)
        row = round((y_um - self.y_min) / self.fov_y)
        col = max(0, min(self.n_cols - 1, col))
        row = max(0, min(self.n_rows - 1, row))
        return col, row

    def xy_at_cell(self, col: int, row: int) -> tuple[float, float]:
        return self.x_min + col * self.fov_x, self.y_min + row * self.fov_y

    def um_to_pixel(
        self,
        x_um: float,
        y_um: float,
        tile_w: int,
        tile_h: int,
        swap_axes: bool = False,
    ) -> tuple[float, float]:
        """
        Convierte coordenadas físicas (µm) a posición en canvas (px).

        Convención normal: X stage → px (horizontal), Y stage → py (vertical, y_min abajo).
        Con ``swap_axes=True``: Y stage → px, X stage → py (cámara rotada 90°).
        """
        if swap_axes:
            px = (y_um - self.y_min) / self.fov_y * tile_w
            py = (self.x_max - x_um) / self.fov_x * tile_h
        else:
            px = (x_um - self.x_min) / self.fov_x * tile_w
            py = (self.y_max - y_um) / self.fov_y * tile_h
        return px, py

    def pixel_offset_from_nominal(
        self,
        x_actual_um: float,
        y_actual_um: float,
        x_nominal_um: float,
        y_nominal_um: float,
        tile_w: int,
        tile_h: int,
    ) -> tuple[float, float]:
        """Offset en píxeles entre posición real y nominal."""
        px_act, py_act = self.um_to_pixel(x_actual_um, y_actual_um, tile_w, tile_h)
        px_nom, py_nom = self.um_to_pixel(x_nominal_um, y_nominal_um, tile_w, tile_h)
        return px_act - px_nom, py_act - py_nom

    def snap_cell_from_xy(self, x_um: float, y_um: float) -> tuple[int, int]:
        """
        Asigna celda con tolerancia de snap: si el punto cae cerca del borde
        entre celdas, se asigna a la celda más cercana dentro de snap_tolerance_um.
        """
        col_f = (x_um - self.x_min) / self.fov_x
        row_f = (y_um - self.y_min) / self.fov_y
        col = int(round(col_f))
        row = int(round(row_f))

        if self.snap_tolerance_um > 0:
            tol_x = self.snap_tolerance_um / self.fov_x
            tol_y = self.snap_tolerance_um / self.fov_y
            if abs(col_f - round(col_f)) > 0.5 - tol_x:
                col = int(round(col_f))
            if abs(row_f - round(row_f)) > 0.5 - tol_y:
                row = int(round(row_f))

        col = max(0, min(self.n_cols - 1, col))
        row = max(0, min(self.n_rows - 1, row))
        return col, row

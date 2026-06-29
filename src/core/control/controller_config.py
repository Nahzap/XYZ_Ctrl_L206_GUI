"""Configuración compartida de controladores PI / H∞."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ControllerConfig:
    """Configuración de un controlador PI equivalente extraído de H∞."""

    Kp: float
    Ki: float
    U_max: float = 150.0
    invert: bool = False
    sensor_key: str = "sensor_1"
    K_plant: float = 1.0  # Ganancia planta G(s)=K/(τs+1) — normaliza error (µm/K → u)

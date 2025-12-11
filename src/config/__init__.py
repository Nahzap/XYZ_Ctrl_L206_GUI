"""Módulo de configuración del sistema."""

from .constants import *
from .settings import setup_logging

__all__ = [
    'SERIAL_PORT',
    'BAUD_RATE',
    'PLOT_LENGTH',
    'ADC_MAX',
    'RECORRIDO_UM',
    'FACTOR_ESCALA',
    'setup_logging'
]

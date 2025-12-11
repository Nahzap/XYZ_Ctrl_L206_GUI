"""
Modelos de datos para el sistema de control.

Encapsulan estructuras de datos y validaciones.
"""

from .motor_state import MotorState
from .sensor_data import SensorData
from .system_config import SystemConfig

__all__ = ['MotorState', 'SensorData', 'SystemConfig']

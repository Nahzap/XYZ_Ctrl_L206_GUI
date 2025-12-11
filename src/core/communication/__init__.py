"""Módulo de comunicación serial."""

from .serial_handler import SerialHandler
from .protocol import MotorProtocol

__all__ = ['SerialHandler', 'MotorProtocol']

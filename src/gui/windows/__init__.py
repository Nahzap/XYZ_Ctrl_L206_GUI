"""
Ventanas auxiliares del sistema.

Este módulo contiene las ventanas independientes para visualización:
- MatplotlibWindow: Ventanas para gráficos matplotlib
- SignalWindow: Ventana de señales en tiempo real
- CameraViewWindow: Ventana para vista de cámara
"""

from .matplotlib_window import MatplotlibWindow
from .signal_window import SignalWindow
from .camera_window import CameraViewWindow

__all__ = ['MatplotlibWindow', 'SignalWindow', 'CameraViewWindow']

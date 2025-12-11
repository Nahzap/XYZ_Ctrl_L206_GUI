"""
Módulo de pestañas de la interfaz GUI.

Cada pestaña es una clase independiente que encapsula su lógica y widgets.
"""

from .base_tab import BaseTab
from .recording_tab import RecordingTab
from .analysis_tab import AnalysisTab
from .camera_tab import CameraTab
from .control_tab import ControlTab
from .test_tab import TestTab
from .hinf_tab import HInfTab
from .img_analysis_tab import ImgAnalysisTab

__all__ = [
    'BaseTab', 
    'RecordingTab', 
    'AnalysisTab', 
    'CameraTab',
    'ControlTab',
    'TestTab',
    'HInfTab',
    'ImgAnalysisTab'
]

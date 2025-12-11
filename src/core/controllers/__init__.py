"""
Módulo de controladores para el sistema.

Contiene implementaciones de controladores H∞ y otros algoritmos de control.
"""

from .hinf_controller import HInfController, SynthesisConfig, SynthesisResult

__all__ = ['HInfController', 'SynthesisConfig', 'SynthesisResult']

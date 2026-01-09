"""
Módulo de controladores del sistema.

Contiene el controlador H∞ para control de motores.
"""

from .hinf_controller import (
    HInfController,
    SynthesisConfig,
    SynthesisResult,
    ValidationResult
)

__all__ = [
    'HInfController',
    'SynthesisConfig',
    'SynthesisResult',
    'ValidationResult'
]

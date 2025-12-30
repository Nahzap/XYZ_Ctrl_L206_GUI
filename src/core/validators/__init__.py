"""
Validadores del sistema.

Centralizan la lógica de validación para evitar duplicación.
"""

from .microscopy_validator import MicroscopyValidator, MicroscopyConfig, ValidationResult

__all__ = ['MicroscopyValidator', 'MicroscopyConfig', 'ValidationResult']

"""
Módulo de análisis de función de transferencia.

Contiene herramientas para identificar parámetros K y τ a partir de
datos experimentales de respuesta al escalón.
"""

from .transfer_function_analyzer import TransferFunctionAnalyzer

__all__ = ['TransferFunctionAnalyzer']

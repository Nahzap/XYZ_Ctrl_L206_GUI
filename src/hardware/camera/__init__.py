"""
Módulo de integración con cámaras Thorlabs.

Contiene el worker para manejar la cámara en un thread separado.
"""

from .camera_worker import CameraWorker

__all__ = ['CameraWorker']

"""
Verificación centralizada de disponibilidad de hardware.

Este módulo centraliza la verificación de disponibilidad de SDKs y hardware
para evitar duplicación de código en múltiples archivos.

Uso:
    from config.hardware_availability import THORLABS_AVAILABLE, Thorlabs
"""

import logging

logger = logging.getLogger('MotorControl_L206')

# =========================================================================
# THORLABS CAMERA SDK
# =========================================================================
THORLABS_AVAILABLE = False
Thorlabs = None

try:
    import pylablib as pll
    # Configurar la ruta del SDK de Thorlabs
    pll.par["devices/dlls/thorlabs_tlcam"] = r"C:\Program Files\Thorlabs\ThorImageCAM\Bin"
    from pylablib.devices import Thorlabs as _Thorlabs
    Thorlabs = _Thorlabs
    THORLABS_AVAILABLE = True
    logger.info("[HardwareAvailability] Thorlabs SDK disponible")
except ImportError:
    logger.warning("[HardwareAvailability] pylablib no instalado - cámara Thorlabs deshabilitada")
except Exception as e:
    logger.warning(f"[HardwareAvailability] Error configurando Thorlabs SDK: {e}")


# =========================================================================
# PYTORCH / CUDA
# =========================================================================
TORCH_AVAILABLE = False
CUDA_AVAILABLE = False

try:
    import torch
    TORCH_AVAILABLE = True
    CUDA_AVAILABLE = torch.cuda.is_available()
    if CUDA_AVAILABLE:
        logger.info(f"[HardwareAvailability] PyTorch con CUDA disponible: {torch.cuda.get_device_name(0)}")
    else:
        logger.info("[HardwareAvailability] PyTorch disponible (CPU only)")
except ImportError:
    logger.warning("[HardwareAvailability] PyTorch no disponible")


__all__ = [
    'THORLABS_AVAILABLE',
    'Thorlabs',
    'TORCH_AVAILABLE',
    'CUDA_AVAILABLE',
]

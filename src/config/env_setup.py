"""
Configuración del entorno Python para evitar conflictos de librerías.

Este módulo DEBE importarse ANTES de cualquier otra librería numérica
(numpy, scipy, torch, etc.) para evitar conflictos de OpenMP.

El problema: PyTorch usa Intel MKL (libiomp5md.dll) mientras que
NumPy/SciPy pueden usar LLVM OpenMP (libomp.dll). Si ambos se cargan,
el programa crashea con "OMP: Error #15".

Solución: Establecer KMP_DUPLICATE_LIB_OK=TRUE antes de importar cualquier
librería que use OpenMP. Esta es la solución oficial documentada por Intel
para entornos donde múltiples runtimes de OpenMP coexisten.

Referencia: http://openmp.llvm.org/
"""

import os

# CRITICAL: Set this BEFORE any library imports
# This allows multiple OpenMP runtimes to coexist (PyTorch's MKL + SciPy's LLVM)
os.environ['KMP_DUPLICATE_LIB_OK'] = 'TRUE'

# Also set MKL threading layer to avoid conflicts
os.environ['MKL_THREADING_LAYER'] = 'GNU'

def setup_environment():
    """
    Configura el entorno para evitar conflictos de OpenMP.
    Esta función se llama automáticamente al importar el módulo.
    
    Returns:
        bool: True si PyTorch está disponible
    """
    import sys
    
    # Obtener el directorio del entorno virtual
    venv_path = os.path.dirname(os.path.dirname(sys.executable))
    
    # Identificar paths de usuario global que pueden causar conflictos
    user_site_packages = os.path.join(
        os.environ.get('APPDATA', ''), 
        'Python', 
        f'Python{sys.version_info.major}{sys.version_info.minor}',
        'site-packages'
    )
    
    # Reordenar sys.path: priorizar entorno virtual
    new_path = []
    user_paths = []
    
    for p in sys.path:
        if user_site_packages in p:
            user_paths.append(p)  # Mover al final
        else:
            new_path.append(p)
    
    # Agregar paths de usuario al final (menor prioridad)
    sys.path = new_path + user_paths
    
    # Importar PyTorch para verificar disponibilidad
    try:
        import torch
        _torch_loaded = True
    except ImportError:
        _torch_loaded = False
    
    return _torch_loaded

# Ejecutar setup al importar este módulo
_pytorch_available = setup_environment()

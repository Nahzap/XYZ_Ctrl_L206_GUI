"""
Mad City Labs C-Focus Piezo Stage Controller
Wrapper para integración con sistema de microscopía.
"""

import ctypes
import time
import logging
from typing import Optional, Tuple

logger = logging.getLogger('MotorControl_L206')


class CFocusController:
    """
    Controlador para piezo C-Focus de Mad City Labs.
    
    Funciones principales:
    - Inicializar/liberar handle del dispositivo
    - Mover a posición Z absoluta (µm)
    - Leer posición Z actual
    - Obtener rango calibrado
    """
    
    def __init__(self, dll_path: str = r"D:\MCL C Focus\Program Files 64\Mad City Labs\NanoDrive\Madlib.dll"):
        """
        Inicializa el controlador C-Focus.
        
        Args:
            dll_path: Ruta al DLL de Mad City Labs
        """
        self.dll_path = dll_path
        self.mcl_dll = None
        self.handle = 0
        self.z_range = 0.0
        self.is_connected = False
        
        self.settle_time = 0.15
        
    def connect(self) -> Tuple[bool, str]:
        """
        Conecta con el dispositivo C-Focus.
        
        Returns:
            (success, message): Tupla con estado y mensaje
        """
        try:
            logger.info(f"Cargando C-Focus DLL: {self.dll_path}")
            self.mcl_dll = ctypes.WinDLL(self.dll_path)
            
            self._setup_function_signatures()
            
            logger.info("Inicializando C-Focus handle...")
            self.handle = self.mcl_dll.MCL_InitHandle()
            
            if self.handle == 0:
                return False, "Error: No se pudo inicializar handle (dispositivo no conectado o en uso)"
            
            self.z_range = self.mcl_dll.MCL_GetCalibration(3, self.handle)
            
            if self.z_range <= 0:
                self.disconnect()
                return False, f"Error: Rango Z inválido ({self.z_range} µm)"
            
            self.is_connected = True
            logger.info(f"C-Focus conectado. Handle: {self.handle}, Rango Z: {self.z_range:.2f} µm")
            return True, f"C-Focus conectado (Rango: 0-{self.z_range:.1f} µm)"
            
        except FileNotFoundError:
            return False, f"DLL no encontrado: {self.dll_path}"
        except OSError as e:
            return False, f"Error cargando DLL: {e}"
        except Exception as e:
            logger.error(f"Error conectando C-Focus: {e}")
            return False, f"Error inesperado: {e}"
    
    def disconnect(self):
        """Desconecta y libera el handle del dispositivo."""
        if self.handle != 0 and self.mcl_dll is not None:
            try:
                logger.info("Liberando C-Focus handle...")
                self.mcl_dll.MCL_ReleaseHandle(self.handle)
                logger.info("C-Focus desconectado correctamente")
            except Exception as e:
                logger.error(f"Error liberando handle: {e}")
            finally:
                self.handle = 0
                self.is_connected = False
    
    def move_z(self, position_um: float) -> bool:
        """
        Mueve el piezo a una posición Z absoluta.
        
        Args:
            position_um: Posición en micrómetros (0 a z_range)
            
        Returns:
            bool: True si el movimiento fue exitoso
        """
        if not self.is_connected:
            logger.error("C-Focus no conectado")
            return False
        
        if position_um < 0 or position_um > self.z_range:
            logger.error(f"Posición Z fuera de rango: {position_um} µm (max: {self.z_range})")
            return False
        
        try:
            error_code = self.mcl_dll.MCL_SingleWriteZ(position_um, self.handle)
            
            if error_code != 0:
                logger.warning(f"C-Focus move retornó código {error_code}")
                return False
            
            time.sleep(self.settle_time)
            return True
            
        except Exception as e:
            logger.error(f"Error moviendo C-Focus: {e}")
            return False
    
    def read_z(self) -> Optional[float]:
        """
        Lee la posición Z actual del piezo.
        
        Returns:
            float: Posición en µm, o None si hay error
        """
        if not self.is_connected:
            logger.error("C-Focus no conectado")
            return None
        
        try:
            position = self.mcl_dll.MCL_SingleReadZ(self.handle)
            return float(position)
        except Exception as e:
            logger.error(f"Error leyendo posición Z: {e}")
            return None
    
    def get_z_range(self) -> float:
        """Retorna el rango calibrado del eje Z."""
        return self.z_range
    
    def _setup_function_signatures(self):
        """Configura los tipos de argumentos y retorno para funciones MCL."""
        self.mcl_dll.MCL_InitHandle.argtypes = []
        self.mcl_dll.MCL_InitHandle.restype = ctypes.c_int
        
        self.mcl_dll.MCL_SingleReadZ.argtypes = [ctypes.c_int]
        self.mcl_dll.MCL_SingleReadZ.restype = ctypes.c_double
        
        self.mcl_dll.MCL_SingleWriteZ.argtypes = [ctypes.c_double, ctypes.c_int]
        self.mcl_dll.MCL_SingleWriteZ.restype = ctypes.c_int
        
        self.mcl_dll.MCL_ReleaseHandle.argtypes = [ctypes.c_int]
        self.mcl_dll.MCL_ReleaseHandle.restype = None
        
        self.mcl_dll.MCL_GetCalibration.argtypes = [ctypes.c_int, ctypes.c_int]
        self.mcl_dll.MCL_GetCalibration.restype = ctypes.c_double

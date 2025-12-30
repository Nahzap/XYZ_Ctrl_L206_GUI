"""
Mad City Labs C-Focus Piezo Stage Controller
Wrapper para integración con sistema de microscopía.

Incluye sistema de offset BPoF (Best Plane of Focus) para trabajar
con posiciones relativas al punto de mejor enfoque.
"""

import ctypes
import time
import logging
from typing import Optional, Tuple
from dataclasses import dataclass

logger = logging.getLogger('MotorControl_L206')


@dataclass
class PositionManager:
    """Gestiona posiciones relativas al BPoF con offset automático.
    
    Permite trabajar con posiciones relativas al Best Plane of Focus:
    - posicion_absoluta = posicion_relativa + offset
    - offset = z_range / 2 (punto medio del recorrido)
    """
    
    z_range: float              # Rango total del piezo (ej: 100 µm)
    bpof_offset: float = None   # Offset = z_range / 2 por defecto
    upper_limit: float = None   # Límite superior desde BPoF
    lower_limit: float = None   # Límite inferior desde BPoF
    
    def __post_init__(self):
        if self.bpof_offset is None:
            self.bpof_offset = self.z_range / 2
        if self.upper_limit is None:
            self.upper_limit = self.z_range / 2
        if self.lower_limit is None:
            self.lower_limit = self.z_range / 2
    
    @classmethod
    def from_symmetric_range(cls, z_range: float, travel_distance: float = None) -> 'PositionManager':
        """Crea manager con rango simétrico alrededor del BPoF.
        
        Args:
            z_range: Rango total del hardware
            travel_distance: Distancia de viaje desde BPoF (si None, usa z_range/2)
        """
        offset = z_range / 2
        if travel_distance is None:
            travel_distance = z_range / 2
        return cls(
            z_range=z_range,
            bpof_offset=offset,
            upper_limit=travel_distance,
            lower_limit=travel_distance
        )
    
    def relative_to_absolute(self, relative_pos: float) -> float:
        """Convierte posición relativa a absoluta.
        
        Args:
            relative_pos: Posición relativa al BPoF (negativo=abajo, positivo=arriba)
            
        Returns:
            Posición absoluta para enviar al hardware
        """
        return relative_pos + self.bpof_offset
    
    def absolute_to_relative(self, absolute_pos: float) -> float:
        """Convierte posición absoluta a relativa al BPoF."""
        return absolute_pos - self.bpof_offset
    
    def validate_relative(self, relative_pos: float) -> Tuple[bool, str]:
        """Valida si una posición relativa está en rango."""
        if relative_pos > self.upper_limit:
            return False, f"Excede límite superior (+{self.upper_limit:.2f} µm)"
        if relative_pos < -self.lower_limit:
            return False, f"Excede límite inferior (-{self.lower_limit:.2f} µm)"
        
        absolute = self.relative_to_absolute(relative_pos)
        if absolute < 0 or absolute > self.z_range:
            return False, f"Posición absoluta {absolute:.2f} fuera de rango hardware"
        
        return True, "OK"
    
    def get_working_range(self) -> Tuple[float, float]:
        """Retorna el rango de trabajo relativo (min, max)."""
        return (-self.lower_limit, self.upper_limit)
    
    def get_center_position(self) -> float:
        """Retorna la posición central (BPoF) en coordenadas absolutas."""
        return self.bpof_offset
    
    def set_bpof_from_current(self, current_z: float):
        """Establece el BPoF actual como nuevo offset."""
        self.bpof_offset = current_z
        logger.info(f"[PositionManager] BPoF establecido en Z={current_z:.2f} µm")


class CFocusController:
    """
    Controlador para piezo C-Focus de Mad City Labs.
    
    Funciones principales:
    - Inicializar/liberar handle del dispositivo
    - Mover a posición Z absoluta (µm)
    - Leer posición Z actual
    - Obtener rango calibrado
    - Sistema de offset BPoF para posiciones relativas
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
        
        # Calibración real del hardware
        self.z_min_calibrated = 0.0
        self.z_max_calibrated = 0.0
        self.z_center_calibrated = None  # Centro REAL de calibración
        self.z_range_calibrated = 0.0
        
        # Sistema de offset BPoF
        self.position_manager: Optional[PositionManager] = None
        
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
        
        # Usar rango calibrado si está disponible, sino usar z_range inicial
        max_range = self.z_max_calibrated if self.z_max_calibrated > 0 else self.z_range
        
        if position_um < 0 or position_um > max_range:
            logger.error(f"Posición Z fuera de rango: {position_um} µm (rango: 0-{max_range:.2f})")
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
        # Usar rango calibrado si existe, sino usar rango de hardware
        return self.z_range_calibrated if self.z_range_calibrated > 0 else self.z_range
    
    def get_z_center(self) -> float:
        """Retorna el centro calibrado del eje Z.
        
        Returns:
            Centro calibrado si existe, sino z_range/2
        """
        if self.z_center_calibrated is not None:
            return self.z_center_calibrated
        return self.z_range / 2.0
    
    def get_calibration_info(self) -> dict:
        """Retorna información completa de calibración."""
        return {
            'z_min': self.z_min_calibrated,
            'z_max': self.z_max_calibrated,
            'z_center': self.z_center_calibrated,
            'z_range': self.z_range_calibrated,
            'is_calibrated': self.z_center_calibrated is not None
        }
    
    def get_center_position(self) -> float:
        """Retorna la posición central del recorrido (z_range / 2)."""
        return self.z_range / 2 if self.z_range > 0 else 0.0
    
    def setup_bpof_mode(self, travel_distance: float = None) -> bool:
        """Configura modo BPoF con rango simétrico.
        
        Args:
            travel_distance: Distancia de viaje desde BPoF (si None, usa z_range/2)
            
        Returns:
            True si se configuró correctamente
        """
        if not self.is_connected:
            logger.error("[CFocus] No conectado - no se puede configurar BPoF")
            return False
        
        self.position_manager = PositionManager.from_symmetric_range(
            z_range=self.z_range,
            travel_distance=travel_distance
        )
        
        center = self.position_manager.get_center_position()
        logger.info(f"[CFocus] Modo BPoF configurado: centro={center:.1f}µm, rango=±{travel_distance or self.z_range/2:.1f}µm")
        return True
    
    def move_to_center(self) -> bool:
        """Mueve el piezo a la posición central (mitad del recorrido).
        
        Returns:
            True si el movimiento fue exitoso
        """
        center = self.get_center_position()
        logger.info(f"[CFocus] Moviendo a posición central: Z={center:.1f}µm")
        return self.move_z(center)
    
    def move_z_relative(self, relative_pos: float) -> bool:
        """Mueve a posición relativa al BPoF.
        
        Args:
            relative_pos: Posición en µm relativa al BPoF
                         (negativo=debajo, positivo=arriba)
        
        Returns:
            True si el movimiento fue exitoso
        """
        if self.position_manager is None:
            logger.warning("[CFocus] Modo BPoF no configurado, usando move_z directo")
            return self.move_z(relative_pos)
        
        valid, msg = self.position_manager.validate_relative(relative_pos)
        if not valid:
            logger.error(f"[CFocus] Posición relativa inválida: {msg}")
            return False
        
        absolute_pos = self.position_manager.relative_to_absolute(relative_pos)
        logger.debug(f"[CFocus] move_z_relative({relative_pos:+.1f}) → absoluto={absolute_pos:.1f}µm")
        return self.move_z(absolute_pos)
    
    def read_z_relative(self) -> Optional[float]:
        """Lee la posición Z actual relativa al BPoF.
        
        Returns:
            Posición relativa en µm, o None si hay error
        """
        absolute = self.read_z()
        if absolute is None:
            return None
        
        if self.position_manager is None:
            return absolute
        
        return self.position_manager.absolute_to_relative(absolute)
    
    def set_current_as_bpof(self) -> bool:
        """Establece la posición actual como BPoF.
        
        Returns:
            True si se estableció correctamente
        """
        current = self.read_z()
        if current is None:
            logger.error("[CFocus] No se pudo leer posición actual")
            return False
        
        if self.position_manager is None:
            self.setup_bpof_mode()
        
        self.position_manager.set_bpof_from_current(current)
        logger.info(f"[CFocus] BPoF establecido en Z={current:.2f}µm")
        return True
    
    def get_bpof_info(self) -> dict:
        """Retorna información del estado BPoF actual."""
        if self.position_manager is None:
            return {
                'configured': False,
                'center': self.get_center_position(),
                'z_range': self.z_range
            }
        
        return {
            'configured': True,
            'center': self.position_manager.bpof_offset,
            'upper_limit': self.position_manager.upper_limit,
            'lower_limit': self.position_manager.lower_limit,
            'z_range': self.z_range,
            'working_range': self.position_manager.get_working_range()
        }
    
    def calibrate_limits(self, callback=None) -> dict:
        """Calibra los límites del C-Focus mediante escaneo completo.
        
        Mueve el piezo desde Z=0 hasta Z_max para detectar los límites
        físicos reales del sistema.
        
        Args:
            callback: Función opcional para reportar progreso (z_current, z_max)
            
        Returns:
            dict con 'z_min', 'z_max', 'z_center', 'z_range'
        """
        if not self.is_connected:
            logger.error("[CFocus] No conectado - no se puede calibrar")
            return None
        
        logger.info("[CFocus] ===== INICIANDO CALIBRACIÓN DE LÍMITES =====")
        
        # Obtener rango del hardware
        z_max_hw = self.z_range
        logger.info(f"[CFocus] Rango hardware: 0 - {z_max_hw:.2f} µm")
        
        # Mover a Z=0 (límite inferior)
        logger.info("[CFocus] Moviendo a Z=0 (límite inferior)...")
        self.move_z(0.0)
        time.sleep(0.5)
        z_min_actual = self.read_z()
        logger.info(f"[CFocus] ✓ Límite inferior confirmado: Z={z_min_actual:.2f} µm")
        
        # Mover a Z_max (límite superior)
        logger.info(f"[CFocus] Moviendo a Z={z_max_hw:.2f} (límite superior)...")
        self.move_z(z_max_hw)
        time.sleep(0.5)
        z_max_actual = self.read_z()
        logger.info(f"[CFocus] ✓ Límite superior confirmado: Z={z_max_actual:.2f} µm")
        
        # Calcular centro
        z_center = (z_min_actual + z_max_actual) / 2.0
        z_range_actual = z_max_actual - z_min_actual
        
        # ALMACENAR VALORES CALIBRADOS
        self.z_min_calibrated = z_min_actual
        self.z_max_calibrated = z_max_actual
        self.z_center_calibrated = z_center
        self.z_range_calibrated = z_range_actual
        self.z_range = z_range_actual  # Actualizar z_range con valor real
        
        # Mover al centro
        logger.info(f"[CFocus] Moviendo al CENTRO: Z={z_center:.2f} µm...")
        self.move_z(z_center)
        time.sleep(0.3)
        
        # Configurar modo BPoF con el centro calibrado
        self.setup_bpof_mode()
        
        # Actualizar PositionManager con centro real
        if self.position_manager:
            self.position_manager.bpof_offset = z_center
            self.position_manager.z_range = z_range_actual
            logger.info(f"[CFocus] PositionManager actualizado: offset={z_center:.2f}µm, range={z_range_actual:.2f}µm")
        
        result = {
            'z_min': z_min_actual,
            'z_max': z_max_actual,
            'z_center': z_center,
            'z_range': z_range_actual
        }
        
        logger.info("[CFocus] ===== CALIBRACIÓN COMPLETADA =====")
        logger.info(f"[CFocus] Límites: {z_min_actual:.2f} - {z_max_actual:.2f} µm")
        logger.info(f"[CFocus] Centro: {z_center:.2f} µm")
        logger.info(f"[CFocus] Rango: {z_range_actual:.2f} µm")
        
        return result
    
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

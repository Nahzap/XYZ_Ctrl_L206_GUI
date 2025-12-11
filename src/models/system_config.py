"""
Configuración del sistema.
"""

from dataclasses import dataclass, field
from typing import Optional, Dict


@dataclass
class SystemConfig:
    """Configuración global del sistema."""
    
    # Comunicación serial
    serial_port: str = 'COM5'
    baud_rate: int = 115200
    
    # Buffer de señales
    plot_length: int = 200
    
    # Calibración
    calibration: Optional[Dict] = None
    
    # Límites de seguridad
    max_power: int = 255
    min_power: int = -255
    
    # Timeouts
    serial_timeout: float = 0.1
    command_delay: float = 0.05
    
    # Paths
    data_directory: str = "data/"
    log_directory: str = "logs/"
    
    def __post_init__(self):
        """Valida la configuración."""
        if not 0 < self.baud_rate <= 921600:
            raise ValueError(f"baud_rate inválido: {self.baud_rate}")
        
        if not 10 <= self.plot_length <= 10000:
            raise ValueError(f"plot_length debe estar entre 10-10000: {self.plot_length}")
        
        if not -255 <= self.min_power <= 0:
            raise ValueError(f"min_power inválido: {self.min_power}")
        
        if not 0 <= self.max_power <= 255:
            raise ValueError(f"max_power inválido: {self.max_power}")
    
    def to_dict(self) -> dict:
        """Convierte la configuración a diccionario."""
        return {
            'serial_port': self.serial_port,
            'baud_rate': self.baud_rate,
            'plot_length': self.plot_length,
            'calibration': self.calibration,
            'max_power': self.max_power,
            'min_power': self.min_power,
            'serial_timeout': self.serial_timeout,
            'command_delay': self.command_delay,
            'data_directory': self.data_directory,
            'log_directory': self.log_directory
        }
    
    @classmethod
    def from_dict(cls, config_dict: dict):
        """Crea SystemConfig desde diccionario."""
        return cls(**config_dict)

"""
Modelo de datos de sensores.
"""

from dataclasses import dataclass
from datetime import datetime
from typing import Optional


@dataclass
class SensorData:
    """Representa una lectura de sensores."""
    
    sensor_1: int  # Valor ADC sensor 1
    sensor_2: int  # Valor ADC sensor 2
    power_a: int  # Potencia motor A
    power_b: int  # Potencia motor B
    timestamp: Optional[datetime] = None
    
    def __post_init__(self):
        """Valida y completa los datos."""
        if self.timestamp is None:
            self.timestamp = datetime.now()
        
        # Validar rangos
        if not 0 <= self.sensor_1 <= 1023:
            raise ValueError(f"sensor_1 debe estar entre 0-1023: {self.sensor_1}")
        if not 0 <= self.sensor_2 <= 1023:
            raise ValueError(f"sensor_2 debe estar entre 0-1023: {self.sensor_2}")
        if not -255 <= self.power_a <= 255:
            raise ValueError(f"power_a debe estar entre -255 y 255: {self.power_a}")
        if not -255 <= self.power_b <= 255:
            raise ValueError(f"power_b debe estar entre -255 y 255: {self.power_b}")
    
    @classmethod
    def from_serial(cls, line: str):
        """
        Crea SensorData desde línea serial.
        
        Formato esperado: "power_a,power_b,sensor_1,sensor_2"
        """
        try:
            parts = line.strip().split(',')
            if len(parts) != 4:
                raise ValueError(f"Formato inválido, esperado 4 valores: {line}")
            
            power_a, power_b, sensor_1, sensor_2 = map(int, parts)
            
            return cls(
                sensor_1=sensor_1,
                sensor_2=sensor_2,
                power_a=power_a,
                power_b=power_b
            )
        except (ValueError, IndexError) as e:
            raise ValueError(f"Error parseando línea serial: {e}")
    
    def to_csv_row(self, start_time: datetime) -> list:
        """Convierte a fila CSV con timestamp relativo."""
        if self.timestamp and start_time:
            time_ms = int((self.timestamp - start_time).total_seconds() * 1000)
        else:
            time_ms = 0
        
        return [time_ms, self.power_a, self.power_b, self.sensor_1, self.sensor_2]

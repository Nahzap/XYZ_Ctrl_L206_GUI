"""
Modelo de estado de motor.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class MotorState:
    """Representa el estado de un motor."""
    
    motor_id: str  # 'A' o 'B'
    power: int = 0  # -255 a 255
    direction: str = 'STOP'  # 'FORWARD', 'BACKWARD', 'STOP'
    mode: str = 'MANUAL'  # 'MANUAL' o 'AUTO'
    target_position: Optional[float] = None  # Posición objetivo (µm)
    current_position: Optional[float] = None  # Posición actual (µm)
    
    def __post_init__(self):
        """Valida los valores después de inicialización."""
        if self.motor_id not in ['A', 'B']:
            raise ValueError(f"motor_id debe ser 'A' o 'B', recibido: {self.motor_id}")
        
        if not -255 <= self.power <= 255:
            raise ValueError(f"power debe estar entre -255 y 255, recibido: {self.power}")
        
        if self.direction not in ['FORWARD', 'BACKWARD', 'STOP']:
            raise ValueError(f"direction inválida: {self.direction}")
        
        if self.mode not in ['MANUAL', 'AUTO']:
            raise ValueError(f"mode inválido: {self.mode}")
    
    def to_command(self) -> str:
        """Convierte el estado a comando serial."""
        if self.power == 0:
            return f"{self.motor_id}S"  # Stop
        elif self.power > 0:
            return f"{self.motor_id}F{abs(self.power)}"  # Forward
        else:
            return f"{self.motor_id}B{abs(self.power)}"  # Backward
    
    @classmethod
    def from_command(cls, command: str):
        """Crea un MotorState desde un comando serial."""
        if len(command) < 2:
            raise ValueError(f"Comando inválido: {command}")
        
        motor_id = command[0]
        direction_char = command[1]
        power_str = command[2:] if len(command) > 2 else "0"
        
        direction_map = {'F': 'FORWARD', 'B': 'BACKWARD', 'S': 'STOP'}
        direction = direction_map.get(direction_char, 'STOP')
        
        try:
            power = int(power_str) if direction != 'STOP' else 0
            if direction == 'BACKWARD':
                power = -power
        except ValueError:
            power = 0
        
        return cls(motor_id=motor_id, power=power, direction=direction)

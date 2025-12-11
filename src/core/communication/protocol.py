"""Protocolo de comunicación con Arduino."""
import logging

logger = logging.getLogger(__name__)


class MotorProtocol:
    """Protocolo de comandos para control de motores L206."""
    
    @staticmethod
    def format_manual_mode():
        """
        Comando para activar modo manual.
        
        Returns:
            str: Comando 'M'
        """
        return 'M'
    
    @staticmethod
    def format_auto_mode():
        """
        Comando para activar modo automático.
        
        Returns:
            str: Comando 'A'
        """
        return 'A'
    
    @staticmethod
    def format_power_command(motor_a_power, motor_b_power):
        """
        Formatea comando de potencia para ambos motores.
        
        Args:
            motor_a_power (int): Potencia motor A (-255 a 255)
            motor_b_power (int): Potencia motor B (-255 a 255)
            
        Returns:
            str: Comando formateado 'A,<pwm_a>,<pwm_b>'
        """
        return f'A,{motor_a_power},{motor_b_power}'
    
    @staticmethod
    def parse_sensor_data(line):
        """
        Parsea línea de datos del Arduino.
        
        Formato esperado: "pot_a,pot_b,sens_1,sens_2"
        
        Args:
            line (str): Línea recibida del serial
            
        Returns:
            tuple: (pot_a, pot_b, sens_1, sens_2) o None si error
        """
        try:
            parts = line.split(',')
            if len(parts) == 4:
                return tuple(map(int, parts))
        except (ValueError, IndexError):
            logger.debug(f"Error parseando datos: {line}")
            return None
        return None
    
    @staticmethod
    def is_info_message(line):
        """
        Verifica si la línea es un mensaje informativo.
        
        Args:
            line (str): Línea recibida
            
        Returns:
            bool: True si es INFO o ERROR
        """
        return line.startswith("INFO:") or line.startswith("ERROR:")
    
    # --- NUEVOS COMANDOS PARA POSITION HOLD Y SETTLING ---
    
    @staticmethod
    def format_position_hold(sensor1_target, sensor2_target):
        """
        Formatea comando de position hold con target de sensores.
        
        Args:
            sensor1_target (int): Target del sensor 1 (valor ADC)
            sensor2_target (int): Target del sensor 2 (valor ADC)
            
        Returns:
            str: Comando formateado 'H,<s1>,<s2>'
        """
        return f'H,{sensor1_target},{sensor2_target}'
    
    @staticmethod
    def format_brake_command():
        """
        Formatea comando de freno activo.
        
        Returns:
            str: Comando 'B'
        """
        return 'B'
    
    @staticmethod
    def format_settling_config(threshold):
        """
        Formatea comando de configuración de asentamiento.
        
        Args:
            threshold (int): Umbral de asentamiento (unidades ADC)
            
        Returns:
            str: Comando formateado 'S,<threshold>'
        """
        return f'S,{threshold}'
    
    @staticmethod
    def parse_sensor_data_with_status(line):
        """
        Parsea línea de datos del Arduino con información de estado.
        
        Formato esperado: "pot_a,pot_b,sens_1,sens_2,estado,settled"
        
        Args:
            line (str): Línea recibida del serial
            
        Returns:
            dict: Datos parseados con claves:
                - pot_a, pot_b: Potencia de motores
                - sens_1, sens_2: Valores de sensores
                - state: Estado del control (MANUAL, AUTO, HOLD, etc.)
                - settled: bool indicando si posición está asentada
            None: Si hay error en parsing
        """
        try:
            parts = line.split(',')
            if len(parts) >= 6:
                return {
                    'pot_a': int(parts[0]),
                    'pot_b': int(parts[1]),
                    'sens_1': int(parts[2]),
                    'sens_2': int(parts[3]),
                    'state': parts[4].strip(),
                    'settled': parts[5].strip() == '1'
                }
        except (ValueError, IndexError) as e:
            logger.debug(f"Error parseando datos con estado: {line} - {e}")
            return None
        return None

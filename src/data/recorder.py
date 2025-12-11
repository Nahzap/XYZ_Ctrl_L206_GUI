"""
Grabación de datos experimentales.

Este módulo maneja la grabación de datos en archivos CSV para análisis posterior.
"""

import csv
import time
import logging

logger = logging.getLogger(__name__)


class DataRecorder:
    """Maneja la grabación de datos experimentales en CSV."""
    
    def __init__(self):
        """Inicializa el grabador de datos."""
        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None
        self.start_time = None
        logger.debug("DataRecorder inicializado")
    
    def start_recording(self, filename):
        """
        Inicia la grabación en archivo CSV.
        
        Args:
            filename: Nombre del archivo CSV
            
        Returns:
            tuple: (success: bool, message: str)
        """
        logger.info(f"Iniciando grabación en: {filename}")
        
        # Asegurar extensión .csv
        if not filename.endswith('.csv'):
            filename += '.csv'
            logger.debug(f"Extensión .csv agregada: {filename}")
        
        try:
            self.csv_file = open(filename, 'w', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["Timestamp_ms", "PotenciaA", "PotenciaB", "Sensor1", "Sensor2"])
            logger.info(f"Archivo CSV creado exitosamente: {filename}")
            
            self.start_time = time.time()
            self.is_recording = True
            logger.info(f"Grabación iniciada en t={self.start_time}")
            
            return (True, f"Grabando en: {filename}")
            
        except (IOError, PermissionError) as e:
            logger.error(f"Error al crear archivo CSV: {e}")
            return (False, f"Error al abrir archivo: {e}")
        except Exception as e:
            logger.critical(f"Error inesperado en start_recording: {e}")
            return (False, f"Error: {e}")
    
    def stop_recording(self):
        """
        Detiene la grabación de datos.
        
        Returns:
            str: Mensaje de estado
        """
        logger.info("Deteniendo grabación")
        self.is_recording = False
        
        if self.csv_file:
            self.csv_file.close()
            logger.info("Archivo CSV cerrado correctamente")
            self.csv_file = None
            self.csv_writer = None
            return "Estado: Detenido"
        else:
            logger.warning("stop_recording llamado pero no había archivo abierto")
            return "Estado: No había grabación activa"
    
    def write_data_point(self, pot_a, pot_b, sens_1, sens_2):
        """
        Escribe un punto de datos al archivo CSV.
        
        Args:
            pot_a: Potencia motor A
            pot_b: Potencia motor B
            sens_1: Valor sensor 1
            sens_2: Valor sensor 2
        """
        if self.is_recording and self.csv_writer and self.start_time:
            try:
                current_time_ms = int((time.time() - self.start_time) * 1000)
                self.csv_writer.writerow([current_time_ms, pot_a, pot_b, sens_1, sens_2])
            except Exception as e:
                logger.error(f"Error al escribir datos: {e}")
    
    def __del__(self):
        """Destructor - asegura que el archivo se cierre."""
        if self.csv_file:
            try:
                self.csv_file.close()
                logger.debug("Archivo CSV cerrado en destructor")
            except:
                pass

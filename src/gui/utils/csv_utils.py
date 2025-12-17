"""
Utilidades para importación/exportación de trayectorias CSV.

Módulo separado para reducir el tamaño de TestTab y centralizar
la lógica de manejo de archivos CSV de trayectorias.
"""

import csv
import logging
from typing import Tuple, Optional, List
import numpy as np

logger = logging.getLogger('MotorControl_L206')


def export_trajectory_csv(trajectory: np.ndarray, filename: str) -> Tuple[bool, str]:
    """
    Exporta una trayectoria a un archivo CSV.
    
    Args:
        trajectory: Array numpy con puntos (N, 2) en µm
        filename: Ruta del archivo a guardar
        
    Returns:
        Tuple (success: bool, message: str)
    """
    if trajectory is None or len(trajectory) == 0:
        return False, "No hay trayectoria para exportar"
    
    try:
        with open(filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            # Header
            writer.writerow(['Punto', 'X_um', 'Y_um'])
            # Data
            for i, point in enumerate(trajectory):
                writer.writerow([i+1, f"{point[0]:.2f}", f"{point[1]:.2f}"])
        
        logger.info(f"Trayectoria exportada a {filename}: {len(trajectory)} puntos")
        return True, f"Trayectoria exportada: {len(trajectory)} puntos guardados"
        
    except Exception as e:
        logger.error(f"Error exportando trayectoria: {e}")
        return False, f"Error exportando: {e}"


def import_trajectory_csv(filename: str) -> Tuple[bool, str, Optional[np.ndarray]]:
    """
    Importa una trayectoria desde un archivo CSV.
    
    Soporta dos formatos:
    - Formato completo: Punto, X_um, Y_um
    - Formato simple: X, Y
    
    Args:
        filename: Ruta del archivo a cargar
        
    Returns:
        Tuple (success: bool, message: str, trajectory: Optional[np.ndarray])
    """
    try:
        points = []
        with open(filename, 'r', encoding='utf-8') as f:
            reader = csv.reader(f)
            header = next(reader)  # Skip header
            
            for row in reader:
                if len(row) >= 3:
                    # Formato: Punto, X_um, Y_um
                    x = float(row[1])
                    y = float(row[2])
                    points.append([x, y])
                elif len(row) >= 2:
                    # Formato simple: X, Y
                    x = float(row[0])
                    y = float(row[1])
                    points.append([x, y])
        
        if len(points) == 0:
            return False, "El archivo CSV no contiene puntos válidos", None
        
        trajectory = np.array(points)
        
        # Calcular estadísticas
        x_min, x_max = trajectory[:, 0].min(), trajectory[:, 0].max()
        y_min, y_max = trajectory[:, 1].min(), trajectory[:, 1].max()
        
        message = f"Trayectoria importada: {len(points)} puntos\n"
        message += f"Rango X: [{x_min:.0f}, {x_max:.0f}] µm\n"
        message += f"Rango Y: [{y_min:.0f}, {y_max:.0f}] µm"
        
        logger.info(f"Trayectoria importada desde {filename}: {len(points)} puntos")
        return True, message, trajectory
        
    except Exception as e:
        logger.error(f"Error importando trayectoria: {e}")
        return False, f"Error importando: {e}", None


def get_trajectory_stats(trajectory: np.ndarray) -> dict:
    """
    Calcula estadísticas de una trayectoria.
    
    Args:
        trajectory: Array numpy con puntos (N, 2) en µm
        
    Returns:
        Dict con estadísticas: n_points, x_min, x_max, y_min, y_max, total_distance
    """
    if trajectory is None or len(trajectory) == 0:
        return {
            'n_points': 0,
            'x_min': 0, 'x_max': 0,
            'y_min': 0, 'y_max': 0,
            'total_distance': 0
        }
    
    x_coords = trajectory[:, 0]
    y_coords = trajectory[:, 1]
    
    # Calcular distancia total
    total_distance = 0.0
    for i in range(1, len(trajectory)):
        dx = trajectory[i, 0] - trajectory[i-1, 0]
        dy = trajectory[i, 1] - trajectory[i-1, 1]
        total_distance += np.sqrt(dx**2 + dy**2)
    
    return {
        'n_points': len(trajectory),
        'x_min': float(x_coords.min()),
        'x_max': float(x_coords.max()),
        'y_min': float(y_coords.min()),
        'y_max': float(y_coords.max()),
        'total_distance': float(total_distance)
    }

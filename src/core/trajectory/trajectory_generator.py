"""
Generador de trayectorias para control de motores.

Este módulo genera trayectorias zig-zag y otros patrones para
pruebas de controladores.
"""

import logging
import numpy as np
import pandas as pd
from matplotlib.figure import Figure

logger = logging.getLogger(__name__)


class TrajectoryGenerator:
    """Generador de trayectorias para pruebas de control."""
    
    def __init__(self):
        """Inicializa el generador de trayectorias."""
        self.current_trajectory = None
        logger.debug("TrajectoryGenerator inicializado")
    
    def generate_zigzag_by_points(self, n_points, x_min, x_max, y_min, y_max, 
                                 step_delay, calibration=None):
        """
        Genera una trayectoria en zig-zag con número específico de puntos.
        
        Args:
            n_points: Número total de puntos
            x_min, x_max: Límites en X (µm)
            y_min, y_max: Límites en Y (µm)
            step_delay: Tiempo entre pasos (s)
            calibration: Dict con calibración (opcional)
            
        Returns:
            dict: Trayectoria generada con keys:
                - success: bool
                - message: str
                - points: np.ndarray de (x, y)
                - figure: matplotlib Figure
                - n_rows, n_cols: int
        """
        logger.info("=== Generando trayectoria por número de puntos ===")
        logger.debug(f"Puntos: {n_points}, X: [{x_min}, {x_max}], Y: [{y_min}, {y_max}]")
        
        try:
            # Validaciones
            if n_points < 1 or n_points > 10000:
                return {
                    'success': False,
                    'message': "Número de puntos debe estar entre 1 y 10000"
                }
            
            if step_delay < 0.1:
                return {
                    'success': False,
                    'message': "Tiempo entre pasos debe ser al menos 0.1s"
                }
            
            # Calcular número de filas para zig-zag
            n_rows = int(np.sqrt(n_points))
            n_cols = int(np.ceil(n_points / n_rows))
            
            # Generar grid homogéneo
            x_positions = np.linspace(x_min, x_max, n_cols)
            y_positions = np.linspace(y_min, y_max, n_rows)
            
            # Generar trayectoria en zig-zag
            trajectory = []
            for i, y in enumerate(y_positions):
                if i % 2 == 0:
                    # Fila par: inicio a fin
                    for x in x_positions:
                        trajectory.append([x, y])
                else:
                    # Fila impar: fin a inicio (zig-zag)
                    for x in reversed(x_positions):
                        trajectory.append([x, y])
            
            # Limitar al número de puntos solicitado
            trajectory = trajectory[:n_points]
            trajectory_array = np.array(trajectory)
            
            logger.info(f"✅ Trayectoria generada: {len(trajectory)} puntos ({n_rows}x{n_cols})")
            
            # Crear figura de preview
            figure = self._create_trajectory_plot_array(trajectory_array, x_min, x_max, y_min, y_max)
            
            # Guardar trayectoria actual
            self.current_trajectory = trajectory_array
            
            return {
                'success': True,
                'message': f'Trayectoria generada: {len(trajectory)} puntos',
                'points': trajectory_array,
                'figure': figure,
                'n_points': len(trajectory),
                'n_rows': n_rows,
                'n_cols': n_cols,
                'step_delay': step_delay,
                'x_mid': (x_min + x_max) / 2.0,
                'y_mid': (y_min + y_max) / 2.0
            }
            
        except Exception as e:
            error_msg = f"Error al generar trayectoria: {str(e)}"
            logger.error(error_msg)
            return {
                'success': False,
                'message': error_msg
            }
    
    def generate_zigzag(self, x_min, x_max, y_min, y_max, step_x, step_y, 
                       velocity, calibration=None):
        """
        Genera una trayectoria en zig-zag por pasos.
        
        Args:
            x_min, x_max: Límites en X (mm)
            y_min, y_max: Límites en Y (mm)
            step_x: Paso en X (mm)
            step_y: Paso en Y (mm)
            velocity: Velocidad de desplazamiento (mm/s)
            calibration: Dict con calibración (opcional)
            
        Returns:
            dict: Trayectoria generada con keys:
                - success: bool
                - message: str
                - points: list of dicts con x, y, adc_x, adc_y
                - figure: matplotlib Figure
        """
        logger.info("=== Generando trayectoria zig-zag ===")
        logger.debug(f"Límites X: [{x_min}, {x_max}], Y: [{y_min}, {y_max}]")
        logger.debug(f"Pasos: ΔX={step_x}mm, ΔY={step_y}mm")
        logger.debug(f"Velocidad: {velocity}mm/s")
        
        try:
            # Validaciones
            if x_max <= x_min or y_max <= y_min:
                return {
                    'success': False,
                    'message': "Error: Los límites máximos deben ser mayores que los mínimos"
                }
            
            if step_x <= 0 or step_y <= 0:
                return {
                    'success': False,
                    'message': "Error: Los pasos deben ser positivos"
                }
            
            if velocity <= 0:
                return {
                    'success': False,
                    'message': "Error: La velocidad debe ser positiva"
                }
            
            # Generar puntos de la trayectoria
            points = []
            current_y = y_min
            direction = 1  # 1 = derecha, -1 = izquierda
            
            while current_y <= y_max:
                if direction == 1:
                    # Barrido de izquierda a derecha
                    x_values = np.arange(x_min, x_max + step_x/2, step_x)
                else:
                    # Barrido de derecha a izquierda
                    x_values = np.arange(x_max, x_min - step_x/2, -step_x)
                
                for x in x_values:
                    point = {'x': float(x), 'y': float(current_y)}
                    
                    # Convertir a ADC si hay calibración
                    if calibration:
                        point['adc_x'] = self._mm_to_adc(x, calibration, 'x')
                        point['adc_y'] = self._mm_to_adc(current_y, calibration, 'y')
                    
                    points.append(point)
                
                # Avanzar en Y
                current_y += step_y
                # Cambiar dirección
                direction *= -1
            
            logger.info(f"✅ Trayectoria generada: {len(points)} puntos")
            
            # Crear figura de preview
            figure = self._create_trajectory_plot(points, x_min, x_max, y_min, y_max)
            
            # Guardar trayectoria actual
            self.current_trajectory = points
            
            return {
                'success': True,
                'message': f'Trayectoria generada: {len(points)} puntos',
                'points': points,
                'figure': figure,
                'n_points': len(points)
            }
            
        except Exception as e:
            error_msg = f"Error al generar trayectoria: {str(e)}"
            logger.error(error_msg)
            return {
                'success': False,
                'message': error_msg
            }
    
    def _mm_to_adc(self, value_mm, calibration, axis):
        """Convierte mm a ADC usando calibración."""
        if not calibration or axis not in calibration:
            return 0
        
        cal = calibration[axis]
        pendiente = cal.get('pendiente_mm', 1.0)
        intercepto = cal.get('intercepto_mm', 0.0)
        
        # Invertir la fórmula: ADC = (mm - intercepto) / pendiente
        adc = (value_mm - intercepto) / pendiente if pendiente != 0 else 0
        return int(adc)
    
    def _create_trajectory_plot_array(self, trajectory_array, x_min, x_max, y_min, y_max):
        """Crea gráfico de visualización de la trayectoria desde numpy array."""
        fig = Figure(figsize=(10, 8), facecolor='#2E2E2E')
        ax = fig.add_subplot(111)
        
        # Extraer coordenadas
        x_coords = trajectory_array[:, 0]
        y_coords = trajectory_array[:, 1]
        
        # Graficar trayectoria
        ax.plot(x_coords, y_coords, 'o-', color='cyan', linewidth=2, 
                markersize=4, label='Trayectoria')
        
        # Marcar inicio y fin
        ax.plot(x_coords[0], y_coords[0], 'go', markersize=12, label='Inicio')
        ax.plot(x_coords[-1], y_coords[-1], 'ro', markersize=12, label='Fin')
        
        # Configurar ejes
        ax.set_xlabel('X (µm)', color='white', fontsize=12)
        ax.set_ylabel('Y (µm)', color='white', fontsize=12)
        ax.set_title(f'Trayectoria Zig-Zag ({len(trajectory_array)} puntos)', 
                    fontsize=14, fontweight='bold', color='white')
        
        # Límites
        margin_x = (x_max - x_min) * 0.1 if x_max > x_min else 1
        margin_y = (y_max - y_min) * 0.1 if y_max > y_min else 1
        ax.set_xlim(x_min - margin_x, x_max + margin_x)
        ax.set_ylim(y_min - margin_y, y_max + margin_y)
        
        # Grid
        ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
        ax.set_aspect('equal', 'box')
        
        # Estilo oscuro
        ax.set_facecolor('#252525')
        ax.tick_params(colors='white')
        ax.legend(facecolor='#383838', edgecolor='#505050', labelcolor='white')
        
        for spine in ax.spines.values():
            spine.set_color('#505050')
        
        fig.tight_layout()
        return fig
    
    def _create_trajectory_plot(self, points, x_min, x_max, y_min, y_max):
        """Crea gráfico de visualización de la trayectoria."""
        fig = Figure(figsize=(10, 8), facecolor='#2E2E2E')
        ax = fig.add_subplot(111)
        
        # Extraer coordenadas
        x_coords = [p['x'] for p in points]
        y_coords = [p['y'] for p in points]
        
        # Graficar trayectoria
        ax.plot(x_coords, y_coords, 'o-', color='cyan', linewidth=2, 
                markersize=4, label='Trayectoria')
        
        # Marcar inicio y fin
        ax.plot(x_coords[0], y_coords[0], 'go', markersize=12, label='Inicio')
        ax.plot(x_coords[-1], y_coords[-1], 'ro', markersize=12, label='Fin')
        
        # Configurar ejes
        ax.set_xlabel('X (mm)', color='white', fontsize=12)
        ax.set_ylabel('Y (mm)', color='white', fontsize=12)
        ax.set_title(f'Trayectoria Zig-Zag ({len(points)} puntos)', 
                    fontsize=14, fontweight='bold', color='white')
        
        # Límites
        margin_x = (x_max - x_min) * 0.1
        margin_y = (y_max - y_min) * 0.1
        ax.set_xlim(x_min - margin_x, x_max + margin_x)
        ax.set_ylim(y_min - margin_y, y_max + margin_y)
        
        # Grid
        ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
        ax.set_aspect('equal', 'box')
        
        # Estilo oscuro
        ax.set_facecolor('#252525')
        ax.tick_params(colors='white')
        ax.legend(facecolor='#383838', edgecolor='#505050', labelcolor='white')
        
        for spine in ax.spines.values():
            spine.set_color('#505050')
        
        fig.tight_layout()
        return fig
    
    def export_to_csv(self, filename, points=None):
        """
        Exporta la trayectoria a archivo CSV.
        
        Args:
            filename: Nombre del archivo
            points: Lista de puntos (usa current_trajectory si None)
            
        Returns:
            dict: Resultado de la exportación
        """
        if points is None:
            points = self.current_trajectory
        
        if not points:
            return {
                'success': False,
                'message': 'No hay trayectoria para exportar'
            }
        
        try:
            # Crear DataFrame
            df = pd.DataFrame(points)
            
            # Guardar a CSV
            df.to_csv(filename, index=False)
            
            logger.info(f"✅ Trayectoria exportada: {filename} ({len(points)} puntos)")
            
            return {
                'success': True,
                'message': f'Exportado a {filename}',
                'n_points': len(points)
            }
            
        except Exception as e:
            error_msg = f"Error al exportar: {str(e)}"
            logger.error(error_msg)
            return {
                'success': False,
                'message': error_msg
            }
    
    def get_trajectory(self):
        """Obtiene la trayectoria actual."""
        return self.current_trajectory
    
    def clear_trajectory(self):
        """Limpia la trayectoria actual."""
        self.current_trajectory = None
        logger.info("Trayectoria limpiada")

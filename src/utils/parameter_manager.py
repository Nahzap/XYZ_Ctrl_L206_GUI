"""
Gestor de parámetros de test para autocompletado.

Este módulo maneja la persistencia y autocompletado de parámetros de test
para mantener homogeneidad entre ejecuciones.
"""

import json
import logging
from pathlib import Path
from typing import Dict, Any, Optional
from datetime import datetime

from config.constants import DEFAULT_FOV_X_UM, DEFAULT_FOV_Y_UM

logger = logging.getLogger(__name__)


class ParameterManager:
    """Gestor de parámetros de test con autocompletado."""
    
    def __init__(self, config_path: Optional[str] = None):
        """
        Inicializa el gestor de parámetros.
        
        Args:
            config_path: Ruta al archivo JSON de configuración.
                        Si es None, usa la ruta por defecto.
        """
        if config_path is None:
            base_dir = Path(__file__).parent.parent
            config_path = base_dir / "config" / "test_parameters_template.json"
        
        self.config_path = Path(config_path)
        self.parameters: Dict[str, Any] = {}
        
        if self.config_path.exists():
            self.load()
        else:
            logger.warning(f"Archivo de parámetros no encontrado: {self.config_path}")
            self._create_default_config()
    
    def load(self) -> Dict[str, Any]:
        """
        Carga los parámetros desde el archivo JSON.
        
        Returns:
            Diccionario con los parámetros cargados.
        """
        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                self.parameters = json.load(f)
            logger.info(f"✅ Parámetros cargados desde {self.config_path}")
            return self.parameters
        except Exception as e:
            logger.error(f"❌ Error cargando parámetros: {e}")
            self._create_default_config()
            return self.parameters
    
    def save(self) -> bool:
        """
        Guarda los parámetros actuales en el archivo JSON.
        
        Returns:
            True si se guardó exitosamente, False en caso contrario.
        """
        try:
            # Actualizar timestamp
            self.parameters['last_updated'] = datetime.now().isoformat()
            
            # Guardar con formato legible
            with open(self.config_path, 'w', encoding='utf-8') as f:
                json.dump(self.parameters, f, indent=2, ensure_ascii=False)
            
            logger.info(f"✅ Parámetros guardados en {self.config_path}")
            return True
        except Exception as e:
            logger.error(f"❌ Error guardando parámetros: {e}")
            return False
    
    def update_trajectory(self, points: int, x_min: float, x_max: float, 
                         y_min: float, y_max: float, delay: float,
                         fov_x: float = DEFAULT_FOV_X_UM,
                         fov_y: float = DEFAULT_FOV_Y_UM):
        """Actualiza parámetros de trayectoria."""
        if 'trajectory' not in self.parameters:
            self.parameters['trajectory'] = {}
        
        self.parameters['trajectory'].update({
            'points': points,
            'fov': {'x': fov_x, 'y': fov_y, 'unit': 'µm'},
            'x_range': {'min': x_min, 'max': x_max, 'unit': 'µm'},
            'y_range': {'min': y_min, 'max': y_max, 'unit': 'µm'},
            'delay_between_points': delay,
            'delay_unit': 'seconds'
        })
        
        logger.info(
            f"📝 Trayectoria actualizada: {points} puntos, "
            f"FOV={fov_x}x{fov_y} µm, X=[{x_min},{x_max}], Y=[{y_min},{y_max}]"
        )
        self.save()
    
    def update_microscopy(self, class_name: str, total_points: int, 
                         autofocus_enabled: bool, af_min: int, af_max: int,
                         channels: str, format: str, bit_depth: int,
                         delay_before: int, delay_after: int):
        """Actualiza parámetros de microscopía."""
        if 'microscopy' not in self.parameters:
            self.parameters['microscopy'] = {}
        
        self.parameters['microscopy'].update({
            'class_name': class_name,
            'total_points': total_points,
            'autofocus': {
                'enabled': autofocus_enabled,
                'area_range': {'min': af_min, 'max': af_max, 'unit': 'px'}
            },
            'channels': channels,
            'format': format,
            'bit_depth': bit_depth,
            'delays': {
                'before_capture': delay_before,
                'after_capture': delay_after,
                'unit': 'ms'
            }
        })
        
        logger.info(f"📝 Microscopía actualizada: {class_name}, {total_points} puntos, AF={autofocus_enabled}")
        self.save()
    
    def update_detection(self, min_circularity: float, min_aspect_ratio: float):
        """Actualiza parámetros de detección."""
        if 'detection' not in self.parameters:
            self.parameters['detection'] = {}
        
        if 'morphological_filters' not in self.parameters['detection']:
            self.parameters['detection']['morphological_filters'] = {}
        
        self.parameters['detection']['morphological_filters'].update({
            'min_circularity': min_circularity,
            'min_aspect_ratio': min_aspect_ratio
        })
        
        logger.info(f"📝 Detección actualizada: circ≥{min_circularity}, aspect≥{min_aspect_ratio}")
        self.save()

    def update_camera_tab(self, settings: Dict[str, Any]) -> bool:
        """Guarda el formulario completo de la pestaña Cámara."""
        self.parameters['camera_tab'] = settings
        logger.debug("📝 Opciones de CameraTab actualizadas")
        return self.save()
    
    def get_trajectory_defaults(self) -> Dict[str, Any]:
        """Obtiene valores por defecto de trayectoria."""
        return self.parameters.get('trajectory', {
            'points': None,
            'fov': {'x': DEFAULT_FOV_X_UM, 'y': DEFAULT_FOV_Y_UM},
            'x_range': {'min': 10000.0, 'max': 19500.0},
            'y_range': {'min': 10000.0, 'max': 19500.0},
            'delay_between_points': 0.5
        })
    
    def get_microscopy_defaults(self) -> Dict[str, Any]:
        """Obtiene valores por defecto de microscopía."""
        return self.parameters.get('microscopy', {
            'class_name': 'Quillaja_Saponaria',
            'total_points': 1024,
            'autofocus': {'enabled': False, 'area_range': {'min': 5000, 'max': 120000}},
            'channels': 'G',
            'format': 'PNG',
            'bit_depth': 16,
            'delays': {'before_capture': 700, 'after_capture': 100}
        })
    
    def get_detection_defaults(self) -> Dict[str, Any]:
        """Obtiene valores por defecto de detección."""
        return self.parameters.get('detection', {
            'morphological_filters': {
                'min_circularity': 0.25,
                'min_aspect_ratio': 0.25
            }
        })

    def get_camera_tab_defaults(self) -> Dict[str, Any]:
        """Obtiene el último formulario completo guardado de CameraTab."""
        return self.parameters.get('camera_tab', {})
    
    def _create_default_config(self):
        """Crea configuración por defecto."""
        self.parameters = {
            "$schema": "http://json-schema.org/draft-07/schema#",
            "title": "Test Parameters Auto-Fill Template",
            "description": "Formulario de autocompletado para mantener tests homogéneos",
            "version": "1.0.0",
            "last_updated": datetime.now().isoformat(),
            
            "trajectory": {
                "points": None,
                "fov": {"x": DEFAULT_FOV_X_UM, "y": DEFAULT_FOV_Y_UM, "unit": "µm"},
                "x_range": {"min": 10000.0, "max": 19500.0, "unit": "µm"},
                "y_range": {"min": 10000.0, "max": 19500.0, "unit": "µm"},
                "delay_between_points": 0.5,
                "delay_unit": "seconds"
            },
            
            "microscopy": {
                "class_name": "Quillaja_Saponaria",
                "total_points": 1024,
                "autofocus": {
                    "enabled": False,
                    "area_range": {"min": 5000, "max": 120000, "unit": "px"}
                },
                "channels": "G",
                "format": "PNG",
                "bit_depth": 16,
                "delays": {
                    "before_capture": 700,
                    "after_capture": 100,
                    "unit": "ms"
                }
            },
            
            "detection": {
                "morphological_filters": {
                    "min_circularity": 0.25,
                    "min_aspect_ratio": 0.25
                }
            },

            "camera_tab": {
                "description": "Últimas opciones editables de la pestaña Cámara"
            }
        }
        
        self.save()
        logger.info("✅ Configuración por defecto creada")


# Singleton global
_parameter_manager: Optional[ParameterManager] = None


def get_parameter_manager() -> ParameterManager:
    """
    Obtiene la instancia singleton del ParameterManager.
    
    Returns:
        Instancia única de ParameterManager.
    """
    global _parameter_manager
    if _parameter_manager is None:
        _parameter_manager = ParameterManager()
    return _parameter_manager

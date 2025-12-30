"""
Validador de Configuración de Microscopía
==========================================

Centraliza todas las validaciones de configuración de microscopía
para evitar duplicación y mejorar mantenibilidad.

Autor: Sistema de Control L206
Fecha: 2025-12-29
"""

import logging
from dataclasses import dataclass
from typing import List, Optional, Tuple, Any

logger = logging.getLogger('MotorControl_L206')


@dataclass
class MicroscopyConfig:
    """
    Configuración completa para microscopía automatizada.
    
    Attributes:
        trajectory: Lista de puntos (x, y) en µm
        autofocus_enabled: Si debe ejecutar autofoco en cada punto
        xy_only: Si es solo XY (sin Z)
        delay_before: Delay antes de captura (s)
        delay_after: Delay después de captura (s)
        save_folder: Carpeta para guardar imágenes
        class_name: Nombre de clase para etiquetado
        img_width: Ancho de imagen (px)
        img_height: Alto de imagen (px)
        channels: Canales RGB a guardar
        learning_mode: Modo de aprendizaje asistido
        learning_target: Número de imágenes para aprendizaje
        cfocus_available: Si C-Focus está disponible
        camera_connected: Si cámara está conectada
    """
    trajectory: List[Tuple[float, float]]
    autofocus_enabled: bool = False
    xy_only: bool = True
    delay_before: float = 2.0
    delay_after: float = 0.2
    save_folder: str = ""
    class_name: str = "sample"
    img_width: int = 1920
    img_height: int = 1200
    channels: List[str] = None
    learning_mode: bool = True
    learning_target: int = 50
    cfocus_available: bool = False
    camera_connected: bool = False
    
    def __post_init__(self):
        if self.channels is None:
            self.channels = ['R', 'G', 'B']


@dataclass
class ValidationResult:
    """
    Resultado de validación.
    
    Attributes:
        is_valid: Si la configuración es válida
        errors: Lista de errores críticos
        warnings: Lista de advertencias no críticas
    """
    is_valid: bool
    errors: List[str]
    warnings: List[str]
    
    def __init__(self, is_valid: bool = True, errors: List[str] = None, warnings: List[str] = None):
        self.is_valid = is_valid
        self.errors = errors or []
        self.warnings = warnings or []


class MicroscopyValidator:
    """
    Validador centralizado para configuración de microscopía.
    
    Valida:
    - Trayectoria no vacía
    - Hardware disponible (cámara, C-Focus)
    - Parámetros de captura válidos
    - Carpeta de guardado válida
    - Delays razonables
    """
    
    def __init__(self):
        """Inicializa el validador."""
        pass
    
    def validate(self, config: MicroscopyConfig) -> ValidationResult:
        """
        Valida configuración completa de microscopía.
        
        Args:
            config: Configuración a validar
        
        Returns:
            ValidationResult con errores y advertencias
        """
        errors = []
        warnings = []
        
        # Validar trayectoria
        traj_errors, traj_warnings = self._validate_trajectory(config.trajectory)
        errors.extend(traj_errors)
        warnings.extend(traj_warnings)
        
        # Validar hardware
        hw_errors, hw_warnings = self._validate_hardware(config)
        errors.extend(hw_errors)
        warnings.extend(hw_warnings)
        
        # Validar parámetros de captura
        cap_errors, cap_warnings = self._validate_capture_params(config)
        errors.extend(cap_errors)
        warnings.extend(cap_warnings)
        
        # Validar carpeta de guardado
        folder_errors, folder_warnings = self._validate_save_folder(config.save_folder)
        errors.extend(folder_errors)
        warnings.extend(folder_warnings)
        
        # Validar delays
        delay_errors, delay_warnings = self._validate_delays(config.delay_before, config.delay_after)
        errors.extend(delay_errors)
        warnings.extend(delay_warnings)
        
        is_valid = len(errors) == 0
        return ValidationResult(is_valid=is_valid, errors=errors, warnings=warnings)
    
    def _validate_trajectory(self, trajectory: List[Tuple[float, float]]) -> Tuple[List[str], List[str]]:
        """Valida la trayectoria."""
        errors = []
        warnings = []
        
        if not trajectory or len(trajectory) == 0:
            errors.append("Trayectoria vacía - genera una trayectoria en TestTab primero")
            return errors, warnings
        
        if len(trajectory) < 2:
            warnings.append(f"Trayectoria muy corta ({len(trajectory)} punto)")
        
        if len(trajectory) > 1000:
            warnings.append(f"Trayectoria muy larga ({len(trajectory)} puntos) - puede tomar mucho tiempo")
        
        # Validar que los puntos sean válidos
        for i, point in enumerate(trajectory):
            if not isinstance(point, (tuple, list)) or len(point) != 2:
                errors.append(f"Punto {i} inválido: debe ser (x, y)")
                continue
            
            x, y = point
            if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
                errors.append(f"Punto {i} inválido: coordenadas deben ser numéricas")
        
        return errors, warnings
    
    def _validate_hardware(self, config: MicroscopyConfig) -> Tuple[List[str], List[str]]:
        """Valida disponibilidad de hardware."""
        errors = []
        warnings = []
        
        # Validar cámara
        if not config.camera_connected:
            errors.append("Cámara no conectada - conecta la cámara primero")
        
        # Validar C-Focus si autofoco está habilitado
        if config.autofocus_enabled and not config.cfocus_available:
            errors.append("Autofoco requiere C-Focus conectado y calibrado")
        
        if not config.xy_only and not config.cfocus_available:
            warnings.append("Modo 3D (con Z) requiere C-Focus - se usará solo XY")
        
        return errors, warnings
    
    def _validate_capture_params(self, config: MicroscopyConfig) -> Tuple[List[str], List[str]]:
        """Valida parámetros de captura."""
        errors = []
        warnings = []
        
        # Validar dimensiones de imagen
        if config.img_width <= 0 or config.img_height <= 0:
            errors.append(f"Dimensiones de imagen inválidas: {config.img_width}x{config.img_height}")
        
        if config.img_width > 10000 or config.img_height > 10000:
            warnings.append(f"Dimensiones muy grandes: {config.img_width}x{config.img_height} - puede consumir mucha memoria")
        
        # Validar canales
        if not config.channels or len(config.channels) == 0:
            errors.append("Debe seleccionar al menos un canal (R, G, B)")
        
        valid_channels = {'R', 'G', 'B'}
        for ch in config.channels:
            if ch not in valid_channels:
                errors.append(f"Canal inválido: {ch} - debe ser R, G o B")
        
        # Validar nombre de clase
        if not config.class_name or config.class_name.strip() == "":
            warnings.append("Nombre de clase vacío - se usará 'sample'")
        
        # Validar modo de aprendizaje
        if config.learning_mode:
            if config.learning_target <= 0:
                errors.append(f"Learning target inválido: {config.learning_target} - debe ser > 0")
            elif config.learning_target > len(config.trajectory):
                warnings.append(f"Learning target ({config.learning_target}) mayor que trayectoria ({len(config.trajectory)})")
        
        return errors, warnings
    
    def _validate_save_folder(self, save_folder: str) -> Tuple[List[str], List[str]]:
        """Valida carpeta de guardado."""
        errors = []
        warnings = []
        
        if not save_folder or save_folder.strip() == "":
            errors.append("Carpeta de guardado no especificada")
            return errors, warnings
        
        import os
        if not os.path.exists(save_folder):
            try:
                os.makedirs(save_folder, exist_ok=True)
                warnings.append(f"Carpeta creada: {save_folder}")
            except Exception as e:
                errors.append(f"No se puede crear carpeta: {e}")
        
        if not os.path.isdir(save_folder):
            errors.append(f"Ruta no es una carpeta: {save_folder}")
        
        # Verificar permisos de escritura
        if os.path.exists(save_folder):
            if not os.access(save_folder, os.W_OK):
                errors.append(f"Sin permisos de escritura en: {save_folder}")
        
        return errors, warnings
    
    def _validate_delays(self, delay_before: float, delay_after: float) -> Tuple[List[str], List[str]]:
        """Valida delays de captura."""
        errors = []
        warnings = []
        
        if delay_before < 0:
            errors.append(f"Delay antes inválido: {delay_before}s - debe ser >= 0")
        
        if delay_after < 0:
            errors.append(f"Delay después inválido: {delay_after}s - debe ser >= 0")
        
        if delay_before > 10:
            warnings.append(f"Delay antes muy largo: {delay_before}s - puede hacer lenta la captura")
        
        if delay_after > 5:
            warnings.append(f"Delay después muy largo: {delay_after}s")
        
        if delay_before < 0.5:
            warnings.append(f"Delay antes muy corto: {delay_before}s - puede no dar tiempo a estabilizar")
        
        return errors, warnings
    
    def estimate_time(self, config: MicroscopyConfig) -> dict:
        """
        Estima tiempo total de microscopía.
        
        Args:
            config: Configuración de microscopía
        
        Returns:
            dict con estimaciones de tiempo
        """
        n_points = len(config.trajectory)
        
        # Tiempo por punto
        time_per_point = config.delay_before + config.delay_after
        
        # Agregar tiempo de movimiento (estimado)
        time_per_point += 1.0  # ~1s por movimiento XY
        
        # Agregar tiempo de autofoco si está habilitado
        if config.autofocus_enabled:
            time_per_point += 5.0  # ~5s por autofoco (estimado)
        
        # Agregar tiempo de captura
        time_per_point += 0.5  # ~0.5s por captura
        
        total_time_s = n_points * time_per_point
        total_time_min = total_time_s / 60.0
        total_time_h = total_time_min / 60.0
        
        return {
            'n_points': n_points,
            'time_per_point_s': time_per_point,
            'total_time_s': total_time_s,
            'total_time_min': total_time_min,
            'total_time_h': total_time_h,
            'estimated_completion': self._format_time(total_time_s)
        }
    
    def _format_time(self, seconds: float) -> str:
        """Formatea tiempo en formato legible."""
        if seconds < 60:
            return f"{seconds:.0f}s"
        elif seconds < 3600:
            return f"{seconds/60:.1f}min"
        else:
            hours = int(seconds / 3600)
            minutes = int((seconds % 3600) / 60)
            return f"{hours}h {minutes}min"
    
    def estimate_storage(self, config: MicroscopyConfig) -> dict:
        """
        Estima espacio de almacenamiento requerido.
        
        Args:
            config: Configuración de microscopía
        
        Returns:
            dict con estimaciones de almacenamiento
        """
        n_points = len(config.trajectory)
        n_channels = len(config.channels)
        
        # Tamaño por imagen (estimado)
        # PNG 16-bit: ~2 bytes por pixel por canal
        bytes_per_pixel = 2 * n_channels
        bytes_per_image = config.img_width * config.img_height * bytes_per_pixel
        
        # Total
        total_bytes = n_points * bytes_per_image
        total_mb = total_bytes / (1024 * 1024)
        total_gb = total_mb / 1024
        
        return {
            'n_points': n_points,
            'n_channels': n_channels,
            'bytes_per_image': bytes_per_image,
            'mb_per_image': bytes_per_image / (1024 * 1024),
            'total_bytes': total_bytes,
            'total_mb': total_mb,
            'total_gb': total_gb,
            'formatted': self._format_storage(total_bytes)
        }
    
    def _format_storage(self, bytes_val: float) -> str:
        """Formatea tamaño de almacenamiento."""
        if bytes_val < 1024:
            return f"{bytes_val:.0f} B"
        elif bytes_val < 1024 * 1024:
            return f"{bytes_val/1024:.1f} KB"
        elif bytes_val < 1024 * 1024 * 1024:
            return f"{bytes_val/(1024*1024):.1f} MB"
        else:
            return f"{bytes_val/(1024*1024*1024):.2f} GB"

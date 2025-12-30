"""
Configuración de Autofoco - Dataclass
======================================

Centraliza todos los parámetros configurables del servicio de autofoco.

Autor: Sistema de Control L206
Fecha: 2025-12-29
"""

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class AutofocusConfig:
    """
    Configuración consolidada para el servicio de autofoco.
    
    Parámetros de búsqueda (para encontrar BPoF):
        z_scan_range: Rango de búsqueda desde posición actual (±µm)
        z_step_coarse: Paso grueso para fase inicial de hill climbing (µm)
        z_step_fine: Paso fino para refinamiento alrededor del pico (µm)
        settle_time: Tiempo de estabilización entre movimientos (s)
        capture_settle_time: Tiempo para captura final en BPoF (s)
        roi_margin: Margen adicional alrededor del bbox para sharpness (px)
        max_coarse_iterations: Límite de iteraciones en fase gruesa
        max_fine_iterations: Límite de iteraciones en fase fina
    
    Parámetros de captura multi-focal (para volumetría):
        n_captures: Número de capturas en Z-stack
        z_step_capture: Paso entre capturas (µm)
        z_range_capture: Rango total de captura (µm)
    """
    
    # Parámetros de búsqueda de BPoF
    z_scan_range: float = 20.0              # µm - rango de búsqueda (±20µm)
    z_step_coarse: float = 0.5              # µm - paso grueso
    z_step_fine: float = 0.1                # µm - paso fino
    settle_time: float = 0.10               # s - tiempo de estabilización
    capture_settle_time: float = 0.50       # s - tiempo para captura final
    roi_margin: int = 20                    # px - margen para sharpness
    max_coarse_iterations: int = 50         # límite fase gruesa
    max_fine_iterations: int = 100          # límite fase fina
    
    # Parámetros de captura multi-focal (Z-stack)
    n_captures: int = 5                     # número de capturas
    z_step_capture: float = 2.0             # µm - paso entre capturas
    z_range_capture: float = 10.0           # µm - rango total
    
    def validate(self) -> Tuple[bool, Optional[str]]:
        """
        Valida la configuración contra límites físicos.
        
        Returns:
            (is_valid, error_message)
        """
        errors = []
        
        # Validar rangos positivos
        if self.z_scan_range <= 0:
            errors.append("z_scan_range debe ser > 0")
        
        if self.z_step_coarse <= 0:
            errors.append("z_step_coarse debe ser > 0")
        
        if self.z_step_fine <= 0:
            errors.append("z_step_fine debe ser > 0")
        
        # Validar relación entre pasos
        if self.z_step_fine >= self.z_step_coarse:
            errors.append("z_step_fine debe ser < z_step_coarse")
        
        # Validar tiempos
        if self.settle_time < 0:
            errors.append("settle_time debe ser >= 0")
        
        if self.capture_settle_time < 0:
            errors.append("capture_settle_time debe ser >= 0")
        
        # Validar iteraciones
        if self.max_coarse_iterations <= 0:
            errors.append("max_coarse_iterations debe ser > 0")
        
        if self.max_fine_iterations <= 0:
            errors.append("max_fine_iterations debe ser > 0")
        
        # Validar captura multi-focal
        if self.n_captures < 1:
            errors.append("n_captures debe ser >= 1")
        
        if self.z_step_capture <= 0:
            errors.append("z_step_capture debe ser > 0")
        
        if self.z_range_capture <= 0:
            errors.append("z_range_capture debe ser > 0")
        
        if errors:
            return False, "; ".join(errors)
        
        return True, None
    
    def get_search_info(self) -> dict:
        """
        Retorna información estimada de la búsqueda.
        
        Returns:
            dict con estimaciones de tiempo e iteraciones
        """
        # Estimación de pasos en fase gruesa (ida y vuelta)
        coarse_steps = int(2 * self.z_scan_range / self.z_step_coarse)
        coarse_steps = min(coarse_steps, self.max_coarse_iterations)
        
        # Estimación de pasos en fase fina (refinamiento local)
        fine_range = 3 * self.z_step_coarse  # Refinar ±3 pasos gruesos
        fine_steps = int(2 * fine_range / self.z_step_fine)
        fine_steps = min(fine_steps, self.max_fine_iterations)
        
        total_steps = coarse_steps + fine_steps
        
        # Tiempo estimado
        time_per_step = self.settle_time + 0.05  # 50ms para captura/procesamiento
        estimated_time = total_steps * time_per_step + self.capture_settle_time
        
        return {
            'coarse_steps': coarse_steps,
            'fine_steps': fine_steps,
            'total_steps': total_steps,
            'estimated_time_s': estimated_time,
            'z_range_um': 2 * self.z_scan_range
        }
    
    def validate_against_cfocus_limits(self, z_min: float, z_max: float, 
                                       current_z: float) -> Tuple[bool, Optional[str]]:
        """
        Valida que el rango de búsqueda no exceda los límites del C-Focus.
        
        Args:
            z_min: Límite inferior del C-Focus (µm)
            z_max: Límite superior del C-Focus (µm)
            current_z: Posición Z actual (µm)
        
        Returns:
            (is_valid, error_message)
        """
        search_min = current_z - self.z_scan_range
        search_max = current_z + self.z_scan_range
        
        if search_min < z_min:
            return False, f"Rango de búsqueda excede límite inferior ({search_min:.1f} < {z_min:.1f} µm)"
        
        if search_max > z_max:
            return False, f"Rango de búsqueda excede límite superior ({search_max:.1f} > {z_max:.1f} µm)"
        
        return True, None

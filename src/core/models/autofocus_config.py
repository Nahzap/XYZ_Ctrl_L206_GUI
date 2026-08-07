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
        z_scan_range: Semirango máximo FINE alrededor de Z_coarse* (±µm)
        z_step_coarse: Paso grueso para fase inicial de hill climbing (µm)
        z_step_fine: Paso fino para refinamiento alrededor del pico (µm)
        z_arrive_tol_um: Tolerancia |Z_read−Z_cmd| para condición de llegada (µm)
        n_fine_planes: Capas fine (impar), paso z_step_fine, centradas en Z_coarse*
        roi_margin: Margen adicional alrededor del bbox para sharpness (px)
        max_coarse_iterations: Límite de iteraciones en fase gruesa
        max_fine_iterations: Límite de iteraciones en fase fina
    
    Parámetros de captura multi-focal (para volumetría):
        n_captures: Número de capturas en Z-stack
        z_step_capture: Caída S objetivo respecto al BPoF (%; nombre legacy)
        z_range_capture: Rango total de captura (µm)
    """
    
    # Parámetros de búsqueda de BPoF
    use_full_range: bool = True             # escaneo completo del rango calibrado
    z_scan_range: float = 20.0              # µm - límite ±Δ máximo zona fine
    z_step_coarse: float = 0.5              # µm - paso grueso
    z_step_fine: float = 0.1                # µm - paso real entre planos FINE
    n_fine_planes: int = 15                 # capas fine (impar)
    z_arrive_tol_um: float = 0.5            # µm - condición |err|≤tol
    z_arrive_timeout_s: float = 3.0         # s - timeout seguridad (no settle)
    # Compat legacy (ignorados como sleep; orquestador puede seguir seteándolos)
    settle_time: float = 0.0
    capture_settle_time: float = 0.0
    roi_margin: int = 20                    # px - margen para sharpness
    max_coarse_iterations: int = 50         # límite fase gruesa
    max_fine_iterations: int = 101          # límite impar; coincide con GUI
    
    # Parámetros de captura multi-focal (Z-stack)
    n_captures: int = 3                     # número de capturas (impar)
    z_step_capture: float = 10.0            # % - caída S objetivo (nombre legacy)
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
        elif self.z_step_coarse > 0 and self.z_step_fine >= self.z_step_coarse:
            errors.append("z_step_fine debe ser menor que z_step_coarse")

        if self.n_fine_planes < 3:
            errors.append("n_fine_planes debe ser ≥ 3")

        if self.z_arrive_tol_um <= 0:
            errors.append("z_arrive_tol_um debe ser > 0")

        if self.z_arrive_timeout_s <= 0:
            errors.append("z_arrive_timeout_s debe ser > 0")
        
        # Validar iteraciones
        if self.max_coarse_iterations <= 0:
            errors.append("max_coarse_iterations debe ser > 0")
        
        if self.max_fine_iterations <= 0:
            errors.append("max_fine_iterations debe ser > 0")
        
        # Validar captura multi-focal
        if self.n_captures < 1 or self.n_captures % 2 == 0:
            errors.append("n_captures debe ser impar ≥ 1")

        if not 0 < self.z_step_capture <= 95:
            errors.append("variación S para captura debe estar en (0, 95]%")

        if self.roi_margin < 0:
            errors.append("roi_margin debe ser ≥ 0")

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
        coarse_steps = int(2 * self.z_scan_range / self.z_step_coarse)
        coarse_steps = min(coarse_steps, self.max_coarse_iterations)
        n_fine = int(self.n_fine_planes)
        if n_fine % 2 == 0:
            n_fine += 1
        fine_steps = min(n_fine, self.max_fine_iterations)
        total_steps = coarse_steps + fine_steps
        
        return {
            'coarse_steps': coarse_steps,
            'fine_steps': fine_steps,
            'total_steps': total_steps,
            'estimated_time_s': total_steps * 0.15,
            'z_range_um': 2 * self.z_scan_range,
            'search_distance_um': 2 * self.z_scan_range,
            'z_arrive_tol_um': self.z_arrive_tol_um,
            'n_fine_planes': n_fine,
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

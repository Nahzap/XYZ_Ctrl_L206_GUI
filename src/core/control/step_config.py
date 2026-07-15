"""Configuración de control de pasos homogéneos (calibration.json → step_control)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from config.constants import _load_calibration

HomogeneityMode = Literal["structural", "temporal", "both"]
AxisOrder = Literal["y_then_x", "x_then_y"]
StepControlMode = Literal["hinf_native", "orchestrated"]
DualAxisMode = Literal["full_dual", "primary_only"]


@dataclass
class StepControlConfig:
    enabled: bool = True
    step_control_mode: StepControlMode = "orchestrated"
    step_um: float = 20.0
    tol_step_um: float = 10.0
    tol_fov_um: float = 8.0
    step_timeout_ms: float = 2000.0
    step_dwell_ms: float = 100.0
    step_inter_step_brake: bool = True
    step_dual_axis_mode: DualAxisMode = "primary_only"
    step_hinf_pwm_min: int = 80
    t_step_nominal_ms: float = 900.0
    t_capture_settle_ms: float = 500.0
    axis_order: AxisOrder = "y_then_x"
    max_step_retries: int = 4
    sensor_max_age_ms: float = 30.0
    use_arduino_settled: bool = True
    integral_carryover: float = 0.5
    homogeneity_mode: HomogeneityMode = "structural"
    deadzone_adc: int = 1
    tol_hysteresis_factor: float = 1.3
    hold_resend_ms: float = 250.0
    approach_kp_scale: float = 0.25
    approach_ki_scale: float = 0.008
    approach_deadzone_adc: int = 0
    coarse_step_threshold_um: float = 60.0
    max_steps_per_axis: int = 10
    step_pwm_cap: int = 80
    step_pwm_cap_coarse: int = 135
    step_pwm_min: int = 80
    step_use_integral: bool = True
    step_use_full_hinf_gains: bool = True
    step_timeout_max_ms: float = 6000.0
    long_approach_threshold_um: float = 500.0
    long_approach_done_um: float = 80.0
    sensor_control_max_age_ms: float = 20.0
    # Cierre fino FOV_VERIFY: un eje a la vez (lock), pulso→freno→reposo→medir
    # Permanencia (2.4): aceptar solo si se sostiene ±tol_fov_um ≥ fov_settle_ms.
    fov_settle_ms: float = 300.0
    fov_axis_settle_ms: float = 120.0
    fov_verify_timeout_ms: float = 8000.0
    fov_verify_max_retries: int = 0
    # Pulso fine en TIEMPO DE PARED (2.1). Reemplaza fov_pulse_dwell_ticks
    # (dependiente de la tasa del reloj) por duraciones deterministas en ms:
    #   fov_pulse_on_ms  = ventana de empuje A,pwm
    #   fov_pulse_rest_ms = freno + reposo/asentamiento antes de medir
    fov_pulse_on_ms: float = 12.0
    fov_pulse_rest_ms: float = 350.0
    # Tras best≤tol_fov: no más átomos (solo observar) — log: best 0.6 µm luego spoil.
    fov_freeze_after_best: bool = True
    fov_pulse_dwell_ticks: int = 30  # LEGACY (no usado; se conserva por compat.)
    fov_pwm_min: int = 22
    fov_pwm_step: int = 2
    fov_pwm_cap: int = 45
    fov_target_step_adc: int = 2
    fov_max_step_adc: int = 3
    fov_no_improve_pulses: int = 12
    fov_max_pulses_per_point: int = 40
    fov_overshoot_lock_count: int = 3
    # Soft-lock / observe: cerca de tol_fov (8 µm).
    fov_pulse_gate_um: float = 10.0
    # No reabrir soft-lock por acoplamiento hasta estar fuera del gate este tiempo.
    fov_gate_unlock_hold_ms: float = 280.0
    # Fine FOV — mutuamente excluyentes (load fuerza atom=False si cz=True):
    #   use_mcu_cz_loop:     MCU C(z) @ 1 MHz (canónico)
    #   use_mcu_atom_pulse:  host dispara P,axis,sign,idx (fallback)
    #   ambos False:         host A,pwm timed (legacy)
    use_mcu_atom_pulse: bool = False
    use_mcu_cz_loop: bool = True
    # LUT MCU 0..7 (duty nativo ARR). Cap bajo = pulsos fluidos.
    fov_atom_idx_min: int = 0
    fov_atom_idx_max: int = 2
    # Estimación inicial µm/átomo idx0 (se adapta con FOV_PULSE moved·slope).
    fov_atom_um_per_idx0: float = 8.0

    @property
    def is_hinf_native(self) -> bool:
        return self.step_control_mode == "hinf_native"

    @property
    def use_temporal_padding(self) -> bool:
        return self.homogeneity_mode in ("temporal", "both")

    def deadzone_um(self) -> float:
        """Zona muerta equivalente en µm (peor eje)."""
        from config.constants import CALIBRATION_X, CALIBRATION_Y, DEADZONE_ADC

        dz_adc = self.deadzone_adc if self.deadzone_adc > 0 else DEADZONE_ADC
        return max(
            dz_adc * CALIBRATION_X["slope"],
            dz_adc * CALIBRATION_Y["slope"],
        )

    def effective_tol_step_um(self) -> float:
        """
        Tolerancia alcanzable: no puede ser menor que la zona muerta del PI.

        Con tol=10µm y deadzone=2 ADC (~31µm) el motor deja de corregir antes de
        entrar en banda → oscilación / timeout perpetuo.
        """
        from config.constants import POSITION_TOLERANCE_UM

        return max(self.tol_step_um, self.deadzone_um(), POSITION_TOLERANCE_UM)


_DEFAULT_STEP_CONTROL = {
    "enabled": True,
    "step_control_mode": "hinf_native",
    "step_um": 20.0,
    "tol_step_um": 25.0,
    "tol_fov_um": 8.0,
    "step_timeout_ms": 2000.0,
    "step_dwell_ms": 0.0,
    "step_inter_step_brake": False,
    "step_dual_axis_mode": "primary_only",
    "step_hinf_pwm_min": 80,
    "T_step_nominal_ms": 900.0,
    "T_capture_settle_ms": 500.0,
    "axis_order": "y_then_x",
    "max_step_retries": 4,
    "sensor_max_age_ms": 60.0,
    "use_arduino_settled": False,
    "integral_carryover": 0.5,
    "homogeneity_mode": "structural",
    "deadzone_adc": 8,
    "tol_hysteresis_factor": 1.3,
    "hold_resend_ms": 250,
    "approach_kp_scale": 0.25,
    "approach_ki_scale": 0.008,
    "approach_deadzone_adc": 0,
    "coarse_step_threshold_um": 60.0,
    "max_steps_per_axis": 10,
    "step_pwm_cap": 80,
    "step_pwm_cap_coarse": 135,
    "step_pwm_min": 80,
    "step_use_integral": True,
    "step_use_full_hinf_gains": True,
    "step_timeout_max_ms": 6000.0,
    "long_approach_threshold_um": 500.0,
    "long_approach_done_um": 80.0,
    "sensor_control_max_age_ms": 20.0,
    "fov_settle_ms": 300.0,
    "fov_axis_settle_ms": 120.0,
    "fov_verify_timeout_ms": 8000.0,
    "fov_verify_max_retries": 0,
    "fov_pulse_on_ms": 12.0,
    "fov_pulse_rest_ms": 350.0,
    "fov_freeze_after_best": True,
    "fov_pulse_dwell_ticks": 30,
    "fov_pwm_min": 22,
    "fov_pwm_step": 2,
    "fov_pwm_cap": 45,
    "fov_target_step_adc": 2,
    "fov_max_step_adc": 3,
    "fov_no_improve_pulses": 12,
    "fov_max_pulses_per_point": 40,
    "fov_overshoot_lock_count": 3,
    "fov_pulse_gate_um": 10.0,
    "fov_gate_unlock_hold_ms": 280.0,
    "use_mcu_atom_pulse": False,
    "use_mcu_cz_loop": True,
    "fov_atom_idx_min": 0,
    "fov_atom_idx_max": 2,
    "fov_atom_um_per_idx0": 8.0,
}


def load_step_control_config() -> StepControlConfig:
    data = _load_calibration()
    raw = data.get("step_control", _DEFAULT_STEP_CONTROL)
    cfg = StepControlConfig(
        enabled=bool(raw.get("enabled", True)),
        step_control_mode=str(raw.get("step_control_mode", "orchestrated")),
        step_um=float(raw.get("step_um", 20.0)),
        tol_step_um=float(raw.get("tol_step_um", 10.0)),
        tol_fov_um=float(raw.get("tol_fov_um", 8.0)),
        step_timeout_ms=float(raw.get("step_timeout_ms", 800.0)),
        step_dwell_ms=float(raw.get("step_dwell_ms", 100.0)),
        step_inter_step_brake=bool(raw.get("step_inter_step_brake", True)),
        step_dual_axis_mode=str(raw.get("step_dual_axis_mode", "primary_only")),
        step_hinf_pwm_min=int(raw.get("step_hinf_pwm_min", raw.get("step_pwm_min", 80))),
        t_step_nominal_ms=float(raw.get("T_step_nominal_ms", 900.0)),
        t_capture_settle_ms=float(raw.get("T_capture_settle_ms", 500.0)),
        axis_order=str(raw.get("axis_order", "y_then_x")),
        max_step_retries=int(raw.get("max_step_retries", 2)),
        sensor_max_age_ms=float(raw.get("sensor_max_age_ms", 30.0)),
        use_arduino_settled=bool(raw.get("use_arduino_settled", True)),
        integral_carryover=float(raw.get("integral_carryover", 0.5)),
        homogeneity_mode=str(raw.get("homogeneity_mode", "structural")),
        deadzone_adc=int(raw.get("deadzone_adc", 1)),
        tol_hysteresis_factor=float(raw.get("tol_hysteresis_factor", 1.3)),
        hold_resend_ms=float(raw.get("hold_resend_ms", 250.0)),
        approach_kp_scale=float(raw.get("approach_kp_scale", 0.25)),
        approach_ki_scale=float(raw.get("approach_ki_scale", 0.008)),
        approach_deadzone_adc=int(raw.get("approach_deadzone_adc", 0)),
        coarse_step_threshold_um=float(raw.get("coarse_step_threshold_um", 60.0)),
        max_steps_per_axis=int(raw.get("max_steps_per_axis", 10)),
        step_pwm_cap=int(raw.get("step_pwm_cap", 80)),
        step_pwm_cap_coarse=int(raw.get("step_pwm_cap_coarse", 135)),
        step_pwm_min=int(raw.get("step_pwm_min", 80)),
        step_use_integral=bool(raw.get("step_use_integral", True)),
        step_use_full_hinf_gains=bool(raw.get("step_use_full_hinf_gains", True)),
        step_timeout_max_ms=float(raw.get("step_timeout_max_ms", 6000.0)),
        long_approach_threshold_um=float(raw.get("long_approach_threshold_um", 500.0)),
        long_approach_done_um=float(raw.get("long_approach_done_um", 200.0)),
        sensor_control_max_age_ms=float(raw.get("sensor_control_max_age_ms", 20.0)),
        fov_settle_ms=float(raw.get("fov_settle_ms", 300.0)),
        fov_axis_settle_ms=float(raw.get("fov_axis_settle_ms", 120.0)),
        fov_verify_timeout_ms=float(raw.get("fov_verify_timeout_ms", 8000.0)),
        fov_verify_max_retries=int(raw.get("fov_verify_max_retries", 0)),
        fov_pulse_on_ms=float(raw.get("fov_pulse_on_ms", 12.0)),
        fov_pulse_rest_ms=float(raw.get("fov_pulse_rest_ms", 350.0)),
        fov_freeze_after_best=bool(raw.get("fov_freeze_after_best", True)),
        fov_pulse_dwell_ticks=int(raw.get("fov_pulse_dwell_ticks", 30)),
        fov_pwm_min=int(raw.get("fov_pwm_min", 22)),
        fov_pwm_step=int(raw.get("fov_pwm_step", 2)),
        fov_pwm_cap=int(raw.get("fov_pwm_cap", 45)),
        fov_target_step_adc=int(raw.get("fov_target_step_adc", 2)),
        fov_max_step_adc=int(raw.get("fov_max_step_adc", 3)),
        fov_no_improve_pulses=int(raw.get("fov_no_improve_pulses", 12)),
        fov_max_pulses_per_point=int(raw.get("fov_max_pulses_per_point", 40)),
        fov_overshoot_lock_count=int(raw.get("fov_overshoot_lock_count", 3)),
        fov_pulse_gate_um=float(raw.get("fov_pulse_gate_um", 10.0)),
        fov_gate_unlock_hold_ms=float(raw.get("fov_gate_unlock_hold_ms", 280.0)),
        use_mcu_atom_pulse=bool(raw.get("use_mcu_atom_pulse", False)),
        use_mcu_cz_loop=bool(raw.get("use_mcu_cz_loop", True)),
        fov_atom_idx_min=int(raw.get("fov_atom_idx_min", 0)),
        fov_atom_idx_max=int(raw.get("fov_atom_idx_max", 2)),
        fov_atom_um_per_idx0=float(raw.get("fov_atom_um_per_idx0", 8.0)),
    )
    # C(z) canónico: no mezclar con host-atom (ambos True era no-op confuso).
    if cfg.use_mcu_cz_loop and cfg.use_mcu_atom_pulse:
        cfg.use_mcu_atom_pulse = False
    return cfg

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
    tol_fov_um: float = 25.0
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
    fov_creep_adc: int = 10
    fov_creep_cooldown_ticks: int = 45

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
    "tol_fov_um": 25.0,
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
    "deadzone_adc": 2,
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
    "fov_creep_adc": 10,
    "fov_creep_cooldown_ticks": 45,
}


def load_step_control_config() -> StepControlConfig:
    data = _load_calibration()
    raw = data.get("step_control", _DEFAULT_STEP_CONTROL)
    return StepControlConfig(
        enabled=bool(raw.get("enabled", True)),
        step_control_mode=str(raw.get("step_control_mode", "orchestrated")),
        step_um=float(raw.get("step_um", 20.0)),
        tol_step_um=float(raw.get("tol_step_um", 10.0)),
        tol_fov_um=float(raw.get("tol_fov_um", 25.0)),
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
        fov_creep_adc=int(raw.get("fov_creep_adc", 10)),
        fov_creep_cooldown_ticks=int(raw.get("fov_creep_cooldown_ticks", 45)),
    )

"""Configuración de control de pasos homogéneos (calibration.json → step_control)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from config.constants import _load_calibration, mcu_supports_cz

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
    step_hinf_pwm_min: int = 25
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
    step_pwm_min: int = 25
    step_use_integral: bool = True
    step_use_full_hinf_gains: bool = True
    step_timeout_max_ms: float = 6000.0
    long_approach_threshold_um: float = 500.0
    long_approach_done_um: float = 100.0
    # Host approach ÚNICO (HostApproachController):
    #   SLEW  — |e| > fine_engage_um  → PWM grueso
    #   FINE  — deceleración u∝|e| con histéresis → handoff MCU (no DualPower)
    fine_engage_um: float = 90.0
    # Banco: actuación solo en [95, 150] (STITION_PWM_MIN/MAX).
    slew_pwm: int = 255
    # Handoff: host acerca hasta ~tol; si ya ≤tol se acepta sin MCU.
    handoff_done_factor: float = 1.0
    handoff_done_min_um: float = 12.0
    handoff_abort_factor: float = 1.05
    # Costa post-approach: solo drenar RX/colas; 40 ms basta (antes 80).
    handoff_coast_s: float = 0.04
    # Costa pre-arm MCU: 10 ms (antes 20).
    handoff_pre_arm_coast_s: float = 0.01
    sensor_control_max_age_ms: float = 50.0
    # Piso de frescura en approach host (RX puede atrasarse con flood telemetría).
    approach_sensor_max_age_ms: float = 80.0
    # Nunca auto-invertir polaridad mid-approach (rompe sync host↔MCU I).
    approach_auto_polarity: bool = False
    # Cierre FOV canónico: MCU C(z). Aceptación = SETTLED ∧ residual≤tol.
    fov_settle_ms: float = 300.0
    fov_pos_confirm_n: int = 8
    fov_pos_confirm_ms: float = 100.0
    fov_axis_settle_ms: float = 120.0
    fov_verify_timeout_ms: float = 25000.0
    fov_verify_max_retries: int = 0
    # Edad máxima telemetría durante FOV (GIL cámara puede atrasar RX).
    fov_verify_sensor_max_age_ms: float = 250.0
    fov_cz_rearm_grace_s: float = 0.60
    fov_cz_max_rearms: int = 1
    fov_prog_log_interval_s: float = 2.0
    # Campos legacy de átomo host (no usados en el camino canónico C(z)).
    fov_pulse_on_ms: float = 12.0
    fov_pulse_rest_ms: float = 350.0
    fov_freeze_after_best: bool = True
    fov_pulse_dwell_ticks: int = 30
    fov_pwm_min: int = 22
    fov_pwm_step: int = 2
    fov_pwm_cap: int = 45
    fov_target_step_adc: int = 2
    fov_max_step_adc: int = 3
    fov_no_improve_pulses: int = 12
    fov_max_pulses_per_point: int = 40
    fov_overshoot_lock_count: int = 3
    fov_pulse_gate_um: float = 10.0
    fov_gate_unlock_hold_ms: float = 280.0
    # Fine FOV — C(z) canónico. atom_pulse queda forzado a False si cz=True.
    use_mcu_atom_pulse: bool = False
    use_mcu_cz_loop: bool = True
    fov_cz_max_fires: int = 0
    # Media host corta: el MCU ya filtra (EMA ~2.6 ms). 40 ms duplicaba el lag.
    sensor_estimate_window_ms: float = 8.0
    fov_freeze_hold_ms: float = 50.0
    fov_atom_idx_min: int = 0
    fov_atom_idx_max: int = 2
    fov_atom_um_per_idx0: float = 8.0

    @property
    def is_hinf_native(self) -> bool:
        return self.step_control_mode == "hinf_native"

    @property
    def use_temporal_padding(self) -> bool:
        return self.homogeneity_mode in ("temporal", "both")

    def deadzone_um(self) -> float:
        """Zona muerta host en µm (magnitud).

        Con C(z) MCU el fine no usa esta deadzone: 8 ADC (~29 µm) > step 20 µm
        dejaba H∞ en PWM=0 y la trayectoria aparentaba no moverse (log 13:03).
        """
        from config.constants import DEADZONE_ADC, lsb_um

        if self.use_mcu_cz_loop:
            return max(lsb_um("x"), lsb_um("y"))
        dz_adc = self.deadzone_adc if self.deadzone_adc > 0 else DEADZONE_ADC
        return max(dz_adc * lsb_um("x"), dz_adc * lsb_um("y"))

    def effective_tol_step_um(self) -> float:
        """
        Tolerancia alcanzable del micro-paso host.

        Con C(z): el cierre ±tol_fov lo hace el MCU; aquí basta tol_step.
        Sin C(z): no puede ser menor que la zona muerta del PI.
        """
        from config.constants import POSITION_TOLERANCE_UM

        if self.use_mcu_cz_loop:
            return max(self.tol_step_um, self.deadzone_um())
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
    "step_hinf_pwm_min": 25,
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
    "step_pwm_min": 25,
    "step_use_integral": True,
    "step_use_full_hinf_gains": True,
    "step_timeout_max_ms": 6000.0,
    "long_approach_threshold_um": 500.0,
    "long_approach_done_um": 100.0,
    "fine_engage_um": 90.0,
    "slew_pwm": 255,
    "handoff_done_factor": 1.0,
    "handoff_done_min_um": 12.0,
    "handoff_abort_factor": 1.05,
    "handoff_coast_s": 0.04,
    "handoff_pre_arm_coast_s": 0.01,
    "sensor_control_max_age_ms": 20.0,
    "approach_sensor_max_age_ms": 80.0,
    "approach_auto_polarity": False,
    "fov_settle_ms": 300.0,
    "fov_axis_settle_ms": 120.0,
    "fov_verify_timeout_ms": 25000.0,
    "fov_verify_max_retries": 0,
    "fov_verify_sensor_max_age_ms": 250.0,
    "fov_cz_rearm_grace_s": 0.60,
    "fov_cz_max_rearms": 1,
    "fov_prog_log_interval_s": 2.0,
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
    "fov_cz_max_fires": 0,
    "sensor_estimate_window_ms": 8.0,
    "fov_freeze_hold_ms": 50.0,
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
        step_hinf_pwm_min=int(raw.get("step_hinf_pwm_min", raw.get("step_pwm_min", 25))),
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
        step_pwm_min=int(raw.get("step_pwm_min", 25)),
        step_use_integral=bool(raw.get("step_use_integral", True)),
        step_use_full_hinf_gains=bool(raw.get("step_use_full_hinf_gains", True)),
        step_timeout_max_ms=float(raw.get("step_timeout_max_ms", 6000.0)),
        long_approach_threshold_um=float(raw.get("long_approach_threshold_um", 500.0)),
        long_approach_done_um=float(raw.get("long_approach_done_um", 100.0)),
        fine_engage_um=float(raw.get("fine_engage_um", 90.0)),
        slew_pwm=int(raw.get("slew_pwm", 150)),
        handoff_done_factor=float(raw.get("handoff_done_factor", 1.0)),
        handoff_done_min_um=float(raw.get("handoff_done_min_um", 12.0)),
        handoff_abort_factor=float(raw.get("handoff_abort_factor", 1.05)),
        handoff_coast_s=float(raw.get("handoff_coast_s", 0.04)),
        handoff_pre_arm_coast_s=float(raw.get("handoff_pre_arm_coast_s", 0.01)),
        sensor_control_max_age_ms=float(raw.get("sensor_control_max_age_ms", 20.0)),
        approach_sensor_max_age_ms=float(raw.get("approach_sensor_max_age_ms", 80.0)),
        approach_auto_polarity=bool(raw.get("approach_auto_polarity", False)),
        fov_settle_ms=float(raw.get("fov_settle_ms", 300.0)),
        fov_axis_settle_ms=float(raw.get("fov_axis_settle_ms", 120.0)),
        fov_verify_timeout_ms=float(raw.get("fov_verify_timeout_ms", 25000.0)),
        fov_verify_max_retries=int(raw.get("fov_verify_max_retries", 0)),
        fov_verify_sensor_max_age_ms=float(raw.get("fov_verify_sensor_max_age_ms", 250.0)),
        fov_cz_rearm_grace_s=float(raw.get("fov_cz_rearm_grace_s", 0.60)),
        fov_cz_max_rearms=int(raw.get("fov_cz_max_rearms", 1)),
        fov_prog_log_interval_s=float(raw.get("fov_prog_log_interval_s", 2.0)),
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
        use_mcu_cz_loop=bool(raw.get("use_mcu_cz_loop", mcu_supports_cz())),
        fov_cz_max_fires=int(raw.get("fov_cz_max_fires", 0)),
        sensor_estimate_window_ms=float(raw.get("sensor_estimate_window_ms", 8.0)),
        fov_freeze_hold_ms=float(raw.get("fov_freeze_hold_ms", 100.0)),
        fov_atom_idx_min=int(raw.get("fov_atom_idx_min", 0)),
        fov_atom_idx_max=int(raw.get("fov_atom_idx_max", 2)),
        fov_atom_um_per_idx0=float(raw.get("fov_atom_um_per_idx0", 8.0)),
    )
    # C(z) canónico: no mezclar con host-atom (ambos True era no-op confuso).
    if cfg.use_mcu_cz_loop and cfg.use_mcu_atom_pulse:
        cfg.use_mcu_atom_pulse = False
    return cfg

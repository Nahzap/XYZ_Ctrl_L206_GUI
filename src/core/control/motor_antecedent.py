"""Antecedente empírico de motor: movimiento open-loop + sensado.

Mueve ~Δµm a PWM∈[umin,umax], registra t/ADC/µm/PWM y resume el
comportamiento real (velocidad, K_eff, signo, stiction aparente).
"""

from __future__ import annotations

import csv
import json
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

from config.constants import STITION_PWM_MAX, STITION_PWM_MIN, adc_to_um, lsb_um


@dataclass
class AntecedentSample:
    t_s: float
    adc: float
    pos_um: float
    pwm: int
    delta_um: float


@dataclass
class MotorAntecedentResult:
    motor: str  # "A" | "B"
    axis: str  # "x" | "y"
    sensor_key: str
    host_invert: bool
    pwm_cmd: int
    target_delta_um: float
    direction: int  # +1 / -1 en espacio µm
    umin: int = STITION_PWM_MIN
    umax: int = STITION_PWM_MAX
    samples: List[AntecedentSample] = field(default_factory=list)
    ok: bool = False
    reason: str = ""
    # Métricas
    delta_um: float = 0.0
    duration_s: float = 0.0
    mean_vel_um_s: float = 0.0
    k_eff_um_s_per_pwm: float = 0.0
    sign_matches_cmd: bool = False
    moved: bool = False
    adc_start: float = 0.0
    adc_end: float = 0.0
    lsb_um: float = 0.0
    csv_path: str = ""
    json_path: str = ""

    def summary_lines(self) -> List[str]:
        return [
            f"Motor {self.motor}/{self.axis} sensor={self.sensor_key} invert={self.host_invert}",
            f"PWM={self.pwm_cmd:+d} ∈[{self.umin},{self.umax}]  target Δ={self.target_delta_um:.0f}µm dir={self.direction:+d}",
            f"Δmedido={self.delta_um:+.1f}µm  t={self.duration_s*1000:.0f}ms  "
            f"v̄={self.mean_vel_um_s:.1f}µm/s  K_eff={self.k_eff_um_s_per_pwm:.4f} µm/s/pwm",
            f"movió={'SÍ' if self.moved else 'NO'}  signo_ok={'SÍ' if self.sign_matches_cmd else 'NO'}  "
            f"ADC {self.adc_start:.0f}→{self.adc_end:.0f}",
            f"ok={self.ok} ({self.reason})",
            f"CSV: {self.csv_path}",
            f"JSON: {self.json_path}",
        ]


def default_antecedent_dir() -> Path:
    root = Path(__file__).resolve().parents[2]  # .../src
    out = root / "config" / "motor_antecedent"
    out.mkdir(parents=True, exist_ok=True)
    return out


def finalize_antecedent(
    result: MotorAntecedentResult,
    *,
    out_dir: Optional[Path] = None,
) -> MotorAntecedentResult:
    """Calcula métricas y persiste CSV+JSON."""
    samples = result.samples
    if len(samples) < 2:
        result.ok = False
        result.reason = "pocas muestras"
        return _persist(result, out_dir)

    s0, s1 = samples[0], samples[-1]
    result.adc_start = s0.adc
    result.adc_end = s1.adc
    result.delta_um = s1.pos_um - s0.pos_um
    result.duration_s = max(1e-6, s1.t_s - s0.t_s)
    result.mean_vel_um_s = result.delta_um / result.duration_s
    pwm_abs = max(1, abs(int(result.pwm_cmd)))
    result.k_eff_um_s_per_pwm = result.mean_vel_um_s / float(pwm_abs)
    result.lsb_um = float(lsb_um(result.axis))
    result.moved = abs(result.delta_um) >= max(50.0, 0.05 * abs(result.target_delta_um))
    # En espacio µm: comando positivo debe producir Δ>0 si invert/host_slew están bien.
    # Aquí comparamos signo del Δ medido vs dirección pedida.
    if result.moved:
        result.sign_matches_cmd = (result.delta_um * float(result.direction)) > 0.0
    else:
        result.sign_matches_cmd = False

    reached = abs(result.delta_um) >= 0.85 * abs(result.target_delta_um)
    if result.moved and reached and result.sign_matches_cmd:
        result.ok = True
        result.reason = "antecedente OK"
    elif result.moved and not result.sign_matches_cmd:
        result.ok = False
        result.reason = "movió pero signo invertido (revisar Invertir PWM / mapa sensor)"
    elif result.moved:
        result.ok = True
        result.reason = "movió (Δ parcial) — usable como antecedente"
    else:
        result.ok = False
        result.reason = "sin movimiento neto (PWM bajo stiction o eje equivocado)"

    return _persist(result, out_dir)


def _persist(
    result: MotorAntecedentResult, out_dir: Optional[Path]
) -> MotorAntecedentResult:
    out = out_dir or default_antecedent_dir()
    out.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = f"Motor{result.motor}_{stamp}"
    csv_path = out / f"{stem}.csv"
    json_path = out / f"{stem}.json"
    latest = out / f"latest_{result.motor}.json"

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "adc", "pos_um", "pwm", "delta_um"])
        for s in result.samples:
            w.writerow([f"{s.t_s:.6f}", f"{s.adc:.3f}", f"{s.pos_um:.3f}", s.pwm, f"{s.delta_um:.3f}"])

    payload: Dict[str, Any] = asdict(result)
    # samples ya serializables
    with json_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    with latest.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)

    result.csv_path = str(csv_path)
    result.json_path = str(json_path)
    return result


def load_latest_antecedent(motor: str) -> Optional[Dict[str, Any]]:
    path = default_antecedent_dir() / f"latest_{motor.upper()}.json"
    if not path.exists():
        return None
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)

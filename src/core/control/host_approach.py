"""Approach host con ley H∞ de la TF cargada.

  SLEW / FINE: u = sat(Kp·e + Ki·∫e, ±U_max)  — misma idea que HinfActuator
  HOLD (|e|≤done): rampa a 0
  u_run: piso anti-stiction (antecedente), no techo de potencia
  U_max: W2 / síntesis H∞ (p.ej. 255)

La potencia baja al acercarse porque |e| baja (PI), no por un open-loop fijo.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

from config.constants import STITION_PWM_MAX, STITION_PWM_MIN
from core.control.motor_antecedent import load_latest_antecedent


@dataclass
class HostApproachConfig:
    done_um: float = 20.0
    engage_um: float = 90.0
    umax: int = STITION_PWM_MAX
    settle_ms: float = 80.0
    t_stop_s: float = 0.40
    slew_up_per_s: float = 350.0
    slew_down_per_s: float = 1200.0  # bajar rápido al corregir (sigue la TF)
    ema_alpha: float = 0.30


@dataclass
class _AxisAp:
    pwm: float = 0.0
    hold_ms: float = 0.0
    e_filt: float = 0.0
    e_init: bool = False
    fine_latched: bool = False
    k_eff: float = 2.0
    u_run: int = 110
    kp: float = 1.0
    ki: float = 0.0
    integral: float = 0.0
    last_e: float = 0.0


@dataclass
class HostApproachController:
    config: HostApproachConfig = field(default_factory=HostApproachConfig)
    _axes: Dict[str, _AxisAp] = field(default_factory=dict)
    _settled: bool = False
    entered_fine: bool = False
    _residual_ok_ms: float = 0.0

    def reset(
        self,
        done_um: float,
        engage_um: float,
        slew_pwm: int = STITION_PWM_MAX,
        *,
        umin: int = STITION_PWM_MIN,
        kp: Optional[float] = None,
        ki: Optional[float] = None,
        kp_x: Optional[float] = None,
        ki_x: Optional[float] = None,
        kp_y: Optional[float] = None,
        ki_y: Optional[float] = None,
        k_eff_x: Optional[float] = None,
        k_eff_y: Optional[float] = None,
    ) -> None:
        del umin
        done = max(5.0, float(done_um))
        engage = max(done + 20.0, float(engage_um))
        umax = max(int(STITION_PWM_MIN), min(int(STITION_PWM_MAX), int(slew_pwm)))
        self.config = HostApproachConfig(
            done_um=done,
            engage_um=engage,
            umax=umax,
        )
        self._axes.clear()
        self._settled = False
        self.entered_fine = False
        self._residual_ok_ms = 0.0

        meta_a = self._load_motor_meta("A", 1.92, 120)
        meta_b = self._load_motor_meta("B", 3.02, 120)
        if k_eff_x is not None:
            meta_a = (float(k_eff_x), meta_a[1])
        if k_eff_y is not None:
            meta_b = (float(k_eff_y), meta_b[1])

        # Ganancias H∞ por eje (fallback a kp/ki genéricos)
        kx = float(kp_x if kp_x is not None else (kp if kp is not None else 1.0))
        kix = float(ki_x if ki_x is not None else (ki if ki is not None else 0.0))
        ky = float(kp_y if kp_y is not None else (kp if kp is not None else 1.0))
        kiy = float(ki_y if ki_y is not None else (ki if ki is not None else 0.0))

        ax = self._ax("x")
        ay = self._ax("y")
        ax.k_eff, ax.u_run = meta_a
        ay.k_eff, ay.u_run = meta_b
        ax.u_run = min(ax.u_run, umax)
        ay.u_run = min(ay.u_run, umax)
        ax.kp, ax.ki = max(1e-6, kx), max(0.0, kix)
        ay.kp, ay.ki = max(1e-6, ky), max(0.0, kiy)

    @staticmethod
    def _load_motor_meta(motor: str, ke_default: float, pwm_default: int) -> Tuple[float, int]:
        ke, pwm = ke_default, pwm_default
        try:
            data = load_latest_antecedent(motor)
            if data:
                ke = float(data.get("k_eff_um_s_per_pwm", ke_default))
                pwm = int(data.get("pwm_cmd", pwm_default))
        except Exception:
            pass
        ke = max(0.4, min(20.0, ke))
        u_run = max(
            int(STITION_PWM_MIN),
            min(int(STITION_PWM_MAX), int(round(0.95 * abs(pwm)))),
        )
        return ke, u_run

    def _ax(self, key: str) -> _AxisAp:
        if key not in self._axes:
            self._axes[key] = _AxisAp()
        return self._axes[key]

    @property
    def settled(self) -> bool:
        return self._settled

    def tick_axis(
        self,
        key: str,
        error_um: float,
        Ts: float,
        *,
        invert: bool = False,
        local_sign: int = 1,
    ) -> Tuple[int, str]:
        Ts = max(1e-4, float(Ts))
        cfg = self.config
        ax = self._ax(key)

        e_act = float(error_um)
        if invert:
            e_act = -e_act
        if int(local_sign) < 0:
            e_act = -e_act

        if not ax.e_init:
            ax.e_filt = e_act
            ax.e_init = True
        else:
            a = cfg.ema_alpha
            ax.e_filt = (1.0 - a) * ax.e_filt + a * e_act

        ae = abs(ax.e_filt)
        sign = 1.0 if ax.e_filt >= 0.0 else -1.0
        umax = float(cfg.umax)
        u_floor = float(ax.u_run)

        if ae <= cfg.done_um:
            ax.hold_ms += Ts * 1000.0
            ax.integral = 0.0
            ax.last_e = ax.e_filt
            step = cfg.slew_down_per_s * Ts
            if ax.pwm > 0:
                ax.pwm = max(0.0, ax.pwm - step)
            elif ax.pwm < 0:
                ax.pwm = min(0.0, ax.pwm + step)
            else:
                ax.pwm = 0.0
            return int(round(ax.pwm)), "HOLD"

        ax.hold_ms = 0.0
        ax.fine_latched = True
        label = "FINE" if ae < cfg.engage_um else "SLEW"

        # --- Ley H∞ PI (como HinfActuator) ---
        if ax.last_e * ax.e_filt < 0.0:
            ax.integral = 0.0
        ax.integral += ax.e_filt * Ts
        u_pi = ax.kp * ax.e_filt + ax.ki * ax.integral
        ax.last_e = ax.e_filt

        # Saturación ±U_max + anti-windup
        if abs(u_pi) > umax:
            ax.integral -= ax.e_filt * Ts
            u_pi = max(-umax, min(umax, u_pi))

        # SLEW lejos: si la PI aún no satura, asegurar avance (≥ piso)
        if label == "SLEW":
            if abs(u_pi) < u_floor:
                u_des = sign * umax  # error grande → potencia TF completa
            else:
                u_des = u_pi
        else:
            # FINE: seguir PI. Cerca de done NO forzar ±u_run (caza/overshoot).
            u_des = u_pi
            near_band = ae <= 2.0 * cfg.done_um
            if near_band:
                if abs(u_des) < float(STITION_PWM_MIN):
                    u_des = 0.0
            elif abs(u_des) > 1.0 and abs(u_des) < u_floor:
                u_des = sign * u_floor
            u_brake = ae / max(1e-3, ax.k_eff * cfg.t_stop_s)
            if near_band:
                u_cap = min(umax, max(u_brake, abs(u_des)))
            else:
                u_cap = min(umax, max(u_floor, u_brake))
            if u_cap > 0 and abs(u_des) > u_cap:
                u_des = sign * u_cap

        # Cambio de signo: drenar antes de invertir
        if abs(ax.pwm) > 1.0 and (ax.pwm * ax.e_filt) < 0:
            u_des = 0.0
            step = cfg.slew_down_per_s * Ts
        else:
            step = (
                cfg.slew_up_per_s * Ts
                if abs(u_des) > abs(ax.pwm)
                else cfg.slew_down_per_s * Ts
            )

        du = u_des - ax.pwm
        if du > step:
            du = step
        elif du < -step:
            du = -step
        ax.pwm += du

        if 0.0 < abs(ax.pwm) < u_floor - 0.5 and abs(u_des) < 1.0:
            ax.pwm = 0.0

        out = int(round(ax.pwm))
        if 0 < abs(out) < ax.u_run and abs(u_des) < 1:
            out = 0
            ax.pwm = 0.0
        elif abs(out) > int(umax):
            out = int(umax if out > 0 else -umax)
            ax.pwm = float(out)

        return out, label

    def update_settle(
        self,
        axis_labels: Tuple[str, str],
        Ts: float,
        residual_um: Optional[float] = None,
    ) -> bool:
        Ts = max(1e-4, float(Ts))
        a = self._ax("x")
        b = self._ax("y")
        done = float(self.config.done_um)

        if residual_um is not None and float(residual_um) <= done + 1e-6:
            self._residual_ok_ms += Ts * 1000.0
        else:
            self._residual_ok_ms = 0.0

        hold_ok = (
            axis_labels[0] == "HOLD"
            and axis_labels[1] == "HOLD"
            and min(a.hold_ms, b.hold_ms) >= self.config.settle_ms
            and abs(a.pwm) < 2.0
            and abs(b.pwm) < 2.0
        )
        residual_ok = self._residual_ok_ms >= self.config.settle_ms

        if hold_ok or residual_ok:
            self._settled = True
            return True
        self._settled = False
        return False

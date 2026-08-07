"""Asignación de potencia suave por eje para Control Dual / approach.

Estados independientes A/B:
  HOLD   — |e| ≤ tol sostenido hold_enter_ms → PWM = 0
  FINE   — tol < |e| ≤ fine                  → PI suave + rampa
  COARSE — |e| > fine                        → PI + rampa (paso mayor)

Approach (suave / unitaria / continua):
  - Kickstart UNA vez al salir de HOLD (±pwm_min); luego ΔPWM ≤ 1 (FINE).
  - Techo suave u ∝ |e| (cerca del target poca potencia).
  - En banda: freno duro PWM=0 (no coast con 70–90 residual).
  - Cruce de signo del error: freno inmediato (anti-overshoot).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional, Tuple

from core.control.controller_config import ControllerConfig


class DualAxisState(str, Enum):
    HOLD = "HOLD"
    FINE = "FINE"
    COARSE = "COARSE"


@dataclass
class DualPowerConfig:
    tol_um: float = 25.0
    fine_band_um: float = 75.0
    gain_fine: float = 0.35
    gain_coarse: float = 1.0
    pwm_min_fine: int = 20
    pwm_min_coarse: int = 40
    slew_fine_per_s: float = 120.0
    slew_coarse_per_s: float = 400.0
    brake_slew_per_s: float = 2500.0
    settle_ms: float = 100.0
    lost_hyst: float = 1.5
    hold_enter_ms: float = 0.0
    kickstart: bool = False
    # True: ΔPWM máximo 1 (FINE) / 2 (COARSE) por tick — rampa unitaria.
    unitary_step: bool = False
    # u_max_soft = max(pwm_min, soft_cap_k * |e|); None desactiva.
    soft_cap_k: Optional[float] = None
    # Al entrar en banda: PWM=0 inmediato (recomendado en approach).
    hard_brake_in_band: bool = False
    # Por debajo de esto: coast/freno sin piso stiction (evita spoil ±95).
    landing_um: Optional[float] = None
    u_max_cap: Optional[float] = None

    @classmethod
    def from_calibration(cls) -> "DualPowerConfig":
        try:
            from config.constants import POSITION_TOLERANCE_UM, _load_calibration
        except Exception:
            return cls()

        tol = float(POSITION_TOLERANCE_UM)
        cfg = cls(tol_um=tol, fine_band_um=max(tol * 3.0, tol + 1.0))
        try:
            raw = _load_calibration().get("dual_power") or {}
        except Exception:
            raw = {}
        if not isinstance(raw, dict):
            return cfg
        for key in (
            "tol_um",
            "fine_band_um",
            "gain_fine",
            "gain_coarse",
            "pwm_min_fine",
            "pwm_min_coarse",
            "slew_fine_per_s",
            "slew_coarse_per_s",
            "brake_slew_per_s",
            "settle_ms",
            "lost_hyst",
            "hold_enter_ms",
            "kickstart",
            "unitary_step",
            "soft_cap_k",
            "hard_brake_in_band",
            "u_max_cap",
        ):
            if key in raw and raw[key] is not None:
                if key == "u_max_cap" or key == "soft_cap_k":
                    setattr(cfg, key, float(raw[key]))
                elif key in ("kickstart", "unitary_step", "hard_brake_in_band"):
                    setattr(cfg, key, bool(raw[key]))
                else:
                    typ = type(getattr(cfg, key))
                    setattr(cfg, key, typ(raw[key]))
        if cfg.fine_band_um <= cfg.tol_um:
            cfg.fine_band_um = cfg.tol_um * 3.0
        return cfg

    @classmethod
    def for_approach(cls, done_um: float) -> "DualPowerConfig":
        """FINE host (solo |e|≤fine_engage): PI suave sobre umbral de stiction.

        Banco real (app_cz.h): pwm≤60 no mueve; UMIN MCU=95. El techo 70
        anterior era 'instrucción fantasma' — la mesa no se movía.
        """
        done = max(5.0, float(done_um))
        return cls(
            tol_um=done,
            # Banda FINE amplia: el COARSE del allocator es raro (SLEW host cubre lejos).
            fine_band_um=max(done * 3.0, 85.0),
            gain_fine=0.18,
            gain_coarse=0.45,
            # Piso ≥ stiction (95). Unitary rampa desde kick=95.
            pwm_min_fine=95,
            pwm_min_coarse=110,
            slew_fine_per_s=160.0,
            slew_coarse_per_s=280.0,
            brake_slew_per_s=4000.0,
            settle_ms=60.0,
            lost_hyst=1.12,
            hold_enter_ms=55.0,
            kickstart=True,
            unitary_step=True,
            soft_cap_k=1.35,
            hard_brake_in_band=True,
            # Sin landing/coast: PWM=0 con |e|>tol parecía "movimiento nulo".
            # Freno solo dentro de tol; fuera mantiene piso stiction continuo.
            landing_um=None,
            u_max_cap=150.0,
        )

    @classmethod
    def for_dual(cls) -> "DualPowerConfig":
        """Control dual host: mismo piso anti-stiction (nunca PWM=1..60 fantasma)."""
        try:
            from config.constants import POSITION_TOLERANCE_UM, STITION_PWM_MIN
            tol = float(POSITION_TOLERANCE_UM)
            floor = int(STITION_PWM_MIN)
        except Exception:
            tol, floor = 25.0, 95
        return cls(
            tol_um=tol,
            fine_band_um=max(tol * 3.0, 75.0),
            gain_fine=0.30,
            gain_coarse=0.85,
            pwm_min_fine=floor,
            pwm_min_coarse=max(floor + 15, 110),
            slew_fine_per_s=400.0,
            slew_coarse_per_s=800.0,
            brake_slew_per_s=4000.0,
            settle_ms=80.0,
            lost_hyst=1.3,
            hold_enter_ms=40.0,
            kickstart=True,
            unitary_step=False,
            soft_cap_k=None,
            hard_brake_in_band=True,
            landing_um=None,
            u_max_cap=150.0,
        )


@dataclass
class _AxisRuntime:
    state: DualAxisState = DualAxisState.HOLD
    integral: float = 0.0
    pwm: float = 0.0
    hold_since_mono: Optional[float] = None
    in_band_since_mono: Optional[float] = None
    kick_pending: bool = False
    last_e: Optional[float] = None
    landing_coast_ms: float = 0.0
    landing_kick_ms: float = 0.0  # pulso ON restante (ms)


@dataclass
class DualPowerAllocator:
    config: DualPowerConfig = field(default_factory=DualPowerConfig.from_calibration)
    _axes: Dict[str, _AxisRuntime] = field(default_factory=dict)
    _both_hold_since_mono: Optional[float] = None
    _both_settled: bool = False

    def reset(self) -> None:
        self._axes.clear()
        self._both_hold_since_mono = None
        self._both_settled = False

    def _axis(self, key: str) -> _AxisRuntime:
        if key not in self._axes:
            self._axes[key] = _AxisRuntime()
        return self._axes[key]

    def state_of(self, key: str) -> DualAxisState:
        return self._axis(key).state

    @property
    def both_settled(self) -> bool:
        return self._both_settled

    def _enter_hold(self, ax: _AxisRuntime, now_mono: float) -> Tuple[int, DualAxisState]:
        ax.state = DualAxisState.HOLD
        ax.pwm = 0.0
        ax.integral = 0.0
        ax.kick_pending = False
        ax.hold_since_mono = now_mono
        ax.in_band_since_mono = now_mono
        return 0, ax.state

    def _apply_slew(
        self,
        ax: _AxisRuntime,
        target: float,
        *,
        Ts: float,
        slew_up: float,
        brake_slew: float,
        unitary: bool,
        unitary_max: float,
        u_max: float,
    ) -> None:
        """Rampa hacia target; freno más rápido al reducir / invertir."""
        braking = abs(target) < abs(ax.pwm) - 1e-6 or (
            abs(ax.pwm) > 1e-6 and target * ax.pwm < 0.0
        )
        if unitary:
            max_step = float(unitary_max)
            if braking:
                max_step = max(max_step, max(1.0, brake_slew * Ts))
        else:
            max_step = max(0.0, (brake_slew if braking else slew_up) * Ts)

        delta = float(target) - ax.pwm
        if delta > max_step:
            ax.pwm += max_step
        elif delta < -max_step:
            ax.pwm -= max_step
        else:
            ax.pwm = float(target)

        if ax.pwm > u_max:
            ax.pwm = u_max
        elif ax.pwm < -u_max:
            ax.pwm = -u_max

    def tick_axis(
        self,
        key: str,
        error_um: float,
        Ts: float,
        ctrl: Optional[ControllerConfig],
        *,
        now_mono: float,
    ) -> Tuple[int, DualAxisState]:
        ax = self._axis(key)
        Ts = max(1e-4, float(Ts))
        e = float(error_um)
        ae = abs(e)
        cfg = self.config

        if ctrl is None:
            ax.last_e = e
            return self._enter_hold(ax, now_mono)

        tol = float(cfg.tol_um)
        fine = float(cfg.fine_band_um)
        exit_hold = tol * float(cfg.lost_hyst)
        hold_enter_ms = max(0.0, float(cfg.hold_enter_ms))

        # Anti-overshoot: cruce de signo → freno duro
        if (
            ax.last_e is not None
            and ax.state != DualAxisState.HOLD
            and e * ax.last_e < 0.0
            and abs(ax.last_e) > tol * 0.5
        ):
            ax.pwm = 0.0
            ax.integral = 0.0
            ax.kick_pending = False

        if ax.state == DualAxisState.HOLD:
            if ae > exit_hold:
                ax.state = DualAxisState.COARSE if ae > fine else DualAxisState.FINE
                ax.hold_since_mono = None
                ax.in_band_since_mono = None
                self._both_settled = False
                ax.kick_pending = bool(cfg.kickstart)
            else:
                ax.pwm = 0.0
                ax.integral = 0.0
                if ax.hold_since_mono is None:
                    ax.hold_since_mono = now_mono
                ax.last_e = e
                return 0, ax.state
        else:
            landing = float(cfg.landing_um) if cfg.landing_um is not None else 0.0
            in_landing = landing > 0.0 and ae <= landing
            if ae <= tol or in_landing:
                if ae <= tol:
                    if ax.in_band_since_mono is None:
                        ax.in_band_since_mono = now_mono
                    held_ms = (now_mono - ax.in_band_since_mono) * 1000.0
                    if held_ms >= hold_enter_ms:
                        ax.last_e = e
                        return self._enter_hold(ax, now_mono)
                    ax.landing_coast_ms = 0.0
                else:
                    # Landing: pulsos cortos ≥stiction (como MCU kick) + coast.
                    ax.in_band_since_mono = None
                    u_dir = float(e)
                    if bool(ctrl.invert):
                        u_dir = -u_dir
                    kick_mag = float(cfg.pwm_min_fine)
                    # Primera entrada a landing → pulso inmediato
                    if ax.landing_kick_ms <= 0.0 and ax.landing_coast_ms <= 0.0:
                        ax.landing_kick_ms = 18.0
                    if ax.landing_kick_ms > 0.0:
                        ax.landing_kick_ms = max(0.0, ax.landing_kick_ms - Ts * 1000.0)
                        ax.pwm = float(kick_mag if u_dir > 0 else -kick_mag)
                        if ax.landing_kick_ms <= 0.0:
                            ax.landing_coast_ms = 0.0
                    else:
                        ax.landing_coast_ms += Ts * 1000.0
                        ax.pwm = 0.0
                        # ~90 ms OFF; si aún fuera de tol, otro pulso ~18 ms ON
                        if ax.landing_coast_ms >= 90.0:
                            ax.landing_kick_ms = 18.0
                            ax.landing_coast_ms = 0.0
                            ax.pwm = float(kick_mag if u_dir > 0 else -kick_mag)
                    ax.integral = 0.0
                    ax.kick_pending = False
                    ax.hold_since_mono = None
                    self._both_settled = False
                    ax.last_e = e
                    pwm_out = int(round(ax.pwm))
                    return (0 if abs(pwm_out) < 1 else pwm_out), DualAxisState.FINE
                # Confirmando banda tol: freno duro.
                ax.state = DualAxisState.FINE
                if cfg.hard_brake_in_band:
                    ax.pwm = 0.0
                else:
                    self._apply_slew(
                        ax,
                        0.0,
                        Ts=Ts,
                        slew_up=float(cfg.slew_fine_per_s),
                        brake_slew=float(cfg.brake_slew_per_s),
                        unitary=False,
                        unitary_max=1.0,
                        u_max=float(cfg.u_max_cap or 150.0),
                    )
                ax.integral = 0.0
                ax.kick_pending = False
                ax.hold_since_mono = None
                self._both_settled = False
                ax.last_e = e
                return int(round(ax.pwm)), ax.state
            # Abortó confirmación de banda (ruido): re-armar kickstart si PWM bajo.
            if cfg.kickstart and abs(ax.pwm) < float(cfg.pwm_min_fine):
                ax.kick_pending = True
            ax.in_band_since_mono = None
            ax.landing_coast_ms = 0.0
            ax.landing_kick_ms = 0.0
            ax.state = DualAxisState.COARSE if ae > fine else DualAxisState.FINE
            ax.hold_since_mono = None
            self._both_settled = False

        if ax.state == DualAxisState.FINE:
            gain = float(cfg.gain_fine)
            pwm_min = int(cfg.pwm_min_fine)
            slew_up = float(cfg.slew_fine_per_s)
            unitary_max = 1.0
        else:
            gain = float(cfg.gain_coarse)
            pwm_min = int(cfg.pwm_min_coarse)
            slew_up = float(cfg.slew_coarse_per_s)
            unitary_max = 2.0

        ax.integral += e * Ts
        u = gain * (float(ctrl.Kp) * e + float(ctrl.Ki) * ax.integral)
        if bool(ctrl.invert):
            u = -u

        u_max = float(getattr(ctrl, "U_max", 150.0) or 150.0)
        if cfg.u_max_cap is not None:
            u_max = min(u_max, float(cfg.u_max_cap))
        # Techo suave ∝ |e|, pero nunca por debajo del piso anti-stiction.
        if cfg.soft_cap_k is not None and cfg.soft_cap_k > 0.0:
            soft = max(float(pwm_min), float(cfg.soft_cap_k) * ae)
            u_max = min(u_max, max(float(pwm_min), soft))

        if abs(u) > u_max:
            ax.integral -= e * Ts
            u = u_max if u > 0 else -u_max

        if abs(u) > 1e-6 and abs(u) < pwm_min:
            u = float(pwm_min if u > 0 else -pwm_min)

        # Kickstart solo al salir de HOLD (una vez). Nunca re-snap en cada tick.
        if cfg.kickstart and ax.kick_pending and abs(u) >= pwm_min - 1e-6:
            ax.pwm = float(pwm_min if u > 0 else -pwm_min)
        ax.kick_pending = False

        self._apply_slew(
            ax,
            float(u),
            Ts=Ts,
            slew_up=slew_up,
            brake_slew=float(cfg.brake_slew_per_s),
            unitary=bool(cfg.unitary_step),
            unitary_max=unitary_max,
            u_max=u_max,
        )

        pwm_out = int(round(ax.pwm))
        if abs(pwm_out) < 1:
            pwm_out = 0
        ax.last_e = e
        return pwm_out, ax.state

    def update_settle(
        self, now_mono: float, axis_keys: Tuple[str, ...] = ("a", "b")
    ) -> bool:
        all_hold = True
        for k in axis_keys:
            if self._axis(k).state != DualAxisState.HOLD:
                all_hold = False
                break
        if not all_hold:
            self._both_hold_since_mono = None
            self._both_settled = False
            return False
        if self._both_hold_since_mono is None:
            self._both_hold_since_mono = now_mono
        if (now_mono - self._both_hold_since_mono) * 1000.0 >= float(
            self.config.settle_ms
        ):
            self._both_settled = True
            return True
        return False

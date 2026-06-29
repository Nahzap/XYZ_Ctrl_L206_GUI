"""Estimación en línea del PWM mínimo crítico por eje (fricción / deadband mecánico)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional


@dataclass
class _AxisCrit:
    learned: int = 0
    stall_ticks: int = 0
    last_adc: Optional[int] = None


@dataclass
class PwmCritEstimator:
    """
    Aprende cuánto PWM hace falta para mover cada eje.

    - Arranca sin piso artificial (solo salida K(z)).
    - Si hay PWM pero el ADC no cambia, sube el mínimo aprendido.
    - Si hay movimiento, registra el menor |PWM| efectivo observado.
    - ``pwm_cap`` limita el piso aprendido (techo de configuración).
    """

    pwm_cap: int = 80
    stall_ticks_before_bump: int = 4
    bump_step: int = 5

    _axes: Dict[str, _AxisCrit] = field(default_factory=lambda: {"x": _AxisCrit(), "y": _AxisCrit()})

    def reset(self) -> None:
        self._axes = {"x": _AxisCrit(), "y": _AxisCrit()}

    def _st(self, axis: str) -> _AxisCrit:
        return self._axes[axis]

    def effective_min(self, axis: str) -> int:
        """0 = confiar solo en K(z); >0 = piso aprendido para vencer fricción."""
        return self._st(axis).learned

    def apply_floor(self, axis: str, pwm: int) -> int:
        if pwm == 0:
            return 0
        crit = self.effective_min(axis)
        if crit <= 0:
            return pwm
        if abs(pwm) < crit:
            return crit if pwm > 0 else -crit
        return pwm

    def observe(self, axis: str, pwm: int, adc: Optional[int]) -> None:
        if adc is None:
            return
        st = self._st(axis)
        if pwm == 0:
            st.stall_ticks = 0
            st.last_adc = adc
            return

        prev = st.last_adc
        st.last_adc = adc
        if prev is None:
            st.stall_ticks = 0
            return

        if adc != prev:
            st.stall_ticks = 0
            mag = abs(pwm)
            if st.learned <= 0:
                st.learned = min(mag, self.pwm_cap)
            else:
                st.learned = max(1, min(st.learned, mag))
            return

        st.stall_ticks += 1
        if st.stall_ticks < self.stall_ticks_before_bump:
            return

        st.stall_ticks = 0
        base = abs(pwm) if st.learned <= 0 else st.learned
        st.learned = min(self.pwm_cap, base + self.bump_step)

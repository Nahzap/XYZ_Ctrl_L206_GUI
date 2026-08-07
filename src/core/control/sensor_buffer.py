"""Buffer thread-safe de lecturas de sensores con historial para estimación estable."""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple


@dataclass
class SensorSample:
    adc: int
    t_monotonic: float


@dataclass
class TelemetryFrame:
    pot_a: int = 0
    pot_b: int = 0
    sensor_1: Optional[int] = None
    sensor_2: Optional[int] = None
    state: str = ""
    settled: bool = False
    t_monotonic: float = 0.0


class SensorBuffer:
    """Telemetría serial + ventana para media/mediana (no decidir con 1 muestra)."""

    def __init__(self, history_len: int = 128):
        self._lock = threading.Lock()
        self._s1: Optional[SensorSample] = None
        self._s2: Optional[SensorSample] = None
        self._hist_1: Deque[SensorSample] = deque(maxlen=max(8, history_len))
        self._hist_2: Deque[SensorSample] = deque(maxlen=max(8, history_len))
        self._last_frame: Optional[TelemetryFrame] = None
        self._update_count = 0

    def update(
        self,
        sensor_1: int,
        sensor_2: int,
        pot_a: int = 0,
        pot_b: int = 0,
        settled: bool = False,
        state: str = "",
    ) -> None:
        now = time.perf_counter()
        s1 = SensorSample(int(sensor_1), now)
        s2 = SensorSample(int(sensor_2), now)
        with self._lock:
            self._s1 = s1
            self._s2 = s2
            self._hist_1.append(s1)
            self._hist_2.append(s2)
            self._last_frame = TelemetryFrame(
                pot_a=int(pot_a),
                pot_b=int(pot_b),
                sensor_1=int(sensor_1),
                sensor_2=int(sensor_2),
                state=str(state),
                settled=bool(settled),
                t_monotonic=now,
            )
            self._update_count += 1

    @property
    def update_count(self) -> int:
        with self._lock:
            return self._update_count

    def get_adc(self, key: str) -> Optional[int]:
        """Última muestra (UI / debug). Control fino debe usar get_adc_mean."""
        sample = self.get_sample(key)
        return sample.adc if sample is not None else None

    def get_sample(self, key: str) -> Optional[SensorSample]:
        with self._lock:
            if key == "sensor_1":
                return self._s1
            if key == "sensor_2":
                return self._s2
        return None

    def _hist(self, key: str) -> Deque[SensorSample]:
        if key == "sensor_1":
            return self._hist_1
        if key == "sensor_2":
            return self._hist_2
        raise KeyError(key)

    def get_adc_mean(self, key: str, window_ms: float = 40.0) -> Optional[float]:
        """Media en ventana temporal (estimación estable para FOV/settle)."""
        now = time.perf_counter()
        with self._lock:
            hist = self._hist(key)
            if not hist:
                return None
            vals = [
                s.adc
                for s in hist
                if (now - s.t_monotonic) * 1000.0 <= window_ms
            ]
            if not vals:
                vals = [hist[-1].adc]
            return float(sum(vals)) / float(len(vals))

    def get_adc_median(self, key: str, window_ms: float = 40.0) -> Optional[float]:
        now = time.perf_counter()
        with self._lock:
            hist = self._hist(key)
            if not hist:
                return None
            vals = sorted(
                s.adc
                for s in hist
                if (now - s.t_monotonic) * 1000.0 <= window_ms
            )
            if not vals:
                return float(hist[-1].adc)
            mid = len(vals) // 2
            if len(vals) % 2:
                return float(vals[mid])
            return 0.5 * (vals[mid - 1] + vals[mid])

    def get_adc_std(self, key: str, window_ms: float = 40.0) -> Optional[float]:
        """Dispersion en ventana (ruido observado en telemetría)."""
        now = time.perf_counter()
        with self._lock:
            hist = self._hist(key)
            if not hist:
                return None
            vals = [
                float(s.adc)
                for s in hist
                if (now - s.t_monotonic) * 1000.0 <= window_ms
            ]
            if len(vals) < 2:
                return 0.0
            mean = sum(vals) / len(vals)
            var = sum((v - mean) ** 2 for v in vals) / (len(vals) - 1)
            return math.sqrt(var)

    def estimate_xy_adc(
        self, key_x: str, key_y: str, window_ms: float = 40.0
    ) -> Tuple[Optional[float], Optional[float]]:
        return self.get_adc_mean(key_x, window_ms), self.get_adc_mean(key_y, window_ms)

    def age_ms(self, key: str) -> float:
        sample = self.get_sample(key)
        if sample is None:
            return float("inf")
        return max(0.0, (time.perf_counter() - sample.t_monotonic) * 1000.0)

    def is_fresh(self, key: str, max_age_ms: float) -> bool:
        return self.age_ms(key) <= max_age_ms

    def history_span_ms(self, key: str) -> float:
        with self._lock:
            hist = self._hist(key)
            if len(hist) < 2:
                return 0.0
            return max(0.0, (hist[-1].t_monotonic - hist[0].t_monotonic) * 1000.0)

    def is_settled(self) -> bool:
        with self._lock:
            if self._last_frame is None:
                return False
            return self._last_frame.settled

    def last_frame(self) -> Optional[TelemetryFrame]:
        with self._lock:
            return self._last_frame

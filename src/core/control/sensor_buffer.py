"""Buffer thread-safe de lecturas de sensores con timestamp."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional


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
    """Almacena la última telemetría serial con marca temporal."""

    def __init__(self):
        self._lock = threading.Lock()
        self._s1: Optional[SensorSample] = None
        self._s2: Optional[SensorSample] = None
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
        with self._lock:
            self._s1 = SensorSample(int(sensor_1), now)
            self._s2 = SensorSample(int(sensor_2), now)
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
        sample = self.get_sample(key)
        return sample.adc if sample is not None else None

    def get_sample(self, key: str) -> Optional[SensorSample]:
        with self._lock:
            if key == "sensor_1":
                return self._s1
            if key == "sensor_2":
                return self._s2
        return None

    def age_ms(self, key: str) -> float:
        sample = self.get_sample(key)
        if sample is None:
            return float("inf")
        return max(0.0, (time.perf_counter() - sample.t_monotonic) * 1000.0)

    def is_fresh(self, key: str, max_age_ms: float) -> bool:
        return self.age_ms(key) <= max_age_ms

    def is_settled(self) -> bool:
        with self._lock:
            if self._last_frame is None:
                return False
            return self._last_frame.settled

    def last_frame(self) -> Optional[TelemetryFrame]:
        with self._lock:
            return self._last_frame

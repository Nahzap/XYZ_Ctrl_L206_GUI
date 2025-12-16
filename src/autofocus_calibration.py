"""Standalone calibration routine for C-Focus using SmartFocusScorer.

This script implements the "Smart Autofocus" morphology-based calibration
routine you described:

- Scans Z with Mad City Labs C-Focus.
- For each Z, grabs an image via a mock `snap_image()` function.
- Computes a morphology-based sharpness score using SmartFocusScorer.get_smart_score.
- Learns the Best Plane of Focus (BPoF) as argmax of the scores.

It is intentionally independent from the PyQt GUI so you can run and profile
it from the command line.
"""

import os
import time
import logging
from typing import List, Tuple, Optional

import numpy as np
import cv2

from hardware.cfocus.cfocus_controller import CFocusController
from core.autofocus.smart_focus_scorer import SmartFocusScorer


logger = logging.getLogger("MotorControl_L206")


class _SimulatedCFocus:
    """Simple simulation of C-Focus when the DLL is not available.

    This allows you to test the calibration loop speed and scoring logic
    even on machines without the Madlib.dll driver or without hardware
    connected.
    """

    def __init__(self, z_range: float = 100.0):
        self._z_range = float(z_range)
        self._position = 0.0

    def move_z(self, position_um: float) -> bool:
        if position_um < 0 or position_um > self._z_range:
            return False
        self._position = float(position_um)
        return True

    def read_z(self) -> float:
        return self._position

    def get_z_range(self) -> float:
        return self._z_range

    def disconnect(self) -> None:  # pragma: no cover - no-op
        return None


def snap_image(image_path: Optional[str] = None) -> np.ndarray:
    """Mock image acquisition.

    - If `image_path` is provided and exists, loads that image from disk.
    - Otherwise, generates a synthetic noisy image (Gaussian noise + blur).
    """

    if image_path is not None and os.path.isfile(image_path):
        img = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if img is not None:
            return img

    # Fallback: synthetic noisy image
    h, w = 512, 512
    noise = np.random.normal(loc=127, scale=30, size=(h, w)).astype(np.uint8)
    img = cv2.GaussianBlur(noise, (5, 5), 0)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img


def calibration_routine(
    z_min: float = 0.0,
    z_max: float = 100.0,
    z_step: float = 5.0,
    image_path: Optional[str] = None,
) -> Tuple[np.ndarray, np.ndarray, float]:
    """Runs a Z-scan calibration using morphology-based Smart Score.

    Args:
        z_min: Minimum Z in microns.
        z_max: Maximum Z in microns.
        z_step: Step in microns.
        image_path: Optional path to a test image. If None or invalid,
            synthetic images are used.

    Returns:
        tuple: (z_positions, scores, best_z)
    """

    # --- 1. Initialize C-Focus (with simulation fallback) ---
    real_cfocus: Optional[CFocusController] = None
    cfocus: object

    try:
        real_cfocus = CFocusController()
        success, msg = real_cfocus.connect()
        if not success:
            logger.warning(
                "[Calibration] No se pudo conectar C-Focus (%s). Usando simulación.",
                msg,
            )
            cfocus = _SimulatedCFocus(z_range=z_max)
        else:
            cfocus = real_cfocus
            # Si el rango real es menor que z_max, adaptarlo
            z_max = min(z_max, cfocus.get_z_range())
    except Exception as e:
        logger.warning(
            "[Calibration] Error cargando DLL Madlib o conectando C-Focus (%s). "
            "Posible conflicto 32/64 bits. Usando simulación.",
            e,
        )
        cfocus = _SimulatedCFocus(z_range=z_max)

    # --- 2. Initialize SmartFocusScorer (MorphologyFocus) ---
    scorer = SmartFocusScorer()
    scorer.load_model()  # Lazy: intentará cargar U2-Net si está disponible

    z_positions: List[float] = []
    scores: List[float] = []

    try:
        z = float(z_min)
        while z <= z_max + 1e-6:
            # Move Stage -> Wait 50ms
            cfocus.move_z(z)
            time.sleep(0.05)

            # Snap Image (mock) and compute Smart Score
            img = snap_image(image_path)
            score, mask = scorer.get_smart_score(img)

            z_positions.append(z)
            scores.append(score)

            logger.info("[Calibration] Z=%.1f µm → SmartScore=%.2f", z, score)

            z += z_step

        if not z_positions:
            logger.error("[Calibration] No se evaluó ninguna posición Z")
            return np.array([]), np.array([]), 0.0

        z_arr = np.array(z_positions, dtype=float)
        s_arr = np.array(scores, dtype=float)

        best_idx = int(np.argmax(s_arr))
        best_z = float(z_arr[best_idx])
        best_score = float(s_arr[best_idx])

        print(
            f"Morphology Characterized. Best Focus Plane: Z = {best_z:.2f} µm "
            f"(Score={best_score:.2f})"
        )

        return z_arr, s_arr, best_z

    finally:
        # Safety: always release real hardware handle if it was used
        if isinstance(cfocus, CFocusController) and cfocus.is_connected:
            try:
                cfocus.disconnect()
            except Exception:
                pass


if __name__ == "__main__":
    # Example execution: simple 0–100 µm scan in 5 µm steps
    calibration_routine()

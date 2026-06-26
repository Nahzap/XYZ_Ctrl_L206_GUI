"""
Métrica unificada de enfoque (índice S) para todo el sistema.

Única implementación usada por AutofocusService, SmartFocusScorer,
overlay de cámara y microscopía automatizada.
"""

from typing import Optional, Tuple

import cv2
import numpy as np

DEFAULT_ROI_MARGIN_PX = 20


def bbox_to_contour(bbox: Tuple[int, int, int, int]) -> np.ndarray:
    """Genera contorno rectangular a partir de un bbox cuando no hay máscara U2-Net."""
    x, y, w, h = bbox
    return np.array(
        [[[x, y]], [[x + w, y]], [[x + w, y + h]], [[x, y + h]]],
        dtype=np.int32,
    )


def calculate_focus_score(
    frame: np.ndarray,
    bbox: Tuple[int, int, int, int],
    contour: Optional[np.ndarray] = None,
    roi_margin: int = DEFAULT_ROI_MARGIN_PX,
) -> float:
    """
    Calcula el índice S sobre ROI cuadrado expandido alrededor del objeto.

    Métrica combinada: 25% var(Laplacian) + 50% Tenengrad + 25% var normalizada.
    Si hay contorno, las métricas se calculan solo sobre la máscara del objeto.
    """
    if contour is None:
        contour = bbox_to_contour(bbox)

    x, y, w, h = bbox
    h_frame, w_frame = frame.shape[:2]

    side = max(w, h)
    center_x = x + w // 2
    center_y = y + h // 2
    x_square = center_x - side // 2
    y_square = center_y - side // 2

    x_expanded = max(0, x_square - roi_margin)
    y_expanded = max(0, y_square - roi_margin)
    side_expanded = side + 2 * roi_margin

    w_expanded = min(side_expanded, w_frame - x_expanded)
    h_expanded = min(side_expanded, h_frame - y_expanded)

    if w_expanded <= 0 or h_expanded <= 0:
        return 0.0

    roi = frame[y_expanded:y_expanded + h_expanded, x_expanded:x_expanded + w_expanded]

    if len(roi.shape) == 3:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    else:
        gray = roi.copy()

    if gray.dtype == np.uint16:
        gray = (gray / 256).astype(np.uint8)

    mask = np.zeros((h_expanded, w_expanded), dtype=np.uint8)
    if contour is not None and len(contour) > 0:
        contour_shifted = contour.copy()
        contour_shifted[:, :, 0] -= x_expanded
        contour_shifted[:, :, 1] -= y_expanded
        cv2.drawContours(mask, [contour_shifted], -1, 255, -1)

    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    laplacian = cv2.Laplacian(gray, cv2.CV_64F, ksize=5)
    gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    gradient_mag = gx ** 2 + gy ** 2

    if np.count_nonzero(mask) > 0:
        lap_values = laplacian[mask > 0]
        grad_values = gradient_mag[mask > 0]
        gray_values = gray[mask > 0]
        lap_var = lap_values.var() if len(lap_values) > 0 else 0.0
        tenengrad = grad_values.mean() if len(grad_values) > 0 else 0.0
        mean_val = gray_values.mean() if len(gray_values) > 0 else 0.0
        norm_var = gray_values.var() / mean_val if mean_val > 0 else 0.0
    else:
        lap_var = laplacian.var()
        tenengrad = gradient_mag.mean()
        mean_val = gray.mean()
        norm_var = gray.var() / mean_val if mean_val > 0 else 0.0

    combined = float((lap_var * 0.25) + (tenengrad * 0.50) + (norm_var * 0.25))
    return combined


def calculate_focus_score_detailed(
    frame: np.ndarray,
    bbox: Tuple[int, int, int, int],
    contour: Optional[np.ndarray] = None,
    roi_margin: int = DEFAULT_ROI_MARGIN_PX,
) -> Tuple[float, dict]:
    """Igual que calculate_focus_score pero retorna desglose para logs DEBUG."""
    if contour is None:
        contour = bbox_to_contour(bbox)

    x, y, w, h = bbox
    h_frame, w_frame = frame.shape[:2]
    side = max(w, h)
    center_x = x + w // 2
    center_y = y + h // 2
    x_expanded = max(0, center_x - side // 2 - roi_margin)
    y_expanded = max(0, center_y - side // 2 - roi_margin)
    side_expanded = side + 2 * roi_margin
    w_expanded = min(side_expanded, w_frame - x_expanded)
    h_expanded = min(side_expanded, h_frame - y_expanded)

    score = calculate_focus_score(frame, bbox, contour, roi_margin)
    details = {
        "bbox": (x, y, w, h),
        "side": side,
        "roi_w": w_expanded,
        "roi_h": h_expanded,
        "score": score,
    }
    return score, details


def build_multifocal_z_positions(
    best_z: float,
    n_captures: int,
    capture_step: float,
    z_min: float,
    z_max: float,
) -> list:
    """
    Posiciones Z para captura multi-focal con BPoF en el centro.

    n=3 → [best_z - step, best_z, best_z + step]
    """
    if n_captures % 2 == 0:
        n_captures += 1

    positions = []
    for i in range(n_captures):
        offset = (i - n_captures // 2) * capture_step
        z_capture = max(z_min, min(z_max, best_z + offset))
        positions.append(z_capture)
    return positions

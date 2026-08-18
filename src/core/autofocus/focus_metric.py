"""Índice S unificado de enfoque dentro de la ROI/contorno.

Pipeline:
  ROI fija + máscara interior → CLAHE → operadores de alta frecuencia
  → fusión Tenengrad/Laplacian/Brenner/DoG.

La máscara se erosiona ligeramente para que el borde de segmentación no gane
al detalle interno de la semilla. CLAHE normaliza iluminación local, pero no
forma parte de la detección: se aplica nuevamente sobre cada plano Z medido.
"""

from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np

MAX_FOCUS_CONTEXT_MARGIN_PX = 16

# Erosión del contorno antes de medir S. Se expresa como fracción del radio
# inscrito del objeto para que escale con su grosor real.
_INTERIOR_EROSION_FRAC = 0.30
_INTERIOR_EROSION_MIN_PX = 3.0
_INTERIOR_EROSION_MAX_PX = 32.0

# Parámetros de la fusión RAW/CLAHE. Se exponen porque son los que gobiernan la
# repetibilidad de S: el 90% del índice sale del promedio del 0.5% de gradientes
# más fuertes, así que unos pocos píxeles (polvo, un borde residual, flicker de
# iluminación) mueven S decenas de puntos entre dos tomas del mismo plano. En el
# log de referencia eso produjo 362.4 y 280.0 en el mismo Z (−29.7%).
#
# Bajar el percentil o repartir peso hacia la rama CLAHE reduce esa fragilidad,
# pero cambia la escala de S y con ella el umbral ΔS del stack. No se puede
# ajustar a ciegas: exige el banco de repeticiones sobre la misma semilla
# (5 barridos, CV de S en el mismo Z ≤ 5%). Hasta entonces los valores por
# defecto reproducen exactamente CLAHE-HF-v4.
STRONG_EDGE_PERCENTILE = 99.5
RAW_BRANCH_WEIGHT = 0.90
CLAHE_BRANCH_WEIGHT = 0.10
CLAHE_CLIP_LIMIT = 2.0
CLAHE_TILE_GRID = (8, 8)


def bbox_to_contour(bbox: Tuple[int, int, int, int]) -> np.ndarray:
    """Contorno rectangular cuando no hay máscara U2-Net."""
    x, y, w, h = bbox
    return np.array(
        [[[x, y]], [[x + w, y]], [[x + w, y + h]], [[x, y + h]]],
        dtype=np.int32,
    )


def _roi_window(
    frame_shape: Tuple[int, ...],
    bbox: Tuple[int, int, int, int],
    roi_margin: int,
) -> Tuple[int, int, int, int, int]:
    """Retorna (x0, y0, w_roi, h_roi, side) del ROI cuadrado + margen."""
    x, y, w, h = bbox
    h_frame, w_frame = int(frame_shape[0]), int(frame_shape[1])
    side = max(int(w), int(h))
    center_x = int(x) + int(w) // 2
    center_y = int(y) + int(h) // 2
    margin = max(0, int(roi_margin))
    x0 = max(0, center_x - side // 2 - margin)
    y0 = max(0, center_y - side // 2 - margin)
    side_exp = side + 2 * margin
    w_roi = min(side_exp, w_frame - x0)
    h_roi = min(side_exp, h_frame - y0)
    return x0, y0, w_roi, h_roi, side


def _object_mask(
    h_roi: int,
    w_roi: int,
    contour: np.ndarray,
    x0: int,
    y0: int,
) -> np.ndarray:
    mask = np.zeros((h_roi, w_roi), dtype=np.uint8)
    if contour is None or len(contour) == 0:
        return mask
    shifted = np.asarray(contour).copy()
    shifted[:, :, 0] -= x0
    shifted[:, :, 1] -= y0
    cv2.drawContours(mask, [shifted], -1, 255, -1)
    return mask


def _to_gray_native(roi: np.ndarray) -> np.ndarray:
    """Gris uint8/uint16 sin reducir la profundidad científica."""
    if roi.ndim == 3:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    else:
        gray = roi.copy()

    if gray.dtype == np.uint8:
        return gray
    if gray.dtype == np.uint16:
        return gray
    if np.issubdtype(gray.dtype, np.floating):
        finite = np.nan_to_num(gray, nan=0.0, posinf=255.0, neginf=0.0)
        if finite.size and float(np.max(finite)) <= 1.0:
            finite = finite * 255.0
        return np.clip(finite, 0.0, 255.0).astype(np.uint8)
    return np.clip(gray, 0, 255).astype(np.uint8)


def _interior_mask(mask: np.ndarray) -> np.ndarray:
    """Deja solo el interior del objeto, sin su silueta ni la orla de desenfoque.

    El salto de intensidad objeto/fondo es de lejos el gradiente más fuerte del
    recorte y apenas cambia con Z, así que si entra en la medida S deja de
    responder al detalle interno: pasa a puntuar lo bien que el contorno abraza
    la silueta. Por eso el radio se toma proporcional al grosor del propio
    objeto (vía transformada de distancia) y no al tamaño de la ventana ROI.

    El radio se reduce si la erosión dejara demasiadas pocas muestras, de modo
    que los objetos pequeños siguen midiéndose.
    """
    n = int(np.count_nonzero(mask))
    if n < 25:
        return mask

    dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    reach = float(dist.max())  # radio del mayor círculo inscrito
    if reach <= 1.0:
        return mask

    radius = float(np.clip(_INTERIOR_EROSION_FRAC * reach,
                           _INTERIOR_EROSION_MIN_PX,
                           _INTERIOR_EROSION_MAX_PX))
    min_keep = max(150, int(0.25 * n))
    inner = (dist >= radius).astype(np.uint8) * 255
    while int(np.count_nonzero(inner)) < min_keep and radius > 1.0:
        radius = max(1.0, radius - 1.0)
        inner = (dist >= radius).astype(np.uint8) * 255

    if int(np.count_nonzero(inner)) < 25:
        return mask
    return inner


def _masked_brenner(gray_f: np.ndarray, mask_bool: np.ndarray) -> float:
    """Energía Brenner a 2 px, usando solo pares dentro del contorno."""
    energies = []
    if gray_f.shape[1] > 2:
        valid_h = mask_bool[:, 2:] & mask_bool[:, :-2]
        if np.any(valid_h):
            dh = gray_f[:, 2:] - gray_f[:, :-2]
            energies.append(np.square(dh[valid_h]))
    if gray_f.shape[0] > 2:
        valid_v = mask_bool[2:, :] & mask_bool[:-2, :]
        if np.any(valid_v):
            dv = gray_f[2:, :] - gray_f[:-2, :]
            energies.append(np.square(dv[valid_v]))
    if not energies:
        return 0.0
    return float(np.mean(np.concatenate(energies)))


def calculate_focus_score(
    frame: np.ndarray,
    bbox: Tuple[int, int, int, int],
    contour: Optional[np.ndarray] = None,
    roi_margin: int = 0,
    **metric_kwargs,
) -> float:
    """Índice S sobre la máscara del objeto dentro del ROI expandido."""
    score, _ = calculate_focus_score_detailed(
        frame, bbox, contour, roi_margin, **metric_kwargs
    )
    return score


def calculate_focus_score_detailed(
    frame: np.ndarray,
    bbox: Tuple[int, int, int, int],
    contour: Optional[np.ndarray] = None,
    roi_margin: int = 0,
    *,
    strong_edge_percentile: Optional[float] = None,
    raw_weight: Optional[float] = None,
    clahe_clip_limit: Optional[float] = None,
) -> Tuple[float, dict]:
    """Índice S + metadatos (ROI, cobertura de máscara).

    Los tres parámetros ópticos quedan expuestos para el banco de repetibilidad
    (ver constantes del módulo); con ``None`` se usa CLAHE-HF-v4 tal cual.
    """
    percentile = (
        STRONG_EDGE_PERCENTILE
        if strong_edge_percentile is None
        else float(np.clip(float(strong_edge_percentile), 50.0, 100.0))
    )
    w_raw = (
        RAW_BRANCH_WEIGHT
        if raw_weight is None
        else float(np.clip(float(raw_weight), 0.0, 1.0))
    )
    w_clahe = 1.0 - w_raw
    clip_limit = (
        CLAHE_CLIP_LIMIT
        if clahe_clip_limit is None
        else max(0.1, float(clahe_clip_limit))
    )

    if frame is None or getattr(frame, "size", 0) == 0:
        return 0.0, {"score": 0.0, "mask_pixels": 0, "roi_w": 0, "roi_h": 0}

    if contour is None:
        contour = bbox_to_contour(bbox)

    x, y, w, h = bbox
    requested_margin = max(0, int(roi_margin))
    effective_margin = min(requested_margin, MAX_FOCUS_CONTEXT_MARGIN_PX)
    x0, y0, w_roi, h_roi, side = _roi_window(
        frame.shape, bbox, effective_margin
    )
    details = {
        "bbox": (int(x), int(y), int(w), int(h)),
        "frame_w": int(frame.shape[1]),
        "frame_h": int(frame.shape[0]),
        "spatial_scale": 1.0,
        "resized": False,
        "side": side,
        "roi_w": w_roi,
        "roi_h": h_roi,
        "roi_margin": effective_margin,
        "roi_margin_requested": requested_margin,
        "roi_margin_effective": effective_margin,
        "roi_origin": (x0, y0),
        "mask_pixels": 0,
        "score": 0.0,
    }
    if w_roi <= 0 or h_roi <= 0:
        return 0.0, details

    roi = frame[y0:y0 + h_roi, x0:x0 + w_roi]
    gray = _to_gray_native(roi)
    storage_bits = int(gray.dtype.itemsize * 8)
    signal_bits = storage_bits
    if gray.dtype == np.uint16 and gray.size and int(np.max(gray)) <= 4095:
        signal_bits = 12
    details["input_storage_bits"] = storage_bits
    details["input_signal_bits"] = signal_bits
    details["compute_dtype"] = "float64"
    details["compute_mantissa_bits"] = 53
    gray_scoring = (
        np.left_shift(gray, 4)
        if gray.dtype == np.uint16 and signal_bits == 12
        else gray
    )
    # Trabajar en una escala equivalente de 12 bits. RAW12 se mantiene
    # íntegro; el shift solo ocupa correctamente el contenedor uint16.
    working_scale = 16.0 if storage_bits >= 16 else 1.0

    mask = _object_mask(h_roi, w_roi, contour, x0, y0)
    mask_pixels = int(np.count_nonzero(mask))
    details["mask_pixels"] = mask_pixels

    if mask_pixels <= 0:
        details["score"] = 0.0
        return 0.0, details

    inner = _interior_mask(mask)
    inner_bool = inner > 0
    inner_pixels = int(np.count_nonzero(inner_bool))
    details["inner_mask_pixels"] = inner_pixels
    if inner_pixels < 25:
        details["score"] = 0.0
        return 0.0, details

    # Aislar el histograma CLAHE del fondo: un patrón fuera del contorno no debe
    # cambiar S. El borde queda fuera gracias a ``inner_bool``.
    clahe_input = gray_scoring.copy()
    object_values = gray_scoring[mask > 0]
    fill = int(np.median(object_values)) if object_values.size else 0
    clahe_input[mask == 0] = fill

    # CLAHE sí forma parte de S: normaliza iluminación local sin saturar contraste.
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=CLAHE_TILE_GRID)
    enhanced = clahe.apply(clahe_input)
    enhanced_f = enhanced.astype(np.float64) / working_scale

    # Rama RAW robusta: un suavizado óptico de sigma=1 elimina ruido de píxel,
    # y el promedio del 0.5% de gradientes más fuertes sigue los detalles del
    # objeto. El promedio global anterior quedaba dominado por ruido de fondo
    # y hacía casi plana la curva S aun con desenfoque real.
    raw_f = gray_scoring.astype(np.float64) / working_scale
    raw_smooth = cv2.GaussianBlur(raw_f, (0, 0), sigmaX=1.0, sigmaY=1.0)
    raw_gx = cv2.Sobel(raw_smooth, cv2.CV_64F, 1, 0, ksize=3)
    raw_gy = cv2.Sobel(raw_smooth, cv2.CV_64F, 0, 1, ksize=3)
    raw_grad = np.hypot(raw_gx, raw_gy)
    raw_grad_values = raw_grad[inner_bool]
    if raw_grad_values.size:
        edge_threshold = float(
            np.percentile(raw_grad_values, percentile)
        )
        strong_values = raw_grad_values[raw_grad_values >= edge_threshold]
        strong_edge_score = (
            float(np.mean(strong_values)) if strong_values.size else 0.0
        )
    else:
        edge_threshold = 0.0
        strong_edge_score = 0.0
    raw_object_values = raw_f[mask > 0]
    nominal_level = 4095.0 if storage_bits >= 16 else 255.0
    intensity_reference = max(
        float(np.median(raw_object_values)) if raw_object_values.size else 0.0,
        0.10 * nominal_level,
    )
    strong_edge_normalized = (
        strong_edge_score / intensity_reference * 1000.0
    )

    # Rama CLAHE complementaria para semillas con iluminación no uniforme.
    gx = cv2.Sobel(enhanced_f, cv2.CV_64F, 1, 0, ksize=3)
    gy = cv2.Sobel(enhanced_f, cv2.CV_64F, 0, 1, ksize=3)
    grad2 = np.square(gx) + np.square(gy)
    laplacian = cv2.Laplacian(enhanced_f, cv2.CV_64F, ksize=3)
    low = cv2.GaussianBlur(enhanced_f, (0, 0), sigmaX=1.2, sigmaY=1.2)
    highpass = enhanced_f - low

    lap_values = laplacian[inner_bool]
    grad_values = grad2[inner_bool]
    hp_values = highpass[inner_bool]
    lap_var = float(np.var(lap_values)) if lap_values.size else 0.0
    tenengrad = float(np.mean(grad_values)) if grad_values.size else 0.0
    brenner = _masked_brenner(enhanced_f, inner_bool)
    highpass_var = float(np.var(hp_values)) if hp_values.size else 0.0

    # Raíz de energías → componentes comparables y menor sensibilidad a outliers.
    ten_s = float(np.sqrt(max(0.0, tenengrad)))
    lap_s = float(np.sqrt(max(0.0, lap_var)))
    brenner_s = float(np.sqrt(max(0.0, brenner)))
    highpass_s = float(np.sqrt(max(0.0, highpass_var)))
    clahe_score = float(
        0.45 * ten_s
        + 0.25 * lap_s
        + 0.20 * brenner_s
        + 0.10 * highpass_s
    )
    score = float(w_raw * strong_edge_normalized + w_clahe * clahe_score)

    details["score"] = score
    details["metric_version"] = (
        "clahe_hf_v4_raw16" if storage_bits >= 16 else "clahe_hf_v4_u8"
    )
    details["clahe_clip_limit"] = clip_limit
    details["lap_var"] = lap_var
    details["tenengrad"] = tenengrad
    details["brenner"] = brenner
    details["highpass_var"] = highpass_var
    details["tenengrad_sqrt"] = ten_s
    details["lap_sqrt"] = lap_s
    details["brenner_sqrt"] = brenner_s
    details["highpass_sqrt"] = highpass_s
    details["strong_edge_percentile"] = percentile
    details["strong_edge_threshold"] = edge_threshold
    details["strong_edge_score"] = strong_edge_score
    details["strong_edge_normalized"] = strong_edge_normalized
    details["intensity_reference"] = intensity_reference
    details["clahe_composite_score"] = clahe_score
    details["raw_weight"] = w_raw
    details["clahe_weight"] = w_clahe
    return score, details


def multifocal_settle_s(
    delta_z_um: float,
    capture_settle_s: float,
    *,
    min_s: float = 0.10,
    s_per_um: float = 0.015,
) -> float:
    """Settle para un salto multi-focal (no usar el settle corto del scan).

    El settle de escaneo (~20 ms) no alcanza para ±10 µm de piezo.
    """
    base = max(0.0, float(capture_settle_s))
    by_dist = max(0.0, float(s_per_um)) * abs(float(delta_z_um))
    return max(base, float(min_s), by_dist)


def build_multifocal_z_positions(
    best_z: float,
    n_captures: int,
    capture_step: float,
    z_min: float,
    z_max: float,
) -> list:
    """Plan estricto de N posiciones Z con BPoF en el centro.

    No clampa planos laterales porque eso crea distancias repetidas. Si el
    stack completo no cabe en hardware, no existe un plan fotográfico válido.
    """
    n = int(n_captures)
    if n < 1 or n % 2 == 0:
        raise ValueError(f"n_captures debe ser impar ≥1 (recibido: {n_captures})")

    step = float(capture_step)
    if step <= 0:
        raise ValueError(f"capture_step debe ser > 0 (recibido: {capture_step})")

    z_lo = float(z_min)
    z_hi = float(z_max)
    if z_hi < z_lo:
        z_lo, z_hi = z_hi, z_lo

    bpof = float(best_z)
    radius = float(n // 2) * step
    if bpof - radius < z_lo - 1e-9 or bpof + radius > z_hi + 1e-9:
        raise ValueError(
            "Stack multi-focal no cabe alrededor del BPoF: "
            f"BPoF={bpof:.3f}µm, N={n}, paso={step:.3f}µm, "
            f"requerido=[{bpof - radius:.3f},{bpof + radius:.3f}]µm, "
            f"hardware=[{z_lo:.3f},{z_hi:.3f}]µm"
        )

    return [bpof + (i - n // 2) * step for i in range(n)]

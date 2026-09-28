"""Geometría de bring-to-center XY (píxel → micra, histéresis, límites).

Sin homografía: la escala es FOV declarado / tamaño del frame científico.
``pixel_size_um`` del sensor no entra aquí.

El jog vive en microscopía (post-detección / pre-AF). Este módulo es puro
para poder testearlo sin hardware.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Sequence, Tuple

# inner_pixels < 25 ⇒ S = 0 en focus_metric. Referencia de ROI usable.
MIN_INNER_PIXELS = 25

DEFAULT_CENTER_ENABLED = True
DEFAULT_HYSTERESIS_UM = 12.0
DEFAULT_HYSTERESIS_PX = 40.0
# |Δpx| por encima de esto (o ROI clipado) dispara un jog best-effort.
# NUNCA es un gate del Z-scan: objeto en FOV → AF aunque |Δpx| >> 40.
FORCE_CENTER_PX = 40.0
# In-range (roi_clip=0): 1 paso opcional, luego AF. Clip/borde: 3–4, luego AF.
IN_RANGE_MAX_CENTER_STEPS = 1
CLIP_MAX_CENTER_STEPS = 4
# Legado (JSON / callers); el lazo usa IN_RANGE / CLIP, no 8.
DEFAULT_MAX_CENTER_STEPS = CLIP_MAX_CENTER_STEPS
DEFAULT_MAX_RETRIES = DEFAULT_MAX_CENTER_STEPS
# Fracción del Δ restante por paso (servo suave; 0.6–0.8).
DEFAULT_STEP_GAIN = 0.7
# Primer paso si |Δpx| > LARGE_OFFSET_PX.
LARGE_OFFSET_PX = 400.0
LARGE_OFFSET_GAIN = 0.5
# Tras cruzar 0 en un eje: gain de ese eje × 0.4 (no 0.7 fijo).
OVERSHOOT_GAIN_SCALE = 0.4
# Techo de TODO el lazo (no por paso). Distinto del FOV 2.5 s.
LOOP_SAFETY_TIMEOUT_S = 75.0
MOTION_EPS_UM = 2.0
# Confirmación de que el stage se movió (log + retry dual si no).
STEP_MOVED_CONFIRM_UM = 10.0
# Primer paso con ROI recortado: 25–40 % del FOV, luego re-medir.
CLIP_STEP_FOV_FRAC = 0.32
# Cap de |Δcmd| por eje en cada paso (40 % FOV ≈ 50–80 µm). Nunca un goto
# al punto viejo de malla (p.ej. 1120 µm). Se reaplica DESPUÉS del workspace.
STEP_CMD_FOV_FRAC = 0.40
# Reintentos de detect tras cada paso. Fallo de grab ≠ lost_lock.
LOCK_REDETECT_TRIES = 2
LOCK_MISS_LIMIT = 2
# Timeout del PASO (ese Δcmd): 1–3 s o distancia/velocidad. El lazo sigue.
STEP_TIMEOUT_MIN_S = 1.0
STEP_TIMEOUT_MAX_S = 3.0
STEP_TIMEOUT_SPEED_UM_S = 50.0
STEP_TIMEOUT_SLACK_S = 0.4
# Identidad del candidato: ±50 % área; caída >4× = lost (debris).
LOCK_AREA_TOL_FRAC = 0.50
LOCK_AREA_LOST_RATIO = 4.0
LOCK_MIN_IOU = 0.15
# 0 = sin cap extra; solo clamp al workspace de trayectoria.
DEFAULT_MAX_DELTA_UM = 0.0
# Δpx = centro_imagen − centroide (hacia el centro). dx = sign · Δpx · fov/W.
# Con la definición vieja e=cx−W/2 el hardware 2026-08-18 usaba sign=(−1,−1)
# y left+top daba dxy=(+127,+85). Al invertir Δ hay que usar sign=+1 para
# el mismo comando de stage (no flip-flop).
DEFAULT_SIGN_X = 1
DEFAULT_SIGN_Y = 1

HARD_STEP_CAP_UM = 35.0  # legado; el lazo usa STEP_GAIN / CLIP_STEP_FOV_FRAC.
STEP_FOV_FRAC = 0.20
CLIP_EDGE_EPS_PX = 2.0
SIGN_FLIP_EPS_PX = 1.0


def object_centroid_px(obj) -> Optional[Tuple[float, float]]:
    """Centroide del candidato; fallback al centro del bbox."""
    centroid = getattr(obj, "centroid", None)
    if centroid is not None and len(centroid) >= 2:
        return float(centroid[0]), float(centroid[1])
    bbox = getattr(obj, "bounding_box", None) or getattr(obj, "bbox", None)
    if bbox is None or len(bbox) < 4:
        return None
    x, y, w, h = bbox
    return float(x) + float(w) / 2.0, float(y) + float(h) / 2.0


def object_bbox(obj) -> Optional[Tuple[float, float, float, float]]:
    bbox = getattr(obj, "bounding_box", None) or getattr(obj, "bbox", None)
    if bbox is None or len(bbox) < 4:
        return None
    return float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])


def object_area(obj) -> float:
    area = float(getattr(obj, "area", 0) or 0)
    if area > 0.0:
        return area
    bbox = object_bbox(obj)
    if bbox is None:
        return 0.0
    return max(0.0, float(bbox[2]) * float(bbox[3]))


def image_center_px(frame_w: float, frame_h: float) -> Tuple[float, float]:
    return float(frame_w) / 2.0, float(frame_h) / 2.0


def pixel_offset_from_center(
    cx: float,
    cy: float,
    frame_w: float,
    frame_h: float,
) -> Tuple[float, float, float]:
    """Δpx hacia el centro de la imagen: (Cix − cx, Ciy − cy, |Δ|).

    Objeto recortado left/top → centroide visible a la izquierda/arriba →
    Δpx_x>0 y Δpx_y>0 (el grano debe ir hacia la derecha y abajo EN LA IMAGEN).
    """
    cix, ciy = image_center_px(frame_w, frame_h)
    dpx_x = cix - float(cx)
    dpx_y = ciy - float(cy)
    dpx = (dpx_x * dpx_x + dpx_y * dpx_y) ** 0.5
    return dpx_x, dpx_y, dpx


def clip_edges(
    bbox: Sequence[float],
    frame_w: float,
    frame_h: float,
    eps_px: float = CLIP_EDGE_EPS_PX,
) -> Tuple[bool, bool, bool, bool]:
    """True si el bbox toca cada borde: left, top, right, bottom."""
    x, y, w, h = (float(v) for v in bbox[:4])
    eps = max(0.0, float(eps_px))
    fw, fh = float(frame_w), float(frame_h)
    left = x <= eps
    top = y <= eps
    right = (x + w) >= (fw - eps)
    bottom = (y + h) >= (fh - eps)
    return left, top, right, bottom


def format_clip_edges(left: bool, top: bool, right: bool, bottom: bool) -> str:
    names = []
    if left:
        names.append("left")
    if top:
        names.append("top")
    if right:
        names.append("right")
    if bottom:
        names.append("bottom")
    return ",".join(names)


def direction_error_px(
    cx: float,
    cy: float,
    bbox: Sequence[float],
    frame_w: float,
    frame_h: float,
) -> Tuple[float, float, float]:
    """Error de dirección hacia el centro geométrico del frame.

    Si el bbox recorta un borde, el centroide de la silueta visible está
    sesgado: se fuerza el signo hacia el interior (p.ej. clip superior →
    e_y < 0, objeto arriba del centro). La magnitud se recorta luego.
    """
    e_x, e_y, e_px = pixel_offset_from_center(cx, cy, frame_w, frame_h)
    del e_px
    left, top, right, bottom = clip_edges(bbox, frame_w, frame_h)
    # Clip: el centroide visible ya apunta al interior; fuerza el signo + hacia
    # el centro de imagen (left→+x, top→+y).
    if top and not bottom:
        e_y = abs(e_y) if abs(e_y) > 1e-6 else 1.0
    elif bottom and not top:
        e_y = -abs(e_y) if abs(e_y) > 1e-6 else -1.0
    if left and not right:
        e_x = abs(e_x) if abs(e_x) > 1e-6 else 1.0
    elif right and not left:
        e_x = -abs(e_x) if abs(e_x) > 1e-6 else -1.0
    return e_x, e_y, hypot(e_x, e_y)


def pixel_offset_to_stage_um(
    e_x_px: float,
    e_y_px: float,
    frame_w: float,
    frame_h: float,
    fov_x_um: float,
    fov_y_um: float,
    *,
    sign_x: int = DEFAULT_SIGN_X,
    sign_y: int = DEFAULT_SIGN_Y,
) -> Tuple[float, float]:
    """Δx, Δy en µm de stage. Escala = fov / resolución; no pixel_size_um.

    ``e`` es Δpx hacia el centro de imagen (Cix−cx, Ciy−cy).
    ``dx = sign_x · Δpx_x · fov_x/fw``  (imagen +x = derecha)
    ``dy = sign_y · Δpx_y · fov_y/fh``  (imagen +y = abajo)

    Default ``sign_x = sign_y = +1``: objeto a la IZQUIERDA (Δpx_x>0) pide
    ΔX stage > 0 con la convención que en banco 2026-08-18 metió el grano
    hacia abajo-derecha EN LA IMAGEN. ``sign_x/y`` ∈ {+1, −1}.
    """
    if frame_w <= 0 or frame_h <= 0:
        return 0.0, 0.0
    sx = 1 if int(sign_x) >= 0 else -1
    sy = 1 if int(sign_y) >= 0 else -1
    dx_um = sx * float(e_x_px) * (float(fov_x_um) / float(frame_w))
    dy_um = sy * float(e_y_px) * (float(fov_y_um) / float(frame_h))
    return dx_um, dy_um


def hypot(a: float, b: float) -> float:
    return (float(a) * float(a) + float(b) * float(b)) ** 0.5


def axis_cmd_cap_um(
    fov_axis_um: float,
    max_delta_um: float = 0.0,
    *,
    frac: float = STEP_CMD_FOV_FRAC,
) -> float:
    """Techo de |Δcmd| por eje: 40 % FOV, opcionalmente recortado más."""
    if float(fov_axis_um) <= 0.0:
        return 0.0
    cap = max(0.0, float(frac) * float(fov_axis_um))
    if float(max_delta_um) > 0.0:
        cap = min(cap, float(max_delta_um))
    return cap


def cap_step_command(
    dx_um: float,
    dy_um: float,
    fov_x_um: float,
    fov_y_um: float,
    *,
    max_delta_um: float = 0.0,
    frac: float = STEP_CMD_FOV_FRAC,
) -> Tuple[float, float]:
    """``xy_tgt = xy_now + cap(Δ)``. El cap no se puede saltar con workspace."""
    cap_x = axis_cmd_cap_um(fov_x_um, max_delta_um, frac=frac)
    cap_y = axis_cmd_cap_um(fov_y_um, max_delta_um, frac=frac)
    dx = float(dx_um)
    dy = float(dy_um)
    if cap_x > 0.0:
        dx = max(-cap_x, min(cap_x, dx))
    else:
        dx = 0.0
    if cap_y > 0.0:
        dy = max(-cap_y, min(cap_y, dy))
    else:
        dy = 0.0
    return dx, dy


def workspace_usable(
    x_min_um: Optional[float],
    x_max_um: Optional[float],
    y_min_um: Optional[float],
    y_max_um: Optional[float],
    *,
    min_span_um: float = 1.0,
) -> bool:
    """False si falta un límite o la caja es un punto (malla 1-pt stale)."""
    if None in (x_min_um, x_max_um, y_min_um, y_max_um):
        return False
    span_x = float(x_max_um) - float(x_min_um)
    span_y = float(y_max_um) - float(y_min_um)
    return span_x > float(min_span_um) or span_y > float(min_span_um)


def stage_inside_workspace(
    stage_x_um: float,
    stage_y_um: float,
    x_min_um: float,
    x_max_um: float,
    y_min_um: float,
    y_max_um: float,
    *,
    slack_um: float = 1.0,
) -> bool:
    slack = max(0.0, float(slack_um))
    return (
        float(x_min_um) - slack <= float(stage_x_um) <= float(x_max_um) + slack
        and float(y_min_um) - slack <= float(stage_y_um) <= float(y_max_um) + slack
    )


def command_dir_label(dx_um: float, dy_um: float, *, eps_um: float = 0.5) -> str:
    """``X+ X- Y+ Y-`` según el signo del Δcmd (no del encoder)."""
    parts = []
    if abs(float(dx_um)) >= float(eps_um):
        parts.append("X+" if float(dx_um) > 0.0 else "X-")
    if abs(float(dy_um)) >= float(eps_um):
        parts.append("Y+" if float(dy_um) > 0.0 else "Y-")
    return " ".join(parts) if parts else "XY0"


def format_center_pwm(
    pwm_a: Optional[int],
    pwm_b: Optional[int],
    *,
    umax: int = 255,
    dx_cmd_um: float = 0.0,
    dy_cmd_um: float = 0.0,
    fov_x_um: float = 0.0,
    fov_y_um: float = 0.0,
) -> str:
    """Duty dual ``(A:45%,B:30%)`` o, si no hay PWM, % del cap FOV + setpoint."""
    ua = max(1, int(umax or 255))
    has_pwm = (
        pwm_a is not None
        and pwm_b is not None
        and (abs(int(pwm_a)) > 0 or abs(int(pwm_b)) > 0)
    )
    if has_pwm:
        pa = 100.0 * abs(int(pwm_a)) / float(ua)
        pb = 100.0 * abs(int(pwm_b)) / float(ua)
        return f"(A:{pa:.0f}%,B:{pb:.0f}%)"
    cap_x = axis_cmd_cap_um(fov_x_um)
    cap_y = axis_cmd_cap_um(fov_y_um)
    px = 100.0 * abs(float(dx_cmd_um)) / cap_x if cap_x > 1e-9 else 0.0
    py = 100.0 * abs(float(dy_cmd_um)) / cap_y if cap_y > 1e-9 else 0.0
    return (
        f"(A:{px:.0f}%FOV,B:{py:.0f}%FOV) "
        f"setpoint=({float(dx_cmd_um):+.1f},{float(dy_cmd_um):+.1f})"
    )


def encoder_sign_error(
    moved_x_um: float,
    moved_y_um: float,
    expected_x_um: float,
    expected_y_um: float,
    *,
    eps_um: float = MOTION_EPS_UM,
) -> Tuple[bool, bool]:
    """True por eje si el stage se movió al revés del Δcmd (run A: X pidió − y fue +)."""
    flip_x = (
        abs(float(expected_x_um)) > float(eps_um)
        and abs(float(moved_x_um)) > float(eps_um)
        and float(expected_x_um) * float(moved_x_um) < 0.0
    )
    flip_y = (
        abs(float(expected_y_um)) > float(eps_um)
        and abs(float(moved_y_um)) > float(eps_um)
        and float(expected_y_um) * float(moved_y_um) < 0.0
    )
    return bool(flip_x), bool(flip_y)


def sign_err_label(flip_x: bool, flip_y: bool) -> str:
    parts = []
    if flip_x:
        parts.append("X")
    if flip_y:
        parts.append("Y")
    return "".join(parts) if parts else "-"


def similar_area_or_iou_match(
    objects: Sequence[Any],
    lock: Optional[IdentityLock],
    *,
    lost_ratio: float = LOCK_AREA_LOST_RATIO,
) -> Any:
    """Candidato visible: área ∈ [lock/4, lock×4] o IoU. No exige ±50 %."""
    objs = list(objects or [])
    if not objs or lock is None:
        return None
    lock_area = float(lock.area or 0.0)
    best = None
    best_score = -1.0
    lo = lock_area / float(lost_ratio) if lock_area > 0.0 else 0.0
    hi = lock_area * float(lost_ratio) if lock_area > 0.0 else float("inf")
    for obj in objs:
        area = object_area(obj)
        iou = bbox_iou(lock.bbox, object_bbox(obj))
        ratio_ok = lo <= area <= hi if lock_area > 0.0 else area > 0.0
        if not ratio_ok and iou < LOCK_MIN_IOU:
            continue
        score = float(iou) * 2.0
        if lock_area > 0.0:
            score += 1.0 - min(1.0, abs(area - lock_area) / lock_area)
        if score > best_score:
            best_score = score
            best = obj
    return best


def should_declare_lost_lock(
    *,
    matched: Any,
    objects: Sequence[Any],
    lock: Optional[IdentityLock],
    consecutive_misses: int,
    miss_limit: int = LOCK_MISS_LIMIT,
) -> bool:
    """lost_lock real: 2 frames sin match Y sin objeto de área ~lock / IoU.

    ``dpx=(0,0)`` por fallo de detect (lista vacía) no es lost.
    """
    if matched is not None:
        return False
    if similar_area_or_iou_match(objects, lock) is not None:
        return False
    if not list(objects or []):
        return False
    return int(consecutive_misses) >= int(miss_limit)


def axis_step_cap_um(fov_axis_um: float, max_delta_um: float = 0.0) -> float:
    """Cap duro por eje: min(35 µm, 20 % FOV), opcionalmente recortado más."""
    if float(fov_axis_um) <= 0.0:
        return 0.0
    cap = min(HARD_STEP_CAP_UM, STEP_FOV_FRAC * float(fov_axis_um))
    if float(max_delta_um) > 0.0:
        cap = min(cap, float(max_delta_um))
    return max(0.0, cap)


def clamp_axis_delta(
    dx_um: float,
    dy_um: float,
    fov_x_um: float,
    fov_y_um: float,
    *,
    max_delta_um: float = 0.0,
    clipped: bool = False,
    need_x: bool = True,
    need_y: bool = True,
) -> Tuple[float, float, bool]:
    """Recorte por eje (no hipotenusa). Clipado: un paso pequeño hacia el interior."""
    cap_x = axis_step_cap_um(fov_x_um, max_delta_um)
    cap_y = axis_step_cap_um(fov_y_um, max_delta_um)
    dx_cmd, dy_cmd = float(dx_um), float(dy_um)
    if not need_x:
        dx_cmd = 0.0
    if not need_y:
        dy_cmd = 0.0
    if clipped:
        if need_x and abs(dx_cmd) > 1e-9 and cap_x > 0.0:
            dx_cmd = cap_x if dx_cmd >= 0.0 else -cap_x
        elif need_x:
            dx_cmd = 0.0
        if need_y and abs(dy_cmd) > 1e-9 and cap_y > 0.0:
            dy_cmd = cap_y if dy_cmd >= 0.0 else -cap_y
        elif need_y:
            dy_cmd = 0.0
    else:
        if cap_x > 0.0:
            dx_cmd = max(-cap_x, min(cap_x, dx_cmd))
        else:
            dx_cmd = 0.0
        if cap_y > 0.0:
            dy_cmd = max(-cap_y, min(cap_y, dy_cmd))
        else:
            dy_cmd = 0.0
    clamped = (abs(dx_cmd - float(dx_um)) > 1e-6) or (abs(dy_cmd - float(dy_um)) > 1e-6)
    return dx_cmd, dy_cmd, clamped


def roi_frame_margin_px(
    bbox: Sequence[float],
    frame_w: float,
    frame_h: float,
) -> float:
    """Holgura mínima bbox→borde del frame (px). Negativo = ya recortado."""
    x, y, w, h = (float(v) for v in bbox[:4])
    return min(x, y, float(frame_w) - (x + w), float(frame_h) - (y + h))


def predict_static_window_clip(
    bbox: Sequence[float],
    frame_w: int,
    frame_h: int,
    pad_px: int,
) -> Tuple[bool, Tuple[int, int, int, int]]:
    """Predice si el cuadrado estático de RoiTracker clampearía al FOV.

    Misma geometría que ``RoiTracker._build_static_windows``: lado =
    max(w,h)+2·pad, luego clamp de (x0,y0) a [0, W−side]×[0, H−side].
    """
    x, y, w, h = (float(v) for v in bbox[:4])
    fw, fh = int(frame_w), int(frame_h)
    pad = max(0, int(pad_px))
    if fw <= 0 or fh <= 0:
        return True, (0, 0, 0, 0)
    side = max(int(w), int(h)) + 2 * pad
    side = min(side, fw, fh)
    cx, cy = x + w / 2.0, y + h / 2.0
    x0_raw = cx - side / 2.0
    y0_raw = cy - side / 2.0
    would_clip = (
        x0_raw < -0.5
        or y0_raw < -0.5
        or (x0_raw + side) > fw + 0.5
        or (y0_raw + side) > fh + 0.5
    )
    x0 = int(round(min(max(0.0, x0_raw), fw - side)))
    y0 = int(round(min(max(0.0, y0_raw), fh - side)))
    return bool(would_clip), (x0, y0, side, side)


def axis_error_worsened(pre: float, post: float, eps: float = SIGN_FLIP_EPS_PX) -> bool:
    return abs(float(post)) > abs(float(pre)) + float(eps)


def axis_crossed_zero(pre: float, post: float, eps: float = SIGN_FLIP_EPS_PX) -> bool:
    """True si el eje atravesó el centro (overshoot de signo)."""
    a, b = float(pre), float(post)
    if abs(a) <= float(eps) or abs(b) <= float(eps):
        return False
    return a * b < 0.0


def apply_overshoot_gains(
    gain_x: float,
    gain_y: float,
    e_x_pre: float,
    e_y_pre: float,
    e_x_post: float,
    e_y_post: float,
    *,
    scale: float = OVERSHOOT_GAIN_SCALE,
) -> Tuple[float, float]:
    """Si Δpx cambia de signo en un eje, gain de ese eje × scale."""
    gx = max(0.05, min(1.0, float(gain_x)))
    gy = max(0.05, min(1.0, float(gain_y)))
    k = max(0.05, min(1.0, float(scale)))
    if axis_crossed_zero(e_x_pre, e_x_post):
        gx = max(0.05, gx * k)
    if axis_crossed_zero(e_y_pre, e_y_post):
        gy = max(0.05, gy * k)
    return gx, gy


def initial_step_gain(e_px: float, base_gain: float = DEFAULT_STEP_GAIN) -> float:
    """Fracción inicial: 0.5 si |Δpx| > 400, si no el gain base."""
    g = max(0.05, min(1.0, float(base_gain)))
    if abs(float(e_px)) > LARGE_OFFSET_PX:
        return min(g, LARGE_OFFSET_GAIN)
    return g


def bbox_iou(a: Optional[Sequence[float]], b: Optional[Sequence[float]]) -> float:
    if a is None or b is None or len(a) < 4 or len(b) < 4:
        return 0.0
    ax, ay, aw, ah = (float(v) for v in a[:4])
    bx, by, bw, bh = (float(v) for v in b[:4])
    ix1, iy1 = max(ax, bx), max(ay, by)
    ix2, iy2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    union = aw * ah + bw * bh - inter
    if union <= 1e-9:
        return 0.0
    return inter / union


@dataclass
class IdentityLock:
    """Candidato del servo en i=0. El área original no se actualiza."""

    bbox: Tuple[float, float, float, float]
    centroid: Optional[Tuple[float, float]]
    area: float
    contour: Any = None


def snapshot_identity_lock(obj) -> Optional[IdentityLock]:
    bbox = object_bbox(obj)
    if bbox is None:
        return None
    return IdentityLock(
        bbox=bbox,
        centroid=object_centroid_px(obj),
        area=object_area(obj),
        contour=getattr(obj, "contour", None),
    )


def update_lock_geometry(lock: IdentityLock, obj) -> IdentityLock:
    """Actualiza bbox/centroide para IoU; conserva el área del lock original."""
    bbox = object_bbox(obj) or lock.bbox
    return IdentityLock(
        bbox=bbox,
        centroid=object_centroid_px(obj) or lock.centroid,
        area=float(lock.area),
        contour=getattr(obj, "contour", None) or lock.contour,
    )


def area_similar(
    area: float,
    lock_area: float,
    tol_frac: float = LOCK_AREA_TOL_FRAC,
) -> bool:
    if float(lock_area) <= 0.0:
        return float(area) > 0.0
    lo = float(lock_area) * (1.0 - float(tol_frac))
    hi = float(lock_area) * (1.0 + float(tol_frac))
    return lo <= float(area) <= hi


def match_locked_object(
    objects: Sequence[Any],
    lock: Optional[IdentityLock],
) -> Any:
    """Elige el lock por IoU / área similar. Nunca el más grande si no matchea.

    Blob 11k con lock 261k → None (área < lock/4), aunque U2-Net vea 1 objeto.
    """
    objs = list(objects or [])
    if not objs or lock is None:
        return None
    lock_area = float(lock.area or 0.0)
    min_area = (
        lock_area / LOCK_AREA_LOST_RATIO if lock_area > 0.0 else 0.0
    )
    best = None
    best_score = -1.0
    for obj in objs:
        area = object_area(obj)
        if lock_area > 0.0 and area < min_area:
            continue
        iou = bbox_iou(lock.bbox, object_bbox(obj))
        similar = area_similar(area, lock_area)
        if iou < LOCK_MIN_IOU and not similar:
            continue
        area_score = 0.0
        if lock_area > 0.0:
            area_score = 1.0 - min(1.0, abs(area - lock_area) / lock_area)
        score = float(iou) * 2.0 + area_score
        if score > best_score:
            best_score = score
            best = obj
    if best is None:
        return similar_area_or_iou_match(objs, lock)
    return best


def pick_tracked_primary(objects: Sequence[Any], previous: Any = None) -> Any:
    """i=0: mayor área. Luego: lock IoU/área. Sin fallback a debris."""
    objs = list(objects or [])
    if not objs:
        return None
    if previous is None:
        return max(objs, key=object_area)
    lock = snapshot_identity_lock(previous)
    return match_locked_object(objs, lock)


def step_goto_timeout_s(dx_cmd_um: float, dy_cmd_um: float) -> float:
    """Timeout de UN paso (ese Δcmd), no del lazo. Clamp 1–3 s."""
    dist = hypot(dx_cmd_um, dy_cmd_um)
    speed = max(1.0, float(STEP_TIMEOUT_SPEED_UM_S))
    t = float(dist) / speed + float(STEP_TIMEOUT_SLACK_S)
    return max(STEP_TIMEOUT_MIN_S, min(STEP_TIMEOUT_MAX_S, t))


@dataclass(frozen=True)
class CenterPlan:
    """Decisión de jog: o se mueve, o se explica por qué no."""

    should_move: bool
    skip_af: bool
    reason: str
    e_x_px: float = 0.0
    e_y_px: float = 0.0
    e_px: float = 0.0
    dx_um: float = 0.0
    dy_um: float = 0.0
    dx_cmd_um: float = 0.0
    dy_cmd_um: float = 0.0
    target_x_um: float = 0.0
    target_y_um: float = 0.0
    would_clip: bool = False
    limit_hit: bool = False
    roi_frame_margin_px: float = 0.0
    delta_clamped: bool = False
    clip_edges: str = ""
    need_x: bool = False
    need_y: bool = False
    img_cx: float = 0.0
    img_cy: float = 0.0
    roi_cx: float = 0.0
    roi_cy: float = 0.0


@dataclass(frozen=True)
class PostJogDecision:
    """Qué hacer tras un paso de centrado (sin Qt)."""

    action: str
    reason: str
    flip_x: bool = False
    flip_y: bool = False


@dataclass
class CenterStepRow:
    """Una fila de la tabla AF_CENTER_STEPS (un SETTLED de trayectoria)."""

    i: int
    x_um: float
    y_um: float
    dpx_x: float
    dpx_y: float
    dpx: float
    ok: str
    xy_cmd: Optional[Tuple[float, float]] = None
    xy_read: Optional[Tuple[float, float]] = None
    residual_um: Optional[Tuple[float, float]] = None
    d_um_x: float = 0.0
    d_um_y: float = 0.0
    dir: str = ""
    pwm: str = ""
    moved_x_um: float = 0.0
    moved_y_um: float = 0.0


def resolve_max_center_steps(max_retries: int, clipped: bool = False) -> int:
    """In-range: 1 paso. Clip/borde: máx 3–4. Luego AF si el lock sigue visible."""
    if not clipped:
        return IN_RANGE_MAX_CENTER_STEPS
    try:
        n = int(max_retries)
    except (TypeError, ValueError):
        n = CLIP_MAX_CENTER_STEPS
    if n <= 0:
        n = CLIP_MAX_CENTER_STEPS
    return max(1, min(CLIP_MAX_CENTER_STEPS, n))


def format_center_steps_table(rows: Sequence[CenterStepRow]) -> str:
    lines = [
        "=== AF_CENTER_STEPS ===",
        " i  xy_cmd            xy_read           dpx          dµm         dir     pwm                    moved      ok",
    ]
    for row in rows:
        cmd = row.xy_cmd or (row.x_um, row.y_um)
        read = row.xy_read or (row.x_um, row.y_um)
        moved = hypot(float(row.moved_x_um), float(row.moved_y_um))
        lines.append(
            f" {int(row.i):<2} "
            f"({float(cmd[0]):.0f},{float(cmd[1]):.0f})  "
            f"({float(read[0]):.0f},{float(read[1]):.0f})  "
            f"({float(row.dpx_x):+.0f},{float(row.dpx_y):+.0f})  "
            f"({float(row.d_um_x):+.1f},{float(row.d_um_y):+.1f})  "
            f"{(row.dir or '-'):<7} "
            f"{(row.pwm or '-'):<22} "
            f"{moved:6.1f}  {row.ok}"
        )
    return "\n".join(lines)


def can_start_zscan(
    *,
    action: str,
    reason: str,
    center_attempted: bool,
    residual_px: Optional[float],
    tau_px: float,
    objects_found: bool,
) -> bool:
    """Z-scan si hay objeto visible. |Δpx| vs umbral NUNCA bloquea.

    Solo lost_lock real (grano fuera del FOV) o 0 objetos impiden AF.
    El centrado XY es best-effort: residual 454 px con roi_clip=0 → AF igual.
    """
    del action, center_attempted, residual_px, tau_px
    if str(reason or "") == "lost_lock":
        return False
    return bool(objects_found)


def decide_post_jog(
    *,
    objects_found: bool,
    e_px_pre: float,
    e_px_post: Optional[float],
    e_x_pre: float,
    e_y_pre: float,
    e_x_post: Optional[float],
    e_y_post: Optional[float],
    would_clip_post: bool,
    timeout: bool,
    attempt: int,
    max_attempts: int,
    tau_px: float,
    would_clip_pre: bool = False,
    sign_flipped: bool = False,
    sign_flipped_x: bool = False,
    sign_flipped_y: bool = False,
) -> PostJogDecision:
    """Parada del acercamiento: centrado, best-effort a AF, o retry.

    Un timeout de paso NUNCA aborta el Z-scan. Objeto visible → proceed a AF
    al agotar pasos (in-range: 1; clip: 3–4). |Δpx| > umbral no es blocker.
    In-range: el flip de signos no lanza otro jog. Clip: un flip y se sigue.
    """
    del e_px_pre
    found = bool(objects_found)
    if not found:
        return PostJogDecision(action="stop", reason="redetect_fail")

    clipped = bool(would_clip_post) or bool(would_clip_pre)
    e_post_known = e_px_post is not None
    if not e_post_known:
        if int(attempt) < int(max_attempts):
            reason = "timeout_continue" if timeout else "no_post_measure"
            return PostJogDecision(action="retry", reason=reason)
        return PostJogDecision(action="proceed", reason="best_effort")

    e_post = abs(float(e_px_post))
    tau = max(0.0, float(tau_px))
    if e_post <= tau:
        return PostJogDecision(action="proceed", reason="centered")

    flipped_x = bool(sign_flipped_x) or bool(sign_flipped)
    flipped_y = bool(sign_flipped_y) or bool(sign_flipped)
    flip_x = False
    flip_y = False
    if e_x_post is not None and not axis_crossed_zero(e_x_pre, e_x_post):
        if axis_error_worsened(e_x_pre, e_x_post) and not flipped_x:
            flip_x = True
    if e_y_post is not None and not axis_crossed_zero(e_y_pre, e_y_post):
        if axis_error_worsened(e_y_pre, e_y_post) and not flipped_y:
            flip_y = True
    if flip_x or flip_y:
        if not clipped:
            return PostJogDecision(action="proceed", reason="best_effort")
        return PostJogDecision(
            action="revert_flip",
            reason="axis_worsened",
            flip_x=bool(flip_x),
            flip_y=bool(flip_y),
        )

    if int(attempt) < int(max_attempts):
        reason = "timeout_continue" if timeout else "not_centered"
        return PostJogDecision(action="retry", reason=reason)
    return PostJogDecision(action="proceed", reason="best_effort")


def plan_center_jog(
    *,
    cx: float,
    cy: float,
    bbox: Sequence[float],
    frame_w: int,
    frame_h: int,
    fov_x_um: float,
    fov_y_um: float,
    stage_x_um: float,
    stage_y_um: float,
    pad_px: int,
    hysteresis_px: float = DEFAULT_HYSTERESIS_PX,
    hysteresis_um: float = DEFAULT_HYSTERESIS_UM,
    max_delta_um: float = DEFAULT_MAX_DELTA_UM,
    sign_x: int = DEFAULT_SIGN_X,
    sign_y: int = DEFAULT_SIGN_Y,
    x_min_um: Optional[float] = None,
    x_max_um: Optional[float] = None,
    y_min_um: Optional[float] = None,
    y_max_um: Optional[float] = None,
    step_gain: float = DEFAULT_STEP_GAIN,
    step_gain_x: Optional[float] = None,
    step_gain_y: Optional[float] = None,
    clip_short_step: bool = False,
) -> CenterPlan:
    """Un paso del lazo: Δpx overlay → un punto de trayectoria XY.

    - Offset px: ``(W/2 − cx, H/2 − cy)`` hacia el centro de la imagen.
      Centroide recortado (parte visible) es válido: ese vector mete el grano
      al centro y el resto de la silueta entra al FOV.
    - Escala = FOV de malla / frame. ``dx = sign·Δpx·fov/W`` (sign default +1).
    - Target = XY actual + gain·Δ (0.6–0.8 por defecto). Si ``clip_short_step``
      y ROI recortado: cap ~32 % FOV y re-medir.
    - Seguridad: cap 40 % FOV por eje. Workspace solo si el stage está
      dentro de una caja real; nunca yankar a un punto stale de malla.
    - |Δpx| > FORCE_CENTER_PX o clip: siempre mover, ignore hysteresis_um.
    """
    fw, fh = int(frame_w), int(frame_h)
    img_cx, img_cy = image_center_px(fw, fh)
    left, top, right, bottom = clip_edges(bbox, fw, fh)
    e_x, e_y, e_px = pixel_offset_from_center(cx, cy, fw, fh)
    dx_um, dy_um = pixel_offset_to_stage_um(
        e_x, e_y, fw, fh, fov_x_um, fov_y_um, sign_x=sign_x, sign_y=sign_y
    )
    would_clip, _window = predict_static_window_clip(bbox, fw, fh, pad_px)
    margin = roi_frame_margin_px(bbox, fw, fh)
    tau_px = max(0.0, float(hysteresis_px))
    tau_um = max(0.0, float(hysteresis_um))
    e_um = hypot(dx_um, dy_um)
    clipped = bool(would_clip or left or top or right or bottom)
    force_center = clipped or (e_px > FORCE_CENTER_PX)
    already = (not force_center) and (e_px <= tau_px and e_um <= tau_um)

    need_x = (abs(e_x) > tau_px or abs(dx_um) > tau_um or left or right) and not already
    need_y = (abs(e_y) > tau_px or abs(dy_um) > tau_um or top or bottom) and not already
    if already:
        need_x = False
        need_y = False

    gain = max(0.05, min(1.0, float(step_gain)))
    gx = gain if step_gain_x is None else max(0.05, min(1.0, float(step_gain_x)))
    gy = gain if step_gain_y is None else max(0.05, min(1.0, float(step_gain_y)))
    dx_cmd = float(dx_um) * gx if need_x else 0.0
    dy_cmd = float(dy_um) * gy if need_y else 0.0
    if bool(clip_short_step) and clipped:
        dx_cmd, dy_cmd = cap_step_command(
            dx_cmd, dy_cmd, fov_x_um, fov_y_um, frac=CLIP_STEP_FOV_FRAC
        )
    dx_cmd, dy_cmd = cap_step_command(
        dx_cmd, dy_cmd, fov_x_um, fov_y_um, max_delta_um=max_delta_um
    )

    # Siempre xy_tgt = xy_read_ahora + Δcap. El workspace NO puede yankar
    # a un punto viejo (run B: now=21040 → tgt=19920, Δcmd=−1120 µm).
    target_x = float(stage_x_um) + dx_cmd
    target_y = float(stage_y_um) + dy_cmd
    limit_hit = False
    ws_ok = workspace_usable(x_min_um, x_max_um, y_min_um, y_max_um)
    inside = ws_ok and stage_inside_workspace(
        float(stage_x_um),
        float(stage_y_um),
        float(x_min_um),
        float(x_max_um),
        float(y_min_um),
        float(y_max_um),
    )
    if inside:
        if target_x < float(x_min_um):
            target_x = float(x_min_um)
            limit_hit = True
        if target_x > float(x_max_um):
            target_x = float(x_max_um)
            limit_hit = True
        if target_y < float(y_min_um):
            target_y = float(y_min_um)
            limit_hit = True
        if target_y > float(y_max_um):
            target_y = float(y_max_um)
            limit_hit = True
        dx_cmd = target_x - float(stage_x_um)
        dy_cmd = target_y - float(stage_y_um)
        dx_cmd, dy_cmd = cap_step_command(
            dx_cmd, dy_cmd, fov_x_um, fov_y_um, max_delta_um=max_delta_um
        )
        target_x = float(stage_x_um) + dx_cmd
        target_y = float(stage_y_um) + dy_cmd
    dx_cmd = target_x - float(stage_x_um)
    dy_cmd = target_y - float(stage_y_um)
    delta_clamped = bool(limit_hit) or (
        abs(dx_cmd - (float(dx_um) if need_x else 0.0)) > 1e-6
        or abs(dy_cmd - (float(dy_um) if need_y else 0.0)) > 1e-6
    )

    base = dict(
        e_x_px=e_x,
        e_y_px=e_y,
        e_px=e_px,
        dx_um=dx_um,
        dy_um=dy_um,
        dx_cmd_um=dx_cmd,
        dy_cmd_um=dy_cmd,
        target_x_um=target_x,
        target_y_um=target_y,
        would_clip=would_clip,
        limit_hit=limit_hit,
        roi_frame_margin_px=margin,
        delta_clamped=delta_clamped,
        clip_edges=format_clip_edges(left, top, right, bottom),
        need_x=bool(need_x and abs(dx_cmd) > 1e-9),
        need_y=bool(need_y and abs(dy_cmd) > 1e-9),
        img_cx=float(img_cx),
        img_cy=float(img_cy),
        roi_cx=float(cx),
        roi_cy=float(cy),
    )

    if already:
        return CenterPlan(
            should_move=False,
            skip_af=False,
            reason="already_centered",
            **base,
        )

    if abs(dx_cmd) < 1e-9 and abs(dy_cmd) < 1e-9:
        no_fov = float(fov_x_um) <= 0.0 or float(fov_y_um) <= 0.0
        if no_fov:
            reason = "no_fov"
            skip_af = False
        elif limit_hit and would_clip:
            reason = "limit"
            skip_af = False
        else:
            reason = "already_centered"
            skip_af = False
        return CenterPlan(
            should_move=False,
            skip_af=skip_af,
            reason=reason,
            **base,
        )

    return CenterPlan(
        should_move=True,
        skip_af=False,
        reason="jog",
        **base,
    )


def center_success(
    *,
    e_px_post: float,
    tau_px: float,
    would_clip_post: bool,
    limit_hit: bool,
    inner_pixels: Optional[int] = None,
) -> bool:
    """Éxito binario del informe §5.5 (sin exigir inner si no se midió)."""
    if limit_hit or would_clip_post:
        return False
    if e_px_post > max(0.0, float(tau_px)):
        return False
    if inner_pixels is not None and int(inner_pixels) < MIN_INNER_PIXELS:
        return False
    return True

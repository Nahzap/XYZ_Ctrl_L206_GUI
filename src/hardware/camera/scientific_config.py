"""
Configuración científica rigurosa para Basler acA2500-14uc.

Lógica pura y GenICam duck-typed (mockeable) — sin dependencia obligatoria de pylon.
Basado en datasheet oficial Basler y prácticas de adquisición reproducible.
"""

from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple
import math

import numpy as np


# ---------------------------------------------------------------------------
# Datasheet
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class CameraDatasheet:
    model: str
    width: int
    height: int
    max_fps_full_frame: float
    pixel_size_um: float
    native_bit_depth: int
    shutter: str
    pixel_format_priority: Tuple[str, ...]
    factory_user_sets: Tuple[str, ...]
    spectrum: str = "Visible"
    interface: str = "USB 3.0"


ACA2500_14UC = CameraDatasheet(
    model="acA2500-14uc",
    width=2590,
    height=1942,
    max_fps_full_frame=14.0,
    pixel_size_um=2.2,
    native_bit_depth=12,
    shutter="rolling",
    pixel_format_priority=(
        "BayerGB12",
        "BayerGB12p",
        "BayerGB8",
        "Mono8",
        "YCbCr422_8",
    ),
    factory_user_sets=(
        "Default",
        "HighGain",
        "AutoFunctions",
        "UserSet1",
        "UserSet2",
        "UserSet3",
    ),
)

DEFAULT_DATASHEET = ACA2500_14UC

# Exposición de trabajo unificada (s). Valor operativo típico de microscopía en el lab.
DEFAULT_SCIENTIFIC_EXPOSURE_S = 0.015
DEFAULT_SCIENTIFIC_FPS = 14.0
DEFAULT_SCIENTIFIC_BUFFER = 5
DEFAULT_GAIN_DB = 0.0
MSB_SHIFT_12_TO_16 = 4  # 12-bit → MSB de uint16


# ---------------------------------------------------------------------------
# Perfil y hallazgos
# ---------------------------------------------------------------------------

@dataclass
class ScientificCameraSettings:
    """Perfil de adquisición fijo y documentado."""

    model: str = ACA2500_14UC.model
    exposure_s: float = DEFAULT_SCIENTIFIC_EXPOSURE_S
    fps: float = DEFAULT_SCIENTIFIC_FPS
    gain_db: float = DEFAULT_GAIN_DB
    buffer_frames: int = DEFAULT_SCIENTIFIC_BUFFER
    binning: int = 1
    width: int = ACA2500_14UC.width
    height: int = ACA2500_14UC.height
    pixel_format: Optional[str] = None
    load_user_set: str = "Default"
    disable_auto_exposure: bool = True
    disable_auto_gain: bool = True
    preserve_bit_depth: bool = True
    acquisition_frame_rate_enable: bool = True

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class AuditFinding:
    check_id: str
    ok: bool
    weight: int
    message: str
    severity: str = "info"  # info | warning | error


@dataclass
class AuditReport:
    findings: List[AuditFinding] = field(default_factory=list)

    @property
    def score(self) -> float:
        total_w = sum(f.weight for f in self.findings)
        if total_w <= 0:
            return 0.0
        ok_w = sum(f.weight for f in self.findings if f.ok)
        return 100.0 * ok_w / total_w

    @property
    def n_ok(self) -> int:
        return sum(1 for f in self.findings if f.ok)

    @property
    def n_total(self) -> int:
        return len(self.findings)

    @property
    def errors(self) -> List[AuditFinding]:
        return [f for f in self.findings if not f.ok and f.severity == "error"]

    @property
    def warnings(self) -> List[AuditFinding]:
        return [f for f in self.findings if not f.ok and f.severity == "warning"]

    def summary(self) -> Dict[str, Any]:
        return {
            "score": round(self.score, 2),
            "n_ok": self.n_ok,
            "n_total": self.n_total,
            "n_errors": len(self.errors),
            "n_warnings": len(self.warnings),
            "findings": [
                {
                    "check_id": f.check_id,
                    "ok": f.ok,
                    "weight": f.weight,
                    "severity": f.severity,
                    "message": f.message,
                }
                for f in self.findings
            ],
        }


@dataclass(frozen=True)
class SaveFrameResult:
    frame: np.ndarray
    bit_depth_native: int
    packed_as: int
    synthetic: bool
    warning: str = ""
    source: str = "unknown"


# ---------------------------------------------------------------------------
# Normalización pura
# ---------------------------------------------------------------------------

def clamp_fps(
    requested: float,
    max_fps: float = DEFAULT_DATASHEET.max_fps_full_frame,
) -> float:
    """Limita FPS al datasheet / máximo hardware."""
    if requested is None or not math.isfinite(float(requested)):
        return float(max_fps)
    req = float(requested)
    if req <= 0:
        return float(max_fps)
    return float(min(req, max_fps))


def unify_exposure_s(
    camera_exposure: Optional[float],
    tab_exposure: Optional[float],
    default: float = DEFAULT_SCIENTIFIC_EXPOSURE_S,
) -> Tuple[float, bool]:
    """
    Unifica exposición entre bloques de config.

    Returns:
        (exposure_s, was_inconsistent)
    """
    vals = []
    for v in (camera_exposure, tab_exposure):
        if v is None:
            continue
        try:
            fv = float(v)
        except (TypeError, ValueError):
            continue
        if math.isfinite(fv) and fv > 0:
            vals.append(fv)

    if not vals:
        return float(default), False

    # Preferir camera_tab (último valor de UI) si ambos existen; marcar inconsistencia
    if len(vals) >= 2 and not math.isclose(vals[0], vals[1], rel_tol=0.0, abs_tol=1e-9):
        return float(vals[-1]), True
    return float(vals[-1]), False


def select_pixel_format(
    available: Sequence[str],
    priority: Sequence[str] = DEFAULT_DATASHEET.pixel_format_priority,
) -> Optional[str]:
    """Elige el primer formato de la prioridad presente en available."""
    avail = set(available or [])
    for fmt in priority:
        if fmt in avail:
            return fmt
    return None


def resolve_camera_resolution(
    *,
    frame_hw: Optional[Tuple[int, ...]] = None,
    configured_wh: Optional[Tuple[int, int]] = None,
    node_wh: Optional[Tuple[int, int]] = None,
    datasheet_wh: Optional[Tuple[int, int]] = None,
) -> Optional[Tuple[int, int]]:
    """Resuelve (width, height) sin inventar 1920×1080.

    Prioridad: frame actual → ROI configurado → nodos GenICam → datasheet.
    """
    if frame_hw is not None and len(frame_hw) >= 2:
        h, w = int(frame_hw[0]), int(frame_hw[1])
        if w > 0 and h > 0:
            return (w, h)
    for candidate in (configured_wh, node_wh, datasheet_wh):
        if candidate is None:
            continue
        w, h = int(candidate[0]), int(candidate[1])
        if w > 0 and h > 0:
            return (w, h)
    return None


def read_camera_roi_wh(camera: Any) -> Optional[Tuple[int, int]]:
    """Lee Width/Height actuales de una cámara GenICam-like."""
    try:
        w = int(_NodeProxy(camera, "Width").get_value())
        h = int(_NodeProxy(camera, "Height").get_value())
        if w > 0 and h > 0:
            return (w, h)
    except Exception:
        return None
    return None


def is_native_microscopy_resolution(
    width: Optional[int],
    height: Optional[int],
    datasheet: CameraDatasheet = DEFAULT_DATASHEET,
) -> bool:
    """True si es nativo o '0' (señal de usar nativo)."""
    if width is None or height is None:
        return False
    w, h = int(width), int(height)
    if w == 0 and h == 0:
        return True
    return w == datasheet.width and h == datasheet.height


def build_scientific_settings(
    *,
    exposure_s: Optional[float] = None,
    fps: Optional[float] = None,
    gain_db: float = DEFAULT_GAIN_DB,
    buffer_frames: Optional[int] = None,
    available_pixel_formats: Optional[Sequence[str]] = None,
    datasheet: CameraDatasheet = DEFAULT_DATASHEET,
) -> ScientificCameraSettings:
    """Construye perfil científico con clamps de datasheet."""
    exp = float(exposure_s) if exposure_s is not None else DEFAULT_SCIENTIFIC_EXPOSURE_S
    if not math.isfinite(exp) or exp <= 0:
        exp = DEFAULT_SCIENTIFIC_EXPOSURE_S

    target_fps = clamp_fps(
        fps if fps is not None else datasheet.max_fps_full_frame,
        datasheet.max_fps_full_frame,
    )
    buf = int(buffer_frames) if buffer_frames is not None else DEFAULT_SCIENTIFIC_BUFFER
    if buf < 1:
        buf = DEFAULT_SCIENTIFIC_BUFFER

    pix = None
    if available_pixel_formats is not None:
        pix = select_pixel_format(available_pixel_formats, datasheet.pixel_format_priority)

    return ScientificCameraSettings(
        model=datasheet.model,
        exposure_s=exp,
        fps=target_fps,
        gain_db=float(gain_db),
        buffer_frames=buf,
        width=datasheet.width,
        height=datasheet.height,
        pixel_format=pix,
    )


def settings_from_parameter_blocks(
    camera_block: Optional[Mapping[str, Any]] = None,
    camera_tab_block: Optional[Mapping[str, Any]] = None,
    datasheet: CameraDatasheet = DEFAULT_DATASHEET,
) -> Tuple[ScientificCameraSettings, bool]:
    """Deriva settings desde dicts de plantilla/UI."""
    camera_block = camera_block or {}
    tab_cam = (camera_tab_block or {}).get("camera", {}) or {}

    exp, inconsistent = unify_exposure_s(
        camera_block.get("exposure"),
        tab_cam.get("exposure"),
    )
    fps_raw = tab_cam.get("fps", camera_block.get("frame_rate"))
    buf_raw = tab_cam.get("buffer_frames", camera_block.get("buffer_size"))

    settings = build_scientific_settings(
        exposure_s=exp,
        fps=float(fps_raw) if fps_raw is not None else None,
        buffer_frames=int(buf_raw) if buf_raw is not None else None,
        datasheet=datasheet,
    )
    return settings, inconsistent


# ---------------------------------------------------------------------------
# Profundidad de bits / guardado honesto
# ---------------------------------------------------------------------------

def align_12bit_to_uint16_msb(frame: np.ndarray) -> np.ndarray:
    """
    Empaqueta valores ~12-bit (LSB) en MSB de uint16 (shift 4).

    Importante: tras WB, max puede superar 4095 sin estar en MSB. Un umbral
    duro en max>4095 dejaba PNGs casi negros en visores 16-bit (~3400/65535).
    """
    arr = np.asarray(frame)
    if arr.dtype != np.uint16:
        arr = arr.astype(np.uint16, copy=False)
    if arr.size == 0:
        return arr

    # Energía en bits altos ⇒ ya está empaquetado para visualización 16-bit.
    p95 = float(np.percentile(arr, 95))
    if p95 > 16383.0:
        return arr

    # Aún vive en la banda baja (12-bit nativo o 12-bit + WB leve).
    # <<4 con saturación: visores 16-bit ven el campo claro, no negro.
    packed = np.left_shift(arr.astype(np.uint32), MSB_SHIFT_12_TO_16)
    return np.clip(packed, 0, 65535).astype(np.uint16)


def bayer_opencv_code(pixel_format: str):
    """Mapea PixelFormat Basler → código OpenCV Bayer→BGR."""
    import cv2

    pf = str(pixel_format or "")
    if "BayerGB" in pf:
        return cv2.COLOR_BayerGB2BGR
    if "BayerBG" in pf:
        return cv2.COLOR_BayerBG2BGR
    if "BayerRG" in pf:
        return cv2.COLOR_BayerRG2BGR
    if "BayerGR" in pf:
        return cv2.COLOR_BayerGR2BGR
    return None


def estimate_brightfield_wb_gains(
    bgr: np.ndarray,
    *,
    bright_percentile: float = 80.0,
) -> Tuple[float, float, float]:
    """
    Ganancias BGR para neutralizar fondo de campo claro.

    Returns:
        (gain_b, gain_g, gain_r) estimadas solo sobre el BGR demosaicado
        científico (mismo motor que el PNG). Iguala el fondo brillante al
        promedio de canales (gray-world de campo claro).
    """
    arr = np.asarray(bgr)
    if arr.ndim != 3 or arr.shape[2] != 3 or arr.size == 0:
        return (1.0, 1.0, 1.0)

    # Submuestreo barato para estimación en vivo
    work = arr
    if work.shape[0] > 240 and work.shape[1] > 240:
        work = work[::8, ::8]
    work_f = work.astype(np.float64, copy=False)
    lum = (
        0.114 * work_f[..., 0]
        + 0.587 * work_f[..., 1]
        + 0.299 * work_f[..., 2]
    )
    thr = float(np.percentile(lum, float(bright_percentile)))
    mask = lum >= thr
    if int(np.count_nonzero(mask)) < 16:
        return (1.0, 1.0, 1.0)

    means = [float(work_f[..., i][mask].mean()) for i in range(3)]
    if min(means) <= 1e-6:
        return (1.0, 1.0, 1.0)

    # Neutro: B/G/R del fondo → promedio. Clip amplio para iluminaciones cálidas
    # sin reutilizar ganancias de otro demosaic (pylon vs OpenCV).
    target = float(sum(means) / 3.0)
    gains = tuple(
        float(np.clip(target / m if m > 1e-6 else 1.0, 0.25, 4.0))
        for m in means
    )
    return gains  # type: ignore[return-value]


def apply_channel_gains(
    bgr: np.ndarray, gains: Tuple[float, float, float]
) -> np.ndarray:
    """Aplica (gain_b, gain_g, gain_r) preservando dtype."""
    arr = np.asarray(bgr)
    if arr.ndim != 3 or arr.shape[2] != 3:
        return arr
    gb, gg, gr = gains
    if abs(gb - 1.0) < 1e-6 and abs(gg - 1.0) < 1e-6 and abs(gr - 1.0) < 1e-6:
        return arr
    out = arr.astype(np.float32, copy=True)
    out[..., 0] *= float(gb)
    out[..., 1] *= float(gg)
    out[..., 2] *= float(gr)
    if arr.dtype == np.uint16:
        return np.clip(out, 0, 65535).astype(np.uint16)
    if arr.dtype == np.uint8:
        return np.clip(out, 0, 255).astype(np.uint8)
    return out.astype(arr.dtype, copy=False)


def apply_brightfield_white_balance(
    bgr: np.ndarray,
    *,
    bright_percentile: float = 80.0,
) -> np.ndarray:
    """
    Balance de blancos para campo claro: el fondo brillante debe ser neutro.

    Sin esto, el demosaic Bayer crudo deja G dominante (tinte verde/cian)
    y distorsiona el color real del espécimen.
    """
    gains = estimate_brightfield_wb_gains(
        bgr, bright_percentile=bright_percentile
    )
    return apply_channel_gains(bgr, gains)


def demosaic_scientific_bayer(
    raw: np.ndarray,
    pixel_format: str,
    *,
    white_balance: bool = True,
) -> np.ndarray:
    """
    Compat: delega al pipeline único ``prepare_scientific_bgr16``.

    Preferir ``hardware.camera.scientific_image.prepare_scientific_bgr16``.
    """
    from hardware.camera.scientific_image import prepare_scientific_bgr16

    return prepare_scientific_bgr16(
        raw,
        pixel_format=pixel_format,
        wb_gains=None if white_balance else (1.0, 1.0, 1.0),
    )


def extract_raw_array_from_grab(grab_array: np.ndarray) -> np.ndarray:
    """Copia defensiva del array crudo del grab."""
    return np.asarray(grab_array).copy()


def resolve_save_frame(
    *,
    raw_frame: Optional[np.ndarray],
    preview_frame: Optional[np.ndarray],
    use_16bit: bool,
    native_bit_depth: int = DEFAULT_DATASHEET.native_bit_depth,
) -> SaveFrameResult:
    """
    DEPRECATED — no usar en producción.

    La única vía es ``worker.acquire_scientific_frame`` +
    ``save_scientific_image``. Este helper queda solo por tests legacy.
    """
    if use_16bit:
        if raw_frame is not None and np.asarray(raw_frame).dtype == np.uint16:
            out = align_12bit_to_uint16_msb(raw_frame)
            return SaveFrameResult(
                frame=out,
                bit_depth_native=native_bit_depth,
                packed_as=16,
                synthetic=False,
                warning="",
                source="raw_uint16",
            )
        # Honesto: no fabricar uint16
        src = preview_frame if preview_frame is not None else raw_frame
        if src is None:
            raise ValueError("No hay frame disponible para guardar")
        frame8 = _ensure_uint8(src)
        return SaveFrameResult(
            frame=frame8,
            bit_depth_native=8,
            packed_as=8,
            synthetic=False,
            warning=(
                "use_16bit solicitado pero no hay raw uint16; "
                "se guarda 8-bit (preview/demosaic)."
            ),
            source="preview_fallback_8bit",
        )

    src = preview_frame if preview_frame is not None else raw_frame
    if src is None:
        raise ValueError("No hay frame disponible para guardar")
    return SaveFrameResult(
        frame=_ensure_uint8(src),
        bit_depth_native=8,
        packed_as=8,
        synthetic=False,
        warning="",
        source="preview_8bit",
    )


def _ensure_uint8(frame: np.ndarray) -> np.ndarray:
    arr = np.asarray(frame)
    if arr.dtype == np.uint8:
        return arr.copy()
    if arr.dtype == np.uint16:
        if arr.size == 0:
            return arr.astype(np.uint8)
        mx = int(arr.max())
        if mx <= 0:
            return np.zeros(arr.shape, dtype=np.uint8)
        # Si está en rango 12-bit, escalar desde 4095; si MSB-aligned, desde 65535
        denom = 65535.0 if mx > 4095 else 4095.0
        return np.clip(arr.astype(np.float64) * (255.0 / denom), 0, 255).astype(np.uint8)
    # Otros dtypes
    mx = float(np.max(arr)) if arr.size else 0.0
    if mx <= 0:
        return np.zeros(arr.shape, dtype=np.uint8)
    return np.clip(arr.astype(np.float64) / mx * 255.0, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------------------
# GenICam duck-typed apply
# ---------------------------------------------------------------------------

class _NodeProxy:
    """Acceso uniforme a nodos estilo pylon (atributo o dict)."""

    def __init__(self, camera: Any, name: str):
        self.camera = camera
        self.name = name

    def _raw(self) -> Any:
        if hasattr(self.camera, self.name):
            return getattr(self.camera, self.name)
        if isinstance(self.camera, Mapping) and self.name in self.camera:
            return self.camera[self.name]
        raise AttributeError(self.name)

    def get_value(self) -> Any:
        node = self._raw()
        if hasattr(node, "GetValue"):
            return node.GetValue()
        if hasattr(node, "value"):
            return node.value
        return node

    def set_value(self, value: Any) -> None:
        node = self._raw()
        if hasattr(node, "SetValue"):
            node.SetValue(value)
            return
        if hasattr(node, "value"):
            node.value = value
            return
        # dict mutable de valores simples
        if isinstance(self.camera, dict):
            self.camera[self.name] = value
            return
        raise TypeError(f"No se puede escribir nodo {self.name}")

    def get_max(self) -> Any:
        node = self._raw()
        if hasattr(node, "GetMax"):
            return node.GetMax()
        if hasattr(node, "max"):
            return node.max
        raise AttributeError(f"{self.name}.max")

    def get_min(self) -> Any:
        node = self._raw()
        if hasattr(node, "GetMin"):
            return node.GetMin()
        if hasattr(node, "min"):
            return node.min
        raise AttributeError(f"{self.name}.min")

    def symbolics(self) -> List[str]:
        node = self._raw()
        if hasattr(node, "Symbolics"):
            return list(node.Symbolics)
        if hasattr(node, "symbolics"):
            return list(node.symbolics)
        return []

    def execute(self) -> None:
        node = self._raw()
        if callable(node):
            node()
            return
        if hasattr(node, "Execute"):
            node.Execute()
            return
        raise TypeError(f"{self.name} no es ejecutable")


def _try_set(camera: Any, name: str, value: Any) -> bool:
    try:
        _NodeProxy(camera, name).set_value(value)
        return True
    except Exception:
        return False


def _try_get(camera: Any, name: str, default: Any = None) -> Any:
    try:
        return _NodeProxy(camera, name).get_value()
    except Exception:
        return default


def apply_scientific_settings(
    camera: Any,
    settings: ScientificCameraSettings,
    *,
    datasheet: CameraDatasheet = DEFAULT_DATASHEET,
) -> Dict[str, Any]:
    """
    Aplica el perfil científico a una cámara GenICam-like.

    Returns:
        dict con acciones realizadas / omitidas (trazabilidad).
    """
    log: Dict[str, Any] = {"applied": [], "skipped": [], "settings": settings.to_dict()}

    # User set Default (solo si el selector existe)
    if settings.load_user_set:
        try:
            sel = _NodeProxy(camera, "UserSetSelector")
            available = []
            try:
                available = sel.symbolics()
            except Exception:
                available = list(datasheet.factory_user_sets)
            if not available or settings.load_user_set in available:
                sel.set_value(settings.load_user_set)
                _NodeProxy(camera, "UserSetLoad").execute()
                log["applied"].append(f"UserSetLoad={settings.load_user_set}")
            else:
                log["skipped"].append(f"UserSet {settings.load_user_set} no disponible")
        except Exception as exc:
            log["skipped"].append(f"UserSet: {exc}")

    # Resolución nativa (Offset → 0 antes de Width/Height máximos)
    try:
        _try_set(camera, "OffsetX", 0)
        _try_set(camera, "OffsetY", 0)
        w_node = _NodeProxy(camera, "Width")
        h_node = _NodeProxy(camera, "Height")
        max_w = int(w_node.get_max()) if hasattr(w_node._raw(), "GetMax") or hasattr(w_node._raw(), "max") else settings.width
        max_h = int(h_node.get_max()) if hasattr(h_node._raw(), "GetMax") or hasattr(h_node._raw(), "max") else settings.height
        w_node.set_value(max_w)
        h_node.set_value(max_h)
        # Releer valores efectivos (inc. alineación GenICam)
        try:
            max_w = int(w_node.get_value())
            max_h = int(h_node.get_value())
        except Exception:
            pass
        log["resolution"] = (max_w, max_h)
        log["applied"].append(f"Resolution={max_w}x{max_h}")
    except Exception as exc:
        log["skipped"].append(f"Resolution: {exc}")

    # Pixel format
    try:
        pf = _NodeProxy(camera, "PixelFormat")
        available = pf.symbolics()
        chosen = settings.pixel_format or select_pixel_format(available)
        if chosen:
            pf.set_value(chosen)
            log["applied"].append(f"PixelFormat={chosen}")
        else:
            log["skipped"].append("PixelFormat: ninguno de la prioridad")
    except Exception as exc:
        log["skipped"].append(f"PixelFormat: {exc}")

    # Autos Off
    if settings.disable_auto_exposure:
        if _try_set(camera, "ExposureAuto", "Off"):
            log["applied"].append("ExposureAuto=Off")
        else:
            log["skipped"].append("ExposureAuto")
    if settings.disable_auto_gain:
        if _try_set(camera, "GainAuto", "Off"):
            log["applied"].append("GainAuto=Off")
        else:
            log["skipped"].append("GainAuto")

    # WB auto Off: el balance de campo claro se aplica en host tras demosaic
    # (reproducible; evita deriva de BalanceWhiteAuto sobre muestras).
    if _try_set(camera, "BalanceWhiteAuto", "Off"):
        log["applied"].append("BalanceWhiteAuto=Off")
    else:
        log["skipped"].append("BalanceWhiteAuto")

    # Exposición (µs)
    try:
        exp_node = _NodeProxy(camera, "ExposureTime")
        exposure_us = float(settings.exposure_s) * 1e6
        try:
            exposure_us = max(float(exp_node.get_min()), min(exposure_us, float(exp_node.get_max())))
        except Exception:
            pass
        exp_node.set_value(exposure_us)
        log["applied"].append(f"ExposureTime={exposure_us}")
    except Exception as exc:
        log["skipped"].append(f"ExposureTime: {exc}")

    # Gain
    if _try_set(camera, "Gain", float(settings.gain_db)):
        log["applied"].append(f"Gain={settings.gain_db}")
    else:
        log["skipped"].append("Gain")

    # FPS
    if settings.acquisition_frame_rate_enable:
        _try_set(camera, "AcquisitionFrameRateEnable", True)
        try:
            fr = _NodeProxy(camera, "AcquisitionFrameRate")
            max_fps = float(settings.fps)
            try:
                max_fps = min(float(settings.fps), float(fr.get_max()))
            except Exception:
                max_fps = clamp_fps(settings.fps, datasheet.max_fps_full_frame)
            target = clamp_fps(max_fps, datasheet.max_fps_full_frame)
            # también respetar max hardware si es menor que datasheet
            try:
                hw_max = float(fr.get_max())
                target = min(target, hw_max)
            except Exception:
                pass
            fr.set_value(float(target))
            log["applied"].append(f"AcquisitionFrameRate={target}")
        except Exception as exc:
            log["skipped"].append(f"AcquisitionFrameRate: {exc}")

    # Buffer
    if _try_set(camera, "MaxNumBuffer", int(settings.buffer_frames)):
        log["applied"].append(f"MaxNumBuffer={settings.buffer_frames}")
    else:
        log["skipped"].append("MaxNumBuffer")

    # Binning
    ok_bh = _try_set(camera, "BinningHorizontal", int(settings.binning))
    ok_bv = _try_set(camera, "BinningVertical", int(settings.binning))
    if ok_bh or ok_bv:
        log["applied"].append(f"Binning={settings.binning}x{settings.binning}")
    else:
        log["skipped"].append("Binning")

    return log


# ---------------------------------------------------------------------------
# Auditoría de configuración de proyecto
# ---------------------------------------------------------------------------

def audit_project_camera_config(
    parameters: Mapping[str, Any],
    *,
    datasheet: CameraDatasheet = DEFAULT_DATASHEET,
    has_raw_uint16_path: bool = False,
) -> AuditReport:
    """Audita dict de parámetros (plantilla o runtime) contra checks del plan."""
    report = AuditReport()
    camera = parameters.get("camera", {}) or {}
    tab = parameters.get("camera_tab", {}) or {}
    tab_cam = tab.get("camera", {}) or {}
    micro = tab.get("microscopy", {}) or {}
    capture = tab.get("capture", {}) or {}

    model = str(camera.get("model", ""))
    model_ok = datasheet.model.lower() in model.lower()
    report.findings.append(
        AuditFinding(
            "CHK_MODEL",
            model_ok,
            1,
            f"Modelo='{model}'" if model_ok else f"Modelo inesperado: '{model}'",
            "error" if not model_ok else "info",
        )
    )

    fps_candidates = [tab_cam.get("fps"), camera.get("frame_rate")]
    fps_vals = [float(v) for v in fps_candidates if v is not None]
    fps_ok = bool(fps_vals) and all(v <= datasheet.max_fps_full_frame + 1e-9 for v in fps_vals)
    report.findings.append(
        AuditFinding(
            "CHK_FPS",
            fps_ok,
            2,
            f"FPS={fps_vals} ≤ {datasheet.max_fps_full_frame}"
            if fps_ok
            else f"FPS fuera de datasheet: {fps_vals}",
            "error" if not fps_ok else "info",
        )
    )

    exp_cam = camera.get("exposure")
    exp_tab = tab_cam.get("exposure")
    unified = (
        exp_cam is not None
        and exp_tab is not None
        and math.isclose(float(exp_cam), float(exp_tab), abs_tol=1e-12)
    )
    report.findings.append(
        AuditFinding(
            "CHK_EXP_UNIFIED",
            unified,
            2,
            f"Exposición unificada={exp_cam}"
            if unified
            else f"Exposición inconsistente camera={exp_cam} tab={exp_tab}",
            "error" if not unified else "info",
        )
    )

    # Gain: en parámetros puede no existir; se asume OK si no hay gain>0 declarado
    gain = tab_cam.get("gain_db", camera.get("gain_db", 0.0))
    try:
        gain_ok = float(gain) <= 0.0 + 1e-9
    except (TypeError, ValueError):
        gain_ok = True
    report.findings.append(
        AuditFinding(
            "CHK_GAIN0",
            gain_ok,
            1,
            f"gain_db={gain}",
            "warning" if not gain_ok else "info",
        )
    )

    sci = camera.get("scientific", {}) or tab.get("scientific", {}) or {}
    # Exige declaración explícita (no asumir True por omisión)
    auto_keys_present = (
        ("disable_auto_exposure" in tab_cam or "disable_auto_exposure" in sci)
        and ("disable_auto_gain" in tab_cam or "disable_auto_gain" in sci)
    )
    auto_off = auto_keys_present and bool(
        (sci.get("disable_auto_exposure", tab_cam.get("disable_auto_exposure", False)))
    ) and bool(
        (sci.get("disable_auto_gain", tab_cam.get("disable_auto_gain", False)))
    )
    report.findings.append(
        AuditFinding(
            "CHK_AUTO_OFF",
            auto_off,
            2,
            "Autos Off declarados" if auto_off else "Autos no desactivados explícitamente",
            "error" if not auto_off else "info",
        )
    )

    native = is_native_microscopy_resolution(
        micro.get("img_width"), micro.get("img_height"), datasheet
    )
    report.findings.append(
        AuditFinding(
            "CHK_NATIVE_RES",
            native,
            2,
            f"Microscopía {micro.get('img_width')}x{micro.get('img_height')}",
            "error" if not native else "info",
        )
    )

    use_16 = bool(capture.get("use_16bit", False))
    bit_ok = (not use_16) or has_raw_uint16_path or bool(
        sci.get("preserve_bit_depth", False)
    )
    report.findings.append(
        AuditFinding(
            "CHK_BITDEPTH_HONEST",
            bit_ok,
            3,
            "use_16bit con ruta raw/preserve declarada"
            if bit_ok
            else "use_16bit sin ruta raw uint16 (riesgo de 16-bit sintético)",
            "error" if not bit_ok else "info",
        )
    )

    buf = tab_cam.get("buffer_frames", camera.get("buffer_size"))
    try:
        buf_ok = buf is not None and int(buf) >= 1
    except (TypeError, ValueError):
        buf_ok = False
    report.findings.append(
        AuditFinding(
            "CHK_BUFFER",
            buf_ok,
            1,
            f"buffer={buf}",
            "warning" if not buf_ok else "info",
        )
    )

    # Pixel format: debe estar declarado en bloque scientific
    pf_priority = sci.get("pixel_format_priority") or []
    pf_ok = bool(pf_priority) and str(pf_priority[0]).startswith("BayerGB12")
    report.findings.append(
        AuditFinding(
            "CHK_PIXEL_FMT",
            pf_ok,
            2,
            f"Prioridad pixel format={list(pf_priority)[:3]}"
            if pf_ok
            else "pixel_format_priority no declarado en scientific",
            "warning" if not pf_ok else "info",
        )
    )

    return report


def recommended_template_camera_blocks(
    datasheet: CameraDatasheet = DEFAULT_DATASHEET,
) -> Dict[str, Any]:
    """Bloques recomendados para alinear la plantilla JSON."""
    return {
        "camera": {
            "description": "Parámetros base de cámara (perfil científico acA2500-14uc)",
            "model": f"Basler {datasheet.model}",
            "serial_number": "",
            "frame_rate": datasheet.max_fps_full_frame,
            "frame_rate_unit": "FPS",
            "exposure": DEFAULT_SCIENTIFIC_EXPOSURE_S,
            "exposure_unit": "seconds",
            "buffer_size": DEFAULT_SCIENTIFIC_BUFFER,
            "buffer_unit": "frames",
            "gain_db": DEFAULT_GAIN_DB,
            "scientific": {
                "disable_auto_exposure": True,
                "disable_auto_gain": True,
                "preserve_bit_depth": True,
                "load_user_set": "Default",
                "pixel_format_priority": list(datasheet.pixel_format_priority),
                "native_width": datasheet.width,
                "native_height": datasheet.height,
                "pixel_size_um": datasheet.pixel_size_um,
                "native_bit_depth": datasheet.native_bit_depth,
                "shutter": datasheet.shutter,
            },
        },
        "camera_tab_camera": {
            "exposure": DEFAULT_SCIENTIFIC_EXPOSURE_S,
            "fps": datasheet.max_fps_full_frame,
            "buffer_frames": DEFAULT_SCIENTIFIC_BUFFER,
            "gain_db": DEFAULT_GAIN_DB,
            "disable_auto_exposure": True,
            "disable_auto_gain": True,
        },
        "camera_tab_microscopy_size": {
            "img_width": datasheet.width,
            "img_height": datasheet.height,
        },
    }

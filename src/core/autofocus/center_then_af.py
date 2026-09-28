"""Bring-to-center XY post-detección / pre-AF.

Compartido por microscopía (malla) y AF manual (botón Enfocar). La geometría
vive en ``center_candidate``; este módulo orquesta el lazo cerrado
(detectar → goto_xy 1-pt acotado → SETTLED o timeout de ese Δcmd → re-detectar
el lock) y emite AF_CENTER / AF_CENTER_STEPS. No persigue debris. No inventa
ROI si se pierde el objeto. El centrado XY es best-effort: objeto en FOV
(roi_clip=0) arranca Z-scan aunque |Δpx| no haya cerrado a 40 px.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable, List, Optional, Sequence, Tuple

from PyQt5.QtCore import QCoreApplication, QObject, QThread, QTimer, Qt, pyqtSignal

from core.autofocus.af_kpi import AfCycleKpi
from core.autofocus.center_candidate import (
    DEFAULT_CENTER_ENABLED,
    DEFAULT_HYSTERESIS_PX,
    DEFAULT_HYSTERESIS_UM,
    DEFAULT_MAX_DELTA_UM,
    DEFAULT_MAX_RETRIES,
    DEFAULT_SIGN_X,
    DEFAULT_SIGN_Y,
    DEFAULT_STEP_GAIN,
    LOCK_MISS_LIMIT,
    LOCK_REDETECT_TRIES,
    LOOP_SAFETY_TIMEOUT_S,
    MOTION_EPS_UM,
    STEP_MOVED_CONFIRM_UM,
    CenterPlan,
    CenterStepRow,
    apply_overshoot_gains,
    can_start_zscan,
    command_dir_label,
    decide_post_jog,
    encoder_sign_error,
    format_center_pwm,
    format_center_steps_table,
    hypot,
    initial_step_gain,
    match_locked_object,
    object_bbox,
    object_centroid_px,
    pick_tracked_primary,
    pixel_offset_from_center,
    pixel_offset_to_stage_um,
    plan_center_jog,
    predict_static_window_clip,
    resolve_max_center_steps,
    roi_frame_margin_px,
    should_declare_lost_lock,
    sign_err_label,
    similar_area_or_iou_match,
    snapshot_identity_lock,
    step_goto_timeout_s,
    update_lock_geometry,
)

logger = logging.getLogger("MotorControl_L206")

FilterFn = Callable[[Sequence[Any]], List[Any]]
GrabFn = Callable[[], Any]
ValidFn = Callable[[], bool]


@dataclass
class CenterContext:
    """Inputs de sesión para un ciclo detectar → (jog) → re-detectar."""

    enabled: bool = DEFAULT_CENTER_ENABLED
    fov_x_um: float = 0.0
    fov_y_um: float = 0.0
    hysteresis_um: float = DEFAULT_HYSTERESIS_UM
    hysteresis_px: float = DEFAULT_HYSTERESIS_PX
    max_retries: int = DEFAULT_MAX_RETRIES
    max_delta_um: float = DEFAULT_MAX_DELTA_UM
    sign_x: int = DEFAULT_SIGN_X
    sign_y: int = DEFAULT_SIGN_Y
    step_gain: float = DEFAULT_STEP_GAIN
    jog_timeout_ms: int = 8000
    pad_px: int = 20
    stage_x_um: float = 0.0
    stage_y_um: float = 0.0
    workspace: Optional[Tuple[float, float, float, float]] = None
    settle_ms: int = 500
    point_index: Optional[int] = None
    t_detect_s: Optional[float] = None
    log_prefix: str = "[CenterThenAf]"
    has_stage: bool = False
    point_timeout_s: float = 6.0
    loop_timeout_s: float = LOOP_SAFETY_TIMEOUT_S
    gain_x: Optional[float] = None
    gain_y: Optional[float] = None


@dataclass
class CenterResult:
    """Salida del ciclo: AF en sitio, AF con ROI nuevo, o abortar."""

    action: str
    reason: str
    objects: List[Any] = field(default_factory=list)
    primary: Any = None
    kpi: dict = field(default_factory=dict)
    frame: Any = None
    steps: List[CenterStepRow] = field(default_factory=list)


def evaluate_center_pre_af(
    *,
    enabled: bool,
    fov_x_um: float,
    fov_y_um: float,
    plan: Optional[CenterPlan],
    has_stage: bool,
) -> Tuple[str, str]:
    """Decisión síncrona: proceed | jog | abort. Sin Qt ni hardware.

    ``abort`` ya no se usa para clip/límite: objeto visible → AF en sitio.
    ``proceed`` = AF (feature off, sin FOV, ya centrado, stage ausente, límite).
    """
    if not enabled:
        return "proceed", "feature_off"
    if float(fov_x_um) <= 0.0 or float(fov_y_um) <= 0.0:
        return "proceed", "no_fov"
    if plan is None:
        return "proceed", "no_centroid"
    if not plan.should_move:
        return "proceed", str(plan.reason)
    if not has_stage:
        return "proceed", "no_stage"
    return "jog", "jog"


def plan_from_object(
    obj, frame, ctx: CenterContext, *, clip_short_step: bool = False
) -> Optional[CenterPlan]:
    bbox = object_bbox(obj)
    if bbox is None or frame is None:
        return None
    # Silueta U2-Net si existe; si no, centro del ROI cuadrado. Clipado: la
    # parte visible sigue siendo un centroide válido hacia el centro de imagen.
    centroid = object_centroid_px(obj)
    if centroid is not None:
        cx, cy = float(centroid[0]), float(centroid[1])
    else:
        cx = float(bbox[0]) + float(bbox[2]) / 2.0
        cy = float(bbox[1]) + float(bbox[3]) / 2.0
    fh, fw = int(frame.shape[0]), int(frame.shape[1])
    ws = ctx.workspace
    return plan_center_jog(
        cx=cx,
        cy=cy,
        bbox=bbox,
        frame_w=fw,
        frame_h=fh,
        fov_x_um=ctx.fov_x_um,
        fov_y_um=ctx.fov_y_um,
        stage_x_um=ctx.stage_x_um,
        stage_y_um=ctx.stage_y_um,
        pad_px=int(ctx.pad_px),
        hysteresis_px=float(ctx.hysteresis_px),
        hysteresis_um=float(ctx.hysteresis_um),
        max_delta_um=float(ctx.max_delta_um),
        sign_x=int(ctx.sign_x),
        sign_y=int(ctx.sign_y),
        x_min_um=None if ws is None else ws[0],
        x_max_um=None if ws is None else ws[1],
        y_min_um=None if ws is None else ws[2],
        y_max_um=None if ws is None else ws[3],
        step_gain=float(getattr(ctx, "step_gain", DEFAULT_STEP_GAIN) or DEFAULT_STEP_GAIN),
        step_gain_x=getattr(ctx, "gain_x", None),
        step_gain_y=getattr(ctx, "gain_y", None),
        clip_short_step=bool(clip_short_step),
    )


def kpi_fields_from_plan(
    plan: Optional[CenterPlan],
    reason: str,
    ctx: CenterContext,
    action: str = "proceed",
) -> dict:
    attempted = action == "jog"
    skipped = action != "jog"
    if plan is None:
        return {
            "xy_offset_pre_px": None,
            "center_attempted": attempted,
            "center_success": False,
            "center_skipped": skipped,
            "center_skip_reason": reason,
            "roi_clipped": None,
            "roi_frame_margin_px": None,
            "t_detect": ctx.t_detect_s,
            "tau_px": float(ctx.hysteresis_px),
        }
    return {
        "xy_offset_pre_px": float(plan.e_px),
        "xy_offset_post_px": float(plan.e_px) if reason == "already_centered" else None,
        "center_attempted": attempted,
        "center_success": reason == "already_centered",
        "center_skipped": skipped,
        "center_skip_reason": reason,
        "roi_clipped": bool(plan.would_clip),
        "roi_frame_margin_px": float(plan.roi_frame_margin_px),
        "t_detect": ctx.t_detect_s,
        "tau_px": float(ctx.hysteresis_px),
    }


def format_center_log(kpi_fields: dict, ctx: CenterContext, extra: str = "") -> str:
    kpi = AfCycleKpi(point_index=ctx.point_index, t_detect=ctx.t_detect_s)
    for key, value in kpi_fields.items():
        if hasattr(kpi, key):
            setattr(kpi, key, value)
    line = kpi.format_center_line()
    if extra:
        line = f"{line} {extra}"
    return line


def pick_primary(objects: Sequence[Any]) -> Any:
    return pick_tracked_primary(objects, previous=None)


def order_primary_first(objects: Sequence[Any], primary: Any) -> List[Any]:
    objs = list(objects or [])
    if primary is None:
        return objs
    rest = [obj for obj in objs if obj is not primary]
    return [primary] + rest


class CenterThenAf(QObject):
    """Lazo XY cerrado: cada paso = 1 punto de trayectoria (FOV + SETTLED).

    Vive en el hilo GUI. ``trajectory_point_reached`` llega queued desde el
    worker; no bloquea la UI ni mueve XY durante el Z-scan. Centrado
    best-effort: proceed-a-AF con objeto en campo aunque |Δpx| > umbral.
    """

    completed = pyqtSignal(object)
    status_message = pyqtSignal(str)
    sign_changed = pyqtSignal(int, int)
    overlay_update = pyqtSignal(object)
    # Worker→GUI: start_trajectory / QTimer desde QThread no avanza el FOV.
    _goto_requested = pyqtSignal(float, float, str, float)
    # Settle/finish siempre en el hilo del QObject (QTimer.singleShot sin padre se GC).
    _finish_requested = pyqtSignal(int)

    def __init__(
        self,
        parent=None,
        *,
        test_service=None,
        grab_bgr_frame: Optional[GrabFn] = None,
        filter_objects: Optional[FilterFn] = None,
        still_valid: Optional[ValidFn] = None,
        log_prefix: str = "[CenterThenAf]",
    ):
        super().__init__(parent)
        self._test_service = None
        self._grab_bgr_frame = grab_bgr_frame
        self._filter_objects = filter_objects or (lambda objs: list(objs or []))
        self._still_valid = still_valid
        self._log_prefix = log_prefix
        self._pending = None
        self._scorer = None
        self._watchdog = QTimer(self)
        self._watchdog.setSingleShot(True)
        self._watchdog.timeout.connect(self._on_jog_watchdog)
        self._step_watchdog = QTimer(self)
        self._step_watchdog.setSingleShot(True)
        self._step_watchdog.timeout.connect(self._on_step_timeout)
        self._live_overlay = QTimer(self)
        self._live_overlay.setInterval(400)
        self._live_overlay.timeout.connect(self._on_live_overlay)
        self._settle_timer = QTimer(self)
        self._settle_timer.setSingleShot(True)
        self._settle_timer.timeout.connect(self._finish_after_settle)
        self._goto_requested.connect(self._execute_goto, Qt.QueuedConnection)
        self._finish_requested.connect(self._on_finish_requested, Qt.QueuedConnection)
        self.bind_test_service(test_service)

    def bind_test_service(self, test_service) -> None:
        if self._test_service is test_service:
            return
        if self._test_service is not None:
            for sig_name, slot in (
                ("jog_xy_reached", self._on_jog_reached),
                ("trajectory_point_reached", self._on_trajectory_point_reached),
            ):
                if hasattr(self._test_service, sig_name):
                    try:
                        getattr(self._test_service, sig_name).disconnect(slot)
                    except Exception:
                        pass
        self._test_service = test_service
        if test_service is None:
            return
        if hasattr(test_service, "jog_xy_reached"):
            test_service.jog_xy_reached.connect(
                self._on_jog_reached, Qt.QueuedConnection
            )
        if hasattr(test_service, "trajectory_point_reached"):
            test_service.trajectory_point_reached.connect(
                self._on_trajectory_point_reached, Qt.QueuedConnection
            )

    def configure(
        self,
        *,
        grab_bgr_frame: Optional[GrabFn] = None,
        filter_objects: Optional[FilterFn] = None,
        still_valid: Optional[ValidFn] = None,
        log_prefix: Optional[str] = None,
    ) -> None:
        if grab_bgr_frame is not None:
            self._grab_bgr_frame = grab_bgr_frame
        if filter_objects is not None:
            self._filter_objects = filter_objects
        if still_valid is not None:
            self._still_valid = still_valid
        if log_prefix is not None:
            self._log_prefix = log_prefix

    def _term(self, line: str) -> None:
        """Línea visible en terminal de cámara / Test (no solo logger.debug)."""
        prefix = (self._pending or {}).get("prefix") or self._log_prefix
        text = str(line)
        logger.info("%s %s", prefix, text)
        self.status_message.emit(text)
        ts = self._test_service
        if ts is not None and hasattr(ts, "log_message"):
            try:
                ts.log_message.emit(text)
            except Exception:
                pass

    def _pwm_snapshot(self, dx_cmd: float = 0.0, dy_cmd: float = 0.0):
        ts = self._test_service
        pwm_a = pwm_b = None
        umax = 255
        if ts is not None and hasattr(ts, "last_xy_pwm"):
            try:
                pair = ts.last_xy_pwm()
                pwm_a, pwm_b = int(pair[0]), int(pair[1])
            except Exception:
                pwm_a = pwm_b = None
        if ts is not None and hasattr(ts, "xy_pwm_umax"):
            try:
                umax = int(ts.xy_pwm_umax())
            except Exception:
                umax = 255
        ctx = (self._pending or {}).get("ctx")
        fov_x = float(getattr(ctx, "fov_x_um", 0.0) or 0.0) if ctx is not None else 0.0
        fov_y = float(getattr(ctx, "fov_y_um", 0.0) or 0.0) if ctx is not None else 0.0
        label = format_center_pwm(
            pwm_a,
            pwm_b,
            umax=umax,
            dx_cmd_um=float(dx_cmd),
            dy_cmd_um=float(dy_cmd),
            fov_x_um=fov_x,
            fov_y_um=fov_y,
        )
        return (pwm_a, pwm_b), label

    def _refresh_stage_xy(self, ctx: CenterContext) -> Tuple[float, float]:
        """XY leído ahora. Nunca reutilizar el punto de la run anterior."""
        fallback = (float(ctx.stage_x_um), float(ctx.stage_y_um))
        ts = self._test_service
        if ts is None or not hasattr(ts, "read_current_position_um"):
            return fallback
        try:
            sx, sy, _ex, _ey = ts.read_current_position_um(fallback[0], fallback[1])
        except Exception:
            return fallback
        if sx is not None:
            ctx.stage_x_um = float(sx)
        if sy is not None:
            ctx.stage_y_um = float(sy)
        return (float(ctx.stage_x_um), float(ctx.stage_y_um))

    def cancel(self) -> None:
        self._stop_all_center_timers()
        self._pending = None

    def is_pending(self) -> bool:
        return self._pending is not None

    def begin(self, objects: Sequence[Any], frame, ctx: CenterContext) -> None:
        """Arranca el ciclo. Siempre acaba emitiendo ``completed`` (sync o async)."""
        self._stop_all_center_timers()
        self._pending = None
        objs = list(objects or [])
        primary = pick_primary(objs)
        prefix = ctx.log_prefix or self._log_prefix
        if bool(ctx.has_stage) or self._test_service is not None:
            self._refresh_stage_xy(ctx)

        if primary is None or frame is None:
            kpi = kpi_fields_from_plan(None, "no_centroid", ctx)
            self._emit_center_log(kpi, ctx, prefix)
            self._emit_result(CenterResult(
                action="proceed",
                reason="no_centroid",
                objects=objs,
                primary=primary,
                kpi=kpi,
                frame=frame,
            ))
            return

        plan = plan_from_object(primary, frame, ctx)
        if plan is not None:
            init_gain = initial_step_gain(float(plan.e_px), float(ctx.step_gain))
            ctx.gain_x = float(init_gain)
            ctx.gain_y = float(init_gain)
            plan = plan_from_object(primary, frame, ctx) or plan
        action, reason = evaluate_center_pre_af(
            enabled=bool(ctx.enabled),
            fov_x_um=ctx.fov_x_um,
            fov_y_um=ctx.fov_y_um,
            plan=plan,
            has_stage=bool(ctx.has_stage),
        )
        kpi = kpi_fields_from_plan(plan, reason, ctx, action=action)
        extra = ""
        if plan is not None:
            extra = (
                f"img_c=({plan.img_cx:.1f},{plan.img_cy:.1f}) "
                f"roi_c=({plan.roi_cx:.1f},{plan.roi_cy:.1f}) "
                f"dpx=({plan.e_x_px:+.1f},{plan.e_y_px:+.1f}) "
                f"e_x={plan.e_x_px:+.1f} e_y={plan.e_y_px:+.1f} "
                f"xy_now=({ctx.stage_x_um:.0f},{ctx.stage_y_um:.0f}) "
                f"xy_tgt=({plan.target_x_um:.0f},{plan.target_y_um:.0f}) "
                f"dxy_um=({plan.dx_um:+.1f},{plan.dy_um:+.1f}) "
                f"would_clip={int(plan.would_clip)} limit={int(plan.limit_hit)} "
                f"clip={plan.clip_edges or '-'} "
                f"axes={int(plan.need_x)}{int(plan.need_y)} "
                f"sign=({int(ctx.sign_x):+d},{int(ctx.sign_y):+d}) "
                f"via=trajectory_1pt"
            )
        self._emit_center_log(kpi, ctx, prefix, extra=extra)

        if reason == "no_fov":
            logger.error(
                "%s AF_CENTER: FOV ausente o 0 — no hay escala px→µm; "
                "NO se hace jog a ciegas; AF en sitio",
                prefix,
            )
            self.status_message.emit(
                "❌ AF_CENTER: FOV=0 — no se mueve XY (configura FOV en Test)"
            )
        elif reason == "no_stage":
            logger.error(
                "%s AF_CENTER: stage no disponible (TestService/controladores); "
                "AF en sitio",
                prefix,
            )
            self.status_message.emit(
                "❌ AF_CENTER: stage no conectado — AF en sitio"
            )

        if action != "jog":
            self._emit_result(CenterResult(
                action="proceed",
                reason=reason,
                objects=objs,
                primary=primary,
                kpi=kpi,
                frame=frame,
            ))
            return

        ts = self._test_service
        if ts is None or not (
            hasattr(ts, "goto_xy_um") or hasattr(ts, "start_center_jog")
        ):
            logger.warning("%s AF_CENTER: sin goto_xy_um; AF en sitio", prefix)
            kpi["center_skip_reason"] = "no_stage"
            kpi["center_skipped"] = True
            kpi["center_attempted"] = False
            self._emit_center_log(kpi, ctx, prefix)
            self._emit_result(CenterResult(
                action="proceed",
                reason="no_stage",
                objects=objs,
                primary=primary,
                kpi=kpi,
                frame=frame,
            ))
            return

        xy_session = (float(ctx.stage_x_um), float(ctx.stage_y_um))
        max_attempts = resolve_max_center_steps(
            int(ctx.max_retries), clipped=bool(plan.would_clip)
        )
        clip_short = bool(plan.would_clip)
        if clip_short:
            short_plan = plan_from_object(
                primary, frame, ctx, clip_short_step=True
            )
            if short_plan is not None and short_plan.should_move:
                plan = short_plan
        kpi["tau_px"] = float(ctx.hysteresis_px)
        self._pending = {
            "phase": "jog",
            "objects": objs,
            "primary": primary,
            "frame": frame,
            "plan": plan,
            "ctx": ctx,
            "kpi": dict(kpi),
            "t0": time.perf_counter(),
            "attempt": 1,
            "e_px_pre": float(plan.e_px),
            "e_x_pre": float(plan.e_x_px),
            "e_y_pre": float(plan.e_y_px),
            "would_clip_pre": bool(plan.would_clip),
            "max_attempts": max_attempts,
            "prefix": prefix,
            "xy_session": xy_session,
            "xy_step": xy_session,
            "xy_cmd": (float(plan.target_x_um), float(plan.target_y_um)),
            "pre_objects": objs,
            "pre_primary": primary,
            "pre_frame": frame,
            "sign_flipped": False,
            "sign_flipped_x": False,
            "sign_flipped_y": False,
            "clip_short_used": bool(clip_short),
            "steps": [],
            "settle_armed": False,
            "motion_steps": 0,
            "xy_before": xy_session,
            "identity_lock": snapshot_identity_lock(primary),
            "wait_id": 0,
            "goto_via": "trajectory_1pt",
            "dual_retried": False,
            "lost_misses": 0,
            "expected_cmd": (float(plan.dx_cmd_um), float(plan.dy_cmd_um)),
        }
        try:
            setattr(primary, "_center_locked", True)
        except Exception:
            pass
        lock = self._pending.get("identity_lock")
        lock_area = float(getattr(lock, "area", 0) or 0) if lock is not None else 0.0
        self._publish_overlay(objs, primary, frame)
        self._append_step_row(
            i=0,
            x_um=float(ctx.stage_x_um),
            y_um=float(ctx.stage_y_um),
            dpx_x=float(plan.e_x_px),
            dpx_y=float(plan.e_y_px),
            dpx=float(plan.e_px),
            ok="move",
            xy_cmd=(float(plan.target_x_um), float(plan.target_y_um)),
            xy_read=xy_session,
            d_um_x=float(plan.dx_um),
            d_um_y=float(plan.dy_um),
            dir=command_dir_label(plan.dx_cmd_um, plan.dy_cmd_um),
        )
        logger.info(
            "%s AF_CENTER lock area=%.0f bbox=%s dpx=(%+.0f,%+.0f) Δcmd=(%+.1f,%+.1f)µm",
            prefix,
            lock_area,
            getattr(lock, "bbox", None),
            float(plan.e_x_px),
            float(plan.e_y_px),
            float(plan.dx_cmd_um),
            float(plan.dy_cmd_um),
        )
        self.status_message.emit(
            f"  ◎ AF_CENTER best-effort "
            f"{'clip' if plan.would_clip else 'en rango'} "
            f"{max_attempts} paso{'s' if max_attempts != 1 else ''} max "
            f"|Δpx|={plan.e_px:.0f} roi_clip={int(bool(plan.would_clip))} "
            f"gain=({ctx.gain_x:.2f},{ctx.gain_y:.2f}) "
            f"→ cmd=({plan.target_x_um:.0f},{plan.target_y_um:.0f})µm "
            f"xy_read=({xy_session[0]:.0f},{xy_session[1]:.0f}) "
            f"Δcmd=({plan.dx_cmd_um:+.1f},{plan.dy_cmd_um:+.1f})"
        )
        if not self._launch_goto(plan.target_x_um, plan.target_y_um, "center_candidate"):
            if self._pending is None:
                return
            logger.warning("%s AF_CENTER: jog no iniciado; AF en sitio", prefix)
            pending_objs = objs
            self._pending = None
            kpi["center_skip_reason"] = "jog_start_fail"
            kpi["center_skipped"] = True
            kpi["center_attempted"] = True
            self._emit_center_log(kpi, ctx, prefix)
            self._emit_result(CenterResult(
                action="proceed",
                reason="jog_start_fail",
                objects=pending_objs,
                primary=primary,
                kpi=kpi,
                frame=frame,
            ))

    def _launch_goto(self, x_um: float, y_um: float, reason: str) -> bool:
        ts = self._test_service
        if ts is None:
            return False
        if not (
            hasattr(ts, "goto_xy_um")
            or hasattr(ts, "start_center_jog")
            or hasattr(ts, "goto_xy_control_jog")
            or hasattr(ts, "start_dual_control")
        ):
            return False
        ctx = (self._pending or {}).get("ctx")
        plan = (self._pending or {}).get("plan")
        loop_s = LOOP_SAFETY_TIMEOUT_S
        t0 = float((self._pending or {}).get("t0") or time.perf_counter())
        if ctx is not None:
            loop_s = float(getattr(ctx, "loop_timeout_s", LOOP_SAFETY_TIMEOUT_S) or LOOP_SAFETY_TIMEOUT_S)
        remaining = max(0.5, loop_s - (time.perf_counter() - t0))
        if plan is not None:
            tmo = step_goto_timeout_s(
                float(getattr(plan, "dx_cmd_um", 0.0) or 0.0),
                float(getattr(plan, "dy_cmd_um", 0.0) or 0.0),
            )
        else:
            tmo = 2.0
        tmo = max(0.5, min(float(tmo), remaining, 3.0))
        if QThread.currentThread() == self.thread():
            self._execute_goto(float(x_um), float(y_um), str(reason), float(tmo))
            return self._pending is not None
        self._goto_requested.emit(float(x_um), float(y_um), str(reason), float(tmo))
        return True

    def _execute_goto(self, x_um: float, y_um: float, reason: str, tmo_s: float) -> None:
        if self._pending is None:
            return
        ts = self._test_service
        if ts is None:
            self._fail_goto_start()
            return
        via = str((self._pending or {}).get("goto_via") or "trajectory_1pt")
        xy_before = self._read_xy_now()[0]
        started = self._start_goto_move(
            ts, float(x_um), float(y_um), reason, float(tmo_s), via
        )
        if not started:
            self._fail_goto_start()
            return
        if self._pending is not None:
            wait_id = int(self._pending.get("wait_id") or 0) + 1
            self._pending["wait_id"] = wait_id
            self._pending["settle_armed"] = False
            self._pending["finish_scheduled"] = False
            self._pending["xy_cmd"] = (float(x_um), float(y_um))
            self._pending["xy_before"] = xy_before
            self._pending["step_tmo_s"] = float(tmo_s)
            self._pending["step_t0"] = time.perf_counter()
            self._pending["step_deadline_mono"] = time.perf_counter() + float(tmo_s)
            self._pending["step_timeout_busy"] = False
            self._pending["goto_via"] = via
            plan = self._pending.get("plan")
            dx_cmd = float(getattr(plan, "dx_cmd_um", 0.0) or 0.0) if plan is not None else (
                float(x_um) - float(xy_before[0])
            )
            dy_cmd = float(getattr(plan, "dy_cmd_um", 0.0) or 0.0) if plan is not None else (
                float(y_um) - float(xy_before[1])
            )
            self._pending["expected_cmd"] = (dx_cmd, dy_cmd)
            step_i = int(self._pending.get("motion_steps") or 0)
            self._log_center_pwm(i=step_i, dx_cmd=dx_cmd, dy_cmd=dy_cmd, dpx=float(getattr(plan, "e_px", 0.0) or 0.0) if plan else 0.0)
            prefix = self._pending.get("prefix") or self._log_prefix
            logger.info(
                "%s AF_CENTER goto start via=%s xy_read_before=(%.1f, %.1f) "
                "xy_cmd=(%.1f, %.1f) wait_s=%.2f wait_id=%d",
                prefix,
                via,
                float(xy_before[0]),
                float(xy_before[1]),
                float(x_um),
                float(y_um),
                float(tmo_s),
                wait_id,
            )
        self._arm_watchdog()
        self._arm_step_watchdog(float(tmo_s))
        self._start_live_overlay()

    def _start_goto_move(
        self, ts, x_um: float, y_um: float, reason: str, tmo_s: float, via: str
    ) -> bool:
        if via == "control_dual":
            if hasattr(ts, "goto_xy_control_jog"):
                return bool(ts.goto_xy_control_jog(float(x_um), float(y_um), reason=reason))
            if hasattr(ts, "start_dual_control"):
                return bool(ts.start_dual_control(float(x_um), float(y_um)))
            return False
        if hasattr(ts, "goto_xy_um"):
            try:
                return bool(
                    ts.goto_xy_um(
                        float(x_um),
                        float(y_um),
                        reason=reason,
                        point_timeout_s=float(tmo_s),
                    )
                )
            except TypeError:
                return bool(ts.goto_xy_um(float(x_um), float(y_um), reason=reason))
        if hasattr(ts, "start_center_jog"):
            return bool(ts.start_center_jog(float(x_um), float(y_um), reason=reason))
        return False

    def _fail_goto_start(self) -> None:
        pending = self._pending
        prefix = (pending or {}).get("prefix") or self._log_prefix
        logger.warning("%s AF_CENTER: jog no iniciado; AF en sitio", prefix)
        if pending is None:
            self._emit_result(CenterResult(action="proceed", reason="jog_start_fail"))
            return
        kpi = dict(pending.get("kpi") or {})
        kpi["center_skip_reason"] = "jog_start_fail"
        kpi["center_skipped"] = True
        kpi["center_attempted"] = True
        ctx = pending.get("ctx")
        if ctx is not None:
            self._emit_center_log(kpi, ctx, prefix)
        self._pending = None
        self._emit_result(CenterResult(
            action="proceed",
            reason="jog_start_fail",
            objects=list(pending.get("objects") or pending.get("pre_objects") or []),
            primary=pending.get("primary") or pending.get("pre_primary"),
            kpi=kpi,
            frame=pending.get("frame") or pending.get("pre_frame"),
        ))

    def _launch_jog(self, x_um: float, y_um: float, reason: str) -> bool:
        return self._launch_goto(x_um, y_um, reason)

    def _arm_watchdog(self) -> None:
        """Techo de TODO el lazo (75 s). El paso tiene su propio timeout."""
        pending = self._pending or {}
        ctx = pending.get("ctx")
        loop_s = LOOP_SAFETY_TIMEOUT_S
        if ctx is not None:
            loop_s = float(
                getattr(ctx, "loop_timeout_s", LOOP_SAFETY_TIMEOUT_S)
                or LOOP_SAFETY_TIMEOUT_S
            )
        t0 = float(pending.get("t0") or time.perf_counter())
        remaining_ms = int(round(max(1.0, loop_s - (time.perf_counter() - t0)) * 1000.0))
        self._watchdog.stop()
        self._watchdog.start(max(1000, remaining_ms))

    def _arm_step_watchdog(self, tmo_s: float) -> None:
        """Corta ESTE Δcmd. Deadline + QTimer; no aborta el lazo."""
        ms = int(round(max(0.5, min(float(tmo_s), 3.0)) * 1000.0))
        self._step_watchdog.stop()
        self._step_watchdog.start(ms)

    def _stop_all_center_timers(self) -> None:
        self._stop_watchdog()
        self._stop_step_watchdog()
        self._stop_live_overlay()
        self._stop_settle_timer()

    def _stop_watchdog(self) -> None:
        if self._watchdog.isActive():
            self._watchdog.stop()

    def _stop_step_watchdog(self) -> None:
        if self._step_watchdog.isActive():
            self._step_watchdog.stop()

    def _stop_settle_timer(self) -> None:
        if self._settle_timer.isActive():
            self._settle_timer.stop()

    def _schedule_finish(self, ms: int) -> None:
        pending = self._pending
        if pending is not None:
            pending["finish_scheduled"] = True
        self._stop_step_watchdog()
        self._stop_live_overlay()
        self._finish_requested.emit(max(0, int(ms)))

    def _on_finish_requested(self, ms: int) -> None:
        if self._pending is None:
            return
        self._stop_settle_timer()
        if int(ms) <= 0:
            self._finish_after_settle()
            return
        self._settle_timer.start(int(ms))

    def _start_live_overlay(self) -> None:
        if not self._live_overlay.isActive():
            self._live_overlay.start()

    def _stop_live_overlay(self) -> None:
        if self._live_overlay.isActive():
            self._live_overlay.stop()

    def _on_live_overlay(self) -> None:
        """Cruce magenta del lock; no cambia el candidato del servo ni persigue debris."""
        if self._pending is None:
            self._stop_live_overlay()
            return
        if str(self._pending.get("phase") or "") != "jog":
            return
        deadline = self._pending.get("step_deadline_mono")
        if deadline is not None and time.perf_counter() >= float(deadline):
            self._on_step_timeout()
            return
        frame_bgr, objects, matched = self._redetect()
        if frame_bgr is None:
            return
        pending = self._pending
        if matched is None:
            fallback = similar_area_or_iou_match(objects, pending.get("identity_lock"))
            if fallback is None:
                return
            matched = fallback
        self._publish_overlay(objects, matched, frame_bgr)
        pwm_pair, _pwm_lbl = self._pwm_snapshot()
        if pwm_pair[0] is not None:
            pending["pwm_seen"] = pwm_pair
        centroid = object_centroid_px(matched)
        if centroid is None:
            return
        fh, fw = int(frame_bgr.shape[0]), int(frame_bgr.shape[1])
        _ex, _ey, e_post = pixel_offset_from_center(centroid[0], centroid[1], fw, fh)
        ctx: CenterContext = pending["ctx"]
        tau = float(ctx.hysteresis_px)
        if e_post <= tau:
            logger.info(
                "%s AF_CENTER coincidencia en caza |Δpx|=%.0f ≤ %.0f — settle, no más goto",
                pending.get("prefix") or self._log_prefix,
                float(e_post),
                tau,
            )
            self._freeze_xy("center_coincidence")
            pending["timeout"] = False
            pending["jog_status"] = "centroid_match"
            if not bool(pending.get("settle_armed")):
                pending["settle_armed"] = True
                self._stop_step_watchdog()
                self._stop_watchdog()
                self._stop_live_overlay()
                self._schedule_finish(max(0, int(ctx.settle_ms)))

    def _on_jog_watchdog(self) -> None:
        if self._pending is None:
            return
        logger.warning(
            "%s AF_CENTER loop_timeout=%.0fs — AF best-effort (objeto en campo)",
            self._pending.get("prefix") or self._log_prefix,
            float(
                getattr(
                    (self._pending.get("ctx") if self._pending else None),
                    "loop_timeout_s",
                    LOOP_SAFETY_TIMEOUT_S,
                )
                or LOOP_SAFETY_TIMEOUT_S
            ),
        )
        self._freeze_xy("center_loop_timeout")
        self._proceed_in_place("loop_timeout")

    def _on_step_timeout(self) -> None:
        """El Δcmd de este paso venció: freeze XY, aceptar posición, seguir el lazo."""
        pending = self._pending
        if pending is None:
            return
        if bool(pending.get("settle_armed")) and bool(pending.get("finish_scheduled")):
            return
        if str(pending.get("phase") or "jog") not in ("jog",):
            return
        if bool(pending.get("step_timeout_busy")):
            return
        pending["step_timeout_busy"] = True
        pending["step_deadline_mono"] = None
        self._stop_step_watchdog()
        wait_s, xy_before, xy_after, moved_um = self._step_wait_metrics()
        via = str(pending.get("goto_via") or "trajectory_1pt")
        prefix = pending.get("prefix") or self._log_prefix
        moved = moved_um >= float(STEP_MOVED_CONFIRM_UM)
        logger.warning(
            "%s AF_CENTER step_timeout=%.1fs xy_read_before=(%.1f, %.1f) "
            "xy_read_after=(%.1f, %.1f) moved_um=%.1f wait_s=%.2f via=%s%s",
            prefix,
            float(pending.get("step_tmo_s") or 0.0),
            float(xy_before[0]),
            float(xy_before[1]),
            float(xy_after[0]),
            float(xy_after[1]),
            float(moved_um),
            float(wait_s),
            via,
            " goto_failed" if not moved else "",
        )
        self.status_message.emit(
            f"  ⏱ step_timeout wait_s={wait_s:.2f} xy_read_before="
            f"({xy_before[0]:.0f},{xy_before[1]:.0f}) xy_read_after="
            f"({xy_after[0]:.0f},{xy_after[1]:.0f}) moved_um={moved_um:.1f}"
            f"{' goto_failed' if not moved else ''}"
        )
        pending["closed_wait_id"] = int(pending.get("wait_id") or 0)
        ts = self._test_service
        if ts is not None and hasattr(ts, "accept_center_step_timeout"):
            try:
                ts.accept_center_step_timeout()
            except Exception:
                pass
        self._freeze_xy("center_step_timeout")
        if self._pending is None:
            return
        pending = self._pending
        pending["timeout"] = True
        pending["jog_status"] = "step_timeout"
        pending["jog_xy"] = xy_after
        if (not moved) and (not bool(pending.get("dual_retried"))):
            cmd = pending.get("xy_cmd") or xy_after
            pending["dual_retried"] = True
            pending["goto_via"] = "control_dual"
            pending["settle_armed"] = False
            pending["finish_scheduled"] = False
            logger.warning(
                "%s AF_CENTER goto_failed moved_um=%.1f — retry via=control_dual "
                "xy_cmd=(%.1f, %.1f)",
                prefix,
                float(moved_um),
                float(cmd[0]),
                float(cmd[1]),
            )
            self.status_message.emit(
                f"  ↻ goto_failed moved_um={moved_um:.1f} — retry control dual "
                f"→ ({float(cmd[0]):.0f},{float(cmd[1]):.0f})µm"
            )
            if not self._launch_goto(float(cmd[0]), float(cmd[1]), "center_control_jog"):
                if self._pending is None:
                    return
                pending["settle_armed"] = True
                self._schedule_finish(0)
            return
        pending["settle_armed"] = True
        ctx = pending.get("ctx")
        settle_ms = min(150, max(0, int(getattr(ctx, "settle_ms", 0) or 0))) if ctx else 0
        self._schedule_finish(settle_ms)

    def _step_wait_metrics(self):
        pending = self._pending or {}
        xy_before = pending.get("xy_before") or pending.get("xy_step") or (0.0, 0.0)
        xy_after, _residual = self._read_xy_now()
        moved_um = hypot(
            float(xy_after[0]) - float(xy_before[0]),
            float(xy_after[1]) - float(xy_before[1]),
        )
        t0 = float(pending.get("step_t0") or pending.get("t0") or time.perf_counter())
        wait_s = max(0.0, time.perf_counter() - t0)
        return wait_s, (float(xy_before[0]), float(xy_before[1])), xy_after, moved_um

    def _freeze_xy(self, reason: str) -> None:
        ts = self._test_service
        if ts is None:
            return
        if hasattr(ts, "pause_xy_for_capture"):
            try:
                ts.pause_xy_for_capture(reason)
            except Exception:
                pass

    def _on_trajectory_point_reached(self, index, x, y, status) -> None:
        del index
        if self._pending is None:
            return
        ts = self._test_service
        if ts is not None and callable(getattr(ts, "_mesh_in_progress", None)):
            if ts._mesh_in_progress():
                return
        self._on_jog_reached(x, y, status)

    def _alive(self) -> bool:
        if self._pending is None:
            return False
        if self._still_valid is not None and not self._still_valid():
            return False
        return True

    def _on_jog_reached(self, x: float, y: float, status: str) -> None:
        if self._pending is None:
            return
        if bool(self._pending.get("settle_armed")):
            return
        wait_id = int(self._pending.get("wait_id") or 0)
        if int(self._pending.get("closed_wait_id") or -1) == wait_id:
            return
        if not self._alive():
            self._fail_safe_complete("stale")
            return
        wait_s, xy_before, xy_after, moved_um = self._step_wait_metrics()
        via = str(self._pending.get("goto_via") or "trajectory_1pt")
        moved = moved_um >= float(STEP_MOVED_CONFIRM_UM)
        logger.info(
            "%s AF_CENTER wait_done xy_read_before=(%.1f, %.1f) "
            "xy_read_after=(%.1f, %.1f) moved_um=%.1f wait_s=%.2f via=%s %s%s",
            self._pending["prefix"],
            float(xy_before[0]),
            float(xy_before[1]),
            float(xy_after[0]),
            float(xy_after[1]),
            float(moved_um),
            float(wait_s),
            via,
            status,
            " goto_failed" if not moved else "",
        )
        self.status_message.emit(
            f"  ✓ wait_s={wait_s:.2f} xy_read_before=({xy_before[0]:.0f},{xy_before[1]:.0f}) "
            f"xy_read_after=({xy_after[0]:.0f},{xy_after[1]:.0f}) moved_um={moved_um:.1f}"
            f"{' goto_failed' if not moved else ''}"
        )
        if (not moved) and (not bool(self._pending.get("dual_retried"))):
            self._pending["closed_wait_id"] = wait_id
            self._pending["timeout"] = True
            self._on_step_timeout()
            return
        self._pending["settle_armed"] = True
        self._pending["closed_wait_id"] = wait_id
        self._stop_step_watchdog()
        self._stop_live_overlay()
        timeout = "t/o" in str(status).lower() or "timeout" in str(status).lower()
        self._pending["timeout"] = timeout
        self._pending["jog_status"] = status
        self._pending["jog_xy"] = xy_after
        self._pending["residual_um"] = None
        ctx: CenterContext = self._pending["ctx"]
        phase = str(self._pending.get("phase") or "jog")
        settle_ms = 0 if phase in ("revert", "restore") else max(0, int(ctx.settle_ms))
        if timeout:
            logger.warning(
                "%s trajectory_point_reached (center) timeout — xy_read=(%.1f, %.1f) "
                "moved_um=%.1f — CONTINÚA el lazo (no abort)",
                self._pending["prefix"],
                xy_after[0],
                xy_after[1],
                moved_um,
            )
            settle_ms = min(settle_ms, 150)
        self._schedule_finish(settle_ms)

    def _read_xy_now(self):
        pending = self._pending or {}
        ctx = pending.get("ctx")
        fallback = (
            float(getattr(ctx, "stage_x_um", 0.0) or 0.0),
            float(getattr(ctx, "stage_y_um", 0.0) or 0.0),
        )
        ts = self._test_service
        residual = None
        if ts is not None and hasattr(ts, "read_current_position_um"):
            sx, sy, ex, ey = ts.read_current_position_um(fallback[0], fallback[1])
            if sx is not None:
                fallback = (float(sx), fallback[1])
                if ctx is not None:
                    ctx.stage_x_um = float(sx)
            if sy is not None:
                fallback = (fallback[0], float(sy))
                if ctx is not None:
                    ctx.stage_y_um = float(sy)
            if sx is not None or sy is not None:
                residual = (float(ex), float(ey))
        jog = pending.get("jog_xy")
        if residual is None and jog is not None:
            fallback = (float(jog[0]), float(jog[1]))
        return fallback, residual

    def _fail_safe_complete(self, reason: str) -> None:
        """Desbloquea GUI/XY aunque el jog quede a medias."""
        self._stop_all_center_timers()
        pending = self._pending
        self._pending = None
        if pending is None:
            self._emit_result(CenterResult(action="proceed", reason=reason))
            return
        kpi = dict(pending.get("kpi") or {})
        kpi["center_skip_reason"] = reason
        kpi["center_attempted"] = True
        action = "abort" if reason == "lost_lock" else "proceed"
        self._emit_result(CenterResult(
            action=action,
            reason=reason,
            objects=list(pending.get("pre_objects") or pending.get("objects") or []),
            primary=pending.get("pre_primary") or pending.get("primary"),
            kpi=kpi,
            frame=pending.get("pre_frame") or pending.get("frame"),
            steps=list(pending.get("steps") or []),
        ))

    def _proceed_in_place(self, reason: str, objects=None, primary=None, frame=None) -> None:
        pending = self._pending or {}
        kpi = dict(pending.get("kpi") or {})
        kpi["center_skip_reason"] = reason
        kpi["t_center_xy"] = time.perf_counter() - float(pending.get("t0") or time.perf_counter())
        objs = list(objects if objects is not None else (
            pending.get("objects") or pending.get("pre_objects") or []
        ))
        prim = primary if primary is not None else (
            pending.get("primary") or pending.get("pre_primary")
        )
        frm = frame if frame is not None else (
            pending.get("frame") or pending.get("pre_frame")
        )
        steps = list(pending.get("steps") or [])
        ctx = pending.get("ctx")
        tau = float(getattr(ctx, "hysteresis_px", 40.0) or 40.0) if ctx is not None else 40.0
        residual = kpi.get("xy_offset_post_px")
        if residual is None:
            residual = kpi.get("xy_offset_pre_px")
        has_obj = bool(objs) or prim is not None
        if not can_start_zscan(
            action="proceed",
            reason=reason,
            center_attempted=True,
            residual_px=residual if residual is not None else None,
            tau_px=tau,
            objects_found=has_obj,
        ):
            self._abort_center("lost_lock" if reason == "lost_lock" else "redetect_fail")
            return
        if reason not in ("centered", "already_centered"):
            logger.info(
                "%s AF_CENTER best-effort → Z-scan |Δpx|=%s tau=%.0f reason=%s",
                (pending.get("prefix") or self._log_prefix) if pending else self._log_prefix,
                residual if residual is not None else "na",
                tau,
                reason,
            )
            self.status_message.emit(
                f"  ◎ AF_CENTER best-effort |Δpx|="
                f"{residual if residual is not None else 'na'} — Z-scan"
            )
        self._freeze_xy("center_done_af")
        self._stop_all_center_timers()
        self._pending = None
        self._emit_result(CenterResult(
            action="proceed",
            reason=reason,
            objects=objs,
            primary=prim,
            kpi=kpi,
            frame=frm,
            steps=steps,
        ))

    def _start_restore(self, reason: str) -> None:
        pending = self._pending
        if pending is None:
            self._emit_result(CenterResult(action="proceed", reason=reason))
            return
        xy = pending.get("xy_session") or pending.get("xy_step")
        pending["phase"] = "restore"
        pending["restore_reason"] = reason
        if xy is None:
            self._proceed_in_place(reason)
            return
        logger.info(
            "%s AF_CENTER restore XY → (%.1f, %.1f)µm (%s)",
            pending["prefix"],
            float(xy[0]),
            float(xy[1]),
            reason,
        )
        self.status_message.emit(
            f"  ↩ Restore XY ({reason}) → ({float(xy[0]):.0f}, {float(xy[1]):.0f})µm"
        )
        if not self._launch_jog(xy[0], xy[1], f"center_restore_{reason}"):
            self._proceed_in_place(reason)

    def _start_revert_then_retry(self, flip_x: bool, flip_y: bool) -> None:
        """Flip de signo del eje malo y continuar desde XY leído. Sin revertir el bueno."""
        pending = self._pending
        if pending is None:
            return
        self._apply_sign_flip(bool(flip_x), bool(flip_y))
        self._retry_from_current(pending.get("primary"), pending.get("frame"))

    def _apply_sign_flip(self, flip_x: bool, flip_y: bool) -> None:
        pending = self._pending
        if pending is None:
            return
        ctx: CenterContext = pending["ctx"]
        if flip_x and not bool(pending.get("sign_flipped_x")):
            ctx.sign_x = -1 if int(ctx.sign_x) >= 0 else 1
            pending["sign_flipped_x"] = True
        if flip_y and not bool(pending.get("sign_flipped_y")):
            ctx.sign_y = -1 if int(ctx.sign_y) >= 0 else 1
            pending["sign_flipped_y"] = True
        pending["sign_flipped"] = bool(
            pending.get("sign_flipped_x") or pending.get("sign_flipped_y")
        )
        pending["ctx"] = ctx
        self.sign_changed.emit(int(ctx.sign_x), int(ctx.sign_y))
        logger.warning(
            "%s AF_CENTER sign_flip x=%d y=%d → sign=(%+d,%+d) — continuar desde xy_read",
            pending["prefix"],
            int(flip_x),
            int(flip_y),
            int(ctx.sign_x),
            int(ctx.sign_y),
        )

    def _retry_from_current(self, obj, frame) -> None:
        pending = self._pending
        if pending is None:
            return
        ctx: CenterContext = pending["ctx"]
        xy_now, residual = self._read_xy_now()
        ctx.stage_x_um = float(xy_now[0])
        ctx.stage_y_um = float(xy_now[1])
        pending["xy_step"] = xy_now
        pending["ctx"] = ctx
        pending["phase"] = "jog"
        pending["timeout"] = False
        if obj is None or frame is None:
            frame_bgr, objects, matched = self._redetect(retries=LOCK_REDETECT_TRIES)
            if matched is not None:
                pending["objects"] = objects
                pending["primary"] = matched
                pending["frame"] = frame_bgr
                obj, frame = matched, frame_bgr
            else:
                pending["lost_misses"] = int(pending.get("lost_misses") or 0) + 1
                if should_declare_lost_lock(
                    matched=None,
                    objects=objects,
                    lock=pending.get("identity_lock"),
                    consecutive_misses=int(pending.get("lost_misses") or 0),
                    miss_limit=LOCK_MISS_LIMIT,
                ):
                    self._abort_lost_lock()
                    return
                if frame_bgr is not None:
                    pending["frame"] = frame_bgr
                self._term(
                    f"AF_CENTER_IMG  i={int(pending.get('motion_steps') or 0)} "
                    f"dpx=keep |dpx|=na  lock_area="
                    f"{float(getattr(pending.get('identity_lock'), 'area', 0) or 0):.0f} "
                    f"match=retry objects={len(objects or [])}"
                )
                if int(pending.get("motion_steps") or 0) >= int(pending.get("max_attempts") or 1):
                    self._proceed_in_place(
                        "best_effort",
                        objects=pending.get("objects") or pending.get("pre_objects"),
                        primary=pending.get("primary") or pending.get("pre_primary"),
                        frame=pending.get("frame") or pending.get("pre_frame"),
                    )
                    return
                self._schedule_finish(80)
                return
        motion_steps = int(pending.get("motion_steps") or 0)
        max_attempts = int(pending.get("max_attempts") or 1)
        clip_short = bool(pending.get("would_clip_pre")) and not bool(
            pending.get("clip_short_used")
        )
        retry_plan = plan_from_object(
            obj, frame, ctx, clip_short_step=clip_short
        )
        if clip_short:
            pending["clip_short_used"] = True
        if retry_plan is None:
            if obj is not None or pending.get("primary") or pending.get("pre_primary"):
                self._proceed_in_place(
                    "best_effort",
                    objects=pending.get("objects") or pending.get("pre_objects"),
                    primary=obj or pending.get("primary") or pending.get("pre_primary"),
                    frame=frame or pending.get("frame") or pending.get("pre_frame"),
                )
                return
            self._abort_center("redetect_fail")
            return
        if not retry_plan.should_move:
            self._proceed_in_place(
                "centered", objects=pending.get("objects"), primary=obj, frame=frame
            )
            return
        if motion_steps >= max_attempts:
            self._proceed_in_place(
                "best_effort", objects=pending.get("objects"), primary=obj, frame=frame
            )
            return
        pending["attempt"] = motion_steps + 1
        pending["plan"] = retry_plan
        pending["e_px_pre"] = float(retry_plan.e_px)
        pending["e_x_pre"] = float(retry_plan.e_x_px)
        pending["e_y_pre"] = float(retry_plan.e_y_px)
        pending["would_clip_pre"] = bool(retry_plan.would_clip)
        pending["xy_cmd"] = (float(retry_plan.target_x_um), float(retry_plan.target_y_um))
        pending["xy_before"] = xy_now
        pending["dual_retried"] = False
        pending["goto_via"] = "trajectory_1pt"
        pending["settle_armed"] = False
        pending["finish_scheduled"] = False
        pending["expected_cmd"] = (float(retry_plan.dx_cmd_um), float(retry_plan.dy_cmd_um))
        logger.info(
            "%s AF_CENTER step %d/%d Δcmd=(%+.1f,%+.1f)µm |Δpx|=%.0f "
            "sign=(%+d,%+d) gain=(%.2f,%.2f) xy_read=(%.0f,%.0f) residual=%s",
            pending["prefix"],
            pending["attempt"],
            max_attempts,
            retry_plan.dx_cmd_um,
            retry_plan.dy_cmd_um,
            float(retry_plan.e_px),
            int(ctx.sign_x),
            int(ctx.sign_y),
            float(ctx.gain_x if ctx.gain_x is not None else ctx.step_gain),
            float(ctx.gain_y if ctx.gain_y is not None else ctx.step_gain),
            xy_now[0],
            xy_now[1],
            residual,
        )
        self.status_message.emit(
            f"  ◎ Centro paso {pending['attempt']}/{max_attempts} "
            f"ΔXY=({retry_plan.dx_cmd_um:+.1f},{retry_plan.dy_cmd_um:+.1f})µm "
            f"|Δpx|={retry_plan.e_px:.0f} xy_read=({xy_now[0]:.0f},{xy_now[1]:.0f})"
        )
        if not self._launch_jog(
            retry_plan.target_x_um, retry_plan.target_y_um, "center_step"
        ):
            if self._pending is not None:
                self._proceed_in_place("jog_start_fail")

    def _finish_after_settle(self) -> None:
        if self._pending is None:
            return
        if not self._alive():
            self._fail_safe_complete("stale")
            return
        pending = self._pending
        phase = str(pending.get("phase") or "jog")
        if phase == "restore":
            self._finish_restore()
            return
        if phase == "revert":
            self._finish_revert()
            return
        self._finish_jog_step()

    def _finish_restore(self) -> None:
        pending = self._pending
        reason = str(pending.get("restore_reason") or "restore")
        self._proceed_in_place(reason)

    def _finish_revert(self) -> None:
        pending = self._pending
        frame_bgr, objects, largest = self._redetect(retries=LOCK_REDETECT_TRIES)
        if objects and largest is not None:
            pending["objects"] = objects
            pending["primary"] = largest
            pending["frame"] = frame_bgr
            pending["lost_misses"] = 0
            lock = pending.get("identity_lock")
            if lock is not None:
                pending["identity_lock"] = update_lock_geometry(lock, largest)
            self._publish_overlay(objects, largest, frame_bgr)
            self._retry_from_current(largest, frame_bgr)
            return
        pending["lost_misses"] = int(pending.get("lost_misses") or 0) + 1
        if should_declare_lost_lock(
            matched=largest,
            objects=objects,
            lock=pending.get("identity_lock"),
            consecutive_misses=int(pending.get("lost_misses") or 0),
            miss_limit=LOCK_MISS_LIMIT,
        ):
            self._abort_lost_lock()
            return
        self._retry_from_current(pending.get("primary"), pending.get("frame"))

    def _scorer_obj(self):
        parent_scorer = getattr(self, "_scorer", None)
        if callable(parent_scorer):
            return parent_scorer()
        return parent_scorer

    def _redetect(self, retries: int = 1):
        last_frame, last_objects, last_match = None, [], None
        prefix = (self._pending or {}).get("prefix") or self._log_prefix
        n_try = max(1, int(retries))
        for attempt in range(n_try):
            frame_bgr = None
            if self._grab_bgr_frame is not None:
                try:
                    frame_bgr = self._grab_bgr_frame()
                except Exception as exc:
                    logger.warning("%s AF_CENTER redetect: grab falló: %s", prefix, exc)
                    frame_bgr = None
            scorer = self._scorer_obj()
            if frame_bgr is None or scorer is None or not hasattr(scorer, "assess_image"):
                last_frame, last_objects, last_match = frame_bgr, [], None
            else:
                result = scorer.assess_image(frame_bgr)
                objects = self._filter_objects(result.objects if result else [])
                lock = (self._pending or {}).get("identity_lock")
                if lock is not None:
                    matched = match_locked_object(objects, lock)
                    if matched is None:
                        matched = similar_area_or_iou_match(objects, lock)
                else:
                    prev = (self._pending or {}).get("primary")
                    matched = pick_tracked_primary(objects, prev)
                last_frame, last_objects, last_match = frame_bgr, objects, matched
                if matched is not None:
                    return frame_bgr, objects, matched
            if attempt + 1 < n_try:
                app = QCoreApplication.instance()
                if app is not None:
                    app.processEvents()
        return last_frame, last_objects, last_match

    def _finish_jog_step(self) -> None:
        pending = self._pending
        ctx: CenterContext = pending["ctx"]
        prefix = pending["prefix"]
        t_center = time.perf_counter() - float(pending["t0"])
        kpi = dict(pending.get("kpi") or {})
        timeout = bool(pending.get("timeout"))

        xy_read, residual_um = self._read_xy_now()
        pending["jog_xy"] = xy_read
        ctx.stage_x_um = float(xy_read[0])
        ctx.stage_y_um = float(xy_read[1])
        xy_before = pending.get("xy_before") or pending.get("xy_step") or xy_read
        moved_x = float(xy_read[0]) - float(xy_before[0])
        moved_y = float(xy_read[1]) - float(xy_before[1])
        moved_um = hypot(moved_x, moved_y)
        moved = moved_um >= MOTION_EPS_UM
        expected = pending.get("expected_cmd") or (0.0, 0.0)
        exp_x, exp_y = float(expected[0]), float(expected[1])
        flip_x_enc, flip_y_enc = encoder_sign_error(moved_x, moved_y, exp_x, exp_y)
        step_i = int(pending.get("motion_steps") or 0)
        self._log_center_move(
            i=step_i,
            moved_x=moved_x,
            moved_y=moved_y,
            exp_x=exp_x,
            exp_y=exp_y,
            flip_x=flip_x_enc,
            flip_y=flip_y_enc,
            xy=xy_read,
        )
        wait_s = 0.0
        step_t0 = pending.get("step_t0")
        if step_t0 is not None:
            wait_s = max(0.0, time.perf_counter() - float(step_t0))
        logger.info(
            "%s AF_CENTER step_done xy_read_before=(%.1f, %.1f) "
            "xy_read_after=(%.1f, %.1f) moved_um=%.1f wait_s=%.2f via=%s%s",
            prefix,
            float(xy_before[0]),
            float(xy_before[1]),
            float(xy_read[0]),
            float(xy_read[1]),
            float(moved_um),
            float(wait_s),
            str(pending.get("goto_via") or "trajectory_1pt"),
            " goto_failed" if moved_um < float(STEP_MOVED_CONFIRM_UM) else "",
        )
        if moved:
            pending["motion_steps"] = int(pending.get("motion_steps") or 0) + 1
        motion_steps = int(pending.get("motion_steps") or 0)
        if (flip_x_enc or flip_y_enc) and not (
            (flip_x_enc and pending.get("sign_flipped_x"))
            or (flip_y_enc and pending.get("sign_flipped_y"))
        ):
            self._apply_sign_flip(flip_x_enc, flip_y_enc)
            ctx = pending["ctx"]

        frame_bgr, objects, largest = self._redetect(retries=LOCK_REDETECT_TRIES)
        found = largest is not None
        e_x = e_y = None
        e_post = None
        would_clip_post = False
        dx_um = dy_um = 0.0
        match_lbl = "ok" if found else "fail"
        if found:
            pending["lost_misses"] = 0
            centroid = object_centroid_px(largest)
            bbox = object_bbox(largest)
            if centroid is not None and bbox is not None and frame_bgr is not None:
                fh, fw = int(frame_bgr.shape[0]), int(frame_bgr.shape[1])
                e_x, e_y, e_post = pixel_offset_from_center(
                    centroid[0], centroid[1], fw, fh
                )
                dx_um, dy_um = pixel_offset_to_stage_um(
                    e_x, e_y, fw, fh, ctx.fov_x_um, ctx.fov_y_um,
                    sign_x=ctx.sign_x, sign_y=ctx.sign_y,
                )
                would_clip_post, _w = predict_static_window_clip(
                    bbox, fw, fh, int(ctx.pad_px)
                )
            lock = pending.get("identity_lock")
            if lock is not None:
                pending["identity_lock"] = update_lock_geometry(lock, largest)
            self._publish_overlay(objects, largest, frame_bgr)
        else:
            pending["lost_misses"] = int(pending.get("lost_misses") or 0) + 1
            similar = similar_area_or_iou_match(objects, pending.get("identity_lock"))
            if similar is not None:
                found = True
                largest = similar
                match_lbl = "area"
                pending["lost_misses"] = 0
                centroid = object_centroid_px(largest)
                bbox = object_bbox(largest)
                if centroid is not None and bbox is not None and frame_bgr is not None:
                    fh, fw = int(frame_bgr.shape[0]), int(frame_bgr.shape[1])
                    e_x, e_y, e_post = pixel_offset_from_center(
                        centroid[0], centroid[1], fw, fh
                    )
                    dx_um, dy_um = pixel_offset_to_stage_um(
                        e_x, e_y, fw, fh, ctx.fov_x_um, ctx.fov_y_um,
                        sign_x=ctx.sign_x, sign_y=ctx.sign_y,
                    )
                self._publish_overlay(objects, largest, frame_bgr)
        lock_area = float(getattr(pending.get("identity_lock"), "area", 0) or 0)
        dpx_log_x = float(e_x) if e_x is not None else float(pending.get("e_x_pre") or 0.0)
        dpx_log_y = float(e_y) if e_y is not None else float(pending.get("e_y_pre") or 0.0)
        dpx_log = float(e_post) if e_post is not None else hypot(dpx_log_x, dpx_log_y)
        self._log_center_img(
            i=motion_steps,
            dpx_x=dpx_log_x,
            dpx_y=dpx_log_y,
            dpx=dpx_log,
            lock_area=lock_area,
            match=match_lbl if found else "fail",
        )
        if found and e_x is not None and e_y is not None:
            gx0 = float(ctx.gain_x if ctx.gain_x is not None else ctx.step_gain)
            gy0 = float(ctx.gain_y if ctx.gain_y is not None else ctx.step_gain)
            ctx.gain_x, ctx.gain_y = apply_overshoot_gains(
                gx0, gy0,
                float(pending.get("e_x_pre") or 0.0),
                float(pending.get("e_y_pre") or 0.0),
                float(e_x),
                float(e_y),
            )
            pending["ctx"] = ctx
            row_ok = "done" if float(e_post) <= float(ctx.hysteresis_px) else "move"
        elif found:
            row_ok = "move"
        else:
            row_ok = "detect_fail"
        dir_lbl = command_dir_label(exp_x, exp_y)
        _pwm_pair, pwm_lbl = self._pwm_snapshot(exp_x, exp_y)
        self._append_step_row(
            i=len(pending.get("steps") or []),
            x_um=float(xy_read[0]),
            y_um=float(xy_read[1]),
            dpx_x=dpx_log_x if found else dpx_log_x,
            dpx_y=dpx_log_y if found else dpx_log_y,
            dpx=dpx_log if found else dpx_log,
            ok=row_ok,
            xy_cmd=pending.get("xy_cmd"),
            xy_read=(float(xy_read[0]), float(xy_read[1])),
            residual_um=residual_um,
            d_um_x=float(dx_um),
            d_um_y=float(dy_um),
            dir=dir_lbl,
            pwm=pwm_lbl,
            moved_x_um=moved_x,
            moved_y_um=moved_y,
        )
        if not found:
            if should_declare_lost_lock(
                matched=None,
                objects=objects,
                lock=pending.get("identity_lock"),
                consecutive_misses=int(pending.get("lost_misses") or 0),
                miss_limit=LOCK_MISS_LIMIT,
            ):
                self._abort_lost_lock()
                return
            self._retry_from_current(
                pending.get("primary"),
                frame_bgr if frame_bgr is not None else pending.get("frame"),
            )
            return
        if residual_um is not None:
            logger.info(
                "%s POINT_METRICS center residual=(%.1f,%.1f)µm xy_read=(%.1f,%.1f) "
                "xy_cmd=%s moved=%d motion_steps=%d/%d",
                prefix,
                float(residual_um[0]),
                float(residual_um[1]),
                float(xy_read[0]),
                float(xy_read[1]),
                pending.get("xy_cmd"),
                int(moved),
                motion_steps,
                int(pending.get("max_attempts") or 1),
            )

        decision = decide_post_jog(
            objects_found=found,
            e_px_pre=float(pending.get("e_px_pre") or 0.0),
            e_px_post=e_post,
            e_x_pre=float(pending.get("e_x_pre") or 0.0),
            e_y_pre=float(pending.get("e_y_pre") or 0.0),
            e_x_post=e_x,
            e_y_post=e_y,
            would_clip_post=bool(would_clip_post),
            would_clip_pre=bool(pending.get("would_clip_pre", False)),
            timeout=timeout,
            attempt=int(motion_steps),
            max_attempts=int(pending.get("max_attempts") or 1),
            tau_px=float(ctx.hysteresis_px),
            sign_flipped=bool(pending.get("sign_flipped")),
            sign_flipped_x=bool(pending.get("sign_flipped_x")),
            sign_flipped_y=bool(pending.get("sign_flipped_y")),
        )
        # Timeout de un paso: forzar retry, nunca return not_centered.
        if timeout and decision.action not in ("proceed", "stop") and found:
            if decision.action == "abort_not_centered" and motion_steps < int(
                pending.get("max_attempts") or 1
            ):
                decision = decide_post_jog(
                    objects_found=found,
                    e_px_pre=float(pending.get("e_px_pre") or 0.0),
                    e_px_post=e_post,
                    e_x_pre=float(pending.get("e_x_pre") or 0.0),
                    e_y_pre=float(pending.get("e_y_pre") or 0.0),
                    e_x_post=e_x,
                    e_y_post=e_y,
                    would_clip_post=bool(would_clip_post),
                    timeout=False,
                    attempt=max(0, motion_steps - 1),
                    max_attempts=int(pending.get("max_attempts") or 1),
                    tau_px=float(ctx.hysteresis_px),
                    sign_flipped=bool(pending.get("sign_flipped")),
                    sign_flipped_x=bool(pending.get("sign_flipped_x")),
                    sign_flipped_y=bool(pending.get("sign_flipped_y")),
                )
        sign_flip = decision.action == "revert_flip"
        kpi.update({
            "xy_offset_post_px": float(e_post) if e_post is not None else None,
            "xy_offset_post_um": hypot(dx_um, dy_um) if e_post is not None else None,
            "center_attempted": True,
            "center_success": (
                decision.action == "proceed"
                and decision.reason == "centered"
                and found
            ),
            "center_skipped": False,
            "center_skip_reason": decision.reason,
            "tau_px": float(ctx.hysteresis_px),
            "roi_clipped": bool(would_clip_post) if found else True,
            "roi_frame_margin_px": (
                roi_frame_margin_px(object_bbox(largest), frame_bgr.shape[1], frame_bgr.shape[0])
                if found and frame_bgr is not None and object_bbox(largest)
                else None
            ),
            "t_center_xy": t_center,
            "sign_flip_suspect": sign_flip or bool(pending.get("sign_flipped")),
        })
        if e_x is not None and e_y is not None:
            e_post_txt = f"e_post_px={e_post:+.1f} e_post=({e_x:+.1f},{e_y:+.1f})px"
        else:
            e_post_txt = "e_post_px=na e_post=na"
        extra = (
            f"{e_post_txt} "
            f"would_clip_post={int(bool(would_clip_post and found))} "
            f"sign_flip={int(sign_flip)} timeout={int(timeout)} "
            f"action={decision.action} success={int(bool(kpi.get('center_success')))} "
            f"step={motion_steps}/{int(pending.get('max_attempts') or 1)} "
            f"xy_read=({xy_read[0]:.0f},{xy_read[1]:.0f}) "
            f"gain=({float(ctx.gain_x if ctx.gain_x is not None else ctx.step_gain):.2f},"
            f"{float(ctx.gain_y if ctx.gain_y is not None else ctx.step_gain):.2f}) "
            f"roi_clip={int(bool(would_clip_post) if found else 1)}"
        )
        self._emit_center_log(kpi, ctx, prefix, extra=extra)
        pending["kpi"] = dict(kpi)
        if found:
            pending["objects"] = objects
            pending["primary"] = largest
            pending["frame"] = frame_bgr
            pending["would_clip_pre"] = bool(would_clip_post)

        if decision.action == "revert_flip":
            self._start_revert_then_retry(decision.flip_x, decision.flip_y)
            return
        if decision.action == "retry":
            self._retry_from_current(
                largest if found else pending.get("primary"),
                frame_bgr or pending.get("frame"),
            )
            return
        if decision.action in ("stop", "abort_not_centered"):
            if found or pending.get("primary") or pending.get("pre_primary"):
                self._proceed_in_place(
                    "best_effort" if found else decision.reason,
                    objects=objects if found else pending.get("pre_objects"),
                    primary=largest if found else pending.get("pre_primary"),
                    frame=frame_bgr if found else pending.get("pre_frame"),
                )
                return
            self._abort_center(decision.reason)
            return
        self._proceed_in_place(
            decision.reason,
            objects=objects if found else pending.get("pre_objects"),
            primary=largest if found else pending.get("pre_primary"),
            frame=frame_bgr if found else pending.get("pre_frame"),
        )

    def set_scorer(self, scorer) -> None:
        """Scorer o getter. Necesario para re-detectar tras el settle."""
        self._scorer = scorer

    def _publish_overlay(self, objects, primary, frame) -> None:
        del objects, frame
        if primary is None:
            return
        try:
            setattr(primary, "_center_locked", True)
        except Exception:
            pass
        self.overlay_update.emit([primary])
        app = QCoreApplication.instance()
        if app is not None:
            app.processEvents()

    def _log_center_pwm(self, *, i: int, dx_cmd: float, dy_cmd: float, dpx: float) -> None:
        dir_lbl = command_dir_label(dx_cmd, dy_cmd)
        _pair, pwm_lbl = self._pwm_snapshot(dx_cmd, dy_cmd)
        self._term(
            f"AF_CENTER_PWM i={int(i)} dir={dir_lbl}  pwm={pwm_lbl}  "
            f"Δcmd_um=({float(dx_cmd):+.1f},{float(dy_cmd):+.1f})  |dpx|={float(dpx):.0f}"
        )

    def _log_center_move(
        self,
        *,
        i: int,
        moved_x: float,
        moved_y: float,
        exp_x: float,
        exp_y: float,
        flip_x: bool,
        flip_y: bool,
        xy,
    ) -> None:
        _pair, pwm_lbl = self._pwm_snapshot(exp_x, exp_y)
        self._term(
            f"AF_CENTER_MOVE i={int(i)} moved_um=({float(moved_x):+.1f},{float(moved_y):+.1f})  "
            f"esperado=({float(exp_x):+.1f},{float(exp_y):+.1f})  "
            f"sign_err={sign_err_label(flip_x, flip_y)}  "
            f"xy=({float(xy[0]):.0f},{float(xy[1]):.0f})  pwm={pwm_lbl}"
        )

    def _log_center_img(
        self,
        *,
        i: int,
        dpx_x: float,
        dpx_y: float,
        dpx: float,
        lock_area: float,
        match: str,
    ) -> None:
        self._term(
            f"AF_CENTER_IMG  i={int(i)} dpx=({float(dpx_x):+.0f},{float(dpx_y):+.0f}) "
            f"|dpx|={float(dpx):.0f}  lock_area={float(lock_area):.0f}  match={match}"
        )

    def _append_step_row(
        self,
        *,
        i: int,
        x_um: float,
        y_um: float,
        dpx_x: float,
        dpx_y: float,
        dpx: float,
        ok: str,
        xy_cmd=None,
        xy_read=None,
        residual_um=None,
        d_um_x: float = 0.0,
        d_um_y: float = 0.0,
        dir: str = "",
        pwm: str = "",
        moved_x_um: float = 0.0,
        moved_y_um: float = 0.0,
    ) -> None:
        pending = self._pending
        if pending is None:
            return
        row = CenterStepRow(
            i=int(i),
            x_um=float(x_um),
            y_um=float(y_um),
            dpx_x=float(dpx_x),
            dpx_y=float(dpx_y),
            dpx=float(dpx),
            ok=str(ok),
            xy_cmd=xy_cmd,
            xy_read=xy_read,
            residual_um=residual_um,
            d_um_x=float(d_um_x),
            d_um_y=float(d_um_y),
            dir=str(dir or ""),
            pwm=str(pwm or ""),
            moved_x_um=float(moved_x_um),
            moved_y_um=float(moved_y_um),
        )
        steps = list(pending.get("steps") or [])
        steps.append(row)
        pending["steps"] = steps
        line = (
            f"AF_CENTER_STEP i={row.i} xy_cmd="
            f"{'na' if row.xy_cmd is None else f'({float(row.xy_cmd[0]):.0f},{float(row.xy_cmd[1]):.0f})'} "
            f"xy_read="
            f"{'na' if row.xy_read is None else f'({float(row.xy_read[0]):.0f},{float(row.xy_read[1]):.0f})'} "
            f"dpx=({row.dpx_x:+.0f},{row.dpx_y:+.0f}) "
            f"dµm=({row.d_um_x:+.1f},{row.d_um_y:+.1f}) "
            f"dir={row.dir or '-'} pwm={row.pwm or '-'} "
            f"moved=({row.moved_x_um:+.1f},{row.moved_y_um:+.1f}) ok={row.ok}"
        )
        if row.residual_um is not None:
            line += (
                f" residual_um=({float(row.residual_um[0]):+.1f},"
                f"{float(row.residual_um[1]):+.1f})"
            )
        self._term(line)

    def _dump_steps_table(self, pending=None) -> None:
        blob = pending if pending is not None else self._pending
        rows = list((blob or {}).get("steps") or [])
        if not rows:
            return
        table = format_center_steps_table(rows)
        prefix = (blob or {}).get("prefix") or self._log_prefix
        for line in table.splitlines():
            logger.info("%s %s", prefix, line)
            self.status_message.emit(line)
            ts = self._test_service
            if ts is not None and hasattr(ts, "log_message"):
                ts.log_message.emit(line)

    def _abort_lost_lock(self) -> None:
        pending = self._pending
        lock = (pending or {}).get("identity_lock")
        area_was = float(getattr(lock, "area", 0) or 0) if lock is not None else 0.0
        prefix = (pending or {}).get("prefix") or self._log_prefix
        logger.warning(
            "%s AF_CENTER lost_lock area_was=%.0f — STOP motores, no AF",
            prefix,
            area_was,
        )
        self.status_message.emit(f"❌ lost_lock area_was={area_was:.0f}")
        self._freeze_xy("lost_lock")
        self._abort_center("lost_lock")

    def _abort_center(self, reason: str) -> None:
        pending = self._pending
        prefix = (pending or {}).get("prefix") or self._log_prefix
        kpi = dict((pending or {}).get("kpi") or {})
        kpi["center_attempted"] = True
        kpi["center_success"] = False
        kpi["center_skipped"] = False
        kpi["center_skip_reason"] = reason
        residual = kpi.get("xy_offset_post_px")
        steps = list((pending or {}).get("steps") or [])
        if steps and steps[-1].ok == "move":
            steps[-1].ok = "lost" if reason in ("redetect_fail", "lost_lock") else "fail"
        logger.warning(
            "%s AF_CENTER abort %s residual_px=%s — no AF (lost_lock / sin objeto)",
            prefix,
            reason,
            residual,
        )
        self.status_message.emit(
            f"❌ AF_CENTER: {reason} — no Z-scan (objeto perdido o ausente)"
        )
        self._stop_all_center_timers()
        self._pending = None
        if pending is None:
            self._emit_result(CenterResult(action="abort", reason=reason, kpi=kpi))
            return
        self._emit_result(CenterResult(
            action="abort",
            reason=reason,
            objects=list(pending.get("objects") or pending.get("pre_objects") or []),
            primary=pending.get("primary") or pending.get("pre_primary"),
            kpi=kpi,
            frame=pending.get("frame") or pending.get("pre_frame"),
            steps=list(pending.get("steps") or []),
        ))

    def _emit_center_log(
        self, kpi_fields: dict, ctx: CenterContext, prefix: str, extra: str = ""
    ) -> None:
        line = format_center_log(kpi_fields, ctx, extra=extra)
        logger.info("%s %s", prefix, line)
        self.status_message.emit(line)

    def _emit_result(self, result: CenterResult) -> None:
        self._stop_all_center_timers()
        pending = self._pending
        if pending is not None and not result.steps:
            result.steps = list(pending.get("steps") or [])
        if result.steps:
            self._dump_steps_table({"steps": result.steps, "prefix": self._log_prefix})
        self._pending = None
        self.completed.emit(result)

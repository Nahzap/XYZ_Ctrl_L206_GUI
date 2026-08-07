"""Bridge SRP: frames live → ventana + flag de preview al servicio.

CameraTab no debe mezclar este path con microscopía/C-Focus/persistencia.
"""

from __future__ import annotations

import logging
import time
from typing import Any, Callable, Optional

logger = logging.getLogger("MotorControl_L206")


class CameraLiveBridge:
    """Única responsabilidad: enrutar frames live y gobernar preview_enabled."""

    def __init__(
        self,
        *,
        get_window: Callable[[], Any],
        camera_service: Any = None,
        sync_resolution: Optional[Callable[[], bool]] = None,
        apply_resolution_from_qimage: Optional[Callable[[int, int], None]] = None,
    ):
        self._get_window = get_window
        self._camera_service = camera_service
        self._sync_resolution = sync_resolution
        self._apply_resolution_from_qimage = apply_resolution_from_qimage
        self._resolution_synced = False
        self._ui_frame_count = 0
        self._ui_frame_log_time = 0.0
        self._preview_enabled = True

    def set_camera_service(self, camera_service: Any) -> None:
        self._camera_service = camera_service
        self._push_preview_flag()

    def notify_window_visibility(self, visible: bool) -> None:
        """Ventana visible ⇒ construir preview; oculta ⇒ skip QImage en worker."""
        self._preview_enabled = bool(visible)
        self._push_preview_flag()

    def _push_preview_flag(self) -> None:
        svc = self._camera_service
        if svc is not None and hasattr(svc, "set_preview_enabled"):
            try:
                svc.set_preview_enabled(self._preview_enabled)
            except Exception as exc:
                logger.debug("[CameraLiveBridge] set_preview_enabled: %s", exc)

    def on_frame(self, q_image, raw_frame=None) -> bool:
        """
        Entrega frame a la ventana si está visible.

        Returns:
            True si se pintó; False si se descartó (sin ventana / sin qimage).
        """
        if not self._resolution_synced:
            synced = False
            if self._sync_resolution is not None:
                try:
                    synced = bool(self._sync_resolution())
                except Exception:
                    synced = False
            if (
                not synced
                and q_image is not None
                and hasattr(q_image, "width")
                and q_image.width() > 0
                and q_image.height() > 0
                and self._apply_resolution_from_qimage is not None
            ):
                self._apply_resolution_from_qimage(
                    int(q_image.width()), int(q_image.height())
                )
                synced = True
            self._resolution_synced = synced

        window = self._get_window() if self._get_window else None
        visible = bool(
            window is not None and getattr(window, "isVisible", lambda: False)()
        )
        if visible != self._preview_enabled:
            self.notify_window_visibility(visible)
        if not visible or q_image is None:
            return False

        if self._ui_frame_count == 0:
            logger.info(
                "[CameraLiveBridge] Primer frame a ventana: qimage=%dx%d",
                q_image.width() if q_image else 0,
                q_image.height() if q_image else 0,
            )

        self._ui_frame_count += 1
        now = time.perf_counter()
        if now - self._ui_frame_log_time >= 5.0:
            metrics = None
            if self._camera_service is not None and hasattr(
                self._camera_service, "get_live_metrics"
            ):
                try:
                    metrics = self._camera_service.get_live_metrics()
                except Exception:
                    metrics = None
            logger.info(
                "[CameraLiveBridge] UI frames=%d metrics=%s",
                self._ui_frame_count,
                metrics,
            )
            self._ui_frame_log_time = now

        window.update_frame(q_image, raw_frame)
        return True

    @property
    def ui_frame_count(self) -> int:
        return self._ui_frame_count

    @property
    def preview_enabled(self) -> bool:
        return self._preview_enabled

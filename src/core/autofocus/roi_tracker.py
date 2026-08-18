"""ROI Tracker adaptativo para Z-scan de autofoco.

En cada plano Z se re-detectan los objetos de la escena con U2-Net y se
emparejan con los ROI que se están siguiendo. El ROI resultante se recoloca
sobre el objeto, de modo que S se mide siempre sobre la misma semilla aunque
ésta se desplace durante el barrido.

La caja delimitadora es la ventana de medida; la segmentación de cada plano se
**compara** contra ella (¿sigue ahí el grano?, ¿cuánto se movió?) pero nunca la
sustituye. Tres decisiones sostienen la comparabilidad de S entre planos:

1. **El tracker es dueño de su estado.** ``update()`` no depende de que el
   llamador le devuelva el ROI del paso anterior: mantiene ``current_rois``
   internamente. Si el estado viviera en el llamador, cada plano volvería a
   partir del ROI original y el seguimiento no avanzaría nunca.
2. **La geometría de medida se congela al inicializar la referencia.** Cambiar
   la caja a mitad del barrido reescala S y parte la curva en dos tramos que no
   se pueden comparar: un grano desenfocado se segmenta a trozos, y adoptar ese
   trozo como máscara hizo saltar S un +51% de un plano al siguiente sin que la
   óptica cambiara. Después de ``init_reference`` sólo se traslada.
3. **Lo que se ve y lo que se mide son dos cosas distintas.** La silueta que
   U2-Net segmenta cambia de extensión con el desenfoque, así que usarla como
   máscara mezcla "cuánta área" con "cuán nítido". ``current_rois`` (medida)
   traslada el contorno de referencia sin deformarlo, con área constante;
   ``display_rois`` publica la detección viva para el overlay.
   ``adopt_contour=True`` fuerza que la medida también adopte la silueta; sólo
   tiene sentido para diagnóstico.

Uso típico dentro de AutofocusService::

    tracker = RoiTracker()
    tracker.init_reference(frame0, rois)     # primer plano Z
    ...
    for z in z_planes:
        frame = acquire()
        rois = tracker.update(frame)         # ← ROIs recolocados
        s = score_rois_on_frame(frame, rois)
"""

from __future__ import annotations

import logging
from typing import List, Optional, Sequence, Tuple

import numpy as np

from core.autofocus.focus_metric import bbox_to_contour
from core.detection.u2net_detector import U2NetDetector

logger = logging.getLogger("MotorControl_L206")

Bbox = Tuple[int, int, int, int]
Roi = Tuple[Bbox, np.ndarray]


class _Detection:
    """Detección normalizada (bbox, contorno, centro, área)."""

    __slots__ = ("bbox", "contour", "cx", "cy", "area")

    def __init__(self, bbox: Bbox, contour: Optional[np.ndarray]):
        x, y, w, h = (int(v) for v in bbox)
        self.bbox: Bbox = (x, y, w, h)
        self.contour = contour if contour is not None else bbox_to_contour(self.bbox)
        self.cx = x + w / 2.0
        self.cy = y + h / 2.0
        self.area = float(max(1, w * h))


class RoiTracker:
    """Tracking adaptativo de ROI durante Z-scan por re-detección U2-Net.

    Parámetros
    ----------
    max_drift_px : int
        Desplazamiento máximo admitido (L∞, en píxeles) respecto a la posición
        de referencia. Actúa como reja de candidatos: una detección cuyo centro
        caiga fuera de este radio nunca se empareja, así el ROI no salta a un
        objeto vecino.
    update_threshold_px : float
        Desplazamiento mínimo para recolocar el ROI. Por debajo de este umbral
        se conserva el ROI actual, evitando jitter de ±1 px entre planos.
    enabled : bool
        Si ``False``, ``update()`` retorna los ROIs sin modificar.
    adopt_contour : bool
        Si ``True``, la máscara de medida adopta el contorno detectado en cada
        plano. Rompe la comparabilidad de S (el área varía con el desenfoque) y
        se reserva para diagnóstico: el overlay ya recibe la silueta viva por
        ``display_rois`` sin tocar la medida.
    max_area_ratio : float
        Relación de área máxima admitida entre la detección y la referencia.
        Descarta blobs de desenfoque que engloban varios objetos.
    snap_reference : bool
        Si ``True``, la referencia se reancla a la detección correspondiente
        del primer frame científico, de modo que la máscara de medida provenga
        del mismo detector que después hace el seguimiento. Sólo ocurre antes
        de la primera medición: a partir de ahí la geometría queda congelada.
    anchor_area_ratio : float
        Relación de área máxima para que una detección sirva de referencia de
        posición. Un grano desenfocado se segmenta a trozos y el centro de un
        trozo no dice dónde está el grano: esas detecciones se ignoran para
        recolocar (siguen viéndose en el overlay).
    static_window : bool
        Si ``True`` (por defecto) la medida usa un único ROI cuadrado estático
        por objeto, holgado para que la segmentación quepa dentro en todos los
        planos. Ni se mueve ni cambia de tamaño durante el barrido, así que
        todas las S del barrido se calculan sobre exactamente los mismos
        píxeles. Con ``False`` se vuelve al modo que traslada la máscara según
        el centro detectado.
    static_pad_px : int
        Holgura por lado del ROI cuadrado estático. Es el espacio que absorbe
        el engorde de la segmentación al desenfocar y su deriva; si la
        segmentación se sale, el tracker lo avisa para poder ampliarlo.
    detect_interval : int
        Cada cuántos planos se re-detecta con U2-Net **cuando la ventana es
        estática**. Con ventana estática la detección no mueve la geometría de
        medida: sólo comprueba que la segmentación siga dentro. Ejecutarla en
        los 61 planos de un ciclo costaba una inferencia de 5 Mpx por plano sin
        cambiar ni un píxel de la máscara. Con ventana trasladable se ignora,
        porque ahí la detección sí decide dónde está el ROI.
    """

    def __init__(
        self,
        max_drift_px: int = 40,
        update_threshold_px: float = 2.0,
        enabled: bool = True,
        *,
        adopt_contour: bool = False,
        max_area_ratio: float = 3.0,
        snap_reference: bool = True,
        anchor_area_ratio: float = 2.0,
        static_window: bool = True,
        static_pad_px: int = 50,
        detect_interval: int = 1,
    ):
        self.static_window = bool(static_window)
        self.static_pad_px = max(0, int(static_pad_px))
        self.detect_interval = max(1, int(detect_interval))
        self.max_drift_px = max(1, int(max_drift_px))
        self.update_threshold_px = max(0.0, float(update_threshold_px))
        self.enabled = bool(enabled)
        self.adopt_contour = bool(adopt_contour)
        self.max_area_ratio = max(1.1, float(max_area_ratio))
        self.snap_reference = bool(snap_reference)
        self.anchor_area_ratio = max(1.05, float(anchor_area_ratio))

        # Estado interno
        self._initialized = False
        self._anchor_bboxes: List[Bbox] = []
        self._anchor_contours: List[np.ndarray] = []
        self._current_rois: List[Roi] = []
        self._display_rois: List[Roi] = []
        self._cumulative_dx: List[float] = []
        self._cumulative_dy: List[float] = []
        self._misses: List[int] = []
        self._anchored: List[bool] = []
        # Centro de detección que corresponde a desplazamiento cero. La semilla
        # y U2-Net son pipelines distintos, así que sus centros no coinciden;
        # sin este cero la primera deriva sería el desajuste entre pipelines.
        self._ref_centers: List[Optional[Tuple[float, float]]] = []
        self._fragments: List[int] = []
        self._geometry_locked = False
        self._overflows: List[int] = []
        self._max_overflow_px: List[int] = []
        self._n_updates: int = 0
        self._n_matched: int = 0
        self._n_detections: int = 0
        self._n_detect_skipped: int = 0

        # Detector unificado (Singleton)
        self.detector = U2NetDetector.get_instance()

    # ------------------------------------------------------------------
    # API pública
    # ------------------------------------------------------------------

    def init_reference(
        self,
        frame: np.ndarray,
        rois: Sequence[Roi],
    ) -> List[Roi]:
        """Fija los ROI de referencia sobre el primer frame del barrido.

        Parameters
        ----------
        frame : np.ndarray
            Frame completo (cualquier dtype, 1 o 3 canales).
        rois : list[(bbox, contour)]
            ROIs originales seleccionados para autofoco.

        Returns
        -------
        list[(bbox, contour)]
            Los ROI de referencia, reanclados a la detección si procede.
        """
        self.reset()

        for bbox, contour in rois:
            b: Bbox = tuple(int(v) for v in bbox)  # type: ignore[assignment]
            c = contour if contour is not None else bbox_to_contour(b)
            self._anchor_bboxes.append(b)
            self._anchor_contours.append(np.asarray(c, dtype=np.int32).copy())
            self._current_rois.append((b, np.asarray(c, dtype=np.int32).copy()))
            self._display_rois.append((b, np.asarray(c, dtype=np.int32).copy()))
            self._cumulative_dx.append(0.0)
            self._cumulative_dy.append(0.0)
            self._misses.append(0)
            self._anchored.append(False)
            self._ref_centers.append(None)
            self._fragments.append(0)
            self._overflows.append(0)
            self._max_overflow_px.append(0)

        self._initialized = True

        n_snapped = 0
        if self.snap_reference and self._anchor_bboxes:
            n_snapped = self._snap_reference_to_detection(frame)

        if self.static_window:
            self._build_static_windows(frame.shape)

        # A partir de aquí ya se empieza a medir S: la caja no puede cambiar
        # de tamaño sin invalidar todo lo medido antes.
        self._geometry_locked = True

        logger.info(
            "[RoiTracker] Referencia inicializada (U2-Net): %d ROI, "
            "enabled=%s, reanclados=%d, medida=%s | ventanas: %s",
            len(self._anchor_bboxes),
            self.enabled,
            n_snapped,
            (
                f"ROI cuadrado estático (holgura {self.static_pad_px}px/lado)"
                if self.static_window
                else f"máscara trasladable (max_drift={self.max_drift_px}px)"
            ),
            ", ".join(
                f"ROI{i}={bbox[2]}x{bbox[3]}px"
                for i, (bbox, _) in enumerate(self._current_rois)
            ),
        )
        return self.current_rois

    def _build_static_windows(self, frame_shape: Tuple[int, ...]) -> None:
        """Convierte cada referencia en un ROI cuadrado único y holgado.

        La segmentación es la variable del problema: engorda, adelgaza y se
        parte según el desenfoque. Encerrarla en un cuadrado fijo hace que S
        se calcule siempre sobre los mismos píxeles, que es la condición para
        que los valores de distintos planos sean comparables entre sí.
        """
        frame_h, frame_w = int(frame_shape[0]), int(frame_shape[1])
        for i, (bbox, _) in enumerate(self._current_rois):
            x, y, w, h = bbox
            side = max(int(w), int(h)) + 2 * self.static_pad_px
            side = min(side, frame_w, frame_h)
            cx, cy = x + w / 2.0, y + h / 2.0
            x0 = int(round(min(max(0.0, cx - side / 2.0), frame_w - side)))
            y0 = int(round(min(max(0.0, cy - side / 2.0), frame_h - side)))
            window: Bbox = (x0, y0, side, side)
            self._anchor_bboxes[i] = window
            self._anchor_contours[i] = bbox_to_contour(window)
            self._current_rois[i] = (window, bbox_to_contour(window))
            logger.info(
                "[RoiTracker] ROI %d: ventana estática %s (objeto %dx%dpx "
                "+ %dpx por lado); la segmentación se medirá dentro de ella",
                i,
                window,
                w,
                h,
                self.static_pad_px,
            )

    def update(
        self,
        frame: np.ndarray,
        rois: Optional[Sequence[Roi]] = None,
    ) -> List[Roi]:
        """Re-detecta la escena y recoloca los ROI seguidos.

        ``rois`` es opcional y sólo se usa como semilla si el tracker todavía
        no tiene referencia; el estado vigente es siempre el interno.

        Returns
        -------
        list[(bbox, contour)]
            ROIs para medir S en este frame.
        """
        if not self.enabled:
            return list(rois) if rois is not None else self.current_rois

        if not self._initialized:
            if rois is None:
                return []
            return self.init_reference(frame, rois)

        if frame is None or getattr(frame, "size", 0) == 0:
            return self.current_rois

        self._n_updates += 1
        if not self._should_detect():
            self._n_detect_skipped += 1
            return self.current_rois

        detections = self._detect(frame)
        if not detections:
            for i in range(len(self._misses)):
                self._misses[i] += 1
            logger.debug(
                "[RoiTracker] Paso %d: sin detecciones — ROIs conservados",
                self._n_updates,
            )
            return self.current_rois

        # Un ROI que todavía no fijó su cero de posición conserva el radio
        # ancho: la semilla y U2-Net son pipelines distintos y sus cajas pueden
        # diferir en decenas de píxeles antes de calibrarse entre sí.
        pending = [i for i, ok in enumerate(self._anchored) if not ok]
        radius = [
            float(self.max_drift_px) if self._anchored[i]
            else max(float(self.max_drift_px), float(max(bbox[2], bbox[3])))
            for i, bbox in enumerate(self._anchor_bboxes)
        ]
        pairs = self._match(
            detections,
            radius_px=radius,
            area_ratio=max(self.max_area_ratio, 6.0) if pending else None,
            diagnose=bool(pending) and self._n_updates <= 3,
        )

        n_moved = 0
        for i, det_idx in enumerate(pairs):
            if det_idx < 0:
                self._misses[i] += 1
                continue

            det = detections[det_idx]
            self._n_matched += 1

            # La silueta viva se publica siempre para el overlay, aunque la
            # máscara de medida no se mueva: ver el seguimiento no debe costar
            # comparabilidad de S.
            self._display_rois[i] = (
                det.bbox,
                np.asarray(det.contour, dtype=np.int32).copy(),
            )

            if self.static_window:
                self._check_containment(i, det)
                continue

            # Comparación caja vs segmentación: un trozo suelto del grano no
            # informa de dónde está el grano, así que no mueve la ventana.
            if not self._area_comparable(i, det):
                self._fragments[i] += 1
                continue

            if self._ref_centers[i] is None:
                self._set_reference_center(i, det)
                continue

            ax, ay = self._ref_centers[i]
            dx = float(np.clip(det.cx - ax, -self.max_drift_px, self.max_drift_px))
            dy = float(np.clip(det.cy - ay, -self.max_drift_px, self.max_drift_px))

            moved = max(
                abs(dx - self._cumulative_dx[i]),
                abs(dy - self._cumulative_dy[i]),
            )
            if moved < self.update_threshold_px and not self.adopt_contour:
                continue

            if self.adopt_contour:
                new_roi: Roi = (
                    det.bbox,
                    np.asarray(det.contour, dtype=np.int32).copy(),
                )
                self._cumulative_dx[i] = det.cx - ax
                self._cumulative_dy[i] = det.cy - ay
            else:
                new_roi, dx, dy = self._shift_anchor(i, dx, dy, frame.shape)
                self._cumulative_dx[i] = dx
                self._cumulative_dy[i] = dy

            self._current_rois[i] = new_roi
            n_moved += 1

        if n_moved and self._n_updates % 10 == 0:
            logger.debug(
                "[RoiTracker] Paso %d: %d/%d ROI recolocados, drift máx=%.1f px",
                self._n_updates,
                n_moved,
                len(self._current_rois),
                self.get_total_drift(),
            )

        return self.current_rois

    # ------------------------------------------------------------------
    # Detección y emparejamiento
    # ------------------------------------------------------------------

    def _should_detect(self) -> bool:
        """¿Toca inferencia U2-Net en este plano?

        Con ventana trasladable siempre: la detección es la que decide dónde
        medir. Con ventana estática basta muestrear, porque lo único que aporta
        es el chequeo de contención y la silueta del overlay.
        """
        if not self.static_window or self.detect_interval <= 1:
            return True
        return (self._n_updates - 1) % self.detect_interval == 0

    def _detect(self, frame: np.ndarray) -> List[_Detection]:
        """Ejecuta U2-Net sobre el frame y normaliza las detecciones."""
        self._n_detections += 1
        try:
            _, objs = self.detector.detect(frame)
        except Exception as exc:  # el AF no debe caerse por el detector
            logger.warning("[RoiTracker] Detección falló: %s", exc)
            return []

        detections: List[_Detection] = []
        for obj in objs or []:
            bbox = getattr(obj, "bbox", None)
            if bbox is None:
                bbox = getattr(obj, "bounding_box", None)
            if bbox is None or len(bbox) != 4:
                continue
            detections.append(_Detection(bbox, getattr(obj, "contour", None)))
        return detections

    def _match(
        self,
        detections: List[_Detection],
        *,
        radius_px: Optional[List[float]] = None,
        area_ratio: Optional[float] = None,
        diagnose: bool = False,
    ) -> List[int]:
        """Asignación 1:1 codiciosa entre ROI seguidos y detecciones.

        ``radius_px`` permite ensanchar la reja por ROI (se usa al reanclar la
        referencia, donde la semilla viene de otro pipeline de segmentación).

        Retorna, por ROI, el índice de detección asignado o ``-1``.
        """
        max_ratio = float(area_ratio or self.max_area_ratio)
        candidates: List[Tuple[float, int, int]] = []
        rejections: List[str] = []

        for i, (cur_bbox, _) in enumerate(self._current_rois):
            anchor = self._anchor_bboxes[i]
            ax, ay = self._center(anchor)
            anchor_area = float(max(1, anchor[2] * anchor[3]))
            cx, cy = self._center(cur_bbox)
            radius = (
                float(radius_px[i]) if radius_px else float(self.max_drift_px)
            )

            for j, det in enumerate(detections):
                # Si un centro cae dentro de la otra caja se trata del mismo
                # objeto aunque la segmentación haya cambiado de extensión;
                # esa evidencia manda sobre las rejas métricas.
                overlaps = (
                    self._contains(anchor, det.cx, det.cy)
                    or self._contains(det.bbox, ax, ay)
                )
                offset = max(abs(det.cx - ax), abs(det.cy - ay))
                ratio = det.area / anchor_area

                if not overlaps:
                    # Reja 1: sin solape, la detección no puede alejarse del
                    # ancla más que el radio de búsqueda.
                    if offset > radius:
                        if diagnose:
                            rejections.append(
                                f"ROI{i}/det{j}: offset={offset:.0f}px "
                                f"> radio={radius:.0f}px"
                            )
                        continue
                    # Reja 2: un blob de desenfoque que engloba varios objetos
                    # tiene un área desproporcionada respecto a la semilla.
                    if ratio > max_ratio or ratio < 1.0 / max_ratio:
                        if diagnose:
                            rejections.append(
                                f"ROI{i}/det{j}: area_ratio={ratio:.2f} "
                                f"fuera de [1/{max_ratio:.1f}, {max_ratio:.1f}]"
                            )
                        continue

                dist = float(np.hypot(det.cx - cx, det.cy - cy))
                proximity = 1.0 - min(1.0, dist / max(1.0, radius))
                area_sim = min(ratio, 1.0 / ratio) if ratio > 0 else 0.0
                score = (
                    0.50 * self._iou(cur_bbox, det.bbox)
                    + 0.35 * proximity
                    + 0.15 * area_sim
                )
                candidates.append((score, i, j))

        pairs = [-1] * len(self._current_rois)
        if not candidates:
            if diagnose:
                self._log_no_match(detections, rejections)
            return pairs

        candidates.sort(key=lambda item: item[0], reverse=True)
        taken_det: set = set()
        for _score, i, j in candidates:
            if pairs[i] >= 0 or j in taken_det:
                continue
            pairs[i] = j
            taken_det.add(j)

        return pairs

    def _log_no_match(
        self,
        detections: List[_Detection],
        rejections: List[str],
    ) -> None:
        """Vuelca ancla y detecciones cuando ninguna pareja pasa las rejas."""
        anchors = ", ".join(
            f"ROI{i}={bbox}" for i, bbox in enumerate(self._anchor_bboxes)
        )
        dets = ", ".join(
            f"det{j}={det.bbox}" for j, det in enumerate(detections)
        )
        logger.warning(
            "[RoiTracker] Sin emparejamiento | anclas: %s | detecciones: %s | "
            "motivos: %s",
            anchors or "-",
            dets or "-",
            "; ".join(rejections) or "-",
        )

    def _snap_reference_to_detection(self, frame: np.ndarray) -> int:
        """Reancla la referencia a la detección del primer frame medido.

        La semilla llega de SmartFocusScorer (umbral fijo sobre la saliencia),
        mientras que el seguimiento usa U2NetDetector (umbral adaptativo +
        morfología). Las dos cajas pueden diferir en decenas de píxeles, así
        que este reanclaje busca con un radio del tamaño del objeto: una vez
        hecho, el ancla y las detecciones posteriores salen del mismo pipeline
        y ``max_drift_px`` vuelve a medir deriva real.
        """
        detections = self._detect(frame)
        if not detections:
            logger.warning(
                "[RoiTracker] Reanclaje sin detecciones en el primer frame"
            )
            return 0

        radius = [
            max(float(self.max_drift_px), float(max(bbox[2], bbox[3])))
            for bbox in self._anchor_bboxes
        ]
        pairs = self._match(
            detections,
            radius_px=radius,
            area_ratio=max(self.max_area_ratio, 6.0),
            diagnose=True,
        )

        n_snapped = 0
        for i, det_idx in enumerate(pairs):
            if det_idx < 0:
                continue
            det = detections[det_idx]
            # Si el primer frame sólo entrega un trozo del grano, conservar la
            # semilla: es peor medir todo el barrido sobre un cuarto del objeto
            # que arrastrar la diferencia de encuadre entre los dos pipelines.
            if not self._area_comparable(i, det):
                logger.warning(
                    "[RoiTracker] ROI %d: detección %s descartada como "
                    "referencia (área %.0f%% de la semilla %s); se mantiene "
                    "la semilla como ventana de medida",
                    i,
                    det.bbox,
                    100.0 * det.area / float(
                        max(1, self._anchor_bboxes[i][2]
                            * self._anchor_bboxes[i][3])
                    ),
                    self._anchor_bboxes[i],
                )
                continue
            self._adopt_anchor(i, det)
            n_snapped += 1
        return n_snapped

    def _check_containment(self, index: int, det: _Detection) -> None:
        """Comprueba que la segmentación siga dentro del ROI cuadrado.

        Si se desborda, la S de ese plano mide un objeto recortado y deja de
        ser comparable con las demás: se avisa con cuántos píxeles falta para
        que el usuario pueda ampliar la holgura.
        """
        wx, wy, ww, wh = self._current_rois[index][0]
        dx, dy, dw, dh = det.bbox
        overflow = max(
            wx - dx,
            wy - dy,
            (dx + dw) - (wx + ww),
            (dy + dh) - (wy + wh),
        )
        if overflow <= 0:
            return

        self._overflows[index] += 1
        self._max_overflow_px[index] = max(
            self._max_overflow_px[index], int(overflow)
        )
        if self._overflows[index] == 1 or self._overflows[index] % 10 == 0:
            logger.warning(
                "[RoiTracker] ROI %d: la segmentación %s se sale %dpx del ROI "
                "cuadrado %s (paso %d). Aumentar la holgura a ≥%dpx para que "
                "S siga midiendo el grano completo",
                index,
                det.bbox,
                int(overflow),
                self._current_rois[index][0],
                self._n_updates,
                self.static_pad_px + int(overflow),
            )

    def _area_comparable(self, index: int, det: _Detection) -> bool:
        """¿La segmentación describe el grano entero o sólo un trozo?

        Comparar la caja delimitadora con la segmentación es lo que distingue
        un grano desenfocado (segmentado a trozos) de uno bien resuelto. Sólo
        una segmentación de área comparable a la caja puede decir dónde está
        el grano; un fragmento del 25% arrastraría la ventana fuera del objeto.
        """
        x, y, w, h = self._anchor_bboxes[index]
        ratio = det.area / float(max(1, w * h))
        return 1.0 / self.anchor_area_ratio <= ratio <= self.anchor_area_ratio

    def _adopt_anchor(self, index: int, det: _Detection) -> None:
        """Adopta la detección como geometría de medida (sólo pre-barrido)."""
        if self._geometry_locked:
            raise RuntimeError(
                "[RoiTracker] Intento de cambiar la geometría de medida con "
                "el barrido en curso: invalidaría las S ya medidas"
            )
        logger.info(
            "[RoiTracker] ROI %d anclado a U2-Net: semilla=%s → %s",
            index,
            self._anchor_bboxes[index],
            det.bbox,
        )
        contour = np.asarray(det.contour, dtype=np.int32)
        self._anchor_bboxes[index] = det.bbox
        self._anchor_contours[index] = contour.copy()
        self._current_rois[index] = (det.bbox, contour.copy())
        self._display_rois[index] = (det.bbox, contour.copy())
        self._cumulative_dx[index] = 0.0
        self._cumulative_dy[index] = 0.0
        self._ref_centers[index] = (det.cx, det.cy)
        self._anchored[index] = True

    def _set_reference_center(self, index: int, det: _Detection) -> None:
        """Fija el cero de posición sin mover ni redimensionar la ventana.

        Se usa cuando el grano aparece por primera vez ya empezado el barrido
        (los primeros planos suelen estar tan desenfocados que U2-Net no
        detecta nada). Adoptar ahí la caja detectada reescalaría S a mitad de
        curva, así que sólo se registra a qué centro de detección corresponde
        la posición actual de la ventana.
        """
        self._ref_centers[index] = (det.cx, det.cy)
        self._anchored[index] = True
        logger.info(
            "[RoiTracker] ROI %d: cero de posición fijado en la detección %s "
            "(paso %d); ventana de medida intacta en %s",
            index,
            det.bbox,
            self._n_updates,
            self._current_rois[index][0],
        )

    @staticmethod
    def _contains(bbox: Bbox, px: float, py: float) -> bool:
        """True si el punto cae dentro del bbox."""
        x, y, w, h = bbox
        return x <= px <= x + w and y <= py <= y + h

    # ------------------------------------------------------------------
    # Geometría
    # ------------------------------------------------------------------

    def _shift_anchor(
        self,
        index: int,
        dx: float,
        dy: float,
        frame_shape: Tuple[int, ...],
    ) -> Tuple[Roi, float, float]:
        """Traslada bbox y contorno de referencia sin deformarlos.

        El desplazamiento se recorta para que el ROI quede completo dentro del
        frame: recortar la caja cambiaría el área de la máscara y con ella S.
        """
        x, y, w, h = self._anchor_bboxes[index]
        frame_h, frame_w = int(frame_shape[0]), int(frame_shape[1])

        nx = int(round(x + dx))
        ny = int(round(y + dy))
        nx = max(0, min(nx, max(0, frame_w - w)))
        ny = max(0, min(ny, max(0, frame_h - h)))

        real_dx = nx - x
        real_dy = ny - y

        contour = self._anchor_contours[index].copy()
        if contour.size:
            contour[..., 0] += real_dx
            contour[..., 1] += real_dy

        return ((nx, ny, w, h), contour), float(real_dx), float(real_dy)

    @staticmethod
    def _center(bbox: Bbox) -> Tuple[float, float]:
        x, y, w, h = bbox
        return x + w / 2.0, y + h / 2.0

    @staticmethod
    def _iou(box_a: Bbox, box_b: Bbox) -> float:
        """Intersection over Union entre dos bboxes (x, y, w, h)."""
        ax1, ay1, aw, ah = box_a
        bx1, by1, bw, bh = box_b
        ax2, ay2 = ax1 + aw, ay1 + ah
        bx2, by2 = bx1 + bw, by1 + bh

        inter_w = min(ax2, bx2) - max(ax1, bx1)
        inter_h = min(ay2, by2) - max(ay1, by1)
        if inter_w <= 0 or inter_h <= 0:
            return 0.0

        inter = float(inter_w * inter_h)
        union = float(aw * ah + bw * bh) - inter
        return inter / union if union > 0 else 0.0

    # ------------------------------------------------------------------
    # Consulta de estado
    # ------------------------------------------------------------------

    @property
    def current_rois(self) -> List[Roi]:
        """ROI de medida: contorno de referencia trasladado, área constante."""
        return [(bbox, contour) for bbox, contour in self._current_rois]

    @property
    def display_rois(self) -> List[Roi]:
        """ROI de visualización: última silueta segmentada por U2-Net.

        Sólo para el overlay. Medir S sobre esta máscara la haría depender del
        tamaño que el detector le asigne al objeto en cada plano.
        """
        return [(bbox, contour) for bbox, contour in self._display_rois]

    @property
    def n_updates(self) -> int:
        return self._n_updates

    @property
    def n_detections(self) -> int:
        """Inferencias U2-Net reales (incluye el reanclaje de referencia)."""
        return self._n_detections

    @property
    def n_detect_skipped(self) -> int:
        return self._n_detect_skipped

    def get_overflow_stats(self) -> Tuple[int, int]:
        """(planos con segmentación desbordada, desborde máximo en px)."""
        if not self._overflows:
            return 0, 0
        return int(sum(self._overflows)), int(max(self._max_overflow_px))

    def get_cumulative_drift(self) -> List[Tuple[float, float]]:
        """Drift acumulado (dx, dy) por ROI respecto a la referencia."""
        return list(zip(self._cumulative_dx, self._cumulative_dy))

    def get_total_drift(self) -> float:
        """Drift máximo (norma L∞) entre todos los ROI."""
        if not self._cumulative_dx:
            return 0.0
        return max(
            max(abs(dx), abs(dy))
            for dx, dy in zip(self._cumulative_dx, self._cumulative_dy)
        )

    def get_summary(self) -> str:
        """Resumen legible del estado del tracker."""
        if not self._initialized:
            return "[RoiTracker] No inicializado"
        lines = [
            f"[RoiTracker U2-Net] {self._n_updates} pasos, "
            f"{self._n_detections} inferencias (cada {self.detect_interval}), "
            f"{self._n_matched} emparejamientos, {len(self._current_rois)} ROI:"
        ]
        for i, (dx, dy) in enumerate(self.get_cumulative_drift()):
            bbox = self._current_rois[i][0]
            if self.static_window:
                lines.append(
                    f"  ROI {i}: ventana estática {bbox[2]}x{bbox[3]}px en "
                    f"({bbox[0]},{bbox[1]}) sin_match={self._misses[i]} "
                    f"desbordes={self._overflows[i]} "
                    f"(máx {self._max_overflow_px[i]}px)"
                )
            else:
                lines.append(
                    f"  ROI {i}: drift=(Δx={dx:+.1f}, Δy={dy:+.1f}) px "
                    f"bbox={bbox} sin_match={self._misses[i]} "
                    f"fragmentos_ignorados={self._fragments[i]}"
                )
        return "\n".join(lines)

    def reset(self) -> None:
        """Limpia todo el estado del tracker."""
        self._initialized = False
        self._anchor_bboxes = []
        self._anchor_contours = []
        self._current_rois = []
        self._display_rois = []
        self._cumulative_dx = []
        self._cumulative_dy = []
        self._misses = []
        self._anchored = []
        self._ref_centers = []
        self._fragments = []
        self._overflows = []
        self._max_overflow_px = []
        self._geometry_locked = False
        self._n_updates = 0
        self._n_matched = 0
        self._n_detections = 0
        self._n_detect_skipped = 0

    @property
    def is_active(self) -> bool:
        """True si el tracker está inicializado y habilitado."""
        return self.enabled and self._initialized

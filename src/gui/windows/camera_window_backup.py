"""
Ventana para visualización de cámara Thorlabs con overlay de detección U2-Net.

ARQUITECTURA SIMPLE:
- Cámara muestra frames en tiempo real (30 FPS)
- Detección se ejecuta SOLO cada N segundos (configurable)
- El mapa de saliencia se SUPERPONE sobre el feed en vivo
- La detección NO bloquea la visualización
"""

import logging
import numpy as np
import cv2
import time
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QCheckBox, QSpinBox, QDoubleSpinBox)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QPixmap, QImage
from gui.styles.dark_theme import DARK_STYLESHEET

logger = logging.getLogger(__name__)


class DetectionWorker(QThread):
    """
    Hilo separado para detección usando SmartFocusScorer.
    MISMO MÉTODO que ImgAnalysisTab._analyze_current()
    """
    
    detection_done = pyqtSignal(object, list, float, object)  # probability_map, objects, time_ms, frame_used
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.scorer = None  # SmartFocusScorer (mismo que ImgAnalysisTab)
        self.frame = None
        self.running = False
    
    def set_scorer(self, scorer):
        """Configura SmartFocusScorer (mismo que usa ImgAnalysisTab)."""
        self.scorer = scorer
        logger.info(f"DetectionWorker: SmartFocusScorer configurado")
    
    def set_params(self, min_area, max_area, threshold=None):
        """Actualiza parámetros del scorer."""
        if self.scorer:
            self.scorer.set_parameters(
                threshold=threshold if threshold else 0.5,
                min_area=min_area
            )
            logger.debug(f"Scorer params: min_area={min_area}, threshold={threshold}")
    
    def detect_frame(self, frame):
        """Encola un frame para detección (solo si no está ocupado)."""
        if self.running or self.isRunning():
            return False
        self.frame = frame.copy() if frame is not None else None
        self.start()
        return True
    
    def run(self):
        """
        Ejecuta detección usando SmartFocusScorer.
        EMITE EL FRAME ORIGINAL para que overlay coincida.
        """
        self.running = True
        
        if self.frame is None or self.scorer is None:
            logger.warning(f"DetectionWorker: frame={self.frame is not None}, scorer={self.scorer is not None}")
            self.running = False
            return
        
        try:
            t_start = time.perf_counter()
            
            # Guardar frame ORIGINAL para emitir después
            original_frame = self.frame.copy()
            
            # Convertir para el modelo (grayscale uint8)
            frame_for_model = self.frame
            if frame_for_model.dtype == np.uint16:
                frame_for_model = (frame_for_model / 256).astype(np.uint8)
            if len(frame_for_model.shape) == 3:
                frame_for_model = cv2.cvtColor(frame_for_model, cv2.COLOR_BGR2GRAY)
            
            # Procesar con scorer
            result = self.scorer.assess_image(frame_for_model)
            
            t_ms = (time.perf_counter() - t_start) * 1000
            
            probability_map = result.probability_map
            objects = result.objects
            
            n_obj = len(objects) if objects else 0
            logger.info(f"Detección: {n_obj} objetos, score={result.focus_score:.2f}, {t_ms:.0f}ms")
            
            if probability_map is not None:
                # Emitir frame ORIGINAL (uint16) para overlay consistente
                self.detection_done.emit(probability_map, objects, t_ms, original_frame)
            
        except Exception as e:
            logger.error(f"Error en detección: {e}", exc_info=True)
        finally:
            self.frame = None
            self.running = False


class CameraViewWindow(QWidget):
    """
    Ventana de visualización de cámara con overlay de saliencia.
    
    - Feed de cámara: 30 FPS (tiempo real)
    - Detección: cada N segundos (configurable, default 2s)
    - Overlay: se dibuja la última máscara sobre cada frame
    """
    
    COLORS = [(0,255,0), (255,0,0), (0,255,255), (255,0,255), (255,255,0)]
    
    def __init__(self, parent=None):
        super().__init__(parent, Qt.Window)
        self.setWindowTitle('🎥 Vista de Cámara - Tiempo Real')
        self.setGeometry(200, 200, 800, 600)
        self.setStyleSheet(DARK_STYLESHEET)
        
        layout = QVBoxLayout(self)
        
        # === CONTROLES DE DETECCIÓN ===
        ctrl_layout = QHBoxLayout()
        
        # Checkbox para activar detección periódica
        self.auto_detect_cb = QCheckBox("🔍 Auto")
        self.auto_detect_cb.setChecked(False)
        self.auto_detect_cb.stateChanged.connect(self._toggle_auto_detection)
        ctrl_layout.addWidget(self.auto_detect_cb)
        
        # Intervalo de detección
        self.interval_spin = QSpinBox()
        self.interval_spin.setRange(1, 10)
        self.interval_spin.setValue(2)
        self.interval_spin.setSuffix("s")
        self.interval_spin.setFixedWidth(50)
        self.interval_spin.valueChanged.connect(self._update_timer_interval)
        ctrl_layout.addWidget(self.interval_spin)
        
        ctrl_layout.addWidget(QLabel("|"))
        
        # Checkboxes de overlay
        self.show_saliency_cb = QCheckBox("Mapa")
        self.show_saliency_cb.setChecked(True)
        ctrl_layout.addWidget(self.show_saliency_cb)
        
        self.show_boxes_cb = QCheckBox("Boxes")
        self.show_boxes_cb.setChecked(True)
        ctrl_layout.addWidget(self.show_boxes_cb)
        
        ctrl_layout.addWidget(QLabel("|"))
        
        # Info
        self.detection_info = QLabel("Sin detección")
        self.detection_info.setStyleSheet("color: #3498DB;")
        ctrl_layout.addWidget(self.detection_info)
        ctrl_layout.addStretch()
        
        layout.addLayout(ctrl_layout)
        
        # === PARÁMETROS U2-NET (editables) ===
        params_layout = QHBoxLayout()
        
        params_layout.addWidget(QLabel("Área:"))
        self.min_area_spin = QSpinBox()
        self.min_area_spin.setRange(10, 100000)
        self.min_area_spin.setValue(100)
        self.min_area_spin.setFixedWidth(70)
        self.min_area_spin.setToolTip("Área mínima (px)")
        self.min_area_spin.valueChanged.connect(self._on_params_changed)
        params_layout.addWidget(self.min_area_spin)
        
        params_layout.addWidget(QLabel("-"))
        self.max_area_spin = QSpinBox()
        self.max_area_spin.setRange(100, 1000000)
        self.max_area_spin.setValue(500000)
        self.max_area_spin.setFixedWidth(80)
        self.max_area_spin.setToolTip("Área máxima (px)")
        self.max_area_spin.valueChanged.connect(self._on_params_changed)
        params_layout.addWidget(self.max_area_spin)
        
        params_layout.addWidget(QLabel("| Umbral:"))
        self.threshold_spin = QDoubleSpinBox()
        self.threshold_spin.setRange(0.05, 0.95)
        self.threshold_spin.setValue(0.3)
        self.threshold_spin.setSingleStep(0.05)
        self.threshold_spin.setFixedWidth(60)
        self.threshold_spin.setToolTip("Umbral de saliencia (0.1=sensible, 0.9=estricto)")
        self.threshold_spin.valueChanged.connect(self._on_params_changed)
        params_layout.addWidget(self.threshold_spin)
        
        params_layout.addStretch()
        layout.addLayout(params_layout)
        
        # === VIDEO ===
        self.video_label = QLabel("Esperando frames...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: #000; border: 2px solid #505050;")
        self.video_label.setMinimumSize(640, 480)
        layout.addWidget(self.video_label)
        
        # Info de frame
        self.frame_info = QLabel("Frame: 0")
        self.frame_info.setStyleSheet("color: #95A5A6; font-size: 10px;")
        layout.addWidget(self.frame_info)
        
        # === ESTADO ===
        self.frame_count = 0
        self.last_raw_frame = None
        self.frame_size = None  # (w, h) del último frame
        
        # Resultados de detección (persisten hasta nueva detección)
        self.saliency_map = None
        self.saliency_colormap = None  # Cache del colormap aplicado
        self.detected_objects = []
        self.last_detection_ms = 0
        self.detection_frame = None  # Frame usado para detección (para overlay consistente)
        
        # Worker de detección (asíncrono - NO bloquea UI)
        self.detection_worker = DetectionWorker()
        self.detection_worker.detection_done.connect(self._on_detection_done, Qt.QueuedConnection)
        
        # Timer para detección periódica
        self.detection_timer = QTimer(self)
        self.detection_timer.timeout.connect(self._trigger_detection)
        
        # SmartFocusScorer (mismo que ImgAnalysisTab)
        self.scorer = None
        
        logger.debug("CameraViewWindow creada")
        
    # === MÉTODOS DE CONTROL ===
    
    def _toggle_auto_detection(self, state):
        """Activa/desactiva detección periódica."""
        if state:
            interval_ms = self.interval_spin.value() * 1000
            self.detection_timer.start(interval_ms)
            self.detection_info.setText(f"⏱️ Detectando cada {self.interval_spin.value()}s...")
            # Ejecutar primera detección inmediatamente
            self._trigger_detection()
        else:
            self.detection_timer.stop()
            self.detection_info.setText("Auto-detección desactivada")
    
    def _update_timer_interval(self, value):
        """Actualiza intervalo del timer."""
        if self.detection_timer.isActive():
            self.detection_timer.setInterval(value * 1000)
            self.detection_info.setText(f"⏱️ Detectando cada {value}s...")
    
    def _trigger_detection(self):
        """Dispara una detección sobre el frame actual."""
        if self.last_raw_frame is not None and self.scorer is not None:
            self.detection_worker.detect_frame(self.last_raw_frame)
        elif self.scorer is None:
            logger.warning("SmartFocusScorer no configurado - usa set_scorer()")
    
    def _on_params_changed(self):
        """Callback cuando cambian los parámetros de detección."""
        min_area = self.min_area_spin.value()
        max_area = self.max_area_spin.value()
        threshold = self.threshold_spin.value()
        
        # Actualizar worker (que actualiza el scorer)
        self.detection_worker.set_params(min_area, max_area, threshold)
        
        logger.debug(f"Parámetros: área=[{min_area}-{max_area}], umbral={threshold}")
    
    # === ACTUALIZACIÓN DE FRAME ===
    
    def update_frame(self, q_image, raw_frame=None):
        """
        Actualiza el frame mostrado.
        Si hay overlay activo: muestra FRAME DE DETECCIÓN (congelado) con overlay
        Si no hay overlay: muestra frame en vivo
        """
        try:
            self.frame_count += 1
            
            # Guardar frame raw para próxima detección
            if raw_frame is not None:
                self.last_raw_frame = raw_frame
            
            # ¿Hay overlay para mostrar?
            has_overlay = (self.show_saliency_cb.isChecked() and self.saliency_colormap is not None) or \
                          (self.show_boxes_cb.isChecked() and self.detected_objects)
            
            if has_overlay and self.detection_frame is not None:
                # MOSTRAR FRAME DE DETECCIÓN con overlay (frames coinciden)
                display_frame = self._draw_overlay(self.detection_frame)
                if display_frame is not None:
                    q_image = self._frame_to_qimage(display_frame)
            # Si no hay overlay, mostrar q_image original (frame en vivo)
            
            # Mostrar
            pixmap = QPixmap.fromImage(q_image)
            scaled = pixmap.scaled(self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.video_label.setPixmap(scaled)
            
            # Info
            n_obj = len(self.detected_objects) if self.detected_objects else 0
            mode = "� DETECT" if has_overlay else "🎥 LIVE"
            self.frame_info.setText(f"{mode} | Frame: {self.frame_count} | {n_obj} obj")
            
        except Exception as e:
            logger.error(f"Error actualizando frame: {e}", exc_info=True)
    
    def _on_detection_done(self, saliency_map, objects, time_ms, frame_used):
        """Callback cuando el worker termina la detección."""
        self.saliency_map = saliency_map
        self.detected_objects = objects
        self.last_detection_ms = time_ms
        self.detection_frame = frame_used  # Guardar frame usado para overlay consistente
        
        # Pre-calcular colormap con tamaño del frame de detección
        if frame_used is not None:
            h, w = frame_used.shape[:2]
            self.frame_size = (w, h)
        
        self._update_colormap_cache()
        
        n_obj = len(objects) if objects else 0
        has_colormap = "✓" if self.saliency_colormap is not None else "✗"
        self.detection_info.setText(f"{n_obj} obj | {time_ms:.0f}ms | map:{has_colormap}")
        logger.info(f"Detección: {n_obj} objetos en {time_ms:.0f}ms, colormap={has_colormap}")
    
    def _update_colormap_cache(self):
        """Pre-calcula el colormap usando el tamaño del DETECTION_FRAME."""
        if self.saliency_map is None or self.detection_frame is None:
            self.saliency_colormap = None
            return
        
        try:
            # Usar tamaño del DETECTION_FRAME (el frame que generó el saliency_map)
            h, w = self.detection_frame.shape[:2]
            
            # Pre-calcular heatmap
            sal_resized = cv2.resize(self.saliency_map, (w, h), interpolation=cv2.INTER_LINEAR)
            self.saliency_colormap = cv2.applyColorMap(
                (sal_resized * 255).astype(np.uint8), cv2.COLORMAP_JET
            )
            logger.info(f"Colormap: saliency {self.saliency_map.shape} -> frame {w}x{h}")
        except Exception as e:
            logger.error(f"Error en colormap: {e}")
            self.saliency_colormap = None
    
    # === DIBUJO DE OVERLAY ===
    
    def _draw_overlay(self, frame):
        """
        Dibuja overlay usando heatmap PRE-CALCULADO (no hacer resize cada frame).
        """
        if frame is None:
            return None
        
        try:
            # Convertir frame a uint8 BGR
            if frame.dtype == np.uint16:
                frame_uint8 = (frame / 256).astype(np.uint8)
            else:
                frame_uint8 = frame.astype(np.uint8)
            
            if len(frame_uint8.shape) == 2:
                vis = cv2.cvtColor(frame_uint8, cv2.COLOR_GRAY2BGR)
            else:
                vis = frame_uint8.copy()
            
            # Overlay heatmap PRE-CALCULADO (no resize aquí!)
            if self.show_saliency_cb.isChecked() and self.saliency_colormap is not None:
                if vis.shape[:2] == self.saliency_colormap.shape[:2]:
                    vis = cv2.addWeighted(vis, 0.5, self.saliency_colormap, 0.5, 0)
            
            # Bounding boxes
            if self.show_boxes_cb.isChecked() and self.detected_objects:
                for i, obj in enumerate(self.detected_objects):
                    x, y, bw, bh = obj.bounding_box
                    color = (0, 255, 0) if obj.is_focused else (0, 0, 255)
                    cv2.rectangle(vis, (x, y), (x+bw, y+bh), color, 2)
                    label = f"#{i+1} S:{obj.focus_score:.1f}"
                    cv2.putText(vis, label, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            return vis
            
        except Exception as e:
            logger.error(f"_draw_overlay error: {e}")
            return None
    
    def _frame_to_qimage(self, frame):
        """Convierte frame numpy a QImage de forma segura."""
        if frame is None:
            return QImage()
        
        # Asegurar uint8
        if frame.dtype != np.uint8:
            frame = frame.astype(np.uint8)
        
        # Asegurar contiguo
        frame = np.ascontiguousarray(frame)
        
        if len(frame.shape) == 2:
            h, w = frame.shape
            return QImage(frame.data, w, h, w, QImage.Format_Grayscale8).copy()
        else:
            h, w, ch = frame.shape
            if ch == 3:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                rgb = np.ascontiguousarray(rgb)
                return QImage(rgb.data, w, h, 3*w, QImage.Format_RGB888).copy()
            else:
                return QImage()
    
    def set_scorer(self, scorer):
        """Configura SmartFocusScorer (mismo que usa ImgAnalysisTab)."""
        self.scorer = scorer
        self.detection_worker.set_scorer(scorer)
        logger.info("SmartFocusScorer configurado en CameraViewWindow")
    
    def set_detection_params(self, min_area: int, max_area: int):
        """Actualiza parámetros de detección."""
        self.detection_params = {'min_area': min_area, 'max_area': max_area}
        self.detection_worker.set_params(min_area, max_area)
    
    def clear_detection(self):
        """Limpia resultados de detección."""
        self.detected_objects = []
        self.saliency_map = None
        self.saliency_colormap = None
        self.detection_info.setText("Sin detección")
    
    def resizeEvent(self, event):
        """Reescalar imagen cuando se redimensiona la ventana."""
        super().resizeEvent(event)
    
    def closeEvent(self, event):
        """Detener timer al cerrar ventana."""
        self.detection_timer.stop()
        super().closeEvent(event)

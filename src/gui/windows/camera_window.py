"""
Ventana de visualización de cámara con overlay de detección U2-Net.
Solo visualización - los controles de detección están en CameraTab.
"""

import logging
import numpy as np
import cv2
import time
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox, QListWidget, QListWidgetItem, QSplitter
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QPixmap, QImage
from gui.styles.dark_theme import DARK_STYLESHEET

logger = logging.getLogger(__name__)


class DetectionWorker(QThread):
    """Hilo de detección asíncrona usando SmartFocusScorer."""
    
    # Emite: (probability_map, objects, time_ms, frame_bgr)
    detection_done = pyqtSignal(object, list, float, object)
    
    def __init__(self):
        super().__init__()
        self.scorer = None
        self.frame = None
        self.running = False
        self.filter_min_area = 100
        self.filter_max_area = 999999
    
    def set_scorer(self, scorer):
        self.scorer = scorer
        logger.info("DetectionWorker: scorer configurado")
    
    def set_params(self, min_area, max_area, threshold):
        """Guarda parámetros de filtro pero usa min_area bajo para detección visual."""
        self.filter_min_area = min_area
        self.filter_max_area = max_area
        if self.scorer:
            # Para visualización: detectar TODOS los objetos (min_area bajo)
            # El filtro de área se aplica después para autofoco
            self.scorer.set_parameters(threshold=threshold, min_area=100, max_area=999999)
            logger.info(f"DetectionWorker: filtro área [{min_area}-{max_area}], detección con min_area=100")
    
    def detect(self, frame):
        """Inicia detección si no está ocupado."""
        if self.running:
            return False
        self.frame = frame.copy()
        self.start()
        return True
    
    def run(self):
        self.running = True
        
        if self.frame is None or self.scorer is None:
            self.running = False
            return
        
        try:
            t0 = time.perf_counter()
            
            # Convertir frame uint16 -> uint8 (normalizar por max como camera_worker)
            if self.frame.dtype == np.uint16:
                frame_max = self.frame.max()
                if frame_max > 0:
                    frame_uint8 = (self.frame / frame_max * 255).astype(np.uint8)
                else:
                    frame_uint8 = np.zeros_like(self.frame, dtype=np.uint8)
            else:
                frame_uint8 = self.frame.astype(np.uint8)
            
            if len(frame_uint8.shape) == 2:
                frame_bgr = cv2.cvtColor(frame_uint8, cv2.COLOR_GRAY2BGR)
            else:
                frame_bgr = frame_uint8
            
            # Ejecutar detección
            result = self.scorer.assess_image(frame_bgr)
            t_ms = (time.perf_counter() - t0) * 1000
            
            prob_map = result.probability_map
            objects = result.objects if result.objects else []
            
            logger.info(f"Detección: {len(objects)} obj, score={result.focus_score:.2f}, {t_ms:.0f}ms")
            self.detection_done.emit(prob_map, objects, t_ms, frame_bgr)
            
        except Exception as e:
            logger.error(f"Error detección: {e}", exc_info=True)
        finally:
            self.frame = None
            self.running = False


class CameraViewWindow(QWidget):
    """Ventana de cámara con overlay de saliencia."""
    
    def __init__(self, parent=None):
        super().__init__(parent, Qt.Window)
        self.setWindowTitle('🎥 Vista de Cámara - Tiempo Real')
        self.setMinimumSize(800, 650)
        self.setStyleSheet(DARK_STYLESHEET)
        
        self._setup_ui()
        self._setup_state()
        
        logger.debug("CameraViewWindow creada")
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(5)
        
        # Controles de visualización
        ctrl_row = QHBoxLayout()
        
        self.show_contours_cb = QCheckBox("🔲 Contornos")
        self.show_contours_cb.setChecked(True)
        ctrl_row.addWidget(self.show_contours_cb)
        
        self.show_boxes_cb = QCheckBox("📦 ROI")
        self.show_boxes_cb.setChecked(True)
        ctrl_row.addWidget(self.show_boxes_cb)
        
        self.status_label = QLabel("Listo")
        ctrl_row.addWidget(self.status_label)
        ctrl_row.addStretch()
        layout.addLayout(ctrl_row)
        
        # Splitter: Video + Lista de objetos
        splitter = QSplitter(Qt.Horizontal)
        
        # Video
        self.video_label = QLabel("Esperando frames...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: #000; border: 2px solid #505050;")
        self.video_label.setMinimumSize(640, 480)
        splitter.addWidget(self.video_label)
        
        # Lista de objetos detectados
        objects_panel = QWidget()
        objects_layout = QVBoxLayout(objects_panel)
        objects_layout.setContentsMargins(5, 0, 5, 0)
        
        objects_title = QLabel("📋 Objetos Detectados")
        objects_title.setStyleSheet("font-weight: bold; color: #3498DB;")
        objects_layout.addWidget(objects_title)
        
        self.objects_list = QListWidget()
        self.objects_list.setStyleSheet("""
            QListWidget { background-color: #2C2C2C; border: 1px solid #505050; }
            QListWidget::item { padding: 3px; }
            QListWidget::item:selected { background-color: #3498DB; }
        """)
        self.objects_list.setMinimumWidth(180)
        self.objects_list.setMaximumWidth(250)
        objects_layout.addWidget(self.objects_list)
        
        splitter.addWidget(objects_panel)
        splitter.setSizes([700, 200])
        layout.addWidget(splitter)
        
        # Info
        self.info_label = QLabel("🎥 LIVE | Frame: 0")
        self.info_label.setStyleSheet("color: #95A5A6; font-size: 10px;")
        layout.addWidget(self.info_label)
    
    def _setup_state(self):
        self.frame_count = 0
        self.last_frame = None  # Último frame de cámara (uint16)
        self.detection_result = None  # {contours, boxes, frame_size, n_objects}
        
        # Worker para detección asíncrona
        self.worker = DetectionWorker()
        self.worker.detection_done.connect(self._on_detection_done, Qt.QueuedConnection)
        
        self.scorer = None
    
    def _on_detection_done(self, prob_map, objects, time_ms, frame_bgr):
        """Guarda resultado de detección - PRE-CALCULA contornos para overlay liviano."""
        if prob_map is None:
            return
        
        # Pre-calcular contornos UNA VEZ (no en cada frame)
        h, w = frame_bgr.shape[:2] if frame_bgr is not None else prob_map.shape[:2]
        prob_resized = cv2.resize(prob_map, (w, h))
        binary_mask = (prob_resized > 0.3).astype(np.uint8) * 255
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Obtener filtros de área del worker
        min_area = self.worker.filter_min_area
        max_area = self.worker.filter_max_area
        
        # Extraer coordenadas, área y scores - marcar si está en rango de filtro
        boxes = []
        n_in_range = 0
        for obj in objects:
            area = getattr(obj, 'area', 0)
            in_range = min_area <= area <= max_area
            if in_range:
                n_in_range += 1
            boxes.append({
                'bbox': obj.bounding_box,
                'area': area,
                'score': getattr(obj, 'focus_score', 0),
                'is_focused': getattr(obj, 'is_focused', False),
                'in_filter_range': in_range
            })
        
        # Guardar datos livianos para overlay
        self.detection_result = {
            'contours': contours,
            'boxes': boxes,
            'frame_size': (w, h),
            'n_objects': len(objects),
            'n_in_range': n_in_range,
            'filter_range': (min_area, max_area)
        }
        
        # Actualizar lista de objetos
        self._update_objects_list(boxes, min_area, max_area)
        
        self.status_label.setText(f"✅ {len(objects)} obj ({n_in_range} en rango) | {time_ms:.0f}ms")
        logger.info(f"Detección: {len(objects)} objetos, {n_in_range} en rango [{min_area}-{max_area}]")
    
    def _update_objects_list(self, boxes, min_area=0, max_area=999999):
        """Actualiza la lista de objetos detectados con sus áreas."""
        self.objects_list.clear()
        
        if not boxes:
            item = QListWidgetItem("Sin objetos detectados")
            item.setForeground(Qt.gray)
            self.objects_list.addItem(item)
            return
        
        # Header con rango de filtro
        header = QListWidgetItem(f"Filtro: [{min_area}-{max_area}] px")
        header.setForeground(Qt.cyan)
        self.objects_list.addItem(header)
        
        for i, box in enumerate(boxes):
            area = box.get('area', 0)
            score = box.get('score', 0)
            in_range = box.get('in_filter_range', False)
            
            # Formato: #N | XXXXX px | S:XX.X [✓ si en rango]
            text = f"#{i+1} | {area:.0f} px | S:{score:.1f}"
            if in_range:
                text += " ✓"
            
            item = QListWidgetItem(text)
            if in_range:
                item.setForeground(Qt.green)  # Verde si está en rango
            else:
                item.setForeground(Qt.yellow)  # Amarillo si fuera de rango
            self.objects_list.addItem(item)
    
    # === ACTUALIZACIÓN DE FRAME ===
    
    def update_frame(self, q_image, raw_frame=None):
        """Actualiza visualización - frame EN VIVO con overlay LIVIANO."""
        try:
            self.frame_count += 1
            
            if raw_frame is not None:
                self.last_frame = raw_frame
            
            # ¿Hay overlay disponible?
            has_overlay = (self.show_contours_cb.isChecked() or self.show_boxes_cb.isChecked()) \
                          and self.detection_result is not None
            
            # Aplicar overlay LIVIANO directamente sobre QImage (sin conversión numpy)
            if has_overlay:
                q_image = self._draw_overlay_on_qimage(q_image)
            
            # Mostrar frame
            pixmap = QPixmap.fromImage(q_image)
            scaled = pixmap.scaled(self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.video_label.setPixmap(scaled)
            
            n_obj = self.detection_result.get('n_objects', 0) if self.detection_result else 0
            mode = "🔍" if has_overlay else "🎥"
            self.info_label.setText(f"{mode} LIVE | Frame: {self.frame_count} | {n_obj} obj")
            
        except Exception as e:
            logger.error(f"Error update_frame: {e}")
    
    def _draw_overlay_on_qimage(self, q_image):
        """Dibuja overlay A COLOR sobre QImage usando QPainter."""
        from PyQt5.QtGui import QPainter, QPen, QColor, QFont
        
        if self.detection_result is None:
            return q_image
        
        try:
            # Convertir a RGB32 para poder dibujar colores (grayscale no soporta colores)
            if q_image.format() == QImage.Format_Grayscale8:
                result = q_image.convertToFormat(QImage.Format_RGB32)
            else:
                result = q_image.copy()
            
            painter = QPainter(result)
            
            # Escala entre frame original y QImage
            orig_w, orig_h = self.detection_result.get('frame_size', (1920, 1200))
            scale_x = result.width() / orig_w
            scale_y = result.height() / orig_h
            
            # Dibujar contornos pre-calculados (AMARILLO)
            if self.show_contours_cb.isChecked():
                contours = self.detection_result.get('contours', [])
                pen = QPen(QColor(255, 255, 0), 2)  # Amarillo
                painter.setPen(pen)
                for contour in contours:
                    if len(contour) > 1:
                        points = [(int(pt[0][0] * scale_x), int(pt[0][1] * scale_y)) for pt in contour]
                        for i in range(len(points) - 1):
                            painter.drawLine(points[i][0], points[i][1], points[i+1][0], points[i+1][1])
                        painter.drawLine(points[-1][0], points[-1][1], points[0][0], points[0][1])
            
            # Dibujar boxes con color según si está en rango de filtro
            if self.show_boxes_cb.isChecked():
                boxes = self.detection_result.get('boxes', [])
                font = QFont("Arial", 10, QFont.Bold)
                painter.setFont(font)
                
                for i, box in enumerate(boxes):
                    x, y, bw, bh = box['bbox']
                    x, y = int(x * scale_x), int(y * scale_y)
                    bw, bh = int(bw * scale_x), int(bh * scale_y)
                    
                    # Color según si está en rango: Verde=en rango, Rojo=fuera
                    in_range = box.get('in_filter_range', False)
                    if in_range:
                        pen = QPen(QColor(0, 255, 0), 2)  # Verde
                    else:
                        pen = QPen(QColor(255, 100, 100), 2)  # Rojo claro
                    painter.setPen(pen)
                    painter.drawRect(x, y, bw, bh)
                    
                    # Mostrar número, área y score
                    area = box.get('area', 0)
                    score = box.get('score', 0)
                    label = f"#{i+1} {area:.0f}px"
                    if in_range:
                        label += " ✓"
                    painter.drawText(x + 2, y - 5, label)
            
            # Mostrar info general en esquina
            n_obj = self.detection_result.get('n_objects', 0)
            if n_obj > 0:
                painter.setPen(QPen(QColor(255, 255, 255)))
                painter.drawText(10, 20, f"Objetos: {n_obj}")
            
            painter.end()
            return result
            
        except Exception as e:
            logger.error(f"Error _draw_overlay_on_qimage: {e}")
            return q_image
    
    def _to_qimage(self, frame):
        """Convierte numpy BGR a QImage."""
        if frame is None:
            return QImage()
        
        frame = np.ascontiguousarray(frame)
        h, w = frame.shape[:2]
        
        if len(frame.shape) == 2:
            return QImage(frame.data, w, h, w, QImage.Format_Grayscale8).copy()
        else:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb = np.ascontiguousarray(rgb)
            return QImage(rgb.data, w, h, 3*w, QImage.Format_RGB888).copy()
    
    # === API PÚBLICA ===
    
    def set_scorer(self, scorer):
        """Configura SmartFocusScorer."""
        self.scorer = scorer
        self.worker.set_scorer(scorer)
        logger.info("Scorer configurado en CameraViewWindow")
    
    def set_detection_params(self, min_area: int, max_area: int, threshold: float = 0.3):
        """Actualiza parámetros de detección (llamado desde CameraTab)."""
        self.worker.set_params(min_area, max_area, threshold)
    
    def trigger_detection(self):
        """Dispara detección manualmente (llamado desde CameraTab)."""
        if self.last_frame is not None and self.scorer is not None:
            self.worker.detect(self.last_frame)
    
    def clear_detection(self):
        """Limpia resultado de detección."""
        self.detection_result = None
        self.status_label.setText("Limpiado")
    
    def closeEvent(self, event):
        super().closeEvent(event)

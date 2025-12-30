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
    
    # Señales para comunicación con MicroscopyService
    skip_roi_requested = pyqtSignal()
    pause_toggled = pyqtSignal(bool)
    
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
        
        # MEJORA 4: Botones de control de microscopía
        from PyQt5.QtWidgets import QPushButton
        microscopy_ctrl_row = QHBoxLayout()
        
        self.skip_roi_btn = QPushButton("⏭️ No registrar ROI")
        self.skip_roi_btn.setStyleSheet("""
            QPushButton {
                background-color: #E67E22;
                color: white;
                font-weight: bold;
                padding: 8px 15px;
                border-radius: 4px;
            }
            QPushButton:hover { background-color: #D35400; }
            QPushButton:disabled { background-color: #555; color: #888; }
        """)
        self.skip_roi_btn.setEnabled(False)
        self.skip_roi_btn.clicked.connect(self._on_skip_roi)
        microscopy_ctrl_row.addWidget(self.skip_roi_btn)
        
        self.pause_btn = QPushButton("⏸️ Pausar")
        self.pause_btn.setStyleSheet("""
            QPushButton {
                background-color: #3498DB;
                color: white;
                font-weight: bold;
                padding: 8px 15px;
                border-radius: 4px;
            }
            QPushButton:hover { background-color: #2980B9; }
            QPushButton:disabled { background-color: #555; color: #888; }
        """)
        self.pause_btn.setEnabled(False)
        self.pause_btn.clicked.connect(self._on_pause_toggle)
        microscopy_ctrl_row.addWidget(self.pause_btn)
        
        self.learning_label = QLabel("📚 Modo: Normal")
        self.learning_label.setStyleSheet("color: #95A5A6; font-weight: bold;")
        microscopy_ctrl_row.addWidget(self.learning_label)
        
        microscopy_ctrl_row.addStretch()
        layout.addLayout(microscopy_ctrl_row)
        
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
        
        # Tabla profesional de objetos
        from PyQt5.QtWidgets import QTableWidget, QTableWidgetItem, QHeaderView
        self.objects_table = QTableWidget()
        self.objects_table.setColumnCount(3)
        self.objects_table.setHorizontalHeaderLabels(["Nº", "Score", "Área (px)"])
        self.objects_table.setStyleSheet("""
            QTableWidget {
                background-color: #2C2C2C;
                border: 1px solid #505050;
                gridline-color: #404040;
            }
            QTableWidget::item {
                padding: 5px;
                color: #FFFFFF;
            }
            QTableWidget::item:selected {
                background-color: #3498DB;
                color: white;
            }
            QHeaderView::section {
                background-color: #1E1E1E;
                color: #3498DB;
                padding: 5px;
                border: 1px solid #404040;
                font-weight: bold;
            }
        """)
        self.objects_table.horizontalHeader().setStretchLastSection(False)
        self.objects_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Fixed)
        self.objects_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.objects_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.objects_table.setColumnWidth(0, 40)  # Nº
        self.objects_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.objects_table.setSelectionMode(QTableWidget.SingleSelection)
        self.objects_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.objects_table.setMinimumWidth(180)
        self.objects_table.setMaximumWidth(250)
        self.objects_table.itemSelectionChanged.connect(self._on_object_selected)
        objects_layout.addWidget(self.objects_table)
        
        # Mantener referencia a lista antigua para compatibilidad (deprecada)
        self.objects_list = self.objects_table
        
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
        
        # Estado de control de microscopía
        self.is_paused = False
        self.microscopy_active = False
        self.current_point_number = 0
        
        # Estado de autofoco para overlay de score (SIEMPRE VISIBLE)
        self.autofocus_active = False
        self.current_z_position = 0.0
        self.current_focus_score = 0.0
        self.autofocus_status_msg = ""
        
        # Referencia al controlador C-Focus para leer Z en tiempo real
        self.cfocus_controller = None
        
        # Throttling para cálculo de score (evita congelar UI)
        self._last_score_update = 0
        self._score_update_interval = 0.2  # Actualizar score cada 200ms (5 Hz)
        
        # Selección de objeto para resaltar ROI
        self.selected_object_index = None
    
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
        """Actualiza la tabla de objetos detectados con sus datos."""
        from PyQt5.QtWidgets import QTableWidgetItem
        from PyQt5.QtCore import Qt
        from PyQt5.QtGui import QColor
        
        self.objects_table.setRowCount(0)  # Limpiar tabla
        
        if not boxes:
            return
        
        # Llenar tabla con datos de objetos
        for i, box in enumerate(boxes):
            area = box.get('area', 0)
            score = box.get('score', 0)
            in_range = box.get('in_filter_range', False)
            
            self.objects_table.insertRow(i)
            
            # Columna 0: Número
            num_item = QTableWidgetItem(f"{i+1}")
            num_item.setTextAlignment(Qt.AlignCenter)
            if in_range:
                num_item.setForeground(QColor(0, 255, 0))  # Verde
            else:
                num_item.setForeground(QColor(255, 200, 100))  # Amarillo
            self.objects_table.setItem(i, 0, num_item)
            
            # Columna 1: Score
            score_item = QTableWidgetItem(f"{score:.1f}")
            score_item.setTextAlignment(Qt.AlignCenter)
            self.objects_table.setItem(i, 1, score_item)
            
            # Columna 2: Área
            area_item = QTableWidgetItem(f"{area:.0f}")
            area_item.setTextAlignment(Qt.AlignCenter)
            self.objects_table.setItem(i, 2, area_item)
    
    def _on_object_selected(self):
        """Handler cuando se selecciona un objeto en la tabla."""
        selected_rows = self.objects_table.selectedIndexes()
        if selected_rows:
            row = selected_rows[0].row()
            self.selected_object_index = row
            logger.info(f"[CameraWindow] Objeto #{row+1} seleccionado para resaltar")
        else:
            self.selected_object_index = None
    
    # === ACTUALIZACIÓN DE FRAME ===
    
    def update_frame(self, q_image, raw_frame=None):
        """Actualiza visualización - frame EN VIVO con overlay LIVIANO.
        
        El overlay de Z y Score SIEMPRE se muestra y se actualiza en cada frame.
        """
        try:
            self.frame_count += 1
            
            if raw_frame is not None:
                self.last_frame = raw_frame
                # Calcular score en tiempo real si hay scorer configurado
                self._update_realtime_score(raw_frame)
            
            # SIEMPRE dibujar overlay (Z y Score siempre visibles)
            q_image = self._draw_overlay_on_qimage(q_image)
            
            # Mostrar frame
            pixmap = QPixmap.fromImage(q_image)
            scaled = pixmap.scaled(self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.video_label.setPixmap(scaled)
            
            n_obj = self.detection_result.get('n_objects', 0) if self.detection_result else 0
            mode = "🔴 AF" if self.autofocus_active else "🎥"
            self.info_label.setText(f"{mode} LIVE | Frame: {self.frame_count} | S:{self.current_focus_score:.0f}")
            
        except Exception as e:
            logger.error(f"Error update_frame: {e}")
    
    def _update_realtime_score(self, raw_frame):
        """Calcula el score de enfoque con THROTTLING para no bloquear UI.
        
        Solo actualiza cada _score_update_interval segundos (default 200ms).
        También actualiza la posición Z si el C-Focus está conectado.
        """
        import time as time_module
        
        current_time = time_module.time()
        
        # Throttling: solo actualizar si pasó suficiente tiempo
        if current_time - self._last_score_update < self._score_update_interval:
            return
        
        self._last_score_update = current_time
        
        # Actualizar posición Z si C-Focus está disponible (lectura rápida)
        if self.cfocus_controller is not None and self.cfocus_controller.is_connected:
            try:
                z_pos = self.cfocus_controller.read_z()
                if z_pos is not None:
                    self.current_z_position = z_pos
            except Exception:
                pass  # Silenciar errores de lectura Z
        
        # Calcular sharpness si hay scorer (operación costosa, por eso el throttling)
        if self.scorer is None:
            return
        
        try:
            # Calcular sharpness global del frame
            score = self.scorer.calculate_sharpness(raw_frame)
            self.current_focus_score = score
        except Exception as e:
            logger.debug(f"Error calculando score en tiempo real: {e}")
    
    def set_cfocus_controller(self, controller):
        """Configura el controlador C-Focus para lectura de Z en tiempo real."""
        self.cfocus_controller = controller
        logger.info("[CameraWindow] C-Focus controller configurado para lectura Z en tiempo real")
    
    def _draw_overlay_on_qimage(self, q_image):
        """Dibuja overlay A COLOR sobre QImage usando QPainter.
        
        El overlay de Z y Score SIEMPRE se dibuja.
        """
        from PyQt5.QtGui import QPainter, QPen, QColor, QFont
        
        try:
            # Convertir a RGB32 para poder dibujar colores (grayscale no soporta colores)
            if q_image.format() == QImage.Format_Grayscale8:
                result = q_image.convertToFormat(QImage.Format_RGB32)
            else:
                result = q_image.copy()
            
            painter = QPainter(result)
            
            # Dibujar overlays de detección solo si hay detection_result
            if self.detection_result is not None:
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
                        
                        # AZUL si está seleccionado, sino color según filtro
                        if self.selected_object_index is not None and i == self.selected_object_index:
                            pen = QPen(QColor(50, 150, 255), 4)  # AZUL BRILLANTE, grosor 4
                        else:
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
                        in_range = box.get('in_filter_range', False)
                        if in_range:
                            label += " ✓"
                        painter.drawText(x + 2, y - 5, label)
                
                # Mostrar info general en esquina
                n_obj = self.detection_result.get('n_objects', 0)
                if n_obj > 0:
                    painter.setPen(QPen(QColor(255, 255, 255)))
                    painter.drawText(10, 20, f"Objetos: {n_obj}")
            
            # OVERLAY DE SCORE SIEMPRE VISIBLE (esquina superior izquierda, ROJO)
            # Fondo semi-transparente para mejor legibilidad
            painter.setBrush(QColor(0, 0, 0, 200))
            painter.setPen(Qt.NoPen)
            painter.drawRect(5, 5, 280, 75)
            
            # Texto en ROJO GRANDE para Z y Score
            font_large = QFont("Arial", 22, QFont.Bold)
            painter.setFont(font_large)
            painter.setPen(QPen(QColor(255, 50, 50)))  # Rojo brillante
            
            # Mostrar Z y Score (siempre actualizados)
            painter.drawText(12, 35, f"Z: {self.current_z_position:.1f} µm")
            painter.drawText(12, 65, f"S: {self.current_focus_score:.1f}")
            
            # Indicador de estado de autofoco (pequeño, a la derecha)
            if self.autofocus_active:
                font_small = QFont("Arial", 10, QFont.Bold)
                painter.setFont(font_small)
                painter.setPen(QPen(QColor(50, 255, 50)))  # Verde
                painter.drawText(200, 20, "● AF")
            
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
    
    def _on_skip_roi(self):
        """Handler para botón 'No registrar ROI'."""
        logger.info("[CameraWindow] Usuario solicitó saltar ROI actual")
        self.skip_roi_requested.emit()
    
    def _on_pause_toggle(self):
        """Handler para botón 'Pausa/Continuar'."""
        self.is_paused = not self.is_paused
        
        if self.is_paused:
            self.pause_btn.setText("▶️ Continuar")
            self.pause_btn.setStyleSheet("""
                QPushButton { 
                    background-color: #27AE60; 
                    font-size: 12px; 
                    font-weight: bold; 
                    padding: 8px; 
                }
                QPushButton:hover { background-color: #2ECC71; }
            """)
            logger.info("[CameraWindow] Microscopía PAUSADA por usuario")
        else:
            self.pause_btn.setText("⏸️ Pausar")
            self.pause_btn.setStyleSheet("""
                QPushButton { 
                    background-color: #E67E22; 
                    font-size: 12px; 
                    font-weight: bold; 
                    padding: 8px; 
                }
                QPushButton:hover { background-color: #F39C12; }
            """)
            logger.info("[CameraWindow] Microscopía REANUDADA por usuario")
        
        self.pause_toggled.emit(self.is_paused)
    
    def show_autofocus_masks(self, objects_data):
        """
        Muestra máscaras/ROIs durante el autofoco en tiempo real.
        
        Args:
            objects_data: Lista de objetos con bbox, area, circularity, etc.
        """
        if not objects_data:
            return
        
        # Convertir a formato compatible con detection_result
        boxes = []
        for i, obj in enumerate(objects_data):
            boxes.append({
                'bbox': obj.get('bbox', obj.get('bounding_box', (0, 0, 0, 0))),
                'area': obj.get('area', 0),
                'score': obj.get('score', 0),
                'is_focused': obj.get('is_focused', False),
                'in_filter_range': True  # Durante autofoco, todos son válidos
            })
        
        # Actualizar detection_result para mostrar overlay
        self.detection_result = {
            'contours': [],
            'boxes': boxes,
            'frame_size': (640, 480),  # Placeholder
            'n_objects': len(boxes),
            'n_in_range': len(boxes),
            'filter_range': (0, 999999)
        }
        
        # Forzar actualización visual
        logger.info(f"[CameraWindow] Mostrando {len(boxes)} máscaras de autofoco")
    
    def clear_autofocus_masks(self):
        """Limpia las máscaras de autofoco después de completar el proceso."""
        self.detection_result = None
        logger.info("[CameraWindow] Máscaras de autofoco limpiadas")
    
    def set_microscopy_active(self, active: bool, point_number: int = 0):
        """Habilita/deshabilita botones según estado de microscopía."""
        self.microscopy_active = active
        self.current_point_number = point_number
        self.skip_roi_btn.setEnabled(active)
        self.pause_btn.setEnabled(active)
        
        if not active:
            self.is_paused = False
            self.pause_btn.setText("⏸️ Pausar")
    
    # === MÉTODOS PARA OVERLAY DE AUTOFOCO ===
    
    def set_autofocus_active(self, active: bool):
        """Activa/desactiva el overlay de autofoco."""
        self.autofocus_active = active
        if not active:
            self.current_z_position = 0.0
            self.current_focus_score = 0.0
            self.autofocus_status_msg = ""
        logger.info(f"[CameraWindow] Autofocus overlay: {'ACTIVO' if active else 'INACTIVO'}")
    
    def update_autofocus_score(self, z_position: float, score: float):
        """Actualiza el score de autofoco mostrado en el overlay.
        
        Args:
            z_position: Posición Z actual en µm
            score: Score de enfoque actual
        """
        self.current_z_position = z_position
        self.current_focus_score = score
        # No necesita forzar repaint - se actualiza en el próximo frame
    
    def set_autofocus_status(self, message: str):
        """Establece mensaje de estado del autofoco.
        
        Args:
            message: Mensaje corto de estado (se trunca a 30 chars)
        """
        self.autofocus_status_msg = message[:30] if message else ""
    
    def closeEvent(self, event):
        super().closeEvent(event)

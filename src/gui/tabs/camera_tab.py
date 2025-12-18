"""
Pestaña de Control de Cámara Thorlabs.

REFACTORIZACIÓN 2025-12-17:
- UI builders movidos a gui/utils/camera_tab_ui_builder.py
- Lógica de cámara movida a core/services/camera_service.py
- Este archivo solo contiene coordinación UI y señales/slots

Reducción: 1472 → ~450 líneas
"""

import os
import logging
import numpy as np
import cv2
from datetime import datetime

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QScrollArea,
                             QFileDialog, QMessageBox)
from PyQt5.QtCore import pyqtSignal, Qt

from config.hardware_availability import THORLABS_AVAILABLE, Thorlabs
from gui.windows.camera_window import CameraViewWindow
from gui.utils.camera_tab_ui_builder import (
    create_connection_section,
    create_live_view_section,
    create_config_section,
    create_capture_section,
    create_microscopy_section,
    create_autofocus_section,
    create_log_section
)

logger = logging.getLogger('MotorControl_L206')


class CameraTab(QWidget):
    """
    Pestaña para control de cámara Thorlabs y microscopía automatizada.
    
    Solo contiene:
    - Configuración de UI usando builders externos
    - Handlers de UI (actualización de widgets)
    - Conexión de señales con CameraService
    
    Signals:
        exposure_changed: Nuevo valor de exposición (float)
        fps_changed: Nuevo valor de FPS (int)
        buffer_changed: Nuevo valor de buffer (int)
        microscopy_start_requested: Solicita iniciar microscopía (dict config)
        microscopy_stop_requested: Solicita detener microscopía
    """
    
    # Señales para comunicación con servicios externos
    exposure_changed = pyqtSignal(float)
    fps_changed = pyqtSignal(int)
    buffer_changed = pyqtSignal(int)
    microscopy_start_requested = pyqtSignal(dict)
    microscopy_stop_requested = pyqtSignal()
    
    def __init__(self, thorlabs_available=False, parent=None, camera_service=None):
        """
        Inicializa la pestaña de cámara.
        
        Args:
            thorlabs_available: Si pylablib está disponible
            parent: Widget padre (ArduinoGUI)
            camera_service: Instancia de CameraService
        """
        super().__init__(parent)
        self.thorlabs_available = thorlabs_available
        self.parent_gui = parent
        self.camera_service = camera_service
        
        # Configurar disponibilidad en el servicio
        if self.camera_service is not None:
            try:
                self.camera_service.set_thorlabs_available(self.thorlabs_available)
            except Exception:
                pass
        
        # Variables de estado
        self.camera_view_window = None
        self._trajectory_n_points = 0
        self._microscopy_image_counter = 0
        self._pending_capture = False  # Flag para captura después de autofoco
        self.saliency_widget = None  # Widget de saliency (si existe)
        
        # Referencia a TestTab para obtener trayectoria
        self.test_tab = None
        
        # Configurar UI
        self._setup_ui()
        
        # Conectar señales del servicio
        self._connect_service_signals()
        
        logger.debug("CameraTab inicializado (refactorizado)")
    
    # ==================================================================
    # CONFIGURACIÓN DE UI
    # ==================================================================
    
    def _setup_ui(self):
        """Configura la interfaz de usuario usando builders externos."""
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        
        content_widget = QWidget()
        main_layout = QVBoxLayout(content_widget)
        
        # Diccionario para almacenar referencias a widgets
        self._widgets = {}
        
        # Sección 1: Conexión
        main_layout.addWidget(create_connection_section(
            self._widgets, self.thorlabs_available,
            self._on_connect_clicked, self._on_disconnect_clicked, self._on_detect_clicked
        ))
        
        # Sección 2: Vista en vivo
        main_layout.addWidget(create_live_view_section(
            self._widgets,
            self._on_view_clicked, self._on_start_live_clicked, self._on_stop_live_clicked
        ))
        
        # Sección 3: Configuración
        main_layout.addWidget(create_config_section(
            self._widgets,
            self._on_apply_exposure, self._on_apply_fps, self._on_apply_buffer
        ))
        
        # Sección 4: Captura
        main_layout.addWidget(create_capture_section(
            self._widgets,
            self._browse_folder, self._on_capture_clicked, self._on_focus_clicked
        ))
        
        # Sección 5: Microscopía
        main_layout.addWidget(create_microscopy_section(
            self._widgets,
            self.refresh_trajectory_from_test_tab,
            self._on_start_microscopy, self._on_stop_microscopy,
            self._browse_microscopy_folder, self._update_storage_estimate
        ))
        
        # Sección 6: Autofoco
        main_layout.addWidget(create_autofocus_section(
            self._widgets,
            self._on_connect_cfocus, self._on_disconnect_cfocus,
            self._on_test_detection, self._update_detection_params
        ))
        
        # Sección 7: Log
        main_layout.addWidget(create_log_section(
            self._widgets,
            lambda: self._widgets['camera_terminal'].clear()
        ))
        
        # Mapear widgets al objeto
        self._map_widgets()
        
        main_layout.addStretch()
        scroll_area.setWidget(content_widget)
        
        tab_layout = QVBoxLayout(self)
        tab_layout.setContentsMargins(0, 0, 0, 0)
        tab_layout.addWidget(scroll_area)
    
    def _map_widgets(self):
        """Mapea widgets del diccionario a atributos del objeto."""
        # Conexión
        self.connect_btn = self._widgets.get('connect_btn')
        self.disconnect_btn = self._widgets.get('disconnect_btn')
        self.detect_btn = self._widgets.get('detect_btn')
        self.camera_info_label = self._widgets.get('camera_info_label')
        
        # Vista en vivo
        self.view_btn = self._widgets.get('view_btn')
        self.start_live_btn = self._widgets.get('start_live_btn')
        self.stop_live_btn = self._widgets.get('stop_live_btn')
        
        # Configuración
        self.exposure_input = self._widgets.get('exposure_input')
        self.fps_input = self._widgets.get('fps_input')
        self.buffer_input = self._widgets.get('buffer_input')
        self.apply_exposure_btn = self._widgets.get('apply_exposure_btn')
        self.apply_fps_btn = self._widgets.get('apply_fps_btn')
        self.apply_buffer_btn = self._widgets.get('apply_buffer_btn')
        
        # Captura
        self.save_folder_input = self._widgets.get('save_folder_input')
        self.image_format_combo = self._widgets.get('image_format_combo')
        self.use_16bit_check = self._widgets.get('use_16bit_check')
        self.capture_btn = self._widgets.get('capture_btn')
        self.focus_btn = self._widgets.get('focus_btn')
        
        # Volumetría
        self.capture_simple_radio = self._widgets.get('capture_simple_radio')
        self.capture_volumetry_radio = self._widgets.get('capture_volumetry_radio')
        self.volumetry_n_images_spin = self._widgets.get('volumetry_n_images_spin')
        self.volumetry_z_step_spin = self._widgets.get('volumetry_z_step_spin')
        self.volumetry_distribution_combo = self._widgets.get('volumetry_distribution_combo')
        self.volumetry_include_bpof_check = self._widgets.get('volumetry_include_bpof_check')
        self.volumetry_save_json_check = self._widgets.get('volumetry_save_json_check')
        self.volumetry_params_widget = self._widgets.get('volumetry_params_widget')
        
        # Microscopía
        self.trajectory_status = self._widgets.get('trajectory_status')
        self.class_name_input = self._widgets.get('class_name_input')
        self.img_width_input = self._widgets.get('img_width_input')
        self.img_height_input = self._widgets.get('img_height_input')
        self.channel_r_check = self._widgets.get('channel_r_check')
        self.channel_g_check = self._widgets.get('channel_g_check')
        self.channel_b_check = self._widgets.get('channel_b_check')
        self.storage_estimate_label = self._widgets.get('storage_estimate_label')
        self.microscopy_folder_input = self._widgets.get('microscopy_folder_input')
        self.delay_before_input = self._widgets.get('delay_before_input')
        self.delay_after_input = self._widgets.get('delay_after_input')
        self.microscopy_start_btn = self._widgets.get('microscopy_start_btn')
        self.microscopy_stop_btn = self._widgets.get('microscopy_stop_btn')
        self.microscopy_progress_label = self._widgets.get('microscopy_progress_label')
        
        # Autofoco
        self.autofocus_enabled_cb = self._widgets.get('autofocus_enabled_cb')
        self.cfocus_connect_btn = self._widgets.get('cfocus_connect_btn')
        self.cfocus_disconnect_btn = self._widgets.get('cfocus_disconnect_btn')
        self.test_detection_btn = self._widgets.get('test_detection_btn')
        self.full_scan_cb = self._widgets.get('full_scan_cb')
        self.min_pixels_spin = self._widgets.get('min_pixels_spin')
        self.max_pixels_spin = self._widgets.get('max_pixels_spin')
        self.circularity_spin = self._widgets.get('circularity_spin')
        self.aspect_ratio_spin = self._widgets.get('aspect_ratio_spin')
        self.z_range_spin = self._widgets.get('z_range_spin')
        self.z_tolerance_spin = self._widgets.get('z_tolerance_spin')
        self.cfocus_status_label = self._widgets.get('cfocus_status_label')
        
        # Log
        self.camera_terminal = self._widgets.get('camera_terminal')
    
    def _connect_service_signals(self):
        """Conecta señales del CameraService con handlers de UI.
        
        NOTA: Las conexiones principales se hacen en main.py para evitar duplicación.
        Este método solo conecta señales que NO están en main.py.
        """
        if self.camera_service is None:
            return
        
        # Solo conectar error_occurred que no está en main.py
        self.camera_service.error_occurred.connect(self._on_error)
    
    # ==================================================================
    # HANDLERS DE BOTONES (delegan a CameraService)
    # ==================================================================
    
    def _on_connect_clicked(self):
        """Handler para botón Conectar."""
        if self.camera_service is None:
            self.log_message("❌ Error: CameraService no disponible")
            return
        
        try:
            buffer_size = int(self.buffer_input.text())
        except ValueError:
            buffer_size = 2
        
        # CameraService emite status_changed con el mensaje apropiado
        self.camera_service.connect_camera(buffer_size=buffer_size)
    
    def _on_disconnect_clicked(self):
        """Handler para botón Desconectar."""
        if self.camera_service:
            self.camera_service.disconnect_camera()
        
        if self.camera_view_window:
            self.camera_view_window.close()
            self.camera_view_window = None
    
    def _on_detect_clicked(self):
        """Handler para botón Detectar."""
        if self.camera_service is None:
            self.log_message("❌ Error: CameraService no disponible")
            return
        
        self.detect_btn.setEnabled(False)
        cameras = self.camera_service.detect_cameras()
        self.detect_btn.setEnabled(True)
        
        if cameras:
            msg = f"¡Cámaras encontradas! Total: {len(cameras)}\n\n"
            for i, cam in enumerate(cameras, 1):
                msg += f"Cámara {i}: {cam}\n"
            QMessageBox.information(self.parent_gui, "Detección Exitosa", msg)
        else:
            QMessageBox.information(self.parent_gui, "Detección",
                                   "No se encontraron cámaras Thorlabs.\n\n"
                                   "Verificar:\n"
                                   "1. Conexión USB\n"
                                   "2. Drivers instalados\n"
                                   "3. Alimentación de cámara")
    
    def _on_view_clicked(self):
        """Handler para botón Ver Cámara."""
        if not self.camera_service or not self.camera_service.is_connected:
            self.log_message("❌ Error: Conecta la cámara primero")
            QMessageBox.warning(self.parent_gui, "Error", "Conecta la cámara primero")
            return
        
        if self.camera_view_window is None:
            self.camera_view_window = CameraViewWindow(self.parent_gui)
            
            # Configurar SmartFocusScorer
            if self.parent_gui and hasattr(self.parent_gui, 'smart_focus_scorer'):
                self.camera_view_window.set_scorer(self.parent_gui.smart_focus_scorer)
                self.log_message("🔍 SmartFocusScorer configurado")
            
            # Conectar señales con MicroscopyService
            if self.parent_gui and hasattr(self.parent_gui, 'microscopy_service'):
                self.camera_view_window.skip_roi_requested.connect(
                    self.parent_gui.microscopy_service.skip_current_point
                )
                self.camera_view_window.pause_toggled.connect(
                    self.parent_gui.microscopy_service.set_paused
                )
                self.log_message("🔗 Botones de control conectados a MicroscopyService")
        
        self._update_detection_params()
        self.camera_view_window.show()
        self.camera_view_window.raise_()
        self.camera_view_window.activateWindow()
        self.log_message("📹 Ventana de cámara abierta")
    
    def _on_start_live_clicked(self):
        """Handler para botón Iniciar Live."""
        if self.camera_service is None:
            self.log_message("❌ Error: CameraService no disponible")
            return
        
        try:
            exposure_s = float(self.exposure_input.text())
            fps = int(self.fps_input.text())
            buffer_size = int(self.buffer_input.text())
        except ValueError:
            exposure_s, fps, buffer_size = 0.01, 60, 2
        
        self.camera_service.start_live(exposure_s, fps, buffer_size)
        
        self.start_live_btn.setEnabled(False)
        self.stop_live_btn.setEnabled(True)
        self.capture_btn.setEnabled(True)
        self.focus_btn.setEnabled(True)
    
    def _on_stop_live_clicked(self):
        """Handler para botón Detener Live."""
        if self.camera_service:
            self.camera_service.stop_live()
        
        self.start_live_btn.setEnabled(True)
        self.stop_live_btn.setEnabled(False)
        self.capture_btn.setEnabled(False)
        self.focus_btn.setEnabled(False)
    
    def _on_apply_exposure(self):
        """Handler para aplicar exposición."""
        try:
            exposure = float(self.exposure_input.text())
            self.exposure_changed.emit(exposure)
            if self.camera_service:
                self.camera_service.apply_exposure(exposure)
        except ValueError:
            self.log_message("❌ Error: Valor de exposición inválido")
    
    def _on_apply_fps(self):
        """Handler para aplicar FPS."""
        try:
            fps = int(self.fps_input.text())
            self.fps_changed.emit(fps)
            if self.camera_service:
                self.camera_service.apply_fps(fps)
        except ValueError:
            self.log_message("❌ Error: Valor de FPS inválido")
    
    def _on_apply_buffer(self):
        """Handler para aplicar buffer."""
        try:
            buffer_size = int(self.buffer_input.text())
            if buffer_size < 1 or buffer_size > 10:
                self.log_message("❌ Error: Buffer debe estar entre 1 y 10")
                return
            self.buffer_changed.emit(buffer_size)
            if self.camera_service:
                self.camera_service.apply_buffer(buffer_size)
        except ValueError:
            self.log_message("❌ Error: Valor de buffer inválido")
    
    def _on_capture_clicked(self):
        """Handler para botón Capturar."""
        if self.camera_service is None:
            self.log_message("❌ Error: CameraService no disponible")
            return
        
        # Verificar si es volumetría
        if self.capture_volumetry_radio and self.capture_volumetry_radio.isChecked():
            self._start_volumetry_capture()
            return
        
        # Si autofoco está habilitado, ejecutar autofoco primero
        if (self.autofocus_enabled_cb.isChecked() and 
            self.parent_gui and getattr(self.parent_gui, 'cfocus_enabled', False)):
            self.log_message("🎯 Autofoco habilitado - ejecutando Z-scan antes de captura...")
            self._run_autofocus(capture_after=True)
            return
        
        # Captura simple
        folder = self.save_folder_input.text()
        if not folder:
            folder = QFileDialog.getExistingDirectory(self.parent_gui, "Seleccionar Carpeta")
            if folder:
                self.save_folder_input.setText(folder)
        
        if folder:
            img_format = self.image_format_combo.currentText().lower()
            self.camera_service.capture_image(folder, img_format)
    
    def _start_volumetry_capture(self):
        """Inicia captura volumétrica (Método 1: múltiples planos Z)."""
        # Verificar C-Focus conectado
        if not self.parent_gui or not getattr(self.parent_gui, 'cfocus_enabled', False):
            self.log_message("❌ Error: C-Focus no conectado (requerido para volumetría)")
            QMessageBox.warning(self.parent_gui, "Error", 
                              "Volumetría requiere C-Focus conectado para control de Z")
            return
        
        # Verificar SmartFocusScorer disponible
        if not hasattr(self.parent_gui, 'smart_focus_scorer') or self.parent_gui.smart_focus_scorer is None:
            self.log_message("❌ Error: SmartFocusScorer no disponible")
            return
        
        # Obtener carpeta
        folder = self.save_folder_input.text()
        if not folder:
            folder = QFileDialog.getExistingDirectory(self.parent_gui, "Seleccionar Carpeta para Volumetría")
            if folder:
                self.save_folder_input.setText(folder)
            else:
                return
        
        # Construir configuración de volumetría
        z_step = self.volumetry_z_step_spin.value() if self.volumetry_z_step_spin else 1.0
        config = {
            'n_images': self.volumetry_n_images_spin.value() if self.volumetry_n_images_spin else 10,
            'distribution': self.volumetry_distribution_combo.currentText() if self.volumetry_distribution_combo else 'uniform',
            'include_bpof': self.volumetry_include_bpof_check.isChecked() if self.volumetry_include_bpof_check else True,
            'save_json': self.volumetry_save_json_check.isChecked() if self.volumetry_save_json_check else True,
            'save_folder': folder,
            'img_format': self.image_format_combo.currentText().lower(),
            'use_16bit': self.use_16bit_check.isChecked() if self.use_16bit_check else True,
            'z_range': self.z_range_spin.value() if self.z_range_spin else 50.0,
            'z_step': z_step,  # Paso configurable (0.01 - 10.0 µm)
            'min_area': self.min_pixels_spin.value() if self.min_pixels_spin else 5000,
            'max_area': self.max_pixels_spin.value() if self.max_pixels_spin else 50000,
            'score_threshold': 0.3,
            'class_name': 'volumetry',
            'exposure_ms': float(self.exposure_input.text()) if self.exposure_input else 50.0
        }
        
        self.log_message("=" * 40)
        self.log_message("🔬 INICIANDO VOLUMETRÍA")
        self.log_message(f"   Imágenes: {config['n_images']}")
        self.log_message(f"   Distribución: {config['distribution']}")
        self.log_message(f"   Formato: {config['img_format'].upper()} ({'16-bit' if config['use_16bit'] else '8-bit'})")
        self.log_message(f"   Rango Z: ±{config['z_range']}µm, paso: {config['z_step']}µm")
        self.log_message("=" * 40)
        
        # Ejecutar volumetría en thread separado
        import threading
        thread = threading.Thread(target=self._execute_volumetry, args=(config,))
        thread.daemon = True
        thread.start()
    
    def _execute_volumetry(self, config: dict):
        """Ejecuta la volumetría (en thread separado)."""
        from core.services.volumetry_service import VolumetryService
        
        # Crear servicio de volumetría
        volumetry_service = VolumetryService(
            get_current_frame=lambda: self.camera_service.current_frame if self.camera_service else None,
            smart_focus_scorer=self.parent_gui.smart_focus_scorer,
            move_z=self._volumetry_move_z,
            get_z_position=self._volumetry_get_z,
            capture_image=self._volumetry_capture_image,
            parent=None
        )
        
        # Conectar señales
        volumetry_service.volumetry_progress.connect(self._on_volumetry_progress)
        volumetry_service.volumetry_image_captured.connect(self._on_volumetry_image)
        volumetry_service.volumetry_complete.connect(self._on_volumetry_complete)
        volumetry_service.volumetry_error.connect(self._on_volumetry_error)
        
        # Ejecutar
        volumetry_service.start_volumetry(config)
    
    def _volumetry_move_z(self, z_position: float):
        """Mueve el eje Z a la posición especificada (para volumetría)."""
        if self.parent_gui and hasattr(self.parent_gui, 'cfocus_controller'):
            cfocus = self.parent_gui.cfocus_controller
            if cfocus is not None:
                cfocus.move_z(z_position)
    
    def _volumetry_get_z(self) -> float:
        """Obtiene la posición Z actual (para volumetría)."""
        if self.parent_gui and hasattr(self.parent_gui, 'cfocus_controller'):
            cfocus = self.parent_gui.cfocus_controller
            if cfocus is not None:
                z = cfocus.read_z()
                return z if z is not None else 0.0
        return 0.0
    
    def _volumetry_capture_image(self, filepath: str, config: dict) -> bool:
        """Captura y guarda una imagen usando EXACTAMENTE la misma lógica de capture_image."""
        if not self.camera_service or not self.camera_service.worker:
            logger.error("[CameraTab] No hay camera_service o worker disponible")
            return False
        
        if self.camera_service.worker.current_frame is None:
            logger.error("[CameraTab] current_frame es None")
            return False
        
        try:
            # Obtener frame actual - COPIA
            frame = self.camera_service.worker.current_frame.copy()
            img_format = config.get('img_format', 'png')
            
            # EXACTAMENTE la misma lógica de CameraService.capture_image
            if frame.dtype == np.uint16:
                frame_min, frame_max = frame.min(), frame.max()
                logger.debug(f"[Volumetry] Frame: [{frame_min}, {frame_max}]")
                
                if img_format == 'tiff':
                    # TIFF: mantener 16 bits original
                    success = cv2.imwrite(filepath, frame)
                else:
                    # PNG/JPG: normalizar a 8 bits (IGUAL que capture_image)
                    if frame_max > 0:
                        frame_norm = (frame / frame_max * 255).astype(np.uint8)
                    else:
                        frame_norm = np.zeros_like(frame, dtype=np.uint8)
                    
                    if img_format == 'jpg':
                        success = cv2.imwrite(filepath, frame_norm, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    else:  # png
                        success = cv2.imwrite(filepath, frame_norm, [cv2.IMWRITE_PNG_COMPRESSION, 6])
            else:
                # Frame ya es uint8
                if img_format == 'jpg':
                    success = cv2.imwrite(filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                elif img_format == 'png':
                    success = cv2.imwrite(filepath, frame, [cv2.IMWRITE_PNG_COMPRESSION, 6])
                else:
                    success = cv2.imwrite(filepath, frame)
            
            return success
            
        except Exception as e:
            logger.error(f"[CameraTab] Error en _volumetry_capture_image: {e}")
            return False
    
    def _on_volumetry_progress(self, current: int, total: int, z: float):
        """Callback de progreso de volumetría."""
        self.log_message(f"   📸 Capturando {current}/{total} (Z={z:.1f}µm)")
    
    def _on_volumetry_image(self, z: float, score: float, filepath: str):
        """Callback cuando se captura una imagen de volumetría."""
        import os
        filename = os.path.basename(filepath)
        self.log_message(f"   ✅ {filename} (score={score:.2f})")
    
    def _on_volumetry_complete(self, result):
        """Callback cuando termina la volumetría."""
        self.log_message("=" * 40)
        self.log_message("✅ VOLUMETRÍA COMPLETADA")
        self.log_message(f"   Imágenes: {len(result.images)}")
        self.log_message(f"   BPoF: Z={result.z_bpof:.1f}µm (score={result.score_bpof:.2f})")
        self.log_message(f"   Rango detectado: [{result.z_min_detected:.1f}, {result.z_max_detected:.1f}]µm")
        self.log_message(f"   Carpeta: {result.folder_path}")
        self.log_message("=" * 40)
    
    def _on_volumetry_error(self, error_msg: str):
        """Callback de error en volumetría."""
        self.log_message(f"❌ Error en volumetría: {error_msg}")
    
    def _on_focus_clicked(self):
        """Handler para botón Enfocar."""
        if not self.parent_gui or not getattr(self.parent_gui, 'cfocus_enabled', False):
            self.log_message("❌ Error: C-Focus no conectado")
            QMessageBox.warning(self.parent_gui, "Error", "Conecta C-Focus primero")
            return
        
        self.log_message("🎯 Iniciando rutina de enfoque (sin captura)...")
        self._run_autofocus(capture_after=False)
    
    # ==================================================================
    # HANDLERS DE MICROSCOPÍA
    # ==================================================================
    
    def _on_start_microscopy(self):
        """Handler para iniciar microscopía."""
        if self._trajectory_n_points == 0:
            self.log_message("❌ Error: No hay trayectoria generada")
            return
        
        if not (self.channel_r_check.isChecked() or 
                self.channel_g_check.isChecked() or 
                self.channel_b_check.isChecked()):
            self.log_message("❌ Error: Selecciona al menos un canal RGB")
            return
        
        try:
            config = {
                'class_name': self.class_name_input.text().strip().replace(' ', '_'),
                'save_folder': self.microscopy_folder_input.text(),
                'img_width': int(self.img_width_input.text()),
                'img_height': int(self.img_height_input.text()),
                'img_format': self.image_format_combo.currentText().lower(),  # tiff/png/jpg
                'use_16bit': self.use_16bit_check.isChecked(),  # True=16-bit, False=8-bit
                'channels': {
                    'R': self.channel_r_check.isChecked(),
                    'G': self.channel_g_check.isChecked(),
                    'B': self.channel_b_check.isChecked()
                },
                'delay_before': float(self.delay_before_input.text()),
                'delay_after': float(self.delay_after_input.text()),
                'n_points': self._trajectory_n_points,
                'autofocus_enabled': self.autofocus_enabled_cb.isChecked(),
                'min_pixels': self.min_pixels_spin.value(),
                'max_pixels': self.max_pixels_spin.value(),
                'z_range': self.z_range_spin.value(),
                'z_tolerance': self.z_tolerance_spin.value()
            }
        except ValueError as e:
            self.log_message(f"❌ Error en parámetros: {e}")
            return
        
        if not config['save_folder']:
            self.log_message("❌ Error: Selecciona una carpeta de destino")
            return
        
        import os
        os.makedirs(config['save_folder'], exist_ok=True)
        
        # Log de inicio
        self.log_message("=" * 40)
        self.log_message("INICIANDO MICROSCOPÍA AUTOMATIZADA")
        self.log_message(f"   Clase: {config['class_name']}")
        self.log_message(f"   Puntos: {config['n_points']}")
        channels_str = ''.join([c for c in ['R', 'G', 'B'] if config['channels'][c]])
        self.log_message(f"   Canales: {channels_str}")
        fmt = config['img_format'].upper()
        bits = "16-bit" if config['use_16bit'] else "8-bit"
        if fmt == 'JPG' and config['use_16bit']:
            self.log_message(f"   Formato: {fmt} (⚠️ JPG solo soporta 8-bit)")
        else:
            self.log_message(f"   Formato: {fmt} ({bits})")
        self.log_message("=" * 40)
        
        # Actualizar UI
        self.microscopy_start_btn.setEnabled(False)
        self.microscopy_stop_btn.setEnabled(True)
        self._microscopy_image_counter = 0
        self.set_microscopy_progress(0, config['n_points'])
        
        # Deshabilitar volumetría durante microscopía (Método 2 es el único disponible)
        if self.capture_volumetry_radio:
            self.capture_simple_radio.setChecked(True)  # Forzar captura simple
            self.capture_volumetry_radio.setEnabled(False)
            self.capture_simple_radio.setEnabled(False)
        
        if self.camera_view_window:
            self.camera_view_window.set_microscopy_active(True, 0)
        
        self.microscopy_start_requested.emit(config)
    
    def _on_stop_microscopy(self):
        """Handler para detener microscopía."""
        self.log_message("⏹️ DETENIENDO MICROSCOPÍA...")
        self.microscopy_start_btn.setEnabled(True)
        self.microscopy_stop_btn.setEnabled(False)
        
        # Rehabilitar selección de método de captura
        if self.capture_volumetry_radio:
            self.capture_volumetry_radio.setEnabled(True)
            self.capture_simple_radio.setEnabled(True)
        
        if self.camera_view_window:
            self.camera_view_window.set_microscopy_active(False)
        
        self.microscopy_stop_requested.emit()
    
    # ==================================================================
    # HANDLERS DE AUTOFOCO / C-FOCUS
    # ==================================================================
    
    def _on_connect_cfocus(self):
        """Handler para conectar C-Focus."""
        if self.parent_gui:
            success = self.parent_gui.connect_cfocus()
            if success:
                self.cfocus_connect_btn.setEnabled(False)
                self.cfocus_disconnect_btn.setEnabled(True)
                self.cfocus_status_label.setText("C-Focus: ✅ Conectado")
                self.cfocus_status_label.setStyleSheet("color: #27AE60; font-weight: bold;")
                self.parent_gui.initialize_autofocus()
    
    def _on_disconnect_cfocus(self):
        """Handler para desconectar C-Focus."""
        if self.parent_gui and self.parent_gui.cfocus_controller:
            self.parent_gui.disconnect_cfocus()
            self.cfocus_connect_btn.setEnabled(True)
            self.cfocus_disconnect_btn.setEnabled(False)
            self.cfocus_status_label.setText("C-Focus: No conectado")
            self.cfocus_status_label.setStyleSheet("color: #888; font-style: italic;")
            self.log_message("C-Focus desconectado")
    
    def _on_test_detection(self):
        """Handler para test de detección."""
        if self.camera_view_window is None or not self.camera_view_window.isVisible():
            self.log_message("⚠️ Abre la ventana de cámara primero (botón 'Ver')")
            QMessageBox.information(
                self.parent_gui, 
                "Ventana de Cámara",
                "Abre la ventana de cámara primero.\n\n"
                "1. Conecta la cámara\n"
                "2. Presiona 'Ver' para abrir la ventana\n"
                "3. Inicia la vista en vivo\n"
                "4. Presiona 'Test Detección'"
            )
            return
        
        self._update_detection_params()
        self.camera_view_window.trigger_detection()
        self.log_message(f"🔍 TEST Detección - Área: [{self.min_pixels_spin.value()}-{self.max_pixels_spin.value()}] px")
    
    def _update_detection_params(self):
        """Actualiza parámetros de detección."""
        if self.camera_view_window:
            min_area = self.min_pixels_spin.value()
            max_area = self.max_pixels_spin.value()
            self.camera_view_window.set_detection_params(min_area, max_area, threshold=0.3)
        
        if (self.parent_gui and 
            hasattr(self.parent_gui, 'smart_focus_scorer') and 
            self.parent_gui.smart_focus_scorer is not None and
            hasattr(self.parent_gui.smart_focus_scorer, 'set_morphology_params')):
            min_circ = self.circularity_spin.value()
            min_aspect = self.aspect_ratio_spin.value()
            self.parent_gui.smart_focus_scorer.set_morphology_params(
                min_circularity=min_circ,
                min_aspect_ratio=min_aspect
            )
    
    def _run_autofocus(self, capture_after=False):
        """Ejecuta detección + autofoco. Opcionalmente captura después."""
        import numpy as np
        import cv2
        
        # Obtener frame actual
        current_frame = None
        if self.camera_service and self.camera_service.current_frame is not None:
            current_frame = self.camera_service.current_frame
        elif self.camera_worker and self.camera_worker.current_frame is not None:
            current_frame = self.camera_worker.current_frame
        
        if current_frame is None:
            self.log_message("❌ No hay frame disponible")
            return
        
        # Actualizar parámetros de detección
        self._update_detection_params()
        min_area = self.min_pixels_spin.value()
        max_area = self.max_pixels_spin.value()
        self.log_message(f"🔍 Detectando objetos (área: {min_area}-{max_area} px)...")
        
        # Usar el scorer directamente para detección síncrona
        if self.parent_gui and hasattr(self.parent_gui, 'smart_focus_scorer'):
            scorer = self.parent_gui.smart_focus_scorer
            frame = current_frame.copy()
            
            # Convertir frame uint16 -> uint8
            if frame.dtype == np.uint16:
                frame_max = frame.max()
                if frame_max > 0:
                    frame_uint8 = (frame / frame_max * 255).astype(np.uint8)
                else:
                    frame_uint8 = np.zeros_like(frame, dtype=np.uint8)
            else:
                frame_uint8 = frame.astype(np.uint8)
            
            if len(frame_uint8.shape) == 2:
                frame_bgr = cv2.cvtColor(frame_uint8, cv2.COLOR_GRAY2BGR)
            else:
                frame_bgr = frame_uint8
            
            # Detectar objetos
            result = scorer.assess_image(frame_bgr)
            all_objects = result.objects if result.objects else []
            
            # Filtrar por rango de área del usuario
            objects = [obj for obj in all_objects if min_area <= obj.area <= max_area]
            
            if not objects:
                self.log_message(f"⚠️ No hay objetos en rango [{min_area}-{max_area}] px")
                self.log_message(f"   (Detectados {len(all_objects)} objetos totales)")
                if capture_after:
                    self.log_message("   Capturando sin autofoco...")
                    self._do_capture_image()
                return
            
            self.log_message(f"✅ {len(objects)} objeto(s) en rango (de {len(all_objects)} detectados)")
            for i, obj in enumerate(objects):
                self.log_message(f"   #{i+1}: área={obj.area:.0f}px, score={obj.focus_score:.1f}")
            
            # Iniciar autofoco asíncrono
            if hasattr(self.parent_gui, 'autofocus_service') and self.parent_gui.autofocus_service:
                self.log_message("🎯 Iniciando Z-scan autofoco...")
                self.parent_gui.autofocus_service.start_autofocus(objects)
                self._pending_capture = capture_after
            else:
                self.log_message("⚠️ AutofocusService no disponible")
                if capture_after:
                    self._do_capture_image()
        else:
            self.log_message("⚠️ Scorer no disponible")
            if capture_after:
                self._do_capture_image()
    
    # ==================================================================
    # CALLBACKS DE SERVICIO
    # ==================================================================
    
    def _on_camera_connected(self, success: bool, info: str):
        """Callback cuando la cámara se conecta."""
        if success:
            self.set_connected(True, info)
        else:
            self.log_message(f"❌ Fallo al conectar: {info}")
            QMessageBox.critical(self.parent_gui, "Error", f"Fallo al conectar:\n{info}")
            self.set_connected(False)
    
    def on_camera_frame(self, q_image, raw_frame=None):
        """Callback cuando llega un frame de cámara."""
        if self.camera_view_window and self.camera_view_window.isVisible():
            self.camera_view_window.update_frame(q_image, raw_frame)
    
    # Alias para compatibilidad interna
    _on_camera_frame = on_camera_frame
    
    def _on_error(self, error_msg: str):
        """Callback cuando ocurre un error."""
        self.log_message(f"❌ {error_msg}")
    
    # ==================================================================
    # MÉTODOS DE ACTUALIZACIÓN DE UI
    # ==================================================================
    
    def set_connected(self, connected: bool, info: str = ""):
        """Actualiza UI cuando cambia estado de conexión."""
        if connected:
            self.camera_info_label.setText(f"Estado: Conectada - {info}")
            self.camera_info_label.setStyleSheet("color: #27AE60; font-weight: bold;")
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.view_btn.setEnabled(True)
            self.start_live_btn.setEnabled(True)
            self.apply_exposure_btn.setEnabled(True)
            self.apply_fps_btn.setEnabled(True)
            self.apply_buffer_btn.setEnabled(True)
            self.capture_btn.setEnabled(True)
            self.focus_btn.setEnabled(True)
        else:
            self.camera_info_label.setText("Estado: Desconectada")
            self.camera_info_label.setStyleSheet("color: #E74C3C; font-weight: bold;")
            self.connect_btn.setEnabled(self.thorlabs_available)
            self.disconnect_btn.setEnabled(False)
            self.view_btn.setEnabled(False)
            self.start_live_btn.setEnabled(False)
            self.stop_live_btn.setEnabled(False)
            self.apply_exposure_btn.setEnabled(False)
            self.apply_fps_btn.setEnabled(False)
            self.apply_buffer_btn.setEnabled(False)
            self.capture_btn.setEnabled(False)
            self.focus_btn.setEnabled(False)
    
    def set_trajectory_status(self, has_trajectory: bool = None, n_points: int = 0, ready: bool = None):
        """Actualiza estado de trayectoria.
        
        Args:
            has_trajectory: Si hay trayectoria disponible
            n_points: Número de puntos en la trayectoria
            ready: Alias para has_trajectory (compatibilidad con main.py)
        """
        # Compatibilidad: ready es alias de has_trajectory
        if ready is not None:
            has_trajectory = ready
        if has_trajectory is None:
            has_trajectory = False
            
        self._trajectory_n_points = n_points if has_trajectory else 0
        
        if has_trajectory and n_points > 0:
            self.trajectory_status.setText(f"✅ Trayectoria lista: {n_points} puntos")
            self.trajectory_status.setStyleSheet("color: #27AE60; font-weight: bold;")
            self.microscopy_start_btn.setEnabled(True)
        else:
            self.trajectory_status.setText("⚪ Sin trayectoria")
            self.trajectory_status.setStyleSheet("color: #95A5A6; font-weight: bold;")
            self.microscopy_start_btn.setEnabled(False)
        
        self._update_storage_estimate()
    
    def set_microscopy_progress(self, current: int, total: int):
        """Actualiza progreso de microscopía."""
        self.microscopy_progress_label.setText(f"Progreso: {current} / {total} imágenes capturadas")
        
        if current == 0:
            self.microscopy_progress_label.setStyleSheet("font-weight: bold; color: #3498DB;")
        elif current < total:
            self.microscopy_progress_label.setStyleSheet("font-weight: bold; color: #F39C12;")
        else:
            self.microscopy_progress_label.setStyleSheet("font-weight: bold; color: #27AE60;")
    
    def log_message(self, message: str):
        """Escribe un mensaje en la terminal de log."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.camera_terminal.append(f"[{timestamp}] {message}")
    
    # ==================================================================
    # UTILIDADES
    # ==================================================================
    
    def _browse_folder(self):
        """Abre diálogo para seleccionar carpeta."""
        folder = QFileDialog.getExistingDirectory(self, "Seleccionar carpeta de guardado")
        if folder:
            self.save_folder_input.setText(folder)
    
    def _browse_microscopy_folder(self):
        """Abre diálogo para seleccionar carpeta de microscopía."""
        folder = QFileDialog.getExistingDirectory(self, "Seleccionar carpeta para microscopía")
        if folder:
            self.microscopy_folder_input.setText(folder)
    
    def _update_storage_estimate(self):
        """Calcula y actualiza la estimación de almacenamiento."""
        try:
            width = int(self.img_width_input.text()) if self.img_width_input.text() else 1920
            height = int(self.img_height_input.text()) if self.img_height_input.text() else 1080
            
            n_channels = sum([
                self.channel_r_check.isChecked(),
                self.channel_g_check.isChecked(),
                self.channel_b_check.isChecked()
            ])
            
            if n_channels == 0:
                n_channels = 1
            
            bytes_per_pixel = 1 if n_channels == 1 else 3
            n_points = self._trajectory_n_points
            
            bytes_per_image = width * height * bytes_per_pixel * 0.5
            total_bytes = bytes_per_image * max(1, n_points)
            total_mb = total_bytes / (1024 * 1024)
            
            if total_mb < 1:
                self.storage_estimate_label.setText(f"~{total_bytes/1024:.1f} KB")
            elif total_mb < 1024:
                self.storage_estimate_label.setText(f"~{total_mb:.1f} MB")
            else:
                self.storage_estimate_label.setText(f"~{total_mb/1024:.2f} GB")
                
        except ValueError:
            self.storage_estimate_label.setText("~0 MB")
    
    def set_test_tab_reference(self, test_tab):
        """Configura la referencia a TestTab para sincronizar trayectoria."""
        self.test_tab = test_tab
        if hasattr(test_tab, 'trajectory_changed'):
            test_tab.trajectory_changed.connect(self._on_trajectory_changed)
    
    def _on_trajectory_changed(self, n_points):
        """Callback cuando cambia la trayectoria en TestTab."""
        self.set_trajectory_status(n_points > 0, n_points)
    
    def refresh_trajectory_from_test_tab(self):
        """Actualiza el estado de trayectoria desde TestTab."""
        if self.test_tab and hasattr(self.test_tab, 'current_trajectory'):
            trajectory = self.test_tab.current_trajectory
            if trajectory is not None and len(trajectory) > 0:
                self.set_trajectory_status(True, len(trajectory))
                self.log_message(f"📍 Trayectoria sincronizada: {len(trajectory)} puntos")
                return True
        self.set_trajectory_status(False, 0)
        return False
    
    # ==================================================================
    # CALLBACKS DE DETECCIÓN Y AUTOFOCO (conectados desde main)
    # ==================================================================
    
    def on_detection_ready(self, saliency_map, objects):
        """Callback cuando hay nuevos resultados de detección."""
        if hasattr(self, 'saliency_widget') and self.saliency_widget:
            self.saliency_widget.update_detection(saliency_map, objects)
    
    def on_detection_status(self, status: str):
        """Callback cuando cambia el estado del servicio de detección."""
        self.log_message(f"🔍 {status}")
    
    def on_autofocus_started(self, obj_index: int, total: int):
        """Callback cuando inicia autofoco de un objeto."""
        self.log_message(f"🎯 Enfocando objeto {obj_index + 1}/{total}...")
    
    def on_autofocus_z_changed(self, z: float, score: float, roi_frame):
        """Callback en cada posición Z evaluada."""
        if hasattr(self, 'saliency_widget') and self.saliency_widget:
            self.saliency_widget.update_autofocus_state(z, score, 0)
    
    def on_object_focused(self, obj_index: int, z_optimal: float, score: float):
        """Callback cuando se encuentra el foco óptimo de un objeto."""
        self.log_message(f"  ✓ Obj{obj_index}: Z={z_optimal:.1f}µm, S={score:.1f}")
    
    # ==================================================================
    # PROPIEDADES PARA COMPATIBILIDAD
    # ==================================================================
    
    @property
    def camera_worker(self):
        """Retorna el worker de cámara del servicio."""
        if self.camera_service:
            return self.camera_service.worker
        return None
    
    def capture_microscopy_image(self, config: dict, image_index: int) -> bool:
        """Captura una imagen para microscopía (delega a CameraService)."""
        if self.camera_service:
            return self.camera_service.capture_microscopy_image(config, image_index)
        return False
    
    def _do_capture_image(self):
        """Realiza la captura de imagen (sin autofoco). Delega a CameraService."""
        folder = self.save_folder_input.text()
        if not folder:
            folder = QFileDialog.getExistingDirectory(self.parent_gui, "Seleccionar Carpeta")
            if folder:
                self.save_folder_input.setText(folder)
        
        if folder and self.camera_service:
            img_format = self.image_format_combo.currentText().lower()
            self.camera_service.capture_image(folder, img_format)

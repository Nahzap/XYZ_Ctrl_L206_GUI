"""
Pestaña de Control de Cámara Thorlabs.

REFACTORIZACIÓN 2025-12-17:
- UI builders movidos a gui/utils/camera_tab_ui_builder.py
- Lógica de cámara movida a core/services/camera_service.py
- Este archivo solo contiene coordinación UI y señales/slots

Reducción: 1472 → ~450 líneas
"""

import logging
import os
import time
import json
import numpy as np
import cv2
from datetime import datetime

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QScrollArea,
                             QFileDialog, QMessageBox)
from PyQt5.QtCore import pyqtSignal, Qt, QTimer

from gui.windows.camera_window import CameraViewWindow
from gui.tabs.camera_live_bridge import CameraLiveBridge
from gui.utils.camera_tab_ui_builder import (
    create_connection_section,
    create_live_view_section,
    create_config_section,
    create_capture_section,
    create_microscopy_section,
    create_autofocus_section,
    create_u2net_config_section,
    create_log_section
)
from core.services import CameraOrchestrator
from core.models import AutofocusConfig
from core.utils.folder_reveal import reveal_folder
from hardware.camera.scientific_image import save_scientific_image
from utils.parameter_manager import get_parameter_manager

# Defaults Basler acA2500 nativo (se reemplazan al conectar con ROI real)
_DEFAULT_IMG_WIDTH = 2590
_DEFAULT_IMG_HEIGHT = 1942

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
    
    def __init__(self, thorlabs_available=False, parent=None, camera_service=None, camera_orchestrator=None):
        """
        Inicializa la pestaña de cámara.
        
        Args:
            thorlabs_available: Si pylablib está disponible
            parent: Widget padre (CTRL_GUI)
            camera_service: Instancia de CameraService
            camera_orchestrator: Instancia de CameraOrchestrator (NUEVO)
        """
        super().__init__(parent)
        self.thorlabs_available = thorlabs_available
        self.parent_gui = parent
        self.camera_service = camera_service
        self.orchestrator = camera_orchestrator
        
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

        # Path live (SRP): frames → ventana + preview_enabled
        self._live_bridge = CameraLiveBridge(
            get_window=lambda: self.camera_view_window,
            camera_service=self.camera_service,
            sync_resolution=lambda: self._sync_resolution_from_camera(persist=True),
            apply_resolution_from_qimage=lambda w, h: self._apply_camera_resolution(
                w, h, persist=True
            ),
        )
        # Sin ventana al inicio: no construir QImage full/preview en worker
        self._live_bridge.notify_window_visibility(False)
        
        # Configurar UI
        self._setup_ui()
        
        # Conectar señales del orchestrator
        if self.orchestrator:
            self._connect_orchestrator_signals()
        
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
            self._on_browse_folder, self._on_capture_clicked, self._on_focus_clicked
        ))
        
        # Sección 5: Microscopía
        main_layout.addWidget(create_microscopy_section(
            self._widgets,
            self.refresh_trajectory_from_test_tab,
            self._on_start_microscopy, self._on_stop_microscopy,
            self._browse_microscopy_folder, self._update_storage_estimate,
            self._open_microscopy_folder,
        ))
        self._resolution_synced_from_frame = False
        self._opened_save_folder_this_run = False
        
        # Sección 6: Autofoco
        main_layout.addWidget(create_autofocus_section(
            self._widgets,
            self._on_connect_cfocus, self._on_disconnect_cfocus,
            self._on_test_detection, self._update_detection_params
        ))
        
        # Sección 6.5: Configuración Detector U2NET
        main_layout.addWidget(create_u2net_config_section(
            self._widgets,
            self._on_detection_mode_changed,
            self._update_u2net_params
        ))
        
        # Sección 7: Log
        main_layout.addWidget(create_log_section(
            self._widgets,
            lambda: self._widgets['camera_terminal'].clear()
        ))
        
        # Mapear widgets al objeto PRIMERO
        self._map_widgets()
        
        # Cargar parámetros por defecto
        self._load_default_parameters()
        self._setup_settings_persistence()
        
        # Conectar botón de calibración DESPUÉS del mapeo
        self.cfocus_calibrate_btn.clicked.connect(self._on_calibrate_cfocus)
        
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
        
        # Volumetría / Z-Stack
        self.capture_simple_radio = self._widgets.get('capture_simple_radio')
        self.capture_zstack_radio = self._widgets.get('capture_zstack_radio')  # Alias para compatibilidad
        self.capture_volumetry_radio = self._widgets.get('capture_volumetry_radio')
        self.volumetry_n_images_spin = self._widgets.get('volumetry_n_images_spin')
        self.volumetry_z_step_spin = self._widgets.get('volumetry_z_step_spin')
        self.volumetry_distribution_combo = self._widgets.get('volumetry_distribution_combo')
        self.volumetry_include_bpof_check = self._widgets.get('volumetry_include_bpof_check')
        self.volumetry_save_json_check = self._widgets.get('volumetry_save_json_check')
        self.volumetry_params_widget = self._widgets.get('volumetry_params_widget')
        
        # Z-Stack params (mapear también con nombres alternativos)
        self.zstack_n_images_spin = self._widgets.get('zstack_n_images_spin')
        self.zstack_z_step_spin = self._widgets.get('zstack_z_step_spin')
        self.zstack_z_min_spin = self._widgets.get('zstack_z_min_spin')
        self.zstack_z_max_spin = self._widgets.get('zstack_z_max_spin')
        self.zstack_save_json_check = self._widgets.get('zstack_save_json_check')
        self.zstack_params_widget = self._widgets.get('zstack_params_widget')
        self.zstack_channel_r_check = self._widgets.get('zstack_channel_r_check')
        self.zstack_channel_g_check = self._widgets.get('zstack_channel_g_check')
        self.zstack_channel_b_check = self._widgets.get('zstack_channel_b_check')
        self.zstack_storage_estimate_label = self._widgets.get('zstack_storage_estimate_label')
        
        # Microscopía
        self.trajectory_status = self._widgets.get('trajectory_status')
        self.class_name_input = self._widgets.get('class_name_input')
        self.xy_only_cb = self._widgets.get('xy_only_cb')
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
        self.resume_point_spin = self._widgets.get('resume_point_spin')
        self.resume_hint_label = self._widgets.get('resume_hint_label')
        if self.resume_point_spin is not None:
            self.resume_point_spin.valueChanged.connect(self._update_resume_button_label)
        
        # Autofoco
        self.autofocus_enabled_cb = self._widgets.get('autofocus_enabled_cb')
        self.cfocus_connect_btn = self._widgets.get('cfocus_connect_btn')
        self.cfocus_disconnect_btn = self._widgets.get('cfocus_disconnect_btn')
        self.cfocus_calibrate_btn = self._widgets.get('cfocus_calibrate_btn')  # NUEVO
        self.test_detection_btn = self._widgets.get('test_detection_btn')
        self.full_scan_cb = self._widgets.get('full_scan_cb')
        self.min_pixels_spin = self._widgets.get('min_pixels_spin')
        self.max_pixels_spin = self._widgets.get('max_pixels_spin')
        self.circularity_spin = self._widgets.get('circularity_spin')
        self.aspect_ratio_spin = self._widgets.get('aspect_ratio_spin')
        self.z_scan_range_spin = self._widgets.get('z_scan_range_spin')
        self.z_step_coarse_spin = self._widgets.get('z_step_coarse_spin')
        self.z_step_fine_spin = self._widgets.get('z_step_fine_spin')
        self.n_captures_spin = self._widgets.get('n_captures_spin')
        self.z_step_capture_spin = self._widgets.get('z_step_capture_spin')
        self.z_settle_spin = self._widgets.get('z_settle_spin')  # alias Tol. Z
        self.z_arrive_tol_spin = self._widgets.get('z_arrive_tol_spin') or self.z_settle_spin
        self.n_fine_planes_spin = self._widgets.get('n_fine_planes_spin')
        self.roi_margin_spin = self._widgets.get('roi_margin_spin')
        self.estimated_images_label = self._widgets.get('estimated_images_label')
        self.cfocus_status_label = self._widgets.get('cfocus_status_label')
        
        # U2NET Config
        self.detection_mode_combo = self._widgets.get('detection_mode_combo')
        self.saliency_threshold_spin = self._widgets.get('saliency_threshold_spin')
        self.adaptive_k_spin = self._widgets.get('adaptive_k_spin')
        self.morph_kernel_combo = self._widgets.get('morph_kernel_combo')
        self.clahe_clip_spin = self._widgets.get('clahe_clip_spin')
        self.clahe_tile_combo = self._widgets.get('clahe_tile_combo')
        self.u2net_status_label = self._widgets.get('u2net_status_label')
        
        # Terminal
        self.camera_terminal = self._widgets.get('camera_terminal')
        
        # Solo conectar error_occurred que no está en main.py
        if self.camera_service and hasattr(self.camera_service, 'error_occurred'):
            self.camera_service.error_occurred.connect(self._on_error)

        # Wiring dinámico de Z-Stack (canal monobanda + estimación tamaño)
        if self.zstack_z_min_spin:
            self.zstack_z_min_spin.valueChanged.connect(self._update_zstack_storage_estimate)
        if self.zstack_z_max_spin:
            self.zstack_z_max_spin.valueChanged.connect(self._update_zstack_storage_estimate)
        if self.zstack_z_step_spin:
            self.zstack_z_step_spin.valueChanged.connect(self._update_zstack_storage_estimate)
        if self.capture_zstack_radio:
            self.capture_zstack_radio.toggled.connect(self._on_capture_mode_toggled)
        if self.image_format_combo:
            self.image_format_combo.currentTextChanged.connect(self._on_zstack_format_changed)
        if self.use_16bit_check:
            self.use_16bit_check.toggled.connect(lambda _: self._update_zstack_storage_estimate())

        for channel in (self.zstack_channel_r_check, self.zstack_channel_g_check, self.zstack_channel_b_check):
            if channel:
                channel.toggled.connect(lambda checked, ch=channel: self._on_zstack_channel_toggled(ch, checked))

        self._on_capture_mode_toggled(self.capture_zstack_radio.isChecked() if self.capture_zstack_radio else False)
        self._update_zstack_storage_estimate()
    
    def _load_default_parameters(self):
        """Carga el formulario CameraTab desde JSON camera_tab (último guardado).

        Única fuente: parameters['camera_tab']. Sin secciones legacy ni re-aplicación.
        """
        try:
            pm = get_parameter_manager()
            saved = pm.get_camera_tab_defaults()
            if not saved:
                logger.warning(
                    "[CameraTab] Sin sección camera_tab en JSON; "
                    "se mantienen valores del builder hasta el primer guardado"
                )
                return

            camera = saved.get('camera', {})
            capture = saved.get('capture', {})
            microscopy = saved.get('microscopy', {})
            autofocus = saved.get('autofocus', {})
            u2net = saved.get('u2net', {})

            self._set_text(self.exposure_input, camera.get('exposure'))
            self._set_text(self.fps_input, camera.get('fps'))
            self._set_text(self.buffer_input, camera.get('buffer_frames'))

            self._set_text(self.save_folder_input, capture.get('save_folder'))
            self._set_combo(self.image_format_combo, capture.get('image_format'))
            self._set_checked(self.use_16bit_check, capture.get('use_16bit'))
            mode = capture.get('mode')
            if mode == 'zstack' and self.capture_zstack_radio:
                self.capture_zstack_radio.setChecked(True)
            elif mode == 'simple' and self.capture_simple_radio:
                self.capture_simple_radio.setChecked(True)
            zstack = capture.get('zstack', {})
            self._set_value(self.zstack_z_step_spin, zstack.get('z_step_um'))
            self._set_checked(self.zstack_save_json_check, zstack.get('save_json'))
            z_channels = zstack.get('channels', {})
            self._set_checked(self.zstack_channel_r_check, z_channels.get('R'))
            self._set_checked(self.zstack_channel_g_check, z_channels.get('G'))
            self._set_checked(self.zstack_channel_b_check, z_channels.get('B'))

            self._set_text(self.class_name_input, microscopy.get('class_name'))
            self._set_checked(self.xy_only_cb, microscopy.get('xy_only'))
            self._set_text(self.img_width_input, microscopy.get('img_width'))
            self._set_text(self.img_height_input, microscopy.get('img_height'))
            self._set_text(
                self.microscopy_folder_input, microscopy.get('save_folder')
            )
            self._set_text(
                self.delay_before_input, microscopy.get('delay_before_s')
            )
            self._set_text(
                self.delay_after_input, microscopy.get('delay_after_s')
            )
            micro_channels = microscopy.get('channels', {})
            self._set_checked(self.channel_r_check, micro_channels.get('R'))
            self._set_checked(self.channel_g_check, micro_channels.get('G'))
            self._set_checked(self.channel_b_check, micro_channels.get('B'))

            self._set_checked(
                self.autofocus_enabled_cb, autofocus.get('enabled')
            )
            self._set_checked(self.full_scan_cb, autofocus.get('full_scan'))
            for widget, key in (
                (self.min_pixels_spin, 'min_pixels'),
                (self.max_pixels_spin, 'max_pixels'),
                (self.circularity_spin, 'min_circularity'),
                (self.aspect_ratio_spin, 'min_aspect_ratio'),
                (self.z_scan_range_spin, 'z_scan_range_um'),
                (self.z_step_coarse_spin, 'z_step_coarse_um'),
                (self.z_step_fine_spin, 'z_step_fine_um'),
                (self.n_captures_spin, 'n_captures'),
                (self.z_arrive_tol_spin, 'z_arrive_tol_um'),
                (self.n_fine_planes_spin, 'n_fine_planes'),
                (self.roi_margin_spin, 'roi_margin_px'),
            ):
                self._set_value(widget, autofocus.get(key))
            self._set_value(
                self.z_step_capture_spin,
                autofocus.get(
                    'capture_s_drop_percent',
                    autofocus.get('z_step_capture_um'),
                ),
            )

            # Bloquear combo de modo: currentIndexChanged aplica presets y
            # pisaría valores ya restaurados desde camera_tab.
            if self.detection_mode_combo is not None:
                self.detection_mode_combo.blockSignals(True)
            self._set_combo(
                self.detection_mode_combo, u2net.get('detection_mode')
            )
            if self.detection_mode_combo is not None:
                self.detection_mode_combo.blockSignals(False)
            self._set_value(
                self.saliency_threshold_spin, u2net.get('saliency_threshold')
            )
            self._set_value(self.adaptive_k_spin, u2net.get('adaptive_k'))
            self._set_combo(self.morph_kernel_combo, u2net.get('morph_kernel'))
            self._set_value(self.clahe_clip_spin, u2net.get('clahe_clip'))
            self._set_combo(self.clahe_tile_combo, u2net.get('clahe_tile'))

            # Propagar formulario → scorer / U2-Net / autofoco
            self.sync_runtime_params_from_ui()

            logger.info("✅ CameraTab cargado desde JSON camera_tab")
        except Exception as e:
            logger.warning(f"No se pudieron cargar parámetros de CameraTab: {e}")

    @staticmethod
    def _set_text(widget, value):
        if widget is not None and value is not None:
            widget.setText(str(value))

    @staticmethod
    def _set_checked(widget, value):
        if widget is not None and value is not None:
            widget.setChecked(bool(value))

    @staticmethod
    def _set_value(widget, value):
        """Aplica el valor del JSON tal cual; amplía el rango del spin si hace falta."""
        if widget is None or value is None:
            return
        try:
            numeric = float(value) if isinstance(widget.value(), float) else int(round(float(value)))
        except (TypeError, ValueError):
            return
        if hasattr(widget, 'minimum') and hasattr(widget, 'setMinimum'):
            if numeric < widget.minimum():
                widget.setMinimum(numeric)
        if hasattr(widget, 'maximum') and hasattr(widget, 'setMaximum'):
            if numeric > widget.maximum():
                widget.setMaximum(numeric)
        widget.setValue(numeric)

    @staticmethod
    def _set_combo(widget, value):
        if widget is not None and value is not None:
            index = widget.findText(str(value))
            if index >= 0:
                widget.setCurrentIndex(index)

    def _setup_settings_persistence(self):
        """Autoguarda las opciones editables con debounce de 750 ms."""
        self._settings_save_timer = QTimer(self)
        self._settings_save_timer.setSingleShot(True)
        self._settings_save_timer.setInterval(750)
        self._settings_save_timer.timeout.connect(self.save_camera_tab_settings)

        line_edits = (
            self.exposure_input,
            self.fps_input,
            self.buffer_input,
            self.save_folder_input,
            self.class_name_input,
            self.img_width_input,
            self.img_height_input,
            self.microscopy_folder_input,
            self.delay_before_input,
            self.delay_after_input,
        )
        combos = (
            self.image_format_combo,
            self.detection_mode_combo,
            self.morph_kernel_combo,
            self.clahe_tile_combo,
        )
        checks = (
            self.use_16bit_check,
            self.capture_simple_radio,
            self.capture_zstack_radio,
            self.zstack_save_json_check,
            self.zstack_channel_r_check,
            self.zstack_channel_g_check,
            self.zstack_channel_b_check,
            self.xy_only_cb,
            self.channel_r_check,
            self.channel_g_check,
            self.channel_b_check,
            self.autofocus_enabled_cb,
            self.full_scan_cb,
        )
        numeric = (
            self.zstack_z_step_spin,
            self.min_pixels_spin,
            self.max_pixels_spin,
            self.circularity_spin,
            self.aspect_ratio_spin,
            self.z_scan_range_spin,
            self.z_step_coarse_spin,
            self.z_step_fine_spin,
            self.n_captures_spin,
            self.z_step_capture_spin,
            self.z_arrive_tol_spin,
            self.n_fine_planes_spin,
            self.roi_margin_spin,
            self.saliency_threshold_spin,
            self.adaptive_k_spin,
            self.clahe_clip_spin,
        )
        for widget in line_edits:
            if widget:
                widget.textChanged.connect(self._schedule_settings_save)
        for widget in combos:
            if widget:
                widget.currentTextChanged.connect(self._schedule_settings_save)
        for widget in checks:
            if widget:
                widget.toggled.connect(self._schedule_settings_save)
        for widget in numeric:
            if widget:
                widget.valueChanged.connect(self._schedule_settings_save)
        # Materializa la sección camera_tab aun si el usuario solo abre la app.
        self._schedule_settings_save()

    def _schedule_settings_save(self, *_):
        if hasattr(self, '_settings_save_timer'):
            self._settings_save_timer.start()

    @staticmethod
    def _number_from_text(widget, converter, default):
        try:
            return converter(widget.text())
        except (AttributeError, TypeError, ValueError):
            return default

    def _spin_value(self, widget, default):
        try:
            return widget.value() if widget is not None else default
        except (AttributeError, TypeError, ValueError):
            return default

    def read_detection_form_params(self) -> dict:
        """Lee en vivo filtros de detección / morfología desde el formulario."""
        return {
            'min_pixels': int(self._spin_value(self.min_pixels_spin, 500)),
            'max_pixels': int(self._spin_value(self.max_pixels_spin, 3_000_000)),
            'min_circularity': float(self._spin_value(self.circularity_spin, 0.25)),
            'min_aspect_ratio': float(self._spin_value(self.aspect_ratio_spin, 0.25)),
            'saliency_threshold': float(
                self._spin_value(self.saliency_threshold_spin, 0.30)
            ),
            'adaptive_k': float(self._spin_value(self.adaptive_k_spin, 0.5)),
            'clahe_clip': float(self._spin_value(self.clahe_clip_spin, 2.0)),
            'morph_kernel_index': (
                self.morph_kernel_combo.currentIndex()
                if self.morph_kernel_combo is not None else 1
            ),
            'clahe_tile_index': (
                self.clahe_tile_combo.currentIndex()
                if self.clahe_tile_combo is not None else 1
            ),
        }

    def read_autofocus_form_params(self) -> dict:
        """Lee en vivo pasos Z / ROI / tol. llegada Z desde el formulario."""
        if self.z_arrive_tol_spin is None or self.z_step_coarse_spin is None:
            raise RuntimeError("Spins de autofoco no inicializados")
        n_fine = int(self.n_fine_planes_spin.value()) if self.n_fine_planes_spin else 15
        if n_fine % 2 == 0:
            n_fine += 1
        return {
            'autofocus_enabled': bool(
                self.autofocus_enabled_cb.isChecked()
                if self.autofocus_enabled_cb else False
            ),
            'full_scan': bool(
                self.full_scan_cb.isChecked() if self.full_scan_cb else False
            ),
            'z_scan_range_um': float(self.z_scan_range_spin.value()),
            'z_step_coarse': float(self.z_step_coarse_spin.value()),
            'z_step_fine': float(self.z_step_fine_spin.value()),
            'n_fine_planes': n_fine,
            'z_step_capture': round(float(self.z_step_capture_spin.value()), 3),
            'n_captures': int(self.n_captures_spin.value()),
            'z_arrive_tol_um': float(self.z_arrive_tol_spin.value()),
            'roi_margin_px': int(self.roi_margin_spin.value()),
        }

    def sync_runtime_params_from_ui(self, *, apply_u2net_advanced: bool = True) -> dict:
        """Propaga el formulario Camera → U2-Net, SmartFocusScorer y Autofocus.

        Misma idea que TestTab.sync_trajectory_params_from_ui: la UI es la
        fuente de verdad; no dejar valores cacheados en el scorer (microscopía
        usa assess_image del scorer, no solo U2NetDetector).
        """
        det = self.read_detection_form_params()
        af = self.read_autofocus_form_params()
        min_area = max(1, int(det['min_pixels']))
        max_area = max(min_area, int(det['max_pixels']))
        saliency = float(det['saliency_threshold'])
        min_circ = float(det['min_circularity'])
        min_aspect = float(det['min_aspect_ratio'])

        try:
            from core.detection.u2net_detector import U2NetDetector
            detector = U2NetDetector.get_instance()
            detector.set_parameters(
                min_area=min_area,
                max_area=max_area,
                saliency_threshold=saliency,
            )
            if apply_u2net_advanced:
                kernel_sizes = [3, 5, 7]
                tile_sizes = [(4, 4), (8, 8), (16, 16)]
                k_idx = max(0, min(2, int(det['morph_kernel_index'])))
                t_idx = max(0, min(2, int(det['clahe_tile_index'])))
                detector.set_advanced_parameters(
                    saliency_threshold=saliency,
                    adaptive_k=float(det['adaptive_k']),
                    morph_kernel_size=kernel_sizes[k_idx],
                    clahe_clip_limit=float(det['clahe_clip']),
                    clahe_tile_size=tile_sizes[t_idx],
                )
        except Exception as e:
            logger.warning("[CameraTab] No se pudo sincronizar U2NetDetector: %s", e)

        # Scorer usado por microscopía / Test Detección (assess_image)
        scorer = None
        if self.orchestrator is not None:
            scorer = getattr(self.orchestrator, 'scorer', None)
        if scorer is None and self.parent_gui is not None:
            scorer = getattr(self.parent_gui, 'smart_focus_scorer', None)
        if scorer is not None:
            try:
                if hasattr(scorer, 'set_parameters'):
                    scorer.set_parameters(
                        threshold=saliency,
                        min_area=min_area,
                        max_area=max_area,
                    )
                if hasattr(scorer, 'set_morphology_params'):
                    scorer.set_morphology_params(
                        min_circularity=min_circ,
                        min_aspect_ratio=min_aspect,
                    )
                if hasattr(scorer, 'roi_margin'):
                    scorer.roi_margin = int(af['roi_margin_px'])
            except Exception as e:
                logger.warning("[CameraTab] No se pudo sincronizar SmartFocusScorer: %s", e)

        if self.camera_view_window:
            try:
                self.camera_view_window.set_detection_params(
                    min_area, max_area, threshold=saliency
                )
            except Exception:
                pass

        if self.parent_gui is not None:
            det_svc = getattr(self.parent_gui, 'detection_service', None)
            if det_svc is not None and hasattr(det_svc, 'set_parameters'):
                try:
                    det_svc.set_parameters(min_area, max_area, saliency)
                except Exception:
                    pass

        if self.orchestrator is not None:
            try:
                self.orchestrator.update_scorer_morphology_params(
                    min_circularity=min_circ,
                    min_aspect_ratio=min_aspect,
                )
            except Exception:
                pass
            try:
                from core.models import AutofocusConfig
                af_cfg = AutofocusConfig(
                    use_full_range=bool(af['full_scan']),
                    z_scan_range=float(af['z_scan_range_um']),
                    z_step_coarse=float(af['z_step_coarse']),
                    z_step_fine=float(af['z_step_fine']),
                    n_fine_planes=int(af['n_fine_planes']),
                    z_arrive_tol_um=float(af['z_arrive_tol_um']),
                    settle_time=0.0,
                    capture_settle_time=0.0,
                    roi_margin=int(af['roi_margin_px']),
                    n_captures=int(af['n_captures']),
                    z_step_capture=float(af['z_step_capture']),
                )
                self.orchestrator.update_autofocus_params(af_cfg)
            except Exception as e:
                logger.warning("[CameraTab] No se pudo sincronizar Autofocus: %s", e)

        merged = {**det, **af, 'min_pixels': min_area, 'max_pixels': max_area}
        logger.info(
            "[CameraTab] UI→runtime: área=[%d-%d]px circ≥%.2f aspect≥%.2f "
            "saliency=%.2f Z coarse=%.3f Δfine=±%.1f capas=%d tolZ=±%.2f "
            "capture_ΔS=%.1f%% n=%d margin=%dpx",
            min_area,
            max_area,
            min_circ,
            min_aspect,
            saliency,
            af['z_step_coarse'],
            af['z_scan_range_um'],
            af['n_fine_planes'],
            af['z_arrive_tol_um'],
            af['z_step_capture'],
            af['n_captures'],
            af['roi_margin_px'],
        )
        return merged

    def get_microscopy_execution_config(self) -> dict:
        """Config de microscopía leída en vivo del formulario (+ sync runtime)."""
        runtime = self.sync_runtime_params_from_ui()
        delay_before_val = self._get_numeric_widget_value(
            self.delay_before_input, default=0.7
        )
        delay_after_val = self._get_numeric_widget_value(
            self.delay_after_input, default=0.1
        )
        class_name = (
            self.class_name_input.text().strip().replace(' ', '_')
            if self.class_name_input else 'sample'
        )
        xy_only = bool(self.xy_only_cb.isChecked()) if self.xy_only_cb else False
        af_enabled = False if xy_only else bool(runtime.get('autofocus_enabled', False))
        return {
            'class_name': class_name or 'sample',
            'save_folder': (
                self.microscopy_folder_input.text()
                if self.microscopy_folder_input else ''
            ),
            'img_width': int(
                self._number_from_text(
                    self.img_width_input, int, _DEFAULT_IMG_WIDTH
                )
            ),
            'img_height': int(
                self._number_from_text(
                    self.img_height_input, int, _DEFAULT_IMG_HEIGHT
                )
            ),
            'img_format': (
                self.image_format_combo.currentText().lower()
                if self.image_format_combo else 'png'
            ),
            'use_16bit': bool(
                self.use_16bit_check.isChecked() if self.use_16bit_check else True
            ),
            'channels': {
                'R': bool(self.channel_r_check.isChecked()) if self.channel_r_check else False,
                'G': bool(self.channel_g_check.isChecked()) if self.channel_g_check else True,
                'B': bool(self.channel_b_check.isChecked()) if self.channel_b_check else False,
            },
            'delay_before': float(delay_before_val),
            'delay_after': float(delay_after_val),
            'n_points': int(self._trajectory_n_points),
            'autofocus_enabled': af_enabled,
            'xy_only': xy_only,
            'min_pixels': int(runtime['min_pixels']),
            'max_pixels': int(runtime['max_pixels']),
            'min_circularity': float(runtime['min_circularity']),
            'min_aspect_ratio': float(runtime['min_aspect_ratio']),
            'saliency_threshold': float(runtime['saliency_threshold']),
            'z_step_coarse': float(runtime['z_step_coarse']),
            'z_step_fine': float(runtime['z_step_fine']),
            'z_step_capture': float(runtime['z_step_capture']),
            'z_scan_range_um': float(runtime['z_scan_range_um']),
            'n_captures': int(runtime['n_captures']),
            'n_fine_planes': int(runtime['n_fine_planes']),
            'z_arrive_tol_um': float(runtime['z_arrive_tol_um']),
            'roi_margin_px': int(runtime['roi_margin_px']),
            'full_scan': bool(runtime['full_scan']),
            'start_point_1based': int(
                self.resume_point_spin.value()
                if self.resume_point_spin is not None else 1
            ),
        }

    def get_area_range(self) -> tuple:
        """Rango de área en vivo (para MicroscopyService / lambdas)."""
        det = self.read_detection_form_params()
        return int(det['min_pixels']), int(det['max_pixels'])

    def _camera_tab_settings(self):
        """Serializa únicamente opciones de usuario, no estados de conexión."""
        return {
            'version': 1,
            'description': 'Últimas opciones editables de la pestaña Cámara',
            'camera': {
                'exposure': self._number_from_text(
                    self.exposure_input, float, 0.015
                ),
                'fps': self._number_from_text(self.fps_input, int, 30),
                'buffer_frames': self._number_from_text(
                    self.buffer_input, int, 1
                ),
            },
            'capture': {
                'save_folder': self.save_folder_input.text(),
                'image_format': self.image_format_combo.currentText(),
                'use_16bit': self.use_16bit_check.isChecked(),
                'mode': (
                    'zstack'
                    if self.capture_zstack_radio.isChecked()
                    else 'simple'
                ),
                'zstack': {
                    'z_step_um': self.zstack_z_step_spin.value(),
                    'save_json': self.zstack_save_json_check.isChecked(),
                    'channels': {
                        'R': self.zstack_channel_r_check.isChecked(),
                        'G': self.zstack_channel_g_check.isChecked(),
                        'B': self.zstack_channel_b_check.isChecked(),
                    },
                },
            },
            'microscopy': {
                'class_name': self.class_name_input.text(),
                'xy_only': self.xy_only_cb.isChecked(),
                'img_width': self._number_from_text(
                    self.img_width_input, int, _DEFAULT_IMG_WIDTH
                ),
                'img_height': self._number_from_text(
                    self.img_height_input, int, _DEFAULT_IMG_HEIGHT
                ),
                'save_folder': self.microscopy_folder_input.text(),
                'delay_before_s': self._number_from_text(
                    self.delay_before_input, float, 2.0
                ),
                'delay_after_s': self._number_from_text(
                    self.delay_after_input, float, 0.2
                ),
                'channels': {
                    'R': self.channel_r_check.isChecked(),
                    'G': self.channel_g_check.isChecked(),
                    'B': self.channel_b_check.isChecked(),
                },
            },
            'autofocus': {
                'enabled': self.autofocus_enabled_cb.isChecked(),
                'full_scan': self.full_scan_cb.isChecked(),
                'min_pixels': self.min_pixels_spin.value(),
                'max_pixels': self.max_pixels_spin.value(),
                'min_circularity': self.circularity_spin.value(),
                'min_aspect_ratio': self.aspect_ratio_spin.value(),
                'z_scan_range_um': self.z_scan_range_spin.value(),
                'z_step_coarse_um': self.z_step_coarse_spin.value(),
                'z_step_fine_um': self.z_step_fine_spin.value(),
                'n_captures': self.n_captures_spin.value(),
                'capture_s_drop_percent': self.z_step_capture_spin.value(),
                'n_fine_planes': (
                    self.n_fine_planes_spin.value()
                    if self.n_fine_planes_spin else 15
                ),
                'z_arrive_tol_um': (
                    self.z_arrive_tol_spin.value()
                    if self.z_arrive_tol_spin else 0.5
                ),
                'roi_margin_px': self.roi_margin_spin.value(),
            },
            'u2net': {
                'detection_mode': self.detection_mode_combo.currentText(),
                'saliency_threshold': self.saliency_threshold_spin.value(),
                'adaptive_k': self.adaptive_k_spin.value(),
                'morph_kernel': self.morph_kernel_combo.currentText(),
                'clahe_clip': self.clahe_clip_spin.value(),
                'clahe_tile': self.clahe_tile_combo.currentText(),
            },
        }

    def save_camera_tab_settings(self):
        """Persiste inmediatamente el formulario; se usa también al cerrar."""
        try:
            if hasattr(self, '_settings_save_timer'):
                self._settings_save_timer.stop()
            get_parameter_manager().update_camera_tab(
                self._camera_tab_settings()
            )
        except Exception as e:
            logger.warning(f"No se pudieron guardar opciones de CameraTab: {e}")
    
    def _connect_orchestrator_signals(self):
        """Conecta señales del CameraOrchestrator con handlers de UI."""
        if not self.orchestrator:
            return
        
        # Conectar señales de estado
        self.orchestrator.status_message.connect(self.log_message)
        self.orchestrator.validation_error.connect(lambda msg: self.log_message(f"❌ {msg}"))
        
        # Conectar señales de autofoco
        self.orchestrator.autofocus_complete.connect(self._on_orchestrator_autofocus_complete)
        self.orchestrator.detection_complete.connect(self._on_orchestrator_detection_complete)
    
    def _on_orchestrator_autofocus_complete(self, results):
        """Handler cuando el orchestrator completa autofoco."""
        # Si hay captura pendiente, ejecutarla
        if self.orchestrator.is_pending_capture():
            self.log_message("📸 Capturando imagen post-autofoco...")
            self._do_capture_image()
            self.orchestrator.clear_pending_capture()
            # Compatibilidad con callback legado en main.py
            self._pending_capture = False
    
    def _on_orchestrator_detection_complete(self, objects):
        """Handler cuando el orchestrator completa detección."""
        logger.info(f"[CameraTab] ✅ RECIBIDO orchestrator detection_complete: {len(objects)} objetos")
        
        # Actualizar lista de objetos en ventana de cámara
        if self.camera_view_window and self.camera_view_window.isVisible():
            # Crear mapa de saliencia dummy (orchestrator usa SmartFocusScorer, no U2-Net directamente)
            dummy_saliency = np.zeros((100, 100), dtype=np.float32)
            logger.info(f"[CameraTab] Llamando a camera_view_window.update_detection_from_service (orchestrator)")
            self.camera_view_window.update_detection_from_service(dummy_saliency, objects)
            self.log_message(f"✅ {len(objects)} objetos detectados")
        else:
            logger.warning(f"[CameraTab] ⚠️ Ventana de cámara NO visible (orchestrator)")
    
    def _on_microscopy_detection_complete(self, objects):
        """Handler cuando MicroscopyService completa detección durante microscopía."""
        logger.info(f"[CameraTab] ✅ RECIBIDO microscopy detection_complete: {len(objects)} objetos")
        
        # Actualizar lista de objetos en ventana de cámara
        if self.camera_view_window and self.camera_view_window.isVisible():
            # Crear mapa de saliencia dummy (microscopy usa SmartFocusScorer)
            dummy_saliency = np.zeros((100, 100), dtype=np.float32)
            logger.info(f"[CameraTab] Llamando a camera_view_window.update_detection_from_service (microscopy)")
            self.camera_view_window.update_detection_from_service(dummy_saliency, objects)
            self.log_message(f"✅ {len(objects)} objetos detectados (Microscopía)")
        else:
            logger.warning(f"[CameraTab] ⚠️ Ventana de cámara NO visible (microscopy)")
    
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
        logger.info("[CameraTab] _on_view_clicked: abriendo ventana de cámara")
        if not self.camera_service or not self.camera_service.is_connected:
            logger.warning("[CameraTab] Ver cámara rechazado: cámara no conectada")
            self.log_message("❌ Error: Conecta la cámara primero")
            QMessageBox.warning(self.parent_gui, "Error", "Conecta la cámara primero")
            return
        
        streaming = self.camera_service.is_streaming()
        logger.info("[CameraTab] Estado cámara: conectada=True streaming=%s", streaming)
        
        if self.camera_view_window is None:
            logger.info("[CameraTab] Creando CameraViewWindow...")
            t0 = time.perf_counter()
            self.camera_view_window = CameraViewWindow(self.parent_gui)
            logger.info(
                "[CameraTab] CameraViewWindow creada en %.1fms",
                (time.perf_counter() - t0) * 1000.0,
            )
            
            # Configurar SmartFocusScorer desde orchestrator
            if self.orchestrator and self.orchestrator.scorer:
                self.camera_view_window.set_scorer(self.orchestrator.scorer)
                self.log_message("🔍 SmartFocusScorer configurado")
                logger.info("[CameraTab] SmartFocusScorer asignado a ventana de cámara")
            
            # Conectar señales con MicroscopyService
            if self.parent_gui and hasattr(self.parent_gui, 'microscopy_service'):
                self.camera_view_window.skip_roi_requested.connect(
                    self.parent_gui.microscopy_service.skip_current_point
                )
                self.camera_view_window.pause_toggled.connect(
                    self.parent_gui.microscopy_service.set_paused
                )
                self.log_message("🔗 Botones de control conectados a MicroscopyService")
                logger.info("[CameraTab] Señales microscopía conectadas a CameraViewWindow")
            self.camera_view_window.window_closed.connect(
                lambda: self._live_bridge.notify_window_visibility(False)
            )
        
        logger.info("[CameraTab] Actualizando parámetros de detección antes de mostrar ventana")
        self._update_detection_params()
        self.camera_view_window.show()
        self.camera_view_window.raise_()
        self.camera_view_window.activateWindow()
        self._live_bridge.notify_window_visibility(True)
        self.log_message("📹 Ventana de cámara abierta")
        logger.info("[CameraTab] Ventana de cámara visible")
    
    def _on_start_live_clicked(self):
        """Handler para botón Iniciar Live."""
        logger.info("[CameraTab] _on_start_live_clicked")
        if self.camera_service is None:
            logger.error("[CameraTab] start_live: CameraService no disponible")
            self.log_message("❌ Error: CameraService no disponible")
            return
        
        try:
            exposure_s = float(self.exposure_input.text())
            fps = int(self.fps_input.text())
            buffer_size = int(self.buffer_input.text())
        except ValueError:
            logger.warning("[CameraTab] Parámetros live inválidos, usando defaults")
            exposure_s, fps, buffer_size = 0.01, 60, 2
        
        logger.info(
            "[CameraTab] Solicitando live: exp=%ss fps=%d buffer=%d",
            exposure_s, fps, buffer_size,
        )
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
        logger.info("[CameraTab] _on_capture_clicked: Iniciando captura")
        
        if self.camera_service is None:
            self.log_message("❌ Error: CameraService no disponible")
            logger.error("[CameraTab] CameraService no disponible")
            return
        
        # Verificar si es Z-Stack
        if self.capture_zstack_radio and self.capture_zstack_radio.isChecked():
            logger.info("[CameraTab] Modo Z-Stack seleccionado")
            self._start_zstack_capture()
            return
        
        # CAPTURA SIMPLE
        logger.info("[CameraTab] Modo Captura Simple seleccionado")
        self.log_message("📸 Ejecutando captura simple...")
        
        # Obtener carpeta
        folder = self.save_folder_input.text()
        if not folder:
            folder = QFileDialog.getExistingDirectory(self.parent_gui, "Seleccionar Carpeta")
            if folder:
                self.save_folder_input.setText(folder)
        
        if not folder:
            self.log_message("❌ Error: No se seleccionó carpeta")
            logger.warning("[CameraTab] No se seleccionó carpeta de destino")
            return
        
        img_format = self.image_format_combo.currentText().lower()
        logger.info(f"[CameraTab] Capturando imagen simple - Carpeta: {folder}, Formato: {img_format}")
        
        # Capturar directamente SIN autofoco (captura simple inmediata)
        self.camera_service.capture_image(folder, img_format)
        self.log_message(f"✅ Imagen capturada en {folder}")
        logger.info(f"[CameraTab] Captura simple completada: {folder}")
    
    def _start_zstack_capture(self):
        """Inicia captura de Z-Stack (múltiples planos Z comandados por Paso Z)."""
        # Verificar C-Focus conectado
        if not self.parent_gui or not getattr(self.parent_gui, 'cfocus_enabled', False):
            self.log_message("❌ Error: C-Focus no conectado (requerido para Z-Stack)")
            QMessageBox.warning(self.parent_gui, "Error", 
                              "Z-Stack requiere C-Focus conectado para control de Z")
            return
        
        # Verificar calibración del C-Focus
        cfocus = getattr(self.parent_gui, 'cfocus_controller', None) if self.parent_gui else None
        if not cfocus or not hasattr(cfocus, 'get_calibration_info'):
            self.log_message("❌ Error: Controlador C-Focus no disponible")
            return

        calib_info = cfocus.get_calibration_info() or {}
        if not calib_info.get('is_calibrated', False):
            self.log_message("❌ Error: Debes calibrar C-Focus antes del Z-Stack")
            QMessageBox.warning(
                self.parent_gui,
                "Calibración requerida",
                "Debes calibrar C-Focus antes de ejecutar Z-Stack."
            )
            return

        cfocus_z_min = float(calib_info.get('z_min', 0.0))
        cfocus_z_max = float(calib_info.get('z_max', 0.0))

        # Verificar SmartFocusScorer disponible
        if not self.orchestrator or not self.orchestrator.scorer:
            self.log_message("❌ Error: SmartFocusScorer no disponible")
            return
        
        # Obtener carpeta
        folder = self.save_folder_input.text()
        if not folder:
            folder = QFileDialog.getExistingDirectory(self.parent_gui, "Seleccionar Carpeta para Z-Stack")
            if folder:
                self.save_folder_input.setText(folder)
        
        if not folder:
            self.log_message("❌ Error: No se seleccionó carpeta")
            return
        
        # Z min/max se toman SIEMPRE desde calibración de hardware (solo lectura en UI)
        z_min = cfocus_z_min
        z_max = cfocus_z_max
        z_step = self.zstack_z_step_spin.value() if self.zstack_z_step_spin else 0.05
        
        # Validar que z_max > z_min
        if z_max <= z_min:
            self.log_message("❌ Error: Z Max debe ser mayor que Z Min")
            logger.error(f"[CameraTab] Z-Stack inválido: z_min={z_min}, z_max={z_max}")
            QMessageBox.warning(self.parent_gui, "Error", 
                              f"Z Max ({z_max:.2f}µm) debe ser mayor que Z Min ({z_min:.2f}µm)")
            return

        # Reflejar límites hardware en UI por seguridad
        if self.zstack_z_min_spin:
            self.zstack_z_min_spin.setValue(z_min)
        if self.zstack_z_max_spin:
            self.zstack_z_max_spin.setValue(z_max)
        
        # Calcular número de imágenes automáticamente
        z_range_total = z_max - z_min
        n_images = int(z_range_total / z_step) + 1
        if self.zstack_n_images_spin:
            self.zstack_n_images_spin.setValue(n_images)

        # Canal monobanda obligatorio para evitar datos redundantes
        channel_map = {
            'R': self.zstack_channel_r_check.isChecked() if self.zstack_channel_r_check else False,
            'G': self.zstack_channel_g_check.isChecked() if self.zstack_channel_g_check else False,
            'B': self.zstack_channel_b_check.isChecked() if self.zstack_channel_b_check else False,
        }
        selected_channels = [c for c, enabled in channel_map.items() if enabled]
        if len(selected_channels) != 1:
            self.log_message("❌ Error: selecciona exactamente 1 canal para Z-Stack monobanda (R/G/B)")
            QMessageBox.warning(
                self.parent_gui,
                "Canal inválido",
                "Z-Stack monobanda requiere seleccionar exactamente un canal (R, G o B)."
            )
            return
        selected_channel = selected_channels[0]
        
        img_format = self.image_format_combo.currentText().lower() if self.image_format_combo else 'tiff'
        use_16bit = self.use_16bit_check.isChecked() if self.use_16bit_check else True
        if img_format == 'jpg':
            use_16bit = False
            self.log_message("⚠️ JPG solo soporta 8-bit. Se ajusta automáticamente.")

        logger.info(
            f"[CameraTab] Z-Stack config: z_min={z_min:.2f}, z_max={z_max:.2f}, "
            f"z_step={z_step:.3f}, n_images={n_images}, format={img_format}, 16bit={use_16bit}"
        )
        
        config = {
            'z_min': z_min,  # PARÁMETRO EDITABLE POR USUARIO
            'z_max': z_max,  # PARÁMETRO EDITABLE POR USUARIO
            'z_step': z_step,  # Paso Z que COMANDA las slices
            'n_images': n_images,  # CALCULADO AUTOMÁTICAMENTE
            'z_range': z_range_total,  # z_max - z_min
            'save_json': self.zstack_save_json_check.isChecked() if self.zstack_save_json_check else True,
            'save_folder': folder,
            'img_format': img_format,
            'use_16bit': use_16bit,
            'channel_mode': selected_channel,
            'min_area': self.min_pixels_spin.value() if self.min_pixels_spin else 5000,
            'max_area': self.max_pixels_spin.value() if self.max_pixels_spin else 50000,
            'score_threshold': 0.3,
            'class_name': 'zstack',
            'exposure_ms': float(self.exposure_input.text()) if self.exposure_input else 50.0
        }
        
        self.log_message("=" * 60)
        self.log_message("🔬 INICIANDO CAPTURA Z-STACK")
        self.log_message(f"   📍 Rango Z: {z_min:.2f} → {z_max:.2f} µm ({z_range_total:.2f}µm total)")
        self.log_message(f"   📏 Paso Z: {z_step:.3f} µm")
        self.log_message(f"   📸 Imágenes: {n_images} (calculado automáticamente)")
        self.log_message(f"   💾 Carpeta: {folder}")
        self.log_message(f"   🎯 Canal monobanda: {selected_channel}")
        bits_text = "16-bit" if use_16bit else "8-bit"
        self.log_message(f"   🎨 Formato: {img_format.upper()} ({bits_text})")
        self.log_message(f"   📊 JSON: {'Sí' if config['save_json'] else 'No'}")
        self.log_message("=" * 60)
        logger.info(f"[CameraTab] Iniciando Z-Stack: {n_images} imágenes, rango {z_min:.2f}-{z_max:.2f}µm")
        
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
            smart_focus_scorer=self.orchestrator.scorer if self.orchestrator else None,
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
        """Captura Z-Stack solo vía acquire_scientific_frame (única vía CMOS)."""
        if not self.camera_service:
            logger.error("[CameraTab] No hay camera_service disponible")
            return False

        try:
            from hardware.camera.scientific_image import (
                image16_to_u8_preview,
                save_scientific_image,
            )

            sci = self.camera_service.acquire_scientific_frame(timeout_s=2.0)
            frame = np.asarray(sci.image16)
            channel_mode = str(config.get('channel_mode', 'G')).upper()
            img_format = str(config.get('img_format', 'tiff')).lower()
            use_16bit = bool(config.get('use_16bit', True))
            if channel_mode not in ('R', 'G', 'B'):
                channel_mode = 'G'
            if img_format == 'jpg':
                use_16bit = False

            if frame.ndim == 3:
                channel_idx = {'B': 0, 'G': 1, 'R': 2}[channel_mode]
                mono16 = frame[:, :, channel_idx]
            else:
                mono16 = frame

            if use_16bit and img_format in ('png', 'tiff'):
                success = save_scientific_image(
                    filepath, mono16, already_prepared=True
                )
            else:
                from core.utils.image_io import safe_imwrite

                mono8 = image16_to_u8_preview(mono16)
                if img_format == 'jpg':
                    success = safe_imwrite(
                        filepath, mono8, [cv2.IMWRITE_JPEG_QUALITY, 95]
                    )
                else:
                    success = safe_imwrite(filepath, mono8)
            if success:
                dtype_saved = (
                    "uint16"
                    if (use_16bit and img_format in ('png', 'tiff'))
                    else "uint8"
                )
                logger.debug(
                    "[CameraTab] Z-Stack %s canal=%s via %s dtype=%s shape=%s",
                    filepath,
                    channel_mode,
                    sci.pipeline_id,
                    dtype_saved,
                    mono16.shape,
                )
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
        """Handler para botón Enfocar Objetos usando métrica S."""
        logger.info("[CameraTab] _on_focus_clicked: Iniciando rutina de enfoque")
        
        if not self.parent_gui or not getattr(self.parent_gui, 'cfocus_enabled', False):
            self.log_message("❌ Error: C-Focus no conectado")
            logger.error("[CameraTab] C-Focus no conectado para autofoco")
            QMessageBox.warning(self.parent_gui, "Error", "Conecta C-Focus primero")
            return
        
        self.log_message("=" * 50)
        self.log_message("🎯 INICIANDO RUTINA DE ENFOQUE AUTOMÁTICO")
        self.log_message("   Método: SmartFocusScorer con métrica S")
        self.log_message("=" * 50)
        logger.info("[CameraTab] Ejecutando autofoco con SmartFocusScorer")
        
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
            config = self.get_microscopy_execution_config()
            if config['delay_before'] < 0 or config['delay_after'] < 0:
                raise ValueError("Las demoras no pueden ser negativas")
            if not config['class_name']:
                raise ValueError("El nombre de clase no puede estar vacío")
            if config['delay_before'] > 0.5:
                self.log_message(
                    f"⚠️ Aviso: Delay antes ({config['delay_before']}s) "
                    f"se sumará a la pausa de trayectoria."
                )

            # Guardar parámetros para autocompletado futuro
            try:
                pm = get_parameter_manager()
                pm.update_microscopy(
                    class_name=config['class_name'],
                    total_points=config['n_points'],
                    autofocus_enabled=config['autofocus_enabled'],
                    af_min=config['min_pixels'],
                    af_max=config['max_pixels'],
                    channels=''.join([c for c in ['R','G','B'] if config['channels'][c]]),
                    format=config['img_format'].upper(),
                    bit_depth=16 if config['use_16bit'] else 8,
                    delay_before=int(round(config['delay_before'] * 1000)),
                    delay_after=int(round(config['delay_after'] * 1000))
                )
                pm.update_detection(
                    min_circularity=config['min_circularity'],
                    min_aspect_ratio=config['min_aspect_ratio']
                )
                self.save_camera_tab_settings()
            except Exception as e:
                logger.warning(f"No se pudieron guardar parámetros: {e}")
        except ValueError as e:
            self.log_message(f"❌ Error en parámetros: {e}")
            return
        
        if not config['save_folder']:
            self.log_message("❌ Error: Selecciona una carpeta de destino")
            return
        
        save_abs = os.path.abspath(config['save_folder'])
        os.makedirs(save_abs, exist_ok=True)
        config['save_folder'] = save_abs
        if self.microscopy_folder_input:
            self.microscopy_folder_input.setText(save_abs)
        self._opened_save_folder_this_run = False
        
        start_pt = int(config.get('start_point_1based', 1) or 1)
        # Log de inicio
        self.log_message("=" * 40)
        self.log_message(f"💾 Carpeta destino (absoluta): {save_abs}")
        reveal_folder(save_abs, create=False)
        self._opened_save_folder_this_run = True
        if start_pt > 1:
            self.log_message(
                f"CONTINUANDO MICROSCOPÍA DESDE PUNTO {start_pt}/{config['n_points']}"
            )
        else:
            self.log_message("INICIANDO MICROSCOPÍA AUTOMATIZADA")
        self.log_message(f"   Clase: {config['class_name']}")
        self.log_message(f"   Puntos: {config['n_points']} (desde P{start_pt})")
        self.log_message(f"   Autofoco: {'ACTIVADO' if config['autofocus_enabled'] else 'DESACTIVADO'}")
        self.log_message(
            f"   Detección: área=[{config['min_pixels']}-{config['max_pixels']}]px "
            f"circ≥{config['min_circularity']:.2f} aspect≥{config['min_aspect_ratio']:.2f} "
            f"saliency={config['saliency_threshold']:.2f}"
        )
        if config['autofocus_enabled']:
            self.log_message(
                f"   AF Z: coarse={config['z_step_coarse']:.3f} "
                f"fine={config['z_step_fine']:.3f} "
                f"capture_ΔS={config['z_step_capture']:.1f}% "
                f"margin={config['roi_margin_px']}px"
            )
        
        channels_str = ''.join([c for c in ['R', 'G', 'B'] if config['channels'][c]])
        self.log_message(f"   Canales: {channels_str}")
        fmt = config['img_format'].upper()
        bits = "16-bit" if config['use_16bit'] else "8-bit"
        if fmt == 'JPG' and config['use_16bit']:
            self.log_message(f"   Formato: {fmt} (⚠️ JPG solo soporta 8-bit)")
        else:
            self.log_message(f"   Formato: {fmt} ({bits})")
        self.log_message("=" * 40)
        
        if config['autofocus_enabled']:
            if not self.camera_service or not self.camera_service.is_streaming():
                self.log_message("❌ Error: Inicia vista en vivo antes de microscopía con autofoco")
                logger.error("[CameraTab] Microscopía con AF rechazada: cámara sin stream")
                return
            if self.parent_gui and hasattr(self.parent_gui, 'initialize_autofocus'):
                if not self.parent_gui.initialize_autofocus():
                    self.log_message("❌ Error: Configura C-Focus y cámara antes del autofoco")
                    logger.error("[CameraTab] initialize_autofocus falló antes de microscopía")
                    return
                # Re-sync tras initialize_autofocus (puede resetear pasos)
                self.sync_runtime_params_from_ui()
            logger.info(
                "[CameraTab] Microscopía con autofoco: coarse=%.2f fine=%.2f "
                "capture_ΔS=%.1f%%",
                config['z_step_coarse'],
                config['z_step_fine'],
                config['z_step_capture'],
            )
        
        # Actualizar UI
        self.microscopy_start_btn.setEnabled(False)
        self.microscopy_stop_btn.setEnabled(True)
        if self.resume_point_spin is not None:
            self.resume_point_spin.setEnabled(False)
        self._microscopy_image_counter = max(0, start_pt - 1)
        self.set_microscopy_progress(max(0, start_pt - 1), config['n_points'])
        
        # Deshabilitar volumetría durante microscopía (Método 2 es el único disponible)
        if self.capture_volumetry_radio:
            self.capture_simple_radio.setChecked(True)  # Forzar captura simple
            self.capture_volumetry_radio.setEnabled(False)
            self.capture_simple_radio.setEnabled(False)
        
        if self.camera_view_window:
            self.camera_view_window.set_microscopy_active(True, max(0, start_pt - 1))
        
        self.microscopy_start_requested.emit(config)
    
    def _on_stop_microscopy(self):
        """Handler para detener microscopía — corte inmediato de TODO."""
        self.log_message("⏹ DETENIENDO MICROSCOPÍA AHORA (hard stop)...")
        # Emitir PRIMERO para cortar motores/trayectoria/autofoco sin esperar UI
        self.microscopy_stop_requested.emit()

        self.microscopy_start_btn.setEnabled(True)
        self.microscopy_stop_btn.setEnabled(False)
        if self.resume_point_spin is not None:
            self.resume_point_spin.setEnabled(True)
        self._update_resume_button_label()
        
        # Rehabilitar selección de método de captura
        if self.capture_volumetry_radio:
            self.capture_volumetry_radio.setEnabled(True)
            self.capture_simple_radio.setEnabled(True)
        
        if self.camera_view_window:
            self.camera_view_window.set_microscopy_active(False)
    
    # ==================================================================
    # HANDLERS DE AUTOFOCO / C-FOCUS
    # ==================================================================
    
    def _on_connect_cfocus(self):
        """Handler para conectar C-Focus."""
        if self.parent_gui:
            # DESHABILITAR BOTÓN INMEDIATAMENTE para evitar múltiples clics
            self.cfocus_connect_btn.setEnabled(False)
            self.cfocus_connect_btn.setText("Conectando...")
            self.log_message("🔌 Conectando C-Focus...")
            
            # Forzar actualización de UI antes de operación bloqueante
            from PyQt5.QtWidgets import QApplication
            QApplication.processEvents()
            
            success = self.parent_gui.connect_cfocus()
            
            if success:
                self.cfocus_disconnect_btn.setEnabled(True)
                self.cfocus_calibrate_btn.setEnabled(True)
                self.update_cfocus_status(True, "Conectado")
                self.cfocus_connect_btn.setText("Conectar C-Focus")
            else:
                # Re-habilitar si falla
                self.cfocus_connect_btn.setEnabled(True)
                self.cfocus_connect_btn.setText("Conectar C-Focus")
    
    def _on_disconnect_cfocus(self):
        """Handler para desconectar C-Focus."""
        if self.parent_gui and self.parent_gui.cfocus_controller:
            self.parent_gui.disconnect_cfocus()
            self.cfocus_connect_btn.setEnabled(True)
            self.cfocus_disconnect_btn.setEnabled(False)
            self.cfocus_calibrate_btn.setEnabled(False)  # Deshabilitar calibración
            self.update_cfocus_status(False)
            self.log_message("C-Focus desconectado")
    
    def _on_calibrate_cfocus(self):
        """Handler para calibrar C-Focus."""
        if self.parent_gui:
            self.parent_gui.calibrate_cfocus()
            self.update_cfocus_status(True, "Calibrado")
    
    def update_cfocus_status(self, connected: bool, info: str = ""):
        """Actualiza el estado del C-Focus en la UI."""
        if self.cfocus_status_label:
            if connected:
                self.cfocus_status_label.setText(f"C-Focus: {info}")
                self.cfocus_status_label.setStyleSheet("color: #27AE60; font-weight: bold;")
                self.log_message(f"C-Focus conectado: {info}")
                
                # Actualizar rango en Z-Stack UI si está calibrado
                if self.parent_gui and hasattr(self.parent_gui, 'cfocus_controller'):
                    calib_info = self.parent_gui.cfocus_controller.get_calibration_info()
                    if calib_info['is_calibrated']:
                        z_min = calib_info['z_min']
                        z_max = calib_info['z_max']
                        
                        # Actualizar label informativo
                        if 'zstack_cfocus_range_label' in self._widgets:
                            self._widgets['zstack_cfocus_range_label'].setText(f"{z_min:.2f} - {z_max:.2f} µm")
                            self._widgets['zstack_cfocus_range_label'].setStyleSheet("color: #27AE60; font-weight: bold;")
                        
                        # Reflejar límites hardware en los spinboxes de solo lectura
                        if self.zstack_z_min_spin:
                            self.zstack_z_min_spin.setRange(z_min, z_max)
                            self.zstack_z_min_spin.setValue(z_min)
                            logger.info(f"[CameraTab] Z_min configurado: {z_min:.2f} µm")
                        
                        if self.zstack_z_max_spin:
                            self.zstack_z_max_spin.setRange(z_min, z_max)
                            self.zstack_z_max_spin.setValue(z_max)
                            logger.info(f"[CameraTab] Z_max configurado: {z_max:.2f} µm")
                        
                        logger.info(f"[CameraTab] Rango C-Focus actualizado en UI: {z_min:.2f} - {z_max:.2f} µm")
                        self._update_zstack_storage_estimate()
                    else:
                        # Si aún no hay calibración, mostrar rango de hardware para referencia rápida
                        z_hw = float(calib_info.get('z_range_hw', 0.0) or 0.0)
                        if z_hw > 0:
                            if 'zstack_cfocus_range_label' in self._widgets:
                                self._widgets['zstack_cfocus_range_label'].setText(f"0.00 - {z_hw:.2f} µm (HW)")
                                self._widgets['zstack_cfocus_range_label'].setStyleSheet("color: #F39C12; font-weight: bold;")
                            if self.zstack_z_min_spin:
                                self.zstack_z_min_spin.setRange(0.0, z_hw)
                                self.zstack_z_min_spin.setValue(0.0)
                            if self.zstack_z_max_spin:
                                self.zstack_z_max_spin.setRange(0.0, z_hw)
                                self.zstack_z_max_spin.setValue(z_hw)
                            logger.info(f"[CameraTab] Rango hardware C-Focus en UI: 0.00 - {z_hw:.2f} µm")
                            self._update_zstack_storage_estimate()
            else:
                self.cfocus_status_label.setText("C-Focus: No conectado")
                self.cfocus_status_label.setStyleSheet("color: #888; font-style: italic;")
                self.log_message("C-Focus desconectado")
                
                # Resetear rango en Z-Stack UI
                if 'zstack_cfocus_range_label' in self._widgets:
                    self._widgets['zstack_cfocus_range_label'].setText("0.0 - 0.0 µm")
                    self._widgets['zstack_cfocus_range_label'].setStyleSheet("color: #888; font-style: italic;")

                if self.zstack_z_min_spin:
                    self.zstack_z_min_spin.setRange(0.0, 200.0)
                    self.zstack_z_min_spin.setValue(0.0)
                if self.zstack_z_max_spin:
                    self.zstack_z_max_spin.setRange(0.0, 200.0)
                    self.zstack_z_max_spin.setValue(76.0)
                self._update_zstack_storage_estimate()
    
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
    
    def _on_detection_mode_changed(self):
        """Callback cuando cambia el modo de detección (aplica presets)."""
        from core.detection.u2net_detector import U2NetDetector, DetectionMode
        
        logger.info("[CameraTab] _on_detection_mode_changed() LLAMADO")
        
        detector = U2NetDetector.get_instance()
        
        mode_idx = self.detection_mode_combo.currentIndex()
        mode_map = {0: DetectionMode.NORMAL, 1: DetectionMode.SENSITIVE, 2: DetectionMode.ROBUST}
        mode = mode_map.get(mode_idx, DetectionMode.NORMAL)
        
        logger.info(f"[CameraTab] Cambiando modo a: {mode.value} (index={mode_idx})")
        
        # Aplicar preset del modo
        detector.set_detection_mode(mode)

        # Presets de área / morfología usados por microscopía (_get_area_range + scorer)
        preset_filters = {
            DetectionMode.SENSITIVE: {
                "min_pixels": 100,
                "max_pixels": 5_000_000,
                "min_circularity": 0.15,
                "min_aspect_ratio": 0.15,
            },
            DetectionMode.ROBUST: {
                "min_pixels": 1000,
                "max_pixels": 3_000_000,
                "min_circularity": 0.35,
                "min_aspect_ratio": 0.30,
            },
            DetectionMode.NORMAL: {
                "min_pixels": 500,
                "max_pixels": 3_000_000,
                "min_circularity": 0.25,
                "min_aspect_ratio": 0.25,
            },
        }
        filt = preset_filters.get(mode, preset_filters[DetectionMode.NORMAL])
        
        # Actualizar UI con valores del preset (sin disparar callbacks)
        self.saliency_threshold_spin.blockSignals(True)
        self.adaptive_k_spin.blockSignals(True)
        self.morph_kernel_combo.blockSignals(True)
        self.clahe_clip_spin.blockSignals(True)
        self.clahe_tile_combo.blockSignals(True)
        if self.min_pixels_spin:
            self.min_pixels_spin.blockSignals(True)
        if self.max_pixels_spin:
            self.max_pixels_spin.blockSignals(True)
        if self.circularity_spin:
            self.circularity_spin.blockSignals(True)
        if self.aspect_ratio_spin:
            self.aspect_ratio_spin.blockSignals(True)
        
        self.saliency_threshold_spin.setValue(detector.saliency_threshold)
        self.adaptive_k_spin.setValue(detector.adaptive_k)
        self.clahe_clip_spin.setValue(detector.clahe_clip_limit)
        
        kernel_map = {3: 0, 5: 1, 7: 2}
        self.morph_kernel_combo.setCurrentIndex(kernel_map.get(detector.morph_kernel_size, 1))
        
        tile_map = {(4, 4): 0, (8, 8): 1, (16, 16): 2}
        self.clahe_tile_combo.setCurrentIndex(tile_map.get(detector.clahe_tile_size, 1))

        if self.min_pixels_spin:
            self.min_pixels_spin.setValue(filt["min_pixels"])
        if self.max_pixels_spin:
            self.max_pixels_spin.setValue(filt["max_pixels"])
        if self.circularity_spin:
            self.circularity_spin.setValue(filt["min_circularity"])
        if self.aspect_ratio_spin:
            self.aspect_ratio_spin.setValue(filt["min_aspect_ratio"])
        
        self.saliency_threshold_spin.blockSignals(False)
        self.adaptive_k_spin.blockSignals(False)
        self.morph_kernel_combo.blockSignals(False)
        self.clahe_clip_spin.blockSignals(False)
        self.clahe_tile_combo.blockSignals(False)
        if self.min_pixels_spin:
            self.min_pixels_spin.blockSignals(False)
        if self.max_pixels_spin:
            self.max_pixels_spin.blockSignals(False)
        if self.circularity_spin:
            self.circularity_spin.blockSignals(False)
        if self.aspect_ratio_spin:
            self.aspect_ratio_spin.blockSignals(False)

        # Propagar a detector/scorer/UI de cámara
        self._update_detection_params()
        
        # Actualizar label de estado
        params = detector.get_parameters()
        device_str = "GPU" if "cuda" in params['device'] else "CPU"
        model_str = "U2NETP" if params['model_loaded'] else "Contornos"
        if self.u2net_status_label:
            self.u2net_status_label.setText(f"Modelo: {model_str} | Device: {device_str}")
        
        self.log_message(
            f"✅ Modo {mode.value}: thr={params['saliency_threshold']:.2f}, "
            f"área=[{filt['min_pixels']}-{filt['max_pixels']}], "
            f"circ≥{filt['min_circularity']:.2f}, aspect≥{filt['min_aspect_ratio']:.2f}"
        )
        logger.info(
            f"[CameraTab] ✅ Modo aplicado: {mode.value}, thr={params['saliency_threshold']:.2f}, "
            f"k={params['adaptive_k']:.1f}, área=[{filt['min_pixels']}-{filt['max_pixels']}], "
            f"circ≥{filt['min_circularity']:.2f}, aspect≥{filt['min_aspect_ratio']:.2f}"
        )
    
    def _update_u2net_params(self, restore_defaults=False):
        """Actualiza parámetros individuales del detector U2NET desde el formulario."""
        logger.info(
            "[CameraTab] _update_u2net_params() LLAMADO (restore_defaults=%s)",
            restore_defaults,
        )
        if restore_defaults:
            logger.info("[CameraTab] Restaurando defaults...")
            self._on_detection_mode_changed()
            return

        runtime = self.sync_runtime_params_from_ui(apply_u2net_advanced=True)
        self.log_message(
            f"✅ Parámetros U2NET actualizados: thr={runtime['saliency_threshold']:.2f}, "
            f"k={runtime['adaptive_k']:.1f}"
        )
    
    def _update_detection_params(self):
        """Actualiza detección/autofoco desde el formulario (única vía: sync UI→runtime)."""
        runtime = self.sync_runtime_params_from_ui(apply_u2net_advanced=True)
        z_scan_range = float(runtime['z_scan_range_um'])
        n_captures = int(runtime['n_captures'])

        if self.orchestrator and self.estimated_images_label:
            from core.models import AutofocusConfig
            n_fine = int(runtime.get('n_fine_planes', 15))
            z_tol = float(runtime.get('z_arrive_tol_um', 0.5))
            config = AutofocusConfig(
                use_full_range=bool(runtime['full_scan']),
                z_scan_range=z_scan_range,
                z_step_coarse=float(runtime['z_step_coarse']),
                z_step_fine=float(runtime['z_step_fine']),
                n_fine_planes=n_fine,
                z_arrive_tol_um=z_tol,
                settle_time=0.0,
                capture_settle_time=0.0,
                roi_margin=int(runtime['roi_margin_px']),
                n_captures=n_captures,
                z_step_capture=float(runtime['z_step_capture']),
            )
            search_info = self.orchestrator.get_autofocus_search_info()
            if search_info:
                cfocus_limits = None
                if (
                    self.parent_gui
                    and getattr(self.parent_gui, 'cfocus_enabled', False)
                ):
                    cfocus = getattr(self.parent_gui, 'cfocus_controller', None)
                    if cfocus:
                        calib = (
                            cfocus.get_calibration_info()
                            if hasattr(cfocus, 'get_calibration_info')
                            else {}
                        )
                        current_z = (
                            cfocus.read_z() if hasattr(cfocus, 'read_z') else None
                        )
                        cfocus_limits = {
                            'z_min': calib.get('z_min', 0.0),
                            'z_max': calib.get('z_max', 0.0),
                            'current_z': (
                                current_z
                                if current_z is not None
                                else calib.get('z_center', 0.0)
                            ),
                        }

                is_valid, msg = self.orchestrator.validate_autofocus_params(
                    config, cfocus_limits
                )
                if not is_valid:
                    self.estimated_images_label.setText("⚠️ Config inválida")
                    self.estimated_images_label.setStyleSheet(
                        "color: #E74C3C; font-weight: bold;"
                    )
                    self.estimated_images_label.setToolTip(f"⚠️ {msg}")
                else:
                    fine_step = float(runtime['z_step_fine'])
                    fine_half_span = min(
                        z_scan_range, fine_step * max(1, n_fine // 2)
                    )
                    self.estimated_images_label.setText(
                        f"FINE ±{fine_half_span:.1f}µm · {fine_step:.3f}µm · N={n_fine}"
                    )
                    self.estimated_images_label.setStyleSheet(
                        "color: #3498DB; font-weight: bold;"
                    )
                    self.estimated_images_label.setToolTip(
                        f"COARSE → FINE centrado en Z_c*\n"
                        f"Zona FINE real: ±{fine_half_span:.3f}µm · "
                        f"paso {fine_step:.3f}µm · N={n_fine}\n"
                        f"Límite Δ GUI: ±{z_scan_range:.3f}µm\n"
                        f"Paso grueso: {runtime['z_step_coarse']}µm\n"
                        f"Tol. llegada Z: ±{z_tol:.2f}µm (condición, no settle)\n"
                        f"FINE → BPoF → capturas multi-focales: {n_captures} "
                        f"por caída S={runtime['z_step_capture']}% "
                        "(desde curva COARSE+FINE; sin segundo barrido)\n"
                        f"ROI margin: {runtime['roi_margin_px']}px"
                    )
    
    def _run_autofocus(self, capture_after=False):
        """Ejecuta detección + autofoco manual desde la UI."""
        logger.info("[CameraTab] _run_autofocus: Iniciando (capture_after=%s)", capture_after)

        parent = self.parent_gui
        if parent and hasattr(parent, 'initialize_autofocus'):
            if not parent.initialize_autofocus():
                return
            # Reaplicar JSON/UI tras cablear hardware (única vía de params)
            self.sync_runtime_params_from_ui()

        if self.camera_service and not self.camera_service.is_streaming():
            self.log_message("❌ Inicia la vista en vivo de la cámara antes del autofoco")
            logger.error("[CameraTab] Vista en vivo no activa")
            return

        if self.orchestrator and self.orchestrator.autofocus:
            self.orchestrator.autofocus.microscopy_mode = False

        # Obtener frame actual
        current_frame = None
        if self.camera_service is not None:
            try:
                from hardware.camera.scientific_image import image16_to_u8_preview

                sci = self.camera_service.acquire_scientific_frame(timeout_s=1.5)
                # Detección U2-Net: derivado u8 del frame científico (no preview paralelo)
                current_frame = image16_to_u8_preview(sci.image16)
                logger.debug(
                    "[CameraTab] Frame AF desde acquire_scientific_frame id=%s",
                    sci.frame_id,
                )
            except Exception as exc:
                logger.error("[CameraTab] acquire_scientific_frame falló: %s", exc)

        if current_frame is None:
            self.log_message("❌ No hay frame científico disponible")
            logger.error("[CameraTab] No hay frame científico para autofoco")
            return

        # Validar orchestrator y scorer
        if self.orchestrator is None:
            self.log_message("❌ CameraOrchestrator no disponible")
            logger.error("[CameraTab] CameraOrchestrator no disponible")
            if capture_after:
                self._do_capture_image()
            return
        
        if not self.orchestrator.scorer:
            self.log_message("❌ SmartFocusScorer no disponible")
            logger.error("[CameraTab] SmartFocusScorer no está inicializado")
            return
        
        # Actualizar parámetros de detección
        self._update_detection_params()
        min_area = self.min_pixels_spin.value()
        max_area = self.max_pixels_spin.value()
        
        self.log_message(f"🔍 Detectando objetos salientes...")
        self.log_message(f"   Área filtro: {min_area}-{max_area} px")
        self.log_message(f"   Scorer: {self.orchestrator.scorer.__class__.__name__}")
        logger.info(f"[CameraTab] Parámetros autofoco - min_area={min_area}, max_area={max_area}")
        
        # Actualizar frame en orchestrator
        self.orchestrator.set_current_frame(current_frame)
        logger.debug("[CameraTab] Frame actualizado en orchestrator")
        
        # Delegar a orchestrator (usa SmartFocusScorer internamente con métrica S)
        self.log_message("🎯 Ejecutando barrido Z para encontrar mejor plano focal...")
        logger.info("[CameraTab] Delegando autofoco a CameraOrchestrator")
        
        self.orchestrator.run_autofocus(
            capture_after=capture_after,
            min_area=min_area,
            max_area=max_area
        )
        
        logger.info("[CameraTab] Autofoco delegado correctamente")
    
    # ==================================================================
    # CALLBACKS DE SERVICIO
    # ==================================================================
    
    def _apply_camera_resolution(self, width: int, height: int, *, persist: bool = True) -> None:
        """Rellena tamaño imagen con ROI real y opcionalmente persiste en JSON."""
        if width <= 0 or height <= 0:
            return
        changed = False
        if self.img_width_input and self.img_width_input.text().strip() != str(width):
            self.img_width_input.setText(str(width))
            changed = True
        if self.img_height_input and self.img_height_input.text().strip() != str(height):
            self.img_height_input.setText(str(height))
            changed = True
        if changed:
            logger.info(
                "[CameraTab] Tamaño imagen actualizado a %dx%d (resolución real)",
                width,
                height,
            )
            self.log_message(f"📐 Resolución detectada: {width}x{height}px")
            self._update_storage_estimate()
            if persist:
                self.save_camera_tab_settings()

    def _sync_resolution_from_camera(self, *, persist: bool = True) -> bool:
        """Lee resolución del CameraService y actualiza UI. True si aplicó."""
        if not self.camera_service:
            return False
        resolved = self.camera_service.get_resolution()
        if not resolved:
            return False
        width, height = resolved
        self._apply_camera_resolution(width, height, persist=persist)
        return True

    def _on_camera_connected(self, success: bool, info: str):
        """Callback cuando la cámara se conecta."""
        if success:
            self.set_connected(True, info)
            self._resolution_synced_from_frame = False
            self._live_bridge._resolution_synced = False
            if not self._sync_resolution_from_camera(persist=True):
                self.log_message(
                    "⚠️ Cámara conectada; resolución pendiente del primer frame"
                )
        else:
            self.log_message(f"❌ Fallo al conectar: {info}")
            QMessageBox.critical(self.parent_gui, "Error", f"Fallo al conectar:\n{info}")
            self.set_connected(False)
    
    def on_camera_frame(self, q_image, raw_frame=None):
        """Callback live → bridge SRP (DirectConnection desde CameraService)."""
        painted = self._live_bridge.on_frame(q_image, raw_frame)
        if painted:
            self._resolution_synced_from_frame = True
    
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

        # Si solo dicen ready=True sin n_points, conservar el total ya cargado
        if has_trajectory and (n_points is None or int(n_points) <= 0):
            n_points = int(self._trajectory_n_points or 0)
            
        self._trajectory_n_points = int(n_points) if has_trajectory else 0
        
        if has_trajectory and self._trajectory_n_points > 0:
            self.trajectory_status.setText(
                f"✅ Trayectoria lista: {self._trajectory_n_points} puntos"
            )
            self.trajectory_status.setStyleSheet("color: #27AE60; font-weight: bold;")
            self.microscopy_start_btn.setEnabled(True)
            if self.resume_point_spin is not None:
                prev = int(self.resume_point_spin.value())
                self.resume_point_spin.setMaximum(self._trajectory_n_points)
                self.resume_point_spin.setMinimum(1)
                self.resume_point_spin.setValue(
                    min(max(1, prev), self._trajectory_n_points)
                )
                self.resume_point_spin.setEnabled(True)
            if self.resume_hint_label is not None:
                self.resume_hint_label.setText(
                    f"(1…{self._trajectory_n_points}; 1 = inicio)"
                )
            self._update_resume_button_label()
        else:
            self.trajectory_status.setText("⚪ Sin trayectoria")
            self.trajectory_status.setStyleSheet("color: #95A5A6; font-weight: bold;")
            self.microscopy_start_btn.setEnabled(False)
            if self.resume_point_spin is not None:
                self.resume_point_spin.setRange(1, 1)
                self.resume_point_spin.setValue(1)
            if self.resume_hint_label is not None:
                self.resume_hint_label.setText("(sin trayectoria)")
        
        self._update_storage_estimate()

    def _update_resume_button_label(self, *_):
        """Cambia el texto del botón según el punto de continuación."""
        if self.microscopy_start_btn is None:
            return
        pt = 1
        if self.resume_point_spin is not None:
            pt = int(self.resume_point_spin.value())
        n = int(self._trajectory_n_points or 0)
        if pt > 1 and n > 0:
            self.microscopy_start_btn.setText(
                f"▶️ Continuar desde P{pt}/{n}"
            )
            self.microscopy_start_btn.setStyleSheet("""
                QPushButton { font-size: 13px; font-weight: bold; padding: 10px; background-color: #E67E22; }
                QPushButton:hover { background-color: #F39C12; }
                QPushButton:disabled { background-color: #505050; color: #808080; }
            """)
        else:
            self.microscopy_start_btn.setText("🚀 Iniciar / Continuar Microscopía")
            self.microscopy_start_btn.setStyleSheet("""
                QPushButton { font-size: 13px; font-weight: bold; padding: 10px; background-color: #27AE60; }
                QPushButton:hover { background-color: #2ECC71; }
                QPushButton:disabled { background-color: #505050; color: #808080; }
            """)

    def suggest_resume_point(self, point_1based: int, total_points: int = 0):
        """Tras fallo FOV / stop: rellena el spin y habilita Continuar."""
        total = int(total_points) or int(self._trajectory_n_points or 0)
        if total <= 0:
            total = max(1, int(point_1based))
        point = max(1, min(int(point_1based), total))
        self._trajectory_n_points = max(self._trajectory_n_points, total)
        if self.resume_point_spin is not None:
            self.resume_point_spin.blockSignals(True)
            self.resume_point_spin.setMaximum(self._trajectory_n_points)
            self.resume_point_spin.setMinimum(1)
            self.resume_point_spin.setValue(point)
            self.resume_point_spin.setEnabled(True)
            self.resume_point_spin.blockSignals(False)
        if self.resume_hint_label is not None:
            self.resume_hint_label.setText(
                f"⚠ detenido en P{point} — reintenta o elige otro"
            )
            self.resume_hint_label.setStyleSheet(
                "color: #E67E22; font-weight: bold;"
            )
        self.microscopy_start_btn.setEnabled(True)
        self.microscopy_stop_btn.setEnabled(False)
        self._update_resume_button_label()
        self.set_microscopy_progress(max(0, point - 1), self._trajectory_n_points)
        self.log_message(
            f"⏸ Listo para continuar desde punto {point}/{self._trajectory_n_points}. "
            f"Ajusta el spin si quieres saltar y pulsa Continuar."
        )
    
    def set_microscopy_progress(self, current: int, total: int):
        """Actualiza progreso de microscopía."""
        self.microscopy_progress_label.setText(
            f"Progreso: {current} / {total} (punto actual ≈ {min(current + 1, total) if total else 0})"
        )
        
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
    
    def _on_browse_folder(self):
        """Abre diálogo para seleccionar carpeta."""
        folder = QFileDialog.getExistingDirectory(self, "Seleccionar carpeta de guardado")
        if folder:
            self.save_folder_input.setText(folder)
            # Actualizar label de Z-Stack para mostrar dónde se guardarán los datos
            if 'zstack_save_folder_label' in self._widgets:
                self._widgets['zstack_save_folder_label'].setText(folder)
                self._widgets['zstack_save_folder_label'].setStyleSheet("color: #27AE60; font-weight: bold;")
                logger.info(f"[CameraTab] Carpeta Z-Stack actualizada: {folder}")
    
    def _browse_microscopy_folder(self):
        """Abre diálogo para seleccionar carpeta de microscopía."""
        folder = QFileDialog.getExistingDirectory(self, "Seleccionar carpeta para microscopía")
        if folder:
            self.microscopy_folder_input.setText(folder)
            self.save_camera_tab_settings()

    def _open_microscopy_folder(self):
        """Abre la carpeta destino de microscopía en el Explorador."""
        folder = (
            self.microscopy_folder_input.text().strip()
            if self.microscopy_folder_input else ""
        )
        if not folder:
            self.log_message("❌ Error: Carpeta destino vacía")
            return
        if reveal_folder(folder, create=True):
            self.log_message(f"📂 Abriendo: {os.path.abspath(folder)}")
        else:
            self.log_message(f"❌ No se pudo abrir: {folder}")
    
    def _update_storage_estimate(self):
        """Calcula y actualiza la estimación de almacenamiento."""
        try:
            width = int(self.img_width_input.text()) if self.img_width_input.text() else _DEFAULT_IMG_WIDTH
            height = int(self.img_height_input.text()) if self.img_height_input.text() else _DEFAULT_IMG_HEIGHT
            
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

    def _on_zstack_channel_toggled(self, channel_widget, checked: bool):
        """Mantiene selección monobanda (exactamente 1 canal activo)."""
        if not checked:
            # Evitar estado inválido sin canal seleccionado
            if not any([
                self.zstack_channel_r_check.isChecked() if self.zstack_channel_r_check else False,
                self.zstack_channel_g_check.isChecked() if self.zstack_channel_g_check else False,
                self.zstack_channel_b_check.isChecked() if self.zstack_channel_b_check else False,
            ]):
                channel_widget.blockSignals(True)
                channel_widget.setChecked(True)
                channel_widget.blockSignals(False)
            self._update_zstack_storage_estimate()
            return

        # Si se activa uno, desactivar los demás (checkboxes tipo radio)
        for widget in (self.zstack_channel_r_check, self.zstack_channel_g_check, self.zstack_channel_b_check):
            if widget is not None and widget is not channel_widget:
                widget.blockSignals(True)
                widget.setChecked(False)
                widget.blockSignals(False)

        self._update_zstack_storage_estimate()

    def _on_capture_mode_toggled(self, zstack_mode: bool):
        """Ajusta UI al modo de captura seleccionado."""
        if zstack_mode:
            self.log_message("ℹ️ Z-Stack: monobanda activa. Puedes usar TIFF/PNG/JPG.")
            self._on_zstack_format_changed(self.image_format_combo.currentText() if self.image_format_combo else "TIFF")
        else:
            if self.image_format_combo:
                self.image_format_combo.setEnabled(True)
            if self.use_16bit_check:
                self.use_16bit_check.setEnabled(True)
        self._update_zstack_storage_estimate()

    def _on_zstack_format_changed(self, fmt_text: str):
        """Ajusta opciones de profundidad al formato seleccionado en modo Z-Stack."""
        if not self.capture_zstack_radio or not self.capture_zstack_radio.isChecked():
            return
        fmt = (fmt_text or "").strip().upper()
        if fmt == "JPG":
            if self.use_16bit_check:
                self.use_16bit_check.setChecked(False)
                self.use_16bit_check.setEnabled(False)
            self.log_message("ℹ️ Z-Stack JPG: guardado en 8-bit (limitación del formato).")
        else:
            if self.use_16bit_check:
                self.use_16bit_check.setEnabled(True)
        self._update_zstack_storage_estimate()

    def _update_zstack_storage_estimate(self):
        """Estimación rápida de tamaño para stack monobanda."""
        if not self.zstack_storage_estimate_label:
            return
        try:
            z_min = self.zstack_z_min_spin.value() if self.zstack_z_min_spin else 0.0
            z_max = self.zstack_z_max_spin.value() if self.zstack_z_max_spin else 0.0
            z_step = self.zstack_z_step_spin.value() if self.zstack_z_step_spin else 0.1
            if z_step <= 0 or z_max < z_min:
                self.zstack_storage_estimate_label.setText("~0 MB")
                return

            n_images = int((z_max - z_min) / z_step) + 1
            if self.zstack_n_images_spin:
                self.zstack_n_images_spin.setValue(n_images)

            if self.camera_service:
                resolved = self.camera_service.get_resolution()
                width, height = resolved if resolved else (_DEFAULT_IMG_WIDTH, _DEFAULT_IMG_HEIGHT)
            else:
                width, height = _DEFAULT_IMG_WIDTH, _DEFAULT_IMG_HEIGHT

            fmt = self.image_format_combo.currentText().strip().upper() if self.image_format_combo else "TIFF"
            use_16bit = self.use_16bit_check.isChecked() if self.use_16bit_check else True
            if fmt == "JPG":
                effective_bpp = 1
                bit_label = "8-bit"
            else:
                effective_bpp = 2 if use_16bit else 1
                bit_label = "16-bit" if use_16bit else "8-bit"

            total_bytes = max(1, n_images) * width * height * effective_bpp
            if total_bytes < 1024 * 1024:
                size_text = f"~{total_bytes / 1024:.1f} KB"
            elif total_bytes < 1024 * 1024 * 1024:
                size_text = f"~{total_bytes / (1024 * 1024):.1f} MB"
            else:
                size_text = f"~{total_bytes / (1024 * 1024 * 1024):.2f} GB"

            channel = 'G'
            if self.zstack_channel_r_check and self.zstack_channel_r_check.isChecked():
                channel = 'R'
            elif self.zstack_channel_b_check and self.zstack_channel_b_check.isChecked():
                channel = 'B'
            self.zstack_storage_estimate_label.setText(
                f"{size_text} ({n_images} img, mono-{channel}, {bit_label}, {fmt})"
            )
        except Exception:
            self.zstack_storage_estimate_label.setText("~0 MB")
    
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
        logger.info(f"[CameraTab] ✅ RECIBIDO detection_ready: {len(objects)} objetos")
        
        if hasattr(self, 'saliency_widget') and self.saliency_widget:
            self.saliency_widget.update_detection(saliency_map, objects)
            logger.info(f"[CameraTab] Saliency widget actualizado")
        
        # CRÍTICO: Actualizar lista de objetos en ventana de cámara
        if self.camera_view_window and self.camera_view_window.isVisible():
            logger.info(f"[CameraTab] Ventana de cámara visible, actualizando lista de objetos...")
            self.camera_view_window.update_detection_from_service(saliency_map, objects)
            self.log_message(f"✅ {len(objects)} objetos detectados por SAM")
            logger.info(f"[CameraTab] Lista de objetos actualizada en ventana de cámara")
        else:
            logger.warning(f"[CameraTab] ⚠️ Ventana de cámara NO visible: window={self.camera_view_window}, visible={self.camera_view_window.isVisible() if self.camera_view_window else 'N/A'}")
    
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

    def save_manual_autofocus_stacks(self, results: list) -> int:
        """Guarda exactamente N planos GUI por resultado de autofoco manual."""
        if not results:
            return 0

        config = self.get_microscopy_execution_config()
        save_folder = str(config.get("save_folder", "") or "").strip()
        if not save_folder and self.save_folder_input is not None:
            save_folder = self.save_folder_input.text().strip()
        if not save_folder:
            self.log_message(
                "❌ Autofoco calculó los planos, pero falta carpeta destino"
            )
            return 0

        expected_n = int(self.read_autofocus_form_params()["n_captures"])
        class_name = str(config.get("class_name", "sample") or "sample")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        total_saved = 0

        for result in results:
            frames = list(getattr(result, "frames", None) or [])
            z_positions = list(getattr(result, "z_positions", None) or [])
            scores = list(getattr(result, "focus_scores", None) or [])
            if len(frames) != expected_n:
                self.log_message(
                    f"❌ Obj{result.object_index}: stack incompleto "
                    f"{len(frames)}/{expected_n}; no se guarda parcialmente"
                )
                continue

            records = []
            object_saved = 0
            created_paths = []
            for index in range(expected_n):
                frame = frames[index]
                if frame is None or getattr(frame, "size", 0) == 0:
                    break
                frame_to_save = np.asarray(frame)
                filename = (
                    f"{class_name}_manualAF_{stamp}_"
                    f"obj{int(result.object_index):02d}_f{index}.png"
                )
                filepath = os.path.join(save_folder, filename)
                if not save_scientific_image(
                    filepath, frame_to_save, already_prepared=True
                ):
                    break
                created_paths.append(filepath)
                object_saved += 1
                records.append(
                    {
                        "file": filename,
                        "f_index": index,
                        "z_um": (
                            round(float(z_positions[index]), 6)
                            if index < len(z_positions) else None
                        ),
                        "S": (
                            round(float(scores[index]), 6)
                            if index < len(scores) else None
                        ),
                        "is_bpof": index
                        == int(getattr(result, "bpof_index", -1)),
                        "channels": (
                            int(frame_to_save.shape[2])
                            if frame_to_save.ndim == 3 else 1
                        ),
                    }
                )

            if object_saved != expected_n:
                for created_path in created_paths:
                    try:
                        os.remove(created_path)
                    except OSError:
                        pass
                self.log_message(
                    f"❌ Obj{result.object_index}: guardado incompleto "
                    f"{object_saved}/{expected_n}"
                )
                continue

            metadata_name = (
                f"{class_name}_manualAF_{stamp}_"
                f"obj{int(result.object_index):02d}_focus.json"
            )
            metadata_path = os.path.join(save_folder, metadata_name)
            metadata = {
                "mode": "manual_autofocus",
                "object_index": int(result.object_index),
                "requested_n_captures": expected_n,
                "saved_n_captures": object_saved,
                "z_bpof_um": float(result.z_optimal),
                "bpof_index": int(getattr(result, "bpof_index", -1)),
                "capture_mode": str(
                    getattr(result, "capture_mode", "optical_s_drop")
                ),
                "captures": records,
                "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            }
            try:
                os.makedirs(save_folder, exist_ok=True)
                with open(metadata_path, "w", encoding="utf-8") as stream:
                    json.dump(metadata, stream, indent=2, ensure_ascii=False)
            except Exception as exc:
                self.log_message(f"⚠️ No se guardó metadata AF manual: {exc}")

            total_saved += object_saved
            self.log_message(
                f"✅ Obj{result.object_index}: {object_saved}/{expected_n} "
                f"planos RGB16 guardados en {save_folder}"
            )

        return total_saved
    
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

    def _set_numeric_widget_value(self, widget, value: float):
        """Setea valores numéricos tanto en QLineEdit como en spinboxes."""
        if widget is None:
            return
        if hasattr(widget, 'setValue'):
            widget.setValue(value)
            return
        if hasattr(widget, 'setText'):
            widget.setText(f"{value:.3f}".rstrip('0').rstrip('.'))

    def _get_numeric_widget_value(self, widget, default: float = 0.0) -> float:
        """Lee valores numéricos tanto de QLineEdit como de spinboxes."""
        if widget is None:
            return default
        if hasattr(widget, 'value'):
            return float(widget.value())
        if hasattr(widget, 'text'):
            text_value = widget.text().strip()
            return float(text_value) if text_value else default
        return default

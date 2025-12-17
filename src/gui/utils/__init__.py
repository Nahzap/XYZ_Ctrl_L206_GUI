"""
Utilidades de GUI.
"""

from .trajectory_preview import show_trajectory_preview
from .csv_utils import export_trajectory_csv, import_trajectory_csv, get_trajectory_stats
from .test_tab_ui_builder import (
    create_controllers_section,
    create_motor_sensor_section,
    create_calibration_section,
    create_position_control_section,
    create_trajectory_section,
    create_zigzag_section
)
from .camera_tab_ui_builder import (
    create_connection_section,
    create_live_view_section,
    create_config_section,
    create_capture_section,
    create_microscopy_section,
    create_autofocus_section,
    create_log_section
)

__all__ = [
    'show_trajectory_preview',
    'export_trajectory_csv',
    'import_trajectory_csv',
    'get_trajectory_stats',
    # TestTab builders
    'create_controllers_section',
    'create_motor_sensor_section',
    'create_calibration_section',
    'create_position_control_section',
    'create_trajectory_section',
    'create_zigzag_section',
    # CameraTab builders
    'create_connection_section',
    'create_live_view_section',
    'create_config_section',
    'create_capture_section',
    'create_microscopy_section',
    'create_autofocus_section',
    'create_log_section'
]

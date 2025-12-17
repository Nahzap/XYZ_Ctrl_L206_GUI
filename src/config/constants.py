"""
Constantes del sistema físico y configuración serial.

MEJORAS 2025-12-17:
- Calibración cargada desde calibration.json (NO hardcodeada)
- Funciones de conversión dinámicas
- Recarga en caliente con reload_calibration()
"""

import json
import os
import logging

logger = logging.getLogger('MotorControl_L206')

# --- CONFIGURACIÓN SERIAL ---
SERIAL_PORT = 'COM5' 
BAUD_RATE = 1000000
PLOT_LENGTH = 100

# =============================================================================
# CARGA DINÁMICA DE CALIBRACIÓN DESDE JSON
# =============================================================================

_CONFIG_DIR = os.path.dirname(os.path.abspath(__file__))
_CALIBRATION_FILE = os.path.join(_CONFIG_DIR, 'calibration.json')

# Valores por defecto (solo si no existe el archivo JSON)
_DEFAULT_CALIBRATION = {
    'x_axis': {'intercept_um': 21601.0, 'slope_um_per_adc': 12.22},
    'y_axis': {'intercept_um': 21601.0, 'slope_um_per_adc': 12.22}
}
_DEFAULT_CONTROL = {
    'deadzone_adc': 2,
    'position_tolerance_um': 25.0,
    'settling_cycles': 4,
    'max_attempts_per_point': 500,
    'fallback_tolerance_multiplier': 2.0,
    'default_trajectory_pause_s': 2.0
}
_DEFAULT_SYSTEM = {
    'adc_max': 1023.0,
    'recorrido_um': 20000.0
}


def _load_calibration() -> dict:
    """Carga la calibración desde el archivo JSON."""
    if os.path.exists(_CALIBRATION_FILE):
        try:
            with open(_CALIBRATION_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                logger.info(f"✅ Calibración cargada desde {_CALIBRATION_FILE}")
                return data
        except Exception as e:
            logger.warning(f"⚠️ Error cargando calibration.json: {e}. Usando valores por defecto.")
    else:
        logger.warning(f"⚠️ No existe {_CALIBRATION_FILE}. Usando valores por defecto.")
    
    return {
        'calibration': _DEFAULT_CALIBRATION,
        'control': _DEFAULT_CONTROL,
        'system': _DEFAULT_SYSTEM
    }


def save_calibration(calibration_x: dict, calibration_y: dict, control: dict = None) -> bool:
    """
    Guarda la calibración en el archivo JSON.
    
    Args:
        calibration_x: {'intercept': float, 'slope': float}
        calibration_y: {'intercept': float, 'slope': float}
        control: Parámetros de control opcionales
        
    Returns:
        True si se guardó correctamente
    """
    try:
        data = {
            "_comment": "Archivo de calibración del sistema XYZ. Editar según mediciones reales.",
            "_updated": __import__('datetime').datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "calibration": {
                "x_axis": {
                    "description": "Motor A, Sensor 2 - Eje X",
                    "intercept_um": calibration_x.get('intercept', calibration_x.get('intercept_um', 21601.0)),
                    "slope_um_per_adc": calibration_x.get('slope', calibration_x.get('slope_um_per_adc', 12.22))
                },
                "y_axis": {
                    "description": "Motor B, Sensor 1 - Eje Y",
                    "intercept_um": calibration_y.get('intercept', calibration_y.get('intercept_um', 21601.0)),
                    "slope_um_per_adc": calibration_y.get('slope', calibration_y.get('slope_um_per_adc', 12.22))
                }
            },
            "control": control or {
                "deadzone_adc": DEADZONE_ADC,
                "position_tolerance_um": POSITION_TOLERANCE_UM,
                "settling_cycles": SETTLING_CYCLES,
                "default_trajectory_pause_s": DEFAULT_TRAJECTORY_PAUSE
            },
            "system": {
                "adc_max": ADC_MAX,
                "recorrido_um": RECORRIDO_UM
            }
        }
        
        with open(_CALIBRATION_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=4, ensure_ascii=False)
        
        logger.info(f"✅ Calibración guardada en {_CALIBRATION_FILE}")
        reload_calibration()
        return True
        
    except Exception as e:
        logger.error(f"❌ Error guardando calibración: {e}")
        return False


def reload_calibration():
    """Recarga la calibración desde el archivo JSON."""
    global CALIBRATION_X, CALIBRATION_Y, DEADZONE_ADC, POSITION_TOLERANCE_UM
    global SETTLING_CYCLES, DEFAULT_TRAJECTORY_PAUSE, ADC_MAX, RECORRIDO_UM, FACTOR_ESCALA
    global MAX_ATTEMPTS_PER_POINT, FALLBACK_TOLERANCE_MULTIPLIER
    
    data = _load_calibration()
    
    # Calibración de ejes
    cal = data.get('calibration', _DEFAULT_CALIBRATION)
    x_cal = cal.get('x_axis', _DEFAULT_CALIBRATION['x_axis'])
    y_cal = cal.get('y_axis', _DEFAULT_CALIBRATION['y_axis'])
    
    CALIBRATION_X = {
        'intercept': x_cal.get('intercept_um', 21601.0),
        'slope': x_cal.get('slope_um_per_adc', 12.22)
    }
    CALIBRATION_Y = {
        'intercept': y_cal.get('intercept_um', 21601.0),
        'slope': y_cal.get('slope_um_per_adc', 12.22)
    }
    
    # Parámetros de control
    ctrl = data.get('control', _DEFAULT_CONTROL)
    DEADZONE_ADC = ctrl.get('deadzone_adc', 2)
    POSITION_TOLERANCE_UM = ctrl.get('position_tolerance_um', 25.0)
    SETTLING_CYCLES = ctrl.get('settling_cycles', 4)
    MAX_ATTEMPTS_PER_POINT = ctrl.get('max_attempts_per_point', 500)
    FALLBACK_TOLERANCE_MULTIPLIER = ctrl.get('fallback_tolerance_multiplier', 2.0)
    DEFAULT_TRAJECTORY_PAUSE = ctrl.get('default_trajectory_pause_s', 2.0)
    
    # Sistema
    sys_cfg = data.get('system', _DEFAULT_SYSTEM)
    ADC_MAX = sys_cfg.get('adc_max', 1023.0)
    RECORRIDO_UM = sys_cfg.get('recorrido_um', 20000.0)
    FACTOR_ESCALA = RECORRIDO_UM / ADC_MAX
    
    logger.info(f"📐 Calibración X: intercept={CALIBRATION_X['intercept']}µm, slope={CALIBRATION_X['slope']}µm/ADC")
    logger.info(f"📐 Calibración Y: intercept={CALIBRATION_Y['intercept']}µm, slope={CALIBRATION_Y['slope']}µm/ADC")
    logger.info(f"⚙️ Control: deadzone={DEADZONE_ADC}ADC, tolerance={POSITION_TOLERANCE_UM}µm, settling={SETTLING_CYCLES}, max_attempts={MAX_ATTEMPTS_PER_POINT}")


# --- Cargar calibración al importar el módulo ---
CALIBRATION_X = {}
CALIBRATION_Y = {}
DEADZONE_ADC = 2
POSITION_TOLERANCE_UM = 25.0
SETTLING_CYCLES = 4
MAX_ATTEMPTS_PER_POINT = 500
FALLBACK_TOLERANCE_MULTIPLIER = 2.0
DEFAULT_TRAJECTORY_PAUSE = 2.0
ADC_MAX = 1023.0
RECORRIDO_UM = 20000.0
FACTOR_ESCALA = RECORRIDO_UM / ADC_MAX

reload_calibration()


# =============================================================================
# FUNCIONES DE CONVERSIÓN
# =============================================================================

def um_to_adc(um: float, axis: str = 'x') -> float:
    """
    Convierte posición en µm a valor ADC.
    
    Args:
        um: Posición en micrómetros
        axis: 'x' o 'y' para seleccionar calibración
        
    Returns:
        Valor ADC correspondiente (0-1023)
    """
    cal = CALIBRATION_X if axis.lower() == 'x' else CALIBRATION_Y
    adc = (cal['intercept'] - um) / cal['slope']
    return max(0, min(ADC_MAX, adc))


def adc_to_um(adc: float, axis: str = 'x') -> float:
    """
    Convierte valor ADC a posición en µm.
    
    Args:
        adc: Valor ADC (0-1023)
        axis: 'x' o 'y' para seleccionar calibración
        
    Returns:
        Posición en micrómetros
    """
    cal = CALIBRATION_X if axis.lower() == 'x' else CALIBRATION_Y
    return cal['intercept'] - (adc * cal['slope'])


def get_calibration_info() -> dict:
    """Retorna información de calibración actual para mostrar en UI."""
    return {
        'x': CALIBRATION_X.copy(),
        'y': CALIBRATION_Y.copy(),
        'deadzone_adc': DEADZONE_ADC,
        'tolerance_um': POSITION_TOLERANCE_UM,
        'settling_cycles': SETTLING_CYCLES,
        'config_file': _CALIBRATION_FILE
    }
# --------------------

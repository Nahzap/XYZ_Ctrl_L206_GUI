"""Perfiles de controlador XY (STM32 MycoViT, Arduino UNO emergencia, FPGA).

STM32 y Arduino comparten baud 1e6 y comandos M / A,a,b / B / N.
STM32: C(z) F/I/P, telemetría nativa 12-bit, stiction banco [95,150].
Arduino: sin C(z); sensores 10-bit escalados ×4 en firmware; arranque ≥110.
FPGA (Tang Nano 9K, Motor_CTRL): 115200; cuenta de encoders 0..999999;
A en % (±80); lazo de posición propio; M = los potes mandan la posición.
"""

from __future__ import annotations

import json
import logging
import os
from typing import Any

logger = logging.getLogger("MotorControl_L206")

MCU_STM32 = "STM32"
MCU_ARDUINO = "ARDUINO"
MCU_FPGA = "FPGA"

_CONFIG_DIR = os.path.dirname(os.path.abspath(__file__))
_MCU_PREFS_FILE = os.path.join(_CONFIG_DIR, "mcu_prefs.json")

MCU_PROFILES: dict[str, dict[str, Any]] = {
    MCU_STM32: {
        "label": "STM32F767ZI (MycoViT)",
        "adc_max": 4095.0,
        "stiction_pwm_min": 95,
        "stiction_pwm_max": 150,
        "use_mcu_cz_loop": True,
        "supports_cz": True,
        "baud": 1000000,
        "power_max": 255,
        "calibration_file": "calibration.json",
        "telemetry": "6-field Estado/Settled",
        "firmware_hint": "MycoViT_XY_Controller",
        "close_commands": ["A,0,0"],
    },
    MCU_ARDUINO: {
        "label": "Arduino UNO (DRV8871)",
        "adc_max": 1023.0,  # UNO 10-bit (1024 niveles: 0…1023)
        "stiction_pwm_min": 110,
        "stiction_pwm_max": 255,
        "use_mcu_cz_loop": False,
        "supports_cz": False,
        "baud": 1000000,
        "power_max": 255,
        "calibration_file": "calibration.json",
        "telemetry": "6-field 10-bit (Settled=0; F/I/P ignorados)",
        "firmware_hint": "XYZ_Ctrl_L206_v0.1",
        "close_commands": ["A,0,0"],
    },
    MCU_FPGA: {
        "label": "Tang Nano 9K FPGA (Motor_CTRL)",
        "adc_max": 999999.0,  # cuenta de 6 dígitos de la trama
        "stiction_pwm_min": 30,  # % sin medir (paso 0 del plan v3.0)
        "stiction_pwm_max": 80,  # CFG_POWER: la FPGA recorta A a ±80 %
        "use_mcu_cz_loop": False,
        "supports_cz": False,
        "baud": 115200,
        "power_max": 80,
        "calibration_file": "calibration_fpga.json",
        "telemetry": "8 campos: PotA/PotB = pedido; Sensor1 = X, Sensor2 = Y",
        "firmware_hint": "Motor_CTRL (Tang Nano 9K)",
        "counts_per_edge": 84,  # STEP de motion.v: 1 flanco del Hall; CFG_GATE = 1 flanco
        # Al cerrar la FPGA sigue como estaba: A,0,0 la dejaría en AUTO y los potes no mandarían.
        "close_commands": [],
    },
}


def list_mcu_ids() -> list[str]:
    return [MCU_STM32, MCU_ARDUINO, MCU_FPGA]


def get_profile(mcu_id: str) -> dict[str, Any]:
    key = (mcu_id or MCU_STM32).strip().upper()
    if key not in MCU_PROFILES:
        key = MCU_STM32
    return dict(MCU_PROFILES[key])


def load_saved_mcu() -> str:
    if os.path.exists(_MCU_PREFS_FILE):
        try:
            with open(_MCU_PREFS_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
            mcu = str(data.get("mcu", MCU_ARDUINO)).strip().upper()
            if mcu in MCU_PROFILES:
                return mcu
        except Exception as e:
            logger.warning("No se pudo leer mcu_prefs.json: %s", e)
    return MCU_ARDUINO  # emergencia activa por defecto


def save_mcu(mcu_id: str) -> bool:
    key = (mcu_id or MCU_ARDUINO).strip().upper()
    if key not in MCU_PROFILES:
        key = MCU_ARDUINO
    try:
        with open(_MCU_PREFS_FILE, "w", encoding="utf-8") as f:
            json.dump(
                {
                    "mcu": key,
                    "label": MCU_PROFILES[key]["label"],
                    "_comment": "Perfil activo del controlador XY. STM32, ARDUINO o FPGA.",
                },
                f,
                indent=2,
                ensure_ascii=False,
            )
        return True
    except Exception as e:
        logger.error("No se pudo guardar mcu_prefs.json: %s", e)
        return False


def apply_mcu_profile(mcu_id: str) -> dict[str, Any]:
    """Aplica el perfil a constantes globales del proceso (runtime)."""
    import config.constants as constants

    profile = get_profile(mcu_id)
    key = (mcu_id or MCU_STM32).strip().upper()
    if key not in MCU_PROFILES:
        key = MCU_STM32

    # La calibración se carga antes: su "system.adc_max" no debe pisar al del perfil.
    constants.set_calibration_file(profile["calibration_file"])

    constants.MCU_TYPE = key
    constants.ADC_MAX = float(profile["adc_max"])
    constants.STITION_PWM_MIN = int(profile["stiction_pwm_min"])
    constants.STITION_PWM_MAX = int(profile["stiction_pwm_max"])
    constants.MCU_SUPPORTS_CZ = bool(profile["supports_cz"])
    constants.MCU_USE_CZ_DEFAULT = bool(profile["use_mcu_cz_loop"])
    constants.BAUD_RATE = int(profile["baud"])
    constants.POWER_MAX = int(profile["power_max"])
    constants.FACTOR_ESCALA = constants.RECORRIDO_UM / constants.ADC_MAX

    save_mcu(key)
    logger.info(
        "MCU perfil=%s adc_max=%s stiction=[%s,%s] cz=%s baud=%s calib=%s",
        key,
        constants.ADC_MAX,
        constants.STITION_PWM_MIN,
        constants.STITION_PWM_MAX,
        constants.MCU_SUPPORTS_CZ,
        constants.BAUD_RATE,
        profile["calibration_file"],
    )
    return profile

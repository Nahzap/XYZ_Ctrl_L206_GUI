"""Protocolo de comunicación con controlador XY (STM32F767ZI / Arduino / FPGA).

Comandos vivos:
  M | B | A,<a>,<b> | P,<axis>,<sign>,<idx> | F,<rx>,<ry>[,gate] | I,<ix>,<iy> | N
Estados telemetría: MANUAL|AUTO|BRAKE|PULSE|FINE|HOLD|SETTLED
FPGA Motor_CTRL: RESET|MANUAL|AUTO|BRAKE|PULSE|PC
FPGA marcas: x_zero | y_zero | x_final | y_final | reset
"""
import logging
from typing import Optional

logger = logging.getLogger(__name__)

FPGA_STATES = ("RESET", "MANUAL", "AUTO", "BRAKE", "PULSE", "PC")
# Marcas de calibración de la FPGA, una línea cada una, en el orden del COM.
FPGA_ZERO = ("x_zero", "y_zero")
FPGA_FINAL = ("x_final", "y_final")
FPGA_RESET = ("reset",)


def _is_fpga_frame(parts) -> bool:
    """FPGA: 8 campos, cuentas en 5 y 6, estado en 7 (Lab 206 lleva el estado en 5)."""
    return (
        len(parts) >= 8
        and parts[4].strip().isdigit()
        and parts[5].strip().isdigit()
        and parts[6].strip().isalpha()
    )


class MotorProtocol:
    """Protocolo de comandos para control de motores L206."""

    @staticmethod
    def format_manual_mode():
        return "M"

    @staticmethod
    def format_power_command(motor_a_power, motor_b_power):
        return f"A,{motor_a_power},{motor_b_power}"

    @staticmethod
    def format_brake_command():
        return "B"

    @staticmethod
    def format_atom_pulse(axis: str, sign: int, idx: int) -> str:
        """Fallback fine host→MCU: P,<axis>,<sign>,<idx> (si use_mcu_cz_loop=False)."""
        ax = str(axis).strip().upper()
        if ax in ("X", "0"):
            ax = "A"
        elif ax in ("Y", "1"):
            ax = "B"
        s = 1 if int(sign) >= 0 else -1
        return f"P,{ax},{s},{int(idx)}"

    @staticmethod
    def format_cz_fine(
        ref_x_adc: int, ref_y_adc: int, gate_adc: Optional[int] = None
    ) -> str:
        """Fine canónico: F,<ref_x_adc>,<ref_y_adc>[,gate_adc]."""
        rx = max(0, min(4095, int(ref_x_adc)))
        ry = max(0, min(4095, int(ref_y_adc)))
        if gate_adc is None:
            return f"F,{rx},{ry}"
        g = max(1, min(40, int(gate_adc)))
        return f"F,{rx},{ry},{g}"

    @staticmethod
    def format_cz_invert(inv_x: bool, inv_y: bool) -> str:
        return f"I,{1 if inv_x else 0},{1 if inv_y else 0}"

    @staticmethod
    def format_cz_off() -> str:
        """Apaga C(z)/átomo sin freno (N)."""
        return "N"

    @staticmethod
    def full_halt_commands():
        """Secuencia única de parada dura: N → B → A,0,0 → M (FPGA: solo B).

        En la FPGA, M le devuelve el mando a los potes y la platina iría a
        donde estén; B frena desde cualquier modo.
        Un solo productor debe emitirla; duplicarla provoca carrera en RX MCU.
        """
        from config import constants
        from config.mcu_profiles import MCU_FPGA

        if constants.MCU_TYPE == MCU_FPGA:
            return (MotorProtocol.format_brake_command(),)
        return (
            MotorProtocol.format_cz_off(),
            MotorProtocol.format_brake_command(),
            MotorProtocol.format_power_command(0, 0),
            MotorProtocol.format_manual_mode(),
        )

    @staticmethod
    def parse_sensor_data(line):
        """Parsea línea LEGACY de 4 campos: pot_a,pot_b,sens_1,sens_2."""
        try:
            parts = line.split(",")
            if len(parts) == 4:
                return tuple(map(int, parts))
        except (ValueError, IndexError):
            logger.debug(f"Error parseando datos: {line}")
            return None
        return None

    @staticmethod
    def is_info_message(line):
        return line.startswith("INFO:") or line.startswith("ERROR:")

    @staticmethod
    def parse_sensor_data_with_status(line):
        """Telemetría Lab 206 o FPGA Motor_CTRL.

        Lab 206: pot_a,pot_b,sens_1,sens_2,estado,settled
        FPGA:    PotenciaA,PotenciaB,PotA,PotB,Sensor1,Sensor2,Estado,Settled
                 Sensor1 = X y Sensor2 = Y; se devuelven como sens_2 = X y
                 sens_1 = Y, la convención del resto del programa.
                 PotA/PotB = pedido que sigue la FPGA (cuentas).
        """
        try:
            parts = line.split(",")
            if _is_fpga_frame(parts):
                return {
                    "pot_a": int(parts[0]),
                    "pot_b": int(parts[1]),
                    "sens_1": int(parts[5]),
                    "sens_2": int(parts[4]),
                    "state": parts[6].strip(),
                    "settled": parts[7].strip() == "1",
                    "target_x": int(parts[2]),
                    "target_y": int(parts[3]),
                    "frame": "FPGA",
                }
            if len(parts) >= 6:
                state = parts[4].strip()
                if state not in (
                    "MANUAL",
                    "AUTO",
                    "BRAKE",
                    "PULSE",
                    "FINE",
                    "HOLD",
                    "SETTLED",
                    "SETTLING",
                    "LEGACY",
                ):
                    if not state.isalpha():
                        return None
                return {
                    "pot_a": int(parts[0]),
                    "pot_b": int(parts[1]),
                    "sens_1": int(parts[2]),
                    "sens_2": int(parts[3]),
                    "state": state,
                    "settled": parts[5].strip() == "1",
                }
        except (ValueError, IndexError) as e:
            logger.debug(f"Error parseando datos con estado: {line} - {e}")
            return None
        return None

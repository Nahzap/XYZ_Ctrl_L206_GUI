"""Protocolo de comunicación con controlador XY (STM32F767ZI).

Comandos vivos:
  M | B | A,<a>,<b> | P,<axis>,<sign>,<idx> | F,<rx>,<ry> | I,<ix>,<iy> | N
Estados telemetría: MANUAL|AUTO|BRAKE|PULSE|FINE
"""
import logging

logger = logging.getLogger(__name__)


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
    def format_cz_fine(ref_x_adc: int, ref_y_adc: int) -> str:
        """Fine canónico: F,<ref_x_adc>,<ref_y_adc>."""
        rx = max(0, min(4095, int(ref_x_adc)))
        ry = max(0, min(4095, int(ref_y_adc)))
        return f"F,{rx},{ry}"

    @staticmethod
    def format_cz_invert(inv_x: bool, inv_y: bool) -> str:
        return f"I,{1 if inv_x else 0},{1 if inv_y else 0}"

    @staticmethod
    def format_cz_off() -> str:
        """Apaga C(z)/átomo sin freno (N)."""
        return "N"

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
        """Telemetría: pot_a,pot_b,sens_1,sens_2,estado,settled."""
        try:
            parts = line.split(",")
            if len(parts) >= 6:
                state = parts[4].strip()
                if state not in (
                    "MANUAL",
                    "AUTO",
                    "BRAKE",
                    "PULSE",
                    "FINE",
                    "HOLD",
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

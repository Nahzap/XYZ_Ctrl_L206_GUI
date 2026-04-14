"""
Persistencia de sesión para flujo Análisis -> H∞.

Guarda y restaura:
- Slots de análisis por par motor/sensor
- Lista de funciones de transferencia identificadas
- Modelos H∞ sintetizados por slot
- Controladores transferidos a TestTab
"""

from __future__ import annotations

import json
import logging
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

logger = logging.getLogger("MotorControl_L206")


class SessionStore:
    """Gestor de persistencia para sesión de identificación y síntesis."""

    SCHEMA_VERSION = 1

    def __init__(self, session_path: Optional[str] = None):
        base_dir = Path(__file__).resolve().parents[2]  # .../src
        default_path = base_dir / "config" / "hinf_session.json"
        self.session_path = Path(session_path) if session_path else default_path
        self._session: Dict[str, Any] = self._default_session()

    @staticmethod
    def slot_key(motor: str, sensor: str) -> str:
        """Normaliza clave de slot con formato M_S, ej. A_2."""
        return f"{str(motor).upper()}_{str(sensor)}"

    def _default_session(self) -> Dict[str, Any]:
        return {
            "schema_version": self.SCHEMA_VERSION,
            "saved_at": None,
            "analysis": {
                "slots": {},
                "identified_functions": [],
            },
            "hinf": {
                "slots": {},
                "last_slot": None,
            },
            "test": {
                "controllers": {},
                "sensor_map": {},
                "invert_map": {},
            },
        }

    def load(self) -> Dict[str, Any]:
        """Carga archivo de sesión si existe; retorna estado en memoria."""
        if not self.session_path.exists():
            logger.info(f"Sesión no encontrada, se usará estado vacío: {self.session_path}")
            self._session = self._default_session()
            return deepcopy(self._session)

        try:
            with open(self.session_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if not isinstance(data, dict):
                raise ValueError("El archivo de sesión no contiene un objeto JSON válido")

            merged = self._default_session()
            self._deep_update(merged, data)
            self._session = merged
            logger.info(f"Sesión cargada: {self.session_path}")
        except Exception as e:
            logger.error(f"No se pudo cargar sesión {self.session_path}: {e}")
            self._session = self._default_session()

        return deepcopy(self._session)

    def save(self) -> bool:
        """Guarda el estado actual de sesión en disco."""
        try:
            self._session["saved_at"] = datetime.now().isoformat(timespec="seconds")
            self.session_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.session_path, "w", encoding="utf-8") as f:
                json.dump(self._session, f, indent=2, ensure_ascii=False)
            logger.info(f"Sesión guardada en: {self.session_path}")
            return True
        except Exception as e:
            logger.error(f"Error guardando sesión {self.session_path}: {e}")
            return False

    def get_session(self) -> Dict[str, Any]:
        return deepcopy(self._session)

    def set_analysis_slot(self, slot_key: str, payload: Dict[str, Any]) -> None:
        self._session["analysis"]["slots"][slot_key] = deepcopy(payload)

    def set_identified_functions(self, entries: list[Dict[str, Any]]) -> None:
        self._session["analysis"]["identified_functions"] = deepcopy(entries)

    def set_hinf_slot(self, slot_key: str, payload: Dict[str, Any]) -> None:
        self._session["hinf"]["slots"][slot_key] = deepcopy(payload)
        self._session["hinf"]["last_slot"] = slot_key

    def set_test_controller(self, motor: str, payload: Optional[Dict[str, Any]]) -> None:
        if payload is None:
            self._session["test"]["controllers"].pop(motor, None)
            return
        self._session["test"]["controllers"][motor] = deepcopy(payload)

    def set_test_preferences(self, sensor_map: Dict[str, str], invert_map: Dict[str, bool]) -> None:
        self._session["test"]["sensor_map"] = deepcopy(sensor_map)
        self._session["test"]["invert_map"] = deepcopy(invert_map)

    @staticmethod
    def _deep_update(base: Dict[str, Any], new_data: Dict[str, Any]) -> None:
        for key, value in new_data.items():
            if isinstance(value, dict) and isinstance(base.get(key), dict):
                SessionStore._deep_update(base[key], value)
            else:
                base[key] = value

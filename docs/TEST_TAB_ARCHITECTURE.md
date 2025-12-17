# Arquitectura de TestTab - Refactorización 2025-12-17

## Resumen Ejecutivo

| Métrica | Antes | Después | Reducción |
|---------|-------|---------|-----------|
| **Líneas en TestTab** | 1,699 | 842 | **-857 (-50.4%)** |
| **Métodos de UI** | 6 `_create_*` | 0 (externalizados) | -100% |
| **Lógica de control** | En TestTab | En TestService | Separado |
| **Lógica CSV** | En TestTab | En csv_utils | Separado |

---

## 1. Estructura de Archivos

```
src/
├── gui/
│   ├── tabs/
│   │   └── test_tab.py              # 842 líneas - Solo UI y handlers
│   └── utils/
│       ├── __init__.py              # Exporta todas las utilidades
│       ├── test_tab_ui_builder.py   # ~350 líneas - Builders de UI
│       ├── csv_utils.py             # ~130 líneas - Import/Export CSV
│       └── trajectory_preview.py    # ~150 líneas - Vista previa
│
├── core/
│   └── services/
│       └── test_service.py          # ~800 líneas - Lógica de control
│
└── config/
    └── constants.py                 # Calibración y constantes
```

---

## 2. Separación de Responsabilidades

### 2.1 TestTab (GUI Layer)
**Archivo:** `src/gui/tabs/test_tab.py`

**Responsabilidades:**
- Configuración de UI usando builders externos
- Mapeo de widgets a atributos
- Handlers de señales del servicio
- Actualización visual de estado
- Emisión de señales a otros módulos

**NO contiene:**
- ❌ Lógica de control PI
- ❌ Ejecución de trayectorias
- ❌ Manejo de archivos CSV
- ❌ Creación detallada de widgets

### 2.2 TestService (Business Logic Layer)
**Archivo:** `src/core/services/test_service.py`

**Responsabilidades:**
- Control dual PI con calibración dinámica
- Ejecución de trayectorias con settling
- Bloqueo inteligente de ejes
- Corrección de eje bloqueado (error > 100µm)
- Comunicación con hardware via callbacks

**Señales emitidas:**
```python
# Control Dual
dual_control_started = pyqtSignal()
dual_control_stopped = pyqtSignal()
dual_position_update = pyqtSignal(float, float, int, int)
dual_position_reached = pyqtSignal(float, float, float, float)
dual_position_lost = pyqtSignal()

# Trayectoria
trajectory_started = pyqtSignal(int)
trajectory_stopped = pyqtSignal(int, int)
trajectory_completed = pyqtSignal(int)
trajectory_point_reached = pyqtSignal(int, float, float, str)
trajectory_feedback = pyqtSignal(float, float, float, float, bool, bool, int)

# General
log_message = pyqtSignal(str)
error_occurred = pyqtSignal(str)
```

### 2.3 UI Builders (Presentation Layer)
**Archivo:** `src/gui/utils/test_tab_ui_builder.py`

**Funciones:**
```python
create_controllers_section(widgets, clear_callback) -> QGroupBox
create_motor_sensor_section(widgets) -> QGroupBox
create_calibration_section(widgets, reload_callback) -> QGroupBox
create_position_control_section(widgets, start_callback, stop_callback) -> QGroupBox
create_trajectory_section(widgets, generate_cb, preview_cb, export_cb, import_cb) -> QGroupBox
create_zigzag_section(widgets, start_callback, stop_callback) -> QGroupBox
```

**Patrón:**
- Cada función recibe un diccionario `widgets` donde almacena referencias
- Los callbacks se pasan como parámetros para conectar señales
- Retorna un QGroupBox completamente configurado

### 2.4 CSV Utilities (Data Layer)
**Archivo:** `src/gui/utils/csv_utils.py`

**Funciones:**
```python
export_trajectory_csv(trajectory: np.ndarray, filename: str) -> Tuple[bool, str]
import_trajectory_csv(filename: str) -> Tuple[bool, str, Optional[np.ndarray]]
get_trajectory_stats(trajectory: np.ndarray) -> dict
```

---

## 3. Flujo de Datos

```
┌─────────────────────────────────────────────────────────────────┐
│                         TestTab (GUI)                            │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
│  │ UI Builders │  │   Widgets   │  │  Handlers   │              │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘              │
│         │                │                │                      │
│         └────────────────┼────────────────┘                      │
│                          │                                       │
│                    ┌─────▼─────┐                                 │
│                    │  Signals  │                                 │
│                    └─────┬─────┘                                 │
└──────────────────────────┼──────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│                      TestService (Logic)                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
│  │ Dual Control│  │ Trajectory  │  │ Axis Lock   │              │
│  │     PI      │  │  Execution  │  │ Correction  │              │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘              │
│         │                │                │                      │
│         └────────────────┼────────────────┘                      │
│                          │                                       │
│                    ┌─────▼─────┐                                 │
│                    │ Callbacks │                                 │
│                    └─────┬─────┘                                 │
└──────────────────────────┼──────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│                        Hardware (Arduino)                        │
│  ┌─────────────┐  ┌─────────────┐                               │
│  │   Motors    │  │   Sensors   │                               │
│  └─────────────┘  └─────────────┘                               │
└─────────────────────────────────────────────────────────────────┘
```

---

## 4. Configuración de Hardware

### Inyección de Dependencias
```python
# En main.py o ArduinoGUI
test_tab.set_hardware_callbacks(
    send_command=self.send_command,
    get_sensor_value=self.get_sensor_value,
    get_mode_label=lambda: self.mode_label
)
```

### Configuración de Controladores
```python
# Desde HInfTab
test_tab.set_controller('A', {
    'Kp': 0.5,
    'Ki': 0.1,
    'U_max': 150,
    'gamma': 1.2
})
```

---

## 5. Ejecución de Trayectorias

### Flujo de Ejecución
1. **Generar trayectoria** → `TrajectoryGenerator.generate_zigzag_by_points()`
2. **Iniciar ejecución** → `TestService.start_trajectory()`
3. **Loop de control** → `_execute_trajectory_step()` @ 100Hz
4. **Detección de bloqueo** → `_detect_axis_lock()`
5. **Corrección de eje** → `_execute_locked_axis_correction()` (si error > 100µm)
6. **Aceptar punto** → `_accept_trajectory_point()` o `_accept_corrected_point()`
7. **Pausa** → Esperar `pause_s` segundos
8. **Siguiente punto** → Repetir desde paso 3

### Corrección de Eje Bloqueado
```
Condición: error del eje bloqueado > 100µm
Acción: 
  1. Pausar movimiento del eje activo
  2. Mover SOLO el motor del eje bloqueado
  3. Cuando error < tolerancia, aceptar punto
  4. Continuar con siguiente punto
```

---

## 6. Señales entre Módulos

### TestTab → Otros Módulos
```python
dual_control_start_requested = pyqtSignal(float, float)  # ref_a, ref_b
dual_control_stop_requested = pyqtSignal()
trajectory_generate_requested = pyqtSignal(dict)
trajectory_preview_requested = pyqtSignal()
zigzag_start_requested = pyqtSignal()
zigzag_stop_requested = pyqtSignal()
controller_clear_requested = pyqtSignal(str)  # 'A' or 'B'
trajectory_changed = pyqtSignal(int)  # n_points
```

### TestService → TestTab
```python
# Conectadas en _connect_service_signals()
test_service.dual_control_started.connect(self._on_dual_control_started)
test_service.trajectory_feedback.connect(self._on_trajectory_feedback)
test_service.log_message.connect(self._on_log_message)
# ... etc
```

---

## 7. Uso de Utilidades

### Exportar Trayectoria
```python
from gui.utils.csv_utils import export_trajectory_csv

success, message = export_trajectory_csv(trajectory, "output.csv")
```

### Importar Trayectoria
```python
from gui.utils.csv_utils import import_trajectory_csv

success, message, trajectory = import_trajectory_csv("input.csv")
```

### Vista Previa
```python
from gui.utils.trajectory_preview import show_trajectory_preview

show_trajectory_preview(parent_widget, trajectory)
```

### Crear Sección de UI
```python
from gui.utils.test_tab_ui_builder import create_controllers_section

widgets = {}
group = create_controllers_section(widgets, clear_callback)
# widgets ahora contiene referencias a todos los widgets creados
```

---

## 8. Próximos Pasos (Opcional)

Para reducir aún más las líneas de TestTab (objetivo < 600):

1. **Mover handlers de señales** a una clase separada `TestTabHandlers`
2. **Extraer métodos de actualización de estado** a `TestTabStateManager`
3. **Simplificar `_map_widgets()`** usando introspección

---

## 9. Historial de Cambios

| Fecha | Cambio | Líneas |
|-------|--------|--------|
| 2025-12-17 | Estado inicial | 1,699 |
| 2025-12-17 | Eliminar legacy control dual | 1,554 |
| 2025-12-17 | Eliminar legacy trajectory execution | 1,304 |
| 2025-12-17 | Mover preview a utilidad | 1,119 |
| 2025-12-17 | Mover UI builders a utilidad | 842 |

---

## 10. Dependencias

```python
# TestTab imports
from gui.utils.trajectory_preview import show_trajectory_preview
from gui.utils.csv_utils import export_trajectory_csv, import_trajectory_csv
from gui.utils.test_tab_ui_builder import (
    create_controllers_section,
    create_motor_sensor_section,
    create_calibration_section,
    create_position_control_section,
    create_trajectory_section,
    create_zigzag_section
)
from core.services.test_service import TestService, ControllerConfig
from config.constants import POSITION_TOLERANCE_UM, SETTLING_CYCLES
```

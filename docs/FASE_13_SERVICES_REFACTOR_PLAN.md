# FASE 13: SEPARACIÓN LÓGICA/UI EN TABS
## Plan de Refactorización de Servicios

**Documento creado:** 2025-12-12  
**Estado:** PLANIFICACIÓN  
**Objetivo:** Separar lógica de negocio de la interfaz en las tabs

---

## 📊 DIAGNÓSTICO ACTUAL

### Estado Real del Código (2025-12-12)

| Componente | Líneas | UI (%) | Lógica (%) | Estado |
|------------|--------|--------|------------|--------|
| `main.py` | 964 | 30% | 70% | 🟡 Aceptable |
| `camera_tab.py` | 1338 | 43% | **57%** | 🔴 Crítico |
| `hinf_tab.py` | 2141 | 14% | **86%** | 🔴 Crítico |
| `test_tab.py` | 1332 | 30% | **70%** | 🔴 Crítico |
| `control_tab.py` | 472 | 70% | 30% | 🟡 Aceptable |
| `analysis_tab.py` | ~400 | 60% | 40% | 🟡 Aceptable |
| `recording_tab.py` | ~150 | 80% | 20% | 🟢 Bueno |

### Discrepancia con Documentación Anterior

⚠️ **NOTA:** `REFACTOR_PLAN.md` indica main.py con 462 líneas, pero el archivo actual tiene **964 líneas**. Esto sugiere:
1. Se agregó funcionalidad nueva (microscopía, autofoco, detección)
2. La lógica migrada a tabs creció significativamente
3. Las tabs absorbieron lógica que debería estar en servicios

---

## 🎯 PROBLEMA PRINCIPAL

### Patrón Anti-Pattern Identificado: "Fat Tab"

Las tabs actuales violan el principio de responsabilidad única:

```
┌─────────────────────────────────────────────────────────┐
│                      CameraTab (1338 líneas)            │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐ │
│  │ UI Widgets  │  │ Lógica de   │  │ Procesamiento   │ │
│  │ (botones,   │  │ Cámara      │  │ de Imágenes     │ │
│  │  labels)    │  │ (conexión,  │  │ (uint16→uint8,  │ │
│  │             │  │  captura)   │  │  resize, etc.)  │ │
│  └─────────────┘  └─────────────┘  └─────────────────┘ │
│                         ↓                               │
│              Acceso directo a parent_gui                │
└─────────────────────────────────────────────────────────┘
```

### Problemas Específicos

1. **Acoplamiento Fuerte**: Tabs acceden a `self.parent_gui.*` directamente
2. **Lógica de Hardware en UI**: `connect_camera()`, `capture_image()` en tabs
3. **Procesamiento en UI**: Conversión uint16→uint8, resize en tabs
4. **Callbacks Directos**: En lugar de señales PyQt

---

## 🏗️ ARQUITECTURA OBJETIVO

### Patrón: Tab → Service → Hardware

```
┌──────────────────┐     Signals      ┌──────────────────┐
│    CameraTab     │ ←───────────────→│  CameraService   │
│  (Solo UI)       │                  │  (Lógica)        │
│  ~400 líneas     │                  │  ~500 líneas     │
└──────────────────┘                  └────────┬─────────┘
                                               │
                                               ↓
                                      ┌──────────────────┐
                                      │  CameraWorker    │
                                      │  (Hardware)      │
                                      │  ~400 líneas     │
                                      └──────────────────┘
```

---

## 📋 PLAN DE EJECUCIÓN

### FASE 13A: Crear Servicios Faltantes

| Servicio | Origen | Líneas a Mover | Prioridad |
|----------|--------|----------------|-----------|
| `CameraService` | camera_tab.py | ~500 | 🔴 Alta |
| `MicroscopyService` | camera_tab.py + main.py | ~300 | 🟡 Media |
| `HInfService` | hinf_tab.py | ~800 | 🔴 Alta |
| `TrajectoryService` | test_tab.py | ~400 | 🟡 Media |
| `DualControlService` | test_tab.py | ~300 | 🟡 Media |

### FASE 13B: Refactorizar Tabs

| Tab | Líneas Actuales | Objetivo | Reducción |
|-----|-----------------|----------|-----------|
| `camera_tab.py` | 1338 | ~400 | -938 (-70%) |
| `hinf_tab.py` | 2141 | ~400 | -1741 (-81%) |
| `test_tab.py` | 1332 | ~400 | -932 (-70%) |

### FASE 13C: Limpiar main.py

| Acción | Líneas |
|--------|--------|
| Mover microscopía a MicroscopyService | ~150 |
| Simplificar callbacks | ~50 |
| **Objetivo final** | ~700 líneas |

---

## 🔧 DETALLE POR SERVICIO

### 1. CameraService (PRIORIDAD ALTA)

**Ubicación:** `src/core/services/camera_service.py`

**Métodos a mover desde camera_tab.py:**
```python
# Conexión (~100 líneas)
- detect_thorlabs_camera()
- connect_camera()
- disconnect_camera()
- _on_camera_connected()

# Vista en vivo (~80 líneas)
- start_camera_live_view()
- stop_camera_live_view()
- on_camera_frame()

# Captura (~200 líneas)
- capture_single_image()
- _do_capture_image()
- capture_microscopy_image()

# Autofoco (~150 líneas)
- _run_autofocus()
- _focus_objects_only()
- _test_detection()
```

**Señales del servicio:**
```python
class CameraService(QObject):
    # Conexión
    connected = pyqtSignal(bool, str)  # success, info
    disconnected = pyqtSignal()
    
    # Vista en vivo
    frame_ready = pyqtSignal(object, object)  # QImage, raw
    live_started = pyqtSignal()
    live_stopped = pyqtSignal()
    
    # Captura
    image_captured = pyqtSignal(str)  # filepath
    capture_failed = pyqtSignal(str)  # error
    
    # Estado
    status_changed = pyqtSignal(str)  # message
```

---

### 2. HInfService (PRIORIDAD ALTA)

**Ubicación:** `src/core/services/hinf_service.py`

**Métodos a mover desde hinf_tab.py:**
```python
# Síntesis (~400 líneas)
- synthesize_hinf_controller()

# Simulación (~150 líneas)
- simulate_step_response()
- plot_bode()

# Control en tiempo real (~200 líneas)
- start_hinf_control()
- stop_hinf_control()
- toggle_hinf_control()
- execute_hinf_control()

# Persistencia (~100 líneas)
- export_controller()
- load_previous_controller()
```

**Señales del servicio:**
```python
class HInfService(QObject):
    # Síntesis
    synthesis_started = pyqtSignal()
    synthesis_progress = pyqtSignal(str)  # step description
    synthesis_completed = pyqtSignal(dict)  # results
    synthesis_failed = pyqtSignal(str)  # error
    
    # Simulación
    step_response_ready = pyqtSignal(object)  # Figure
    bode_ready = pyqtSignal(object)  # Figure
    
    # Control
    control_started = pyqtSignal()
    control_stopped = pyqtSignal()
    control_output = pyqtSignal(float, float)  # reference, actual
```

---

### 3. MicroscopyService (PRIORIDAD MEDIA)

**Ubicación:** `src/core/services/microscopy_service.py`

**Métodos a mover desde main.py y camera_tab.py:**
```python
# Desde main.py (~150 líneas)
- _start_microscopy()
- _stop_microscopy()
- _execute_microscopy_point()
- _on_microscopy_complete()

# Desde camera_tab.py (~100 líneas)
- _start_microscopy() (validación UI)
- set_microscopy_progress()
```

**Señales del servicio:**
```python
class MicroscopyService(QObject):
    started = pyqtSignal(dict)  # config
    point_reached = pyqtSignal(int, int)  # current, total
    image_captured = pyqtSignal(int, str)  # index, filepath
    completed = pyqtSignal(int)  # total images
    stopped = pyqtSignal()
    error = pyqtSignal(str)
```

---

### 4. TrajectoryService (PRIORIDAD MEDIA)

**Ubicación:** `src/core/services/trajectory_service.py`

**Métodos a mover desde test_tab.py:**
```python
# Generación (~150 líneas)
- generate_zigzag_trajectory()
- _preview_trajectory()

# Ejecución (~200 líneas)
- execute_trajectory()
- _execute_next_point()
- stop_trajectory()

# Importación/Exportación (~100 líneas)
- _export_trajectory_csv()
- _import_trajectory_csv()
```

---

### 5. DualControlService (PRIORIDAD MEDIA)

**Ubicación:** `src/core/services/dual_control_service.py`

**Métodos a mover desde test_tab.py:**
```python
# Control dual (~300 líneas)
- start_dual_control()
- stop_dual_control()
- execute_dual_control()
- _update_control_loop()
```

---

## 📁 ESTRUCTURA DE CARPETAS FINAL

```
src/core/services/
├── __init__.py
├── autofocus_service.py     ✅ (existe)
├── detection_service.py     ✅ (existe)
├── camera_service.py        🆕 (crear)
├── microscopy_service.py    🆕 (crear)
├── hinf_service.py          🆕 (crear)
├── trajectory_service.py    🆕 (crear)
└── dual_control_service.py  🆕 (crear)
```

---

## 📊 MÉTRICAS OBJETIVO

### Antes (Estado Actual)

| Componente | Líneas |
|------------|--------|
| main.py | 964 |
| camera_tab.py | 1338 |
| hinf_tab.py | 2141 |
| test_tab.py | 1332 |
| **TOTAL** | **5775** |

### Después (Objetivo)

| Componente | Líneas | Cambio |
|------------|--------|--------|
| main.py | ~700 | -264 |
| camera_tab.py | ~400 | -938 |
| hinf_tab.py | ~400 | -1741 |
| test_tab.py | ~400 | -932 |
| camera_service.py | ~500 | +500 |
| hinf_service.py | ~800 | +800 |
| microscopy_service.py | ~300 | +300 |
| trajectory_service.py | ~400 | +400 |
| dual_control_service.py | ~300 | +300 |
| **TOTAL** | **4200** | -1575 (-27%) |

### Beneficios

- **Tabs limpias**: Solo UI (~400 líneas cada una)
- **Servicios testeables**: Lógica aislada y testeable
- **Sin acoplamiento**: Comunicación por señales
- **Mantenibilidad**: Responsabilidades claras

---

## 🚀 ORDEN DE EJECUCIÓN RECOMENDADO

### Sprint 1: Servicios Críticos (4-6 horas)
1. ✅ Crear `CameraService` - Mover lógica de cámara
2. ✅ Refactorizar `camera_tab.py` - Solo UI

### Sprint 2: Control H∞ (4-6 horas)
3. ✅ Crear `HInfService` - Mover síntesis y control
4. ✅ Refactorizar `hinf_tab.py` - Solo UI

### Sprint 3: Trayectorias (3-4 horas)
5. ✅ Crear `TrajectoryService` + `DualControlService`
6. ✅ Refactorizar `test_tab.py` - Solo UI

### Sprint 4: Microscopía (2-3 horas)
7. ✅ Crear `MicroscopyService`
8. ✅ Limpiar `main.py`

### Sprint 5: Verificación (2 horas)
9. ✅ Pruebas de integración
10. ✅ Documentación actualizada

---

## ✅ CHECKLIST DE IMPLEMENTACIÓN

### CameraService
- [ ] Crear archivo `src/core/services/camera_service.py`
- [ ] Mover métodos de conexión desde camera_tab.py
- [ ] Mover métodos de vista en vivo
- [ ] Mover métodos de captura
- [ ] Definir señales PyQt
- [ ] Actualizar camera_tab.py para usar servicio
- [ ] Verificar funcionalidad

### HInfService
- [ ] Crear archivo `src/core/services/hinf_service.py`
- [ ] Mover synthesize_hinf_controller()
- [ ] Mover simulate_step_response() y plot_bode()
- [ ] Mover control en tiempo real
- [ ] Definir señales PyQt
- [ ] Actualizar hinf_tab.py para usar servicio
- [ ] Verificar funcionalidad

### TrajectoryService
- [ ] Crear archivo `src/core/services/trajectory_service.py`
- [ ] Mover generación de trayectorias
- [ ] Mover ejecución de trayectorias
- [ ] Definir señales PyQt
- [ ] Actualizar test_tab.py para usar servicio
- [ ] Verificar funcionalidad

### DualControlService
- [ ] Crear archivo `src/core/services/dual_control_service.py`
- [ ] Mover control dual
- [ ] Definir señales PyQt
- [ ] Actualizar test_tab.py para usar servicio
- [ ] Verificar funcionalidad

### MicroscopyService
- [ ] Crear archivo `src/core/services/microscopy_service.py`
- [ ] Mover lógica de microscopía desde main.py
- [ ] Definir señales PyQt
- [ ] Actualizar main.py y camera_tab.py
- [ ] Verificar funcionalidad

---

## 📝 NOTAS IMPORTANTES

### Patrón de Comunicación

```python
# En la Tab (solo UI):
class CameraTab(QWidget):
    def __init__(self, camera_service: CameraService):
        self.service = camera_service
        
        # Conectar señales del servicio a métodos de UI
        self.service.connected.connect(self._on_connected)
        self.service.frame_ready.connect(self._on_frame)
        
        # Conectar botones a métodos del servicio
        self.connect_btn.clicked.connect(self.service.connect)
    
    def _on_connected(self, success: bool, info: str):
        # Solo actualizar UI, sin lógica
        if success:
            self.status_label.setText(f"Conectado: {info}")
            self.connect_btn.setEnabled(False)
```

### Inyección de Dependencias

```python
# En main.py:
class ArduinoGUI(QMainWindow):
    def __init__(self):
        # Crear servicios
        self.camera_service = CameraService()
        self.hinf_service = HInfService()
        
        # Inyectar en tabs
        self.camera_tab = CameraTab(self.camera_service)
        self.hinf_tab = HInfTab(self.hinf_service)
```

---

## 🎯 CRITERIOS DE ÉXITO

1. **Tabs < 500 líneas**: Cada tab debe tener máximo 500 líneas
2. **Sin lógica de hardware en UI**: Toda lógica en servicios
3. **Comunicación por señales**: Sin acceso directo a parent_gui
4. **Servicios testeables**: Cada servicio testeable de forma aislada
5. **main.py < 800 líneas**: Solo orquestación

---

**Próximo paso:** Comenzar con Sprint 1 - Crear CameraService

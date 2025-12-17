# Auditoría del Proyecto XYZ_Ctrl_L206_GUI
## Fecha: 2025-12-17 17:15 (Actualizado)

---

## 1. Resumen Ejecutivo

| Componente | Líneas | Estado | Notas |
|------------|--------|--------|-------|
| **main.py** | 781 | ✅ Refactorizado | Reducido desde ~3500 |
| **test_tab.py** | 843 | ✅ Refactorizado | Reducido desde 1699 (-50%) |
| **test_service.py** | 823 | ✅ Nuevo | Lógica de control extraída |
| **camera_tab.py** | ~855 | ✅ Refactorizado | Reducido desde 1472 (-42%) |
| **camera_service.py** | 490 | ✅ Expandido | Lógica de cámara centralizada |
| **camera_tab_ui_builder.py** | 578 | ✅ Nuevo | Builders de UI |
| **hinf_tab.py** | 614 | ✅ OK | Dentro de límites |
| **hinf_service.py** | 723 | ✅ OK | Servicio separado |

---

## 2. Arquitectura Actual

```
XYZ_Ctrl_L206_GUI/
├── src/
│   ├── main.py                     # 781 líneas - Punto de entrada
│   │
│   ├── config/
│   │   ├── constants.py            # 222 líneas - Calibración y constantes
│   │   └── calibration.json        # Datos de calibración
│   │
│   ├── core/
│   │   ├── controllers/
│   │   │   └── hinf_controller.py  # 1171 líneas - Síntesis H∞
│   │   │
│   │   └── services/
│   │       ├── test_service.py     # 823 líneas - Control dual y trayectorias
│   │       ├── hinf_service.py     # 723 líneas - Servicio H∞
│   │       ├── microscopy_service.py # 789 líneas - Microscopía
│   │       ├── autofocus_service.py  # 468 líneas - Autofocus
│   │       └── detection_service.py  # Detección de objetos
│   │
│   ├── gui/
│   │   ├── tabs/
│   │   │   ├── test_tab.py         # 843 líneas ✅ REFACTORIZADO
│   │   │   ├── camera_tab.py       # ~855 líneas ✅ REFACTORIZADO
│   │   │   ├── hinf_tab.py         # 614 líneas
│   │   │   ├── control_tab.py      # 521 líneas
│   │   │   ├── analysis_tab.py     # 348 líneas
│   │   │   └── img_analysis_tab.py # 408 líneas
│   │   │
│   │   ├── utils/
│   │   │   ├── __init__.py         # Exportaciones
│   │   │   ├── test_tab_ui_builder.py # 420 líneas - Builders UI TestTab
│   │   │   ├── camera_tab_ui_builder.py # 578 líneas - Builders UI CameraTab
│   │   │   ├── csv_utils.py        # ~130 líneas - Import/Export CSV
│   │   │   └── trajectory_preview.py # ~150 líneas - Vista previa
│   │   │
│   │   └── windows/
│   │       └── camera_window.py    # 538 líneas
│   │
│   ├── trajectory/
│   │   └── trajectory_generator.py # 362 líneas
│   │
│   └── vision/
│       ├── smart_focus_scorer.py   # 818 líneas
│       ├── sharpness_detector.py   # 567 líneas
│       ├── u2net_detector.py       # 463 líneas
│       └── ...
│
└── docs/
    ├── PROJECT_AUDIT_2025-12-17.md # Este archivo
    └── TEST_TAB_ARCHITECTURE.md    # Documentación TestTab
```

---

## 3. Refactorización de TestTab (Completada)

### 3.1 Métricas

| Métrica | Antes | Después | Cambio |
|---------|-------|---------|--------|
| Líneas en TestTab | 1,699 | 843 | **-856 (-50.4%)** |
| Métodos de UI | 6 `_create_*` | 0 | Externalizados |
| Lógica de control | En TestTab | En TestService | Separado |
| Lógica CSV | En TestTab | En csv_utils | Separado |

### 3.2 Archivos Creados

1. **`src/core/services/test_service.py`** (823 líneas)
   - Control dual PI con calibración dinámica
   - Ejecución de trayectorias con settling
   - Bloqueo inteligente de ejes
   - Corrección de eje bloqueado (error > 100µm)
   - Señales PyQt para comunicación

2. **`src/gui/utils/test_tab_ui_builder.py`** (420 líneas)
   - `create_controllers_section()`
   - `create_motor_sensor_section()`
   - `create_calibration_section()`
   - `create_position_control_section()`
   - `create_trajectory_section()`
   - `create_zigzag_section()`

3. **`src/gui/utils/csv_utils.py`** (~130 líneas)
   - `export_trajectory_csv()`
   - `import_trajectory_csv()`
   - `get_trajectory_stats()`

4. **`src/gui/utils/trajectory_preview.py`** (~150 líneas)
   - `show_trajectory_preview()`

### 3.3 Funcionalidades de TestTab

| Funcionalidad | Estado | Ubicación |
|---------------|--------|-----------|
| Controladores H∞ transferidos | ✅ | TestTab + UI Builder |
| Asignación Motor-Sensor | ✅ | UI Builder (CheckBoxes) |
| Inversión de PWM | ✅ | UI Builder |
| Calibración dinámica | ✅ | TestTab + constants.py |
| Control dual PI | ✅ | TestService |
| Generación de trayectorias | ✅ | TrajectoryGenerator |
| Vista previa de trayectoria | ✅ | trajectory_preview.py |
| Import/Export CSV | ✅ | csv_utils.py |
| Ejecución de trayectoria | ✅ | TestService |
| Bloqueo inteligente de ejes | ✅ | TestService |
| Corrección de eje bloqueado | ✅ | TestService |
| Feedback visual en tiempo real | ✅ | TestTab (handlers) |

---

## 4. Próxima Refactorización: CameraTab

### 4.1 Estado Actual
- **Líneas:** 1,471
- **Objetivo:** < 600 líneas
- **Reducción esperada:** ~60%

### 4.2 Análisis Preliminar

Funcionalidades a extraer:
1. **Lógica de cámara** → `CameraService`
2. **Procesamiento de imagen** → Ya en `MicroscopyService`
3. **Autofocus** → Ya en `AutofocusService`
4. **UI builders** → `camera_tab_ui_builder.py`
5. **Captura de imágenes** → Utilidad separada

### 4.3 Servicios Existentes Relacionados

| Servicio | Líneas | Funcionalidad |
|----------|--------|---------------|
| microscopy_service.py | 789 | Procesamiento de microscopía |
| autofocus_service.py | 468 | Algoritmos de autofocus |
| detection_service.py | ~200 | Detección de objetos |
| camera_worker.py | 386 | Worker de cámara |

---

## 5. Principios de Arquitectura

### 5.1 Separación de Responsabilidades

```
┌─────────────────────────────────────────────────────────────┐
│                      GUI Layer (Tabs)                        │
│  - Solo UI y handlers de señales                            │
│  - NO lógica de negocio                                     │
│  - Actualización visual de estado                           │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼ Señales PyQt
┌─────────────────────────────────────────────────────────────┐
│                    Service Layer                             │
│  - Lógica de negocio                                        │
│  - Control de hardware                                       │
│  - Procesamiento de datos                                    │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼ Callbacks
┌─────────────────────────────────────────────────────────────┐
│                    Hardware Layer                            │
│  - Arduino (motores, sensores)                              │
│  - Cámara                                                   │
└─────────────────────────────────────────────────────────────┘
```

### 5.2 Comunicación

- **GUI → Service:** Llamadas directas a métodos
- **Service → GUI:** Señales PyQt
- **Service → Hardware:** Callbacks inyectados

### 5.3 Límites de Líneas

| Tipo de archivo | Límite recomendado |
|-----------------|-------------------|
| Tab (GUI) | < 600 líneas |
| Service | < 800 líneas |
| Controller | < 1000 líneas |
| Utility | < 300 líneas |

---

## 6. Archivos por Tamaño (> 200 líneas)

| Archivo | Líneas | Prioridad |
|---------|--------|-----------|
| hinf_controller.py | 1171 | Media |
| camera_tab.py | ~855 | ✅ Completado |
| test_tab.py | 843 | ✅ Completado |
| test_service.py | 823 | ✅ Nuevo |
| smart_focus_scorer.py | 818 | Baja |
| microscopy_service.py | 789 | Baja |
| main.py | 781 | ✅ Completado |
| hinf_service.py | 723 | Baja |
| hinf_tab.py | 614 | OK |
| camera_tab_ui_builder.py | 578 | ✅ Nuevo |
| camera_service.py | 490 | ✅ Expandido |

---

## 7. Historial de Cambios

| Fecha | Cambio | Impacto |
|-------|--------|---------|
| 2025-12-17 | Crear TestService | +823 líneas (nuevo) |
| 2025-12-17 | Refactorizar TestTab | -856 líneas (-50%) |
| 2025-12-17 | Crear test_tab_ui_builder | +420 líneas (nuevo) |
| 2025-12-17 | Crear csv_utils | +130 líneas (nuevo) |
| 2025-12-17 | Crear trajectory_preview | +150 líneas (nuevo) |
| 2025-12-17 | Corrección eje bloqueado | Funcionalidad nueva |
| 2025-12-17 | Refactorizar CameraTab | -617 líneas (-42%) |
| 2025-12-17 | Expandir CameraService | +342 líneas |
| 2025-12-17 | Crear camera_tab_ui_builder | +578 líneas (nuevo) |
| 2025-12-17 | Consolidar backups | Organización |

---

## 8. Próximos Pasos

1. **[COMPLETADO]** ~~Refactorizar `camera_tab.py`~~ ✅
2. **[MEDIO]** Revisar `hinf_controller.py` (1171 líneas)
3. **[BAJO]** Optimizar servicios de visión

---

## 9. Refactorización de CameraTab (Completada)

### 9.1 Métricas

| Métrica | Antes | Después | Cambio |
|---------|-------|---------|--------|
| Líneas en CameraTab | 1,472 | ~855 | **-617 (-42%)** |
| Métodos de UI | 7 secciones inline | 0 | Externalizados |
| Lógica de cámara | En CameraTab | En CameraService | Separado |

### 9.2 Archivos Creados/Modificados

1. **`src/gui/utils/camera_tab_ui_builder.py`** (578 líneas)
   - `create_connection_section()`
   - `create_live_view_section()`
   - `create_config_section()`
   - `create_capture_section()`
   - `create_microscopy_section()`
   - `create_autofocus_section()`
   - `create_log_section()`

2. **`src/core/services/camera_service.py`** (490 líneas, expandido desde 148)
   - `detect_cameras()` - Detección de cámaras Thorlabs
   - `connect_camera()` / `disconnect_camera()`
   - `start_live()` / `stop_live()`
   - `apply_exposure()` / `apply_fps()` / `apply_buffer()`
   - `capture_image()` / `capture_microscopy_image()`
   - Propiedades: `is_connected`, `current_frame`

### 9.3 Arquitectura CameraTab

```
┌─────────────────────────────────────────────────────────────────┐
│                    camera_tab.py (~855 líneas)                  │
│  - _setup_ui() usando builders externos                         │
│  - Handlers de UI (_on_*_clicked)                               │
│  - Callbacks de servicios (on_*)                                │
│  - Métodos de actualización UI (set_*)                          │
│  - _run_autofocus() con detección + Z-scan                      │
└───────────────┬─────────────────────────────────────────────────┘
                │ signals/slots
                ▼
┌─────────────────────────────────────────────────────────────────┐
│                 camera_service.py (490 líneas)                  │
│  - Lógica de conexión y configuración de cámara                 │
│  - Captura de imágenes (simple y microscopía)                   │
│  - Señales: connected, frame_ready, capture_completed           │
└───────────────┬─────────────────────────────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────────────────────────────┐
│              camera_tab_ui_builder.py (578 líneas)              │
│  - 7 funciones create_*_section()                               │
│  - Widgets almacenados en diccionario para mapeo                │
└─────────────────────────────────────────────────────────────────┘
```

---

## 10. Notas Técnicas

### 10.1 Corrección de Eje Bloqueado
- **Trigger:** Error del eje bloqueado > 100µm
- **Acción:** Mover SOLO el motor del eje bloqueado
- **Condición de éxito:** Error < tolerancia (25µm)
- **Momento:** Después de que el eje móvil llegue al objetivo, antes de aceptar el punto

### 9.2 Calibración Dinámica
- Archivo: `config/calibration.json`
- Cargada en: `config/constants.py`
- Recarga: Botón en TestTab o `reload_calibration()`

### 9.3 Asignación Motor-Sensor
- Motor A puede leer Sensor 1 o Sensor 2
- Motor B puede leer Sensor 1 o Sensor 2
- Selección independiente para cada motor
- Inversión de PWM configurable por motor

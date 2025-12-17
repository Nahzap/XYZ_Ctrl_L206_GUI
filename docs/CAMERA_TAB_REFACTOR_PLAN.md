# Refactorización de CameraTab - COMPLETADO

**Fecha:** 2025-12-17
**Estado:** ✅ COMPLETADO

## Estado Anterior (1472 líneas)

### Análisis de Responsabilidades Mezcladas

#### 1. LÓGICA DE CÁMARA (→ CameraService) ~350 líneas
- `detect_thorlabs_camera()` - Detección de cámaras
- `connect_camera()` - Conexión con cámara
- `disconnect_camera()` - Desconexión
- `start_camera_live_view()` - Iniciar vista en vivo
- `stop_camera_live_view()` - Detener vista en vivo
- `_apply_exposure()` - Aplicar exposición
- `_apply_fps()` - Aplicar FPS
- `_apply_buffer()` - Aplicar buffer
- `capture_single_image()` - Captura simple
- `_do_capture_image()` - Lógica de captura
- `capture_microscopy_image()` - Captura para microscopía
- `on_camera_frame()` - Callback de frame

#### 2. LÓGICA DE AUTOFOCO (→ ya en AutofocusService)
- `_run_autofocus()` - Ejecutar autofoco
- `_focus_objects_only()` - Solo enfocar
- `_update_detection_params()` - Actualizar parámetros
- `_test_detection()` - Test de detección
- `_connect_cfocus()` - Conectar C-Focus
- `_disconnect_cfocus()` - Desconectar C-Focus

#### 3. LÓGICA DE MICROSCOPÍA (→ ya en MicroscopyService)
- `_start_microscopy()` - Iniciar microscopía
- `_stop_microscopy()` - Detener microscopía
- `_update_storage_estimate()` - Calcular almacenamiento

#### 4. UI BUILDERS (→ camera_tab_ui_builder.py) ~450 líneas
- `_create_connection_section()`
- `_create_live_view_section()`
- `_create_config_section()`
- `_create_capture_section()`
- Sección de microscopía inline en `_setup_ui()`
- Sección de autofoco inline en `_setup_ui()`
- Sección de log inline en `_setup_ui()`

#### 5. HANDLERS DE UI (permanecen en CameraTab) ~200 líneas
- `set_connected()` - Actualizar UI de conexión
- `set_trajectory_status()` - Actualizar estado trayectoria
- `set_microscopy_progress()` - Actualizar progreso
- `log_message()` - Escribir en log
- `_browse_folder()` - Diálogo de carpeta
- `_browse_microscopy_folder()` - Diálogo carpeta microscopía
- Callbacks de detección/autofoco (`on_detection_*`, `on_autofocus_*`)

---

## Arquitectura Objetivo

```
┌─────────────────────────────────────────────────────────────────┐
│                         CameraTab                                │
│  (Solo UI: señales, slots, actualización de widgets)            │
│  ~400 líneas                                                     │
├─────────────────────────────────────────────────────────────────┤
│  - _setup_ui() usando builders externos                         │
│  - _map_widgets() para acceso a widgets                         │
│  - Handlers de UI (set_*, log_message, browse_*)                │
│  - Callbacks de servicios (on_*)                                │
└───────────────┬─────────────────────────────────────────────────┘
                │ signals/slots
                ▼
┌─────────────────────────────────────────────────────────────────┐
│                      CameraService                               │
│  (Lógica de cámara: conexión, captura, configuración)           │
│  ~300 líneas                                                     │
├─────────────────────────────────────────────────────────────────┤
│  Signals:                                                        │
│  - connection_changed(bool, str)                                │
│  - frame_ready(QImage, ndarray)                                 │
│  - capture_completed(str)                                       │
│  - status_update(str)                                           │
│  - error_occurred(str)                                          │
│                                                                  │
│  Métodos:                                                        │
│  - detect_cameras()                                             │
│  - connect_camera(buffer_size)                                  │
│  - disconnect_camera()                                          │
│  - start_live(exposure, fps, buffer)                            │
│  - stop_live()                                                  │
│  - apply_exposure(value)                                        │
│  - apply_fps(value)                                             │
│  - apply_buffer(value)                                          │
│  - capture_image(folder, format)                                │
│  - capture_microscopy_image(config, index)                      │
└───────────────┬─────────────────────────────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────────────────────────────┐
│                      CameraWorker                                │
│  (Hardware: comunicación directa con Thorlabs SDK)              │
│  Ya existe en hardware/camera/camera_worker.py                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Archivos a Crear/Modificar

### 1. CREAR: `src/core/services/camera_service.py`
Mover toda la lógica de cámara desde CameraTab.

### 2. YA CREADO: `src/gui/utils/camera_tab_ui_builder.py`
Builders de UI para todas las secciones.

### 3. MODIFICAR: `src/gui/tabs/camera_tab.py`
- Eliminar métodos de lógica
- Usar builders externos
- Conectar con CameraService via señales

---

## Orden de Implementación

1. ✅ Crear `camera_tab_ui_builder.py` (578 líneas)
2. ✅ Expandir `camera_service.py` con toda la lógica (490 líneas)
3. ✅ Refactorizar `camera_tab.py` para usar builders y servicio (~855 líneas)
4. ✅ CameraService ya estaba instanciado en main.py
5. ✅ Verificar funcionamiento completo
6. ✅ Corregir métodos faltantes para compatibilidad con main.py
7. ✅ Restaurar lógica completa de autofoco

---

## Resultado Final

### Conteo de Líneas
| Archivo | Antes | Después | Cambio |
|---------|-------|---------|--------|
| camera_tab.py | 1472 | ~855 | -42% |
| camera_service.py | 148 | 490 | +231% (expandido) |
| camera_tab_ui_builder.py | 0 | 578 | nuevo |

### Separación de Responsabilidades

```
┌─────────────────────────────────────────────────────────────────┐
│                    camera_tab.py (771 líneas)                   │
│  - _setup_ui() usando builders externos                         │
│  - _map_widgets() para acceso a widgets                         │
│  - Handlers de UI (_on_*_clicked)                               │
│  - Callbacks de servicios (on_*)                                │
│  - Métodos de actualización UI (set_*)                          │
└───────────────┬─────────────────────────────────────────────────┘
                │ signals/slots
                ▼
┌─────────────────────────────────────────────────────────────────┐
│                 camera_service.py (490 líneas)                  │
│  - detect_cameras()                                             │
│  - connect_camera() / disconnect_camera()                       │
│  - start_live() / stop_live()                                   │
│  - apply_exposure() / apply_fps() / apply_buffer()              │
│  - capture_image() / capture_microscopy_image()                 │
│  - Propiedades: is_connected, current_frame                     │
└───────────────┬─────────────────────────────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────────────────────────────┐
│              camera_tab_ui_builder.py (578 líneas)              │
│  - create_connection_section()                                  │
│  - create_live_view_section()                                   │
│  - create_config_section()                                      │
│  - create_capture_section()                                     │
│  - create_microscopy_section()                                  │
│  - create_autofocus_section()                                   │
│  - create_log_section()                                         │
└─────────────────────────────────────────────────────────────────┘
```

### Beneficios
1. **Separación clara**: UI, lógica y builders en archivos separados
2. **Testabilidad**: CameraService puede testearse independientemente
3. **Mantenibilidad**: Cambios en UI no afectan lógica y viceversa
4. **Reutilización**: CameraService puede usarse desde otros módulos

# FASE 14: Sistema de Microscopia Inteligente
## Integración de Cámara, Trayectorias, C-Focus y Autofoco

**Documento creado:** 2025-12-15  
**Estado:** EN PROGRESO (C-Focus y Autofocus OK; pendiente modularizar microscopía completa)  
**Objetivo:** Reorganizar TODO el sistema de microscopía (trayectorias, captura, C-Focus, autofoco, detección) para que la lógica viva en servicios dedicados y tabs, manteniendo `main.py` solo como orquestador de señales.

---

## 📊 Diagnóstico Actual

### Componentes involucrados

- `main.py`
  - Métodos: `connect_cfocus()`, `disconnect_cfocus()`, `initialize_autofocus()`, `_microscopy_capture_with_autofocus()`.
  - Crea servicios: `DetectionService`, `CameraService`, `AutofocusService`.
- `CameraTab`
  - UI para botones: **Conectar C-Focus**, **Enfocar Objs**, **Autofoco Multi-Objeto**.
  - Llama a `parent_gui.connect_cfocus()` y `parent_gui.initialize_autofocus()`.
- `AutofocusService`
  - Servicio asíncrono (QThread) que realiza Z-scanning/hill-climbing.
  - Requiere `cfocus_controller` y `get_frame_callback`.
- `CameraService` + `CameraWorker`
  - Fuente oficial de frames de cámara.
  - Aplica exposición/FPS/buffer directamente sobre el SDK de Thorlabs.

### Problema observado (inicial)

- Log `motor_control_20251215.log` indicaba:
  - `Error C-Focus: Error: No se pudo inicializar handle (dispositivo no conectado o en uso)`.
- La ruta de integración (Tabs/Servicios) era correcta, pero:
  - `initialize_autofocus()` dependía directamente de `camera_tab.camera_worker`.
  - Errores de `AutofocusService` no se mostraban en `CameraTab`.

### Estado actual

- C-Focus ya se conecta y funciona correctamente desde `CameraTab`.
- Autofoco multi-objeto (vía `AutofocusService`) está operativo usando frames de `CameraService.worker`.
- Los errores del servicio de autofoco se reportan en el log de `CameraTab`.
- Los parámetros de cámara (exposición, FPS, buffer) se aplican ahora de forma coherente sobre el SDK de Thorlabs y se usan tanto para la vista en vivo como para la captura y la microscopía.

---

## 🏗️ Arquitectura Objetivo

Patrón general:

```
CameraTab (UI)
    ├─ botones C-Focus / Autofoco
    │   ↓ señales / callbacks
main.py (orquestador)
    ├─ CameraService  → CameraWorker (hardware cámara)
    └─ AutofocusService → CFocusController (hardware Z)
```

- `CameraTab`:
  - Solo crea UI y llama a métodos del `ArduinoGUI` (orquestador).
  - Muestra logs/estados en la interfaz.
- `ArduinoGUI` (`main.py`):
  - Orquesta servicios (`CameraService`, `AutofocusService`).
  - Inyecta callbacks y controladores hardware.
- `AutofocusService`:
  - Usa `cfocus_controller` + `get_frame_callback` para leer frames desde `CameraService.worker`.
  - Emite señales de progreso y errores que se reflejan en `CameraTab`.

---

## 📋 Plan de Ejecución (FASE 14)

1. **Desacoplar Autofocus de CameraTab**
   - [x] Hacer que `initialize_autofocus()` use preferentemente `camera_service.worker` como fuente de frames.
   - [x] Mantener fallback a `camera_tab.camera_worker` por compatibilidad.

2. **Propagar errores de Autofocus a la UI**
   - [x] Conectar `AutofocusService.error_occurred` a `CameraTab.log_message`.
   - [ ] Ajustar mensajes de UI si es necesario (traducciones, emojis, etc.).

3. **Validación funcional**
   - [x] Caso 1: Cámara conectada, C-Focus desconectado → botón "Enfocar" muestra mensaje claro y no rompe servicios.
   - [x] Caso 2: Cámara + C-Focus conectados → `_run_autofocus()` inicia escaneo y completa sin errores (flujo base probado).
   - [ ] Caso 3: Microscopía con autofoco (`_microscopy_capture_with_autofocus`) debe capturar y avanzar puntos correctamente.

4. **Próximas fases (opcional)**
   - [ ] Evaluar creación de `CFocusService` dedicado (similar a `CameraService`).
   - [ ] Mover lógica de `_microscopy_capture_with_autofocus()` a `MicroscopyService`.

---

## ✅ Checklist de Implementación (FASE 14)

- [x] Actualizar `initialize_autofocus()` para usar `CameraService.worker` como fuente de frames.
- [x] Mantener compatibilidad con `camera_tab.camera_worker` (alias).
- [x] Conectar `AutofocusService.error_occurred` → `CameraTab.log_message`.
- [x] Probar manualmente flujo de conexión C-Focus + Autofoco desde CameraTab (C-Focus operativo).
- [x] Ajustar documentación de usuario si cambian los mensajes de error.

### ✅ Integración con SDK de Thorlabs (Cámara)

- El SDK de Thorlabs se configura de forma centralizada en `main.py`:
  - `pll.par["devices/dlls/thorlabs_tlcam"] = r"C:\\Program Files\\Thorlabs\\ThorImageCAM\\Bin"`
  - Se importa `Thorlabs` y se define `THORLABS_AVAILABLE` como bandera global.
- `CameraTab` recibe `thorlabs_available` desde `main.py` y **no** vuelve a detectar el SDK por su cuenta.
  - Esto evita estados inconsistentes entre módulos.
  - `CameraTab` propaga esta bandera a `CameraService.set_thorlabs_available()`.
- `CameraService`:
  - Crea un `CameraWorker` cuando se conecta la cámara.
  - Propaga mensajes de estado y frames (`frame_ready`) hacia `CameraTab`.
- `CameraWorker`:
  - Usa el SDK de Thorlabs (`ThorlabsTLCamera`) para aplicar:
    - `set_exposure(exposure_s)`
    - `set_trigger_mode("int")` (trigger interno por ahora)
    - `set_frame_period(1/fps)` para fijar el frame rate
    - `setup_acquisition(nframes=buffer_size)` para el tamaño de buffer
  - En la vista en vivo convierte los frames uint16 → uint8 con
    `frame_uint8 = (frame / frame.max() * 255).astype(np.uint8)` antes de construir el `QImage`.
- Desde la UI (`CameraTab`):
  - `_apply_exposure()` llama a `CameraWorker.change_exposure()` (cuando hay worker), que a su vez hace `cam.set_exposure(...)`.
  - `_apply_fps()` ahora llama a `CameraWorker.change_fps()`, que recalcula y aplica el `frame_period` en la cámara.
  - `_apply_buffer()` actualiza `CameraWorker.buffer_size`; el nuevo valor se aplica en la **próxima** llamada a `start_live_view()` (comportamiento esperado del SDK).

### 🎨 Coherencia imagen observada vs imagen guardada

Se detectó que la imagen mostrada en la ventana de cámara y la imagen guardada en disco podían verse diferentes, aun con los mismos parámetros de cámara.

- **Causa:**
  - La vista en vivo (`CameraWorker`) normalizaba uint16 → uint8 con
    `frame_uint8 = (frame / frame.max() * 255).astype(np.uint8)`.
  - `_do_capture_image()` en `CameraTab` usaba `cv2.normalize` con min/max, lo que cambiaba el contraste global.
- **Corrección:**
  - `_do_capture_image()` ahora utiliza exactamente la misma estrategia de normalización que `CameraWorker` para PNG/JPG:
    - Si `frame_max > 0`: `frame_norm = (frame / frame_max * 255).astype(np.uint8)`.
    - Si `frame_max == 0`: frame negro.
  - Para TIFF se mantiene el frame uint16 original sin normalizar.

Resultado: **la imagen guardada coincide visualmente con la imagen observada** (a igualdad de parámetros de cámara), salvo por el posible reescalado espacial o de canales definido por la configuración de microscopía.

---

## 🧠 Siguiente paso: Modularización de Microscopía Inteligente

Ahora que la integración C-Focus / Autofocus / CameraService está estable, el siguiente objetivo es **sacar la lógica inteligente de microscopía de `main.py` y `CameraTab`**.

### 1. Componentes con lógica de microscopía

- `main.py`
  - `_microscopy_capture_with_autofocus()`
  - `_advance_microscopy_point()`
  - `_microscopy_move_to_point()`
  - `_start_microscopy()` / `_stop_microscopy()` (o equivalentes según versión actual).
- `CameraTab`
  - `_start_microscopy()` / `_stop_microscopy()` (disparo desde UI, más parte de la lógica de validación).

Actualmente, `main.py` hace más que orquestar: contiene reglas de negocio de cómo capturar en cada punto, cómo avanzar la trayectoria y cuándo llamar al autofoco.

### 2. Objetivo arquitectónico

- Crear un `MicroscopyService(QObject)` en `src/core/services/microscopy_service.py` que:
  - Coordine:
    - `CameraService` (frames y capturas).
    - `AutofocusService` (enfoque multi-objeto).
    - La trayectoria de microscopía (puntos a visitar).
  - Exponga señales de alto nivel a la UI y a `main.py`, por ejemplo:
    - `status_changed(str)` → para logs.
    - `progress_changed(current:int, total:int)` → para barra de progreso.
    - `capture_done(point_idx:int, filepath:str)`.
    - `microscopy_finished()` / `microscopy_cancelled()`.
- `CameraTab` quedará solo con:
  - Lectura de parámetros desde la UI.
  - Botones que emiten señales ("iniciar microscopía", "detener", etc.).
  - Actualización de log y progreso a partir de las señales del servicio.
- `main.py` se limitará a:
  - Instanciar `MicroscopyService`.
  - Conectar sus señales a `CameraTab` y otros módulos si es necesario.

### 3. Plan preliminar para MicroscopyService

1. **Diseño detallado en docs**
   - Enumerar exactamente qué métodos se moverán de `main.py` y `CameraTab`.
   - Definir API pública del servicio (métodos y señales).
2. **Implementación del servicio**
   - Crear `MicroscopyService` en `src/core/services`.
   - Inyectar dependencias: `camera_service`, `autofocus_service`, trayectoria.
3. **Refactor progresivo**
   - Mover `_microscopy_capture_with_autofocus()` al servicio manteniendo la firma lógica.
   - Mover `_advance_microscopy_point()` y `_microscopy_move_to_point()`.
   - Reducir `main.py` a simples conexiones de señales.
4. **Validación**
   - Probar microscopía sin autofoco.
   - Probar microscopía con autofoco multi-objeto (usando `AutofocusService`). **(COMPLETADO)**
     - `MicroscopyService._capture_with_autofocus()`:
       - Obtiene el frame actual desde la cámara (`get_current_frame`).
       - Ejecuta detección de objetos con `SmartFocusScorer.assess_image(...)`.
       - Filtra por área `[min_pixels, max_pixels]` proveniente de `CameraTab`.
       - Llama a `AutofocusService.start_autofocus(objects)`.
     - `AutofocusService` realiza el hill-climbing en Z usando `CFocusController` y, al completar:
       - Emite `scan_complete(results)`.
       - `ArduinoGUI._on_autofocus_complete()` delega en `MicroscopyService.handle_autofocus_complete()` cuando hay microscopia activa.
     - `MicroscopyService.handle_autofocus_complete()` captura la imagen en la mejor posición de foco vía `CameraTab.capture_microscopy_image(...)` y avanza al siguiente punto de la trayectoria.

De esta forma, **cada trigger lógico de trayectoria** (posición alcanzada en X/Y) provoca:

1. Delay de estabilización (`delay_before`).
2. Detección + autofoco Z (si está habilitado y C-Focus conectado).
3. Captura de imagen en el BPoF.
4. Avance al siguiente punto (`delay_after`), repitiendo el ciclo.

Este plan se desarrollará con más detalle en la siguiente fase de documentación (o ampliando esta FASE 14) antes de tocar código, para respetar el principio de que `main.py` solo orquesta y la lógica vive en servicios dedicados.

---

## 🧩 Mapa de funciones de `ArduinoGUI` y responsable lógico

Esta tabla resume **todas las funciones actuales de `ArduinoGUI`** en `main.py` y el **módulo/clase responsable** donde debería vivir su lógica en la arquitectura modular.

| Método `ArduinoGUI`                         | Tipo actual                         | Responsable lógico futuro                                 | Notas |
|--------------------------------------------|-------------------------------------|-----------------------------------------------------------|-------|
| `__init__`                                 | Composición + wiring                | `ArduinoGUI` (orquestador)                               | Mantener, pero reduciendo lógica de microscopía / C-Focus a servicios. |
| `open_signal_window`                       | UI (ventana secundaria)             | `ArduinoGUI` / posible `SignalWindowController`          | Aceptable aquí; opcional modularizar más adelante. |
| `_on_serial_reconnect`                     | Lógica de comunicación serial       | Futuro `SerialService` / `ControlTab`                    | Debería vivir en un servicio de comunicación o en ControlTab. |
| `_update_connection_status`                | Lógica de estado de conexión        | `ControlTab` / `SerialService`                           | Solo debería actualizar modelo/servicio; UI actualizada por señales. |
| `update_data`                              | Parsing de datos + actualización UI | Futuro `SerialService` / `ControlTab`                    | Claro candidato a servicio dedicado de adquisición. |
| `_on_recording_started`                    | Logging                             | `RecordingTab`                                           | Puede quedar como simple callback; sin lógica extra. |
| `_on_recording_stopped`                    | Logging                             | `RecordingTab`                                           | Igual que el anterior. |
| `_on_analysis_completed`                   | Logging                             | `AnalysisTab` / `TransferFunctionAnalyzer`               | Orquestación ligera; OK en `ArduinoGUI`. |
| `_on_show_plot` (desde AnalysisTab)        | Apertura de `MatplotlibWindow`      | `ArduinoGUI` / posible `PlotService`                     | Puede quedarse como orquestador de ventanas. |
| `send_command`                             | Envío directo a hardware            | `SerialService` / `ControlTab`                           | A largo plazo debe ser API de servicio serial, no de GUI. |
| `_on_position_hold`                        | Formateo y envío de comando         | `ControlTab` / `MotorProtocol` / `SerialService`         | Lógica de alto nivel de control → mover a módulo de control. |
| `_on_brake`                                | Formateo y envío de comando         | `ControlTab` / `MotorProtocol` / `SerialService`         | Igual que anterior. |
| `_on_settling_config`                      | Formateo y envío de comando         | `ControlTab` / `MotorProtocol` / `SerialService`         | Igual que anterior. |
| `_start_microscopy`                        | Lógica de negocio de microscopía    | **Nuevo `MicroscopyService`**                            | Coordina trayectoria, delays y captura; no debería estar en GUI raíz. |
| `_microscopy_move_to_point`                | Lógica de movimiento a punto        | `MicroscopyService`                                      | Usa `TestTab` como cliente de hardware; servicio debería orquestar. |
| `_microscopy_check_position`               | Verificación de llegada a posición  | `MicroscopyService`                                      | Algoritmo de timeout y checks → lógica de negocio. |
| `_microscopy_capture`                      | Lógica de captura por punto         | `MicroscopyService` / `CameraService`                    | Decide entre captura simple vs. autofoco. |
| `_stop_microscopy`                         | Stop de microscopía                 | `MicroscopyService`                                      | La UI debería solo disparar evento de stop. |
| `_finish_microscopy`                       | Finalización y resumen              | `MicroscopyService`                                      | Incluye logging y actualización de progreso. |
| `_setup_detection_services`                | Wiring servicios ↔ CameraTab        | `ArduinoGUI` (orquestador)                               | Aquí solo hay conexiones de señales; es correcto. |
| `_on_detection_ready`                      | Callback de resultados de detección | `CameraTab` / `ImgAnalysisTab`                           | Visualización podría residir en una de las tabs; `ArduinoGUI` solo enruta. |
| `_on_detection_status`                     | Mensajes de estado de detección     | `CameraTab`                                              | Lógica mínima de UI; puede moverse a la tab. |
| `_on_autofocus_started`                    | Mensaje de inicio de autofoco       | `CameraTab`                                              | UI pura; candidato a moverse. |
| `_on_autofocus_z_changed`                  | Actualización de estado Z           | `CameraTab`                                              | Pertenece a la tab/ventana que muestra el estado. |
| `_on_object_focused`                       | Mensaje de resultado por objeto     | `CameraTab`                                              | Igual que anterior. |
| `_on_autofocus_complete`                   | Manejo de fin de autofoco           | `MicroscopyService` / `CameraTab`                        | Contiene lógica de captura pendiente y microscopía; debe migrar. |
| `start_realtime_detection`                 | Inicio de detección en tiempo real  | `CameraTab` / `DetectionService`                         | UI debería llamar directamente al servicio o a un ImagingService. |
| `stop_realtime_detection`                  | Stop de detección en tiempo real    | `CameraTab` / `DetectionService`                         | Igual que anterior. |
| `connect_cfocus`                           | Conexión hardware C-Focus           | Futuro `CFocusService` / `AutofocusService`              | `ArduinoGUI` solo debería orquestar. |
| `disconnect_cfocus`                        | Desconexión C-Focus                 | `CFocusService` / `AutofocusService`                     | Igual que anterior. |
| `initialize_autofocus`                     | Configuración AutofocusService      | `AutofocusService` / `MicroscopyService`                 | Podría convertirse en método del servicio con parámetros bien definidos. |
| `_microscopy_capture_with_autofocus`       | Microscopía + detección + autofoco  | `MicroscopyService` (usa `AutofocusService` + `CameraService`) | Es la pieza central de "microscopía inteligente". |
| `_advance_microscopy_point`                | Avance de trayectoria               | `MicroscopyService`                                      | Lógica de negocio pura. |
| `closeEvent`                               | Cierre de app + limpieza hardware   | `ArduinoGUI` (lifecycle) + servicios                     | Bien que viva aquí, pero delegando `stop()` a servicios. |
| `main`                                     | Entry point de aplicación           | Módulo `main.py`                                         | Solo debe crear `QApplication` y `ArduinoGUI`. |

Este mapa servirá como **contrato de refactor**: antes de mover código verificaremos aquí a qué servicio/tab debe ir cada responsabilidad para no romper la funcionalidad actual.

---

## 📝 Notas

- El error actual de C-Focus en el log (`no se pudo inicializar handle`) proviene del controlador hardware (`CFocusController`), no de la integración de servicios.
- Para pruebas de código, es importante que el dispositivo C-Focus esté encendido, conectado por USB y sin ser usado por otra aplicación (p. ej. software de Mad City Labs).

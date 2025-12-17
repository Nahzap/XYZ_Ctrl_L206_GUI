# 📌 Perspectiva Integrada del Estado del Software
## XYZ_Ctrl_L206_GUI

**Fecha y hora:** 2025-12-16 23:26:04 -03:00  
**Autor:** Cascade (perspectiva técnica integrada)  
**Alcance:** Estado arquitectónico/funcional del sistema, riesgos, deuda técnica y plan recomendado.  

---

## 0) Dashboard (indicadores de progreso)

### 0.1 Estado por macro-capacidad (producto)

| Capacidad | Estado | Evidencia directa | Riesgo principal |
|---|---:|---|---|
| Serial + telemetría + UI base | ✅ Operativo | `src/main.py:update_data()`, `src/core/communication/serial_handler.py` | Evolución de protocolo (4 vs 6 campos) sin tests |
| H∞ (síntesis + simulación + export) | ✅ Operativo | `src/core/controllers/hinf_controller.py`, `src/core/services/hinf_service.py`, `src/gui/tabs/hinf_tab.py` | Evitar refactors invasivos (“no tocar lo que funciona”) |
| Cámara Thorlabs (connect + live + capture) | ✅ Operativo | `src/core/services/camera_service.py`, `src/hardware/camera/camera_worker.py` | UI aún contiene lógica/hardware fallback |
| Detección (U2-Net + fallback contornos) | ✅ Operativo | `src/core/detection/u2net_detector.py`, `src/core/services/detection_service.py` | 2 implementaciones coexistiendo (U2NetDetector vs SmartFocusScorer) |
| Autofoco (Z-scan, máscara por contorno, captura BPoF) | ✅ Verificado | `src/core/services/autofocus_service.py` | Pipeline sensible a “quién captura y cuándo” |
| Microscopía inteligente (trayectoria→movimiento→detección→autofoco→captura) | 🟡 Operativo pero frágil | `src/core/services/microscopy_service.py`, wiring en `src/main.py` | Demasiadas dependencias inyectadas por callbacks + duplicación |
| Trayectorias + Control Dual (TestTab) | 🟡 Funcional, deuda alta | `src/gui/tabs/test_tab.py` | “Fat Tab” y lógica no testeable |

### 0.2 Separación UI ↔ lógica (arquitectura)

| Área | Estado | Indicador | Comentario |
|---|---:|---:|---|
| Tabs “livianas” | 🟡 Parcial | `parent_gui` aparece en 7 archivos (56 matches) | Aún hay acoplamiento directo y fallbacks legacy |
| Servicios asíncronos | ✅ Sólido | `DetectionService(QThread)`, `AutofocusService(QThread)`, `CameraWorker(QThread)` | Patrón correcto y repetible |
| Orquestación en `main.py` | 🟡 Aceptable | `ArduinoGUI` aún concentra wiring + algunos “decision points” | Ideal: solo composición + señales |

### 0.3 Salud de ingeniería (métricas observables)

| Métrica | Valor observado | Interpretación |
|---|---:|---|
| `parent_gui` en `src/` | 56 matches / 7 archivos | Señal de acoplamiento UI↔lógica aún presente |
| `TODO` en `src/` | 71 matches / 21 archivos | Hay backlog implícito; falta priorización/criterios de cierre |
| `FIXME` en `src/` | 0 matches | Bien, pero puede ocultar deuda “no etiquetada” |
| Duplicación detectada | `MicroscopyService.stop_microscopy()` aparece 2 veces | Riesgo de comportamiento inconsistente (shadowing) |

---

## 1) Resumen ejecutivo

El proyecto está en un estado **funcional y estable** para las capacidades principales ya integradas (serial/GUI base, captura de cámara Thorlabs, detección, autofoco, y diseño H∞), con señales claras de madurez en:

- separación progresiva por capas (`gui/`, `core/`, `hardware/`, `config/`, `models/`),
- uso consistente de **servicios asíncronos** para tareas pesadas (autofoco/detección/cámara),
- unificación de modelos y utilidades (dataclasses en `core/models`, métricas en `core/utils`).

La **principal deuda técnica actual** no está en la “calidad intrínseca” de los algoritmos, sino en la **distribución de responsabilidades y acoplamientos** (medibles):

- `gui/tabs/camera_tab.py` (~1425 líneas) y `gui/tabs/test_tab.py` (~1324 líneas) siguen concentrando **lógica de negocio** (“Fat Tab”).
- La “microscopía inteligente” (trayectorias + captura + detección + autofoco + C-Focus) está operativa, pero aún requiere consolidación para que el flujo viva en `MicroscopyService` y no en callbacks dispersos.

Y además, aparece deuda concreta en el código:

- existe un **método duplicado** `stop_microscopy()` dentro de `src/core/services/microscopy_service.py`.
- el patrón `parent_gui` aparece fuertemente concentrado en `CameraTab` (39 matches) y también en `TestTab`.

**Lectura global:** el sistema funciona, pero el costo de cambio seguirá creciendo hasta completar la Fase 13/14 (servicios + desacoplamiento UI), especialmente alrededor de cámara/microscopía/trayectorias.

---

## 2) Contexto histórico (línea temporal)

Esta historia es importante porque explica por qué el proyecto hoy es “funcional pero con deuda localizada”.

### 2.1 Noviembre 2025: salida del monolito
- Se partió de un `main.py` monolítico de miles de líneas (documentado en `PLAN_MODULARIZACION.md`).
- Se creó una estructura modular (carpetas `config/`, `core/`, `gui/`, `hardware/`, etc.) y se migraron componentes.

### 2.2 2025-12-12: diagnóstico “Fat Tab” (Fase 13)
- Documentado en `ARCHITECTURE_STATUS_2025-12.md` y `FASE_13_SERVICES_REFACTOR_PLAN.md`.
- Se reconoce que las tabs crecieron absorbiendo lógica (hardware/procesamiento/flujo), y se propone una capa de servicios dedicada.

### 2.3 2025-12-15: refactor con impacto fuerte
- `ARCHITECTURE_AUDIT_FINAL_2025-12-15.md` + `REFACTOR_PROGRESS_2025-12-15.md`.
- Resultados:
  - eliminación de duplicación (ej. SmartFocusScorer duplicado eliminado),
  - centralización de disponibilidad de hardware,
  - creación de modelos unificados (`DetectedObject`, `AutofocusResult`, etc.).

### 2.4 2025-12-16: estabilización de autofoco/microscopía y auditorías por módulo
- `AUDIT_2025-12-16.md`, `AUTOFOCUS_AUDIT_2025-12-16.md`, `CAMERA_AUDIT_2025-12-16.md`, `ARDUINO_CONNECTION_AUDIT_2025-12-16.md`, `CHANGELOG_2025-12-16.md`.
- Se corrigieron problemas de calidad de dato y flujo (captura en mejor foco, sharpness sobre máscara, UI durante microscopía, autodetección de puertos seriales, coherencia de normalización de imagen). 

---

## 3) Arquitectura actual (foto del sistema)

### 3.1 Capas y responsabilidades (intención ya lograda en gran parte)
- **`gui/`**: tabs y ventanas. Idealmente solo UI + delegación por señales.
- **`core/`**: lógica de negocio (control, análisis, detección, autofoco, servicios, trayectorias).
- **`hardware/`**: drivers y workers (Thorlabs, C-Focus, etc.).
- **`config/`**: logging, constantes, disponibilidad hardware.
- **`core/models/`**: dataclasses unificadas para intercambio de datos.

### 3.2 Patrón dominante recomendado
**Tab → Service → Hardware**, comunicando por **señales PyQt**, evitando `parent_gui.*` como API.

---

## 4) Estado por subsistema

### 4.1 Comunicación Serial (Arduino)
**Estado:** ✅ Corregido y más robusto.

Evidencias/documentos:
- `ARDUINO_CONNECTION_AUDIT_2025-12-16.md`

Fortalezas actuales:
- autodetección de puertos disponibles,
- baudrate coherente en UI y configuración,
- feedback más claro cuando falla la conexión.

Riesgo residual:
- aún hay superficies candidatas a “servicio serial” dedicado para que `main.py` solo enrute señales y la UI no procese datos.

### 4.2 Control H∞
**Estado:** ✅ Funcional y estable.

Evidencias/documentos:
- `AUDIT_2025-12-16.md`

Puntos fuertes:
- síntesis robusta con validación y escalado,
- extracción de PI equivalente,
- simulación consistente,
- logging suficiente.

Precaución operativa:
- hay consenso documental en **no tocar** lo que funciona salvo refactors estrictamente seguros.

### 4.3 Cámara Thorlabs
**Estado:** ✅ Funciona, ⚠️ requiere refactor estructural.

Evidencias/documentos:
- `CAMERA_AUDIT_2025-12-16.md`
- `FASE_14_CFOCUS_AUTOFOKUS_INTEGRATION.md`

Fortalezas:
- `CameraWorker` (thread adquisición) y `CameraService` (orquestación) son una base correcta.
- se corrigió bug de import (`Thorlabs` no definido) y se modularizó parte del setup UI.
- mejor coherencia entre imagen vista vs guardada (normalización consistente).

Deuda actual:
- `camera_tab.py` sigue mezclando UI + lógica + flujo (microscopía, detección, autofoco, sincronización con trayectorias).

### 4.4 Detección U2-Net
**Estado:** ✅ Funcional.

Fortalezas:
- modelo pesado cargado con patrón singleton,
- servicio asíncrono disponible,
- objetos detectados unificados vía modelos compartidos.

### 4.5 Autofoco + Microscopía inteligente
**Estado:** ✅ “end-to-end” operativo, con mejoras críticas recientes.

Evidencias/documentos:
- `AUTOFOCUS_AUDIT_2025-12-16.md`
- `CHANGELOG_2025-12-16.md`

Mejoras clave ya logradas:
- cálculo de sharpness usando **máscara del contorno U2-Net**, evitando sesgo de fondo,
- captura del frame en el **Best Position of Focus (BPoF)** y guardado usando `result.frame` (evita “guardar desenfocado”),
- control correcto de UI durante microscopía.

Riesgo residual:
- la “microscopía inteligente” todavía se siente como un pipeline sensible a la ubicación del código (si está en `main.py` vs servicio).
- el siguiente gran salto de mantenibilidad es terminar de mover la lógica a `MicroscopyService` con una API clara y señales.

### 4.6 Trayectorias y TestTab
**Estado:** ✅ Funcional, ⚠️ alto riesgo de mantenibilidad.

Evidencias/documentos:
- `AUDIT_2025-12-16.md` (tamaño y foco de refactor)
- `FASE_13_SERVICES_REFACTOR_PLAN.md`

Problema:
- `test_tab.py` concentra lógica de generación/ejecución y control dual.

Recomendación:
- crear `TrajectoryService` y `DualControlService` para bajar complejidad de UI.

### 4.7 Logging y configuración
**Estado:** ✅ Bien instrumentado.

Hechos relevantes:
- logging por niveles consistente,
- el archivo de log de sesión se **reinicia por ejecución** (útil para debugging por sesión).

---

## 5) Invariantes y criterios de estabilidad (lo que NO conviene romper)

- H∞: mantener comportamiento y resultados; cualquier refactor debe ser mecánico y testeado.
- Adquisición de cámara: no bloquear UI; mantener loop de adquisición en worker/thread.
- Autofoco: mantener captura del frame en BPoF y sharpness sobre máscara.
- Comunicación: nunca volver a hardcodear puertos; siempre autodetección + fallback.

---

## 6) Riesgos actuales (priorizados)

### 6.1 Riesgo alto: “Fat Tabs”
Impacto:
- baja testabilidad,
- errores difíciles de aislar,
- cambios pequeños disparan regresiones.

Ubicaciones:
- `camera_tab.py`, `test_tab.py`.

### 6.2 Riesgo alto: flujo de microscopía distribuido
Si el flujo vive en callbacks cruzados entre `main.py`/tabs/servicios, aumenta:
- fragilidad del pipeline,
- dificultad para manejar cancelación, timeouts, reintentos.

### 6.3 Riesgo medio: falta de pruebas automatizadas
No hay red de seguridad para:
- regresiones en parsing serial,
- regresiones en normalización/guardado,
- cambios en interfaces de dataclasses.

---

## 7) Roadmap recomendado (accionable)

### Sprint A (alta prioridad): reducir superficie de riesgo
- consolidar microscopía en `MicroscopyService` con señales claras (started/progress/captured/completed/error).
- reducir `camera_tab.py` a UI (ideal: < 600 líneas como primer objetivo; luego < 500).

### Sprint B (alta prioridad): TestTab → servicios
- crear `TrajectoryService` y `DualControlService`.
- dejar `test_tab.py` como UI + wiring.

### Sprint C (media): pruebas mínimas y “gates”
- agregar tests unitarios mínimos para:
  - normalización/guardado de frames,
  - selección/filtrado de objetos (área/circularidad/aspect ratio),
  - autodetección de puertos seriales (mock).

---

## 8) Definición de “hecho” para cerrar la fase de arquitectura

- `camera_tab.py`, `test_tab.py` < 500 líneas o justificadamente cerca.
- sin llamadas directas a `parent_gui.*` para lógica/hardware (solo señales/servicios).
- microscopía (con y sin autofoco) corre desde un servicio con API estable.
- pruebas mínimas pasan y cubren los flujos críticos.

---

## 9) Referencias consultadas (docs)

- `PLAN_MODULARIZACION.md`
- `ARCHITECTURE_STATUS_2025-12.md`
- `FASE_13_SERVICES_REFACTOR_PLAN.md`
- `FASE_14_CFOCUS_AUTOFOKUS_INTEGRATION.md`
- `ARCHITECTURE_AUDIT_FINAL_2025-12-15.md`
- `REFACTOR_PROGRESS_2025-12-15.md`
- `AUDIT_2025-12-16.md`
- `CAMERA_AUDIT_2025-12-16.md`
- `AUTOFOCUS_AUDIT_2025-12-16.md`
- `ARDUINO_CONNECTION_AUDIT_2025-12-16.md`
- `CHANGELOG_2025-12-16.md`

---

## 10) Análisis del proyecto (basado en el código fuente)

Esta sección **no se basa en los reportes**, sino en inspección directa del estado actual en `src/`.

### 10.1 Entradas, wiring y composición (qué hace realmente `main.py`)

**`src/main.py`** cumple hoy 3 roles simultáneos:

1. **Composición** (instancia de controladores, servicios, tabs y ventanas).
2. **Wiring** (conexión de señales: `DetectionService`→`CameraTab`, `AutofocusService`→`CameraTab` y delegación a `MicroscopyService`).
3. **Lógica residual** (puntos de decisión/flujo):
   - autodetección de Arduino (`_detect_arduino_port()`),
   - parseo/validación de telemetría (`update_data()`),
   - callback de autofoco (`_on_autofocus_complete()` delega y maneja “pending capture”).

**Lectura arquitectónica:** (1) y (2) son sanos; (3) debería migrar progresivamente a servicios/modelos para bajar fragilidad.

### 10.2 Flujo de datos real: microscopía inteligente (pipeline end-to-end)

El pipeline efectivo, mirando `src/main.py` + `src/core/services/microscopy_service.py` + `src/core/services/autofocus_service.py`, queda así:

1. **Usuario inicia microscopía** en UI (`CameraTab` emite `microscopy_start_requested(config)`).
2. `ArduinoGUI` conecta esa señal a `MicroscopyService.start_microscopy(config)`.
3. `MicroscopyService` obtiene trayectoria vía callback (`get_trajectory`) que apunta a `TestTab.current_trajectory`.
4. Por cada punto:
   - setea refs X/Y (`set_dual_refs`),
   - inicia control dual (`start_dual_control`),
   - espera condición de llegada (`is_position_reached`) con timeout.
5. Si `autofocus_enabled` y `cfocus_enabled`:
   - toma frame (`get_current_frame`),
   - detecta objetos (`smart_focus_scorer.assess_image()`),
   - filtra por área y morfología (circularidad/aspect ratio),
   - elige el objeto mayor,
   - dispara autofoco async (`AutofocusService.start_autofocus([largest_object])`).
6. Al completar autofoco:
   - `ArduinoGUI._on_autofocus_complete()` delega a `MicroscopyService.handle_autofocus_complete(results)`.
   - `MicroscopyService` guarda `result.frame` (BPoF) y opcional `result.frame_alt`.
7. Avanza punto y repite.

**Fortaleza:** el diseño ya tiene señales y separación razonable.  
**Debilidad:** el pipeline depende de **muchas dependencias inyectadas como callbacks** (más difícil de testear y de mantener coherencia).

### 10.3 Hallazgos críticos (bugs o fragilidad por estructura)

#### 10.3.1 Duplicación interna en `MicroscopyService`

En `src/core/services/microscopy_service.py` existe `def stop_microscopy(self)` **dos veces**. En Python esto significa que la segunda definición **pisa** a la primera.

Impacto típico:

- comportamiento divergente (por ejemplo: la primera versión emite `stopped`, la segunda no),
- debugging difícil (tu lectura del código puede estar mirando la versión “equivocada”),
- riesgo real en UI (stop no detiene dual control o no limpia flags coherentemente).

Esto es un **bug estructural**, no una preferencia de estilo.

#### 10.3.2 Acoplamiento UI↔lógica medible (uso de `parent_gui`)

Resultados de búsqueda en `src/`:

- `parent_gui`: 56 matches en 7 archivos
- concentración mayor en `src/gui/tabs/camera_tab.py` (39 matches)

Ejemplos de por qué importa (observado en `CameraTab`):

- fallback a `parent_gui.camera_service` si no se inyectó `camera_service`,
- acceso directo a `parent_gui.smart_focus_scorer`, `parent_gui.autofocus_service`, `parent_gui.cfocus_enabled`,
- wiring a `parent_gui.microscopy_service` desde la ventana de cámara.

**Lectura arquitectónica:** hay una intención clara de inyección (`CameraTab(... camera_service=...)`), pero todavía hay rutas legacy que mantienen el acoplamiento.

#### 10.3.3 Doble “motor” de detección coexistiendo

Hoy conviven dos stacks de detección:

- `U2NetDetector` + `DetectionService` (detección asíncrona para tiempo real / overlay).
- `SmartFocusScorer.assess_image()` (detección + evaluación de foco, usada para decidir autofoco y filtrar ROIs).

Esto es válido si está bien delimitado, pero introduce riesgos:

- parámetros de umbral/área pueden divergir,
- el concepto “objeto detectado” no es exactamente el mismo (`DetectedObject` vs `ObjectInfo`).

**Recomendación técnica:** declarar explícitamente “fuente de verdad” para cada caso de uso:

- para overlay en vivo: `U2NetDetector` (asíncrono, performance).
- para selección de ROI y foco: `SmartFocusScorer` (morfología + score + máscara).

### 10.4 Calidad de implementación (lo que está muy bien hecho)

- **SerialHandler**: reconstrucción de líneas con buffer circular (acertado para 1 Mbps).
- **AutofocusService**: captura en BPoF + retorno a BPoF, y sharpness sobre máscara por contorno.
- **CameraWorker**: loop en thread, emite `QImage` + `raw_frame` y contiene mitigaciones de timeouts y limpieza.
- **Modelos unificados** (`core/models`) reducen “errores por incompatibilidad” entre módulos.

---

## 11) Backlog priorizado (con Definition of Done)

### 11.1 Prioridad 🔴 Crítica (riesgo funcional / bug)

- **(C1) Eliminar duplicación de `stop_microscopy()`**
  - **Dónde:** `src/core/services/microscopy_service.py`
  - **DoD:**
    - existe una sola definición,
    - detener microscopía corta `_microscopy_active`, limpia pausa, y detiene control dual si estaba activo,
    - se emiten señales coherentes (`status_changed`, `stopped`) y se mantiene compatibilidad con `CameraTab`.

### 11.2 Prioridad 🟠 Alta (deuda que aumenta el costo de cambio)

- **(A1) Reducir acoplamiento `CameraTab`→`parent_gui`**
  - **DoD:**
    - `CameraTab` opera solo vía dependencias inyectadas (`camera_service`, `autofocus_service` opcional, `smart_focus_scorer`),
    - no hay fallbacks a `parent_gui.*` para lógica/hardware,
    - wiring de `CameraViewWindow` se hace desde `main.py` (orquestador), no desde la tab.

- **(A2) Extraer control dual + ejecución de trayectoria fuera de `TestTab`**
  - **DoD:**
    - `TestTab` queda como UI + señales,
    - existe `DualControlService`/`TrajectoryService` o equivalente,
    - `MicroscopyService` deja de depender de métodos UI (start/stop dual) y depende de un servicio.

### 11.3 Prioridad 🟡 Media (robustez y mantenibilidad)

- **(M1) Consolidar normalización uint16→uint8**
  - **Motivo:** hoy está repetida en `CameraTab` y `MicroscopyService`.
  - **DoD:** util común (en `core/utils`) y uso consistente en captura/vista.

- **(M2) Tests mínimos de regresión (smoke tests)**
  - **DoD:** tests que validen:
    - parsing de `update_data()` para formatos 4 y 6,
    - `AutofocusResult` trae `frame` y se usa para guardar,
    - filtros de área/morfología no devuelven objetos vacíos para inputs conocidos.

---

## 12) Criterio de cierre de la fase (estado “arquitectura bajo control”)

- `MicroscopyService` sin duplicaciones internas, API estable y testeable.
- `CameraTab` y `TestTab` bajan su dependencia de `parent_gui` a casi cero.
- `main.py` queda predominantemente como composición + wiring (sin lógica de negocio).
- existe un conjunto mínimo de pruebas para evitar regresiones en flujos críticos.

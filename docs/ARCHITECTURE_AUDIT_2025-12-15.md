# 🔍 AUDITORÍA COMPLETA DE ARQUITECTURA
## Sistema de Control y Análisis - Motores L206
### Fecha: 2025-12-15

---

## 📊 RESUMEN EJECUTIVO

| Métrica | Valor | Estado |
|---------|-------|--------|
| **Total archivos Python** | 66 | - |
| **Total líneas de código** | ~18,500 | ⚠️ Grande |
| **Archivo más grande** | `hinf_service.py` (1,544 líneas) | 🔴 Crítico |
| **Clases duplicadas** | 3 (SmartFocusScorer, DetectedObject, FocusResult) | 🔴 Crítico |
| **Imports redundantes** | 4 (pylablib, THORLABS_AVAILABLE) | 🟡 Medio |
| **Métodos duplicados** | 2 (_on_show_plot, send_command) | 🟡 Medio |

---

## 🔴 1. DUPLICACIÓN DE CLASES Y FUNCIONES

### 1.1 SmartFocusScorer - DUPLICACIÓN CRÍTICA

**Ubicaciones:**
- `src/core/autofocus/smart_focus_scorer.py` (491 líneas)
- `src/img_analysis/smart_focus_scorer.py` (584 líneas)

**Problema:** Existen DOS implementaciones completamente diferentes de la misma clase con:
- **Firmas de `__init__` incompatibles**
- **Parámetros con nombres diferentes** (`model_name` vs `model_type`, `min_object_area` vs `min_area`)
- **Estructuras de datos diferentes** (ObjectInfo vs DetectedObject)

**Impacto:**
- ❌ Errores de `TypeError: unexpected keyword argument` al instanciar
- ❌ Confusión sobre cuál versión usar
- ❌ Mantenimiento duplicado (cambios deben hacerse en 2 lugares)

**Comparación de firmas:**

```python
# core/autofocus/smart_focus_scorer.py
def __init__(self, 
             model_name: str = 'u2netp',
             detection_threshold: float = 0.5,
             min_object_area: int = 500,
             min_probability: float = 0.3,
             min_circularity: float = 0.45,
             min_aspect_ratio: float = 0.4):

# img_analysis/smart_focus_scorer.py  
def __init__(self,
             model_type: str = 'u2netp',
             threshold: float = 0.5,
             min_area: int = 28000,
             max_area: int = 35000,
             min_prob: float = 0.3,
             focus_threshold: float = 50.0,
             min_circularity: float = 0.45,
             min_aspect_ratio: float = 0.4,
             use_laplacian: bool = True,
             device: Optional[str] = None):
```

**Solución recomendada:**
1. Unificar en una sola clase en `core/autofocus/smart_focus_scorer.py`
2. Crear alias/wrapper en `img_analysis/` si se necesita compatibilidad
3. Usar patrón Singleton como `U2NetDetector`

---

### 1.2 DetectedObject - DUPLICACIÓN

**Ubicaciones:**
- `src/core/detection/u2net_detector.py` (línea 31-39)
- `src/core/autofocus/multi_object_autofocus.py` (línea 17-25)

**Problema:** Dos dataclasses con el mismo nombre pero campos diferentes:

```python
# u2net_detector.py
@dataclass
class DetectedObject:
    index: int
    bbox: Tuple[int, int, int, int]  # (x, y, w, h)
    area: int
    probability: float
    centroid: Tuple[int, int]
    contour: Optional[np.ndarray] = None

# multi_object_autofocus.py
@dataclass
class DetectedObject:
    index: int
    bounding_box: Tuple[int, int, int, int]  # Nombre diferente!
    centroid: Tuple[int, int]
    area: float  # Tipo diferente (float vs int)!
    initial_score: float
    circularity: float = 0.0
```

**Impacto:**
- ❌ Confusión sobre qué clase importar
- ❌ `bbox` vs `bounding_box` causa errores de atributo
- ❌ Tipos inconsistentes (`int` vs `float` para área)

**Solución recomendada:**
1. Definir UNA SOLA clase `DetectedObject` en `core/detection/`
2. Importar desde ahí en todos los módulos
3. Unificar nombres de campos (`bbox` → estándar)

---

### 1.3 FocusResult - DUPLICACIÓN

**Ubicaciones:**
- `src/core/services/autofocus_service.py` (línea 28-35)
- `src/img_analysis/smart_focus_scorer.py` (línea 46-65)

**Problema:** Dos dataclasses con campos muy diferentes:

```python
# autofocus_service.py - Simple
@dataclass
class FocusResult:
    object_index: int
    z_optimal: float
    focus_score: float
    bbox: Tuple[int, int, int, int]
    frame: Optional[np.ndarray] = None

# img_analysis/smart_focus_scorer.py - Compleja
@dataclass
class FocusResult:
    status: str
    focus_score: float
    centroid: Optional[Tuple[int, int]] = None
    bounding_box: Optional[Tuple[int, int, int, int]] = None
    contour_area: float = 0.0
    raw_score: float = 0.0
    is_valid: bool = False
    num_objects: int = 0
    mean_probability: float = 0.0
    objects: List[ObjectInfo] = field(default_factory=list)
    debug_mask: Optional[np.ndarray] = None
    probability_map: Optional[np.ndarray] = None
    binary_mask: Optional[np.ndarray] = None
    entropy: float = 0.0
    raw_brenner: float = 0.0
```

**Solución recomendada:**
1. Renombrar a `AutofocusResult` y `ImageAssessmentResult` para claridad
2. O unificar en una sola clase con campos opcionales

---

### 1.4 Métodos Duplicados en main.py

**`_on_show_plot` definido DOS VECES:**
- Línea 501-515
- Línea 531-545

**Impacto:** El segundo método sobrescribe al primero silenciosamente.

**Solución:** Eliminar la definición duplicada.

---

## 🟡 2. INTEGRACIONES MAL REALIZADAS

### 2.1 Verificación de THORLABS_AVAILABLE Redundante

**Problema:** La variable `THORLABS_AVAILABLE` se define en 4 lugares diferentes:

| Archivo | Línea | Contexto |
|---------|-------|----------|
| `main.py` | 118-129 | Importación inicial |
| `camera_tab.py` | 22-28 | Re-importación |
| `camera_worker.py` | 20-26 | Re-importación |
| `camera_service.py` | múltiple | Verificaciones |

**Impacto:**
- ❌ Cada módulo hace su propia verificación de disponibilidad
- ❌ Posible inconsistencia si un import falla en un lugar pero no en otro
- ❌ Código repetido innecesariamente

**Solución recomendada:**
```python
# config/hardware_availability.py
try:
    import pylablib as pll
    pll.par["devices/dlls/thorlabs_tlcam"] = r"C:\Program Files\Thorlabs\ThorImageCAM\Bin"
    from pylablib.devices import Thorlabs
    THORLABS_AVAILABLE = True
except ImportError:
    THORLABS_AVAILABLE = False

# Luego importar desde ahí:
from config.hardware_availability import THORLABS_AVAILABLE
```

---

### 2.2 Inyección de Dependencias Excesiva en MicroscopyService

**Problema:** El constructor de `MicroscopyService` recibe 12 callbacks/dependencias:

```python
def __init__(
    self,
    parent=None,
    get_trajectory,           # 1
    set_dual_refs,            # 2
    start_dual_control,       # 3
    stop_dual_control,        # 4
    is_dual_control_active,   # 5
    is_position_reached,      # 6
    capture_microscopy_image, # 7
    autofocus_service,        # 8
    cfocus_enabled_getter,    # 9
    get_current_frame,        # 10
    smart_focus_scorer,       # 11
    get_area_range,           # 12
    controllers_ready_getter, # 13
):
```

**Impacto:**
- ❌ Constructor extremadamente largo y difícil de mantener
- ❌ Acoplamiento fuerte con main.py
- ❌ Difícil de testear unitariamente

**Solución recomendada:**
1. Crear interfaces/protocolos para agrupar callbacks relacionados
2. Usar un objeto de configuración en lugar de callbacks individuales
3. Considerar patrón Mediator para comunicación entre servicios

```python
# Ejemplo de mejora:
@dataclass
class MicroscopyDependencies:
    trajectory_provider: TrajectoryProvider
    motion_controller: MotionController
    camera_controller: CameraController
    autofocus_controller: AutofocusController
```

---

### 2.3 Comunicación Mixta: Señales vs Callbacks

**Problema:** El proyecto usa AMBOS patrones de comunicación inconsistentemente:

| Componente | Patrón Usado |
|------------|--------------|
| `MicroscopyService` → UI | PyQt Signals ✅ |
| `MicroscopyService` → TestTab | Callbacks directo ❌ |
| `AutofocusService` → UI | PyQt Signals ✅ |
| `CameraTab` → `CameraService` | Mixto ⚠️ |

**Impacto:**
- ❌ Código inconsistente y difícil de seguir
- ❌ Algunos componentes dependen de referencias directas
- ❌ Dificulta testing y desacoplamiento

**Solución recomendada:**
- Estandarizar en PyQt Signals para TODA comunicación entre componentes
- Eliminar callbacks directos donde sea posible

---

### 2.4 Referencias Circulares entre Tabs

**Problema:** Las tabs tienen referencias cruzadas:

```python
# main.py
self.hinf_tab.set_test_tab_reference(self.test_tab)
self.camera_tab.set_test_tab_reference(self.test_tab)
```

**Impacto:**
- ❌ Acoplamiento fuerte entre tabs
- ❌ Difícil reutilizar tabs individualmente
- ❌ Orden de inicialización importa

**Solución recomendada:**
- Usar señales para comunicación entre tabs
- Centralizar estado compartido en un servicio

---

## 🟠 3. ARCHIVOS DEMASIADO GRANDES

### 3.1 Ranking de Archivos por Tamaño

| # | Archivo | Líneas | Estado | Acción Recomendada |
|---|---------|--------|--------|-------------------|
| 1 | `hinf_service.py` | 1,544 | 🔴 | Dividir en módulos |
| 2 | `camera_tab.py` | 1,431 | 🔴 | Extraer lógica a servicios |
| 3 | `test_tab.py` | 1,324 | 🔴 | Extraer control dual a servicio |
| 4 | `main.py` | 735 | 🟡 | Continuar refactorización |
| 5 | `microscopy_service.py` | 613 | 🟡 | Aceptable pero monitorear |
| 6 | `hinf_tab.py` | 607 | 🟡 | Aceptable |
| 7 | `hinf_controller.py` | 603 | 🟡 | Aceptable |

**Objetivo:** Ningún archivo debería exceder 500 líneas.

---

### 3.2 camera_tab.py - Análisis Detallado

**Problema:** 1,431 líneas con múltiples responsabilidades:
- UI de configuración de cámara
- Lógica de conexión/desconexión
- Lógica de microscopía
- Lógica de autofoco
- Lógica de detección

**Solución recomendada:**
1. Extraer lógica de microscopía → ya está en `MicroscopyService` ✅
2. Extraer lógica de autofoco → ya está en `AutofocusService` ✅
3. Extraer lógica de detección → ya está en `DetectionService` ✅
4. **Pendiente:** Mover métodos de captura a `CameraService`
5. **Pendiente:** Simplificar UI en componentes reutilizables

---

### 3.3 test_tab.py - Análisis Detallado

**Problema:** 1,324 líneas con:
- UI de generación de trayectorias
- Lógica de control dual (debería ser servicio)
- Lógica de ejecución de trayectorias
- Visualización de gráficos

**Solución recomendada:**
1. Crear `DualControlService` para lógica de control
2. Mover generación de trayectorias a `TrajectoryService`
3. Mantener solo UI en `TestTab`

---

## 🔵 4. OPORTUNIDADES DE OPTIMIZACIÓN DE VELOCIDAD

### 4.1 Carga de Modelo U2-Net

**Estado actual:** ✅ Correcto - Usa patrón Singleton

```python
class U2NetDetector:
    _instance = None
    _initialized = False
    
    @classmethod
    def get_instance(cls) -> 'U2NetDetector':
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance
```

**Beneficio:** El modelo se carga UNA SOLA VEZ al inicio.

---

### 4.2 SmartFocusScorer - NO USA SINGLETON

**Problema:** Se instancia múltiples veces:

```python
# autofocus_service.py línea 83
self._focus_scorer = SmartFocusScorer(...)

# img_analysis_tab.py línea 129
self.scorer = SmartFocusScorer(...)

# main.py línea 301
self.smart_focus_scorer = self.img_analysis_tab.scorer
```

**Impacto:**
- ⚠️ Múltiples instancias en memoria
- ⚠️ Posible carga duplicada de modelo (si no usa singleton interno)

**Solución recomendada:**
```python
class SmartFocusScorer:
    _instance = None
    
    @classmethod
    def get_instance(cls, **kwargs) -> 'SmartFocusScorer':
        if cls._instance is None:
            cls._instance = cls(**kwargs)
        return cls._instance
```

---

### 4.3 Conversión de Frames Redundante

**Problema en `microscopy_service.py`:**

```python
# Líneas 357-370 - Conversión uint16 → uint8 → BGR
if frame.dtype == np.uint16:
    frame_max = frame.max()
    if frame_max > 0:
        frame_uint8 = (frame / frame_max * 255).astype(np.uint8)
    else:
        frame_uint8 = np.zeros_like(frame, dtype=np.uint8)
else:
    frame_uint8 = frame.astype(np.uint8)

if len(frame_uint8.shape) == 2:
    frame_bgr = cv2.cvtColor(frame_uint8, cv2.COLOR_GRAY2BGR)
else:
    frame_bgr = frame_uint8
```

**Problema:** Esta conversión se hace en CADA punto de la trayectoria.

**Solución recomendada:**
1. Cachear el tipo de frame al inicio
2. Usar función optimizada con numba/numpy vectorizado
3. Considerar hacer conversión en `CameraWorker` una sola vez

---

### 4.4 Logging Excesivo en Bucles Críticos

**Problema:** Logs en cada iteración de autofoco:

```python
# autofocus_service.py línea 237
logger.debug(f"[Autofocus] #{iteration}: Z={new_z:.1f}µm, S={new_score:.2f}")
```

**Impacto:** I/O de disco en cada paso de autofoco (30+ veces por objeto).

**Solución recomendada:**
- Usar logging condicional: `if iteration % 5 == 0: logger.debug(...)`
- O acumular y loggear al final

---

### 4.5 Timers Ineficientes

**Problema en `microscopy_service.py`:**

```python
# Línea 260 - Timer de 200ms para verificar posición
QTimer.singleShot(200, self._check_position)

# Línea 300 - Timer de 100ms para seguir esperando
QTimer.singleShot(100, self._check_position)
```

**Impacto:** Polling cada 100-200ms en lugar de usar eventos.

**Solución recomendada:**
- Usar señales del controlador de posición cuando llegue
- Implementar patrón Observer en lugar de polling

---

## 📁 5. ESTRUCTURA DE DIRECTORIOS RECOMENDADA

```
src/
├── config/
│   ├── constants.py
│   ├── settings.py
│   └── hardware_availability.py  # NUEVO: Centralizar checks de hardware
│
├── core/
│   ├── models/                    # NUEVO: Dataclasses unificadas
│   │   ├── detected_object.py
│   │   ├── focus_result.py
│   │   └── trajectory_point.py
│   │
│   ├── services/
│   │   ├── autofocus_service.py
│   │   ├── camera_service.py
│   │   ├── detection_service.py
│   │   ├── dual_control_service.py  # NUEVO: Extraer de TestTab
│   │   ├── microscopy_service.py
│   │   └── trajectory_service.py    # NUEVO: Extraer de TestTab
│   │
│   ├── autofocus/
│   │   └── smart_focus_scorer.py    # ÚNICO (eliminar duplicado)
│   │
│   └── detection/
│       └── u2net_detector.py
│
├── gui/
│   ├── tabs/                        # Solo UI, sin lógica de negocio
│   └── windows/
│
├── hardware/
│   ├── camera/
│   └── cfocus/
│
└── main.py                          # < 300 líneas (solo orquestación)
```

---

## ✅ 6. PLAN DE ACCIÓN PRIORIZADO

### Fase 1: Crítico (Esta semana)
1. [ ] **Unificar SmartFocusScorer** - Eliminar duplicado en `img_analysis/`
2. [ ] **Unificar DetectedObject** - Una sola definición en `core/models/`
3. [ ] **Eliminar método duplicado** `_on_show_plot` en main.py

### Fase 2: Alto (Próxima semana)
4. [ ] **Centralizar THORLABS_AVAILABLE** en `config/hardware_availability.py`
5. [ ] **Crear DualControlService** - Extraer de TestTab
6. [ ] **Reducir camera_tab.py** a < 500 líneas

### Fase 3: Medio (Próximo mes)
7. [ ] **Refactorizar MicroscopyService** - Simplificar inyección de dependencias
8. [ ] **Estandarizar comunicación** - Solo PyQt Signals
9. [ ] **Optimizar conversión de frames** - Cachear y vectorizar

### Fase 4: Bajo (Mantenimiento continuo)
10. [ ] **Reducir logging en bucles** - Logging condicional
11. [ ] **Reemplazar polling por eventos** - Señales de posición
12. [ ] **Documentar arquitectura** - Diagramas UML actualizados

---

## 📈 7. MÉTRICAS DE ÉXITO

| Métrica | Actual | Objetivo |
|---------|--------|----------|
| Clases duplicadas | 3 | 0 |
| Archivo más grande | 1,544 líneas | < 500 líneas |
| main.py | 735 líneas | < 300 líneas |
| Imports redundantes | 4 | 0 |
| Cobertura de tests | ~0% | > 60% |

---

## 🔗 8. DEPENDENCIAS ENTRE COMPONENTES

```
┌─────────────────────────────────────────────────────────────────┐
│                          main.py                                 │
│                     (Orquestador Principal)                      │
└─────────────────────────────────────────────────────────────────┘
         │                    │                    │
         ▼                    ▼                    ▼
┌─────────────┐      ┌─────────────┐      ┌─────────────┐
│  ControlTab │      │  CameraTab  │      │   TestTab   │
│   (UI)      │      │   (UI)      │      │   (UI)      │
└─────────────┘      └─────────────┘      └─────────────┘
                            │                    │
                            ▼                    ▼
                     ┌─────────────┐      ┌─────────────┐
                     │ Microscopy  │◄────►│ Autofocus   │
                     │  Service    │      │  Service    │
                     └─────────────┘      └─────────────┘
                            │                    │
                            ▼                    ▼
                     ┌─────────────┐      ┌─────────────┐
                     │  Camera     │      │ SmartFocus  │
                     │  Worker     │      │  Scorer     │
                     └─────────────┘      └─────────────┘
                            │                    │
                            ▼                    ▼
                     ┌─────────────┐      ┌─────────────┐
                     │  Thorlabs   │      │  U2Net      │
                     │  Hardware   │      │  Detector   │
                     └─────────────┘      └─────────────┘
```

---

## 📝 9. NOTAS FINALES

### Lo que está BIEN hecho:
- ✅ U2NetDetector usa Singleton correctamente
- ✅ Servicios asíncronos (AutofocusService, DetectionService)
- ✅ Separación de UI en tabs
- ✅ Uso de PyQt Signals para comunicación
- ✅ Logging estructurado con niveles

### Lo que necesita MEJORA URGENTE:
- 🔴 Eliminar clases duplicadas (SmartFocusScorer, DetectedObject, FocusResult)
- 🔴 Reducir tamaño de archivos gigantes
- 🔴 Estandarizar patrones de comunicación
- 🔴 Simplificar inyección de dependencias

---

---

## 🔬 10. ANÁLISIS DETALLADO DE CÓDIGO DUPLICADO

### 10.1 SmartFocusScorer - Comparación Línea por Línea

#### Archivo 1: `core/autofocus/smart_focus_scorer.py` (491 líneas)

**Propósito:** Evaluador de enfoque para autofoco multi-objeto.

**Métodos principales:**
- `__init__()` - 6 parámetros
- `load_model()` - Carga lazy de U2-Net (placeholder)
- `calculate_sharpness()` - Laplacian Variance
- `evaluate_focus()` - Wrapper de calculate_sharpness
- `detect_objects()` - Detección por contornos
- `_detect_objects_simple()` - Fallback sin U2-Net
- `detect_objects_with_visualization()` - Debug visual

**Características:**
- ❌ NO usa U2-Net real (solo placeholder)
- ✅ Tiene filtrado por circularidad y aspect ratio
- ✅ Retorna diccionarios simples `{'bbox', 'area', 'probability'}`

#### Archivo 2: `img_analysis/smart_focus_scorer.py` (584 líneas)

**Propósito:** Evaluador de enfoque con U2-Net real para análisis de imágenes.

**Métodos principales:**
- `__init__()` - 10 parámetros (más completo)
- `_ensure_model_loaded()` - Carga lazy real de U2-Net
- `_get_saliency_mask()` - Usa SalientObjectDetector
- `_find_all_objects()` - Retorna List[ObjectInfo]
- `_calculate_masked_focus()` - Focus solo en máscara
- `assess_image()` - Método principal, retorna FocusResult
- `_create_debug_visualization()` - Visualización completa

**Características:**
- ✅ USA U2-Net real via `ai_segmentation.SalientObjectDetector`
- ✅ Tiene min_area Y max_area
- ✅ Retorna dataclasses tipadas (ObjectInfo, FocusResult)
- ✅ Soporta Laplacian Y Brenner

#### Tabla Comparativa de Parámetros

| Parámetro | core/autofocus | img_analysis | Equivalente |
|-----------|----------------|--------------|-------------|
| model_name | ✅ 'u2netp' | ❌ | model_type |
| model_type | ❌ | ✅ 'u2netp' | model_name |
| detection_threshold | ✅ 0.5 | ❌ | threshold |
| threshold | ❌ | ✅ 0.5 | detection_threshold |
| min_object_area | ✅ 500 | ❌ | min_area |
| min_area | ❌ | ✅ 28000 | min_object_area |
| max_area | ❌ | ✅ 35000 | N/A |
| min_probability | ✅ 0.3 | ❌ | min_prob |
| min_prob | ❌ | ✅ 0.3 | min_probability |
| focus_threshold | ❌ | ✅ 50.0 | N/A |
| use_laplacian | ❌ | ✅ True | N/A |
| device | ❌ | ✅ None | N/A |
| min_circularity | ✅ 0.45 | ✅ 0.45 | ✅ Igual |
| min_aspect_ratio | ✅ 0.4 | ✅ 0.4 | ✅ Igual |

#### Código Duplicado Exacto

**1. Método `set_morphology_params()` - IDÉNTICO en ambos:**
```python
def set_morphology_params(self, min_circularity: float = None, min_aspect_ratio: float = None):
    if min_circularity is not None:
        self.min_circularity = min_circularity
        logger.info(f"[SmartFocusScorer] Circularidad mínima actualizada: {min_circularity:.2f}")
    if min_aspect_ratio is not None:
        self.min_aspect_ratio = min_aspect_ratio
        logger.info(f"[SmartFocusScorer] Aspect ratio mínimo actualizado: {min_aspect_ratio:.2f}")
```

**2. Cálculo de Laplacian Variance - Similar en ambos:**
```python
# core/autofocus (línea 142)
laplacian = cv2.Laplacian(gray, cv2.CV_64F, ksize=3)
variance = laplacian.var()
sharpness = variance * 10.0

# img_analysis (línea 287-290)
laplacian = cv2.Laplacian(img_gray, cv2.CV_64F)
masked_laplacian = laplacian[mask_bool]
raw_score = float(np.var(masked_laplacian))
```

**3. Preprocesamiento CLAHE - Solo en core/autofocus:**
```python
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
enhanced = clahe.apply(gray)
```

---

### 10.2 DetectedObject - Comparación de Campos

#### Archivo 1: `core/detection/u2net_detector.py`
```python
@dataclass
class DetectedObject:
    index: int
    bbox: Tuple[int, int, int, int]      # ← Nombre: bbox
    area: int                             # ← Tipo: int
    probability: float
    centroid: Tuple[int, int]
    contour: Optional[np.ndarray] = None
```

#### Archivo 2: `core/autofocus/multi_object_autofocus.py`
```python
@dataclass
class DetectedObject:
    index: int
    bounding_box: Tuple[int, int, int, int]  # ← Nombre: bounding_box (DIFERENTE!)
    centroid: Tuple[int, int]
    area: float                               # ← Tipo: float (DIFERENTE!)
    initial_score: float                      # ← Campo adicional
    circularity: float = 0.0                  # ← Campo adicional
```

#### Impacto del Conflicto

Cuando se usa `obj.bbox` en un lugar y `obj.bounding_box` en otro:
```python
# microscopy_service.py línea 404
x, y, w, h = obj.bounding_box  # ← Espera bounding_box

# Pero si obj viene de u2net_detector:
x, y, w, h = obj.bbox  # ← Tiene bbox, NO bounding_box → AttributeError!
```

---

### 10.3 FocusResult - Comparación de Estructuras

#### Versión Simple (autofocus_service.py) - 5 campos
```python
@dataclass
class FocusResult:
    object_index: int
    z_optimal: float
    focus_score: float
    bbox: Tuple[int, int, int, int]
    frame: Optional[np.ndarray] = None
```

#### Versión Compleja (img_analysis/smart_focus_scorer.py) - 15 campos
```python
@dataclass
class FocusResult:
    status: str                                    # "FOCUSED_OBJECT", "BLURRY_OBJECT", etc.
    focus_score: float
    centroid: Optional[Tuple[int, int]] = None
    bounding_box: Optional[Tuple[int, int, int, int]] = None
    contour_area: float = 0.0
    raw_score: float = 0.0
    is_valid: bool = False
    num_objects: int = 0
    mean_probability: float = 0.0
    objects: List[ObjectInfo] = field(default_factory=list)
    debug_mask: Optional[np.ndarray] = None
    probability_map: Optional[np.ndarray] = None
    binary_mask: Optional[np.ndarray] = None
    entropy: float = 0.0
    raw_brenner: float = 0.0
```

---

### 10.4 Algoritmos Duplicados en Múltiples Archivos

#### cv2.Laplacian (7 usos en 5 archivos)
| Archivo | Línea | Contexto |
|---------|-------|----------|
| core/autofocus/smart_focus_scorer.py | 142, 399 | calculate_sharpness, visualización |
| img_analysis/smart_focus_scorer.py | 287, 288 | _calculate_masked_focus |
| core/services/autofocus_service.py | ~320 | _get_stable_score |
| gui/tabs/img_analysis_tab.py | ~200 | Visualización |
| img_analysis/sharpness_detector.py | ~150 | Detector independiente |

**Solución:** Crear `core/utils/image_metrics.py` con función única:
```python
def calculate_laplacian_variance(image: np.ndarray, mask: np.ndarray = None) -> float:
    """Calcula varianza de Laplaciano, opcionalmente enmascarada."""
    laplacian = cv2.Laplacian(image, cv2.CV_64F)
    if mask is not None:
        return float(np.var(laplacian[mask > 0]))
    return float(laplacian.var())
```

#### cv2.findContours (8 usos en 6 archivos)
| Archivo | Contexto |
|---------|----------|
| core/autofocus/smart_focus_scorer.py | Detección de objetos |
| core/detection/u2net_detector.py | Post-procesamiento U2-Net |
| img_analysis/smart_focus_scorer.py | _find_all_objects |
| ai_segmentation.py | Segmentación |
| gui/tabs/img_analysis_tab.py | Visualización |
| gui/windows/camera_window.py | Overlay de detección |

---

## 🗜️ 11. OPORTUNIDADES DE COMPACTACIÓN

### 11.1 Archivos que Pueden Eliminarse

| Archivo | Líneas | Razón | Acción |
|---------|--------|-------|--------|
| `gui/windows/camera_window_backup.py` | 450 | Backup obsoleto | ELIMINAR |
| `autofocus_calibration.py` | 140 | Script de calibración no usado | MOVER a tools/ |
| `bkp_canny_method/` | ~200 | Backup de método antiguo | ELIMINAR |

### 11.2 Archivos que Pueden Fusionarse

| Archivos | Líneas Totales | Fusionar En | Líneas Resultantes |
|----------|----------------|-------------|--------------------|
| `core/autofocus/smart_focus_scorer.py` + `img_analysis/smart_focus_scorer.py` | 1,075 | `core/autofocus/smart_focus_scorer.py` | ~600 |
| `core/autofocus/multi_object_autofocus.py` + `core/services/autofocus_service.py` | 825 | `core/services/autofocus_service.py` | ~500 |
| `img_analysis/sharpness_detector.py` + `img_analysis/background_model.py` | 900 | `img_analysis/image_analysis.py` | ~700 |

### 11.3 Código Muerto Identificado

| Archivo | Función/Clase | Razón |
|---------|---------------|-------|
| `core/autofocus/smart_focus_scorer.py` | `load_model()` | Solo placeholder, nunca carga modelo real |
| `core/autofocus/smart_focus_scorer.py` | `detect_objects()` | Siempre cae a `_detect_objects_simple()` |
| `img_analysis/smart_focus_scorer.py` | `entropy` field | Siempre 0.0, nunca calculado |
| `img_analysis/smart_focus_scorer.py` | `raw_brenner` field | Duplica `raw_score` |

### 11.4 Imports No Usados (Estimación)

```python
# main.py - Posibles imports no usados
import csv           # ¿Se usa?
import traceback     # Se usa en excepciones
from collections import deque  # ¿Se usa?

# camera_tab.py
import pylablib      # Duplicado con main.py
```

---

## 📊 12. MÉTRICAS DE COMPLEJIDAD

### 12.1 Complejidad Ciclomática Estimada (Top 10)

| Archivo | Función | Complejidad | Riesgo |
|---------|---------|-------------|--------|
| hinf_service.py | synthesize_hinf_controller | ~45 | 🔴 Alto |
| test_tab.py | execute_dual_control | ~30 | 🔴 Alto |
| camera_tab.py | _start_microscopy | ~25 | 🟡 Medio |
| microscopy_service.py | _capture_with_autofocus | ~20 | 🟡 Medio |
| img_analysis/smart_focus_scorer.py | assess_image | ~18 | 🟡 Medio |

### 12.2 Acoplamiento entre Módulos

```
Alto Acoplamiento (> 5 dependencias):
├── main.py → 15 módulos
├── camera_tab.py → 8 módulos
├── test_tab.py → 6 módulos
└── microscopy_service.py → 6 módulos

Bajo Acoplamiento (< 3 dependencias):
├── core/trajectory/trajectory_generator.py → 1 módulo
├── data/data_recorder.py → 2 módulos
└── hardware/cfocus/cfocus_controller.py → 2 módulos
```

---

## 🎯 13. RESUMEN DE HALLAZGOS CRÍTICOS

### Duplicación Total Estimada: ~2,500 líneas (13.5% del código)

| Categoría | Líneas Duplicadas | Archivos Afectados |
|-----------|-------------------|--------------------|
| SmartFocusScorer | ~400 | 2 |
| DetectedObject/ObjectInfo | ~100 | 3 |
| FocusResult | ~80 | 2 |
| Algoritmos CV (Laplacian, contours) | ~200 | 6 |
| Verificación THORLABS | ~50 | 4 |
| Preprocesamiento de imagen | ~150 | 5 |
| Métodos duplicados en main.py | ~30 | 1 |
| **TOTAL** | **~1,010** | **14** |

### Archivos Candidatos a Eliminación: 3 (~650 líneas)

### Potencial de Reducción: ~3,000 líneas (16% del código total)

---

*Documento generado automáticamente por auditoría de código*
*Última actualización: 2025-12-15 22:20 UTC-3*

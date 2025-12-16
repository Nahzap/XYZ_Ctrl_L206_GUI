# 📋 PLAN DE REFACTORIZACIÓN - XYZ_Ctrl_L206_GUI
## Basado en Auditoría del 2025-12-15
### Versión: 1.0

---

## 🎯 OBJETIVO GENERAL

Reducir el código base de **~18,500 líneas** a **~12,000 líneas** (-35%) mediante:
1. Eliminación de código duplicado
2. Unificación de clases
3. Eliminación de código muerto
4. Compactación de archivos grandes

---

## 📊 MÉTRICAS OBJETIVO

| Métrica | Actual | Objetivo | Reducción |
|---------|--------|----------|-----------|
| Total líneas | 18,500 | 12,000 | -35% |
| Clases duplicadas | 3 | 0 | -100% |
| Archivo más grande | 1,544 | < 500 | -68% |
| main.py | 735 | < 300 | -59% |
| Archivos > 500 líneas | 12 | 5 | -58% |

---

## 🔴 FASE 1: CRÍTICO (Semana 1)

### 1.1 Unificar SmartFocusScorer
**Prioridad:** 🔴 CRÍTICA  
**Esfuerzo:** 4 horas  
**Impacto:** Elimina ~475 líneas duplicadas

#### Archivos Afectados:
- `src/core/autofocus/smart_focus_scorer.py` (491 líneas) → MANTENER + MEJORAR
- `src/img_analysis/smart_focus_scorer.py` (584 líneas) → ELIMINAR

#### Pasos:
1. [ ] Copiar funcionalidad de U2-Net real de `img_analysis/` a `core/autofocus/`
2. [ ] Unificar parámetros del `__init__`:
   ```python
   def __init__(
       self,
       model_name: str = 'u2netp',      # Unificado
       threshold: float = 0.5,           # Renombrar detection_threshold
       min_area: int = 500,              # Renombrar min_object_area
       max_area: int = 100000,           # NUEVO
       min_probability: float = 0.3,
       focus_threshold: float = 50.0,    # NUEVO
       min_circularity: float = 0.45,
       min_aspect_ratio: float = 0.4,
       use_laplacian: bool = True,       # NUEVO
       device: str = None                # NUEVO
   ):
   ```
3. [ ] Agregar método `assess_image()` de `img_analysis/`
4. [ ] Agregar soporte para Brenner además de Laplacian
5. [ ] Actualizar imports en todos los archivos que usan SmartFocusScorer:
   - `src/core/services/autofocus_service.py`
   - `src/core/autofocus/multi_object_autofocus.py`
   - `src/gui/tabs/img_analysis_tab.py`
   - `src/gui/tabs/camera_tab.py`
   - `src/gui/windows/camera_window.py`
6. [ ] Crear alias en `src/img_analysis/__init__.py`:
   ```python
   from core.autofocus.smart_focus_scorer import SmartFocusScorer
   ```
7. [ ] Eliminar `src/img_analysis/smart_focus_scorer.py`
8. [ ] Ejecutar tests y verificar funcionamiento

#### Verificación:
```bash
python -c "from core.autofocus.smart_focus_scorer import SmartFocusScorer; print('OK')"
python -c "from img_analysis import SmartFocusScorer; print('Alias OK')"
python src/main.py  # Verificar que inicia sin errores
```

---

### 1.2 Unificar DetectedObject
**Prioridad:** 🔴 CRÍTICA  
**Esfuerzo:** 2 horas  
**Impacto:** Elimina confusión de nombres y errores de AttributeError

#### Archivos Afectados:
- `src/core/detection/u2net_detector.py` (línea 31-39) → MANTENER como fuente única
- `src/core/autofocus/multi_object_autofocus.py` (línea 17-25) → ELIMINAR definición

#### Pasos:
1. [ ] Crear archivo `src/core/models/detected_object.py`:
   ```python
   from dataclasses import dataclass
   from typing import Tuple, Optional
   import numpy as np

   @dataclass
   class DetectedObject:
       """Objeto detectado unificado."""
       index: int
       bbox: Tuple[int, int, int, int]  # (x, y, w, h) - NOMBRE ESTÁNDAR
       area: float
       probability: float
       centroid: Tuple[int, int]
       contour: Optional[np.ndarray] = None
       circularity: float = 0.0
       focus_score: float = 0.0
       
       @property
       def bounding_box(self) -> Tuple[int, int, int, int]:
           """Alias para compatibilidad."""
           return self.bbox
   ```
2. [ ] Actualizar `src/core/detection/u2net_detector.py`:
   ```python
   from core.models.detected_object import DetectedObject
   ```
3. [ ] Actualizar `src/core/autofocus/multi_object_autofocus.py`:
   - Eliminar definición local de DetectedObject
   - Importar desde `core.models`
4. [ ] Actualizar `src/core/services/microscopy_service.py`:
   - Usar `obj.bbox` en lugar de `obj.bounding_box`
5. [ ] Crear `src/core/models/__init__.py`:
   ```python
   from .detected_object import DetectedObject
   ```

---

### 1.3 Eliminar Método Duplicado en main.py
**Prioridad:** 🔴 CRÍTICA  
**Esfuerzo:** 15 minutos  
**Impacto:** Elimina bug silencioso

#### Pasos:
1. [ ] Abrir `src/main.py`
2. [ ] Buscar `def _on_show_plot` (aparece 2 veces: líneas ~501 y ~531)
3. [ ] Eliminar la primera definición (líneas 501-515)
4. [ ] Mantener solo la segunda definición (líneas 531-545)

---

### 1.4 Eliminar Archivos Obsoletos
**Prioridad:** 🔴 CRÍTICA  
**Esfuerzo:** 30 minutos  
**Impacto:** Elimina ~650 líneas de código muerto

#### Pasos:
1. [ ] Eliminar `src/gui/windows/camera_window_backup.py` (450 líneas)
2. [ ] Mover `src/autofocus_calibration.py` a `tools/autofocus_calibration.py`
3. [ ] Eliminar directorio `bkp_canny_method/` si existe

---

## 🟡 FASE 2: ALTO (Semana 2)

### 2.1 Centralizar THORLABS_AVAILABLE
**Prioridad:** 🟡 ALTA  
**Esfuerzo:** 1 hora  
**Impacto:** Elimina 4 verificaciones redundantes

#### Pasos:
1. [ ] Crear `src/config/hardware_availability.py`:
   ```python
   """Verificación centralizada de disponibilidad de hardware."""
   import logging
   
   logger = logging.getLogger('MotorControl_L206')
   
   # Thorlabs Camera SDK
   try:
       import pylablib as pll
       pll.par["devices/dlls/thorlabs_tlcam"] = r"C:\Program Files\Thorlabs\ThorImageCAM\Bin"
       from pylablib.devices import Thorlabs
       THORLABS_AVAILABLE = True
       logger.info("Thorlabs SDK disponible")
   except ImportError:
       THORLABS_AVAILABLE = False
       logger.warning("Thorlabs SDK no disponible")
   except Exception as e:
       THORLABS_AVAILABLE = False
       logger.warning(f"Error configurando Thorlabs: {e}")
   ```
2. [ ] Actualizar imports en:
   - `src/main.py` (eliminar líneas 118-129)
   - `src/gui/tabs/camera_tab.py` (eliminar líneas 22-28)
   - `src/hardware/camera/camera_worker.py` (eliminar líneas 20-26)
   - `src/core/services/camera_service.py`
3. [ ] Usar:
   ```python
   from config.hardware_availability import THORLABS_AVAILABLE
   ```

---

### 2.2 Crear DualControlService
**Prioridad:** 🟡 ALTA  
**Esfuerzo:** 4 horas  
**Impacto:** Reduce test_tab.py de 1,324 a ~800 líneas

#### Pasos:
1. [ ] Crear `src/core/services/dual_control_service.py`
2. [ ] Mover de `test_tab.py`:
   - `start_dual_control()`
   - `stop_dual_control()`
   - `execute_dual_control()`
   - Variables: `dual_control_active`, `dual_control_timer`, `dual_integral_a/b`
3. [ ] Conectar via señales PyQt
4. [ ] Actualizar `test_tab.py` para usar el servicio

---

### 2.3 Unificar FocusResult
**Prioridad:** 🟡 ALTA  
**Esfuerzo:** 2 horas  
**Impacto:** Claridad de código

#### Pasos:
1. [ ] Crear `src/core/models/focus_result.py`:
   ```python
   @dataclass
   class AutofocusResult:
       """Resultado de autofoco Z-scan."""
       object_index: int
       z_optimal: float
       focus_score: float
       bbox: Tuple[int, int, int, int]
       frame: Optional[np.ndarray] = None

   @dataclass
   class ImageAssessmentResult:
       """Resultado de evaluación de imagen."""
       status: str
       focus_score: float
       objects: List[DetectedObject] = field(default_factory=list)
       # ... resto de campos
   ```
2. [ ] Renombrar en `autofocus_service.py`: `FocusResult` → `AutofocusResult`
3. [ ] Renombrar en `img_analysis/smart_focus_scorer.py`: `FocusResult` → `ImageAssessmentResult`

---

### 2.4 Crear Utilidades de Imagen Compartidas
**Prioridad:** 🟡 ALTA  
**Esfuerzo:** 2 horas  
**Impacto:** Elimina ~200 líneas de algoritmos duplicados

#### Pasos:
1. [ ] Crear `src/core/utils/image_metrics.py`:
   ```python
   import cv2
   import numpy as np
   
   def calculate_laplacian_variance(image: np.ndarray, mask: np.ndarray = None) -> float:
       """Calcula varianza de Laplaciano (métrica de nitidez)."""
       if len(image.shape) == 3:
           image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
       laplacian = cv2.Laplacian(image, cv2.CV_64F)
       if mask is not None:
           return float(np.var(laplacian[mask > 0]))
       return float(laplacian.var())
   
   def calculate_brenner_gradient(image: np.ndarray, mask: np.ndarray = None) -> float:
       """Calcula gradiente de Brenner (métrica de nitidez alternativa)."""
       # ... implementación
   
   def preprocess_for_detection(image: np.ndarray) -> np.ndarray:
       """Preprocesamiento estándar: CLAHE + Gaussian blur."""
       if len(image.shape) == 3:
           gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
       else:
           gray = image.copy()
       if gray.dtype == np.uint16:
           gray = (gray / 256).astype(np.uint8)
       clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
       enhanced = clahe.apply(gray)
       return cv2.GaussianBlur(enhanced, (5, 5), 0)
   ```
2. [ ] Actualizar todos los archivos que usan cv2.Laplacian directamente

---

## 🟠 FASE 3: MEDIO (Semana 3-4)

### 3.1 Reducir camera_tab.py
**Prioridad:** 🟠 MEDIA  
**Esfuerzo:** 6 horas  
**Impacto:** Reduce de 1,431 a ~600 líneas

#### Pasos:
1. [ ] Mover lógica de captura a `CameraService`
2. [ ] Extraer widgets complejos a `gui/widgets/`
3. [ ] Simplificar métodos largos

---

### 3.2 Dividir hinf_service.py
**Prioridad:** 🟠 MEDIA  
**Esfuerzo:** 4 horas  
**Impacto:** Reduce de 1,544 a ~400 líneas por archivo

#### Pasos:
1. [ ] Crear `src/core/services/hinf/`:
   - `synthesis.py` - Síntesis de controlador
   - `simulation.py` - Respuesta al escalón, Bode
   - `realtime.py` - Control en tiempo real
   - `io.py` - Export/import de controladores

---

### 3.3 Refactorizar MicroscopyService
**Prioridad:** 🟠 MEDIA  
**Esfuerzo:** 3 horas  
**Impacto:** Mejor mantenibilidad

#### Pasos:
1. [ ] Crear dataclass `MicroscopyDependencies` para agrupar callbacks
2. [ ] Reducir parámetros del constructor de 13 a 3-4
3. [ ] Usar inyección de dependencias más limpia

---

## 🔵 FASE 4: BAJO (Mantenimiento Continuo)

### 4.1 Optimizar Logging
- [ ] Reducir logging en bucles de autofoco
- [ ] Usar logging condicional: `if iteration % 5 == 0`

### 4.2 Reemplazar Polling por Eventos
- [ ] Usar señales de posición en lugar de QTimer polling

### 4.3 Agregar Tests Unitarios
- [ ] Cobertura objetivo: 60%
- [ ] Priorizar: SmartFocusScorer, MicroscopyService, AutofocusService

### 4.4 Documentación
- [ ] Actualizar README.md
- [ ] Crear diagramas UML de arquitectura
- [ ] Documentar API de servicios

---

## 📅 CRONOGRAMA

| Semana | Fase | Tareas | Líneas Eliminadas |
|--------|------|--------|-------------------|
| 1 | CRÍTICO | 1.1, 1.2, 1.3, 1.4 | ~1,200 |
| 2 | ALTO | 2.1, 2.2, 2.3, 2.4 | ~800 |
| 3 | MEDIO | 3.1, 3.2 | ~1,500 |
| 4 | MEDIO | 3.3 + Cleanup | ~500 |
| **Total** | | | **~4,000** |

---

## ✅ CHECKLIST DE VERIFICACIÓN

### Después de cada cambio:
- [ ] `python src/main.py` inicia sin errores
- [ ] Conexión de cámara funciona
- [ ] Autofoco funciona
- [ ] Microscopía automatizada funciona
- [ ] Control dual funciona

### Después de Fase 1:
- [ ] No hay clases duplicadas
- [ ] Todos los imports funcionan
- [ ] No hay archivos backup en src/

### Después de Fase 2:
- [ ] THORLABS_AVAILABLE se importa de un solo lugar
- [ ] DualControlService funciona
- [ ] Métricas de imagen centralizadas

### Final:
- [ ] Total líneas < 14,000
- [ ] Ningún archivo > 600 líneas
- [ ] main.py < 400 líneas
- [ ] Tests pasan

---

## 🚨 RIESGOS Y MITIGACIÓN

| Riesgo | Probabilidad | Impacto | Mitigación |
|--------|--------------|---------|------------|
| Romper funcionalidad existente | Alta | Alto | Commits pequeños, tests frecuentes |
| Conflictos de imports | Media | Medio | Verificar imports después de cada cambio |
| Regresiones en autofoco | Media | Alto | Probar con imágenes reales |
| Tiempo subestimado | Alta | Medio | Buffer de 50% en estimaciones |

---

## 📝 NOTAS

- **Backup antes de empezar:** `git checkout -b refactor-2025-12`
- **Commits frecuentes:** Después de cada tarea completada
- **No refactorizar y agregar features al mismo tiempo**
- **Priorizar funcionalidad sobre perfección**

---

*Plan creado: 2025-12-15*  
*Basado en: ARCHITECTURE_AUDIT_2025-12-15.md*  
*Próxima revisión: Después de Fase 1*

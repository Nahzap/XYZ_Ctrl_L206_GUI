# 🔬 Plan de Integración: Autofoco Multi-Objeto con U2-Net

**Documento creado:** 2025-12-08  
**Última actualización:** 2025-12-08  
**Autor:** Sistema de Control L206  
**Objetivo:** Integrar detección de objetos U2-Net con autofoco individual por objeto para generar una BBDD de imágenes enfocadas eficientemente.

---

## 📋 Resumen Ejecutivo

### Capacidades Actuales del Sistema
1. ✅ Detectar objetos salientes usando U2-Net (Salient Object Detection)
2. ✅ Calcular score de enfoque (Laplaciano) **por cada objeto** detectado
3. ✅ Retornar lista de **TODOS los objetos** con sus bounding boxes y scores
4. ✅ Ejecutar trayectorias XY con captura de imágenes
5. ❌ **NO** pre-detecta objetos antes de capturar
6. ❌ **NO** captura múltiples imágenes por punto (una por objeto)
7. ❌ **NO** realiza autofoco individual por objeto

### Objetivo Principal
Cuando el sistema llegue a un punto de la trayectoria:
1. **Pre-detectar** todos los objetos en el frame actual (sin mover Z)
2. **Para cada objeto detectado:**
   - Mover Z para maximizar el score de enfoque de ESE objeto específico
   - Capturar imagen cuando el objeto esté enfocado
3. **Resultado:** N imágenes por punto, donde N = número de objetos detectados

---

## 🎯 Arquitectura Propuesta

### Flujo de Captura Multi-Objeto con Autofoco

```
┌─────────────────────────────────────────────────────────────────────┐
│                    TRAYECTORIA XY (TestTab)                         │
│  Punto 1 → Punto 2 → ... → Punto N                                  │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼ (Trigger: llegó a punto XY)
┌─────────────────────────────────────────────────────────────────────┐
│              FASE 1: PRE-DETECCIÓN DE OBJETOS                       │
│                                                                      │
│  1. Capturar frame de referencia (Z actual)                         │
│  2. Ejecutar U2-Net → Lista de objetos detectados                   │
│  3. Filtrar por área mínima y probabilidad                          │
│  4. Obtener: [Obj1(bbox, centroid), Obj2(...), ..., ObjM(...)]     │
│                                                                      │
│  Si M = 0: Saltar punto, continuar trayectoria                      │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼ (Para cada objeto i = 1..M)
┌─────────────────────────────────────────────────────────────────────┐
│              FASE 2: AUTOFOCO POR OBJETO                            │
│                                                                      │
│  Para Objeto_i con ROI = bbox_i:                                    │
│    1. Z_start = Z_actual                                            │
│    2. Búsqueda Golden Section en rango [Z_min, Z_max]:              │
│       a. Mover a Z_test                                             │
│       b. Capturar frame                                             │
│       c. Calcular S_i = Laplacian(ROI_i) ← SOLO en bbox del obj    │
│       d. Actualizar rango según gradiente                           │
│    3. Z_optimo_i = posición con max(S_i)                            │
│    4. Mover a Z_optimo_i                                            │
│    5. Capturar imagen final → Clase_XXXXX_objY.png                  │
│                                                                      │
│  Resultado: M imágenes, cada una enfocada en su objeto              │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│              FASE 3: SIGUIENTE PUNTO                                │
│                                                                      │
│  - Restaurar Z a posición neutral (opcional)                        │
│  - Mover a siguiente punto XY de trayectoria                        │
│  - Repetir desde Fase 1                                             │
└─────────────────────────────────────────────────────────────────────┘
```

### Diferencia Clave vs Implementación Anterior

| Aspecto | Plan Original | Plan Multi-Objeto |
|---------|---------------|-------------------|
| Detección | Durante autofoco | **ANTES** del autofoco |
| Imágenes/punto | 1 | **N** (una por objeto) |
| ROI de enfoque | Objeto principal | **Cada objeto individual** |
| Z óptimo | Global | **Por objeto** |
| BBDD resultante | Mezcla de objetos | **Objetos aislados y enfocados** |

### Algoritmo de Búsqueda del Foco Óptimo

Basado en **Control Robusto (Zhou & Doyle)** y literatura de autofoco:

#### Opción A: Hill Climbing con Golden Section Search
```
Z_min, Z_max = rango_busqueda
φ = 0.618  # Golden ratio

while (Z_max - Z_min) > tolerancia:
    Z1 = Z_max - φ * (Z_max - Z_min)
    Z2 = Z_min + φ * (Z_max - Z_min)
    
    S1 = evaluar_enfoque(Z1)
    S2 = evaluar_enfoque(Z2)
    
    if S1 > S2:
        Z_max = Z2
    else:
        Z_min = Z1

Z_optimo = (Z_min + Z_max) / 2
```

#### Opción B: Fibonacci Search (más eficiente)
- Reduce el número de evaluaciones necesarias
- Mejor para sistemas con latencia en movimiento Z

#### Opción C: Gradient Ascent con Momentum
```
Z = Z_inicial
v = 0  # velocidad
α = learning_rate
β = 0.9  # momentum

for i in range(max_iter):
    S_actual = evaluar_enfoque(Z)
    S_delta = evaluar_enfoque(Z + δ) - S_actual
    
    gradiente = S_delta / δ
    v = β * v + α * gradiente
    Z = Z + v
    
    if |v| < tolerancia:
        break
```

---

## 📊 Métrica de Enfoque (Focus Measure)

Según el estudio comparativo de OpenCV y la literatura:

### Métricas Recomendadas (en orden de efectividad):

| Método | Fórmula | Pros | Contras |
|--------|---------|------|---------|
| **Laplacian Variance** | `Var(∇²I)` | Mejor balance velocidad/precisión | Sensible a ruido |
| **Tenengrad** | `Σ(Gx² + Gy²)` | Robusto a iluminación | Más lento |
| **Brenner** | `Σ(I(x+2) - I(x))²` | Muy rápido | Menos preciso |

### Implementación Actual (SmartFocusScorer)

```python
# Ya implementado en smart_focus_scorer.py
laplacian = cv2.Laplacian(img_gray, cv2.CV_64F)
focus_score = laplacian.var()  # Varianza del Laplaciano
```

### Mejora Propuesta: ROI-Based Focus

```python
def evaluar_enfoque_roi(frame, scorer):
    """
    Evalúa enfoque solo en la región del objeto detectado.
    Más robusto que evaluar toda la imagen.
    """
    # 1. Detectar objeto con U2-Net (rápido con GPU)
    result = scorer.assess_image(frame, return_debug_mask=False)
    
    if result.status == "EMPTY":
        return 0.0  # No hay objeto
    
    # 2. Extraer ROI del objeto principal
    x, y, w, h = result.bounding_box
    roi = frame[y:y+h, x:x+w]
    
    # 3. Calcular Laplacian Variance en ROI
    lap = cv2.Laplacian(roi, cv2.CV_64F)
    score = lap.var()
    
    return score
```

---

## 🔧 Componentes a Implementar

### 1. MultiObjectAutofocusController (Nuevo módulo)

**Ubicación:** `src/core/autofocus/multi_object_autofocus.py`

```python
from dataclasses import dataclass
from typing import List, Tuple, Optional, Callable
import numpy as np
import cv2

@dataclass
class DetectedObject:
    """Objeto detectado en pre-escaneo."""
    index: int
    bounding_box: Tuple[int, int, int, int]  # (x, y, w, h)
    centroid: Tuple[int, int]
    area: float
    initial_score: float

@dataclass
class FocusedCapture:
    """Resultado de captura enfocada de un objeto."""
    object_index: int
    z_optimal: float
    focus_score: float
    image_path: str
    bounding_box: Tuple[int, int, int, int]

class MultiObjectAutofocusController:
    """
    Controlador de autofoco multi-objeto.
    
    Flujo:
    1. Pre-detectar todos los objetos en el frame actual
    2. Para cada objeto, buscar su Z óptimo individual
    3. Capturar imagen cuando cada objeto esté enfocado
    """
    
    def __init__(
        self,
        scorer,  # SmartFocusScorer
        move_z_callback: Callable[[float], None],
        get_frame_callback: Callable[[], np.ndarray],
        get_z_position_callback: Callable[[], float]
    ):
        self.scorer = scorer
        self.move_z = move_z_callback
        self.get_frame = get_frame_callback
        self.get_z = get_z_position_callback
        
        # Parámetros de búsqueda
        self.z_range = 100      # µm de rango de búsqueda total
        self.z_tolerance = 1    # µm tolerancia final
        self.max_iterations = 15
        self.settle_time = 0.1  # segundos para estabilización mecánica
        
        # Parámetros de detección
        self.min_area = 100
        self.min_prob = 0.3
    
    def predetect_objects(self) -> List[DetectedObject]:
        """
        FASE 1: Pre-detecta todos los objetos en el frame actual.
        
        Returns:
            Lista de objetos detectados con sus bounding boxes
        """
        frame = self.get_frame()
        result = self.scorer.assess_image(frame, return_debug_mask=False)
        
        detected = []
        for i, obj in enumerate(result.objects):
            detected.append(DetectedObject(
                index=i,
                bounding_box=obj.bounding_box,
                centroid=obj.centroid,
                area=obj.area,
                initial_score=obj.focus_score
            ))
        
        return detected
    
    def focus_single_object(
        self, 
        obj: DetectedObject,
        z_center: Optional[float] = None
    ) -> Tuple[float, float]:
        """
        FASE 2: Busca el Z óptimo para UN objeto específico.
        
        Usa Golden Section Search evaluando SOLO el ROI del objeto.
        
        Args:
            obj: Objeto a enfocar
            z_center: Centro del rango de búsqueda (default: Z actual)
            
        Returns:
            (z_optimal, max_score)
        """
        if z_center is None:
            z_center = self.get_z()
        
        z_min = z_center - self.z_range / 2
        z_max = z_center + self.z_range / 2
        
        phi = 0.618  # Golden ratio
        
        # Golden Section Search
        while (z_max - z_min) > self.z_tolerance:
            z1 = z_max - phi * (z_max - z_min)
            z2 = z_min + phi * (z_max - z_min)
            
            s1 = self._evaluate_object_focus(z1, obj.bounding_box)
            s2 = self._evaluate_object_focus(z2, obj.bounding_box)
            
            if s1 > s2:
                z_max = z2
            else:
                z_min = z1
        
        z_optimal = (z_min + z_max) / 2
        
        # Mover a posición óptima y obtener score final
        self.move_z(z_optimal)
        time.sleep(self.settle_time)
        final_score = self._evaluate_object_focus(z_optimal, obj.bounding_box)
        
        return z_optimal, final_score
    
    def _evaluate_object_focus(
        self, 
        z_position: float, 
        bbox: Tuple[int, int, int, int]
    ) -> float:
        """
        Evalúa el score de enfoque SOLO en el ROI del objeto.
        
        Args:
            z_position: Posición Z a evaluar
            bbox: Bounding box del objeto (x, y, w, h)
            
        Returns:
            Score de enfoque en el ROI
        """
        self.move_z(z_position)
        time.sleep(self.settle_time)
        
        frame = self.get_frame()
        x, y, w, h = bbox
        
        # Extraer ROI
        roi = frame[y:y+h, x:x+w]
        
        # Convertir a grayscale si es necesario
        if len(roi.shape) == 3:
            roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        else:
            roi_gray = roi
        
        # Calcular Laplacian Variance en ROI
        laplacian = cv2.Laplacian(roi_gray, cv2.CV_64F)
        score = float(np.var(laplacian))
        
        return np.sqrt(score)  # Normalizado
    
    def capture_all_objects(
        self,
        objects: List[DetectedObject],
        save_folder: str,
        class_name: str,
        point_index: int
    ) -> List[FocusedCapture]:
        """
        FASE 2+3: Enfoca y captura cada objeto individualmente.
        
        Args:
            objects: Lista de objetos pre-detectados
            save_folder: Carpeta de destino
            class_name: Nombre de la clase para el archivo
            point_index: Índice del punto de trayectoria
            
        Returns:
            Lista de capturas realizadas
        """
        captures = []
        z_start = self.get_z()
        
        for obj in objects:
            # Buscar Z óptimo para este objeto
            z_opt, score = self.focus_single_object(obj, z_center=z_start)
            
            # Capturar imagen
            frame = self.get_frame()
            
            # Nombre: Clase_PPPPP_objOO.png (punto, objeto)
            filename = f"{class_name}_{point_index:05d}_obj{obj.index:02d}.png"
            filepath = os.path.join(save_folder, filename)
            
            cv2.imwrite(filepath, frame)
            
            captures.append(FocusedCapture(
                object_index=obj.index,
                z_optimal=z_opt,
                focus_score=score,
                image_path=filepath,
                bounding_box=obj.bounding_box
            ))
        
        return captures
```

### 2. Integración con main.py (Microscopía)

**Modificar:** `src/main.py` - método `_microscopy_capture()`

```python
def _microscopy_capture_multiobject(self):
    """
    NUEVO: Captura con pre-detección y autofoco multi-objeto.
    
    Reemplaza a _microscopy_capture() cuando autofoco está habilitado.
    """
    if not self.microscopy_active:
        return
    
    config = self.microscopy_config
    point_idx = self.microscopy_current_point
    
    # FASE 1: Pre-detectar objetos
    self.camera_tab.log_message(f"🔍 Pre-detectando objetos...")
    objects = self.autofocus_controller.predetect_objects()
    n_objects = len(objects)
    
    if n_objects == 0:
        self.camera_tab.log_message(f"  ⚠️ Sin objetos detectados - saltando punto")
        self._microscopy_next_point()
        return
    
    self.camera_tab.log_message(f"  ✓ {n_objects} objeto(s) detectado(s)")
    
    # FASE 2: Enfocar y capturar cada objeto
    captures = self.autofocus_controller.capture_all_objects(
        objects=objects,
        save_folder=config.get('save_folder', '.'),
        class_name=config.get('class_name', 'Imagen'),
        point_index=point_idx
    )
    
    # Log resultados
    for cap in captures:
        self.camera_tab.log_message(
            f"  📸 Obj{cap.object_index}: Z={cap.z_optimal:.1f}µm, "
            f"S={cap.focus_score:.1f}"
        )
    
    # Actualizar estadísticas
    self._total_images_captured += len(captures)
    
    # Continuar con siguiente punto
    self._microscopy_next_point()
```

### 3. Integración con CameraTab (UI)

**Modificar:** `src/gui/tabs/camera_tab.py`

```python
# Agregar checkbox en UI de microscopía:

self.autofocus_enabled_cb = QCheckBox("🔍 Autofoco Multi-Objeto")
self.autofocus_enabled_cb.setToolTip(
    "Pre-detecta objetos y captura una imagen enfocada por cada uno.\n"
    "Genera N imágenes por punto, donde N = objetos detectados."
)

# Agregar configuración de autofoco:
autofocus_group = QGroupBox("⚙️ Configuración Autofoco")
autofocus_layout = QFormLayout()

self.z_range_spin = QSpinBox()
self.z_range_spin.setRange(10, 500)
self.z_range_spin.setValue(100)
self.z_range_spin.setSuffix(" µm")
autofocus_layout.addRow("Rango Z:", self.z_range_spin)

self.z_tolerance_spin = QDoubleSpinBox()
self.z_tolerance_spin.setRange(0.1, 10)
self.z_tolerance_spin.setValue(1.0)
self.z_tolerance_spin.setSuffix(" µm")
autofocus_layout.addRow("Tolerancia:", self.z_tolerance_spin)

self.min_objects_spin = QSpinBox()
self.min_objects_spin.setRange(1, 50)
self.min_objects_spin.setValue(1)
autofocus_layout.addRow("Mín. objetos:", self.min_objects_spin)

autofocus_group.setLayout(autofocus_layout)
```

### 3. Comunicación con Motor Z

**Protocolo Arduino existente:**

```python
# Comando para mover motor Z
def move_z(self, position_um):
    """Mueve motor Z a posición absoluta."""
    # Usar protocolo serial existente
    command = f"Z{position_um}\n"
    self.serial_handler.send(command)
    
    # Esperar confirmación
    self.serial_handler.wait_for_response("OK")
```

---

## 📐 Parámetros de Configuración

### UI Propuesta (en CameraTab) - Multi-Objeto

```
┌─────────────────────────────────────────────────────────────┐
│ 🔍 Autofoco Multi-Objeto                                    │
├─────────────────────────────────────────────────────────────┤
│ [✓] Habilitar autofoco por objeto                           │
│                                                             │
│ ═══ Detección de Objetos ═══                                │
│ Área mínima:    [____100____] px²                           │
│ Prob. mínima:   [____0.3____]                               │
│ Max objetos:    [____10_____] por punto                     │
│                                                             │
│ ═══ Búsqueda de Foco ═══                                    │
│ Rango Z:        [____100____] µm                            │
│ Tolerancia:     [_____1_____] µm                            │
│ Tiempo estab.:  [____100____] ms                            │
│                                                             │
│ ═══ Captura ═══                                             │
│ Score mínimo:   [____15_____] (skip obj si <)               │
│ [✓] Guardar metadata JSON                                   │
│ [✓] Crop ROI individual                                     │
└─────────────────────────────────────────────────────────────┘
```

### Nomenclatura de Archivos

```
{save_folder}/
├── {class_name}_00000_obj00.png    # Punto 0, Objeto 0
├── {class_name}_00000_obj01.png    # Punto 0, Objeto 1
├── {class_name}_00000_obj02.png    # Punto 0, Objeto 2
├── {class_name}_00001_obj00.png    # Punto 1, Objeto 0
├── {class_name}_00001_obj01.png    # Punto 1, Objeto 1
├── ...
└── metadata.json                    # Info de todas las capturas
```

### Estructura de Metadata JSON

```json
{
  "session": {
    "timestamp": "2025-12-08T21:48:00",
    "class_name": "Polen",
    "total_points": 100,
    "total_objects": 287,
    "total_images": 287
  },
  "captures": [
    {
      "point_index": 0,
      "object_index": 0,
      "filename": "Polen_00000_obj00.png",
      "z_optimal_um": 1523.5,
      "focus_score": 45.2,
      "bbox": [120, 80, 200, 180],
      "centroid": [220, 170],
      "area_px": 28540
    },
    ...
  ]
}
```

---

## ⚡ Optimizaciones de Rendimiento

### 1. U2-Net Lite para Tiempo Real

El modelo `u2netp` (4.7 MB) ya es ligero, pero para autofoco:

```python
# Reducir resolución de entrada para velocidad
def fast_detect(self, frame):
    # Resize a 160x160 (vs 320x320 normal)
    small = cv2.resize(frame, (160, 160))
    mask = self.detector.predict(small)
    
    # Escalar bbox de vuelta
    scale_x = frame.shape[1] / 160
    scale_y = frame.shape[0] / 160
    # ...
```

### 2. Caché de Detección

```python
# Si el objeto no se movió mucho, reusar ROI
if self._roi_is_similar(new_bbox, cached_bbox):
    roi = cached_roi
else:
    roi = self._extract_roi(frame, new_bbox)
    cached_roi = roi
```

### 3. Evaluación Paralela

```python
# Evaluar múltiples posiciones Z en paralelo (si hardware lo permite)
from concurrent.futures import ThreadPoolExecutor

def evaluate_multiple_z(self, z_positions):
    with ThreadPoolExecutor(max_workers=2) as executor:
        futures = [executor.submit(self.evaluate_at_z, z) for z in z_positions]
        return [f.result() for f in futures]
```

---

## 📈 Métricas de Éxito

| Métrica | Objetivo | Medición |
|---------|----------|----------|
| Tiempo de autofoco/objeto | < 3 segundos | Cronómetro |
| Precisión Z | ± 2 µm | Comparar con manual |
| Tasa de éxito | > 95% | Objetos enfocados / total |
| Score S promedio | > 20 | Estadísticas de sesión |
| Objetos detectados/punto | Variable | Depende de muestra |
| Imágenes generadas | N × M | N puntos × M objetos/punto |

---

## 🗓️ Plan de Implementación

### Fase 1: Infraestructura Multi-Objeto (3-4 horas)
- [ ] Crear `src/core/autofocus/__init__.py`
- [ ] Crear `src/core/autofocus/multi_object_autofocus.py`
- [ ] Implementar `DetectedObject` y `FocusedCapture` dataclasses
- [ ] Implementar `predetect_objects()` usando SmartFocusScorer existente
- [ ] Tests unitarios de detección

### Fase 2: Autofoco por Objeto (3-4 horas)
- [ ] Implementar `focus_single_object()` con Golden Section Search
- [ ] Implementar `_evaluate_object_focus()` con ROI específico
- [ ] Agregar parámetros configurables (z_range, tolerance, settle_time)
- [ ] Validar convergencia con logs detallados

### Fase 3: Captura Multi-Objeto (2-3 horas)
- [ ] Implementar `capture_all_objects()` con nomenclatura correcta
- [ ] Agregar generación de metadata JSON
- [ ] Opción de crop ROI individual vs imagen completa

### Fase 4: Integración con Microscopía (3-4 horas)
- [ ] Modificar `main.py._microscopy_capture()` para modo multi-objeto
- [ ] Agregar UI de configuración en CameraTab
- [ ] Conectar callbacks de motor Z
- [ ] Actualizar barra de progreso (puntos + objetos)

### Fase 5: Optimización y Validación (2-3 horas)
- [ ] Profiling de rendimiento (U2-Net + movimiento Z)
- [ ] Implementar caché de detección si ROI similar
- [ ] Pruebas con muestras reales de polen/células
- [ ] Ajuste de umbrales por defecto

**Tiempo total estimado:** 13-18 horas

### Dependencias Críticas
1. **Motor Z funcional**: Verificar comunicación serial con Arduino
2. **SmartFocusScorer**: Ya implementado y funcionando ✓
3. **Cámara Thorlabs**: Captura de frames en tiempo real ✓

---

## 📚 Referencias

1. **Zhou, K., & Doyle, J. C.** (1998). *Essentials of Robust Control*. Prentice Hall.
   - Capítulo sobre optimización y control robusto

2. **Pech-Pacheco, J. L., et al.** (2000). *Diatom autofocusing in brightfield microscopy: a comparative study*. ICPR.
   - Varianza del Laplaciano como métrica de enfoque

3. **OpenCV Blog** (2025). *Autofocus using OpenCV: A Comparative Study of Focus Measures*.
   - Comparación de Laplacian, Tenengrad, Brenner, Entropy

4. **Nayar, S. K., & Nakagawa, Y.** (1992). *Shape from Focus*. CVPR.
   - Fundamentos teóricos de focus stacking

5. **U2-Net** (2020). *U²-Net: Going Deeper with Nested U-Structure for Salient Object Detection*.
   - Arquitectura del detector de objetos salientes

---

## ✅ Checklist Pre-Implementación

- [x] Algoritmo U2-Net funcionando
- [x] Score de enfoque (Laplaciano) implementado
- [x] Visualización de objetos detectados
- [x] Trayectoria XY funcionando
- [ ] Comunicación con motor Z verificada
- [ ] Latencia de movimiento Z medida
- [ ] Rango focal de la muestra conocido

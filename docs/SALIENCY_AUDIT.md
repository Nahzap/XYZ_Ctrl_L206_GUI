# 🔍 AUDITORÍA: Sistema de Saliencia en CameraViewWindow

**Fecha:** 2025-12-12  
**Problema:** Los frames NO coinciden - el heatmap se aplica a un frame diferente al visualizado

---

## 📋 FLUJO ACTUAL (INCORRECTO)

### 1. Captura de Frame (CameraWorker)
```
CameraWorker.run() → emite frame_ready(q_image, raw_frame)
                              ↓
CameraTab._on_frame_ready() → camera_view_window.update_frame(q_image, raw_frame)
```

### 2. Visualización (CameraViewWindow.update_frame)
```python
def update_frame(self, q_image, raw_frame=None):
    self.last_raw_frame = raw_frame  # ← GUARDA EL FRAME
    
    if should_draw:
        display_frame = self._draw_overlay(raw_frame)  # ← USA FRAME ACTUAL
        q_image = self._frame_to_qimage(display_frame)
    
    self.video_label.setPixmap(pixmap)  # ← MUESTRA
```

### 3. Detección Periódica (Timer cada 2s)
```python
def _trigger_detection(self):
    self.detection_worker.detect_frame(self.last_raw_frame)  # ← USA last_raw_frame
```

### 4. Worker de Detección (Thread separado)
```python
def run(self):
    frame = self.frame  # ← COPIA DEL FRAME
    # Convierte uint16 → uint8
    # Convierte a grayscale
    result = self.scorer.assess_image(frame)  # ← PROCESA
    self.detection_done.emit(probability_map, objects, time_ms, frame)  # ← EMITE
```

### 5. Callback de Detección
```python
def _on_detection_done(self, saliency_map, objects, time_ms, frame_used):
    self.saliency_map = saliency_map  # ← GUARDA MAPA
    self.detected_objects = objects
    self._update_colormap_cache()  # ← PRE-CALCULA HEATMAP
```

---

## ❌ PROBLEMA IDENTIFICADO

### Timeline del Bug:
```
T=0.0s: Frame A llega → se muestra Frame A
T=0.1s: Frame B llega → se muestra Frame B  
T=0.2s: Frame C llega → se muestra Frame C
...
T=2.0s: Timer dispara → detect_frame(last_raw_frame) = Frame X
        Worker copia Frame X y comienza detección
T=2.5s: Frame Y llega → se muestra Frame Y (DIFERENTE a X)
T=2.6s: Worker termina → emite saliency_map de Frame X
        _on_detection_done() guarda saliency_map
T=2.7s: Frame Z llega → _draw_overlay aplica saliency_map de Frame X sobre Frame Z
        ↑↑↑ AQUÍ ESTÁ EL BUG ↑↑↑
```

### Causa Raíz:
1. La cámara sigue enviando frames mientras el worker procesa
2. El saliency_map se genera para Frame X
3. Pero se aplica sobre Frame Y, Z, etc. que son DIFERENTES

---

## 📊 ARCHIVOS INVOLUCRADOS

| Archivo | Función | Problema |
|---------|---------|----------|
| `camera_window.py` | `update_frame()` | Aplica overlay sobre frame actual, no sobre frame de detección |
| `camera_window.py` | `_trigger_detection()` | Envía `last_raw_frame` que puede cambiar |
| `camera_window.py` | `_draw_overlay()` | Usa `saliency_map` de frame anterior |
| `camera_window.py` | `DetectionWorker.run()` | Procesa frame correcto pero overlay no coincide |

---

## ✅ SOLUCIÓN PROPUESTA

### Opción A: Congelar frame durante detección
```python
def _on_detection_done(self, saliency_map, objects, time_ms, frame_used):
    self.saliency_map = saliency_map
    self.detected_objects = objects
    self.detection_frame = frame_used  # ← GUARDAR FRAME DETECTADO
    self._update_colormap_cache()

def _draw_overlay(self, frame):
    # MOSTRAR EL FRAME DE DETECCIÓN, NO EL FRAME ACTUAL
    if self.detection_frame is not None:
        frame_to_show = self.detection_frame
    else:
        frame_to_show = frame
```

### Opción B: Pausar cámara durante detección (NO RECOMENDADO)

### Opción C: Solo mostrar boxes sin heatmap
El heatmap cambia mucho entre frames, los boxes son más estables.

---

## 🔧 CÓDIGO A MODIFICAR

### 1. `_draw_overlay` debe usar `detection_frame`
```python
def _draw_overlay(self, frame):
    # Si hay detección, mostrar el frame detectado con overlay
    # Si no, mostrar frame actual sin overlay
    if self.detection_frame is not None and self.saliency_colormap is not None:
        base_frame = self.detection_frame  # ← FRAME DE DETECCIÓN
    else:
        base_frame = frame
```

### 2. O alternativamente: Solo boxes, sin heatmap
```python
def _draw_overlay(self, frame):
    # Heatmap desactivado para frames en movimiento
    # Solo dibujar boxes sobre frame actual
```

---

## 📝 PRÓXIMOS PASOS

1. [ ] Decidir: ¿Mostrar frame congelado con overlay o frame vivo con solo boxes?
2. [ ] Implementar solución elegida
3. [ ] Probar sincronización
4. [ ] Verificar que overlay coincide con frame

---

## 🎯 COMPARACIÓN CON ImgAnalysisTab

En `ImgAnalysisTab`, el flujo es:
```python
def _analyze_current(self):
    img = self._current_image  # ← IMAGEN FIJA
    result = self.scorer.assess_image(img)  # ← PROCESA
    self._calculate_layers(img, result)  # ← USA MISMA IMAGEN
    self._refresh_view()  # ← MUESTRA MISMA IMAGEN CON OVERLAY
```

**La imagen NUNCA cambia durante el proceso.**

En `CameraViewWindow`, la imagen cambia 30 veces por segundo mientras se procesa.

---

---

## 🔧 CAMBIOS IMPLEMENTADOS (2025-12-12 15:38)

### 1. DetectionWorker emite frame ORIGINAL
```python
def run(self):
    original_frame = self.frame.copy()  # ← GUARDAR ORIGINAL (uint16)
    
    # Convertir para modelo
    frame_for_model = ...  # grayscale uint8
    result = self.scorer.assess_image(frame_for_model)
    
    # Emitir frame ORIGINAL
    self.detection_done.emit(probability_map, objects, t_ms, original_frame)
```

### 2. update_frame muestra frame de detección cuando hay overlay
```python
def update_frame(self, q_image, raw_frame=None):
    if has_overlay and self.detection_frame is not None:
        # MOSTRAR FRAME DE DETECCIÓN (congelado) con overlay
        display_frame = self._draw_overlay(self.detection_frame)
    # Si no hay overlay, mostrar frame en vivo
```

### 3. Indicador de modo en UI
- `🔒 DETECT`: Mostrando frame congelado con overlay
- `🎥 LIVE`: Mostrando feed en vivo

---

---

## 🔧 FIX ADICIONAL (2025-12-12 15:43)

### Bug Encontrado
`_update_colormap_cache()` usaba `last_raw_frame.shape` (frame MÁS RECIENTE de la cámara) para hacer resize del saliency_map, pero el saliency_map corresponde a `detection_frame` (frame DIFERENTE).

### Código Incorrecto
```python
def _update_colormap_cache(self):
    if self.last_raw_frame is not None:
        h, w = self.last_raw_frame.shape[:2]  # ← FRAME INCORRECTO
```

### Código Corregido
```python
def _update_colormap_cache(self):
    if self.detection_frame is not None:
        h, w = self.detection_frame.shape[:2]  # ← FRAME CORRECTO
```

### Flujo Correcto Ahora
```
DetectionWorker.run()
    │
    ├── original_frame = self.frame.copy()  # Frame de cámara (uint16)
    │
    ├── frame_for_model = convertir a grayscale uint8
    │
    ├── result = scorer.assess_image(frame_for_model)
    │
    └── emit(saliency_map, objects, time_ms, original_frame)
            │
            └── _on_detection_done()
                    │
                    ├── self.detection_frame = original_frame
                    │
                    └── _update_colormap_cache()
                            │
                            └── resize saliency_map a detection_frame.shape

update_frame():
    │
    └── Si hay overlay:
            │
            └── _draw_overlay(self.detection_frame)  # USA MISMO FRAME
```

**Estado:** CORREGIDO - Colormap ahora usa detection_frame.shape

---

## 🧹 LIMPIEZA COMPLETA (2025-12-12 15:48)

### Código Reescrito
Se reescribió `camera_window.py` completamente eliminando:
- Código redundante
- Variables no usadas (`detection_params`, `frame_size`, etc.)
- Métodos duplicados

### Cambio Clave en DetectionWorker
**ANTES** (incorrecto):
```python
# Pasaba GRAYSCALE al scorer
frame_for_model = cv2.cvtColor(..., cv2.COLOR_BGR2GRAY)
result = self.scorer.assess_image(frame_for_model)  # ← GRAYSCALE
```

**DESPUÉS** (correcto):
```python
# Pasa BGR al scorer (igual que ImgAnalysisTab)
if len(frame_uint8.shape) == 2:
    frame_bgr = cv2.cvtColor(frame_uint8, cv2.COLOR_GRAY2BGR)
result = self.scorer.assess_image(frame_bgr)  # ← BGR
```

### Flujo Simplificado
```
DetectionWorker.run():
    1. frame uint16 → uint8
    2. grayscale → BGR (para scorer)
    3. result = scorer.assess_image(frame_bgr)
    4. emit(prob_map, objects, time_ms, frame_bgr)  # ← MISMO frame

_on_detection_done():
    1. Guarda frame_bgr que se analizó
    2. Crea colormap del MISMO tamaño
    3. detection_result = {frame_bgr, prob_map, objects, colormap}

_create_overlay():
    1. Usa detection_result['frame_bgr']  # ← MISMO frame
    2. Aplica colormap sobre ese frame
    3. Dibuja boxes
```

### Backup
- Original guardado en: `camera_window_backup.py`

**Estado:** CÓDIGO LIMPIO - Probando sincronización

---

## 🔧 FIX SINCRONIZACIÓN DE FRAMES (2025-12-12 16:00)

### Bug Encontrado
El signal `new_frame_ready` emitía solo `q_image`, pero `camera_tab.py` accedía a `current_frame` después, el cual podía haber sido actualizado por otro frame.

### Código Anterior (INCORRECTO)
```python
# camera_worker.py
new_frame_ready = pyqtSignal(object)  # Solo QImage
self.new_frame_ready.emit(q_image)

# camera_tab.py
def on_camera_frame(self, q_image):
    raw_frame = self.camera_worker.current_frame  # ← PUEDE HABER CAMBIADO!
```

### Código Corregido
```python
# camera_worker.py
new_frame_ready = pyqtSignal(object, object)  # QImage, raw_frame
self.new_frame_ready.emit(q_image, raw_frame)  # ← SINCRONIZADOS

# camera_tab.py
def on_camera_frame(self, q_image, raw_frame=None):
    self.camera_view_window.update_frame(q_image, raw_frame)  # ← MISMO FRAME
```

### Debug Agregado
Se guardan archivos en `C:/CapturasCamara/`:
- `debug_frame_analyzed.png` - Frame que se analizó
- `debug_prob_map.png` - Mapa de probabilidad generado

**Estado:** SINCRONIZACIÓN CORREGIDA - Probando

---

## 🧹 LIMPIEZA Y MEJORAS (2025-12-12 16:20)

### Cambios en camera_window.py

1. **Overlay simplificado:**
   - Eliminado colormap/heatmap que causaba fondo negro
   - Ahora muestra solo:
     - Contornos de saliencia (cyan)
     - ROI rectangulares (rojo)

2. **Controles duplicados eliminados:**
   - Removidos spinboxes de Área y Umbral (controlados desde CameraTab)
   - Renombrado "Mapa" → "Contornos"

3. **Código DEBUG eliminado:**
   - Removidos cv2.imwrite de debug
   - Removidos logs de debug innecesarios

4. **API pública actualizada:**
   - `_trigger_detection()` → `trigger_detection()` (público)
   - `set_detection_params(min_area, max_area, threshold)` simplificado

### Nuevo Servicio: MultiObjectAutofocusService

Creado en `src/services/multi_object_autofocus.py`:
- Detecta objetos con U2-Net
- Z-scan: 50% → 0% → 50% → 100%
- Encuentra mejor score por objeto
- Captura imagen en mejor posición
- Se ejecuta SOLO con trigger de adquisición

**Estado:** IMPLEMENTADO - Pendiente integración con CameraTab

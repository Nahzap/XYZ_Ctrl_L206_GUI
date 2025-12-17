# Auditoría del Sistema de Autofoco - 2025-12-16

## Estado Actual: ✅ VERIFICADO

El sistema de autofoco está correctamente implementado para usar la máscara U2-Net.

---

## 1. Flujo de Datos del Contorno/Máscara

### 1.1 Detección (SmartFocusScorer)
```
U2-Net detecta objeto → genera máscara de probabilidad
                      → extrae contorno con cv2.findContours()
                      → crea ObjectInfo con contour
```

**Archivo:** `src/core/autofocus/smart_focus_scorer.py`
- `assess_image()` → detecta objetos y genera `ObjectInfo` con `contour`

### 1.2 Filtrado (MicroscopyService)
```
MicroscopyService recibe ObjectInfo
                → filtra por área, circularidad, aspect_ratio
                → selecciona largest_object
                → pasa a AutofocusService.start_autofocus([largest_object])
```

**Archivo:** `src/core/services/microscopy_service.py`
- Línea 466: `self._autofocus_service.start_autofocus([largest_object])`

### 1.3 Autofoco (AutofocusService)
```
AutofocusService recibe ObjectInfo
                → extrae bbox y contour
                → en cada posición Z:
                    → _get_stable_score(bbox, contour)
                    → _calculate_sharpness(frame, bbox, contour)
                        → crea máscara binaria del contorno
                        → calcula métricas SOLO sobre píxeles de la máscara
```

**Archivo:** `src/core/services/autofocus_service.py`
- Línea 182: `contour = getattr(obj, 'contour', None)`
- Línea 211: `score = self._get_stable_score(bbox, contour, n_samples=2)`
- Línea 364: `score = self._calculate_sharpness(frame, bbox, contour)`

---

## 2. Cálculo de Sharpness sobre Máscara

### 2.1 Método `_calculate_sharpness` (líneas 377-458)

```python
def _calculate_sharpness(self, frame, bbox, contour=None):
    # 1. Extraer ROI del bbox
    roi = frame[y:y+h, x:x+w]
    
    # 2. Convertir a grayscale
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    
    # 3. Crear máscara del contorno U2-Net
    if contour is not None:
        mask = np.zeros((h, w), dtype=np.uint8)
        contour_shifted = contour.copy()
        contour_shifted[:, :, 0] -= x  # Ajustar a coordenadas ROI
        contour_shifted[:, :, 1] -= y
        cv2.drawContours(mask, [contour_shifted], -1, 255, -1)
    
    # 4. Calcular métricas
    laplacian = cv2.Laplacian(gray, cv2.CV_64F, ksize=5)
    gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    gradient_mag = gx**2 + gy**2
    
    # 5. APLICAR MÁSCARA - Solo píxeles del objeto
    if mask is not None and np.count_nonzero(mask) > 0:
        lap_values = laplacian[mask > 0]      # ✅ Solo máscara
        grad_values = gradient_mag[mask > 0]  # ✅ Solo máscara
        gray_values = gray[mask > 0]          # ✅ Solo máscara
        
        lap_var = lap_values.var()
        tenengrad = grad_values.mean()
        norm_var = gray_values.var() / gray_values.mean()
```

### 2.2 Métricas Utilizadas
| Métrica | Peso | Descripción |
|---------|------|-------------|
| Laplacian Variance | 25% | Sensible a bordes/detalles |
| Tenengrad (Sobel²) | 50% | Gradiente de intensidad |
| Normalized Variance | 25% | Contraste normalizado |

**Fórmula:** `S = 0.25*lap_var + 0.50*tenengrad + 0.25*norm_var`

---

## 3. Flujo de Captura de Imagen

### 3.1 Problema Anterior (CORREGIDO)
```
❌ ANTES:
1. Autofoco encuentra BPoF
2. Captura frame
3. Mueve a Z alternativo
4. Mueve a Z=50 (centro)  ← DESENFOCABA
5. MicroscopyService captura frame actual ← DESENFOCADO
```

### 3.2 Flujo Actual (CORRECTO)
```
✅ AHORA:
1. Autofoco encuentra BPoF (Z óptimo)
2. Captura frame en BPoF → guarda en result.frame
3. Mueve a Z alternativo (+10µm)
4. Captura frame alternativo → guarda en result.frame_alt
5. Vuelve a BPoF
6. Emite scan_complete con results
7. MicroscopyService usa result.frame (YA CAPTURADO)
8. Guarda imagen enfocada
```

**Archivos modificados:**
- `autofocus_service.py`: Eliminado movimiento a Z=50 después de autofoco
- `microscopy_service.py`: Usa `result.frame` en lugar de capturar nuevo frame

---

## 4. Archivos Clave del Sistema de Autofoco

| Archivo | Responsabilidad |
|---------|-----------------|
| `src/core/services/autofocus_service.py` | Escaneo Z, cálculo sharpness con máscara |
| `src/core/services/microscopy_service.py` | Orquestación, guardado de frames |
| `src/core/autofocus/smart_focus_scorer.py` | Detección U2-Net, generación de contornos |
| `src/core/models/focus_result.py` | Estructuras de datos (ObjectInfo, AutofocusResult) |
| `src/hardware/cfocus/cfocus_controller.py` | Control del piezo C-Focus |

---

## 5. Parámetros de Configuración

```python
# autofocus_service.py
z_step_coarse = 5.0      # µm - paso grueso
z_step_fine = 1.0        # µm - paso fino (refinamiento)
settle_time = 0.10       # segundos - estabilización entre pasos
capture_settle_time = 0.50  # segundos - estabilización para captura (500ms)
```

---

## 6. Señales PyQt del AutofocusService

| Señal | Parámetros | Descripción |
|-------|------------|-------------|
| `scan_started` | (obj_index, total) | Inicio de escaneo de objeto |
| `z_changed` | (z, score, roi_frame) | Progreso en cada posición Z |
| `object_focused` | (obj_index, z_optimal, score) | Foco encontrado |
| `scan_complete` | (results: List[FocusResult]) | Proceso completado |
| `error_occurred` | (message: str) | Error durante autofoco |

---

## 7. Verificación de Uso de Máscara

### Log esperado cuando funciona correctamente:
```
[Autofocus] S=125.3 (lap=45.2, ten=180.5, nv=12.1, px=30260)
```

Donde:
- `px=30260` indica que se calculó sobre ~30k píxeles de la máscara
- Si `px` es igual al tamaño del ROI completo, la máscara NO se está aplicando

### Verificación en código:
```python
# Línea 444 de autofocus_service.py
n_pixels = len(lap_values)  # Debe ser < que h*w del ROI
```

---

## 8. Imágenes Guardadas

Por cada punto de microscopía con autofoco:
- `{clase}_{index:04d}.png` → Frame en BPoF (mejor enfoque)
- `{clase}_{index:04d}_alt.png` → Frame alternativo (±10µm offset)

---

## 9. Conclusión

✅ **El sistema está correctamente implementado:**
1. El contorno U2-Net se extrae y pasa al autofoco
2. El cálculo de sharpness usa la máscara del contorno
3. Las métricas se calculan SOLO sobre los píxeles del objeto
4. El frame se captura en el momento correcto (BPoF)
5. Se guarda el frame ya capturado, no uno nuevo

---

## 10. Cambios Realizados (2025-12-16)

1. **`_calculate_sharpness`**: Agregado parámetro `contour` y lógica de máscara
2. **`_get_stable_score`**: Agregado parámetro `contour` para pasarlo al cálculo
3. **`_scan_single_object`**: Extrae `contour` del objeto y lo pasa a todas las funciones
4. **`handle_autofocus_complete`**: Usa `result.frame` en lugar de capturar nuevo frame
5. **Eliminado**: Movimiento a Z=50 después del autofoco que causaba desenfoque
6. **Agregado**: Retorno a BPoF después de captura alternativa

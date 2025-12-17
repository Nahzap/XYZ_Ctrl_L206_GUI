# Changelog - 2025-12-16

## Mejoras en Sistema de Autofoco

### 🔧 Correcciones Críticas

#### 1. Imágenes Desenfocadas (CORREGIDO)
**Problema:** Las imágenes se guardaban desenfocadas porque el sistema movía el eje Z después de encontrar el mejor foco.

**Solución:**
- El frame ahora se captura **durante** el autofoco cuando está en BPoF
- Se eliminó el movimiento a posición central (Z=50µm) que causaba desenfoque
- `MicroscopyService` usa el frame ya capturado (`result.frame`) en lugar de capturar uno nuevo

**Archivos modificados:**
- `src/core/services/autofocus_service.py`
- `src/core/services/microscopy_service.py`
- `src/main.py`

#### 2. Cálculo de Sharpness sobre Máscara U2-Net
**Problema:** El índice de sharpness se calculaba sobre todo el bbox rectangular, incluyendo fondo.

**Solución:**
- `_calculate_sharpness()` ahora recibe el `contour` del objeto
- Crea una máscara binaria del contorno
- Calcula Laplacian, Tenengrad y Normalized Variance **solo sobre los píxeles de la máscara**

**Código clave:**
```python
if mask is not None and np.count_nonzero(mask) > 0:
    lap_values = laplacian[mask > 0]      # Solo máscara
    grad_values = gradient_mag[mask > 0]  # Solo máscara
    gray_values = gray[mask > 0]          # Solo máscara
```

#### 3. Botones de Control Durante Microscopía
**Problema:** Los botones "No registrar ROI" y "Pausar" estaban deshabilitados durante la microscopía.

**Solución:**
- `CameraTab._start_microscopy()` ahora llama a `camera_view_window.set_microscopy_active(True)`
- `CameraTab._stop_microscopy()` llama a `camera_view_window.set_microscopy_active(False)`

---

### 📁 Archivos Modificados

| Archivo | Cambios |
|---------|---------|
| `src/core/services/autofocus_service.py` | Sharpness con máscara, eliminado movimiento post-autofoco |
| `src/core/services/microscopy_service.py` | Usa frame del resultado, métodos de guardado |
| `src/main.py` | Pasa resultados a handle_autofocus_complete |
| `src/gui/tabs/camera_tab.py` | Habilita botones durante microscopía |

---

### 📊 Flujo de Autofoco Actualizado

```
1. MicroscopyService detecta objetos con U2-Net
2. Filtra por área, circularidad, aspect_ratio
3. Selecciona objeto más grande
4. AutofocusService.start_autofocus([objeto])
   ├── Extrae bbox y contour del objeto
   ├── Escaneo grueso Z=0 → Z_max (paso 5µm)
   │   └── En cada Z: _get_stable_score(bbox, contour)
   │       └── _calculate_sharpness(frame, bbox, contour)
   │           └── Calcula métricas SOLO sobre máscara
   ├── Encuentra pico (máximo S)
   ├── Refinamiento ±5µm (paso 1µm)
   ├── Captura frame en BPoF → result.frame
   ├── Mueve a Z alternativo (+10µm)
   ├── Captura frame alternativo → result.frame_alt
   └── Vuelve a BPoF
5. Emite scan_complete(results)
6. MicroscopyService.handle_autofocus_complete(results)
   ├── Guarda result.frame como {clase}_{index}.png
   └── Guarda result.frame_alt como {clase}_{index}_alt.png
7. Avanza al siguiente punto
```

---

### 🧪 Verificación

Para verificar que la máscara se está usando, buscar en el log:
```
[Autofocus] S=125.3 (lap=45.2, ten=180.5, nv=12.1, px=30260)
```

- `px` debe ser menor que el área total del ROI
- Si `px` es igual al área del ROI, la máscara NO se está aplicando

---

### 📝 Documentación Actualizada

- `docs/AUTOFOCUS_AUDIT_2025-12-16.md` - Auditoría completa del sistema
- `docs/CHANGELOG_2025-12-16.md` - Este archivo

# Instrucciones para Commit a GitHub - 2025-12-16

## Archivos Modificados para Commit

### Archivos de Código (Core)
```
src/core/services/autofocus_service.py
src/core/services/microscopy_service.py
src/main.py
src/gui/tabs/camera_tab.py
```

### Documentación
```
docs/AUTOFOCUS_AUDIT_2025-12-16.md
docs/CHANGELOG_2025-12-16.md
docs/GIT_COMMIT_INSTRUCTIONS.md
```

---

## Comandos Git

### 1. Verificar estado
```bash
cd c:\Users\askna\PycharmProjects\XYZ_Ctrl_L206_GUI
git status
```

### 2. Agregar archivos modificados
```bash
# Archivos de código
git add src/core/services/autofocus_service.py
git add src/core/services/microscopy_service.py
git add src/main.py
git add src/gui/tabs/camera_tab.py

# Documentación
git add docs/AUTOFOCUS_AUDIT_2025-12-16.md
git add docs/CHANGELOG_2025-12-16.md
git add docs/GIT_COMMIT_INSTRUCTIONS.md
```

### 3. Commit con mensaje descriptivo
```bash
git commit -m "fix(autofocus): Corregir desenfoque y usar máscara U2-Net para sharpness

CAMBIOS CRÍTICOS:
- Corregido: Imágenes desenfocadas - ahora usa frame capturado en BPoF
- Corregido: Sharpness se calcula SOLO sobre máscara del objeto U2-Net
- Agregado: Botones de control habilitados durante microscopía
- Agregado: Guardado de imagen alternativa (±10µm offset)

ARCHIVOS MODIFICADOS:
- autofocus_service.py: Sharpness con máscara, eliminado mov. post-autofoco
- microscopy_service.py: Usa frame del resultado, métodos de guardado
- main.py: Pasa resultados a handle_autofocus_complete
- camera_tab.py: Habilita botones durante microscopía

DOCUMENTACIÓN:
- AUTOFOCUS_AUDIT_2025-12-16.md: Auditoría completa del sistema
- CHANGELOG_2025-12-16.md: Registro de cambios"
```

### 4. Push a GitHub
```bash
git push origin main
```

---

## Resumen de Cambios por Archivo

### `autofocus_service.py`
- `_calculate_sharpness()`: Nuevo parámetro `contour`, crea máscara y calcula métricas solo sobre ella
- `_get_stable_score()`: Nuevo parámetro `contour` para pasarlo al cálculo
- `_scan_single_object()`: Extrae contorno del objeto y lo pasa a todas las funciones
- Eliminado: Movimiento a Z=50 después del autofoco

### `microscopy_service.py`
- `handle_autofocus_complete()`: Recibe `results` y usa `result.frame` directamente
- Nuevo: `_save_autofocus_frame()` - Guarda frame BPoF
- Nuevo: `_save_autofocus_frame_alt()` - Guarda frame alternativo

### `main.py`
- `_on_autofocus_complete()`: Pasa `results` a `handle_autofocus_complete(results)`

### `camera_tab.py`
- `_start_microscopy()`: Llama a `camera_view_window.set_microscopy_active(True)`
- `_stop_microscopy()`: Llama a `camera_view_window.set_microscopy_active(False)`

---

## Verificación Post-Commit

Después de hacer push, verificar en GitHub que:
1. Los archivos se subieron correctamente
2. El mensaje de commit es visible
3. No hay conflictos con otras ramas

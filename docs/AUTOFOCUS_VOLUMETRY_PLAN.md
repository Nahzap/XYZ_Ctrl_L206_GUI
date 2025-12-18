# Plan de Implementación: Métodos de Autoenfoque y Volumetría
## Fecha: 2025-12-17

---

## 1. Resumen Ejecutivo

Se implementarán **2 métodos de autoenfoque** con propósitos distintos:

| Método | Nombre | Uso | Trigger | Imágenes |
|--------|--------|-----|---------|----------|
| **1** | Volumetría Manual | Análisis detallado de 1 objeto | Botón "Capturar Imagen" | BPoF + X imágenes (arriba/abajo) |
| **2** | Trayectoria Rápida | Microscopía automatizada | Trayectoria automática | BPoF + 1 desenfocada |

---

## 2. Método 1: Volumetría Manual (NUEVO)

### 2.1 Objetivo
Capturar múltiples imágenes de un objeto detectado a diferentes planos Z para generar **volumetría 3D**.

### 2.2 Flujo de Trabajo

```
Usuario presiona "Capturar Imagen"
         │
         ▼
┌─────────────────────────────────────┐
│  1. Detectar objetos en frame       │
│     (SmartFocusScorer + U2Net)      │
└─────────────────┬───────────────────┘
                  │
                  ▼
┌─────────────────────────────────────┐
│  2. Seleccionar objeto más grande   │
│     dentro del rango de área        │
└─────────────────┬───────────────────┘
                  │
                  ▼
┌─────────────────────────────────────┐
│  3. Z-Scan: encontrar BPoF          │
│     (Best Point of Focus)           │
│     Rango: Z_actual ± z_range       │
└─────────────────┬───────────────────┘
                  │
                  ▼
┌─────────────────────────────────────┐
│  4. Determinar Z_min y Z_max        │
│     (límites donde score > umbral)  │
└─────────────────┬───────────────────┘
                  │
                  ▼
┌─────────────────────────────────────┐
│  5. Capturar X imágenes             │
│     distribuidas entre Z_min y Z_max│
│     incluyendo BPoF                 │
└─────────────────┬───────────────────┘
                  │
                  ▼
┌─────────────────────────────────────┐
│  6. Guardar JSON con metadatos:     │
│     - Z de cada imagen              │
│     - Score de cada imagen          │
│     - BPoF identificado             │
│     - Parámetros de detección       │
└─────────────────────────────────────┘
```

### 2.3 Parámetros de Configuración

| Parámetro | Descripción | Valor Default | UI Widget |
|-----------|-------------|---------------|-----------|
| `n_volumetry_images` | Número total de imágenes a capturar | 10 | SpinBox (3-50) |
| `volumetry_distribution` | Distribución de imágenes | "uniform" | ComboBox |
| `include_bpof` | Incluir imagen en BPoF exacto | True | CheckBox |
| `save_all_scores` | Guardar scores de todo el Z-scan | True | CheckBox |

### 2.4 Distribución de Imágenes

```
Ejemplo: n_images=7, Z_min=-50µm, Z_max=+50µm, BPoF=0µm

Distribución Uniforme:
  Z: -50  -33  -17   0   +17  +33  +50
      │    │    │   │    │    │    │
      ▼    ▼    ▼   ▼    ▼    ▼    ▼
     img  img  img BPoF  img  img  img
      1    2    3   4    5    6    7

Distribución Centrada (más densidad cerca del BPoF):
  Z: -50  -25  -10   0   +10  +25  +50
      │    │    │   │    │    │    │
      ▼    ▼    ▼   ▼    ▼    ▼    ▼
     img  img  img BPoF  img  img  img
```

### 2.5 Estructura de Salida

```
CapturaManual_20251217_174500/
├── objeto_001/
│   ├── volumetry_z-050um_score0.45.png
│   ├── volumetry_z-033um_score0.62.png
│   ├── volumetry_z-017um_score0.78.png
│   ├── volumetry_z+000um_score0.95_BPoF.png  ← Mejor enfoque
│   ├── volumetry_z+017um_score0.81.png
│   ├── volumetry_z+033um_score0.58.png
│   ├── volumetry_z+050um_score0.42.png
│   └── metadata.json
```

### 2.6 Estructura del JSON de Metadatos

```json
{
  "timestamp": "2025-12-17T17:45:00",
  "object_id": 1,
  "detection": {
    "centroid_x": 512,
    "centroid_y": 384,
    "area_pixels": 15420,
    "min_area_filter": 5000,
    "max_area_filter": 50000
  },
  "z_scan": {
    "z_start": -100.0,
    "z_end": 100.0,
    "z_step": 5.0,
    "n_steps": 41
  },
  "focus_analysis": {
    "z_min_detected": -50.0,
    "z_max_detected": 50.0,
    "z_bpof": 2.5,
    "score_bpof": 0.95,
    "score_threshold": 0.3
  },
  "volumetry": {
    "n_images": 7,
    "distribution": "uniform",
    "images": [
      {"filename": "volumetry_z-050um_score0.45.png", "z": -50.0, "score": 0.45, "is_bpof": false},
      {"filename": "volumetry_z-033um_score0.62.png", "z": -33.3, "score": 0.62, "is_bpof": false},
      {"filename": "volumetry_z-017um_score0.78.png", "z": -16.7, "score": 0.78, "is_bpof": false},
      {"filename": "volumetry_z+000um_score0.95_BPoF.png", "z": 2.5, "score": 0.95, "is_bpof": true},
      {"filename": "volumetry_z+017um_score0.81.png", "z": 16.7, "score": 0.81, "is_bpof": false},
      {"filename": "volumetry_z+033um_score0.58.png", "z": 33.3, "score": 0.58, "is_bpof": false},
      {"filename": "volumetry_z+050um_score0.42.png", "z": 50.0, "score": 0.42, "is_bpof": false}
    ]
  },
  "camera_settings": {
    "exposure_ms": 50.0,
    "bit_depth": 16,
    "format": "png"
  }
}
```

---

## 3. Método 2: Trayectoria Rápida (EXISTENTE - Optimizar)

### 3.1 Objetivo
Capturar imágenes de múltiples objetos en una trayectoria de forma eficiente.

### 3.2 Flujo Actual (mantener)

```
Para cada punto de trayectoria:
  1. Mover a posición XY
  2. Detectar objeto
  3. Z-scan → encontrar BPoF
  4. Capturar imagen en BPoF
  5. Capturar imagen desenfocada (opcional)
  6. Avanzar al siguiente punto
```

### 3.3 Imágenes por Punto

| Imagen | Z Position | Propósito |
|--------|------------|-----------|
| BPoF | Z óptimo | Imagen principal enfocada |
| Desenfocada | Z + offset | Para segmentación/contraste |

---

## 4. Selector de Método en UI

### 4.1 Ubicación
Sección "Captura de Imagen" en CameraTab

### 4.2 Diseño UI

```
┌─ Captura de Imagen ─────────────────────────────────────┐
│                                                          │
│  Carpeta: [C:\CapturasCamara        ] [📁 Explorar]     │
│                                                          │
│  Formato: [PNG ▼]  ☑ 16-bit                             │
│                                                          │
│  ┌─ Método de Captura ─────────────────────────────────┐│
│  │ ○ Captura Simple (1 imagen)                         ││
│  │ ● Volumetría (múltiples planos Z)                   ││
│  │                                                      ││
│  │   Imágenes: [10 ▲▼]  Distribución: [Uniforme ▼]    ││
│  │   ☑ Incluir BPoF exacto                             ││
│  │   ☑ Guardar JSON con metadatos                      ││
│  └──────────────────────────────────────────────────────┘│
│                                                          │
│  [📸 Capturar Imagen]  [🎯 Enfocar Objs]                │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

### 4.3 Restricciones

| Contexto | Método 1 (Volumetría) | Método 2 (Trayectoria) |
|----------|----------------------|------------------------|
| Botón "Capturar Imagen" | ✅ Disponible | ❌ No aplica |
| Microscopía Automatizada | ❌ Deshabilitado | ✅ Único disponible |

**Nota:** Al iniciar microscopía automatizada, el selector de método se deshabilita y se usa automáticamente el Método 2.

---

## 5. Archivos a Modificar

### 5.1 Nuevos Archivos
- `src/core/services/volumetry_service.py` - Lógica de captura volumétrica

### 5.2 Archivos a Modificar

| Archivo | Cambios |
|---------|---------|
| `camera_tab_ui_builder.py` | Agregar sección de método de captura |
| `camera_tab.py` | Mapear widgets, handler de volumetría |
| `autofocus_service.py` | Agregar método `scan_for_volumetry()` que retorna Z_min, Z_max, scores |

---

## 6. Orden de Implementación

### Fase 1: UI y Estructura ✅
1. ✅ Agregar widgets de selección de método en `camera_tab_ui_builder.py`
2. ✅ Mapear widgets en `camera_tab.py`
3. ✅ Crear `volumetry_service.py` con estructura básica

### Fase 2: Lógica de Volumetría ✅
4. ✅ Implementar Z-scan con detección de límites en `volumetry_service.py`
5. ✅ Implementar captura de múltiples imágenes en `volumetry_service.py`
6. ✅ Implementar generación de JSON de metadatos

### Fase 3: Integración ✅
7. ✅ Conectar botón "Capturar Imagen" con volumetría
8. ✅ Implementar restricciones (deshabilitar en trayectoria)
9. ☐ Testing y ajustes

---

## 7. Señales y Comunicación

```
┌─────────────┐     volumetry_requested      ┌──────────────────┐
│  CameraTab  │ ─────────────────────────────▶│ VolumetryService │
└─────────────┘                               └────────┬─────────┘
       ▲                                               │
       │                                               │
       │  volumetry_progress(current, total)           │
       │  volumetry_image_captured(z, score, path)     │
       │  volumetry_complete(json_path)                │
       └───────────────────────────────────────────────┘
```

---

## 8. Estimación de Tiempo

| Fase | Tarea | Tiempo Estimado |
|------|-------|-----------------|
| 1 | UI y Estructura | 30 min |
| 2 | Lógica de Volumetría | 45 min |
| 3 | Integración | 30 min |
| **Total** | | **~2 horas** |

---

## 9. Notas Adicionales

### 9.1 Consideraciones de Rendimiento
- El Z-scan completo ya se realiza para encontrar BPoF
- Reutilizar los scores del Z-scan para determinar Z_min y Z_max
- No repetir Z-scan para cada imagen de volumetría

### 9.2 Formato de Nombres de Archivo
```
volumetry_z{signo}{valor}um_score{score}.png
```
Ejemplos:
- `volumetry_z-050um_score0.45.png` (Z = -50µm)
- `volumetry_z+000um_score0.95_BPoF.png` (BPoF)
- `volumetry_z+025um_score0.72.png` (Z = +25µm)

### 9.3 Compatibilidad con 16-bit
- Las imágenes de volumetría respetarán el checkbox "16-bit"
- JSON incluirá `bit_depth` en metadatos

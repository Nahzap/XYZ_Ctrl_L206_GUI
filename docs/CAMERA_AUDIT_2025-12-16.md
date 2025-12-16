# 📷 Auditoría del Módulo de Cámara Thorlabs
**Fecha:** 2025-12-16  
**Estado:** ⚠️ Requiere refactorización  
**Prioridad:** Alta

---

## 📊 Resumen de Archivos

| Archivo | Líneas | Responsabilidad | Estado |
|---------|--------|-----------------|--------|
| `gui/tabs/camera_tab.py` | 1,425 | UI + Lógica mezclada | ⚠️ **Muy grande** |
| `gui/windows/camera_window.py` | 532 | Visualización + Detección | ✅ OK |
| `hardware/camera/camera_worker.py` | 378 | Thread de adquisición | ✅ OK |
| `core/services/camera_service.py` | 117 | Orquestador de CameraWorker | ✅ OK |
| `config/hardware_availability.py` | 59 | Disponibilidad SDK | ✅ OK |

**Total:** ~2,511 líneas relacionadas con cámara

---

## 🏗️ Arquitectura Actual

```
┌─────────────────────────────────────────────────────────────────┐
│                         CameraTab (UI)                          │
│                        1,425 líneas ⚠️                          │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ • Conexión/Desconexión                                   │   │
│  │ • Parámetros de cámara (exposición, FPS, buffer)        │   │
│  │ • Vista en vivo                                          │   │
│  │ • Captura de imágenes                                    │   │
│  │ • Microscopía automatizada                               │   │
│  │ • Detección U2-Net                                       │   │
│  │ • Autofoco multi-objeto                                  │   │
│  │ • Sincronización con TestTab                             │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                      CameraService                              │
│                       117 líneas ✅                              │
│  • Orquesta CameraWorker                                        │
│  • Expone señales de alto nivel                                 │
│  • Maneja conexión/desconexión                                  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                      CameraWorker (QThread)                     │
│                       378 líneas ✅                              │
│  • Adquisición de frames en thread separado                     │
│  • Manejo de exposición, FPS, buffer                            │
│  • Conversión a QImage                                          │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                   Thorlabs SDK (pylablib)                       │
│  • ThorlabsTLCamera                                             │
│  • list_cameras_tlcam()                                         │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🐛 Bug Corregido

### Error: `'Thorlabs' is not defined`

**Causa:** `camera_tab.py` importaba `THORLABS_AVAILABLE` pero no `Thorlabs`.

**Solución:**
```python
# Antes
from config.hardware_availability import THORLABS_AVAILABLE

# Después
from config.hardware_availability import THORLABS_AVAILABLE, Thorlabs
```

**Archivo:** `gui/tabs/camera_tab.py` línea 22

---

## 📋 Análisis de CameraTab (1,425 líneas)

### Secciones Identificadas

| Sección | Líneas Aprox. | Descripción |
|---------|---------------|-------------|
| UI Setup | ~300 | Creación de widgets y layouts |
| Conexión | ~100 | connect/disconnect/detect camera |
| Vista en vivo | ~150 | start/stop live view |
| Captura | ~100 | capture_image, save_image |
| Parámetros | ~100 | exposure, FPS, buffer handlers |
| Microscopía | ~300 | start/stop/execute microscopy |
| Detección | ~150 | U2-Net detection handlers |
| Autofoco | ~150 | Multi-object autofocus |
| Utilidades | ~75 | log_message, helpers |

### Problemas Identificados

1. **Mezcla de UI y lógica**: La pestaña contiene lógica de microscopía que debería estar en un servicio
2. **Métodos muy largos**: Algunos métodos superan 100 líneas
3. **Dependencias cruzadas**: Referencias a TestTab, DetectionService, AutofocusService
4. **Callbacks complejos**: Lógica de microscopía automatizada mezclada con UI

---

## 🎯 Plan de Refactorización Propuesto

### Fase 1: Extraer Microscopía a Servicio (Prioridad Alta)

Crear `MicroscopyService` (ya existe parcialmente) y mover:
- `start_microscopy()`
- `stop_microscopy()`
- `execute_microscopy_step()`
- `_microscopy_capture_and_detect()`
- `_microscopy_autofocus()`

**Resultado esperado:** CameraTab reduce ~300 líneas

### Fase 2: Extraer Detección a Servicio (Prioridad Media)

Usar `DetectionService` existente para:
- `on_detection_result()`
- `update_detection_overlay()`

**Resultado esperado:** CameraTab reduce ~100 líneas

### Fase 3: Simplificar UI (Prioridad Baja)

Dividir `_setup_ui()` en métodos más pequeños:
- `_create_connection_section()`
- `_create_parameters_section()`
- `_create_microscopy_section()`
- `_create_detection_section()`

---

## 📁 Estructura de Archivos Propuesta

```
src/
├── core/
│   └── services/
│       ├── camera_service.py      # ✅ Ya existe (117 líneas)
│       ├── microscopy_service.py  # ✅ Ya existe (613 líneas)
│       ├── detection_service.py   # ✅ Ya existe
│       └── autofocus_service.py   # ✅ Ya existe
├── gui/
│   ├── tabs/
│   │   └── camera_tab.py          # ⚠️ Reducir a ~500 líneas
│   └── windows/
│       └── camera_window.py       # ✅ OK (532 líneas)
└── hardware/
    └── camera/
        └── camera_worker.py       # ✅ OK (378 líneas)
```

---

## 🔧 Dependencias del Módulo

### Externas
- `pylablib` - SDK para cámaras Thorlabs
- `opencv-python` - Procesamiento de imágenes
- `numpy` - Arrays numéricos
- `PyQt5` - GUI

### Internas
- `config.hardware_availability` - Verificación de SDK
- `core.services.camera_service` - Orquestador
- `core.services.detection_service` - Detección U2-Net
- `core.services.autofocus_service` - Autofoco
- `core.services.microscopy_service` - Microscopía automatizada
- `hardware.camera.camera_worker` - Thread de adquisición

---

## ✅ Checklist de Funcionalidades

### Conexión
- [x] Detectar cámaras Thorlabs
- [x] Conectar cámara
- [x] Desconectar cámara
- [x] Mostrar info de cámara

### Adquisición
- [x] Vista en vivo
- [x] Ajuste de exposición
- [x] Ajuste de FPS
- [x] Ajuste de buffer
- [x] Captura de imagen

### Procesamiento
- [x] Detección U2-Net
- [x] Overlay de detección
- [x] Cálculo de nitidez

### Automatización
- [x] Microscopía automatizada
- [x] Autofoco multi-objeto
- [x] Sincronización con trayectoria

---

## 📈 Métricas de Calidad

| Métrica | Valor Actual | Objetivo |
|---------|--------------|----------|
| Líneas en CameraTab | 1,425 | < 600 |
| Métodos > 50 líneas | ~8 | < 3 |
| Dependencias directas | 12 | < 8 |
| Cobertura de tests | 0% | > 50% |

---

## 🚀 Próximos Pasos

1. **Inmediato**: ✅ Corregir importación de `Thorlabs` (HECHO)
2. **Corto plazo**: Mover lógica de microscopía a `MicroscopyService`
3. **Mediano plazo**: Simplificar callbacks y reducir acoplamiento
4. **Largo plazo**: Agregar tests unitarios

---

## 📝 Notas Técnicas

### Configuración del SDK Thorlabs
```python
# En config/hardware_availability.py
import pylablib as pll
pll.par["devices/dlls/thorlabs_tlcam"] = r"C:\Program Files\Thorlabs\ThorImageCAM\Bin"
from pylablib.devices import Thorlabs
```

### Flujo de Conexión
1. `CameraTab.connect_camera()` → Llama a `CameraService.connect_camera()`
2. `CameraService` → Crea `CameraWorker` si no existe
3. `CameraWorker.connect_camera()` → Usa `Thorlabs.ThorlabsTLCamera()`
4. Señal `connection_success` → Propaga a UI

### Flujo de Adquisición
1. `CameraWorker.start()` → Inicia thread
2. `CameraWorker.run()` → Loop de adquisición
3. Señal `new_frame_ready(QImage, raw_frame)` → Propaga a UI
4. `CameraViewWindow` → Muestra frame con overlay

---

*Generado automáticamente por Cascade AI - 2025-12-16*

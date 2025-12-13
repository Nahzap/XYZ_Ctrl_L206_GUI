# 📊 ESTADO DE ARQUITECTURA - Diciembre 2025

**Última actualización:** 2025-12-12  
**Autor:** Análisis automatizado  
**Propósito:** Resumen ejecutivo del estado actual y plan de mejora

---

## 🔍 RESUMEN EJECUTIVO

### Estado General: 🟡 REQUIERE REFACTORIZACIÓN

El proyecto ha completado exitosamente la **Fase 12** de modularización, pero las tabs han crecido absorbiendo lógica que debería estar en servicios separados.

| Métrica | Valor | Estado |
|---------|-------|--------|
| Líneas totales en tabs críticas | 4811 | 🔴 Alto |
| Líneas de lógica en UI | ~3500 | 🔴 Crítico |
| Servicios existentes | 2 | 🟡 Insuficiente |
| Servicios necesarios | 7 | - |

---

## 📁 ESTRUCTURA ACTUAL DEL PROYECTO

```
src/
├── main.py                    (964 líneas)  🟡
├── config/
│   ├── constants.py           ✅
│   ├── settings.py            ✅
│   └── env_setup.py           ✅
├── core/
│   ├── analysis/              ✅ TransferFunctionAnalyzer
│   ├── autofocus/             ✅ AutofocusController
│   ├── communication/         ✅ SerialHandler, MotorProtocol
│   ├── controllers/           ✅ HInfController
│   ├── detection/             ✅ U2NetDetector (Singleton)
│   ├── services/
│   │   ├── autofocus_service.py   ✅ (351 líneas)
│   │   └── detection_service.py   ✅ (143 líneas)
│   └── trajectory/            ✅ TrajectoryGenerator
├── gui/
│   ├── tabs/
│   │   ├── camera_tab.py      (1338 líneas) 🔴
│   │   ├── hinf_tab.py        (2141 líneas) 🔴
│   │   ├── test_tab.py        (1332 líneas) 🔴
│   │   ├── control_tab.py     (472 líneas)  🟡
│   │   ├── analysis_tab.py    (~400 líneas) 🟡
│   │   ├── recording_tab.py   (~150 líneas) 🟢
│   │   └── img_analysis_tab.py (~400 líneas) 🟡
│   ├── windows/               ✅
│   ├── widgets/               ✅
│   └── styles/                ✅
├── hardware/
│   ├── camera/                ✅ CameraWorker
│   └── cfocus/                ✅ CFocusController
└── data/                      ✅ DataRecorder
```

---

## 🔴 PROBLEMAS IDENTIFICADOS

### 1. Tabs con Lógica de Negocio (Anti-Pattern "Fat Tab")

| Tab | Líneas | UI | Lógica | Problema |
|-----|--------|-----|--------|----------|
| `hinf_tab.py` | 2141 | 14% | **86%** | Síntesis H∞, simulación, control RT |
| `camera_tab.py` | 1338 | 43% | **57%** | Conexión, captura, autofoco |
| `test_tab.py` | 1332 | 30% | **70%** | Control dual, trayectorias |

### 2. Acoplamiento con parent_gui

```python
# Ejemplo problemático en camera_tab.py:751
if self.parent_gui and self.parent_gui.autofocus_controller:
    self.parent_gui.autofocus_controller.set_pixel_threshold(...)
```

### 3. Procesamiento de Datos en UI

```python
# Ejemplo problemático en camera_tab.py:1076
if frame.dtype == np.uint16:
    frame_uint8 = (frame / frame_max * 255).astype(np.uint8)
```

---

## ✅ LO QUE FUNCIONA BIEN

1. **Estructura de carpetas**: Bien organizada
2. **Servicios existentes**: `AutofocusService` y `DetectionService` son buenos ejemplos
3. **Hardware aislado**: `CameraWorker`, `CFocusController` correctamente separados
4. **Singleton para modelos**: `U2NetDetector` carga una sola vez
5. **Comunicación serial**: `SerialHandler` bien modularizado

---

## 📋 PLAN DE MEJORA: FASE 13

### Objetivo
Separar lógica de negocio de las tabs creando servicios dedicados.

### Servicios a Crear

| Servicio | Prioridad | Líneas Est. | Origen |
|----------|-----------|-------------|--------|
| `CameraService` | 🔴 Alta | ~500 | camera_tab.py |
| `HInfService` | 🔴 Alta | ~800 | hinf_tab.py |
| `MicroscopyService` | 🟡 Media | ~300 | camera_tab + main |
| `TrajectoryService` | 🟡 Media | ~400 | test_tab.py |
| `DualControlService` | 🟡 Media | ~300 | test_tab.py |

### Resultado Esperado

| Componente | Antes | Después | Reducción |
|------------|-------|---------|-----------|
| camera_tab.py | 1338 | ~400 | -70% |
| hinf_tab.py | 2141 | ~400 | -81% |
| test_tab.py | 1332 | ~400 | -70% |
| main.py | 964 | ~700 | -27% |

---

## 🏗️ ARQUITECTURA OBJETIVO

### Patrón de Diseño: Service Layer

```
┌─────────────┐     Signals     ┌─────────────┐     Direct     ┌─────────────┐
│    Tab      │ ←─────────────→ │   Service   │ ─────────────→ │  Hardware   │
│  (Solo UI)  │                 │  (Lógica)   │                │  (Driver)   │
└─────────────┘                 └─────────────┘                └─────────────┘
     │                                │                              │
     │ ~400 líneas                    │ ~500 líneas                  │ ~400 líneas
     │ - Widgets                      │ - Lógica de negocio          │ - I/O
     │ - Layouts                      │ - Validación                 │ - Threads
     │ - Estilos                      │ - Coordinación               │ - Buffers
     │ - Eventos UI                   │ - Estado                     │
```

### Comunicación por Señales

```python
# Tab solo emite y recibe señales
class CameraTab(QWidget):
    def __init__(self, service: CameraService):
        self.service = service
        self.service.connected.connect(self._update_ui_connected)
        self.connect_btn.clicked.connect(self.service.connect)
    
    def _update_ui_connected(self, success, info):
        # Solo actualiza UI, sin lógica
        self.status_label.setText(info)
```

---

## 📈 PROGRESO HISTÓRICO

### Fases Completadas

| Fase | Descripción | Estado |
|------|-------------|--------|
| 1-3 | Configuración, estilos, serial | ✅ |
| 4 | Ventanas auxiliares | ✅ |
| 5 | Hardware cámara | ✅ |
| 6 | Grabación de datos | ✅ |
| 7 | Análisis de transferencia | ✅ |
| 8 | Controlador H∞ | ✅ |
| 9 | Trayectorias | ✅ |
| 10 | Pestañas GUI | ✅ |
| 11 | Modelos de datos | ✅ |
| 12 | Ventana principal | ✅ |
| **13** | **Servicios (separación lógica/UI)** | **🔄 PENDIENTE** |

### Reducción de main.py

```
Versión Original:  7142 líneas (Nov 2025)
Después Fase 12:    462 líneas (Nov 2025) - Documentado
Estado Actual:      964 líneas (Dic 2025) - Creció con nuevas features
```

---

## 🎯 PRÓXIMOS PASOS RECOMENDADOS

### Inmediato (Sprint 1)
1. Crear `CameraService` - Mover lógica de cámara
2. Refactorizar `camera_tab.py` para usar servicio

### Corto Plazo (Sprint 2)
3. Crear `HInfService` - Mover síntesis y control
4. Refactorizar `hinf_tab.py` para usar servicio

### Mediano Plazo (Sprint 3-4)
5. Crear servicios de trayectoria y control dual
6. Refactorizar `test_tab.py`
7. Crear `MicroscopyService`
8. Limpiar `main.py`

---

## 📚 DOCUMENTOS RELACIONADOS

| Documento | Propósito |
|-----------|-----------|
| `FASE_13_SERVICES_REFACTOR_PLAN.md` | Plan detallado de refactorización |
| `PLAN_MODULARIZACION.md` | Historia completa de modularización |
| `REFACTOR_PLAN.md` | Tracking de reducción de main.py |
| `FASE_10_TABS_PLAN.md` | Plan original de tabs |
| `FASE_12_MAINWINDOW_PLAN.md` | Plan de ventana principal |

---

## ✅ CRITERIOS DE ÉXITO FASE 13

- [ ] Cada tab < 500 líneas
- [ ] Sin lógica de hardware en UI
- [ ] Comunicación solo por señales PyQt
- [ ] Servicios testeables de forma aislada
- [ ] main.py < 800 líneas
- [ ] Sin acceso directo a parent_gui

---

**Documento de referencia para el equipo de desarrollo.**

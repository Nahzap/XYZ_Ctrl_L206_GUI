# Algoritmo de Autofoco Multi-Objeto

## Descripción
Sistema de autofoco que detecta múltiples objetos (granos de polen) y realiza un escaneo Z para cada uno, determinando el mejor punto de foco mediante análisis de score.

**IMPORTANTE:** El autofoco **SOLO** se ejecuta cuando:
1. El usuario activa explícitamente el trigger de adquisición en microscopía automatizada
2. El checkbox "Habilitar autofoco por objeto" está marcado en CameraTab
3. C-Focus está conectado

## Arquitectura de Servicios

```
src/core/services/
├── __init__.py
├── detection_service.py    # Detección U2-Net asíncrona (QThread)
└── autofocus_service.py    # Z-scan autofoco (QThread)
```

## Flujo del Algoritmo

```
TRIGGER DE ADQUISICIÓN ACTIVADO (Microscopía Automatizada)
    │
    ├── 1. Capturar frame actual
    │
    ├── 2. Detectar objetos con U2-Net (DetectionService)
    │       └── Obtener lista de centroides y bounding boxes
    │
    ├── 3. Para cada objeto detectado (AutofocusService):
    │       │
    │       ├── 3.1 Z-Scan desde 50% del rango:
    │       │       │
    │       │       ├── Fase 1: 50% → 0%
    │       │       │   └── Evaluar sharpness (Laplacian) en cada paso
    │       │       │
    │       │       └── Fase 2: 50% → 100%
    │       │           └── Evaluar sharpness en cada paso
    │       │
    │       ├── 3.2 Determinar posición Z con mejor score
    │       │
    │       ├── 3.3 Mover a posición Z óptima
    │       │
    │       └── 3.4 Capturar imagen con BPoF
    │
    └── 4. Continuar con siguiente punto de trayectoria
```

## Parámetros de Configuración

| Parámetro | Descripción | Valor Default |
|-----------|-------------|---------------|
| `z_range_um` | Rango total de escaneo Z (µm) | 100 |
| `z_step_um` | Paso de escaneo Z (µm) | 2 |
| `z_start_percent` | Posición inicial (%) | 50 |
| `score_threshold` | Umbral mínimo de score | 0.5 |
| `stabilization_ms` | Tiempo de estabilización (ms) | 50 |

## Integración con Microscopía Automatizada

El autofoco se ejecuta **SOLO** cuando:
1. El sistema de microscopía automatizada está activo
2. El trigger de adquisición se activa
3. El sistema ya se ha movido a la posición XY de destino

### Señal de Trigger
```python
# En camera_tab.py / microscopy automation
def on_acquisition_trigger(self, position):
    """Callback cuando el trigger de adquisición se activa."""
    if self.autofocus_enabled:
        self.autofocus_service.run_multi_object_autofocus(
            callback=self.on_autofocus_complete
        )
    else:
        self.capture_image()
```

## Arquitectura

```
┌─────────────────────────────────────────────────────────┐
│                    CameraTab                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ Microscopy   │  │ Autofocus    │  │ C-Focus      │  │
│  │ Automation   │──│ Service      │──│ Controller   │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
│         │                │                  │          │
│         ▼                ▼                  ▼          │
│  ┌──────────────────────────────────────────────────┐  │
│  │              SmartFocusScorer (U2-Net)           │  │
│  └──────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

## API del Servicio

```python
# src/core/services/autofocus_service.py
class AutofocusService(QThread):
    """Ejecuta en thread separado para no bloquear UI."""
    
    # Señales
    scan_started = pyqtSignal(int, int)  # object_index, total
    z_changed = pyqtSignal(float, float, np.ndarray)  # z, score, roi
    object_focused = pyqtSignal(int, float, float)  # idx, z_opt, score
    scan_complete = pyqtSignal(list)  # List[FocusResult]
    
    def start_autofocus(self, objects: List[DetectedObject]):
        """Inicia autofoco para lista de objetos detectados."""
        pass
```

## Resultado

```python
@dataclass
class FocusResult:
    object_index: int
    z_optimal: float
    focus_score: float
    bbox: Tuple[int, int, int, int]
    frame: Optional[np.ndarray]  # Frame capturado con BPoF
```

## Estado de Implementación

- [x] Documentación del algoritmo
- [x] AutofocusService con Z-scan 50%→0%→50%→100%
- [x] Integración con microscopía automatizada (main.py)
- [ ] Tests y validación

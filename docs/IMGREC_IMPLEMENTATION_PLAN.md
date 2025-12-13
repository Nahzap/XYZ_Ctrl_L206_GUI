# 🔬 Plan de Implementación: ImgRec - Sistema de Autoenfoque Inteligente

**Documento creado:** 2025-12-12  
**Última actualización:** 2025-12-12  
**Versión:** 1.0  
**Autor:** Sistema de Control L206 + C-Focus Piezo  
**Estado:** 📋 EN PLANIFICACIÓN

---

## 📋 Resumen Ejecutivo

### Objetivo Principal
Implementar un sistema de **autoenfoque inteligente** que utilice la cámara Thorlabs con el sistema móvil XY, aplicando el modelo **U2-Net** para detectar objetos salientes (granos de polen) y realizar autoenfoque individual mediante el piezo **C-Focus**, registrando imágenes de alta calidad para cada objeto detectado.

### Objetivos Secundarios
1. **Arquitectura eficiente:** Carga única del modelo U2-Net al inicio
2. **Visualización fluida:** Vista de cámara en tiempo real sin bloqueos
3. **Overlays informativos:** Mapas de saliencia y scores en vivo
4. **Desacoplamiento:** Separación clara entre UI, lógica y hardware

---

## 🔍 Análisis del Estado Actual

### ✅ Componentes Existentes

| Componente | Ubicación | Estado | Problema |
|------------|-----------|--------|----------|
| `SmartFocusScorer` | `core/autofocus/smart_focus_scorer.py` | ⚠️ Parcial | U2-Net no carga, usa fallback |
| `MultiObjectAutofocusController` | `core/autofocus/multi_object_autofocus.py` | ⚠️ Parcial | Bloqueante, sin visualización |
| `CFocusController` | `hardware/cfocus/cfocus_controller.py` | ✅ Funcional | - |
| `CameraWorker` | `hardware/camera/camera_worker.py` | ✅ Funcional | - |
| `CameraTab` | `gui/tabs/camera_tab.py` | ⚠️ Sobrecargada | Demasiadas responsabilidades |
| `ImgAnalysisTab` | `gui/tabs/img_analysis_tab.py` | ✅ Funcional | Separada, no integrada |

### ❌ Problemas Críticos Identificados

#### 1. **Modelo U2-Net NO se carga**
```python
# smart_focus_scorer.py - Línea 61-80
def load_model(self):
    if self.model is not None:
        return
    # ... código que NUNCA ejecuta carga real
    logger.info(f"[SmartFocusScorer] Modelo U2-Net cargado: {self.model_name}")
    # ↑ FALSO: Solo imprime mensaje, no carga modelo
```

**Consecuencia:** Sistema usa detección por contornos (fallback), no U2-Net.

#### 2. **Acoplamiento excesivo CameraTab ↔ MainWindow**
```python
# camera_tab.py - Línea 1211
def _connect_cfocus(self):
    if self.parent_gui:
        success = self.parent_gui.connect_cfocus()  # ← Dependencia directa
```

**Consecuencia:** Código difícil de mantener y probar.

#### 3. **Autofoco bloqueante**
```python
# multi_object_autofocus.py - Línea 159-220
# Z-scanning ejecuta en thread principal
# UI se congela durante ~2 segundos por objeto
```

**Consecuencia:** Usuario no ve progreso, no puede cancelar.

#### 4. **Sin visualización de saliencia en tiempo real**
- No hay overlay de mapas de probabilidad
- No se muestra ROI durante autofoco
- No hay indicador de score S en vivo

---

## 🎯 Arquitectura Propuesta

### Diagrama de Componentes

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           CAPA DE PRESENTACIÓN                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐     │
│  │   CameraTab     │    │  AutofocusPanel │    │  SaliencyView   │     │
│  │  (Vista Live)   │    │  (Controles AF) │    │  (Overlays)     │     │
│  └────────┬────────┘    └────────┬────────┘    └────────┬────────┘     │
│           │                      │                      │               │
│           └──────────────────────┼──────────────────────┘               │
│                                  │                                      │
│                          [Señales PyQt]                                 │
│                                  │                                      │
└──────────────────────────────────┼──────────────────────────────────────┘
                                   │
┌──────────────────────────────────┼──────────────────────────────────────┐
│                           CAPA DE SERVICIOS                             │
├──────────────────────────────────┼──────────────────────────────────────┤
│                                  │                                      │
│  ┌───────────────────────────────┴───────────────────────────────┐     │
│  │                    MicroscopyService                          │     │
│  │  - Coordina flujo de microscopía                              │     │
│  │  - Gestiona estados (IDLE, MOVING, DETECTING, FOCUSING)       │     │
│  │  - Emite señales de progreso                                  │     │
│  └───────────────────────────────┬───────────────────────────────┘     │
│                                  │                                      │
│  ┌─────────────────┐    ┌────────┴────────┐    ┌─────────────────┐     │
│  │ DetectionService│    │ AutofocusService│    │ CaptureService  │     │
│  │ (U2-Net)        │    │ (Z-Scanning)    │    │ (Guardar imgs)  │     │
│  └────────┬────────┘    └────────┬────────┘    └────────┬────────┘     │
│           │                      │                      │               │
└───────────┼──────────────────────┼──────────────────────┼───────────────┘
            │                      │                      │
┌───────────┼──────────────────────┼──────────────────────┼───────────────┐
│           │              CAPA DE HARDWARE               │               │
├───────────┼──────────────────────┼──────────────────────┼───────────────┤
│           │                      │                      │               │
│  ┌────────┴────────┐    ┌────────┴────────┐    ┌───────┴────────┐      │
│  │  CameraWorker   │    │ CFocusController│    │  MotorControl  │      │
│  │  (Thorlabs)     │    │  (Mad City Labs)│    │  (Arduino XY)  │      │
│  └─────────────────┘    └─────────────────┘    └────────────────┘      │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### Principios de Diseño

1. **Carga única de U2-Net:** Singleton en `DetectionService`
2. **Procesamiento asíncrono:** Workers en threads separados
3. **Comunicación por señales:** Sin llamadas directas entre capas
4. **Visualización no-bloqueante:** Overlays en thread de renderizado

---

## 📦 Módulos a Implementar/Refactorizar

### FASE 1: Carga Correcta de U2-Net (CRÍTICO)

**Archivo:** `src/core/detection/u2net_detector.py` (NUEVO)

```python
class U2NetDetector:
    """
    Singleton para detección de objetos salientes con U2-Net.
    Carga el modelo UNA SOLA VEZ al inicio.
    """
    _instance = None
    _model = None
    _device = None
    
    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = cls()
            cls._load_model()
        return cls._instance
    
    @classmethod
    def _load_model(cls):
        """Carga U2-Net (u2netp para velocidad)."""
        import torch
        from models.u2net.model_def import U2NETP
        
        cls._device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        cls._model = U2NETP(3, 1)
        
        # Cargar pesos pre-entrenados
        weights_path = "models/u2net/u2netp.pth"
        cls._model.load_state_dict(torch.load(weights_path, map_location=cls._device))
        cls._model.to(cls._device)
        cls._model.eval()
        
        logger.info(f"[U2NetDetector] Modelo cargado en {cls._device}")
    
    def detect(self, image: np.ndarray) -> Tuple[np.ndarray, List[Dict]]:
        """
        Detecta objetos salientes.
        
        Returns:
            saliency_map: Mapa de probabilidades [0-1]
            objects: Lista de {bbox, area, probability, centroid}
        """
        # Preprocesar
        input_tensor = self._preprocess(image)
        
        # Inferencia
        with torch.no_grad():
            d1, *_ = self._model(input_tensor)
            saliency = torch.sigmoid(d1).squeeze().cpu().numpy()
        
        # Post-procesar: extraer objetos
        objects = self._extract_objects(saliency)
        
        return saliency, objects
```

**Tareas:**
- [ ] Crear archivo `u2net_detector.py`
- [ ] Implementar patrón Singleton
- [ ] Descargar pesos `u2netp.pth` (~4MB)
- [ ] Verificar carga en GPU/CPU
- [ ] Test unitario de detección

---

### FASE 2: Servicio de Detección Asíncrono

**Archivo:** `src/core/services/detection_service.py` (NUEVO)

```python
class DetectionWorker(QThread):
    """Worker para detección en background."""
    
    detection_complete = pyqtSignal(np.ndarray, list)  # saliency_map, objects
    progress_updated = pyqtSignal(str)  # mensaje de estado
    
    def __init__(self, detector: U2NetDetector):
        super().__init__()
        self.detector = detector
        self.frame_queue = Queue(maxsize=1)
        self.running = False
    
    def submit_frame(self, frame: np.ndarray):
        """Envía frame para detección (no bloqueante)."""
        try:
            self.frame_queue.put_nowait(frame)
        except Full:
            pass  # Descartar si hay frame pendiente
    
    def run(self):
        self.running = True
        while self.running:
            try:
                frame = self.frame_queue.get(timeout=0.1)
                saliency, objects = self.detector.detect(frame)
                self.detection_complete.emit(saliency, objects)
            except Empty:
                continue
```

**Tareas:**
- [ ] Crear `detection_service.py`
- [ ] Implementar cola de frames
- [ ] Señales para resultados
- [ ] Manejo de cancelación

---

### FASE 3: Servicio de Autofoco Asíncrono

**Archivo:** `src/core/services/autofocus_service.py` (NUEVO)

```python
class AutofocusWorker(QThread):
    """Worker para Z-scanning en background."""
    
    # Señales para UI
    z_position_changed = pyqtSignal(float, float)  # z_current, score
    scan_progress = pyqtSignal(int, int)  # current_step, total_steps
    focus_found = pyqtSignal(float, float, np.ndarray)  # z_optimal, score, focused_frame
    scan_complete = pyqtSignal(list)  # lista de FocusedCapture
    
    def __init__(self, cfocus: CFocusController, camera_callback):
        super().__init__()
        self.cfocus = cfocus
        self.get_frame = camera_callback
        self.objects_to_focus = []
        self.running = False
    
    def start_scan(self, objects: List[DetectedObject], config: dict):
        """Inicia Z-scanning para lista de objetos."""
        self.objects_to_focus = objects
        self.config = config
        self.start()
    
    def run(self):
        self.running = True
        captures = []
        
        for obj in self.objects_to_focus:
            if not self.running:
                break
            
            z_opt, score, frame = self._scan_single_object(obj)
            captures.append(FocusedCapture(obj, z_opt, score, frame))
        
        self.scan_complete.emit(captures)
    
    def _scan_single_object(self, obj: DetectedObject) -> Tuple[float, float, np.ndarray]:
        """Z-scanning para un objeto con emisión de progreso."""
        z_range = self.cfocus.get_z_range()
        z_step = self.config.get('z_step', 5.0)
        
        z_positions = np.arange(0, z_range, z_step)
        scores = []
        
        for i, z in enumerate(z_positions):
            if not self.running:
                break
            
            self.cfocus.move_z(z)
            time.sleep(0.05)  # Settle
            
            frame = self.get_frame()
            score = self._calculate_sharpness(frame, obj.bounding_box)
            scores.append(score)
            
            # Emitir progreso para UI
            self.z_position_changed.emit(z, score)
            self.scan_progress.emit(i + 1, len(z_positions))
        
        # Encontrar óptimo
        best_idx = np.argmax(scores)
        z_optimal = z_positions[best_idx]
        
        # Mover a posición óptima y capturar
        self.cfocus.move_z(z_optimal)
        time.sleep(0.05)
        final_frame = self.get_frame()
        final_score = scores[best_idx]
        
        self.focus_found.emit(z_optimal, final_score, final_frame)
        
        return z_optimal, final_score, final_frame
```

**Tareas:**
- [ ] Crear `autofocus_service.py`
- [ ] Implementar señales de progreso
- [ ] Permitir cancelación mid-scan
- [ ] Emitir frames para visualización

---

### FASE 4: Panel de Visualización con Overlays

**Archivo:** `src/gui/widgets/saliency_overlay.py` (NUEVO)

```python
class SaliencyOverlayWidget(QWidget):
    """
    Widget que superpone información de detección sobre la imagen de cámara.
    
    Muestra:
    - Mapa de saliencia (semi-transparente)
    - Bounding boxes de objetos detectados
    - Scores de cada objeto
    - Indicador de Z actual durante autofoco
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.current_frame = None
        self.saliency_map = None
        self.detected_objects = []
        self.current_z = 0.0
        self.current_score = 0.0
        self.show_saliency = True
        self.show_boxes = True
        self.show_scores = True
    
    def update_frame(self, frame: np.ndarray):
        """Actualiza frame base."""
        self.current_frame = frame
        self.update()
    
    def update_detection(self, saliency: np.ndarray, objects: list):
        """Actualiza resultados de detección."""
        self.saliency_map = saliency
        self.detected_objects = objects
        self.update()
    
    def update_autofocus_state(self, z: float, score: float, active_obj_idx: int = -1):
        """Actualiza estado de autofoco."""
        self.current_z = z
        self.current_score = score
        self.active_object = active_obj_idx
        self.update()
    
    def paintEvent(self, event):
        """Renderiza frame con overlays."""
        if self.current_frame is None:
            return
        
        painter = QPainter(self)
        
        # 1. Dibujar frame base
        self._draw_frame(painter)
        
        # 2. Overlay de saliencia (si habilitado)
        if self.show_saliency and self.saliency_map is not None:
            self._draw_saliency_overlay(painter)
        
        # 3. Bounding boxes
        if self.show_boxes:
            self._draw_bounding_boxes(painter)
        
        # 4. Scores
        if self.show_scores:
            self._draw_scores(painter)
        
        # 5. Indicador de autofoco
        if self.active_object >= 0:
            self._draw_autofocus_indicator(painter)
```

**Tareas:**
- [ ] Crear `saliency_overlay.py`
- [ ] Implementar renderizado eficiente
- [ ] Controles de visibilidad (checkboxes)
- [ ] Colores configurables

---

### FASE 5: Refactorizar CameraTab

**Cambios en:** `src/gui/tabs/camera_tab.py`

```python
class CameraTab(QWidget):
    """
    Pestaña de cámara SIMPLIFICADA.
    
    Responsabilidades:
    - Conexión/desconexión de cámara
    - Vista en vivo
    - Captura manual
    - Configuración de exposición/FPS
    
    NO incluye:
    - Controles de autofoco (movidos a AutofocusPanel)
    - Lógica de microscopía (movida a MicroscopyService)
    """
    
    # Señales (comunicación con servicios)
    frame_captured = pyqtSignal(np.ndarray)
    camera_connected = pyqtSignal(bool, str)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()
    
    def _setup_ui(self):
        """UI simplificada: solo controles de cámara."""
        layout = QVBoxLayout(self)
        
        # Grupo 1: Conexión
        # Grupo 2: Vista en vivo
        # Grupo 3: Captura manual
        # Grupo 4: Configuración
        
        # SIN: Controles de autofoco, microscopía, C-Focus
```

**Tareas:**
- [ ] Mover controles de autofoco a panel separado
- [ ] Eliminar dependencias a `parent_gui`
- [ ] Usar solo señales para comunicación
- [ ] Reducir a ~400 líneas

---

### FASE 6: Panel de Autofoco Independiente

**Archivo:** `src/gui/panels/autofocus_panel.py` (NUEVO)

```python
class AutofocusPanel(QWidget):
    """
    Panel de controles de autofoco.
    
    Puede integrarse en CameraTab o como widget flotante.
    """
    
    # Señales
    cfocus_connect_requested = pyqtSignal()
    cfocus_disconnect_requested = pyqtSignal()
    autofocus_start_requested = pyqtSignal(dict)  # config
    autofocus_stop_requested = pyqtSignal()
    params_changed = pyqtSignal(dict)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Conexión C-Focus
        self._create_connection_group()
        
        # Modo de escaneo
        self._create_scan_mode_group()
        
        # Parámetros de detección
        self._create_detection_params_group()
        
        # Parámetros de Z
        self._create_z_params_group()
        
        # Estado y progreso
        self._create_status_group()
    
    def update_status(self, connected: bool, z_position: float = 0.0):
        """Actualiza indicadores de estado."""
        pass
    
    def update_progress(self, current: int, total: int, score: float):
        """Actualiza barra de progreso durante autofoco."""
        pass
```

**Tareas:**
- [ ] Crear `autofocus_panel.py`
- [ ] Mover todos los controles de autofoco
- [ ] Agregar barra de progreso
- [ ] Agregar gráfica Z vs Score

---

## 📊 Flujo de Datos Propuesto

### Detección en Tiempo Real

```
┌─────────────┐     ┌──────────────┐     ┌─────────────────┐
│ CameraWorker│────▶│DetectionWorker│────▶│SaliencyOverlay  │
│ (30 FPS)    │     │ (U2-Net)     │     │ (Renderizado)   │
└─────────────┘     └──────────────┘     └─────────────────┘
      │                    │                     │
      │ frame_ready        │ detection_complete  │ paintEvent
      │                    │                     │
      ▼                    ▼                     ▼
   [Frame]           [Saliency, Objects]    [Frame + Overlays]
```

### Autofoco por Objeto

```
┌─────────────┐     ┌────────────────┐     ┌─────────────────┐
│ User clicks │────▶│AutofocusWorker │────▶│ SaliencyOverlay │
│ "Start AF"  │     │ (Z-scanning)   │     │ (Progreso)      │
└─────────────┘     └────────────────┘     └─────────────────┘
                           │
                           │ z_position_changed
                           │ scan_progress
                           │ focus_found
                           ▼
                    [Z, Score, Frame]
                           │
                           ▼
                    ┌─────────────────┐
                    │ CaptureService  │
                    │ (Guardar imagen)│
                    └─────────────────┘
```

---

## 🗓️ Cronograma de Implementación

### Sprint 1: Fundamentos (2-3 horas) ✅ COMPLETADO
- [x] **1.1** Crear `U2NetDetector` con carga real de modelo
- [x] **1.2** Descargar y verificar pesos `u2netp.pth`
- [x] **1.3** Test de detección con imagen estática

### Sprint 2: Servicios Asíncronos (3-4 horas) ✅ COMPLETADO
- [x] **2.1** Crear `DetectionService` con cola de frames
- [x] **2.2** Crear `AutofocusService` con señales de progreso
- [x] **2.3** Integrar con `CFocusController`

### Sprint 3: Visualización (2-3 horas) ✅ COMPLETADO
- [x] **3.1** Crear `SaliencyOverlayWidget`
- [x] **3.2** Implementar renderizado de mapas de probabilidad
- [x] **3.3** Agregar indicadores de score y Z

### Sprint 4: Integración UI (2-3 horas) ✅ COMPLETADO
- [x] **4.1** Crear estructura de archivos
- [x] **4.2** Integrar servicios en main.py
- [x] **4.3** Conectar señales y callbacks

### Sprint 5: Testing Final (1-2 horas) 🔄 EN PROGRESO
- [ ] **5.1** Verificar carga de U2-Net al inicio
- [ ] **5.2** Probar detección en tiempo real
- [ ] **5.3** Probar autofoco con C-Focus

**Total estimado:** 11-16 horas

---

## ✅ Criterios de Aceptación

### Funcionales
- [ ] U2-Net carga una sola vez al inicio de la aplicación
- [ ] Detección de objetos en tiempo real (>10 FPS)
- [ ] Mapa de saliencia visible como overlay
- [ ] Scores de objetos visibles en pantalla
- [ ] Z-scanning no bloquea la UI
- [ ] Progreso de autofoco visible durante escaneo
- [ ] Imágenes guardadas con enfoque óptimo

### No Funcionales
- [ ] Tiempo de carga de U2-Net < 5 segundos
- [ ] Latencia de detección < 100ms por frame
- [ ] Uso de memoria < 2GB adicionales
- [ ] UI responsive durante todo el proceso

---

## 📝 Notas de Implementación

### Dependencias Requeridas
```
torch>=1.9.0
torchvision>=0.10.0
opencv-python>=4.5.0
numpy>=1.20.0
PyQt5>=5.15.0
```

### Estructura de Archivos Final
```
src/
├── core/
│   ├── detection/
│   │   ├── __init__.py
│   │   └── u2net_detector.py      # NUEVO
│   ├── services/
│   │   ├── __init__.py
│   │   ├── detection_service.py   # NUEVO
│   │   ├── autofocus_service.py   # NUEVO
│   │   └── capture_service.py     # NUEVO
│   └── autofocus/
│       ├── smart_focus_scorer.py  # REFACTORIZAR
│       └── multi_object_autofocus.py  # DEPRECAR
├── gui/
│   ├── tabs/
│   │   └── camera_tab.py          # SIMPLIFICAR
│   ├── panels/
│   │   ├── __init__.py
│   │   └── autofocus_panel.py     # NUEVO
│   └── widgets/
│       ├── __init__.py
│       └── saliency_overlay.py    # NUEVO
└── models/
    └── u2net/
        ├── __init__.py
        ├── model_def.py           # EXISTENTE
        └── u2netp.pth             # DESCARGAR
```

---

## 🔄 Registro de Cambios

| Fecha | Versión | Cambios |
|-------|---------|---------|
| 2025-12-12 | 1.0 | Documento inicial creado |
| 2025-12-12 | 2.0 | **IMPLEMENTACIÓN COMPLETADA** - Sprints 1-4 |
| 2025-12-12 | 2.1 | **LIMPIEZA DE CÓDIGO** - Eliminado código redundante |
| 2025-12-12 | 2.2 | **OVERLAY EN VENTANA DE CÁMARA** - Detección integrada |
| 2025-12-12 | 2.3 | **DETECCIÓN EN HILO SEPARADO** - No bloquea UI |
| 2025-12-12 | 2.4 | **OPTIMIZACIÓN GPU** - Preprocesamiento en GPU + logs detallados |
| 2025-12-12 | 2.5 | **DETECCIÓN PERIÓDICA** - Timer cada N segundos, no continua |
| 2025-12-12 | 2.6 | **FIX CONGELAMIENTO** - Cache de colormap, no resize en cada frame |
| 2025-12-12 | 2.7 | **MEJORA LOGS** - Logs detallados + método _update_colormap_cache |
| 2025-12-12 | 2.8 | **PARÁMETROS EDITABLES** - min_area, max_area, threshold en ventana cámara |

---

## 🧹 Limpieza de Código (v2.1)

### Cambios en `main.py`
- **Líneas:** 935 → 909 (-26 líneas)
- **Eliminado:** Variable `autofocus_controller` (redundante)
- **Simplificado:** `initialize_autofocus()` ahora usa `AutofocusService`
- **Refactorizado:** `_microscopy_capture_with_autofocus()` usa `U2NetDetector` singleton
- **Agregado:** `_advance_microscopy_point()` para código más limpio

### Cambios en `camera_tab.py`
- **Actualizado:** `_test_detection()` usa `U2NetDetector` singleton
- **Agregado:** `_create_detection_visualization()` para visualización con saliencia
- **Eliminado:** Dependencia de `SmartFocusScorer` duplicado

### Código Eliminado
- `SmartFocusScorer` ya no se instancia en `initialize_autofocus()`
- `MultiObjectAutofocusController` reemplazado por `AutofocusService`
- Variable `self.autofocus_controller` eliminada

---

## 🎥 Overlay en Ventana de Cámara (v2.2)

### Cambios en `camera_window.py`
- **Checkbox** "🔍 Mostrar Detección U2-Net" para activar/desactivar overlay
- **Checkbox** "Saliencia" para mostrar/ocultar mapa de calor
- **Info label** muestra: objetos detectados, tiempo de inferencia, parámetros de área
- **Métodos nuevos:**
  - `set_detector()` - Configura el detector U2-Net singleton
  - `set_detection_params()` - Actualiza área min/max en tiempo real
  - `_apply_detection_overlay()` - Aplica detección y dibuja overlay
  - `_create_overlay()` - Dibuja saliencia + bounding boxes + labels

### Cambios en `camera_tab.py`
- **`open_camera_view()`** - Configura detector U2-Net al abrir ventana
- **`_update_detection_params()`** - Sincroniza spinboxes con ventana
- **`on_camera_frame()`** - Pasa frame raw para detección
- **`_test_detection()`** - Ahora hace toggle del checkbox en ventana existente
- **Spinboxes** conectados a `_update_detection_params()` para actualización en tiempo real

### Flujo de Uso
1. Conectar cámara
2. Abrir ventana de cámara (botón "Ver")
3. Iniciar vista en vivo
4. Activar checkbox "🔍 Mostrar Detección U2-Net"
5. Ajustar parámetros de área en la pestaña ImgRec
6. La imagen GUARDADA siempre es la ORIGINAL (sin overlays)

---

## 🧵 Detección en Hilo Separado (v2.3)

### Problema Resuelto
- **Antes:** Detección bloqueaba UI (788ms de congelamiento)
- **Ahora:** Detección en `DetectionWorker` (QThread) - UI fluida

### Nuevos Componentes

#### `DetectionWorker` (QThread)
```python
class DetectionWorker(QThread):
    detection_done = pyqtSignal(object, list, float)  # saliency, objects, time_ms
    
    def detect_frame(self, frame):
        """Encola frame para detección (no bloquea)."""
        
    def run(self):
        """Ejecuta detección en hilo separado."""
```

### Cambios en `CameraViewWindow`
- **Checkbox "🔍 Detección Continua"** - Activa detección automática
- **Checkbox "Overlay"** - Muestra/oculta bounding boxes
- **Checkbox "Saliencia"** - Muestra/oculta mapa de calor
- **Info label** - Muestra: objetos, tiempo, FPS equivalente

### Flujo de Detección Continua
1. Usuario activa "Detección Continua"
2. Cada frame se envía al `DetectionWorker` (si no está ocupado)
3. Worker ejecuta U2-Net en hilo separado
4. Resultados se emiten via `detection_done` signal
5. UI actualiza overlay con últimos resultados
6. Frame rate de detección = 1000ms / tiempo_inferencia

### Parámetros de Área (Corregidos)
- **Área mínima:** 1000 px (antes 100 - detectaba ruido)
- **Área máxima:** 500000 px (antes 50000 - muy restrictivo)

---

## 🚀 Optimización GPU (v2.4)

### Problema Identificado
- **300-600ms por detección** - Cuello de botella en preprocesamiento CPU
- **0 objetos detectados** - Umbral de saliencia muy alto (0.5)

### Optimizaciones Implementadas

#### 1. Preprocesamiento en GPU (`_preprocess_gpu`)
```python
# ANTES (CPU - lento):
image = cv2.resize(image, (320, 320))  # CPU
tensor = torch.from_numpy(image).to(device)  # CPU → GPU

# AHORA (GPU - rápido):
tensor = torch.from_numpy(image).to(device)  # CPU → GPU inmediato
tensor = F.interpolate(tensor, size=(320, 320))  # Resize en GPU
```

#### 2. Resize de salida en GPU
```python
# ANTES:
saliency = d0.squeeze().cpu().numpy()
saliency = cv2.resize(saliency, (w_orig, h_orig))  # CPU

# AHORA:
saliency_gpu = F.interpolate(saliency_gpu, size=(h_orig, w_orig))  # GPU
saliency = saliency_gpu.squeeze().cpu().numpy()  # Solo transferencia
```

#### 3. Logs Detallados de Tiempos
```
[U2Net] Total=XXms | Preproc=XXms | Infer=XXms | Resize=XXms | Extract=XXms | Objetos=N
```

### Parámetros Ajustados
| Parámetro | Antes | Ahora | Razón |
|-----------|-------|-------|-------|
| `saliency_threshold` | 0.5 | 0.3 | Más sensible a objetos |
| `min_area` | 100 | 500 | Evitar ruido |
| `max_area` | 50000 | 500000 | Detectar células grandes |

### Rendimiento Esperado
- **Preprocesamiento:** ~5-10ms (antes ~50-100ms)
- **Inferencia:** ~20-30ms (sin cambio - ya era GPU)
- **Resize:** ~5ms (antes ~20ms)
- **Total:** ~50-80ms (antes 300-600ms)

---

## ⏱️ Detección Periódica (v2.5)

### Problema Identificado
La detección continua (cada frame) es innecesaria y consume recursos:
- U2-Net toma ~300-500ms por frame
- 30 FPS de cámara = 30 detecciones/segundo imposible
- Solo necesitamos detectar cuando hay un trigger (trayectoria)

### Solución: Detección Periódica con QTimer

```
┌─────────────────────────────────────────────────────────┐
│                    ARQUITECTURA v2.5                     │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  CÁMARA (30 FPS)          DETECCIÓN (cada N seg)        │
│  ┌──────────────┐         ┌──────────────────┐          │
│  │ Frame 1      │         │                  │          │
│  │ Frame 2      │         │  QTimer (2s)     │          │
│  │ Frame 3      │ ───────▶│       ↓          │          │
│  │ ...          │         │  DetectionWorker │          │
│  │ Frame 60     │         │       ↓          │          │
│  └──────────────┘         │  Saliency Map    │          │
│         ↓                 └────────┬─────────┘          │
│  ┌──────────────┐                  │                    │
│  │ video_label  │◀─────────────────┘                    │
│  │ + overlay    │   (overlay persiste)                  │
│  └──────────────┘                                       │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

### Controles de UI
| Control | Función |
|---------|---------|
| **"🔍 Auto-Detectar"** | Activa/desactiva detección periódica |
| **"cada Xs"** | Intervalo de detección (1-10 segundos) |
| **"Saliencia"** | Muestra/oculta mapa de calor |
| **"Boxes"** | Muestra/oculta bounding boxes |

### Flujo de Datos
1. **Cámara** emite frames a 30 FPS
2. **update_frame()** muestra frame + overlay (si existe)
3. **QTimer** dispara cada N segundos
4. **DetectionWorker** ejecuta U2-Net en hilo separado
5. **Saliency map** se guarda y se superpone en frames siguientes

### Ventajas
- ✅ Cámara fluida a 30 FPS (sin bloqueos)
- ✅ Detección solo cuando es necesario
- ✅ Overlay persiste entre detecciones
- ✅ Intervalo configurable por usuario
- ✅ No consume GPU constantemente

---

## 🐛 Fix Congelamiento (v2.6)

### Problema Identificado
El programa se congelaba al activar el overlay de saliencia.

**Análisis del log:**
```
13:39:37 | [U2Net] Total=534ms | Preproc=85ms | Infer=232ms | Resize=195ms | Extract=21ms
13:39:37 | Detección: 0 objetos en 535ms
13:39:40 | [U2Net] Total=555ms | ...
13:39:41 | Detección: 0 objetos en 557ms
13:39:43 | [U2Net] Total=454ms | ...
13:39:46 | Detección: 0 objetos en 455ms
(programa se congela - no más logs)
```

**Causa raíz:**
El método `_draw_overlay()` se ejecutaba en **cada frame** (30 FPS) y hacía:
```python
# ANTES - Ejecutado 30 veces/segundo:
sal = cv2.resize(self.saliency_map, (1920, 1200))  # ~50ms
sal_color = cv2.applyColorMap(...)                  # ~20ms
vis = cv2.addWeighted(...)                          # ~10ms
# Total: ~80ms × 30 = 2400ms/segundo (imposible)
```

### Solución: Cache de Colormap

**Arquitectura corregida:**
```
┌─────────────────────────────────────────────────────────┐
│                 FLUJO OPTIMIZADO v2.6                    │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  DETECCIÓN (cada 2s)           VISUALIZACIÓN (30 FPS)   │
│  ┌──────────────────┐          ┌──────────────────┐     │
│  │ U2-Net detect()  │          │ update_frame()   │     │
│  │       ↓          │          │       ↓          │     │
│  │ saliency_map     │          │ raw_frame        │     │
│  │       ↓          │          │       +          │     │
│  │ cv2.resize()     │──────────│ saliency_colormap│     │
│  │       ↓          │  (cache) │       ↓          │     │
│  │ cv2.applyColorMap│          │ cv2.addWeighted()│     │
│  │       ↓          │          │   (solo blend)   │     │
│  │ saliency_colormap│          │       ↓          │     │
│  └──────────────────┘          │ video_label      │     │
│   (1 vez cada 2s)              └──────────────────┘     │
│                                 (30 veces/segundo)      │
└─────────────────────────────────────────────────────────┘
```

**Código corregido:**
```python
# EN _on_detection_done() - Solo 1 vez por detección:
sal_resized = cv2.resize(saliency_map, (w, h))
self.saliency_colormap = cv2.applyColorMap(sal_resized, cv2.COLORMAP_JET)

# EN _draw_overlay() - 30 veces/segundo (rápido):
vis = cv2.addWeighted(vis, 0.6, self.saliency_colormap, 0.4, 0)
```

### Variables de Cache Agregadas
```python
self.frame_size = None           # (w, h) del frame actual
self.saliency_colormap = None    # Cache del colormap pre-calculado
```

### Rendimiento
| Operación | Antes | Ahora |
|-----------|-------|-------|
| `cv2.resize()` | 30×/s | 1×/2s |
| `cv2.applyColorMap()` | 30×/s | 1×/2s |
| `cv2.addWeighted()` | 30×/s | 30×/s |
| **Tiempo por frame** | ~80ms | ~5ms |

---

## 📊 Mejora de Logs (v2.7)

### Problema
El log anterior no mostraba suficiente información para diagnosticar problemas:
- No se sabía si el colormap se generaba correctamente
- No se sabía si el overlay se dibujaba

### Cambios Implementados

#### 1. Nuevo método `_update_colormap_cache()`
```python
def _update_colormap_cache(self):
    """Actualiza el cache del colormap cuando hay nuevo saliency_map."""
    if self.saliency_map is None or self.frame_size is None:
        self.saliency_colormap = None
        return
    
    w, h = self.frame_size
    sal_resized = cv2.resize(self.saliency_map, (w, h))
    self.saliency_colormap = cv2.applyColorMap(...)
    logger.debug(f"Colormap cache actualizado: {w}x{h}")
```

#### 2. Logs mejorados en `_on_detection_done()`
```python
has_colormap = "✓" if self.saliency_colormap is not None else "✗"
logger.info(f"Detección: {n_obj} objetos en {time_ms:.0f}ms, colormap={has_colormap}")
```

#### 3. Indicador visual en UI
```python
overlay_status = "🟢" if self.saliency_colormap is not None else "⚪"
self.frame_info.setText(f"Frame: {self.frame_count} | ... | Overlay: {overlay_status}")
```

### Interpretación de Logs
| Log | Significado |
|-----|-------------|
| `colormap=✓` | Colormap generado correctamente |
| `colormap=✗` | Error: frame_size es None (no hay frames) |
| `Overlay: 🟢` | Overlay activo y listo para dibujar |
| `Overlay: ⚪` | Sin overlay (no hay detección o error) |

---

## 🎛️ Parámetros Editables (v2.8)

### Problema
Los parámetros de U2-Net estaban hardcodeados y no se podían modificar desde la UI:
- `min_area = 500` (fijo)
- `max_area = 500000` (fijo)
- `saliency_threshold = 0.3` (fijo)

Esto causaba que siempre se detectaran **0 objetos** porque los parámetros no eran adecuados para la imagen.

### Solución: Controles en Ventana de Cámara

Se agregaron spinboxes editables directamente en `CameraViewWindow`:

```
┌─────────────────────────────────────────────────────────────┐
│ 🔍 Auto [2s] | Mapa ☑ | Boxes ☑ | Sin detección            │
├─────────────────────────────────────────────────────────────┤
│ Área: [100] - [500000] | Umbral: [0.30]                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│                    [VIDEO FEED]                             │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Controles Agregados
| Control | Rango | Default | Descripción |
|---------|-------|---------|-------------|
| `min_area_spin` | 10-100000 | 100 | Área mínima de objeto (px) |
| `max_area_spin` | 100-1000000 | 500000 | Área máxima de objeto (px) |
| `threshold_spin` | 0.05-0.95 | 0.30 | Umbral de saliencia |

### Flujo de Actualización
```python
# Cuando el usuario cambia un parámetro:
_on_params_changed()
    ├── Actualiza self.detection_params
    ├── Actualiza DetectionWorker.set_params()
    └── Actualiza U2NetDetector.set_parameters()
```

### Guía de Ajuste de Parámetros
| Situación | Ajuste |
|-----------|--------|
| No detecta objetos | Bajar `threshold` (ej: 0.1) |
| Detecta ruido/marcas pequeñas | Subir `min_area` (ej: 1000) |
| No detecta objetos grandes | Subir `max_area` (ej: 800000) |
| Detecta demasiados objetos | Subir `threshold` (ej: 0.5) |

---

## 📁 Archivos Creados/Modificados

### Nuevos Módulos
| Archivo | Descripción | Líneas |
|---------|-------------|--------|
| `src/core/detection/__init__.py` | Exports del módulo | 9 |
| `src/core/detection/u2net_detector.py` | **Detector U2-Net Singleton** | ~320 |
| `src/core/services/__init__.py` | Exports de servicios | 9 |
| `src/core/services/detection_service.py` | **Servicio de detección asíncrono** | ~150 |
| `src/core/services/autofocus_service.py` | **Servicio de autofoco asíncrono** | ~230 |
| `src/gui/widgets/__init__.py` | Exports de widgets | 7 |
| `src/gui/widgets/saliency_overlay.py` | **Widget de visualización con overlays** | ~300 |

### Archivos Modificados
| Archivo | Cambios |
|---------|---------|
| `src/main.py` | Imports de nuevos módulos, inicialización de U2-Net al inicio, callbacks de servicios |

---

## ✅ Estado de Verificación

- [x] **U2-Net carga correctamente** - Verificado en CUDA
- [x] **Pesos u2netp.pth encontrados** - `models/weights/u2netp.pth`
- [x] **Imports funcionan** - Todos los módulos importan sin errores
- [ ] **Test en tiempo real** - Pendiente prueba con cámara
- [ ] **Test de autofoco** - Pendiente prueba con C-Focus

---

**Estado:** ✅ IMPLEMENTACIÓN BASE COMPLETADA - Listo para testing

---

## 🔧 v3.0 - MISMO MÉTODO QUE ImgAnalysisTab (2025-12-12)

### Problema Identificado
`U2NetDetector` **NO normaliza** la salida del modelo U2-Net:
```
[Extract] Saliency stats: min=0.000, max=0.037, mean=0.001, threshold=0.3
[Extract] Pixels above threshold: 0 (0.0%)
```
El máximo de saliencia es **0.037** pero el threshold es **0.3** → **0 detecciones**.

`SmartFocusScorer` (vía `SalientObjectDetector`) **SÍ normaliza** a [0,1] → funciona correctamente.

### Solución: MISMO MÉTODO que ImgAnalysisTab

```
┌─────────────────────────────────────────────────────────────────┐
│                        main.py                                   │
│  self.smart_focus_scorer = self.img_analysis_tab.scorer         │
│                            │                                     │
│              ┌─────────────┴─────────────┐                      │
│              ▼                           ▼                      │
│     ImgAnalysisTab                CameraViewWindow              │
│     (archivos)                   (cámara en vivo)               │
│              │                           │                      │
│              └───────────┬───────────────┘                      │
│                          ▼                                      │
│                  SmartFocusScorer                               │
│                  (MISMO para ambos)                             │
│                          │                                      │
│                          ▼                                      │
│               SalientObjectDetector                             │
│               (normaliza salida a [0,1])                        │
└─────────────────────────────────────────────────────────────────┘
```

### Cambios en camera_window.py

**DetectionWorker** usa `SmartFocusScorer` IGUAL que ImgAnalysisTab:
```python
def run(self):
    # PASO 1: Convertir a grayscale uint8 (IGUAL que ImgAnalysisTab)
    frame = self.frame
    if frame.dtype == np.uint16:
        frame = (frame / 256).astype(np.uint8)
    if len(frame.shape) == 3:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # PASO 2: Usar scorer.assess_image() IGUAL que ImgAnalysisTab
    result = self.scorer.assess_image(frame)
    
    # PASO 3: Extraer probability_map y objects
    probability_map = result.probability_map  # Ya normalizado [0,1]
    objects = result.objects  # Lista de ObjectInfo
```

**Objetos detectados** usan `ObjectInfo` de SmartFocusScorer:
```python
# ObjectInfo attributes:
# - bounding_box: (x, y, w, h)
# - centroid: (cx, cy)
# - focus_score: float
# - is_focused: bool
# - area: float
```

### Flujo de Datos (IGUAL que ImgAnalysisTab)
```
CameraTab.open_camera_view()
    │
    └── camera_view_window.set_scorer(parent_gui.smart_focus_scorer)

Cada N segundos:
    │
    └── detection_worker.detect_frame(raw_frame)
            │
            ├── Convertir uint16 → uint8 grayscale
            │
            └── scorer.assess_image(frame)  ← MISMO MÉTODO
                    │
                    └── Emite: (probability_map, objects, time_ms)
```

### Fix Crítico: Conversión de Frame
La cámara Thorlabs envía `uint16`. ImgAnalysisTab usa `cv2.IMREAD_GRAYSCALE` (uint8).
```python
if frame.dtype == np.uint16:
    frame = (frame / 256).astype(np.uint8)
if len(frame.shape) == 3:
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
```

---

## 📊 Resumen de Arquitectura Final

### Componente Único de Detección
| Componente | Archivo | Uso |
|------------|---------|-----|
| `SmartFocusScorer` | `img_analysis/smart_focus_scorer.py` | **AMBOS** (ImgAnalysisTab y CameraViewWindow) |

### Parámetros de SmartFocusScorer
| Parámetro | Default | Descripción |
|-----------|---------|-------------|
| `threshold` | 0.5 | Umbral de saliencia normalizado |
| `min_area` | 500 | Área mínima de objeto (px) |
| `min_prob` | 0.3 | Probabilidad mínima |

---

**Estado:** ✅ v3.0 IMPLEMENTADA - MISMO método que ImgAnalysisTab

---

## 🔧 v3.1 - CORRECCIONES (2025-12-12)

### Errores Corregidos

1. **`_run_detection` no existe** → Cambiado a `_trigger_detection`
   - `camera_tab.py` línea 1208: `self.camera_view_window._trigger_detection()`

2. **Signal con frame para overlay consistente**
   - `DetectionWorker.detection_done` ahora emite: `(probability_map, objects, time_ms, frame_used)`
   - Permite guardar el frame que se usó para detección

3. **Colormap usa tamaño del frame EN VIVO**
   - `_update_colormap_cache()` ahora usa `last_raw_frame.shape` (1920x1200)
   - Garantiza que el overlay coincida con el video en vivo

### Flujo Corregido
```
DetectionWorker.run()
    │
    ├── Convierte frame a uint8 grayscale
    │
    ├── scorer.assess_image(frame)
    │
    └── Emite: (probability_map, objects, time_ms, frame_usado)
            │
            └── _on_detection_done()
                    │
                    ├── Guarda detection_frame
                    │
                    └── _update_colormap_cache()
                            │
                            └── Usa last_raw_frame.shape (1920x1200)
```

**Estado:** ✅ v3.1 - Errores corregidos

---

## 🔧 v3.2 - FIX OVERLAY NEGRO (2025-12-12)

### Problema
La imagen quedaba en NEGRO cuando se activaba el overlay. Solo se veían los bounding boxes sobre fondo negro.

### Causa
El método `_draw_overlay` usaba un sistema de cache de colormap que fallaba cuando los tamaños no coincidían.

### Solución
Reescribir `_draw_overlay` para que sea IDÉNTICO a `ImgAnalysisTab._refresh_view()`:

```python
def _draw_overlay(self, frame):
    # PASO 1: Convertir a uint8 BGR
    if frame.dtype == np.uint16:
        frame_uint8 = (frame / 256).astype(np.uint8)
    
    if len(frame_uint8.shape) == 2:
        vis = cv2.cvtColor(frame_uint8, cv2.COLOR_GRAY2BGR)
    
    # PASO 2: Overlay de probabilidad (resize al tamaño del frame actual)
    if self.saliency_map is not None:
        prob_resized = cv2.resize(self.saliency_map, (w, h))
        prob_uint8 = (prob_resized * 255).astype(np.uint8)
        heatmap = cv2.applyColorMap(prob_uint8, cv2.COLORMAP_JET)
        vis = cv2.addWeighted(vis, 0.5, heatmap, 0.5, 0)
    
    # PASO 3: Bounding boxes
    for obj in self.detected_objects:
        cv2.rectangle(vis, ...)
```

**Estado:** ✅ v3.2 - Overlay corregido

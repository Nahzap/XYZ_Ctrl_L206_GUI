# 🔬 Plan de Integración: Autofoco C-Focus con Detección Multi-Objeto

**Documento creado:** 2025-12-11  
**Última actualización:** 2025-12-11  
**Autor:** Sistema de Control L206 + C-Focus Piezo  
**Objetivo:** Integrar el piezo C-Focus de Mad City Labs con detección U2-Net para autofoco multi-objeto basado en umbral de píxeles.

---

## 📋 Análisis del Mecanismo de Trigger Actual

### Flujo de Captura Existente (sin autofoco)

El sistema actual implementa un mecanismo de trigger en **4 FASES**:

```
┌─────────────────────────────────────────────────────────────────┐
│  FASE 1: MOVIMIENTO XY (_microscopy_move_to_point)             │
│  - Obtiene punto (x, y) de la trayectoria                      │
│  - Configura referencias en TestTab                            │
│  - Inicia control dual (PID) para mover motores                │
│  - Lanza timer para verificar posición (200ms)                 │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼ (cada 100ms)
┌─────────────────────────────────────────────────────────────────┐
│  FASE 2: VERIFICACIÓN DE POSICIÓN (_microscopy_check_position) │
│  - Verifica si _position_reached == True                        │
│  - Timeout: 10 segundos (100 checks × 100ms)                   │
│  - Si alcanzó posición: detiene motores y continúa             │
│  - Si no: sigue verificando cada 100ms                         │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼ (posición alcanzada)
┌─────────────────────────────────────────────────────────────────┐
│  FASE 3: DELAY DE ESTABILIZACIÓN                                │
│  - Espera delay_before_ms (default: 2000ms)                    │
│  - Permite estabilización mecánica de los motores              │
│  - Lanza QTimer.singleShot → _microscopy_capture()            │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│  FASE 4: CAPTURA (_microscopy_capture)                         │
│  - Llama a camera_tab.capture_microscopy_image()              │
│  - Guarda imagen: {class_name}_{index:05d}.png                │
│  - Aplica delay_after_ms (default: 200ms)                     │
│  - Avanza al siguiente punto o finaliza                        │
└─────────────────────────────────────────────────────────────────┘
```

### Puntos Clave del Trigger Actual

1. **Trigger de captura:** Posición XY alcanzada + delay de estabilización
2. **Una imagen por punto:** No hay iteración sobre objetos
3. **Sin eje Z:** El sistema actual solo controla XY
4. **Nomenclatura simple:** `{class_name}_{index:05d}.png`
5. **Método crítico:** `camera_tab.capture_microscopy_image(config, image_index)`

---

## 🎯 Arquitectura Propuesta: C-Focus + Detección Multi-Objeto

### Nuevo Flujo con Autofoco por Objeto

```
┌─────────────────────────────────────────────────────────────────┐
│  FASE 1: MOVIMIENTO XY (sin cambios)                           │
│  - Mover a punto (x, y) de trayectoria                         │
│  - Verificar posición alcanzada                                │
│  - Delay de estabilización mecánica                            │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼ (NUEVO TRIGGER)
┌─────────────────────────────────────────────────────────────────┐
│  FASE 2: PRE-DETECCIÓN DE OBJETOS                              │
│                                                                 │
│  1. Capturar frame de referencia (Z actual)                    │
│  2. Ejecutar U2-Net → detectar objetos salientes              │
│  3. FILTRAR por umbral de píxeles:                             │
│     - min_pixels ≤ área_objeto ≤ max_pixels                   │
│     - Ejemplo: 100 ≤ área ≤ 50000 píxeles                     │
│  4. Resultado: Lista de objetos válidos                        │
│                                                                 │
│  Si N_objetos = 0: Saltar punto, continuar trayectoria        │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼ (Para cada objeto i = 1..N)
┌─────────────────────────────────────────────────────────────────┐
│  FASE 3: AUTOFOCO C-FOCUS POR OBJETO                           │
│                                                                 │
│  Para Objeto_i con ROI = bbox_i:                               │
│    1. Z_start = Z_actual (leer con MCL_SingleReadZ)           │
│    2. Búsqueda Golden Section en [Z_min, Z_max]:              │
│       a. Mover C-Focus: MCL_SingleWriteZ(Z_test)              │
│       b. Esperar settle_time (100-300ms)                       │
│       c. Capturar frame de cámara Thorlabs                     │
│       d. Calcular S_i = Laplacian(ROI_i) ← SOLO bbox          │
│       e. Actualizar rango según S1 vs S2                       │
│    3. Z_optimo_i = posición con max(S_i)                       │
│    4. Mover a Z_optimo_i                                       │
│    5. Capturar imagen → {class}_{point:05d}_obj{i:02d}.png   │
│                                                                 │
│  Resultado: N imágenes, cada una enfocada en su objeto        │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│  FASE 4: SIGUIENTE PUNTO                                        │
│  - Restaurar Z a posición neutral (opcional)                   │
│  - Aplicar delay_after_ms                                      │
│  - Mover a siguiente punto XY                                  │
│  - Repetir desde Fase 1                                        │
└─────────────────────────────────────────────────────────────────┘
```

### Diferencias Clave vs Plan Original

| Aspecto | Plan Original | Plan C-Focus Actualizado |
|---------|---------------|--------------------------|
| Control Z | Motor genérico | **C-Focus Piezo (Madlib.dll)** |
| Rango Z | 100 µm | **Configurable (0-200 µm típico)** |
| Precisión | 1 µm | **~0.1 µm (piezo)** |
| Filtrado | Área mínima genérica | **Umbral min/max píxeles** |
| Settle time | 100ms | **100-300ms (piezo más rápido)** |
| Comunicación | Serial Arduino | **DLL nativa (ctypes)** |

---

## 🔧 Componentes a Implementar

### 1. CFocusController (Nuevo módulo)

**Ubicación:** `src/hardware/cfocus/cfocus_controller.py`

```python
"""
Mad City Labs C-Focus Piezo Stage Controller
Wrapper para integración con sistema de microscopía.
"""

import ctypes
import time
import logging
from typing import Optional, Tuple

logger = logging.getLogger('MotorControl_L206')


class CFocusController:
    """
    Controlador para piezo C-Focus de Mad City Labs.
    
    Funciones principales:
    - Inicializar/liberar handle del dispositivo
    - Mover a posición Z absoluta (µm)
    - Leer posición Z actual
    - Obtener rango calibrado
    """
    
    def __init__(self, dll_path: str = r"D:\MCL C Focus\Program Files 64\Mad City Labs\NanoDrive\Madlib.dll"):
        """
        Inicializa el controlador C-Focus.
        
        Args:
            dll_path: Ruta al DLL de Mad City Labs
        """
        self.dll_path = dll_path
        self.mcl_dll = None
        self.handle = 0
        self.z_range = 0.0
        self.is_connected = False
        
        # Parámetros de operación
        self.settle_time = 0.15  # segundos (150ms para estabilización piezo)
        
    def connect(self) -> Tuple[bool, str]:
        """
        Conecta con el dispositivo C-Focus.
        
        Returns:
            (success, message): Tupla con estado y mensaje
        """
        try:
            # Cargar DLL
            logger.info(f"Cargando C-Focus DLL: {self.dll_path}")
            self.mcl_dll = ctypes.WinDLL(self.dll_path)
            
            # Configurar signatures
            self._setup_function_signatures()
            
            # Inicializar handle
            logger.info("Inicializando C-Focus handle...")
            self.handle = self.mcl_dll.MCL_InitHandle()
            
            if self.handle == 0:
                return False, "Error: No se pudo inicializar handle (dispositivo no conectado o en uso)"
            
            # Obtener rango calibrado del eje Z
            self.z_range = self.mcl_dll.MCL_GetCalibration(3, self.handle)  # Axis 3 = Z
            
            if self.z_range <= 0:
                self.disconnect()
                return False, f"Error: Rango Z inválido ({self.z_range} µm)"
            
            self.is_connected = True
            logger.info(f"C-Focus conectado. Handle: {self.handle}, Rango Z: {self.z_range:.2f} µm")
            return True, f"C-Focus conectado (Rango: 0-{self.z_range:.1f} µm)"
            
        except FileNotFoundError:
            return False, f"DLL no encontrado: {self.dll_path}"
        except OSError as e:
            return False, f"Error cargando DLL: {e}"
        except Exception as e:
            logger.error(f"Error conectando C-Focus: {e}")
            return False, f"Error inesperado: {e}"
    
    def disconnect(self):
        """Desconecta y libera el handle del dispositivo."""
        if self.handle != 0 and self.mcl_dll is not None:
            try:
                logger.info("Liberando C-Focus handle...")
                self.mcl_dll.MCL_ReleaseHandle(self.handle)
                logger.info("C-Focus desconectado correctamente")
            except Exception as e:
                logger.error(f"Error liberando handle: {e}")
            finally:
                self.handle = 0
                self.is_connected = False
    
    def move_z(self, position_um: float) -> bool:
        """
        Mueve el piezo a una posición Z absoluta.
        
        Args:
            position_um: Posición en micrómetros (0 a z_range)
            
        Returns:
            bool: True si el movimiento fue exitoso
        """
        if not self.is_connected:
            logger.error("C-Focus no conectado")
            return False
        
        # Validar rango
        if position_um < 0 or position_um > self.z_range:
            logger.error(f"Posición Z fuera de rango: {position_um} µm (max: {self.z_range})")
            return False
        
        try:
            error_code = self.mcl_dll.MCL_SingleWriteZ(position_um, self.handle)
            
            if error_code != 0:
                logger.warning(f"C-Focus move retornó código {error_code}")
                return False
            
            # Esperar estabilización
            time.sleep(self.settle_time)
            return True
            
        except Exception as e:
            logger.error(f"Error moviendo C-Focus: {e}")
            return False
    
    def read_z(self) -> Optional[float]:
        """
        Lee la posición Z actual del piezo.
        
        Returns:
            float: Posición en µm, o None si hay error
        """
        if not self.is_connected:
            logger.error("C-Focus no conectado")
            return None
        
        try:
            position = self.mcl_dll.MCL_SingleReadZ(self.handle)
            return float(position)
        except Exception as e:
            logger.error(f"Error leyendo posición Z: {e}")
            return None
    
    def get_z_range(self) -> float:
        """Retorna el rango calibrado del eje Z."""
        return self.z_range
    
    def _setup_function_signatures(self):
        """Configura los tipos de argumentos y retorno para funciones MCL."""
        # MCL_InitHandle: int MCL_InitHandle(void)
        self.mcl_dll.MCL_InitHandle.argtypes = []
        self.mcl_dll.MCL_InitHandle.restype = ctypes.c_int
        
        # MCL_SingleReadZ: double MCL_SingleReadZ(int handle)
        self.mcl_dll.MCL_SingleReadZ.argtypes = [ctypes.c_int]
        self.mcl_dll.MCL_SingleReadZ.restype = ctypes.c_double
        
        # MCL_SingleWriteZ: int MCL_SingleWriteZ(double position, int handle)
        self.mcl_dll.MCL_SingleWriteZ.argtypes = [ctypes.c_double, ctypes.c_int]
        self.mcl_dll.MCL_SingleWriteZ.restype = ctypes.c_int
        
        # MCL_ReleaseHandle: void MCL_ReleaseHandle(int handle)
        self.mcl_dll.MCL_ReleaseHandle.argtypes = [ctypes.c_int]
        self.mcl_dll.MCL_ReleaseHandle.restype = None
        
        # MCL_GetCalibration: double MCL_GetCalibration(int axis, int handle)
        self.mcl_dll.MCL_GetCalibration.argtypes = [ctypes.c_int, ctypes.c_int]
        self.mcl_dll.MCL_GetCalibration.restype = ctypes.c_double
```

### 2. MultiObjectAutofocusController (Actualizado para C-Focus)

**Ubicación:** `src/core/autofocus/multi_object_autofocus.py`

```python
"""
Controlador de autofoco multi-objeto con C-Focus piezo.
Integra detección U2-Net con búsqueda de foco por objeto.
"""

import os
import time
import logging
from dataclasses import dataclass
from typing import List, Tuple, Optional, Callable
import numpy as np
import cv2

logger = logging.getLogger('MotorControl_L206')


@dataclass
class DetectedObject:
    """Objeto detectado en pre-escaneo."""
    index: int
    bounding_box: Tuple[int, int, int, int]  # (x, y, w, h)
    centroid: Tuple[int, int]
    area: float  # Área en píxeles
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
    Controlador de autofoco multi-objeto con C-Focus.
    
    Flujo:
    1. Pre-detectar objetos con U2-Net (filtrar por área en píxeles)
    2. Para cada objeto, buscar su Z óptimo usando C-Focus
    3. Capturar imagen cuando cada objeto esté enfocado
    """
    
    def __init__(
        self,
        scorer,  # SmartFocusScorer con U2-Net
        cfocus_controller,  # CFocusController
        get_frame_callback: Callable[[], np.ndarray]
    ):
        """
        Inicializa el controlador de autofoco.
        
        Args:
            scorer: SmartFocusScorer para detección y scoring
            cfocus_controller: Controlador del piezo C-Focus
            get_frame_callback: Función para obtener frame de cámara
        """
        self.scorer = scorer
        self.cfocus = cfocus_controller
        self.get_frame = get_frame_callback
        
        # Parámetros de búsqueda Z
        self.z_search_range = 50.0  # µm de rango total de búsqueda
        self.z_tolerance = 0.5      # µm tolerancia final
        self.max_iterations = 20
        
        # Parámetros de detección (UMBRAL DE PÍXELES)
        self.min_area_pixels = 100     # Área mínima en píxeles
        self.max_area_pixels = 50000   # Área máxima en píxeles
        self.min_probability = 0.3     # Probabilidad mínima U2-Net
        
        logger.info(f"MultiObjectAutofocus inicializado: "
                   f"rango={self.z_search_range}µm, "
                   f"área=[{self.min_area_pixels}, {self.max_area_pixels}] px")
    
    def set_pixel_threshold(self, min_pixels: int, max_pixels: int):
        """
        Configura el umbral de píxeles para filtrar objetos.
        
        Args:
            min_pixels: Área mínima en píxeles
            max_pixels: Área máxima en píxeles
        """
        self.min_area_pixels = min_pixels
        self.max_area_pixels = max_pixels
        logger.info(f"Umbral de píxeles actualizado: [{min_pixels}, {max_pixels}]")
    
    def predetect_objects(self) -> List[DetectedObject]:
        """
        FASE 2: Pre-detecta todos los objetos en el frame actual.
        Filtra por umbral de píxeles (min_area ≤ área ≤ max_area).
        
        Returns:
            Lista de objetos detectados válidos
        """
        frame = self.get_frame()
        
        if frame is None:
            logger.error("No hay frame disponible para pre-detección")
            return []
        
        # Ejecutar U2-Net
        result = self.scorer.assess_image(frame, return_debug_mask=False)
        
        if result.status == "EMPTY":
            logger.info("Pre-detección: sin objetos detectados")
            return []
        
        # Filtrar objetos por área de píxeles
        detected = []
        for i, obj in enumerate(result.objects):
            area_px = obj.area
            
            # FILTRO CRÍTICO: umbral de píxeles
            if self.min_area_pixels <= area_px <= self.max_area_pixels:
                detected.append(DetectedObject(
                    index=i,
                    bounding_box=obj.bounding_box,
                    centroid=obj.centroid,
                    area=area_px,
                    initial_score=obj.focus_score
                ))
                logger.debug(f"  Objeto {i}: área={area_px:.0f}px, bbox={obj.bounding_box}")
            else:
                logger.debug(f"  Objeto {i} RECHAZADO: área={area_px:.0f}px fuera de rango")
        
        logger.info(f"Pre-detección: {len(detected)}/{len(result.objects)} objetos válidos")
        return detected
    
    def focus_single_object(
        self, 
        obj: DetectedObject,
        z_center: Optional[float] = None
    ) -> Tuple[float, float]:
        """
        FASE 3: Busca el Z óptimo para UN objeto específico usando C-Focus.
        
        Usa Golden Section Search evaluando SOLO el ROI del objeto.
        
        Args:
            obj: Objeto a enfocar
            z_center: Centro del rango de búsqueda (default: Z actual)
            
        Returns:
            (z_optimal, max_score)
        """
        if z_center is None:
            z_center = self.cfocus.read_z()
            if z_center is None:
                logger.error("No se pudo leer posición Z actual")
                return 0.0, 0.0
        
        # Calcular rango de búsqueda
        z_min = max(0, z_center - self.z_search_range / 2)
        z_max = min(self.cfocus.get_z_range(), z_center + self.z_search_range / 2)
        
        logger.info(f"Autofoco Obj{obj.index}: búsqueda en [{z_min:.1f}, {z_max:.1f}] µm")
        
        phi = 0.618  # Golden ratio
        iteration = 0
        
        # Golden Section Search
        while (z_max - z_min) > self.z_tolerance and iteration < self.max_iterations:
            z1 = z_max - phi * (z_max - z_min)
            z2 = z_min + phi * (z_max - z_min)
            
            s1 = self._evaluate_object_focus(z1, obj.bounding_box)
            s2 = self._evaluate_object_focus(z2, obj.bounding_box)
            
            if s1 > s2:
                z_max = z2
                best_score = s1
            else:
                z_min = z1
                best_score = s2
            
            iteration += 1
            logger.debug(f"  Iter {iteration}: z1={z1:.2f}(S={s1:.1f}), z2={z2:.2f}(S={s2:.1f})")
        
        z_optimal = (z_min + z_max) / 2
        
        # Mover a posición óptima y verificar
        self.cfocus.move_z(z_optimal)
        final_score = self._evaluate_object_focus(z_optimal, obj.bounding_box)
        
        logger.info(f"Autofoco Obj{obj.index} completado: Z={z_optimal:.2f}µm, S={final_score:.1f}")
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
            Score de enfoque en el ROI (Laplacian variance)
        """
        # Mover C-Focus a posición
        success = self.cfocus.move_z(z_position)
        if not success:
            logger.warning(f"Fallo al mover a Z={z_position:.2f}")
            return 0.0
        
        # Capturar frame
        frame = self.get_frame()
        if frame is None:
            return 0.0
        
        x, y, w, h = bbox
        
        # Validar bbox dentro de frame
        h_frame, w_frame = frame.shape[:2]
        x = max(0, min(x, w_frame - 1))
        y = max(0, min(y, h_frame - 1))
        w = min(w, w_frame - x)
        h = min(h, h_frame - y)
        
        # Extraer ROI
        roi = frame[y:y+h, x:x+w]
        
        if roi.size == 0:
            return 0.0
        
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
        point_index: int,
        config: dict
    ) -> List[FocusedCapture]:
        """
        FASE 3+4: Enfoca y captura cada objeto individualmente.
        
        Args:
            objects: Lista de objetos pre-detectados
            save_folder: Carpeta de destino
            class_name: Nombre de la clase para el archivo
            point_index: Índice del punto de trayectoria
            config: Configuración de microscopía (canales, etc.)
            
        Returns:
            Lista de capturas realizadas
        """
        captures = []
        z_start = self.cfocus.read_z()
        
        if z_start is None:
            logger.error("No se pudo leer Z inicial")
            return captures
        
        logger.info(f"Capturando {len(objects)} objetos en punto {point_index}")
        
        for obj in objects:
            # Buscar Z óptimo para este objeto
            z_opt, score = self.focus_single_object(obj, z_center=z_start)
            
            if score < 5.0:  # Score mínimo para considerar válido
                logger.warning(f"Obj{obj.index}: score bajo ({score:.1f}), saltando captura")
                continue
            
            # Capturar imagen
            frame = self.get_frame()
            if frame is None:
                logger.error(f"No hay frame para capturar Obj{obj.index}")
                continue
            
            # Procesar frame según configuración (canales, etc.)
            processed_frame = self._process_frame(frame, config)
            
            # Nomenclatura: {class}_{point:05d}_obj{i:02d}.png
            filename = f"{class_name}_{point_index:05d}_obj{obj.index:02d}.png"
            filepath = os.path.join(save_folder, filename)
            
            # Guardar imagen
            success = cv2.imwrite(filepath, processed_frame, [cv2.IMWRITE_PNG_COMPRESSION, 6])
            
            if success:
                file_size_kb = os.path.getsize(filepath) / 1024
                logger.info(f"  {filename}: Z={z_opt:.2f}µm, S={score:.1f}, {file_size_kb:.0f}KB")
                
                captures.append(FocusedCapture(
                    object_index=obj.index,
                    z_optimal=z_opt,
                    focus_score=score,
                    image_path=filepath,
                    bounding_box=obj.bounding_box
                ))
            else:
                logger.error(f"Error guardando {filename}")
        
        logger.info(f"Punto {point_index}: {len(captures)} imágenes capturadas")
        return captures
    
    def _process_frame(self, frame: np.ndarray, config: dict) -> np.ndarray:
        """
        Procesa el frame según configuración (canales, resize, etc.).
        Replica lógica de camera_tab.capture_microscopy_image().
        """
        # Normalizar uint16 a uint8
        if frame.dtype == np.uint16:
            if frame.max() > 0:
                frame = (frame / frame.max() * 255).astype(np.uint8)
            else:
                frame = frame.astype(np.uint8)
        
        # Redimensionar si es necesario
        target_width = config.get('img_width', 1920)
        target_height = config.get('img_height', 1080)
        h, w = frame.shape[:2]
        
        if w != target_width or h != target_height:
            frame = cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_LINEAR)
        
        # Procesar canales (lógica de camera_tab)
        channels = config.get('channels', {'R': False, 'G': True, 'B': False})
        selected_channels = [c for c in ['R', 'G', 'B'] if channels.get(c, False)]
        n_selected = len(selected_channels)
        
        if len(frame.shape) == 2:  # Grayscale
            if n_selected >= 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                if n_selected < 3:
                    new_frame = np.zeros_like(frame)
                    if channels.get('B', False):
                        new_frame[:, :, 0] = frame[:, :, 0]
                    if channels.get('G', False):
                        new_frame[:, :, 1] = frame[:, :, 1]
                    if channels.get('R', False):
                        new_frame[:, :, 2] = frame[:, :, 2]
                    frame = new_frame
        
        elif len(frame.shape) == 3:  # Color
            if n_selected == 1:
                channel_map = {'B': 0, 'G': 1, 'R': 2}
                channel_idx = channel_map[selected_channels[0]]
                frame = frame[:, :, channel_idx]
            elif n_selected < 3:
                new_frame = np.zeros_like(frame)
                if channels.get('B', False):
                    new_frame[:, :, 0] = frame[:, :, 0]
                if channels.get('G', False):
                    new_frame[:, :, 1] = frame[:, :, 1]
                if channels.get('R', False):
                    new_frame[:, :, 2] = frame[:, :, 2]
                frame = new_frame
        
        return frame
```

### 3. Integración con main.py

**Modificar:** `src/main.py` - Agregar métodos de autofoco

```python
# En __init__ de ArduinoGUI:

# Inicializar C-Focus (después de cámara)
self.cfocus_controller = None
self.autofocus_controller = None
self.cfocus_enabled = False

# Método para conectar C-Focus
def connect_cfocus(self):
    """Conecta con el piezo C-Focus."""
    from hardware.cfocus.cfocus_controller import CFocusController
    
    if self.cfocus_controller is None:
        self.cfocus_controller = CFocusController()
    
    success, message = self.cfocus_controller.connect()
    
    if success:
        self.cfocus_enabled = True
        self.camera_tab.log_message(f"✅ C-Focus: {message}")
        logger.info(f"C-Focus conectado: {message}")
    else:
        self.cfocus_enabled = False
        self.camera_tab.log_message(f"❌ C-Focus: {message}")
        logger.error(f"Error C-Focus: {message}")
    
    return success

# Método para inicializar autofoco
def initialize_autofocus(self):
    """Inicializa el controlador de autofoco multi-objeto."""
    if not self.cfocus_enabled:
        self.camera_tab.log_message("⚠️ C-Focus no conectado")
        return False
    
    if self.camera_tab.camera_worker is None:
        self.camera_tab.log_message("⚠️ Cámara no conectada")
        return False
    
    # Importar SmartFocusScorer si no está disponible
    try:
        from core.autofocus.smart_focus_scorer import SmartFocusScorer
        from core.autofocus.multi_object_autofocus import MultiObjectAutofocusController
        
        # Crear scorer si no existe
        if not hasattr(self, 'focus_scorer'):
            self.focus_scorer = SmartFocusScorer()
        
        # Crear controlador de autofoco
        self.autofocus_controller = MultiObjectAutofocusController(
            scorer=self.focus_scorer,
            cfocus_controller=self.cfocus_controller,
            get_frame_callback=lambda: self.camera_tab.camera_worker.current_frame
        )
        
        self.camera_tab.log_message("✅ Autofoco multi-objeto inicializado")
        logger.info("Autofoco multi-objeto inicializado")
        return True
        
    except Exception as e:
        self.camera_tab.log_message(f"❌ Error inicializando autofoco: {e}")
        logger.error(f"Error inicializando autofoco: {e}")
        return False

# NUEVO MÉTODO: Captura con autofoco multi-objeto
def _microscopy_capture_with_autofocus(self):
    """
    Captura con pre-detección y autofoco multi-objeto.
    Reemplaza a _microscopy_capture() cuando autofoco está habilitado.
    """
    if not self.microscopy_active:
        return
    
    config = self.microscopy_config
    point_idx = self.microscopy_current_point
    
    # Verificar que autofoco esté inicializado
    if self.autofocus_controller is None:
        self.camera_tab.log_message("⚠️ Autofoco no inicializado, usando captura normal")
        self._microscopy_capture()
        return
    
    # FASE 2: Pre-detectar objetos
    self.camera_tab.log_message(f"🔍 Pre-detectando objetos...")
    objects = self.autofocus_controller.predetect_objects()
    n_objects = len(objects)
    
    if n_objects == 0:
        self.camera_tab.log_message(f"  ⚠️ Sin objetos válidos - saltando punto")
        logger.info(f"Punto {point_idx}: sin objetos detectados")
        
        # Avanzar al siguiente punto
        self.microscopy_current_point += 1
        
        if self.microscopy_current_point < len(self.microscopy_trajectory):
            QTimer.singleShot(self._delay_after_ms, self._microscopy_move_to_point)
        else:
            self._finish_microscopy()
        return
    
    self.camera_tab.log_message(f"  ✓ {n_objects} objeto(s) detectado(s)")
    logger.info(f"Punto {point_idx}: {n_objects} objetos válidos")
    
    # FASE 3: Enfocar y capturar cada objeto
    captures = self.autofocus_controller.capture_all_objects(
        objects=objects,
        save_folder=config.get('save_folder', '.'),
        class_name=config.get('class_name', 'Imagen'),
        point_index=point_idx,
        config=config
    )
    
    # Log resultados
    for cap in captures:
        self.camera_tab.log_message(
            f"  📸 Obj{cap.object_index}: Z={cap.z_optimal:.1f}µm, S={cap.focus_score:.1f}"
        )
    
    # Actualizar progreso
    self.camera_tab.set_microscopy_progress(
        self.microscopy_current_point + 1,
        len(self.microscopy_trajectory)
    )
    
    # Avanzar al siguiente punto
    self.microscopy_current_point += 1
    
    if self.microscopy_current_point < len(self.microscopy_trajectory):
        self.camera_tab.log_message(f"  Pausa post-captura: {self._delay_after_ms}ms")
        QTimer.singleShot(self._delay_after_ms, self._microscopy_move_to_point)
    else:
        self._finish_microscopy()

# MODIFICAR: _microscopy_capture() para soportar autofoco
def _microscopy_capture(self):
    """PASO 3: Captura la imagen después del delay de estabilización."""
    if not self.microscopy_active:
        return
    
    # Si autofoco está habilitado, usar método con autofoco
    if self.microscopy_config.get('autofocus_enabled', False) and self.autofocus_controller:
        self._microscopy_capture_with_autofocus()
        return
    
    # Captura normal (sin autofoco) - código existente
    self.camera_tab.log_message(f"  Capturando imagen...")
    success = self.camera_tab.capture_microscopy_image(
        self.microscopy_config, 
        self.microscopy_current_point
    )
    
    if success:
        logger.info(f"Microscopia: imagen {self.microscopy_current_point + 1} capturada")
    else:
        self.camera_tab.log_message(f"  ERROR: Fallo captura imagen {self.microscopy_current_point + 1}")
        logger.error(f"Microscopia: fallo captura imagen {self.microscopy_current_point + 1}")
    
    # Actualizar progreso
    self.camera_tab.set_microscopy_progress(
        self.microscopy_current_point + 1,
        len(self.microscopy_trajectory)
    )
    
    # Avanzar al siguiente punto
    self.microscopy_current_point += 1
    
    # PASO 4: Aplicar DELAY_AFTER antes de mover al siguiente punto
    if self.microscopy_current_point < len(self.microscopy_trajectory):
        self.camera_tab.log_message(f"  Pausa post-captura: {self._delay_after_ms}ms")
        QTimer.singleShot(self._delay_after_ms, self._microscopy_move_to_point)
    else:
        # Era el último punto
        self._finish_microscopy()
```

### 4. UI en CameraTab

**Modificar:** `src/gui/tabs/camera_tab.py` - Agregar controles de autofoco

```python
# En _setup_ui(), agregar grupo de autofoco:

# Grupo: Autofoco C-Focus
autofocus_group = QGroupBox("🔍 Autofoco Multi-Objeto (C-Focus)")
autofocus_layout = QVBoxLayout()

# Checkbox para habilitar
self.autofocus_enabled_cb = QCheckBox("Habilitar autofoco por objeto")
self.autofocus_enabled_cb.setToolTip(
    "Pre-detecta objetos con U2-Net y captura una imagen enfocada por cada uno.\n"
    "Genera N imágenes por punto, donde N = objetos detectados."
)
autofocus_layout.addWidget(self.autofocus_enabled_cb)

# Botones de conexión C-Focus
cfocus_btn_layout = QHBoxLayout()
self.cfocus_connect_btn = QPushButton("🔌 Conectar C-Focus")
self.cfocus_connect_btn.clicked.connect(self._connect_cfocus)
cfocus_btn_layout.addWidget(self.cfocus_connect_btn)

self.cfocus_disconnect_btn = QPushButton("⏹️ Desconectar")
self.cfocus_disconnect_btn.setEnabled(False)
self.cfocus_disconnect_btn.clicked.connect(self._disconnect_cfocus)
cfocus_btn_layout.addWidget(self.cfocus_disconnect_btn)

autofocus_layout.addLayout(cfocus_btn_layout)

# Parámetros de detección (UMBRAL DE PÍXELES)
detection_form = QFormLayout()

self.min_pixels_spin = QSpinBox()
self.min_pixels_spin.setRange(10, 100000)
self.min_pixels_spin.setValue(100)
self.min_pixels_spin.setSuffix(" px")
self.min_pixels_spin.setToolTip("Área mínima del objeto en píxeles")
detection_form.addRow("Área mínima:", self.min_pixels_spin)

self.max_pixels_spin = QSpinBox()
self.max_pixels_spin.setRange(100, 500000)
self.max_pixels_spin.setValue(50000)
self.max_pixels_spin.setSuffix(" px")
self.max_pixels_spin.setToolTip("Área máxima del objeto en píxeles")
detection_form.addRow("Área máxima:", self.max_pixels_spin)

autofocus_layout.addLayout(detection_form)

# Parámetros de búsqueda Z
search_form = QFormLayout()

self.z_range_spin = QDoubleSpinBox()
self.z_range_spin.setRange(5.0, 200.0)
self.z_range_spin.setValue(50.0)
self.z_range_spin.setSuffix(" µm")
self.z_range_spin.setToolTip("Rango total de búsqueda de foco")
search_form.addRow("Rango Z:", self.z_range_spin)

self.z_tolerance_spin = QDoubleSpinBox()
self.z_tolerance_spin.setRange(0.1, 5.0)
self.z_tolerance_spin.setValue(0.5)
self.z_tolerance_spin.setSuffix(" µm")
self.z_tolerance_spin.setDecimals(2)
self.z_tolerance_spin.setToolTip("Tolerancia de convergencia")
search_form.addRow("Tolerancia:", self.z_tolerance_spin)

autofocus_layout.addLayout(search_form)

# Label de estado
self.cfocus_status_label = QLabel("C-Focus: No conectado")
self.cfocus_status_label.setStyleSheet("color: #888; font-style: italic;")
autofocus_layout.addWidget(self.cfocus_status_label)

autofocus_group.setLayout(autofocus_layout)

# Agregar al layout principal (después de grupo de microscopía)
# main_layout.addWidget(autofocus_group)

# Métodos de callback:

def _connect_cfocus(self):
    """Conecta el piezo C-Focus."""
    if self.parent_gui:
        success = self.parent_gui.connect_cfocus()
        if success:
            self.cfocus_connect_btn.setEnabled(False)
            self.cfocus_disconnect_btn.setEnabled(True)
            self.cfocus_status_label.setText("C-Focus: ✅ Conectado")
            self.cfocus_status_label.setStyleSheet("color: #27AE60; font-weight: bold;")
            
            # Inicializar autofoco
            self.parent_gui.initialize_autofocus()

def _disconnect_cfocus(self):
    """Desconecta el piezo C-Focus."""
    if self.parent_gui and self.parent_gui.cfocus_controller:
        self.parent_gui.cfocus_controller.disconnect()
        self.cfocus_connect_btn.setEnabled(True)
        self.cfocus_disconnect_btn.setEnabled(False)
        self.cfocus_status_label.setText("C-Focus: No conectado")
        self.cfocus_status_label.setStyleSheet("color: #888; font-style: italic;")
        self.log_message("C-Focus desconectado")

# MODIFICAR: _start_microscopy() para incluir config de autofoco
def _start_microscopy(self):
    """Inicia microscopía con la configuración actual."""
    # ... código existente ...
    
    # Agregar configuración de autofoco
    config = {
        # ... config existente ...
        'autofocus_enabled': self.autofocus_enabled_cb.isChecked(),
        'min_pixels': self.min_pixels_spin.value(),
        'max_pixels': self.max_pixels_spin.value(),
        'z_range': self.z_range_spin.value(),
        'z_tolerance': self.z_tolerance_spin.value()
    }
    
    # Actualizar parámetros en autofocus controller si está habilitado
    if config['autofocus_enabled'] and self.parent_gui.autofocus_controller:
        self.parent_gui.autofocus_controller.set_pixel_threshold(
            config['min_pixels'],
            config['max_pixels']
        )
        self.parent_gui.autofocus_controller.z_search_range = config['z_range']
        self.parent_gui.autofocus_controller.z_tolerance = config['z_tolerance']
    
    # Emitir señal
    self.microscopy_start_requested.emit(config)
```

---

## 📐 Parámetros de Configuración

### Valores Recomendados

| Parámetro | Valor Default | Rango | Descripción |
|-----------|---------------|-------|-------------|
| **min_area_pixels** | 100 | 10-10000 | Área mínima del objeto |
| **max_area_pixels** | 50000 | 1000-500000 | Área máxima del objeto |
| **z_search_range** | 50 µm | 10-200 | Rango total de búsqueda |
| **z_tolerance** | 0.5 µm | 0.1-5 | Tolerancia de convergencia |
| **settle_time** | 150 ms | 50-500 | Tiempo de estabilización piezo |
| **min_focus_score** | 5.0 | 1-50 | Score mínimo para captura |

### Nomenclatura de Archivos

```
{save_folder}/
├── {class_name}_00000_obj00.png    # Punto 0, Objeto 0
├── {class_name}_00000_obj01.png    # Punto 0, Objeto 1
├── {class_name}_00001_obj00.png    # Punto 1, Objeto 0
├── {class_name}_00002_obj00.png    # Punto 2, Objeto 0
├── {class_name}_00002_obj01.png    # Punto 2, Objeto 1
├── {class_name}_00002_obj02.png    # Punto 2, Objeto 2
└── ...
```

**Ventajas de esta nomenclatura:**
- Ordena por punto primero, luego por objeto
- Fácil de filtrar por punto o por objeto
- Compatible con herramientas de análisis de imágenes

---

## 🗓️ Plan de Implementación

### Fase 1: Infraestructura C-Focus (2-3 horas)
- [ ] Crear `src/hardware/cfocus/__init__.py`
- [ ] Crear `src/hardware/cfocus/cfocus_controller.py`
- [ ] Implementar `CFocusController` con DLL wrapper
- [ ] Tests de conexión y movimiento básico
- [ ] Validar rango y precisión del piezo

### Fase 2: Autofoco Multi-Objeto (3-4 horas)
- [ ] Crear `src/core/autofocus/__init__.py`
- [ ] Crear `src/core/autofocus/multi_object_autofocus.py`
- [ ] Implementar `MultiObjectAutofocusController`
- [ ] Implementar filtrado por umbral de píxeles
- [ ] Implementar Golden Section Search con C-Focus
- [ ] Tests de convergencia y precisión

### Fase 3: Integración con Microscopía (3-4 horas)
- [ ] Modificar `main.py`: agregar `connect_cfocus()` y `initialize_autofocus()`
- [ ] Implementar `_microscopy_capture_with_autofocus()`
- [ ] Modificar `_microscopy_capture()` para soportar modo autofoco
- [ ] Agregar manejo de errores y timeouts

### Fase 4: UI y Configuración (2-3 horas)
- [ ] Agregar grupo de autofoco en `camera_tab.py`
- [ ] Implementar controles de umbral de píxeles (min/max)
- [ ] Agregar botones de conexión C-Focus
- [ ] Conectar callbacks y señales
- [ ] Actualizar barra de progreso para multi-objeto

### Fase 5: Validación y Optimización (2-3 horas)
- [ ] Pruebas con muestras reales
- [ ] Ajustar parámetros de búsqueda (settle_time, tolerance)
- [ ] Profiling de rendimiento (tiempo por objeto)
- [ ] Documentación de uso

**Tiempo total estimado:** 12-17 horas

---

## 📊 Métricas de Éxito

| Métrica | Objetivo | Medición |
|---------|----------|----------|
| Tiempo autofoco/objeto | < 2 segundos | Cronómetro |
| Precisión Z | ± 0.5 µm | Comparar con manual |
| Tasa de detección | > 90% | Objetos detectados / total |
| Score promedio | > 15 | Estadísticas de sesión |
| Objetos por punto | Variable | Depende de muestra |

---

## ⚠️ Consideraciones Críticas

### 1. Sincronización Cámara-Piezo
- **Problema:** La cámara Thorlabs y el piezo C-Focus deben estar sincronizados
- **Solución:** Usar `settle_time` adecuado (150-300ms) después de cada movimiento Z

### 2. Rango del Piezo
- **C-Focus típico:** 0-200 µm
- **Validar:** Siempre verificar `z_range` al conectar
- **Límites:** Nunca exceder el rango calibrado

### 3. Umbral de Píxeles
- **Crítico:** Ajustar según tamaño de objetos en la muestra
- **Polen:** ~500-5000 píxeles
- **Células:** ~1000-20000 píxeles
- **Partículas pequeñas:** ~100-1000 píxeles

### 4. Manejo de Errores
- **DLL no encontrado:** Verificar path del DLL
- **Handle = 0:** Dispositivo no conectado o en uso
- **Timeout:** Si el autofoco no converge en max_iterations

---

## 📚 Referencias

1. **Mad City Labs C-Focus Manual**
   - API reference para Madlib.dll
   - Especificaciones técnicas del piezo

2. **Zhou, K., & Doyle, J. C.** (1998). *Essentials of Robust Control*.
   - Golden Section Search para optimización

3. **Pech-Pacheco, J. L., et al.** (2000). *Diatom autofocusing in brightfield microscopy*.
   - Laplacian variance como métrica de enfoque

4. **U2-Net** (2020). *Salient Object Detection*.
   - Arquitectura del detector

---

## ✅ Checklist Pre-Implementación

- [x] Algoritmo U2-Net funcionando
- [x] Score de enfoque (Laplaciano) implementado
- [x] Trayectoria XY funcionando
- [x] Trigger de captura funcionando
- [ ] C-Focus conectado y probado
- [ ] DLL path verificado
- [ ] Rango Z del piezo conocido
- [ ] Latencia de movimiento Z medida
- [ ] Umbral de píxeles calibrado para muestra

---

## 🔄 Diferencias vs Plan Original

| Aspecto | Plan Original | Plan C-Focus |
|---------|---------------|--------------|
| Motor Z | Genérico (Arduino) | **C-Focus Piezo (DLL)** |
| Comunicación | Serial | **ctypes + WinDLL** |
| Precisión | 1 µm | **0.1 µm** |
| Velocidad | ~500ms/move | **~150ms/move** |
| Rango | Configurable | **0-200 µm (típico)** |
| Filtrado | Área genérica | **Umbral min/max píxeles** |
| Trigger | Modificado | **Integrado en flujo existente** |

---

**FIN DEL DOCUMENTO**

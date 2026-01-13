"""
Servicio de Microscopía Automatizada
=====================================

Orquesta la ejecución de trayectorias de microscopía con:
- Movimiento XY automatizado
- Autofoco inteligente por punto
- Captura de imágenes multicanal
- Progreso en tiempo real
- Sistema de aprendizaje de ROIs (50 imágenes)

REFACTORIZACIÓN 2025-12-29:
- Usa MicroscopyStateManager para gestión de estado
- Usa MicroscopyValidator para validaciones
- Reducción de código duplicado

Autor: Sistema de Control L206
Fecha: 2025-12-13
"""

import logging
import time
import numpy as np
import cv2
import os
import json
from datetime import datetime

from typing import Callable, Optional, List, Tuple
from PyQt5.QtCore import QObject, pyqtSignal, QTimer, QCoreApplication

from core.services.microscopy_state import MicroscopyStateManager, MicroscopyState
from core.validators import MicroscopyValidator, MicroscopyConfig, ValidationResult

logger = logging.getLogger('MotorControl_L206')


class MicroscopyService(QObject):
    """Servicio que orquesta la microscopía automatizada.

    Coordina trayectoria, control de posición (vía TestTab o callbacks),
    captura de imágenes (vía CameraTab/CameraService) y autofoco (vía AutofocusService).

    Este servicio reemplaza la lógica de microscopia que antes vivía en ArduinoGUI.
    """

    status_changed = pyqtSignal(str)              # Mensajes de log para la UI
    progress_changed = pyqtSignal(int, int)       # current, total
    finished = pyqtSignal(int)                    # total imágenes
    stopped = pyqtSignal()                        # detenido por usuario
    show_masks = pyqtSignal(list)                 # Mostrar máscaras durante autofoco
    clear_masks = pyqtSignal()                    # Limpiar máscaras después de capturar
    detection_complete = pyqtSignal(list)         # Lista de objetos detectados (ObjectInfo)
    # Solicitud de confirmación de aprendizaje (frame, objeto, clase sugerida, confianza, count, target)
    learning_confirmation_requested = pyqtSignal(object, object, str, float, int, int)

    def __init__(
        self,
        parent=None,
        get_trajectory: Optional[Callable[[], Optional[List]]] = None,
        get_trajectory_params: Optional[Callable[[], dict]] = None,
        set_dual_refs: Optional[Callable[[float, float], None]] = None,
        start_dual_control: Optional[Callable[[], None]] = None,
        stop_dual_control: Optional[Callable[[], None]] = None,
        is_dual_control_active: Optional[Callable[[], bool]] = None,
        is_position_reached: Optional[Callable[[], bool]] = None,
        capture_microscopy_image: Optional[Callable[[dict, int], bool]] = None,
        autofocus_service=None,
        cfocus_enabled_getter: Optional[Callable[[], bool]] = None,
        get_current_frame: Optional[Callable[[], Optional[np.ndarray]]] = None,
        smart_focus_scorer=None,
        get_area_range: Optional[Callable[[], tuple]] = None,
        controllers_ready_getter: Optional[Callable[[], bool]] = None,
        test_service=None,
        send_command: Optional[Callable[[str], None]] = None,
    ):
        super().__init__(parent)

        # Callbacks y dependencias externas
        self._get_trajectory = get_trajectory
        self._get_trajectory_params = get_trajectory_params
        self._set_dual_refs = set_dual_refs
        self._start_dual_control = start_dual_control
        self._stop_dual_control = stop_dual_control
        self._is_dual_control_active = is_dual_control_active
        self._is_position_reached = is_position_reached
        self._capture_microscopy_image = capture_microscopy_image
        self._autofocus_service = autofocus_service
        self._cfocus_enabled_getter = cfocus_enabled_getter
        self._get_current_frame = get_current_frame
        self._smart_focus_scorer = smart_focus_scorer
        self._get_area_range = get_area_range
        self._controllers_ready_getter = controllers_ready_getter
        self._test_service = test_service
        self._send_command = send_command

        # REFACTORIZACIÓN: Usar StateManager y Validator
        self._state_manager = MicroscopyStateManager()
        self._validator = MicroscopyValidator()
        
        # Configuración y parámetros
        self._microscopy_config: Optional[dict] = None
        self._delay_before_ms = 0
        self._delay_after_ms = 0
        self._trajectory_tolerance = 25.0
        self._trajectory_pause = 2.0
        
        # Estado temporal para aprendizaje asistido
        self._pending_object = None
        self._pending_frame = None
        self._learning_dialog = None

    # ------------------------------------------------------------------
    # API pública
    # ------------------------------------------------------------------
    def start_microscopy(self, config: dict) -> bool:
        """Inicia la microscopia automatizada con la configuración dada.

        Devuelve True si se pudo iniciar, False si hubo algún error de precondiciones.
        """
        logger.info("[MicroscopyService] === INICIANDO MICROSCOPIA AUTOMATIZADA ===")
        logger.info("[MicroscopyService] Config: %s", config)

        # VALIDACIÓN 1: Proveedor de trayectoria
        if self._get_trajectory is None:
            msg = "❌ Error: No hay proveedor de trayectoria configurado"
            logger.error("[MicroscopyService] %s", msg)
            self.status_changed.emit(msg)
            return False
        logger.info("[MicroscopyService] ✓ Proveedor de trayectoria: OK")

        # VALIDACIÓN 2: Trayectoria definida
        trajectory = self._get_trajectory()
        logger.info("[MicroscopyService] Trayectoria obtenida: %s (tipo: %s)", 
                   trajectory if trajectory is None else f"{len(trajectory)} puntos",
                   type(trajectory).__name__)
        
        # Validar correctamente numpy arrays y listas
        if trajectory is None or len(trajectory) == 0:
            msg = "❌ Error: No hay trayectoria definida"
            logger.error("[MicroscopyService] %s", msg)
            logger.error("[MicroscopyService] La trayectoria está vacía o es None")
            logger.error("[MicroscopyService] Verifica que hayas generado la trayectoria en TestTab")
            self.status_changed.emit(msg)
            return False
        logger.info("[MicroscopyService] ✓ Trayectoria definida: %d puntos", len(trajectory))
        
        # VALIDACIÓN 3: Obtener parámetros de trayectoria (tolerancia y delays)
        trajectory_params = {}
        if self._get_trajectory_params:
            trajectory_params = self._get_trajectory_params()
            logger.info("[MicroscopyService] Parámetros de trayectoria: %s", trajectory_params)
        else:
            logger.warning("[MicroscopyService] No hay proveedor de parámetros, usando defaults")
            trajectory_params = {'tolerance_um': 25.0, 'pause_s': 2.0}
        
        # Guardar parámetros para uso en movimiento entre puntos
        self._trajectory_tolerance = trajectory_params.get('tolerance_um', 25.0)
        self._trajectory_pause = trajectory_params.get('pause_s', 2.0)
        
        logger.info("[MicroscopyService] ✓ Tolerancia: %.1fµm, Pausa: %.1fs", 
                   self._trajectory_tolerance, self._trajectory_pause)

        # VALIDACIÓN 3: Callbacks de control
        if not (self._set_dual_refs and self._start_dual_control and self._stop_dual_control):
            msg = "❌ Error: Callbacks de control dual no configurados"
            logger.error("[MicroscopyService] %s", msg)
            logger.error("[MicroscopyService]   _set_dual_refs: %s", self._set_dual_refs)
            logger.error("[MicroscopyService]   _start_dual_control: %s", self._start_dual_control)
            logger.error("[MicroscopyService]   _stop_dual_control: %s", self._stop_dual_control)
            self.status_changed.emit(msg)
            return False
        logger.info("[MicroscopyService] ✓ Callbacks de control: OK")

        # VALIDACIÓN 4: Controladores de motores
        if self._controllers_ready_getter is not None:
            try:
                controllers_ready = self._controllers_ready_getter()
                logger.info("[MicroscopyService] Verificando controladores: %s", controllers_ready)
                if not controllers_ready:
                    msg = "❌ Error: Se requieren controladores para ambos motores"
                    logger.error("[MicroscopyService] %s", msg)
                    self.status_changed.emit(msg)
                    return False
                logger.info("[MicroscopyService] ✓ Controladores listos: OK")
            except Exception as e:
                logger.error(
                    "[MicroscopyService] ❌ Error verificando controladores listos: %s", e
                )
                return False
        else:
            logger.info("[MicroscopyService] ⚠️ No hay getter de controladores (opcional)")

        # Guardar configuración
        self._microscopy_config = config
        
        # Delays
        self._delay_before_ms = int(config.get('delay_before', 2.0) * 1000)
        self._delay_after_ms = int(config.get('delay_after', 0.2) * 1000)

        # Modo de aprendizaje
        learning_mode = bool(config.get('learning_mode', True))
        learning_target = int(config.get('learning_target', 50))

        # Iniciar estado usando StateManager
        self._state_manager.start(
            trajectory=list(trajectory),
            learning_mode=learning_mode,
            learning_target=learning_target
        )

        total = self._state_manager.total_points
        self.status_changed.emit(f"Iniciando microscopia: {total} puntos")
        self.status_changed.emit(
            f"Delay antes: {self._delay_before_ms}ms, Delay despues: {self._delay_after_ms}ms"
        )
        logger.info(
            "[MicroscopyService] Microscopía: %d puntos, delay_before=%dms, delay_after=%dms",
            total,
            self._delay_before_ms,
            self._delay_after_ms,
        )

        # Notificar progreso inicial
        self.progress_changed.emit(0, total)

        # FASE 1: Pasar trayectoria COMPLETA a TestService UNA SOLA VEZ
        if not self._test_service:
            logger.error("[MicroscopyService] TestService no disponible")
            self.status_changed.emit("❌ Error: TestService no disponible")
            return False
        
        # Conectar señal para recibir notificación cuando llegue a cada punto
        try:
            self._test_service.trajectory_point_reached.disconnect(self._on_test_point_reached)
        except:
            pass
        self._test_service.trajectory_point_reached.connect(self._on_test_point_reached)
        
        # Iniciar trayectoria completa (TestService maneja TODO el control)
        # pause_s reducido a 0.1s porque solo necesita settling, no operaciones
        success = self._test_service.start_trajectory(
            list(trajectory),
            tolerance_um=self._trajectory_tolerance,
            pause_s=0.1  # Solo settling, MicroscopyService controla timing real
        )
        
        if not success:
            logger.error("[MicroscopyService] Error iniciando trayectoria completa en TestService")
            self.status_changed.emit("❌ Error iniciando trayectoria")
            return False
        
        logger.info("[MicroscopyService] ✅ Trayectoria completa iniciada: %d puntos", total)
        return True

    def stop_microscopy(self) -> None:
        """Detiene la microscopia automatizada."""
        if not self._state_manager.is_active:
            return

        logger.info("[MicroscopyService] === DETENIENDO MICROSCOPIA ===")
        self._state_manager.stop()

        if self._is_dual_control_active and self._is_dual_control_active():
            self._stop_dual_control()

        self.status_changed.emit("Microscopia detenida por usuario")
        self.stopped.emit()

    def is_running(self) -> bool:
        """Indica si hay una secuencia de microscopía activa."""
        return self._state_manager.is_active

    # ------------------------------------------------------------------
    # Flujo interno de microscopia
    # ------------------------------------------------------------------
    def _move_to_point(self) -> None:
        """PASO 1: Mueve al punto actual usando TestService (mismo algoritmo probado)."""
        if not self._state_manager.is_active:
            return
        
        # Verificar pausa
        if self._state_manager.is_paused:
            # Esperar 500ms y volver a verificar
            QTimer.singleShot(500, self._move_to_point)
            return

        if self._state_manager.current_point >= self._state_manager.total_points:
            self._finish_microscopy()
            return

        point = self._state_manager.get_current_target()
        if point is None:
            return
            
        x_target = point[0]
        y_target = point[1]

        n = self._state_manager.current_point + 1
        total = self._state_manager.total_points
        self.status_changed.emit(
            f"[{n}/{total}] Moviendo a X={x_target:.1f}, Y={y_target:.1f} um"
        )
        logger.info(
            "[MicroscopyService] Punto %d: (%.1f, %.1f) - usando TestService",
            n,
            x_target,
            y_target,
        )

        # CRÍTICO: Usar TestService para mover (algoritmo de control probado)
        if self._test_service:
            # Crear trayectoria de 1 punto para que TestService lo maneje
            single_point_trajectory = [(x_target, y_target)]
            
            # Conectar señal para saber cuándo llegó
            try:
                self._test_service.trajectory_point_reached.disconnect(self._on_test_point_reached)
            except:
                pass
            
            self._test_service.trajectory_point_reached.connect(self._on_test_point_reached)
            
            # Iniciar movimiento con TestService
            # Usamos una pausa mínima aquí (0.1s) porque nosotros manejamos el delay_before después
            success = self._test_service.start_trajectory(
                single_point_trajectory,
                tolerance_um=self._trajectory_tolerance,
                pause_s=0.1
            )
            
            if not success:
                logger.error("[MicroscopyService] Error iniciando movimiento con TestService")
                self._advance_point()
        else:
            logger.warning("[MicroscopyService] TestService no disponible, usando método legacy")
            self._move_to_point_legacy()
    
    def _on_test_point_reached(self, index: int, x: float, y: float, status: str):
        """Callback cuando TestService alcanza un punto.
        
        TestService YA está PAUSADO (esperando comando explícito).
        Solo ejecutamos delay de usuario y captura.
        """
        if not self._state_manager.is_active:
            return
        
        n = index + 1
        total = self._state_manager.total_points
        logger.info(f"[MicroscopyService] Punto {n}/{total} alcanzado: ({x:.1f}, {y:.1f}) {status}")
        logger.info(f"[MicroscopyService] TestService PAUSADO - ejecutando detección")
        
        # Delay de usuario (para eliminar vibración)
        if self._delay_before_ms > 0:
            self.status_changed.emit(
                f"[{n}/{total}] Posición alcanzada - Esperando {self._delay_before_ms}ms..."
            )
            QTimer.singleShot(self._delay_before_ms, self._capture)
        else:
            self._capture()

    def _capture(self) -> None:
        """PASO 3: Captura la imagen (con o sin autofoco)."""
        if not self._state_manager.is_active:
            return

        if not self._microscopy_config:
            return

        use_autofocus = bool(self._microscopy_config.get('autofocus_enabled', False))
        cfocus_enabled = bool(self._cfocus_enabled_getter()) if self._cfocus_enabled_getter else False

        logger.info(f"[MicroscopyService] _capture check: use_autofocus={use_autofocus}, cfocus_enabled={cfocus_enabled}")

        if use_autofocus and cfocus_enabled:
            self._capture_with_autofocus()
            return
        
        if use_autofocus and not cfocus_enabled:
            logger.warning("[MicroscopyService] Autofoco habilitado pero C-Focus NO disponible/habilitado")
            self.status_changed.emit("⚠️ C-Focus deshabilitado - Saltando autofoco")

        # Captura normal sin autofoco
        self.status_changed.emit("  Capturando imagen (Sin Autofoco)...")
        success = False
        if self._capture_microscopy_image:
            success = self._capture_microscopy_image(self._microscopy_config, self._state_manager.current_point)

        if success:
            logger.info(
                "[MicroscopyService] Imagen %d capturada",
                self._state_manager.current_point + 1,
            )
        else:
            self.status_changed.emit(
                f"  ERROR: Fallo captura imagen {self._state_manager.current_point + 1}"
            )
            logger.error(
                "[MicroscopyService] Fallo captura imagen %d",
                self._state_manager.current_point + 1,
            )

        # Actualizar progreso y avanzar
        self._advance_point()

    def _capture_with_autofocus(self) -> None:
        """Captura RÁPIDA con detección y enfoque simple (NO escaneo completo)."""
        if not self._state_manager.is_active:
            return

        current_idx = self._state_manager.current_point
        total = self._state_manager.total_points
        n = current_idx + 1

        logger.info(f"[MicroscopyService] 🔍 Iniciando captura con autofoco para punto {n}/{total}")
        self.status_changed.emit(f"[{n}/{total}] 🔍 Detectando objetos...")

        # Capturar frame actual
        frame = self._get_current_frame()
        if frame is None:
            logger.warning("[MicroscopyService] No se pudo obtener frame de cámara")
            self.status_changed.emit("⚠️ Error: No hay frame de cámara")
            self._advance_point()
            self._resume_test_service()
            return

        # Convertir uint16 -> uint8
        if frame.dtype == np.uint16:
            frame_max = frame.max()
            if frame_max > 0:
                frame_uint8 = (frame / frame_max * 255).astype(np.uint8)
            else:
                frame_uint8 = np.zeros_like(frame, dtype=np.uint8)
        else:
            frame_uint8 = frame.astype(np.uint8)

        if len(frame_uint8.shape) == 2:
            frame_bgr = cv2.cvtColor(frame_uint8, cv2.COLOR_GRAY2BGR)
        else:
            frame_bgr = frame_uint8

        self.status_changed.emit("🔍 Detectando objetos...")
        result = self._smart_focus_scorer.assess_image(frame_bgr)
        all_objects = result.objects if result.objects else []

        min_area, max_area = self._get_area_range() if self._get_area_range else (0, 1e9)
        
        # DEBUG: Mostrar área de objetos detectados
        logger.info(f"[MicroscopyService] Objetos detectados por U2-Net: {len(all_objects)}")
        for i, obj in enumerate(all_objects[:5]):  # Mostrar solo primeros 5
            logger.info(f"  Objeto {i+1}: área={obj.area:.0f} px")

        # IMPORTANTE: SIEMPRE aplicar filtros de área y morfológicos
        # El modo aprendizaje solo sirve para CONFIRMAR objetos válidos, NO para aceptar basura
        min_circularity = self._smart_focus_scorer.min_circularity if self._smart_focus_scorer else 0.45
        min_aspect_ratio = self._smart_focus_scorer.min_aspect_ratio if self._smart_focus_scorer else 0.4
        
        logger.info(f"[MicroscopyService] Filtros activos: área=[{min_area}-{max_area}]px, circ≥{min_circularity:.2f}, aspect≥{min_aspect_ratio:.2f}")
        
        objects_filtered = []
        for obj in all_objects:
            # Filtro de área
            if not (min_area <= obj.area <= max_area):
                logger.debug(f"[MicroscopyService] ❌ Objeto rechazado por área: {obj.area:.0f}px (rango: {min_area}-{max_area})")
                continue
            
            # Obtener circularidad del objeto
            if hasattr(obj, 'circularity') and obj.circularity > 0:
                circularity = obj.circularity
            else:
                x, y, w, h = obj.bounding_box
                perimeter = 2 * (w + h)
                circularity = (4 * np.pi * obj.area) / (perimeter ** 2) if perimeter > 0 else 0
            
            # Filtro de circularidad (rechazar manchas irregulares)
            if circularity < min_circularity:
                logger.debug(f"[MicroscopyService] ❌ Objeto rechazado por circularidad: área={obj.area:.0f}px, circ={circularity:.2f} < {min_circularity:.2f}")
                continue
            
            # Filtro de aspect ratio (rechazar manchas muy alargadas)
            x, y, w, h = obj.bounding_box
            aspect_ratio = float(w) / float(h) if h > 0 else 1.0
            if aspect_ratio > 1.0:
                aspect_ratio = 1.0 / aspect_ratio
            
            if aspect_ratio < min_aspect_ratio:
                logger.debug(f"[MicroscopyService] ❌ Objeto rechazado por aspect ratio: área={obj.area:.0f}px, aspect={aspect_ratio:.2f} < {min_aspect_ratio:.2f}")
                continue
            
            # Objeto válido
            objects_filtered.append(obj)
            logger.info(f"[MicroscopyService] ✓ Objeto válido: área={obj.area:.0f}px, circ={circularity:.2f}, aspect={aspect_ratio:.2f}")
        
        objects = objects_filtered
        n_objects = len(objects)
        if n_objects == 0:
            self.status_changed.emit(
                f"[{n}/{total}]   ⚠️ Sin objetos en rango [{min_area}-{max_area}] px - saltando punto"
            )
            logger.info(
                "[MicroscopyService] Punto %d: sin objetos en rango (detectados: %d)",
                self._state_manager.current_point,
                len(all_objects),
            )
            # FASE 3: Comando explícito para avanzar (sin objetos)
            self._state_manager.advance_point()
            self._state_manager.reset_position_checks()
            self.progress_changed.emit(self._state_manager.current_point, self._state_manager.total_points)
            
            # Reanudar TestService para que avance al siguiente punto
            if self._test_service:
                logger.info(f"[MicroscopyService] [{n}/{total}] Sin objetos - avanzando a punto {n+1}")
                self.status_changed.emit(f"[{n}/{total}] ➡️  Avanzando a punto {n+1}/{total}")
                self._test_service.resume_trajectory()
            return
        
        # Objetos detectados
        logger.info(f"[MicroscopyService] [{n}/{total}] ✅ {n_objects} objeto(s) detectado(s)")
        self.status_changed.emit(f"[{n}/{total}] ✅ {n_objects} objeto(s) - iniciando autofoco")
        
        # Mostrar máscaras en ventana de cámara
        self._show_autofocus_masks(objects)
        
        # CRÍTICO: Emitir señal con objetos detectados para actualizar lista en ventana de cámara
        logger.info(f"[MicroscopyService] ✅ EMITIENDO detection_complete: {len(objects)} objetos")
        print(f"[MicroscopyService] ✅ EMITIENDO detection_complete: {len(objects)} objetos")
        self.detection_complete.emit(objects)
        logger.info(f"[MicroscopyService] Señal detection_complete emitida correctamente")

        # Enfocar solo en el objeto MÁS GRANDE
        largest_object = max(objects, key=lambda obj: obj.area)
        
        # Sistema de aprendizaje con confirmación (Assisted Labeling)
        # Si estamos en modo aprendizaje y no hemos alcanzado el objetivo
        if self._state_manager.learning_mode and not self._state_manager.learning_completed:
            try:
                logger.info(f"[MicroscopyService] 🎓 Modo aprendizaje activo: {self._state_manager.learning_count}/{self._state_manager.learning_target}")
                logger.info(f"[MicroscopyService] 📍 CHECKPOINT 1: Antes de status_changed.emit")
                self.status_changed.emit(f"❓ Confirmación requerida ({self._state_manager.learning_count + 1}/{self._state_manager.learning_target})")
                logger.info(f"[MicroscopyService] 📍 CHECKPOINT 2: Después de status_changed.emit")
                
                # CRÍTICO: ACTIVAR BRAKE para evitar drift durante confirmación
                # Los motores deben estar completamente detenidos mientras el usuario decide
                if self._send_command:
                    logger.info("[MicroscopyService] 🛑 Activando BRAKE durante confirmación de usuario")
                    self._send_command('B')  # Freno activo
                    time.sleep(0.05)  # Dar tiempo para que el freno se active
                    self._send_command('A,0,0')  # PWM a 0
                    logger.info("[MicroscopyService] ✅ BRAKE activado correctamente")
                else:
                    logger.warning("[MicroscopyService] ⚠️ send_command no disponible - no se puede activar BRAKE")
                
                logger.info(f"[MicroscopyService] 📍 CHECKPOINT 3: Después de BRAKE")
                
                # Guardar estado para reanudar
                self._pending_object = largest_object
                self._pending_frame = frame_bgr
                
                logger.info(f"[MicroscopyService] 📍 CHECKPOINT 4: Estado guardado")
                
                # SIEMPRE emitir señal para que la UI muestre el diálogo
                # Pasamos: frame, objeto, clase sugerida, confianza, progreso actual, objetivo
                logger.info("[MicroscopyService] 📢 EMITIENDO learning_confirmation_requested")
                logger.info(f"[MicroscopyService]   - Frame shape: {frame_bgr.shape}")
                logger.info(f"[MicroscopyService]   - Objeto área: {largest_object.area:.0f} px")
                logger.info(f"[MicroscopyService]   - Progreso: {self._state_manager.learning_count + 1}/{self._state_manager.learning_target}")
                
                logger.info(f"[MicroscopyService] 📍 CHECKPOINT 5: Antes de emit learning_confirmation_requested")
                self.learning_confirmation_requested.emit(
                    frame_bgr,
                    largest_object,
                    self._microscopy_config.get('class_name', 'object'),
                    getattr(largest_object, 'confidence', 0.0),
                    self._state_manager.learning_count + 1,
                    self._state_manager.learning_target,
                )
                logger.info("[MicroscopyService] ✅ Señal learning_confirmation_requested emitida correctamente")
                logger.info(f"[MicroscopyService] 📍 CHECKPOINT 6: Después de emit - RETORNANDO")
                # DETENER FLUJO AQUÍ - Se reanudará cuando el usuario responda vía confirm_learning_step
                return
            except Exception as e:
                logger.error(f"[MicroscopyService] ❌ ERROR CRÍTICO en bloque de aprendizaje: {e}")
                logger.error(f"[MicroscopyService] ❌ Tipo de error: {type(e).__name__}")
                import traceback
                logger.error(f"[MicroscopyService] ❌ Traceback:\n{traceback.format_exc()}")
                # Continuar con captura automática en caso de error
                logger.warning("[MicroscopyService] ⚠️ Continuando con captura automática debido a error")

        # Si no es aprendizaje o ya pasamos el target, continuar automáticamente
        self._proceed_with_capture(largest_object)

    def confirm_learning_step(self, user_accepted, user_class: str = None) -> None:
        """
        Slot para recibir la respuesta del usuario desde la UI.

        user_accepted puede ser:
        - bool: aceptar/rechazar el ROI detectado automáticamente
        - dict: {'accepted': bool, 'replace': bool, 'custom_rois': [(x,y,w,h), ...]}
        """
        if not self._state_manager.is_active or self._pending_object is None:
            return

        # Normalizar respuesta
        accepted = False
        replace = False
        custom_rois = []
        if isinstance(user_accepted, dict):
            accepted = bool(user_accepted.get('accepted', True))
            replace = bool(user_accepted.get('replace', False))
            custom_rois = list(user_accepted.get('custom_rois', []))
        else:
            accepted = bool(user_accepted)

        if not accepted:
            logger.info("[MicroscopyService] Aprendizaje: Usuario rechazó objeto")
            self._advance_point()
            # Limpiar estado
            self._pending_object = None
            self._pending_frame = None
            return

        # Aceptado
        self._state_manager.increment_image_counter()
        logger.info(f"[MicroscopyService] Aprendizaje: Usuario aceptó objeto {self._state_manager.learning_count}/{self._state_manager.learning_target}")
        if user_class:
            logger.info(f"[MicroscopyService] Clase confirmada: {user_class}")
        
        # CRÍTICO: PAUSAR control XY ANTES de proceder con captura
        # NOTA: El BRAKE ya fue activado en _capture_with_autofocus, aquí solo pausamos el control dual
        if self._test_service:
            logger.info("[MicroscopyService] ⏸️  Pausando control XY ANTES de captura")
            self._test_service.pause_dual_control()
            time.sleep(0.3)  # Dar tiempo para que el control se pause completamente

        # Si hay ROIs manuales y se debe reemplazar la segmentación detectada
        if replace and custom_rois and self._pending_frame is not None:
            try:
                self._save_manual_rois_training_data(self._pending_frame, custom_rois, user_class)
            except Exception as e:
                logger.error(f"[MicroscopyService] Error guardando datos de entrenamiento manual: {e}")

            # Usar el PRIMER ROI manual para captura rápida
            x, y, w, h = custom_rois[0]
            area_est = max(1, int(w * h))
            # Crear objeto temporal compatible
            class _ManualObj:
                pass
            temp_obj = _ManualObj()
            temp_obj.bounding_box = (int(x), int(y), int(w), int(h))
            temp_obj.area = area_est
            temp_obj.contour = None
            try:
                if self._smart_focus_scorer is not None:
                    temp_obj.focus_score = self._smart_focus_scorer.calculate_sharpness(self._pending_frame, temp_obj.bounding_box)
            except Exception:
                temp_obj.focus_score = 0.0

            self._proceed_with_capture(temp_obj)
        else:
            # Continuar con el objeto detectado automáticamente
            self._proceed_with_capture(self._pending_object)

        # Limpiar estado
        self._pending_object = None
        self._pending_frame = None

    def _save_manual_rois_training_data(self, frame_bgr: np.ndarray, rois: list, user_class: Optional[str]):
        """Extrae parámetros de ROIs manuales y los guarda para entrenamiento.

        Crea/append un archivo JSONL 'learning_labels.jsonl' en la carpeta de guardado.
        Cada línea incluye: punto, bbox, área, circularidad, aspect_ratio, sharpness, clase, timestamp.
        """
        if self._microscopy_config is None:
            return
        save_folder = self._microscopy_config.get('save_folder', '.')
        os.makedirs(save_folder, exist_ok=True)
        out_path = os.path.join(save_folder, 'learning_labels.jsonl')

        # Asegurar formato uint8 para análisis
        frame = frame_bgr
        if frame.dtype == np.uint16:
            maxv = frame.max()
            frame = (frame / maxv * 255).astype(np.uint8) if maxv > 0 else frame.astype(np.uint8)

        h_img, w_img = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame

        records = []
        for (x, y, w, h) in rois:
            x = max(0, min(int(x), w_img - 1))
            y = max(0, min(int(y), h_img - 1))
            w = max(1, min(int(w), w_img - x))
            h = max(1, min(int(h), h_img - y))

            crop = gray[y:y+h, x:x+w]
            # Segmentación simple dentro del ROI para estimar contorno real
            try:
                blurred = cv2.GaussianBlur(crop, (5, 5), 0)
                _, bin_otsu = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
                contours, _ = cv2.findContours(bin_otsu, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    cnt = max(contours, key=cv2.contourArea)
                    area = float(cv2.contourArea(cnt))
                    perim = float(cv2.arcLength(cnt, True))
                    circularity = float((4 * np.pi * area) / (perim ** 2)) if perim > 0 else 0.0
                else:
                    area = float(w * h)
                    circularity = 0.0
            except Exception:
                area = float(w * h)
                circularity = 0.0

            aspect = float(w) / float(h) if h > 0 else 1.0
            if aspect > 1.0:
                aspect = 1.0 / aspect

            try:
                sharp = float(self._smart_focus_scorer.calculate_sharpness(frame, (x, y, w, h))) if self._smart_focus_scorer else 0.0
            except Exception:
                sharp = 0.0

            record = {
                'point_index': int(self._state_manager.current_point),
                'bbox': [int(x), int(y), int(w), int(h)],
                'area_px': float(area),
                'circularity': float(circularity),
                'aspect_ratio': float(aspect),
                'sharpness': float(sharp),
                'class_name': str(user_class) if user_class else str(self._microscopy_config.get('class_name', 'object')),
                'timestamp': datetime.now().isoformat(timespec='seconds')
            }
            records.append(record)

        with open(out_path, 'a', encoding='utf-8') as f:
            for rec in records:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        logger.info(f"[MicroscopyService] Guardados {len(records)} ROIs manuales en {out_path}")

    def _proceed_with_capture(self, largest_object) -> None:
        """Continúa con la captura después de la confirmación (o si es automático)."""
        n_captures = self._autofocus_service.n_captures if self._autofocus_service.n_captures % 2 == 1 else 3
        
        self.status_changed.emit(
            f"  ✓ Objeto detectado: {largest_object.area:.0f} px - capturando {n_captures} imágenes..."
        )
        logger.info(
            "[MicroscopyService] Punto %d: objeto detectado (área=%.0f px) - captura rápida %d imgs",
            self._state_manager.current_point,
            largest_object.area,
            n_captures,
        )
        
        # ============================================================================
        # CRÍTICO: PAUSAR control dual XY ANTES de captura multifocal
        # Los motores XY NO deben moverse durante el autofoco Z
        # ============================================================================
        if self._test_service:
            logger.info("[MicroscopyService] ⏸️  PAUSANDO control dual XY ANTES de captura multifocal")
            self._test_service.pause_dual_control()
            time.sleep(0.3)  # Dar tiempo para que el control se pause completamente
            logger.info("[MicroscopyService] ✅ Control dual XY PAUSADO - motores XY bloqueados")
        else:
            logger.error("[MicroscopyService] ❌ test_service NO disponible - NO se puede pausar control XY")
        
        # CAPTURA RÁPIDA: N imágenes multi-focales
        # NOTA: Ya NO actualizamos ROI aquí porque causa que los motores se muevan
        # El ROI ya fue detectado correctamente en _capture_with_autofocus
        self._quick_capture_multifocal(largest_object)
    
    def _quick_capture_multifocal(self, obj) -> None:
        """
        Captura RÁPIDA de N imágenes multi-focales (configurado por usuario).
        
        Proceso:
        1. Encuentra BPoF con búsqueda rápida local (±5µm desde posición actual)
        2. Captura N imágenes centradas en BPoF con offsets (según n_captures configurado)
        3. SIEMPRE vuelve a Z medio (centro calibrado)
        
        NOTA: El control XY YA está PAUSADO antes de llamar a este método.
        """
        # Control XY ya está pausado en _proceed_with_capture
        logger.info("[MicroscopyService] 🔒 Iniciando captura multifocal con control XY PAUSADO")
        
        try:
            cfocus = self._autofocus_service.cfocus_controller
            bbox = obj.bounding_box
            contour = getattr(obj, 'contour', None)
            
            # Obtener posición actual y centro calibrado
            z_current = cfocus.read_z()
            calib_info = cfocus.get_calibration_info()
            
            if not calib_info['is_calibrated']:
                logger.error("[MicroscopyService] C-Focus no calibrado, no se puede hacer captura rápida")
                self._advance_point()
                return
            
            z_center_hw = calib_info['z_center']
            z_min_hw = calib_info['z_min']
            z_max_hw = calib_info['z_max']
            
            logger.info(f"[MicroscopyService] Captura rápida: Z actual={z_current:.2f}µm, Z centro={z_center_hw:.2f}µm")
        
            # PASO 1: Búsqueda de BPoF usando configuración de autofoco
            # Si use_full_range=True, escanea TODO el rango calibrado (0-80µm)
            # Si use_full_range=False, escanea ±z_scan_range desde posición actual
            
            use_full_range = self._autofocus_service.use_full_range if self._autofocus_service else False
            
            if use_full_range:
                # ESCANEO COMPLETO: usar TODO el rango calibrado
                z_search_min = z_min_hw
                z_search_max = z_max_hw
                search_range_total = z_max_hw - z_min_hw
                logger.info(f"[MicroscopyService] ESCANEO COMPLETO: {z_search_min:.2f} → {z_search_max:.2f}µm (rango: {search_range_total:.2f}µm)")
            else:
                # ESCANEO LOCAL: ±z_scan_range desde posición actual
                search_range = self._autofocus_service.z_scan_range if self._autofocus_service else 5.0
                z_search_min = max(z_min_hw, z_current - search_range)
                z_search_max = min(z_max_hw, z_current + search_range)
                search_range_total = z_search_max - z_search_min
                logger.info(f"[MicroscopyService] Escaneo local: {z_search_min:.2f} - {z_search_max:.2f}µm (±{search_range:.2f}µm desde Z={z_current:.2f}µm)")
            
            search_step = self._autofocus_service.z_step_coarse if self._autofocus_service else 0.5
            
            best_z = z_current
            best_score = 0.0
            
            z = z_search_min
            n_steps = int((z_search_max - z_search_min) / search_step) + 1
            step_count = 0
            
            while z <= z_search_max:
                step_count += 1
                cfocus.move_z(z)
                time.sleep(0.05)  # 50ms estabilización
                
                # Mantener la UI receptiva durante el escaneo
                try:
                    QCoreApplication.processEvents()
                except Exception:
                    pass
                
                frame = self._get_current_frame()
                if frame is not None:
                    score = self._autofocus_service._get_stable_score(bbox, contour, n_samples=1)
                    if score > best_score:
                        best_z = z
                        best_score = score
                    
                    # Mensaje de progreso en línea única
                    progress_pct = (step_count / n_steps) * 100
                    distance_traveled = z - z_search_min
                    msg = f"[MicroscopyService] SCAN: {distance_traveled:.2f}/{search_range_total:.2f}µm ({progress_pct:.1f}%) | Z={z:.2f}µm | Score={score:.1f} | Best={best_score:.1f}@{best_z:.2f}µm"
                    print(msg, end='\r', flush=True)
                    logger.debug(msg)
                
                z += search_step
            
            # Nueva línea después del progreso
            print()
            logger.info(f"[MicroscopyService] BPoF encontrado: Z={best_z:.2f}µm, Score={best_score:.1f} (recorrido: {search_range_total:.2f}µm)")
        
            # PASO 2: Capturar 3 imágenes (BPoF, +offset, -offset)
            offset_z = self._autofocus_service.z_step_coarse  # Usar paso coarse como offset (ej: 0.5µm)
            
            z_positions = [
                best_z,                    # BPoF (centro)
                min(z_max_hw, best_z + offset_z),  # +offset (arriba)
                max(z_min_hw, best_z - offset_z)   # -offset (abajo)
            ]
            
            frames = []
            scores = []
            
            for i, z_pos in enumerate(z_positions):
                cfocus.move_z(z_pos)
                time.sleep(0.1)  # 100ms estabilización para captura
                try:
                    QCoreApplication.processEvents()
                except Exception:
                    pass
                
                frame = self._get_current_frame()
                if frame is not None:
                    frames.append(frame.copy())
                    score = self._autofocus_service._get_stable_score(bbox, contour, n_samples=1)
                    scores.append(score)
                    
                    label = "BPoF" if i == 0 else f"{'+' if i == 1 else '-'}{offset_z}µm"
                    logger.info(f"[MicroscopyService] Captura {i+1}/3 ({label}): Z={z_pos:.2f}µm, S={score:.1f}")
            
            # PASO 3: Guardar las 3 imágenes
            success = self._save_3images(frames, z_positions, scores, best_z, self._state_manager.current_point)
            
            # PASO 4: SIEMPRE volver a Z medio (centro calibrado)
            logger.info(f"[MicroscopyService] Volviendo a Z medio: {z_center_hw:.2f}µm")
            cfocus.move_z(z_center_hw)
            time.sleep(0.1)
            try:
                QCoreApplication.processEvents()
            except Exception:
                pass
            
            z_final = cfocus.read_z()
            if z_final is not None:
                logger.info(f"[MicroscopyService] ✓ Posición final: Z={z_final:.2f}µm (centro calibrado)")
            else:
                logger.warning("[MicroscopyService] ⚠️ No se pudo leer posición Z final (C-Focus desconectado?)")
            
            if success:
                self.status_changed.emit(f"  ✓ 3 imágenes guardadas - vuelto a Z medio")
            else:
                self.status_changed.emit(f"  ⚠️ Error guardando imágenes")
            
            # FASE 3: Comando explícito para avanzar (después de captura)
            self._state_manager.advance_point()
            self.progress_changed.emit(self._state_manager.current_point, self._state_manager.total_points)
            
            # Delay de usuario (post-captura)
            if self._delay_after_ms > 0:
                time.sleep(self._delay_after_ms / 1000.0)
            
            # DECISIÓN: SIEMPRE reanudar automáticamente después de captura
            # El modo aprendizaje solo pausa ANTES de capturar (para confirmar)
            # Una vez confirmado y capturado, debe continuar automáticamente
            logger.info("[MicroscopyService] ✅ Captura completada - reanudando trayectoria automáticamente")
            if self._test_service:
                self._test_service.resume_trajectory()
        
        finally:
            # PASO FINAL: REACTIVAR control dual XY
            if self._test_service:
                logger.info("[MicroscopyService] ▶️  Reactivando control dual XY")
                self._test_service.resume_dual_control()
    
    def _save_3images(self, frames: list, z_positions: list, scores: list, best_z: float, image_index: int) -> bool:
        """
        Guarda las 3 imágenes capturadas (BPoF + offsets).
        
        Args:
            frames: Lista de 3 frames capturados
            z_positions: Lista de 3 posiciones Z
            scores: Lista de 3 scores de enfoque
            best_z: Posición Z del BPoF
            image_index: Índice de la imagen base
            
        Returns:
            bool: True si se guardaron correctamente todas las imágenes
        """
        if not frames or len(frames) != 3 or self._microscopy_config is None:
            logger.error(f"[MicroscopyService] Error: se esperaban 3 frames, recibidos {len(frames) if frames else 0}")
            return False
        
        save_folder = self._microscopy_config.get('save_folder', '.')
        class_name = self._microscopy_config.get('class_name', 'sample')
        
        logger.info(f"[MicroscopyService] Guardando 3 imágenes para punto {image_index + 1}")
        
        all_success = True
        labels = ['BPoF', '+offset', '-offset']
        
        for i, (frame, z_pos, score, label) in enumerate(zip(frames, z_positions, scores, labels)):
            try:
                frame_copy = frame.copy()
                
                # Normalizar uint16 a uint8 si es necesario
                if frame_copy.dtype == np.uint16:
                    if frame_copy.max() > 0:
                        frame_copy = (frame_copy / frame_copy.max() * 255).astype(np.uint8)
                    else:
                        frame_copy = frame_copy.astype(np.uint8)
                
                # Generar nombre de archivo con sufijo de índice focal
                # Ejemplo: sample_0001_f0.png (BPoF), sample_0001_f1.png (+offset), sample_0001_f2.png (-offset)
                filename = f"{class_name}_{image_index + 1:04d}_f{i}.png"
                filepath = os.path.join(save_folder, filename)
                
                # Guardar imagen
                cv2.imwrite(filepath, frame_copy)
                
                offset_str = f"(Z={z_pos:.1f}µm, offset={z_pos - best_z:+.1f}µm)" if i > 0 else f"(Z={z_pos:.1f}µm)"
                logger.info(f"[MicroscopyService]   {label}: {filename} {offset_str}, S={score:.1f}")
                
            except Exception as e:
                logger.error(f"[MicroscopyService] Error guardando imagen {i}: {e}")
                all_success = False
        
        return all_success

    def _capture_without_autofocus_fallback(self) -> None:
        """Captura sencilla usada como fallback cuando no hay autofoco disponible."""
        success = False
        if self._capture_microscopy_image and self._microscopy_config:
            success = self._capture_microscopy_image(self._microscopy_config, self._state_manager.current_point)

        if success:
            logger.info(
                "[MicroscopyService] Imagen %d capturada (fallback)",
                self._state_manager.current_point + 1,
            )
        else:
            self.status_changed.emit(
                f"  ERROR: Fallo captura imagen {self._state_manager.current_point + 1} (fallback)"
            )
            logger.error(
                "[MicroscopyService] Fallo captura imagen %d (fallback)",
                self._state_manager.current_point + 1,
            )

        self._advance_point()

    def handle_autofocus_complete(self, results: list = None) -> None:
        """Debe llamarse cuando AutofocusService completa el autofoco en microscopia.

        Usa el frame ya capturado durante el autofoco (en BPoF) para guardar la imagen.
        
        Args:
            results: Lista de FocusResult con frames ya capturados en BPoF
        """
        if not self._state_manager.is_active:
            return

        self.status_changed.emit("📸 Guardando imagen con BPoF...")
        success = False
        
        # Usar los frames ya capturados durante el autofoco (evita desenfoque)
        if results and len(results) > 0:
            result = results[0]
            
            # Guardar todas las capturas multi-focales si existen
            if result.frames and len(result.frames) > 0:
                success = self._save_multifocal_frames(result, self._state_manager.current_point)
            # Fallback: guardar solo BPoF si no hay capturas multi-focales
            elif result.frame is not None:
                success = self._save_autofocus_frame(result, self._state_manager.current_point)
                # También guardar frame alternativo si existe (legacy)
                if result.frame_alt is not None:
                    self._save_autofocus_frame_alt(result, self._state_manager.current_point)
        else:
            # Fallback: capturar frame actual (puede estar desenfocado)
            logger.warning("[MicroscopyService] No hay frame en resultado de autofoco, usando frame actual")
            if self._capture_microscopy_image and self._microscopy_config:
                success = self._capture_microscopy_image(self._microscopy_config, self._state_manager.current_point)

        if success:
            logger.info(
                "[MicroscopyService] Imagen %d guardada con autofoco (BPoF)",
                self._state_manager.current_point + 1,
            )
        else:
            self.status_changed.emit(
                f"  ERROR: Fallo guardar imagen {self._state_manager.current_point + 1} tras autofoco"
            )
            logger.error(
                "[MicroscopyService] Fallo guardar imagen %d tras autofoco",
                self._state_manager.current_point + 1,
            )

        self._advance_point()
    
    def _save_autofocus_frame(self, result, image_index: int) -> bool:
        """Guarda el frame capturado durante el autofoco (BPoF).
        
        Args:
            result: FocusResult con el frame ya capturado en BPoF
            image_index: Índice de la imagen
            
        Returns:
            bool: True si se guardó correctamente
        """
        if result.frame is None or self._microscopy_config is None:
            return False
        
        try:
            frame = result.frame.copy()
            
            # Normalizar uint16 a uint8 si es necesario
            if frame.dtype == np.uint16:
                if frame.max() > 0:
                    frame = (frame / frame.max() * 255).astype(np.uint8)
                else:
                    frame = frame.astype(np.uint8)
            
            # Obtener configuración
            save_folder = self._microscopy_config.get('save_folder', '.')
            class_name = self._microscopy_config.get('class_name', 'sample')
            
            # Generar nombre de archivo
            filename = f"{class_name}_{image_index + 1:04d}.png"
            filepath = os.path.join(save_folder, filename)
            
            # Guardar imagen
            cv2.imwrite(filepath, frame)
            logger.info(f"[MicroscopyService] Frame BPoF guardado: {filename} (Z={result.z_optimal:.1f}µm, S={result.focus_score:.1f})")
            
            return True
            
        except Exception as e:
            logger.error(f"[MicroscopyService] Error guardando frame BPoF: {e}")
            return False
    
    def _save_multifocal_frames(self, result, image_index: int) -> bool:
        """Guarda todas las capturas multi-focales (N imágenes).
        
        Args:
            result: FocusResult con lista de frames multi-focales
            image_index: Índice de la imagen base
            
        Returns:
            bool: True si se guardaron correctamente todas las capturas
        """
        if not result.frames or len(result.frames) == 0 or self._microscopy_config is None:
            return False
        
        save_folder = self._microscopy_config.get('save_folder', '.')
        class_name = self._microscopy_config.get('class_name', 'sample')
        n_captures = len(result.frames)
        
        logger.info(f"[MicroscopyService] Guardando {n_captures} capturas multi-focales para imagen {image_index + 1}")
        
        all_success = True
        for i, (frame, z_pos, score) in enumerate(zip(result.frames, result.z_positions, result.focus_scores)):
            try:
                frame_copy = frame.copy()
                
                # Normalizar uint16 a uint8 si es necesario
                if frame_copy.dtype == np.uint16:
                    if frame_copy.max() > 0:
                        frame_copy = (frame_copy / frame_copy.max() * 255).astype(np.uint8)
                    else:
                        frame_copy = frame_copy.astype(np.uint8)
                
                # Generar nombre de archivo con sufijo de índice focal
                # Ejemplo: sample_0001_f0.png, sample_0001_f1.png (BPoF), sample_0001_f2.png
                filename = f"{class_name}_{image_index + 1:04d}_f{i}.png"
                filepath = os.path.join(save_folder, filename)
                
                # Guardar imagen
                cv2.imwrite(filepath, frame_copy)
                
                focus_label = "BPoF" if i == n_captures // 2 else f"offset={z_pos - result.z_optimal:+.1f}µm"
                logger.info(f"[MicroscopyService]   Frame {i+1}/{n_captures} ({focus_label}): {filename} (Z={z_pos:.1f}µm, S={score:.1f})")
                
            except Exception as e:
                logger.error(f"[MicroscopyService] Error guardando frame multi-focal {i}: {e}")
                all_success = False
        
        return all_success
    
    def _save_autofocus_frame_alt(self, result, image_index: int) -> bool:
        """Guarda el frame alternativo (ligeramente desenfocado).
        
        Args:
            result: FocusResult con el frame alternativo
            image_index: Índice de la imagen
            
        Returns:
            bool: True si se guardó correctamente
        """
        if result.frame_alt is None or self._microscopy_config is None:
            return False
        
        try:
            frame = result.frame_alt.copy()
            
            # Normalizar uint16 a uint8 si es necesario
            if frame.dtype == np.uint16:
                if frame.max() > 0:
                    frame = (frame / frame.max() * 255).astype(np.uint8)
                else:
                    frame = frame.astype(np.uint8)
            
            # Obtener configuración
            save_folder = self._microscopy_config.get('save_folder', '.')
            class_name = self._microscopy_config.get('class_name', 'sample')
            
            # Generar nombre de archivo con sufijo _alt
            filename = f"{class_name}_{image_index + 1:04d}_alt.png"
            filepath = os.path.join(save_folder, filename)
            
            # Guardar imagen
            cv2.imwrite(filepath, frame)
            logger.info(f"[MicroscopyService] Frame alternativo guardado: {filename} (Z={result.z_alt:.1f}µm, S={result.score_alt:.1f})")
            
            return True
            
        except Exception as e:
            logger.error(f"[MicroscopyService] Error guardando frame alternativo: {e}")
            return False

    def _advance_point(self) -> None:
        """OBSOLETO: Avanza al siguiente punto (legacy).
        
        NOTA: Con el nuevo protocolo, el avance se hace explícitamente
        llamando a TestService.resume_trajectory() después de cada operación.
        Este método se mantiene por compatibilidad con flujos legacy.
        """
        if not self._state_manager.is_active:
            return

        self._state_manager.advance_point()
        self.progress_changed.emit(self._state_manager.current_point, self._state_manager.total_points)
        
        # Delay de usuario (post-captura)
        if self._delay_after_ms > 0:
            time.sleep(self._delay_after_ms / 1000.0)
        
        # Reanudar TestService
        if self._test_service:
            logger.info("[MicroscopyService] _advance_point (legacy) - comandando avance")
            self._test_service.resume_trajectory(advance_to_next=True)
    
    def enable_learning_mode(self, enabled: bool = True, target_count: int = 50):
        """Activa/desactiva el modo de aprendizaje."""
        # Este método ya no es necesario - el learning mode se configura en start()
        # Mantenido por compatibilidad pero no hace nada
        logger.warning("[MicroscopyService] enable_learning_mode() está obsoleto - usar start_microscopy() con config")
    
    def set_paused(self, paused: bool):
        """
        Pausa/reanuda la microscopía manualmente (por usuario).
        
        IMPORTANTE: Esta es una pausa MANUAL, NO debe avanzar al siguiente punto.
        Solo debe reanudar el flujo en el punto actual.
        """
        if paused:
            self._state_manager.pause()
            logger.info(f"[MicroscopyService] 🛑 Microscopía PAUSADA MANUALMENTE en punto {self._state_manager.current_point + 1}")
        else:
            self._state_manager.resume()
            logger.info(f"[MicroscopyService] ▶️  Microscopía REANUDADA MANUALMENTE en punto {self._state_manager.current_point + 1}")
            # NO llamar a resume_trajectory() porque eso incrementa el índice
            # El usuario solo quiere reanudar en el punto actual, no avanzar
    
    def skip_current_point(self):
        """Salta el punto actual sin capturar."""
        logger.info(f"[MicroscopyService] Usuario solicitó saltar punto {self._state_manager.current_point}")
        self.status_changed.emit(f"⏭️ Punto {self._state_manager.current_point} saltado por usuario")
        # Limpiar máscaras antes de avanzar
        self.clear_masks.emit()
        # Usar StateManager para saltar
        self._state_manager.skip_current_point()
        self._move_to_point()
    
    def _show_autofocus_masks(self, objects):
        """Muestra máscaras de objetos detectados en ventana de cámara."""
        masks_data = []
        for obj in objects:
            masks_data.append({
                'bbox': obj.bounding_box,
                'area': obj.area,
                'score': getattr(obj, 'focus_score', 0),
                'is_focused': getattr(obj, 'is_focused', False)
            })
        self.show_masks.emit(masks_data)
        logger.info(f"[MicroscopyService] 🎯 Mostrando {len(masks_data)} máscaras de autofoco")
    
    def _clear_autofocus_masks(self):
        """Limpia las máscaras de autofoco de la ventana de cámara."""
        self.clear_masks.emit()
        logger.info("[MicroscopyService] 🧹 Máscaras de autofoco limpiadas")
    
    def _confirm_roi_for_learning(self, frame, obj, detection_result) -> bool:
        """Muestra diálogo de confirmación para aprendizaje."""
        try:
            from gui.dialogs import LearningConfirmationDialog
            
            if self._learning_dialog is None:
                self._learning_dialog = LearningConfirmationDialog()
            
            # Obtener máscara del objeto
            prob_map = detection_result.probability_map if detection_result else None
            mask = None
            if prob_map is not None:
                h, w = frame.shape[:2]
                prob_resized = cv2.resize(prob_map, (w, h))
                mask = (prob_resized > 0.3).astype(np.uint8) * 255
            
            # Mostrar diálogo
            response = self._learning_dialog.show_roi_for_confirmation(
                frame=frame,
                roi_bbox=obj.bounding_box,
                roi_mask=mask,
                area=obj.area,
                score=getattr(obj, 'focus_score', 0),
                current_count=self._learning_count,
                total_count=self._learning_target
            )
            
            return response if response is not None else True  # True por defecto
            
        except Exception as e:
            logger.error(f"[MicroscopyService] Error en confirmación de aprendizaje: {e}")
            return True  # Continuar por defecto si hay error
    
    def _save_roi_visualization(self, frame, obj, detection_result):
        """MEJORA 3: Guarda visualización del ROI para referencia."""
        try:
            if not self._microscopy_config:
                return
            
            save_folder = self._microscopy_config.get('save_folder', '')
            if not save_folder:
                return
            
            # Crear subcarpeta para visualizaciones
            viz_folder = os.path.join(save_folder, 'roi_visualizations')
            os.makedirs(viz_folder, exist_ok=True)
            
            # Dibujar ROI en el frame
            frame_viz = frame.copy()
            x, y, w, h = obj.bounding_box
            
            # Dibujar máscara si está disponible
            if detection_result and detection_result.probability_map is not None:
                prob_map = detection_result.probability_map
                h_frame, w_frame = frame.shape[:2]
                prob_resized = cv2.resize(prob_map, (w_frame, h_frame))
                mask = (prob_resized > 0.3).astype(np.uint8) * 255
                
                # Overlay verde semi-transparente
                overlay = frame_viz.copy()
                overlay[mask > 0] = [0, 255, 0]
                cv2.addWeighted(overlay, 0.3, frame_viz, 0.7, 0, frame_viz)
            
            # Bounding box verde
            cv2.rectangle(frame_viz, (x, y), (x + w, y + h), (0, 255, 0), 3)
            
            # Etiquetas
            area = obj.area
            score = getattr(obj, 'focus_score', 0)
            label = f"ROI: {area:.0f}px, S:{score:.1f}"
            cv2.putText(frame_viz, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                       0.7, (0, 255, 0), 2)
            
            # Guardar
            filename = f"roi_point_{self._current_point + 1:04d}.png"
            filepath = os.path.join(viz_folder, filename)
            cv2.imwrite(filepath, frame_viz)
            
            logger.debug(f"[MicroscopyService] ROI visualizado guardado: {filename}")
            
        except Exception as e:
            logger.error(f"[MicroscopyService] Error guardando visualización de ROI: {e}")

    def _finish_microscopy(self) -> None:
        """Finaliza la microscopia automatizada."""
        if not self._state_manager.is_active:
            return

        self._state_manager.complete()

        if self._is_dual_control_active and self._is_dual_control_active():
            self._stop_dual_control()

        total_images = self._state_manager.image_counter
        self.status_changed.emit(
            f"MICROSCOPIA COMPLETADA: {total_images} imagenes capturadas"
        )
        logger.info(
            "[MicroscopyService] MICROSCOPIA COMPLETADA: %d imagenes",
            total_images,
        )
        self.finished.emit(total_images)

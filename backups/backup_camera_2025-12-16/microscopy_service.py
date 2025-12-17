"""
Servicio de Microscopía Automatizada
=====================================

Orquesta la ejecución de trayectorias de microscopía con:
- Movimiento XY automatizado
- Autofoco inteligente por punto
- Captura de imágenes multicanal
- Progreso en tiempo real
- Sistema de aprendizaje de ROIs (50 imágenes)

Autor: Sistema de Control L206
Fecha: 2025-12-13
"""

import logging
import time
import numpy as np
import cv2
import os

from typing import Callable, Optional, List, Tuple
from PyQt5.QtCore import QObject, pyqtSignal, QTimer

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

    def __init__(
        self,
        parent=None,
        get_trajectory: Optional[Callable[[], Optional[List]]] = None,
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
    ):
        super().__init__(parent)

        # Callbacks y dependencias externas
        self._get_trajectory = get_trajectory
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

        # Estado de microscopia
        self._microscopy_active = False
        self._microscopy_config: Optional[dict] = None
        self._microscopy_trajectory: Optional[List] = None
        self._current_point = 0
        self._position_checks = 0
        self._delay_before_ms = 0
        self._delay_after_ms = 0
        
        # MEJORA 2: Sistema de aprendizaje
        self._learning_mode = False
        self._learning_count = 0
        self._learning_target = 50
        self._learning_dialog = None
        
        # MEJORA 4: Control de pausa
        self._is_paused = False

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

        # Guardar configuracion y estado
        self._microscopy_config = config
        self._microscopy_trajectory = list(trajectory)
        self._microscopy_active = True
        self._current_point = 0
        self._position_checks = 0

        # Delays
        self._delay_before_ms = int(config.get('delay_before', 2.0) * 1000)
        self._delay_after_ms = int(config.get('delay_after', 0.2) * 1000)

        total = len(self._microscopy_trajectory)
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

        # Ejecutar primer punto
        self._move_to_point()
        return True

    def stop_microscopy(self) -> None:
        """Detiene la microscopia automatizada."""
        if not self._microscopy_active:
            return

        logger.info("[MicroscopyService] === DETENIENDO MICROSCOPIA ===")
        self._microscopy_active = False

        if self._is_dual_control_active and self._is_dual_control_active():
            self._stop_dual_control()

        self.status_changed.emit("Microscopia detenida por usuario")
        self.stopped.emit()

    def is_running(self) -> bool:
        """Indica si hay una secuencia de microscopía activa."""
        return bool(self._microscopy_active)

    # ------------------------------------------------------------------
    # Flujo interno de microscopia
    # ------------------------------------------------------------------
    def _move_to_point(self) -> None:
        """PASO 1: Mueve al punto actual."""
        if not self._microscopy_active:
            return
        
        # MEJORA 4: Verificar pausa
        if self._is_paused:
            # Esperar 500ms y volver a verificar
            QTimer.singleShot(500, self._move_to_point)
            return

        if self._microscopy_trajectory is None:
            return

        if self._current_point >= len(self._microscopy_trajectory):
            self._finish_microscopy()
            return

        point = self._microscopy_trajectory[self._current_point]
        x_target = point[0]
        y_target = point[1]

        n = self._current_point + 1
        total = len(self._microscopy_trajectory)
        self.status_changed.emit(
            f"[{n}/{total}] Moviendo a X={x_target:.1f}, Y={y_target:.1f} um"
        )
        logger.info(
            "[MicroscopyService] Punto %d: (%.1f, %.1f)",
            n,
            x_target,
            y_target,
        )

        # Configurar referencias en el controlador dual
        if self._set_dual_refs:
            self._set_dual_refs(x_target, y_target)

        # Detener control dual si está activo y reiniciar
        if self._is_dual_control_active and self._is_dual_control_active():
            self._stop_dual_control()

        self._position_checks = 0

        # Iniciar control dual para mover a la posicion
        self._start_dual_control()

        # Comenzar a verificar si llego a la posicion
        QTimer.singleShot(200, self._check_position)

    def _check_position(self) -> None:
        """PASO 2: Verifica si llego a la posicion objetivo."""
        if not self._microscopy_active:
            return

        self._position_checks += 1

        position_reached = False
        if self._is_position_reached:
            try:
                position_reached = bool(self._is_position_reached())
            except Exception as e:
                logger.error("[MicroscopyService] Error evaluando position_reached: %s", e)

        # Timeout: maximo 10 segundos esperando posicion (100 checks * 100ms)
        if self._position_checks > 100:
            self.status_changed.emit("  Timeout esperando posicion - continuando")
            logger.warning(
                "[MicroscopyService] Timeout en punto %d",
                self._current_point + 1,
            )
            position_reached = True

        if position_reached:
            if self._is_dual_control_active and self._is_dual_control_active():
                self._stop_dual_control()

            # PASO 3: DELAY_BEFORE para estabilizacion
            self.status_changed.emit(
                f"  Posicion alcanzada - Esperando {self._delay_before_ms}ms para estabilizar..."
            )
            logger.info(
                "[MicroscopyService] Posición alcanzada, delay_before=%dms",
                self._delay_before_ms,
            )
            QTimer.singleShot(self._delay_before_ms, self._capture)
        else:
            # Seguir esperando - verificar cada 100ms
            QTimer.singleShot(100, self._check_position)

    def _capture(self) -> None:
        """PASO 3: Captura la imagen (con o sin autofoco)."""
        if not self._microscopy_active:
            return

        if not self._microscopy_config:
            return

        use_autofocus = bool(self._microscopy_config.get('autofocus_enabled', False))
        cfocus_enabled = bool(self._cfocus_enabled_getter()) if self._cfocus_enabled_getter else False

        if use_autofocus and cfocus_enabled:
            self._capture_with_autofocus()
            return

        # Captura normal sin autofoco
        self.status_changed.emit("  Capturando imagen...")
        success = False
        if self._capture_microscopy_image:
            success = self._capture_microscopy_image(self._microscopy_config, self._current_point)

        if success:
            logger.info(
                "[MicroscopyService] Imagen %d capturada",
                self._current_point + 1,
            )
        else:
            self.status_changed.emit(
                f"  ERROR: Fallo captura imagen {self._current_point + 1}"
            )
            logger.error(
                "[MicroscopyService] Fallo captura imagen %d",
                self._current_point + 1,
            )

        # Actualizar progreso y avanzar
        self._advance_point()

    def _capture_with_autofocus(self) -> None:
        """Captura con detección y autofoco asíncrono."""
        if not self._microscopy_active:
            return

        if not (self._get_current_frame and self._smart_focus_scorer and self._autofocus_service):
            # Fallback a captura normal
            self.status_changed.emit("⚠️ Autofoco no disponible, capturando normal...")
            self._capture_without_autofocus_fallback()
            return

        frame = self._get_current_frame()
        if frame is None:
            self.status_changed.emit("⚠️ Sin frame disponible")
            self._advance_point()
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
        
        # MEJORA: Filtrar por área Y circularidad para rechazar manchas sin morfología
        # Usar parámetros configurables del SmartFocusScorer
        min_circularity = self._smart_focus_scorer.min_circularity if self._smart_focus_scorer else 0.45
        min_aspect_ratio = self._smart_focus_scorer.min_aspect_ratio if self._smart_focus_scorer else 0.4
        
        objects_filtered = []
        for obj in all_objects:
            # Filtro de área
            if not (min_area <= obj.area <= max_area):
                continue
            
            # Obtener circularidad del objeto (si fue calculada con contorno real)
            # Si no está disponible, calcular aproximación con bbox
            if hasattr(obj, 'circularity') and obj.circularity > 0:
                circularity = obj.circularity
            else:
                x, y, w, h = obj.bounding_box
                perimeter = 2 * (w + h)
                circularity = (4 * np.pi * obj.area) / (perimeter ** 2) if perimeter > 0 else 0
            
            # Filtro de circularidad (rechazar manchas irregulares)
            if circularity < min_circularity:
                logger.info(f"[MicroscopyService] ❌ Objeto rechazado: área={obj.area:.0f}px, circ={circularity:.2f} < {min_circularity:.2f} (mancha irregular)")
                continue
            
            # Filtro de aspect ratio (rechazar manchas muy alargadas)
            x, y, w, h = obj.bounding_box
            aspect_ratio = float(w) / float(h) if h > 0 else 1.0
            if aspect_ratio > 1.0:
                aspect_ratio = 1.0 / aspect_ratio
            
            if aspect_ratio < min_aspect_ratio:
                logger.info(f"[MicroscopyService] ❌ Objeto rechazado: área={obj.area:.0f}px, aspect_ratio={aspect_ratio:.2f} < {min_aspect_ratio:.2f} (muy alargado)")
                continue
            
            # Objeto válido (área, morfología y forma correctas)
            objects_filtered.append(obj)
            logger.info(f"[MicroscopyService] ✓ Objeto válido: área={obj.area:.0f}px, circ={circularity:.2f}, aspect={aspect_ratio:.2f}")
        
        objects = objects_filtered
        n_objects = len(objects)
        if n_objects == 0:
            self.status_changed.emit(
                f"  ⚠️ Sin objetos en rango [{min_area}-{max_area}] px - saltando punto"
            )
            logger.info(
                "[MicroscopyService] Punto %d: sin objetos en rango (detectados: %d)",
                self._current_point,
                len(all_objects),
            )
            # MEJORA: Continuar con el siguiente punto en lugar de detenerse
            self._current_point += 1
            self._position_checks = 0
            self.progress_changed.emit(self._current_point, len(self._microscopy_trajectory))
            self._move_to_point()
            return
        
        # MEJORA: Mostrar máscaras en ventana de cámara durante autofoco
        self._show_autofocus_masks(objects)

        # MEJORA 1: Enfocar solo en el objeto MÁS GRANDE del área objetivo
        # Esto asegura que el autofoco se concentre en el objeto principal
        largest_object = max(objects, key=lambda obj: obj.area)
        
        # MEJORA 2: Sistema de aprendizaje con confirmación
        if self._learning_mode and self._learning_count < self._learning_target:
            should_capture = self._confirm_roi_for_learning(frame_bgr, largest_object, result)
            if not should_capture:
                logger.info("[MicroscopyService] ROI rechazado por usuario en aprendizaje")
                self._advance_point()
                return
            self._learning_count += 1
            logger.info(f"[MicroscopyService] Aprendizaje: {self._learning_count}/{self._learning_target}")
        
        self.status_changed.emit(
            f"  ✓ Enfocando objeto más grande: {largest_object.area:.0f} px (de {n_objects} en rango)"
        )
        logger.info(
            "[MicroscopyService] Punto %d: enfocando objeto más grande (área=%.0f px) de %d objetos válidos",
            self._current_point,
            largest_object.area,
            n_objects,
        )

        # MEJORA 3: Guardar frame con ROI visualizado para referencia
        self._save_roi_visualization(frame_bgr, largest_object, result)

        # Iniciar autofoco asíncrono solo en el objeto más grande
        self._autofocus_service.start_autofocus([largest_object])

    def _capture_without_autofocus_fallback(self) -> None:
        """Captura sencilla usada como fallback cuando no hay autofoco disponible."""
        success = False
        if self._capture_microscopy_image and self._microscopy_config:
            success = self._capture_microscopy_image(self._microscopy_config, self._current_point)

        if success:
            logger.info(
                "[MicroscopyService] Imagen %d capturada (fallback)",
                self._current_point + 1,
            )
        else:
            self.status_changed.emit(
                f"  ERROR: Fallo captura imagen {self._current_point + 1} (fallback)"
            )
            logger.error(
                "[MicroscopyService] Fallo captura imagen %d (fallback)",
                self._current_point + 1,
            )

        self._advance_point()

    def handle_autofocus_complete(self) -> None:
        """Debe llamarse cuando AutofocusService completa el autofoco en microscopia.

        Captura la imagen con mejor foco y avanza al siguiente punto.
        """
        if not self._microscopy_active:
            return

        self.status_changed.emit("📸 Capturando imagen con BPoF...")
        success = False
        if self._capture_microscopy_image and self._microscopy_config:
            success = self._capture_microscopy_image(self._microscopy_config, self._current_point)

        if success:
            logger.info(
                "[MicroscopyService] Imagen %d capturada con autofoco",
                self._current_point + 1,
            )
        else:
            self.status_changed.emit(
                f"  ERROR: Fallo captura imagen {self._current_point + 1} tras autofoco"
            )
            logger.error(
                "[MicroscopyService] Fallo captura imagen %d tras autofoco",
                self._current_point + 1,
            )

        self._advance_point()

    def _advance_point(self) -> None:
        """Avanza al siguiente punto de microscopía."""
        if not self._microscopy_active or self._microscopy_trajectory is None:
            return

        total = len(self._microscopy_trajectory)
        self.progress_changed.emit(self._current_point + 1, total)

        self._current_point += 1
        if self._current_point < total:
            QTimer.singleShot(self._delay_after_ms, self._move_to_point)
        else:
            self._finish_microscopy()

    def stop_microscopy(self) -> None:
        """Detiene la microscopía en curso."""
        if not self._microscopy_active:
            return

        self._microscopy_active = False
        self._is_paused = False
        self.status_changed.emit("Microscopía detenida")
        logger.info("[MicroscopyService] Microscopía detenida por usuario")
    
    def enable_learning_mode(self, enabled: bool = True, target_count: int = 50):
        """Activa/desactiva el modo de aprendizaje."""
        self._learning_mode = enabled
        self._learning_target = target_count
        self._learning_count = 0
        
        if enabled:
            logger.info(f"[MicroscopyService] Modo aprendizaje activado: objetivo {target_count} imágenes")
        else:
            logger.info("[MicroscopyService] Modo aprendizaje desactivado")
    
    def set_paused(self, paused: bool):
        """Pausa/reanuda la microscopía."""
        self._is_paused = paused
        if paused:
            logger.info(f"[MicroscopyService] Microscopía pausada en punto {self._current_point + 1}")
        else:
            logger.info("[MicroscopyService] Microscopía reanudada")
    
    def skip_current_point(self):
        """Salta el punto actual sin capturar."""
        logger.info(f"[MicroscopyService] Usuario solicitó saltar punto {self._current_point}")
        self.status_changed.emit(f"⏭️ Punto {self._current_point} saltado por usuario")
        # Limpiar máscaras antes de avanzar
        self._clear_autofocus_masks()
        # Avanzar al siguiente punto
        self._current_point += 1
        self._position_checks = 0
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
        if not self._microscopy_active:
            return

        self._microscopy_active = False

        if self._is_dual_control_active and self._is_dual_control_active():
            self._stop_dual_control()

        total = len(self._microscopy_trajectory) if self._microscopy_trajectory else 0
        self.status_changed.emit(
            f"MICROSCOPIA COMPLETADA: {total} imagenes capturadas"
        )
        logger.info(
            "[MicroscopyService] MICROSCOPIA COMPLETADA: %d imagenes",
            total,
        )
        self.finished.emit(total)

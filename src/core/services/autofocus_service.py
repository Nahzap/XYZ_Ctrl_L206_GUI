"""
Autofocus Service - Servicio de Autoenfoque Asíncrono
=====================================================

Worker para ejecutar Z-scanning en background sin bloquear la UI.
Emite señales de progreso para visualización en tiempo real.

Autor: Sistema de Control L206
Fecha: 2025-12-12
"""

import time
import logging
import numpy as np

from typing import List, Tuple, Optional, Callable

from PyQt5.QtCore import QThread, pyqtSignal

from core.models.detected_object import DetectedObject
from core.models.focus_result import AutofocusResult
from core.autofocus.smart_focus_scorer import SmartFocusScorer
from core.autofocus.focus_metric import (
    calculate_focus_score_detailed,
    bbox_to_contour,
    build_multifocal_z_positions,
)

logger = logging.getLogger('MotorControl_L206')

# Alias para compatibilidad
FocusResult = AutofocusResult


class AutofocusService(QThread):
    """
    Servicio de autofoco asíncrono.
    
    Realiza Z-scanning para cada objeto detectado y emite progreso
    en tiempo real para actualización de UI.
    
    Signals:
        scan_started: Emitido al iniciar escaneo de un objeto
        z_changed: Emitido en cada posición Z evaluada (z, score, roi_frame)
        object_focused: Emitido cuando se encuentra el foco óptimo de un objeto
        scan_complete: Emitido cuando termina todo el proceso
        error_occurred: Emitido si hay un error
    """
    
    # Señales
    scan_started = pyqtSignal(int, int)  # object_index, total_objects
    z_changed = pyqtSignal(float, float, np.ndarray)  # z_position, score, roi_frame
    object_focused = pyqtSignal(int, float, float)  # object_index, z_optimal, score
    scan_complete = pyqtSignal(list)  # List[FocusResult]
    error_occurred = pyqtSignal(str)
    masks_detected = pyqtSignal(list)  # Máscaras/ROIs detectados para visualización
    status_message = pyqtSignal(str)  # Mensajes de estado para UI y terminal
    score_updated = pyqtSignal(float, float)  # (z_position, score) para overlay en cámara
    progress_updated = pyqtSignal(int, int, str)  # (current_step, total_steps, phase_name)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Referencias a hardware (se configuran externamente)
        self.cfocus_controller = None
        self.get_frame_callback: Callable[[], np.ndarray] = None
        
        # Objetos a enfocar
        self.objects_to_focus: List[DetectedObject] = []
        
        # Parámetros de búsqueda (configurables desde UI)
        # NOTA: Estos parámetros son para BÚSQUEDA de BPoF, NO para captura de volumen
        self.z_scan_range = 70.0       # µm - rango total de escaneo (ajustable por usuario)
        self.use_full_range = True     # Si True, escanea todo el rango calibrado; si False, usa z_scan_range
        self.z_step_coarse = 1.0       # µm - paso grueso OPTIMIZADO para velocidad (reducir pasos totales)
        self.z_step_fine = 0.05        # µm - paso fino para refinamiento preciso del BPoF
        self.refine_window = 2.0       # µm - ventana de refinamiento (±1.0µm del pico coarse)
        # Tiempos de espera optimizados para velocidad
        # settle_time: Espera mínima para estabilización del piezo
        # CRÍTICO: Debe ser > 0 para permitir que el piezo se asiente
        # Valor recomendado: 0.01-0.03s (10-30ms) para velocidad óptima
        self.settle_time = 0.01        # s - tiempo de estabilización del piezo OPTIMIZADO para velocidad
        self.capture_settle_time = 0.3  # s - estabilización para captura final (reducido de 0.5s)
        self.roi_margin = 20    # px - margen alrededor del ROI cuadrado (sincronizado con UI)
        
        # Límites de iteraciones para evitar bucles infinitos
        self.max_coarse_iterations = 50  # Máximo de iteraciones en fase gruesa
        self.max_fine_iterations = 100   # Máximo de iteraciones en fase fina
        
        # Parámetros de captura multi-focal (para trayectoria XY)
        # NOTA: Estas capturas son para obtener imágenes con diferentes niveles de enfoque
        self.n_captures = 3       # Número de capturas (siempre impar: 3, 5, 7, etc.)
        self.capture_step = None  # µm - paso entre capturas (None → z_step_capture)
        self.z_step_capture = 2.0   # µm - paso entre capas multi-focal
        
        # Registro del máximo Z encontrado (para optimizar futuros escaneos)
        self.z_max_recorded = None  # Se actualiza tras primer escaneo completo
        
        # Control
        self.running = False
        self.cancel_requested = False
        # True solo durante autofoco disparado por MicroscopyService (captura multi-focal)
        self.microscopy_mode = False
        
        # Scorer morfológico para Smart Autofocus (usa máscara de morfología)
        self._focus_scorer = SmartFocusScorer(
            min_circularity=0.45,
            min_aspect_ratio=0.4
        )
        try:
            self._focus_scorer.load_model()
        except Exception:
            # Si falla la carga del modelo, el scorer cae a fallback interno
            pass
        
        logger.info("[AutofocusService] Inicializado")
    
    def configure(self, cfocus_controller, get_frame_callback: Callable):
        """
        Configura el servicio con referencias a hardware.
        
        Args:
            cfocus_controller: Controlador del piezo C-Focus
            get_frame_callback: Función que retorna el frame actual de la cámara
        """
        self.cfocus_controller = cfocus_controller
        self.get_frame_callback = get_frame_callback
        logger.info("[AutofocusService] Configurado con C-Focus y cámara")
    
    def get_search_info(self) -> dict:
        """Retorna información sobre los parámetros de búsqueda de autofoco.
        
        NOTA: Autofoco NO captura volúmenes. Usa algoritmo de búsqueda (hill climbing)
        con pasos heterogéneos (coarse → fine) para encontrar 1 posición óptima (BPoF).
        El número de evaluaciones depende del algoritmo, no es predecible.
        
        Para captura de volúmenes con pasos homogéneos, usar VolumetryService (Z-Stack).
        
        Returns:
            dict con parámetros de búsqueda
        """
        return {
            'scan_range_um': self.z_scan_range,
            'z_step_coarse': self.z_step_coarse,
            'z_step_fine': self.z_step_fine,
            'search_distance_um': 2 * self.z_scan_range,
            'algorithm': 'hill_climbing'
        }
    
    def validate_scan_range(self) -> Tuple[bool, str]:
        """
        Valida que el C-Focus esté calibrado para escaneo completo.
        El autofoco ahora escanea TODO el rango calibrado (z_min a z_max).
        
        Returns:
            (is_valid, message)
        """
        if not self.cfocus_controller:
            return False, "C-Focus no conectado"
        
        calib_info = self.cfocus_controller.get_calibration_info()
        if not calib_info['is_calibrated']:
            return False, "C-Focus no calibrado. Ejecutar calibración primero."
        
        z_min_hw = calib_info['z_min']
        z_max_hw = calib_info['z_max']
        z_range = z_max_hw - z_min_hw
        
        # Validar que el rango calibrado sea razonable
        if z_range < 10.0:
            return False, f"Rango calibrado muy pequeño ({z_range:.2f}µm). Re-calibrar C-Focus."
        
        return True, f"Rango de escaneo: {z_min_hw:.2f} - {z_max_hw:.2f}µm ({z_range:.2f}µm total)"

    def _has_live_frame(self) -> bool:
        """Comprueba que el callback de cámara devuelve un frame válido (stream activo)."""
        if not self.get_frame_callback:
            return False
        try:
            frame = self.get_frame_callback()
        except Exception:
            return False
        return frame is not None and getattr(frame, "size", 0) > 0

    def validate_can_run(self) -> Tuple[bool, str]:
        """Valida que el autofoco puede ejecutarse (manual o por algoritmo)."""
        if self.isRunning():
            return False, "Ya hay un escaneo de autofoco en progreso"
        if not self.cfocus_controller:
            return False, "C-Focus no configurado"
        if not self.get_frame_callback:
            return False, "Cámara no configurada para autofoco (conecta y calibra C-Focus)"
        if not self._has_live_frame():
            return False, "No hay transmisión de cámara activa (inicia vista en vivo)"
        is_valid, msg = self.validate_scan_range()
        if not is_valid:
            return False, msg
        return True, "OK"

    def start_autofocus(self, objects: List[DetectedObject]) -> bool:
        """
        Inicia el proceso de autofoco para una lista de objetos (hilo propio).

        Debe invocarse solo desde la UI (manual) o desde MicroscopyService
        cuando el algoritmo detecta un objeto y el autofoco está habilitado.

        Args:
            objects: Lista de objetos detectados a enfocar

        Returns:
            True si el escaneo se inició correctamente.
        """
        can_run, msg = self.validate_can_run()
        if not can_run:
            logger.warning("[AutofocusService] No se puede iniciar autofoco: %s", msg)
            self.error_occurred.emit(msg)
            return False

        self.objects_to_focus = objects
        self.cancel_requested = False
        self.running = True
        self.start()
        return True
    
    def cancel(self):
        """Cancela el escaneo en progreso."""
        self.cancel_requested = True
        logger.info("[AutofocusService] Cancelación solicitada")
    
    def run(self):
        """Loop principal de autofoco."""
        results: List[FocusResult] = []
        total_objects = len(self.objects_to_focus)
        
        logger.info(f"[AutofocusService] Iniciando autofoco para {total_objects} objetos")
        
        for i, obj in enumerate(self.objects_to_focus):
            if self.cancel_requested:
                logger.info("[AutofocusService] Cancelado por usuario")
                break
            
            # Emitir inicio de escaneo
            self.scan_started.emit(i, total_objects)
            
            try:
                # Escanear objeto (pasar índice ya que ObjectInfo no tiene .index)
                result = self._scan_single_object(obj, i)
                results.append(result)
                
                # Emitir resultado
                self.object_focused.emit(i, result.z_optimal, result.focus_score)
                
            except Exception as e:
                logger.error(f"[AutofocusService] Error en objeto {i}: {e}")
                self.error_occurred.emit(f"Error en objeto {i}: {e}")
        
        # NOTA: NO mover Z después del autofoco - el frame ya fue capturado en BPoF
        # El sistema debe permanecer en BPoF hasta que se guarde la imagen
        # El movimiento a posición central se hace DESPUÉS de guardar la imagen
        
        self.running = False
        self.scan_complete.emit(results)
        logger.info(f"[AutofocusService] Completado: {len(results)}/{total_objects} objetos")
    
    def _optimize_focus_simple(
        self,
        bbox,
        contour,
        z_min: float,
        z_max: float,
        z_center: float,
        log_prefix: str = "[Autofocus]",
        microscopy_format: bool = False,
    ) -> tuple:
        """
        ESCANEO COMPLETO de autofocus - recorre TODO el rango calibrado.
        
        Algoritmo:
        1. Escanea desde z_min hasta z_max con paso coarse
        2. Registra el mejor Z encontrado
        3. Refina alrededor del mejor Z con paso fine
        4. Retorna (best_z, best_score)
        
        Garantiza cubrir TODO el rango disponible.
        """
        msg = f"{log_prefix} ESCANEO COMPLETO: {z_min:.2f} -> {z_max:.2f}um (paso={self.z_step_coarse}um)"
        logger.info(msg)
        print(msg)

        coarse_label = "SCAN" if microscopy_format else "COARSE"
        
        # FASE 1: Escaneo completo con paso grueso
        z_range = z_max - z_min
        n_steps = int(z_range / self.z_step_coarse) + 1
        
        msg = f"{log_prefix} Escaneando {n_steps} posiciones en rango completo ({z_range:.2f}µm)..."
        logger.info(msg)
        print(msg)
        
        best_z = z_min
        best_score = 0.0
        
        for i in range(n_steps):
            if self.cancel_requested:
                break
            
            z_current = z_min + (i * self.z_step_coarse)
            if z_current > z_max:
                z_current = z_max
            
            # Emitir progreso
            self.progress_updated.emit(i + 1, n_steps, "Escaneo completo")
            
            # Mover y evaluar
            move_success = self.cfocus_controller.move_z(z_current)
            if not move_success:
                logger.warning(f"[Autofocus] Fallo al mover a Z={z_current:.2f}µm")
                continue
            
            # Esperar estabilización del piezo (settle_time optimizado)
            time.sleep(self.settle_time)
            score = self._get_stable_score(bbox, contour)
            
            # CRÍTICO: Emitir score para actualizar overlay en UI (indicador S)
            self.score_updated.emit(z_current, score)
            
            # Actualizar mejor posición
            if score > best_score:
                best_z = z_current
                best_score = score
            
            # MENSAJE DE PROGRESO EN LÍNEA ÚNICA (se actualiza, no acumula)
            progress_pct = ((i + 1) / n_steps) * 100
            distance_traveled = z_current - z_min
            msg = (
                f"{log_prefix} {coarse_label}: {distance_traveled:.2f}/{z_range:.2f}µm "
                f"({progress_pct:.1f}%) | Z={z_current:.2f}µm | Score={score:.1f} | "
                f"Best={best_score:.1f}@{best_z:.2f}µm"
            )
            if microscopy_format:
                print(msg, end='\r', flush=True)
                try:
                    from PyQt5.QtCore import QCoreApplication
                    QCoreApplication.processEvents()
                except Exception:
                    pass
            logger.debug(msg)
        
        # Línea final del escaneo coarse (nueva línea)
        print()  # Nueva línea después del progreso
        msg = (
            f"{log_prefix} {coarse_label} COMPLETO: Mejor Z={best_z:.2f}µm, "
            f"Score={best_score:.1f} (recorrido: {z_range:.2f}µm)"
        )
        logger.info(msg)
        print(msg)
        
        # FASE 2: Refinamiento con paso fino alrededor del mejor Z
        step = self.z_step_fine
        
        msg = f"{log_prefix} Refinamiento fino (paso={step}µm) alrededor de Z={best_z:.2f}µm..."
        logger.info(msg)
        print(msg)
        
        # Explorar ±z_step_coarse alrededor del pico con paso fino
        z_refine_min = max(z_min, best_z - self.z_step_coarse)
        z_refine_max = min(z_max, best_z + self.z_step_coarse)
        
        # Calcular número total de pasos de refinamiento
        total_refine_steps = int((z_refine_max - z_refine_min) / step) + 1
        total_refine_steps = min(total_refine_steps, self.max_fine_iterations)
        
        z_refine = z_refine_min
        refine_iteration = 0
        
        # Guardar mejor resultado del escaneo grueso
        best_z_coarse = best_z
        best_score_coarse = best_score
        
        refine_range = z_refine_max - z_refine_min
        
        while z_refine <= z_refine_max and refine_iteration < self.max_fine_iterations:
            if self.cancel_requested:
                break
            
            refine_iteration += 1
            
            # Emitir progreso (fase fina)
            self.progress_updated.emit(refine_iteration, total_refine_steps, "Refinamiento fino")
            
            move_success = self.cfocus_controller.move_z(z_refine)
            if not move_success:
                logger.warning(f"[Autofocus] Fallo al mover a Z={z_refine:.2f}µm, abortando refinamiento")
                break
            # Esperar estabilización del piezo (settle_time optimizado)
            time.sleep(self.settle_time)
            score = self._get_stable_score(bbox, contour)
            
            # CRÍTICO: Emitir score para actualizar overlay en UI (indicador S)
            self.score_updated.emit(z_refine, score)
            
            if score > best_score:
                best_z = z_refine
                best_score = score
            
            # MENSAJE DE PROGRESO EN LÍNEA ÚNICA (se actualiza, no acumula)
            progress_pct = (refine_iteration / total_refine_steps) * 100
            distance_traveled = z_refine - z_refine_min
            msg = (
                f"{log_prefix} FINE: {distance_traveled:.2f}/{refine_range:.2f}µm "
                f"({progress_pct:.1f}%) | Z={z_refine:.2f}µm | Score={score:.1f} | "
                f"Best={best_score:.1f}@{best_z:.2f}µm"
            )
            if microscopy_format:
                print(msg, end='\r', flush=True)
                try:
                    from PyQt5.QtCore import QCoreApplication
                    QCoreApplication.processEvents()
                except Exception:
                    pass
            logger.debug(msg)
            
            z_refine += step
        
        # Línea final del refinamiento (nueva línea)
        print()  # Nueva línea después del progreso
        improvement = best_score - best_score_coarse
        msg = (
            f"{log_prefix} OK OPTIMO FINAL: Z={best_z:.2f}um, Score={best_score:.1f} "
            f"(mejora: +{improvement:.1f})"
        )
        logger.info(msg)
        print(msg)
        
        return best_z, best_score

    def _resolve_scan_range(self) -> Tuple[float, float, float, float]:
        """Retorna (z_min, z_max, z_center, z_range_total) según configuración."""
        z_current = self.cfocus_controller.read_z()
        if z_current is None or z_current < 0:
            z_current = 0.0
            logger.warning("[Autofocus] No se pudo leer posición Z, usando 0.0µm")

        calib_info = self.cfocus_controller.get_calibration_info()
        if not calib_info['is_calibrated']:
            raise ValueError("C-Focus no calibrado. Ejecutar calibración antes de usar autofoco.")

        z_min_hw = calib_info['z_min']
        z_max_hw = calib_info['z_max']
        z_center_hw = calib_info['z_center']

        if self.use_full_range:
            z_min = z_min_hw
            z_max = z_max_hw
            z_range_total = z_max_hw - z_min_hw
            logger.info(
                f"[Autofocus] ESCANEO COMPLETO: {z_min:.2f} -> {z_max:.2f}um "
                f"(rango total: {z_range_total:.2f}µm)"
            )
        else:
            z_min = max(z_min_hw, z_current - self.z_scan_range)
            z_max = min(z_max_hw, z_current + self.z_scan_range)
            z_range_total = z_max - z_min
            logger.info(
                f"[Autofocus] Escaneo local: {z_min:.2f}-{z_max:.2f}µm "
                f"({z_range_total:.2f}µm desde Z={z_current:.2f}µm)"
            )

        return z_min, z_max, z_current, z_range_total

    def focus_object_sync(
        self,
        obj,
        obj_index: int = 0,
        return_to_z_center: bool = False,
        log_prefix: str = "[Autofocus]",
        microscopy_format: bool = False,
    ) -> AutofocusResult:
        """
        Pipeline unificado de autofoco (síncrono).

        Usado por microscopía automatizada y por el worker QThread.
        """
        bbox = obj.bounding_box
        contour = getattr(obj, 'contour', None)
        if contour is None:
            contour = bbox_to_contour(bbox)

        is_valid, validation_msg = self.validate_scan_range()
        if not is_valid:
            raise ValueError(validation_msg)

        z_min, z_max, z_center, z_range_total = self._resolve_scan_range()
        z_center_hw = self.cfocus_controller.get_calibration_info()['z_center']

        best_z, best_score = self._optimize_focus_simple(
            bbox, contour, z_min, z_max, z_center,
            log_prefix=log_prefix,
            microscopy_format=microscopy_format,
        )

        # Verificar BPoF con tiempos de CAPTURA (no los del scan rápido)
        best_z, best_score = self._verify_and_refine_bpof(
            best_z, best_score, bbox, contour, z_min, z_max, log_prefix
        )

        if microscopy_format:
            logger.info(
                f"{log_prefix} BPoF encontrado: Z={best_z:.2f}µm, "
                f"Score={best_score:.1f} (recorrido: {z_range_total:.2f}µm)"
            )
        else:
            msg = f"{log_prefix} ✓ BPoF FINAL: Z={best_z:.1f}µm, Score={best_score:.1f}"
            logger.info(msg)
            self.status_message.emit(msg)

        # Verificación en BPoF con estabilización de captura
        self.cfocus_controller.move_z(best_z)
        frame_verify, score_verify = self._capture_at_z(best_z, bbox, contour)
        logger.info(
            f"{log_prefix} BPoF verificado: Z={best_z:.2f}µm, "
            f"S_scan={best_score:.1f}, S_capture={score_verify:.1f}"
        )
        if score_verify > 0:
            best_score = score_verify

        n_captures = self.n_captures if self.n_captures % 2 == 1 else 3
        capture_step = self.capture_step or self.z_step_capture or self.z_step_coarse

        z_positions = build_multifocal_z_positions(
            best_z, n_captures, capture_step, z_min, z_max
        )
        frames = []
        scores = []
        center_idx = n_captures // 2

        logger.info(
            f"{log_prefix} Capturando {n_captures} imágenes multi-focales "
            f"(paso={capture_step}µm, BPoF Z={best_z:.2f}µm)"
        )

        for i, z_capture in enumerate(z_positions):
            frame_i, score_i = self._capture_at_z(z_capture, bbox, contour)
            frames.append(frame_i)
            scores.append(score_i)

            offset = z_capture - best_z
            if i == center_idx:
                label = "BPoF"
            else:
                label = f"offset={offset:+.1f}µm"
            logger.info(
                f"{log_prefix} Captura {i + 1}/{n_captures} ({label}): "
                f"Z={z_capture:.2f}µm, S={score_i:.1f}"
            )

        best_z, best_score, frames, scores, z_positions = self._ensure_bpof_at_center(
            best_z, best_score, frames, scores, z_positions, center_idx, bbox, contour, log_prefix
        )

        if return_to_z_center:
            logger.info(f"{log_prefix} Volviendo a Z medio: {z_center_hw:.2f}µm")
            self.cfocus_controller.move_z(z_center_hw)
            time.sleep(self.settle_time)
            z_final_read = self.cfocus_controller.read_z()
            if z_final_read is not None:
                logger.info(
                    f"{log_prefix} ✓ Posición final: Z={z_final_read:.2f}µm "
                    f"(centro calibrado)"
                )
        else:
            self.cfocus_controller.move_z(best_z)
            time.sleep(self.settle_time)
            z_final_read = self.cfocus_controller.read_z()
            logger.info(
                f"{log_prefix} ✓ Posición final verificada: Z={z_final_read:.2f}µm "
                f"(BPoF={best_z:.2f}µm)"
            )

        final_frame = frames[center_idx] if frames else None
        frame_alt = frames[-1] if len(frames) > 1 else None
        z_alt = z_positions[-1] if z_positions else best_z
        score_alt = scores[-1] if scores else 0.0

        return FocusResult(
            object_index=obj_index,
            z_optimal=best_z,
            focus_score=best_score,
            bbox=bbox,
            frame=final_frame,
            frames=frames,
            z_positions=z_positions,
            focus_scores=scores,
            frame_alt=frame_alt,
            z_alt=z_alt,
            score_alt=score_alt,
        )

    def _scan_single_object(self, obj, obj_index: int) -> FocusResult:
        """Ejecuta el pipeline unificado de autofoco para un objeto (solo desde QThread)."""
        if self.microscopy_mode:
            return self.focus_object_sync(
                obj,
                obj_index=obj_index,
                return_to_z_center=True,
                log_prefix="[MicroscopyService]",
                microscopy_format=True,
            )
        return self.focus_object_sync(
            obj,
            obj_index=obj_index,
            return_to_z_center=False,
            log_prefix="[Autofocus]",
            microscopy_format=False,
        )
    
    def _get_score_at_z(self, z: float, bbox: Tuple[int, int, int, int]) -> float:
        """Mueve a posición Z y obtiene score estable."""
        self.cfocus_controller.move_z(z)
        time.sleep(self.settle_time)
        return self._get_stable_score(bbox, n_samples=2)  # Solo 2 muestras para velocidad
    
    def _verify_and_refine_bpof(
        self,
        best_z: float,
        best_score: float,
        bbox,
        contour,
        z_min: float,
        z_max: float,
        log_prefix: str,
    ) -> Tuple[float, float]:
        """Re-evalúa BPoF con settle de captura y micro-refinamiento fino."""
        step = self.z_step_fine
        candidates = [best_z]
        for dz in (-2 * step, -step, step, 2 * step):
            z_try = max(z_min, min(z_max, best_z + dz))
            if z_try not in candidates:
                candidates.append(z_try)

        best_z_cap = best_z
        best_score_cap = 0.0
        for z_try in candidates:
            _, score_try = self._capture_at_z(z_try, bbox, contour)
            logger.info(
                f"{log_prefix} Refine captura Z={z_try:.2f}µm -> S={score_try:.1f}"
            )
            if score_try > best_score_cap:
                best_score_cap = score_try
                best_z_cap = z_try

        if best_z_cap != best_z:
            logger.warning(
                f"{log_prefix} BPoF ajustado tras verificación: "
                f"Z {best_z:.2f} -> {best_z_cap:.2f}µm, "
                f"S_scan={best_score:.1f} -> S_capture={best_score_cap:.1f}"
            )
        return best_z_cap, best_score_cap if best_score_cap > 0 else best_score

    def _ensure_bpof_at_center(
        self,
        best_z: float,
        best_score: float,
        frames: list,
        scores: list,
        z_positions: list,
        center_idx: int,
        bbox,
        contour,
        log_prefix: str,
    ) -> Tuple[float, float, list, list, list]:
        """Garantiza que el frame central (_f1) sea el más nítido con su score real."""
        if not scores or not frames:
            return best_z, best_score, frames, scores, z_positions

        best_cap_idx = int(np.argmax(scores))
        if best_cap_idx != center_idx:
            logger.warning(
                f"{log_prefix} BPoF f{center_idx} no es el más nítido "
                f"(max f{best_cap_idx}: S={scores[best_cap_idx]:.1f} vs "
                f"f{center_idx}: S={scores[center_idx]:.1f}). Re-capturando en Z óptimo."
            )
            z_best = z_positions[best_cap_idx]
            frame_best, score_best = self._capture_at_z(z_best, bbox, contour)
            frames[center_idx] = frame_best
            scores[center_idx] = score_best
            z_positions[center_idx] = z_best
            best_z = z_best
            best_score = score_best
        else:
            best_z = z_positions[center_idx]
            best_score = scores[center_idx]
            logger.info(
                f"{log_prefix} BPoF confirmado en f{center_idx}: "
                f"Z={best_z:.2f}µm, S={best_score:.1f}"
            )

        return best_z, best_score, frames, scores, z_positions

    def _capture_at_z(
        self,
        z: float,
        bbox,
        contour,
    ) -> Tuple[Optional[np.ndarray], float]:
        """Mueve a Z, espera settle de captura, descarta buffer y captura frame fresco."""
        self.cfocus_controller.move_z(z)
        time.sleep(self.capture_settle_time)
        frame = self._get_fresh_frame()
        if frame is None or frame.size == 0:
            logger.warning(f"[Autofocus] Sin frame en Z={z:.2f}µm")
            return None, 0.0
        score = self._calculate_sharpness(frame, bbox, contour)
        return frame, score

    def _get_fresh_frame(self) -> Optional[np.ndarray]:
        """Descarta frames viejos del buffer y devuelve uno reciente."""
        if not self.get_frame_callback:
            return None
        # Descartar varios frames para asegurar imagen post-movimiento Z
        for _ in range(4):
            self.get_frame_callback()
            time.sleep(0.02)
        return self.get_frame_callback()

    def _get_stable_score(
        self,
        bbox: Tuple[int, int, int, int],
        contour: np.ndarray = None,
        n_samples: int = 1,
        flush_buffer: bool = False,
    ) -> float:
        """Obtiene score de sharpness del frame actual (solo para escaneo Z)."""
        if flush_buffer and self.get_frame_callback:
            for _ in range(2):
                self.get_frame_callback()
                time.sleep(0.01)

        frame = self.get_frame_callback()
        if frame is not None and frame.size > 0:
            return self._calculate_sharpness(frame, bbox, contour)

        logger.warning(f"[Autofocus] Frame inválido para bbox={bbox}")
        return 0.0

    def _calculate_sharpness(
        self,
        frame: np.ndarray,
        bbox: Tuple[int, int, int, int],
        contour: np.ndarray = None,
    ) -> float:
        """Delega en focus_metric (única implementación del índice S)."""
        if contour is None:
            contour = bbox_to_contour(bbox)

        score, details = calculate_focus_score_detailed(
            frame, bbox, contour, self.roi_margin
        )
        x, y, w, h = details["bbox"]
        logger.debug(
            f"[Autofocus] ROI original bbox=({x},{y},{w},{h}) -> "
            f"Cuadrado side={details['side']} -> "
            f"Expandido ({details['roi_w']}x{details['roi_h']})"
        )
        logger.debug(f"[Autofocus] S={score:.1f}")
        return score

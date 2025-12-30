"""
Gestor de Estado de Microscopía
=================================

Maneja el estado de la microscopía automatizada de forma centralizada.
Separa la gestión de estado de la lógica de negocio.

Autor: Sistema de Control L206
Fecha: 2025-12-29
"""

import logging
from enum import Enum
from typing import Optional, List, Tuple

logger = logging.getLogger('MotorControl_L206')


class MicroscopyState(Enum):
    """Estados posibles de la microscopía."""
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    STOPPING = "stopping"
    COMPLETED = "completed"
    ERROR = "error"


class MicroscopyStateManager:
    """
    Gestor centralizado de estado de microscopía.
    
    Maneja:
    - Estado actual (idle, running, paused, etc.)
    - Progreso (punto actual, total de puntos)
    - Contadores (intentos, capturas, etc.)
    - Flags de control (pausa, cancelación)
    
    Attributes:
        state: Estado actual de la microscopía
        current_point: Índice del punto actual (0-based)
        total_points: Total de puntos en la trayectoria
        position_checks: Contador de verificaciones de posición
        learning_count: Contador de imágenes en modo aprendizaje
        image_counter: Contador total de imágenes capturadas
    """
    
    def __init__(self):
        """Inicializa el gestor de estado."""
        self._state = MicroscopyState.IDLE
        self._current_point = 0
        self._total_points = 0
        self._position_checks = 0
        self._learning_count = 0
        self._image_counter = 0
        
        # Configuración de aprendizaje
        self._learning_mode = False
        self._learning_target = 50
        
        # Trayectoria
        self._trajectory: List[Tuple[float, float]] = []
    
    # ==================================================================
    # PROPIEDADES DE ESTADO
    # ==================================================================
    
    @property
    def state(self) -> MicroscopyState:
        """Estado actual."""
        return self._state
    
    @property
    def is_idle(self) -> bool:
        """Indica si está en estado idle."""
        return self._state == MicroscopyState.IDLE
    
    @property
    def is_running(self) -> bool:
        """Indica si está ejecutando."""
        return self._state == MicroscopyState.RUNNING
    
    @property
    def is_paused(self) -> bool:
        """Indica si está pausado."""
        return self._state == MicroscopyState.PAUSED
    
    @property
    def is_active(self) -> bool:
        """Indica si está activo (running o paused)."""
        return self._state in (MicroscopyState.RUNNING, MicroscopyState.PAUSED)
    
    @property
    def is_stopping(self) -> bool:
        """Indica si está deteniéndose."""
        return self._state == MicroscopyState.STOPPING
    
    @property
    def is_completed(self) -> bool:
        """Indica si completó."""
        return self._state == MicroscopyState.COMPLETED
    
    @property
    def is_error(self) -> bool:
        """Indica si hay error."""
        return self._state == MicroscopyState.ERROR
    
    # ==================================================================
    # PROPIEDADES DE PROGRESO
    # ==================================================================
    
    @property
    def current_point(self) -> int:
        """Índice del punto actual (0-based)."""
        return self._current_point
    
    @property
    def total_points(self) -> int:
        """Total de puntos."""
        return self._total_points
    
    @property
    def progress_percent(self) -> float:
        """Progreso en porcentaje (0-100)."""
        if self._total_points == 0:
            return 0.0
        return (self._current_point / self._total_points) * 100.0
    
    @property
    def remaining_points(self) -> int:
        """Puntos restantes."""
        return max(0, self._total_points - self._current_point)
    
    @property
    def position_checks(self) -> int:
        """Contador de verificaciones de posición."""
        return self._position_checks
    
    @property
    def image_counter(self) -> int:
        """Contador total de imágenes."""
        return self._image_counter
    
    @property
    def learning_count(self) -> int:
        """Contador de imágenes en modo aprendizaje."""
        return self._learning_count
    
    @property
    def learning_mode(self) -> bool:
        """Indica si modo aprendizaje está activo."""
        return self._learning_mode
    
    @property
    def learning_target(self) -> int:
        """Objetivo de imágenes para aprendizaje."""
        return self._learning_target
    
    @property
    def learning_completed(self) -> bool:
        """Indica si se completó el aprendizaje."""
        return self._learning_count >= self._learning_target
    
    # ==================================================================
    # MÉTODOS DE CONTROL
    # ==================================================================
    
    def start(self, trajectory: List[Tuple[float, float]], 
              learning_mode: bool = False, learning_target: int = 50):
        """
        Inicia una nueva sesión de microscopía.
        
        Args:
            trajectory: Lista de puntos (x, y)
            learning_mode: Si modo aprendizaje está activo
            learning_target: Objetivo de imágenes para aprendizaje
        """
        self._state = MicroscopyState.RUNNING
        self._trajectory = list(trajectory)
        self._total_points = len(trajectory)
        self._current_point = 0
        self._position_checks = 0
        self._image_counter = 0
        self._learning_count = 0
        self._learning_mode = learning_mode
        self._learning_target = learning_target
        
        logger.info(
            "[MicroscopyState] Iniciado: %d puntos, aprendizaje=%s (target=%d)",
            self._total_points,
            "ON" if learning_mode else "OFF",
            learning_target
        )
    
    def pause(self):
        """Pausa la microscopía."""
        if self._state == MicroscopyState.RUNNING:
            self._state = MicroscopyState.PAUSED
            logger.info("[MicroscopyState] Pausado en punto %d/%d", 
                       self._current_point + 1, self._total_points)
    
    def resume(self):
        """Reanuda la microscopía."""
        if self._state == MicroscopyState.PAUSED:
            self._state = MicroscopyState.RUNNING
            logger.info("[MicroscopyState] Reanudado desde punto %d/%d", 
                       self._current_point + 1, self._total_points)
    
    def stop(self):
        """Detiene la microscopía."""
        if self._state in (MicroscopyState.RUNNING, MicroscopyState.PAUSED):
            self._state = MicroscopyState.STOPPING
            logger.info("[MicroscopyState] Deteniendo en punto %d/%d", 
                       self._current_point + 1, self._total_points)
    
    def complete(self):
        """Marca como completado."""
        self._state = MicroscopyState.COMPLETED
        logger.info("[MicroscopyState] Completado: %d/%d puntos, %d imágenes", 
                   self._current_point, self._total_points, self._image_counter)
    
    def error(self, message: str = ""):
        """Marca como error."""
        self._state = MicroscopyState.ERROR
        logger.error("[MicroscopyState] Error: %s", message)
    
    def reset(self):
        """Resetea a estado inicial."""
        self._state = MicroscopyState.IDLE
        self._current_point = 0
        self._total_points = 0
        self._position_checks = 0
        self._learning_count = 0
        self._image_counter = 0
        self._trajectory = []
        logger.info("[MicroscopyState] Reseteado")
    
    # ==================================================================
    # MÉTODOS DE PROGRESO
    # ==================================================================
    
    def advance_point(self):
        """Avanza al siguiente punto."""
        if self._current_point < self._total_points:
            self._current_point += 1
            logger.debug("[MicroscopyState] Avanzado a punto %d/%d", 
                        self._current_point, self._total_points)
    
    def increment_position_checks(self):
        """Incrementa contador de verificaciones de posición."""
        self._position_checks += 1
    
    def reset_position_checks(self):
        """Resetea contador de verificaciones de posición."""
        self._position_checks = 0
    
    def increment_image_counter(self):
        """Incrementa contador de imágenes."""
        self._image_counter += 1
        
        # Incrementar contador de aprendizaje si está activo
        if self._learning_mode and not self.learning_completed:
            self._learning_count += 1
            logger.debug("[MicroscopyState] Aprendizaje: %d/%d", 
                        self._learning_count, self._learning_target)
    
    def skip_current_point(self):
        """Salta el punto actual."""
        logger.info("[MicroscopyState] Saltando punto %d/%d", 
                   self._current_point + 1, self._total_points)
        self.advance_point()
    
    # ==================================================================
    # MÉTODOS DE CONSULTA
    # ==================================================================
    
    def get_current_target(self) -> Optional[Tuple[float, float]]:
        """
        Obtiene el punto objetivo actual.
        
        Returns:
            (x, y) en µm o None si no hay punto actual
        """
        if 0 <= self._current_point < len(self._trajectory):
            return self._trajectory[self._current_point]
        return None
    
    def get_progress_info(self) -> dict:
        """
        Obtiene información de progreso.
        
        Returns:
            dict con información de progreso
        """
        return {
            'state': self._state.value,
            'current_point': self._current_point,
            'total_points': self._total_points,
            'progress_percent': self.progress_percent,
            'remaining_points': self.remaining_points,
            'image_counter': self._image_counter,
            'learning_mode': self._learning_mode,
            'learning_count': self._learning_count,
            'learning_target': self._learning_target,
            'learning_completed': self.learning_completed
        }
    
    def get_state_summary(self) -> str:
        """
        Obtiene resumen del estado en formato legible.
        
        Returns:
            String con resumen del estado
        """
        if self.is_idle:
            return "Idle - Sin microscopía activa"
        
        if self.is_running:
            return f"Running - Punto {self._current_point + 1}/{self._total_points} ({self.progress_percent:.1f}%)"
        
        if self.is_paused:
            return f"Paused - Punto {self._current_point + 1}/{self._total_points}"
        
        if self.is_stopping:
            return "Stopping - Deteniendo microscopía"
        
        if self.is_completed:
            return f"Completed - {self._image_counter} imágenes capturadas"
        
        if self.is_error:
            return "Error - Microscopía detenida por error"
        
        return "Unknown state"
    
    def __repr__(self) -> str:
        """Representación del estado."""
        return f"MicroscopyStateManager(state={self._state.value}, point={self._current_point}/{self._total_points})"

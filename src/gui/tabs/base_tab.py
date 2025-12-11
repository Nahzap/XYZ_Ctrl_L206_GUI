"""
Clase base para pestañas de la GUI.

Proporciona funcionalidad común para todas las pestañas.
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout


class BaseTab(QWidget):
    """Clase base para pestañas de la interfaz."""
    
    def __init__(self, parent=None):
        """
        Inicializa la pestaña base.
        
        Args:
            parent: Widget padre (típicamente ArduinoGUI)
        """
        super().__init__(parent)
        self.parent = parent
        self.layout = QVBoxLayout(self)
        self.setup_ui()
    
    def setup_ui(self):
        """Configura la interfaz de usuario. Debe ser sobrescrito por subclases."""
        pass
    
    def get_widget(self):
        """Retorna el widget para agregar al QTabWidget."""
        return self

"""
Ventana para mostrar gráficos de matplotlib.

Esta ventana independiente permite mostrar figuras de matplotlib con
interactividad completa, incluyendo barra de herramientas y coordenadas del cursor.
"""

import logging
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from gui.styles.dark_theme import DARK_STYLESHEET

logger = logging.getLogger(__name__)


class MatplotlibWindow(QWidget):
    """Ventana independiente para mostrar gráficos de matplotlib con botón X funcional."""
    
    def __init__(self, figure, title="Gráfico", parent=None):
        """
        Inicializa la ventana de matplotlib.
        
        Args:
            figure: Figura de matplotlib a mostrar
            title: Título de la ventana
            parent: Widget padre (opcional)
        """
        super().__init__(parent, Qt.Window)  # Especificar que es una ventana independiente
        self.setWindowTitle(title)
        self.setGeometry(150, 150, 1000, 800)
        self.setStyleSheet(DARK_STYLESHEET)
        
        # Configurar como ventana independiente
        self.setAttribute(Qt.WA_DeleteOnClose, False)
        
        layout = QVBoxLayout(self)
        
        # Crear canvas de matplotlib
        self.canvas = FigureCanvas(figure)
        layout.addWidget(self.canvas)
        
        # Agregar barra de herramientas interactiva
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.toolbar.setStyleSheet("""
            QToolBar {
                background-color: #383838;
                border: 1px solid #505050;
                padding: 3px;
            }
            QToolButton {
                background-color: #505050;
                border: 1px solid #606060;
                border-radius: 3px;
                padding: 5px;
                margin: 2px;
                color: #F0F0F0;
            }
            QToolButton:hover {
                background-color: #606060;
            }
        """)
        layout.addWidget(self.toolbar)
        
        # Botón para cerrar
        close_btn = QPushButton("Cerrar")
        close_btn.clicked.connect(self.close)
        close_btn.setStyleSheet("font-size: 12px; padding: 8px;")
        layout.addWidget(close_btn)
        
        self.canvas.draw()
        
        # Habilitar cursor interactivo con coordenadas
        self.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        self.coord_label = QLabel("Coordenadas: Mueve el cursor sobre el gráfico")
        self.coord_label.setStyleSheet("color: #5DADE2; font-size: 10px; padding: 5px;")
        layout.insertWidget(2, self.coord_label)  # Insertar después del toolbar
        
        logger.debug(f"MatplotlibWindow creada: {title}")
    
    def on_mouse_move(self, event):
        """Muestra las coordenadas del cursor en tiempo real."""
        if event.inaxes:
            x, y = event.xdata, event.ydata
            self.coord_label.setText(f"📍 Tiempo: {x:.2f} s  |  Valor: {y:.2f}")
        else:
            self.coord_label.setText("Coordenadas: Mueve el cursor sobre el gráfico")
    
    def closeEvent(self, event):
        """Maneja el evento de cierre de la ventana."""
        logger.debug(f"Cerrando ventana: {self.windowTitle()}")
        plt.close('all')  # Cerrar todas las figuras de matplotlib
        event.accept()

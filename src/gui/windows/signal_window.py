"""
Ventana para gráficos de señales en tiempo real.

Esta ventana muestra las señales de control (potencias y sensores) en tiempo real
utilizando PyQtGraph para un rendimiento óptimo.
"""

import logging
import time
import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QCheckBox
from PyQt5.QtCore import Qt
import pyqtgraph as pg
import config.constants as constants
from gui.styles.dark_theme import DARK_STYLESHEET

logger = logging.getLogger(__name__)

# Decimación de plot ~100 Hz (telemetría STM32 puede ser kHz)
_PLOT_MIN_INTERVAL_S = 0.01


class SignalWindow(QWidget):
    """Ventana independiente para visualizar señales en tiempo real."""
    
    def __init__(self, parent=None):
        """
        Inicializa la ventana de señales.
        
        Args:
            parent: Widget padre (opcional)
        """
        super().__init__(parent, Qt.Window)  # Especificar que es una ventana independiente
        self.setWindowTitle('Señales de Control - Tiempo Real')
        self.setGeometry(150, 150, 900, 600)
        self.setStyleSheet(DARK_STYLESHEET)
        self._last_plot_ts = 0.0
        
        layout = QVBoxLayout(self)
        
        # GRÁFICO ULTRA-RÁPIDO - SIN LÍMITES DE FPS
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#252525')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        
        # DESACTIVAR TODO lo que ralentiza
        self.plot_widget.setAntialiasing(False)  # Sin suavizado
        self.plot_widget.setDownsampling(mode='peak')  # Downsample rápido
        self.plot_widget.setClipToView(True)  # Solo renderizar visible
        
        # Desactivar límite de FPS - actualizar TAN RÁPIDO COMO SEA POSIBLE
        pg.setConfigOptions(useOpenGL=False)  # CPU mode más rápido para líneas simples
        
        self.plot_widget.setLabel('left', 'Valor (ADC)', color='#CCCCCC', size='12pt')
        self.plot_widget.setLabel('bottom', 'Muestras', color='#CCCCCC', size='12pt')
        
        self.plot_widget.getAxis('left').setTextPen(color='#CCCCCC')
        self.plot_widget.getAxis('bottom').setTextPen(color='#CCCCCC')
        
        legend = self.plot_widget.addLegend()
        legend.setLabelTextColor('#F0F0F0')
        
        self.refresh_adc_range()
        
        layout.addWidget(self.plot_widget)
        
        # Checkboxes para mostrar/ocultar señales
        checkbox_layout = QHBoxLayout()
        self.checkboxes = {}
        
        for key, name in [("sensor_1", "Sensor 1"), ("sensor_2", "Sensor 2"), 
                          ("power_a", "Potencia A"), ("power_b", "Potencia B")]:
            cb = QCheckBox(name)
            cb.setChecked(True)
            cb.stateChanged.connect(self.update_plot_visibility)
            self.checkboxes[key] = cb
            checkbox_layout.addWidget(cb)
        
        layout.addLayout(checkbox_layout)
        
        # Buffer circular MÍNIMO con NumPy
        self.buffer_size = constants.PLOT_LENGTH
        self.index = 0
        
        # Arrays pre-asignados contiguos en memoria (MÁXIMA velocidad)
        self.data = {
            'power_a': np.zeros(constants.PLOT_LENGTH, dtype=np.int16),  # int16 más rápido que float32
            'power_b': np.zeros(constants.PLOT_LENGTH, dtype=np.int16),
            'sensor_1': np.zeros(constants.PLOT_LENGTH, dtype=np.int16),
            'sensor_2': np.zeros(constants.PLOT_LENGTH, dtype=np.int16),
        }
        
        # LÍNEAS ULTRA-RÁPIDAS - configuración mínima
        # Pens pre-creados (no crear en cada frame)
        pen_a = pg.mkPen('#00FFFF', width=1)  # width=1 más rápido
        pen_b = pg.mkPen('#FF00FF', width=1)
        pen_1 = pg.mkPen('#FFFF00', width=1)
        pen_2 = pg.mkPen('#00FF00', width=1)
        
        self.plot_lines = {
            'power_a': self.plot_widget.plot(pen=pen_a, name="Potencia A"),
            'power_b': self.plot_widget.plot(pen=pen_b, name="Potencia B"),
            'sensor_1': self.plot_widget.plot(pen=pen_1, name="Sensor 1"),
            'sensor_2': self.plot_widget.plot(pen=pen_2, name="Sensor 2"),
        }
        
        logger.debug("SignalWindow creada exitosamente")

    def refresh_adc_range(self):
        """Ajusta el eje Y al ADC_MAX del perfil MCU activo (10-bit / 12-bit)."""
        adc_hi = float(constants.ADC_MAX)
        self.plot_widget.setYRange(0, adc_hi, padding=0)
        logger.debug("SignalWindow YRange -> 0..%s (MCU=%s)", adc_hi, getattr(constants, "MCU_TYPE", "?"))
    
    def update_plot_visibility(self):
        """Muestra u oculta las líneas del gráfico según los checkboxes."""
        for key, cb in self.checkboxes.items():
            if cb.isChecked():
                self.plot_lines[key].show()
            else:
                self.plot_lines[key].hide()
    
    def update_data(self, pot_a, pot_b, sens_1, sens_2):
        """
        Ingiere telemetría en buffer circular; refresca el plot a ~100 Hz.
        """
        idx = self.index
        self.data['power_a'][idx] = abs(pot_a)
        self.data['power_b'][idx] = abs(pot_b)
        self.data['sensor_1'][idx] = sens_1
        self.data['sensor_2'][idx] = sens_2
        self.index = (self.index + 1) % self.buffer_size

        now = time.perf_counter()
        if (now - self._last_plot_ts) < _PLOT_MIN_INTERVAL_S:
            return
        self._last_plot_ts = now

        self.plot_lines['power_a'].setData(self.data['power_a'])
        self.plot_lines['power_b'].setData(self.data['power_b'])
        self.plot_lines['sensor_1'].setData(self.data['sensor_1'])
        self.plot_lines['sensor_2'].setData(self.data['sensor_2'])

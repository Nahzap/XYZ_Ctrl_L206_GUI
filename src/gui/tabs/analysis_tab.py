"""
Pestaña de Análisis de Función de Transferencia.

Encapsula la UI y lógica de análisis de respuesta al escalón.
Usa TransferFunctionAnalyzer para la lógica de identificación.
"""

import logging
import pandas as pd
from matplotlib.figure import Figure

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QGridLayout, QHBoxLayout,
                             QGroupBox, QLabel, QLineEdit, QPushButton,
                             QTextEdit, QCheckBox, QFileDialog)
from PyQt5.QtCore import pyqtSignal

logger = logging.getLogger('MotorControl_L206')


class AnalysisTab(QWidget):
    """
    Pestaña para análisis de función de transferencia.
    
    Signals:
        analysis_completed: Emitido cuando se completa un análisis (dict con resultados)
        show_plot_requested: Emitido cuando se necesita mostrar un gráfico (Figure, title)
    """
    
    analysis_completed = pyqtSignal(dict)
    show_plot_requested = pyqtSignal(object, str)  # Figure, title
    
    def __init__(self, tf_analyzer, parent=None):
        """
        Inicializa la pestaña de análisis.
        
        Args:
            tf_analyzer: Instancia de TransferFunctionAnalyzer
            parent: Widget padre (ArduinoGUI)
        """
        super().__init__(parent)
        self.tf_analyzer = tf_analyzer
        self.parent_gui = parent
        self._setup_ui()
        logger.debug("AnalysisTab inicializado")
    
    def _setup_ui(self):
        """Configura la interfaz de usuario."""
        layout = QVBoxLayout(self)
        
        # Sección 1: Selección de Archivo
        file_group = QGroupBox("📁 Archivo de Datos")
        file_layout = QGridLayout()
        
        file_layout.addWidget(QLabel("Archivo CSV:"), 0, 0)
        self.filename_input = QLineEdit("experimento_escalon.csv")
        self.filename_input.setPlaceholderText("Selecciona o escribe el nombre del archivo...")
        file_layout.addWidget(self.filename_input, 0, 1)
        
        browse_btn = QPushButton("📂 Examinar...")
        browse_btn.clicked.connect(self._browse_file)
        browse_btn.setFixedWidth(120)
        file_layout.addWidget(browse_btn, 0, 2)
        
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)
        
        # Sección 2: Configuración del análisis
        config_group = QGroupBox("⚙️ Configuración")
        config_layout = QGridLayout()
        
        # Selector de Motor
        config_layout.addWidget(QLabel("Motor a analizar:"), 0, 0)
        motor_layout = QHBoxLayout()
        self.motor_a_radio = QCheckBox("Motor A")
        self.motor_b_radio = QCheckBox("Motor B")
        self.motor_a_radio.setChecked(True)
        self.motor_a_radio.stateChanged.connect(lambda: self._toggle_motor('A'))
        self.motor_b_radio.stateChanged.connect(lambda: self._toggle_motor('B'))
        motor_layout.addWidget(self.motor_a_radio)
        motor_layout.addWidget(self.motor_b_radio)
        motor_layout.addStretch()
        config_layout.addLayout(motor_layout, 0, 1, 1, 2)
        
        # Selector de Sensor
        config_layout.addWidget(QLabel("Sensor correspondiente:"), 1, 0)
        sensor_layout = QHBoxLayout()
        self.sensor_1_radio = QCheckBox("Sensor 1")
        self.sensor_2_radio = QCheckBox("Sensor 2")
        self.sensor_1_radio.setChecked(True)
        self.sensor_1_radio.stateChanged.connect(lambda: self._toggle_sensor('1'))
        self.sensor_2_radio.stateChanged.connect(lambda: self._toggle_sensor('2'))
        sensor_layout.addWidget(self.sensor_1_radio)
        sensor_layout.addWidget(self.sensor_2_radio)
        sensor_layout.addStretch()
        config_layout.addLayout(sensor_layout, 1, 1, 1, 2)
        
        # Rango de tiempo
        config_layout.addWidget(QLabel("Tiempo inicio (s):"), 2, 0)
        self.t_inicio_input = QLineEdit("0.0")
        self.t_inicio_input.setFixedWidth(100)
        config_layout.addWidget(self.t_inicio_input, 2, 1)
        
        config_layout.addWidget(QLabel("Tiempo fin (s):"), 2, 2)
        self.t_fin_input = QLineEdit("999.0")
        self.t_fin_input.setFixedWidth(100)
        config_layout.addWidget(self.t_fin_input, 2, 3)
        
        # Distancia real recorrida (para calibración)
        config_layout.addWidget(QLabel("Distancia mín (mm):"), 3, 0)
        self.distancia_min_input = QLineEdit("")
        self.distancia_min_input.setFixedWidth(100)
        self.distancia_min_input.setPlaceholderText("Ej: 10")
        self.distancia_min_input.setToolTip("Distancia real correspondiente al INICIO del tramo.")
        config_layout.addWidget(self.distancia_min_input, 3, 1)
        
        config_layout.addWidget(QLabel("Distancia máx (mm):"), 3, 2)
        self.distancia_max_input = QLineEdit("")
        self.distancia_max_input.setFixedWidth(100)
        self.distancia_max_input.setPlaceholderText("Ej: 20")
        self.distancia_max_input.setToolTip("Distancia real correspondiente al FINAL del tramo.")
        config_layout.addWidget(self.distancia_max_input, 3, 3)
        
        config_group.setLayout(config_layout)
        layout.addWidget(config_group)
        
        # Botones
        buttons_layout = QHBoxLayout()
        view_data_btn = QPushButton("👁️ Ver Datos Completos")
        view_data_btn.clicked.connect(self._view_full_data)
        view_data_btn.setStyleSheet("font-size: 11px; padding: 6px;")
        buttons_layout.addWidget(view_data_btn)
        
        analyze_btn = QPushButton("🔍 Analizar Tramo")
        analyze_btn.clicked.connect(self.run_analysis)
        analyze_btn.setStyleSheet("font-size: 11px; padding: 6px; font-weight: bold; background-color: #3498DB;")
        buttons_layout.addWidget(analyze_btn)
        layout.addLayout(buttons_layout)
        
        # Resultados del análisis
        results_label = QLabel("📊 Resultados del Análisis:")
        results_label.setStyleSheet("font-weight: bold; font-size: 11px; margin-top: 10px;")
        layout.addWidget(results_label)
        
        self.results_text = QTextEdit()
        self.results_text.setReadOnly(True)
        self.results_text.setPlaceholderText("Los resultados del análisis (K, τ) aparecerán aquí...")
        self.results_text.setFixedHeight(360)
        layout.addWidget(self.results_text)
        
        # Lista de funciones de transferencia identificadas
        tf_list_label = QLabel("📋 Funciones de Transferencia Identificadas:")
        tf_list_label.setStyleSheet("font-weight: bold; font-size: 11px; margin-top: 10px;")
        layout.addWidget(tf_list_label)
        
        self.tf_list_text = QTextEdit()
        self.tf_list_text.setReadOnly(True)
        self.tf_list_text.setPlaceholderText("Las funciones de transferencia identificadas se listarán aquí...")
        self.tf_list_text.setFixedHeight(200)
        layout.addWidget(self.tf_list_text)
    
    def _toggle_motor(self, motor):
        """Asegura que solo un motor esté seleccionado."""
        if motor == 'A' and self.motor_a_radio.isChecked():
            self.motor_b_radio.setChecked(False)
        elif motor == 'B' and self.motor_b_radio.isChecked():
            self.motor_a_radio.setChecked(False)
    
    def _toggle_sensor(self, sensor):
        """Asegura que solo un sensor esté seleccionado."""
        if sensor == '1' and self.sensor_1_radio.isChecked():
            self.sensor_2_radio.setChecked(False)
        elif sensor == '2' and self.sensor_2_radio.isChecked():
            self.sensor_1_radio.setChecked(False)
    
    def _browse_file(self):
        """Abre diálogo para seleccionar archivo CSV."""
        logger.info("=== BOTÓN: Examinar archivo presionado ===")
        filename, _ = QFileDialog.getOpenFileName(
            self, "Seleccionar archivo CSV", "", "CSV Files (*.csv);;All Files (*)"
        )
        if filename:
            self.filename_input.setText(filename)
            logger.info(f"Archivo seleccionado: {filename}")
    
    def _view_full_data(self):
        """Muestra gráfico completo del archivo para identificar tramos."""
        logger.info("=== BOTÓN: Ver Datos Completos presionado ===")
        filename = self.filename_input.text()
        
        try:
            df = pd.read_csv(filename)
            logger.info(f"CSV cargado: {len(df)} filas")
            df['Tiempo_s'] = (df['Timestamp_ms'] - df['Timestamp_ms'].iloc[0]) / 1000.0
            
            # Crear figura
            fig = Figure(figsize=(14, 10), facecolor='#2E2E2E')
            axes = fig.subplots(3, 1)
            
            # Gráfico 1: Potencias
            axes[0].plot(df['Tiempo_s'], df['PotenciaA'], label='Motor A', color='magenta', linewidth=1.5)
            axes[0].plot(df['Tiempo_s'], df['PotenciaB'], label='Motor B', color='yellow', linewidth=1.5)
            axes[0].set_title('Entradas de Potencia (PWM)', fontsize=14, fontweight='bold', color='white')
            axes[0].set_ylabel('Potencia (PWM)', color='white')
            axes[0].legend(loc='upper right', facecolor='#383838', edgecolor='#505050', labelcolor='white')
            axes[0].grid(True, alpha=0.5, linestyle='--')
            axes[0].set_facecolor('#252525')
            axes[0].tick_params(colors='white')
            
            # Gráfico 2: Sensor 1
            axes[1].plot(df['Tiempo_s'], df['Sensor1'], label='Sensor 1', color='cyan', linewidth=1.5)
            axes[1].set_title('Sensor 1 (ADC)', fontsize=14, fontweight='bold', color='white')
            axes[1].set_ylabel('Valor ADC', color='white')
            axes[1].legend(loc='upper right', facecolor='#383838', edgecolor='#505050', labelcolor='white')
            axes[1].grid(True, alpha=0.5, linestyle='--')
            axes[1].set_facecolor('#252525')
            axes[1].tick_params(colors='white')
            
            # Gráfico 3: Sensor 2
            axes[2].plot(df['Tiempo_s'], df['Sensor2'], label='Sensor 2', color='lime', linewidth=1.5)
            axes[2].set_title('Sensor 2 (ADC)', fontsize=14, fontweight='bold', color='white')
            axes[2].set_xlabel('Tiempo (s)', color='white')
            axes[2].set_ylabel('Valor ADC', color='white')
            axes[2].legend(loc='upper right', facecolor='#383838', edgecolor='#505050', labelcolor='white')
            axes[2].grid(True, alpha=0.5, linestyle='--')
            axes[2].set_facecolor('#252525')
            axes[2].tick_params(colors='white')
            
            for ax in axes:
                for spine in ax.spines.values():
                    spine.set_color('#505050')
            
            fig.tight_layout()
            
            # Emitir señal para mostrar gráfico
            self.show_plot_requested.emit(fig, "Exploración de Datos Completos")
            
        except Exception as e:
            logger.error(f"Error al cargar datos: {e}")
            self.results_text.setText(f"❌ Error al cargar datos:\n{str(e)}")
    
    def run_analysis(self):
        """Ejecuta análisis de función de transferencia."""
        logger.info("=== BOTÓN: Analizar Tramo presionado ===")
        self.results_text.clear()
        
        # Obtener configuración
        filename = self.filename_input.text()
        motor = 'A' if self.motor_a_radio.isChecked() else 'B'
        sensor = '1' if self.sensor_1_radio.isChecked() else '2'
        
        try:
            t_inicio = float(self.t_inicio_input.text())
            t_fin = float(self.t_fin_input.text())
        except ValueError as e:
            self.results_text.setText("❌ Error: Tiempos deben ser números válidos.")
            return
        
        # Obtener distancias de calibración
        distancia_min_text = self.distancia_min_input.text().strip()
        distancia_max_text = self.distancia_max_input.text().strip()
        distancia_min_mm = float(distancia_min_text) if distancia_min_text else None
        distancia_max_mm = float(distancia_max_text) if distancia_max_text else None
        
        # Ejecutar análisis
        result = self.tf_analyzer.analyze_step_response(
            filename, motor, sensor, t_inicio, t_fin,
            distancia_min_mm, distancia_max_mm
        )
        
        if not result['success']:
            self.results_text.setText(f"❌ {result['message']}")
            return
        
        # Mostrar resultados
        self._display_results(result, motor, sensor, t_inicio, t_fin)
        
        # Actualizar lista de TF
        self.update_tf_list()
        
        # Emitir señal con resultados
        self.analysis_completed.emit(result)
        
        # Mostrar gráfico
        if 'figure' in result:
            self.show_plot_requested.emit(result['figure'], f"Análisis: Motor {motor} → Sensor {sensor}")
        
        logger.info(f"✅ Análisis completado: K={result['K']:.4f}, τ={result['tau']:.4f}s")
    
    def _display_results(self, result, motor, sensor, t_inicio, t_fin):
        """Muestra los resultados del análisis en el widget de texto."""
        K = result['K']
        tau = result['tau']
        tau_slow = result['tau_slow']
        tau_msg = result['tau_msg']
        v_ss = result['v_ss']
        U = result['U']
        calibracion_msg = result['calibracion_msg']
        unidad_velocidad = result['unidad_velocidad']
        sensor_min = result['sensor_min']
        sensor_max = result['sensor_max']
        delta_sensor = result['delta_sensor']
        n_samples = result['n_samples']
        
        results_str = (
            f"✅ Análisis Completado\n"
            f"═══════════════════════════════\n"
            f"Motor: {motor}  |  Sensor: {sensor}\n"
            f"Tramo: {t_inicio:.2f}s → {t_fin:.2f}s ({n_samples} muestras)\n"
            f"───────────────────────────────\n"
            f"Calibración: {calibracion_msg}\n"
            f"───────────────────────────────\n"
            f"Entrada (U):        {U:.2f} PWM\n"
            f"Δ Sensor:           {delta_sensor:.1f} ADC ({sensor_min:.0f}→{sensor_max:.0f})\n"
            f"Velocidad (v_ss):   {v_ss:.2f} {unidad_velocidad}\n"
            f"───────────────────────────────\n"
            f"Ganancia (K):       {K:.4f} {unidad_velocidad}/PWM\n"
            f"Constante (τ):      {tau_msg}\n"
            f"═══════════════════════════════\n"
        )
        
        if tau > 0:
            results_str += f"📐 MODELO IDENTIFICADO:\n"
            results_str += f"───────────────────────────────\n"
            results_str += f"G(s) = K / ((τ₁s + 1)(τ₂s + 1))\n\n"
            results_str += f"Donde:\n"
            results_str += f"  K  = {K:.4f} {unidad_velocidad}/PWM\n"
            results_str += f"  τ₁ = {tau:.4f}s (polo rápido)\n"
            results_str += f"  τ₂ = {tau_slow:.1f}s (polo lento)\n\n"
            results_str += f"Expandido:\n"
            results_str += f"G(s) = {K:.4f} / ({tau*tau_slow:.1f}s² + {tau+tau_slow:.1f}s + 1)\n"
        else:
            results_str += f"G(s) = {K:.4f} / ({tau_slow:.1f}s + 1)  (primer orden)"
        
        self.results_text.setText(results_str)
    
    def update_tf_list(self):
        """Actualiza la lista de funciones de transferencia."""
        list_text = self.tf_analyzer.get_tf_list_text()
        self.tf_list_text.setPlainText(list_text)
        logger.debug(f"Lista TF actualizada: {len(self.tf_analyzer.identified_functions)} entradas")
    
    def get_latest_tf(self):
        """Retorna la última función de transferencia identificada."""
        return self.tf_analyzer.get_latest_tf()
    
    def get_identified_functions(self):
        """Retorna lista de funciones identificadas."""
        return self.tf_analyzer.identified_functions

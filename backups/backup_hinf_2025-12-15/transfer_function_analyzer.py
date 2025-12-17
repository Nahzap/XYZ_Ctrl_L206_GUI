"""
Análisis de función de transferencia a partir de datos experimentales.

Este módulo identifica los parámetros K y τ de funciones de transferencia
a partir de respuestas al escalón.
"""

import logging
import traceback
from datetime import datetime
import pandas as pd
import numpy as np
from matplotlib.figure import Figure

logger = logging.getLogger(__name__)


class TransferFunctionAnalyzer:
    """Analiza datos experimentales para identificar funciones de transferencia."""
    
    def __init__(self):
        """Inicializa el analizador."""
        self.identified_functions = []
        self.global_calibration = None
        self.interpolacion_pendiente = None
        self.interpolacion_intercepto = None
        logger.debug("TransferFunctionAnalyzer inicializado")
    
    def analyze_step_response(self, filename, motor, sensor, t_inicio, t_fin,
                              distancia_min_mm=None, distancia_max_mm=None):
        """
        Analiza una respuesta al escalón y calcula K y τ.
        
        Args:
            filename: Archivo CSV con datos
            motor: 'A' o 'B'
            sensor: '1' o '2'
            t_inicio: Tiempo de inicio del tramo (s)
            t_fin: Tiempo final del tramo (s)
            distancia_min_mm: Distancia al inicio del tramo (mm, opcional)
            distancia_max_mm: Distancia al final del tramo (mm, opcional)
            
        Returns:
            dict: Resultados del análisis con keys:
                - success: bool
                - message: str (mensaje de error o éxito)
                - K: float (ganancia)
                - tau: float (constante de tiempo)
                - tau_slow: float (polo lento)
                - figure: matplotlib Figure
                - calibration_msg: str
                - unidad_posicion: str
                - unidad_velocidad: str
                - v_ss: float
                - U: float
        """
        logger.info(f"=== Iniciando análisis: Motor={motor}, Sensor={sensor}, Archivo={filename} ===")
        logger.debug(f"Rango de tiempo: {t_inicio}s → {t_fin}s")
        
        # Validar parámetros
        if t_inicio >= t_fin:
            error_msg = f"Error: Tiempo inicio ({t_inicio}) debe ser menor que tiempo fin ({t_fin})"
            logger.error(error_msg)
            return {'success': False, 'message': error_msg}
        
        try:
            # 1. Cargar datos
            logger.debug(f"Cargando archivo CSV: {filename}")
            df = pd.read_csv(filename)
            logger.info(f"Archivo cargado: {len(df)} filas totales")
            df['Tiempo_s'] = (df['Timestamp_ms'] - df['Timestamp_ms'].iloc[0]) / 1000.0
            
            # 2. Filtrar por rango de tiempo
            df_tramo = df[(df['Tiempo_s'] >= t_inicio) & (df['Tiempo_s'] <= t_fin)].copy()
            logger.info(f"Tramo filtrado: {len(df_tramo)} muestras en rango [{t_inicio}, {t_fin}]")
            
            if len(df_tramo) < 10:
                error_msg = f"Tramo muy corto ({len(df_tramo)} muestras). Necesita al menos 10."
                logger.error(error_msg)
                return {'success': False, 'message': error_msg}
            
            # 3. Obtener columnas según selección
            motor_col = f'Potencia{motor}'
            sensor_col = f'Sensor{sensor}'
            
            # 4. Calcular entrada promedio en el tramo
            U = df_tramo[motor_col].mean()
            logger.debug(f"Potencia promedio (U): {U:.2f} PWM")
            
            if abs(U) < 1:
                error_msg = f"Potencia muy baja en el tramo (U={U:.2f}). Verifica el rango de tiempo."
                logger.error(error_msg)
                return {'success': False, 'message': error_msg}
            
            # 5. Determinar calibración con interpolación lineal
            calibration_result = self._apply_calibration(
                df_tramo, sensor_col, distancia_min_mm, distancia_max_mm, motor, sensor
            )
            
            df_tramo = calibration_result['df_tramo']
            calibracion_msg = calibration_result['calibracion_msg']
            usar_calibracion = calibration_result['usar_calibracion']
            unidad_posicion = calibration_result['unidad_posicion']
            unidad_velocidad = calibration_result['unidad_velocidad']
            
            # 6. Calcular velocidad
            velocity_result = self._calculate_velocity(df_tramo, unidad_velocidad)
            
            if not velocity_result['success']:
                return {'success': False, 'message': velocity_result['message']}
            
            v_ss = velocity_result['v_ss']
            df_tramo = velocity_result['df_tramo']
            
            # 7. Calcular K (ganancia estática)
            K = v_ss / U
            logger.info(f"Ganancia calculada (K): {K:.4f} {unidad_velocidad}/PWM")
            
            # 8. Calcular tau_fast (constante de tiempo rápida - método del 63.2%)
            tau_result = self._calculate_tau(df_tramo, v_ss, t_inicio)
            tau_fast = tau_result['tau']
            tau_msg = tau_result['tau_msg']
            
            # 9. Agregar polo lento
            tau_slow = 1000.0  # Polo muy lento (1000s)
            logger.info(f"Polo lento agregado: τ_slow = {tau_slow:.1f}s")
            logger.info(f"   Razón: Evitar integrador puro para síntesis H∞/H2")
            
            # 10. Generar gráficos
            figure = self._create_analysis_plots(
                df_tramo, motor, sensor, motor_col, t_inicio, t_fin,
                v_ss, tau_fast, tau_result.get('t_tau'), tau_result.get('v_tau'),
                U, unidad_posicion, unidad_velocidad
            )
            
            # 11. Guardar función de transferencia identificada
            tf_entry = {
                'motor': motor,
                'sensor': sensor,
                'K': K,
                'tau': tau_fast if tau_fast is not None else 0.0,
                'tau_slow': tau_slow,
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'filename': filename,
                'calibration': calibracion_msg
            }
            
            # Actualizar o agregar a la lista
            self._update_tf_list(tf_entry)
            
            logger.info(f"✅ Análisis completado exitosamente: K={K:.4f}, τ={tau_msg}")
            
            return {
                'success': True,
                'message': 'Análisis completado',
                'K': K,
                'tau': tau_fast if tau_fast is not None else 0.0,
                'tau_slow': tau_slow,
                'tau_msg': tau_msg,
                'v_ss': v_ss,
                'U': U,
                'figure': figure,
                'calibracion_msg': calibracion_msg,
                'unidad_posicion': unidad_posicion,
                'unidad_velocidad': unidad_velocidad,
                'sensor_min': df_tramo[sensor_col].min(),
                'sensor_max': df_tramo[sensor_col].max(),
                'delta_sensor': df_tramo[sensor_col].max() - df_tramo[sensor_col].min(),
                'n_samples': len(df_tramo)
            }
            
        except FileNotFoundError:
            error_msg = f"Archivo '{filename}' no encontrado."
            logger.error(error_msg)
            return {'success': False, 'message': error_msg}
        except Exception as e:
            error_msg = f"Error en análisis: {str(e)}\n\n{traceback.format_exc()}"
            logger.error(f"Error en análisis: {e}\n{traceback.format_exc()}")
            return {'success': False, 'message': error_msg}
    
    def _apply_calibration(self, df_tramo, sensor_col, distancia_min_mm, distancia_max_mm, motor, sensor):
        """Aplica calibración por interpolación lineal si se proporcionan distancias."""
        usar_calibracion = False
        
        if distancia_min_mm is not None and distancia_max_mm is not None:
            try:
                # Convertir a µm
                distancia_min_um = distancia_min_mm * 1000.0
                distancia_max_um = distancia_max_mm * 1000.0
                
                # Obtener valores ADC del tramo
                sensor_inicial = df_tramo[sensor_col].iloc[0]
                sensor_final = df_tramo[sensor_col].iloc[-1]
                
                # Calcular delta
                delta_sensor_adc = abs(sensor_final - sensor_inicial)
                
                if delta_sensor_adc > 1:
                    # Crear pares (ADC, distancia)
                    punto1_adc = sensor_inicial
                    punto1_dist_um = distancia_min_um
                    punto2_adc = sensor_final
                    punto2_dist_um = distancia_max_um
                    
                    # Determinar relación
                    if (punto2_adc > punto1_adc and punto2_dist_um > punto1_dist_um) or \
                       (punto2_adc < punto1_adc and punto2_dist_um < punto1_dist_um):
                        relacion = "DIRECTA"
                    else:
                        relacion = "INVERSA"
                    
                    logger.info(f"🎯 Relación {relacion} detectada")
                    
                    # Calcular interpolación lineal
                    pendiente = (punto2_dist_um - punto1_dist_um) / (punto2_adc - punto1_adc)
                    intercepto = punto1_dist_um - pendiente * punto1_adc
                    
                    # Aplicar interpolación
                    df_tramo['Posicion_um'] = df_tramo[sensor_col] * pendiente + intercepto
                    
                    logger.info(f"📐 Interpolación lineal configurada:")
                    logger.info(f"   Pendiente: {pendiente:.4f} µm/ADC")
                    logger.info(f"   Intercepto: {intercepto:.2f} µm")
                    
                    calibracion_msg = f"✅ Interpolado: {distancia_min_mm}→{distancia_max_mm} mm ({relacion})"
                    usar_calibracion = True
                    unidad_posicion = "µm"
                    unidad_velocidad = "µm/s"
                    
                    # Guardar parámetros de interpolación
                    self.interpolacion_pendiente = pendiente
                    self.interpolacion_intercepto = intercepto
                    
                    # Guardar calibración global
                    self.global_calibration = {
                        'adc_punto1': punto1_adc,
                        'adc_punto2': punto2_adc,
                        'dist_punto1_mm': distancia_min_mm,
                        'dist_punto2_mm': distancia_max_mm,
                        'pendiente_mm': pendiente / 1000.0,
                        'intercepto_mm': intercepto / 1000.0,
                        'pendiente_um': pendiente,
                        'intercepto_um': intercepto,
                        'relacion': relacion,
                        'motor': motor,
                        'sensor': sensor
                    }
                    
                    logger.info(f"✅ Calibración global guardada")
                    
                else:
                    logger.warning(f"Δ Sensor muy pequeño ({delta_sensor_adc:.2f}), mostrando datos crudos")
                    df_tramo['Posicion_um'] = df_tramo[sensor_col]
                    calibracion_msg = f"⚠️ Datos crudos en ADC (Δ sensor insuficiente: {delta_sensor_adc:.2f})"
                    usar_calibracion = False
                    unidad_posicion = "ADC"
                    unidad_velocidad = "ADC/s"
                    
            except (ValueError, TypeError) as e:
                logger.warning(f"Distancias inválidas, mostrando datos crudos: {e}")
                df_tramo['Posicion_um'] = df_tramo[sensor_col]
                calibracion_msg = f"⚠️ Datos crudos en ADC (valores inválidos)"
                usar_calibracion = False
                unidad_posicion = "ADC"
                unidad_velocidad = "ADC/s"
        else:
            # SIN distancias → Mostrar datos crudos en ADC
            df_tramo['Posicion_um'] = df_tramo[sensor_col]
            calibracion_msg = f"📊 Datos crudos en ADC (sin calibración)"
            usar_calibracion = False
            unidad_posicion = "ADC"
            unidad_velocidad = "ADC/s"
            logger.debug("Sin distancias ingresadas - mostrando datos crudos en ADC")
        
        return {
            'df_tramo': df_tramo,
            'calibracion_msg': calibracion_msg,
            'usar_calibracion': usar_calibracion,
            'unidad_posicion': unidad_posicion,
            'unidad_velocidad': unidad_velocidad
        }
    
    def _calculate_velocity(self, df_tramo, unidad_velocidad):
        """Calcula velocidad estacionaria y velocidad instantánea."""
        logger.debug("Calculando velocidad...")
        
        # Usar el último 20% del tramo para calcular v_ss
        idx_80 = int(len(df_tramo) * 0.8)
        pos_inicial = df_tramo['Posicion_um'].iloc[idx_80:idx_80+10].mean()
        pos_final = df_tramo['Posicion_um'].iloc[-10:].mean()
        
        t_inicial = df_tramo['Tiempo_s'].iloc[idx_80]
        t_final = df_tramo['Tiempo_s'].iloc[-1]
        
        delta_t = t_final - t_inicial
        
        if delta_t > 0.1:  # Al menos 100ms de datos
            v_ss = (pos_final - pos_inicial) / delta_t
        else:
            v_ss = 0.0
        
        logger.info(f"Velocidad estacionaria (v_ss): {v_ss:.4f} {unidad_velocidad}")
        
        # Calcular velocidad instantánea para graficar
        df_tramo['Velocidad_um_s'] = df_tramo['Posicion_um'].diff() / df_tramo['Tiempo_s'].diff()
        df_tramo['Velocidad_um_s'] = df_tramo['Velocidad_um_s'].replace([float('inf'), float('-inf')], float('nan'))
        df_tramo['Velocidad_um_s'] = df_tramo['Velocidad_um_s'].rolling(window=20, center=True, min_periods=1).mean()
        df_tramo['Velocidad_um_s'] = df_tramo['Velocidad_um_s'].fillna(v_ss)
        
        if abs(v_ss) < 0.01:
            error_msg = f"Velocidad estacionaria muy baja (v_ss={v_ss:.4f} {unidad_velocidad}). Sistema no se mueve."
            logger.error(error_msg)
            return {'success': False, 'message': error_msg}
        
        return {
            'success': True,
            'v_ss': v_ss,
            'df_tramo': df_tramo
        }
    
    def _calculate_tau(self, df_tramo, v_ss, t_inicio):
        """Calcula constante de tiempo usando método del 63.2%."""
        v_tau = v_ss * 0.632
        logger.debug(f"Buscando τ_fast en v_tau = {v_tau:.4f} (63.2% de v_ss)")
        
        # Buscar primer punto donde velocidad >= v_tau
        tau_candidates = df_tramo[df_tramo['Velocidad_um_s'] >= v_tau]
        
        if tau_candidates.empty:
            tau_fast = None
            tau_msg = "No calculado (no alcanzó 63.2%)"
            t_tau = None
            logger.warning("No se alcanzó 63.2% de v_ss, τ_fast no calculado")
        else:
            t_tau = tau_candidates.iloc[0]['Tiempo_s']
            tau_fast = t_tau - t_inicio
            tau_msg = f"{tau_fast:.4f} s"
            logger.info(f"Constante de tiempo rápida (τ_fast): {tau_fast:.4f} s")
        
        return {
            'tau': tau_fast,
            'tau_msg': tau_msg,
            't_tau': t_tau,
            'v_tau': v_tau
        }
    
    def _create_analysis_plots(self, df_tramo, motor, sensor, motor_col, t_inicio, t_fin,
                               v_ss, tau, t_tau, v_tau, U, unidad_posicion, unidad_velocidad):
        """Crea los gráficos de análisis."""
        fig = Figure(figsize=(12, 10), facecolor='#2E2E2E')
        axes = fig.subplots(3, 1)
        
        # Gráfico 1: Posición
        axes[0].plot(df_tramo['Tiempo_s'], df_tramo['Posicion_um'], 
                    label=f'Posición (Sensor {sensor})', color='cyan', linewidth=2)
        axes[0].axvline(x=t_inicio, color='red', linestyle='--', alpha=0.7, linewidth=2, label='Inicio tramo')
        axes[0].axvline(x=t_fin, color='red', linestyle='--', alpha=0.7, linewidth=2, label='Fin tramo')
        axes[0].set_title(f'Motor {motor} → Sensor {sensor}: Posición', fontsize=14, fontweight='bold', color='white')
        axes[0].set_ylabel(f'Posición ({unidad_posicion})', color='white')
        axes[0].legend(loc='best', facecolor='#383838', edgecolor='#505050', labelcolor='white')
        axes[0].grid(True, alpha=0.5, linestyle='--', linewidth=0.5)
        axes[0].set_facecolor('#252525')
        axes[0].tick_params(colors='white')
        for spine in axes[0].spines.values():
            spine.set_color('#505050')
        
        # Gráfico 2: Velocidad
        axes[1].plot(df_tramo['Tiempo_s'], df_tramo['Velocidad_um_s'], 
                    label='Velocidad', color='lime', linewidth=2)
        axes[1].axhline(y=v_ss, color='red', linestyle='--', linewidth=2, alpha=0.8,
                       label=f'v_ss = {v_ss:.2f} {unidad_velocidad}')
        if tau is not None and t_tau is not None and v_tau is not None:
            axes[1].axhline(y=v_tau, color='orange', linestyle=':', linewidth=2, alpha=0.8,
                           label=f'63.2% = {v_tau:.2f} {unidad_velocidad}')
            axes[1].axvline(x=t_tau, color='orange', linestyle=':', linewidth=2, alpha=0.8,
                           label=f'τ = {tau:.4f} s')
        axes[1].set_title('Velocidad (derivada de posición)', fontsize=14, fontweight='bold', color='white')
        axes[1].set_ylabel(f'Velocidad ({unidad_velocidad})', color='white')
        axes[1].legend(loc='best', facecolor='#383838', edgecolor='#505050', labelcolor='white')
        axes[1].grid(True, alpha=0.5, linestyle='--', linewidth=0.5)
        axes[1].set_facecolor('#252525')
        axes[1].tick_params(colors='white')
        for spine in axes[1].spines.values():
            spine.set_color('#505050')
        
        # Gráfico 3: Entrada
        axes[2].plot(df_tramo['Tiempo_s'], df_tramo[motor_col], 
                    label=f'Motor {motor}', color='magenta', linewidth=2)
        axes[2].axhline(y=U, color='yellow', linestyle='--', linewidth=2, alpha=0.8,
                       label=f'U promedio = {U:.2f}')
        axes[2].set_title('Entrada de Potencia', fontsize=14, fontweight='bold', color='white')
        axes[2].set_xlabel('Tiempo (s)', color='white', fontsize=12)
        axes[2].set_ylabel('Potencia (PWM)', color='white')
        axes[2].legend(loc='best', facecolor='#383838', edgecolor='#505050', labelcolor='white')
        axes[2].grid(True, alpha=0.5, linestyle='--', linewidth=0.5)
        axes[2].set_facecolor('#252525')
        axes[2].tick_params(colors='white')
        for spine in axes[2].spines.values():
            spine.set_color('#505050')
        
        fig.tight_layout()
        return fig
    
    def _update_tf_list(self, tf_entry):
        """Actualiza la lista de funciones de transferencia identificadas."""
        motor = tf_entry['motor']
        sensor = tf_entry['sensor']
        
        # Verificar si ya existe esta combinación motor/sensor
        existing_idx = None
        for idx, tf in enumerate(self.identified_functions):
            if tf['motor'] == motor and tf['sensor'] == sensor:
                existing_idx = idx
                break
        
        if existing_idx is not None:
            # Actualizar entrada existente
            self.identified_functions[existing_idx] = tf_entry
            logger.info(f"Función de transferencia actualizada: Motor {motor} / Sensor {sensor}")
        else:
            # Agregar nueva entrada
            self.identified_functions.append(tf_entry)
            logger.info(f"Nueva función de transferencia agregada: Motor {motor} / Sensor {sensor}")
    
    def get_tf_list_text(self):
        """Genera texto formateado de la lista de funciones de transferencia."""
        if not self.identified_functions:
            return "No hay funciones de transferencia identificadas aún.\n\nRealiza un análisis para agregar una."
        
        list_text = "═══════════════════════════════════════════════════════════════\n"
        list_text += "  FUNCIONES DE TRANSFERENCIA IDENTIFICADAS\n"
        list_text += "═══════════════════════════════════════════════════════════════\n\n"
        
        for idx, tf in enumerate(self.identified_functions, 1):
            motor = tf['motor']
            sensor = tf['sensor']
            K = tf['K']
            tau = tf['tau']
            tau_slow = tf.get('tau_slow', 1000.0)
            timestamp = tf['timestamp']
            
            list_text += f"[{idx}] Motor {motor} / Sensor {sensor}\n"
            list_text += f"    ├─ G(s) = {K:.4f} / (({tau:.4f}s + 1)({tau_slow:.1f}s + 1))\n"
            list_text += f"    ├─ K = {K:.4f} µm/s/PWM\n"
            list_text += f"    ├─ τ₁ = {tau:.4f}s (rápido), τ₂ = {tau_slow:.1f}s (lento)\n"
            list_text += f"    ├─ Fecha: {timestamp}\n"
            list_text += f"    └─ Archivo: {tf['filename']}\n\n"
        
        list_text += "═══════════════════════════════════════════════════════════════\n"
        list_text += f"Total: {len(self.identified_functions)} función(es) identificada(s)\n"
        list_text += "═══════════════════════════════════════════════════════════════\n"
        
        return list_text
    
    def get_latest_tf(self):
        """Obtiene la última función de transferencia identificada."""
        if self.identified_functions:
            return self.identified_functions[-1]
        return None
    
    def clear_tf_list(self):
        """Limpia la lista de funciones de transferencia."""
        self.identified_functions = []
        logger.info("Lista de funciones de transferencia limpiada")

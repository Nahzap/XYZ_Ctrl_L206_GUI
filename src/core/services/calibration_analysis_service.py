"""
Servicio de Análisis de Calibración.

Genera gráficos de análisis completo para verificar que el controlador H∞
está correctamente linealizado por la función de transferencia.

Incluye:
- Sensor ADC vs Tiempo
- Posición con línea de homogeneidad y barras de error
- PWM vs Tiempo
- Respuesta al escalón: Predicción (fitted) vs Real

Creado: 2026-01-07
"""

import logging
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from matplotlib.figure import Figure

logger = logging.getLogger('MotorControl_L206')


class CalibrationAnalysisService:
    """Servicio para generar gráficos de análisis de calibración."""
    
    @staticmethod
    def _extract_tf_params_from_log(motor):
        """
        Extrae parámetros K y τ del log más reciente.
        
        Args:
            motor: 'A' o 'B'
            
        Returns:
            tuple: (K, tau)
        """
        import os
        import re
        
        # Buscar log más reciente
        base_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
        log_files = [f for f in os.listdir(base_path) if f.startswith('motor_control_') and f.endswith('.log')]
        
        if not log_files:
            logger.warning(f"No se encontró log, usando valores por defecto para Motor {motor}")
            return (0.9958, 0.0350) if motor == 'A' else (3.3136, 0.0050)
        
        # Usar el log más reciente
        log_file = sorted(log_files)[-1]
        log_path = os.path.join(base_path, log_file)
        
        logger.info(f"Extrayendo parámetros de Motor {motor} desde: {log_file}")
        
        K = None
        tau = None
        
        # Buscar en el log
        try:
            with open(log_path, 'r', encoding='utf-8') as f:
                for line in f:
                    # Buscar líneas de análisis para el motor específico
                    if f'Motor={motor}' in line and 'Ganancia calculada' in line:
                        match = re.search(r'K[:\s=]+([0-9.]+)', line)
                        if match:
                            K = float(match.group(1))
                    
                    if f'Motor={motor}' in line and 'Constante de tiempo rápida' in line:
                        match = re.search(r'τ_fast[:\s=]+([0-9.]+)', line)
                        if match:
                            tau = float(match.group(1))
                    
                    # También buscar en formato "K=X.XXXX, τ=X.XXXX"
                    if f'Motor {motor}' in line or f'Motor{motor}' in line:
                        k_match = re.search(r'K=([0-9.]+)', line)
                        tau_match = re.search(r'τ=([0-9.]+)', line)
                        if k_match:
                            K = float(k_match.group(1))
                        if tau_match:
                            tau = float(tau_match.group(1))
        except Exception as e:
            logger.error(f"Error leyendo log: {e}")
        
        # Valores por defecto si no se encontraron
        if K is None or tau is None:
            logger.warning(f"No se encontraron parámetros en log para Motor {motor}, usando valores por defecto")
            K = 0.9958 if motor == 'A' else 3.3136
            tau = 0.0350 if motor == 'A' else 0.0050
        
        logger.info(f"Motor {motor}: K={K:.4f} µm/s/PWM, τ={tau:.4f} s")
        return K, tau
    
    @staticmethod
    def find_calibration_files(base_path=None):
        """
        Busca automáticamente archivos CSV de calibración.
        
        Args:
            base_path: Ruta base para buscar (default: directorio del proyecto)
            
        Returns:
            dict: {'motor_a': path, 'motor_b': path} o None si no encuentra
        """
        if base_path is None:
            # Buscar en el directorio raíz del proyecto
            base_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
        
        logger.info(f"Buscando archivos de calibración en: {base_path}")
        
        # Patrones de búsqueda
        patterns_a = ['MotorA_Sensor2*.csv', 'motor_a*.csv', 'calibration_a*.csv']
        patterns_b = ['MotorB_Sensor1*.csv', 'motor_b*.csv', 'calibration_b*.csv']
        
        motor_a_file = None
        motor_b_file = None
        
        # Buscar archivos
        for file in os.listdir(base_path):
            if file.endswith('.csv'):
                file_lower = file.lower()
                
                # Motor A
                if 'motora' in file_lower and 'sensor2' in file_lower:
                    motor_a_file = os.path.join(base_path, file)
                    logger.info(f"  ✅ Motor A encontrado: {file}")
                
                # Motor B
                if 'motorb' in file_lower and 'sensor1' in file_lower:
                    motor_b_file = os.path.join(base_path, file)
                    logger.info(f"  ✅ Motor B encontrado: {file}")
        
        if motor_a_file and motor_b_file:
            return {'motor_a': motor_a_file, 'motor_b': motor_b_file}
        else:
            logger.warning("No se encontraron archivos de calibración")
            return None
    
    @staticmethod
    def load_motor_data(csv_path, motor_name):
        """Carga datos de calibración del motor."""
        logger.info(f"Cargando datos de {motor_name} desde {csv_path}")
        df = pd.read_csv(csv_path)
        
        time_ms = df['Timestamp_ms'].values
        time_s = time_ms / 1000.0
        
        if motor_name == "Motor A":
            sensor_adc = df['Sensor2'].values
            pwm = df['PotenciaA'].values
        else:  # Motor B
            sensor_adc = df['Sensor1'].values
            pwm = df['PotenciaB'].values
        
        logger.info(f"  {len(time_s)} muestras cargadas")
        
        return time_s, sensor_adc, pwm
    
    @staticmethod
    def calculate_calibration(sensor_adc, motor_name):
        """
        Calcula calibración lineal del sensor usando datos de calibration.json.
        
        Args:
            sensor_adc: Valores ADC del sensor
            motor_name: "Motor A" o "Motor B"
            
        Returns:
            tuple: (sensor_um, intercept, slope)
        """
        from config.constants import CALIBRATION_X, CALIBRATION_Y
        
        if motor_name == "Motor A":
            cal = CALIBRATION_X
        else:  # Motor B
            cal = CALIBRATION_Y
        
        intercept = cal['intercept']
        slope = cal['slope']
        
        # Aplicar calibración (relación inversa para estos sensores)
        sensor_um = intercept - slope * sensor_adc
        
        logger.info(f"{motor_name}: intercept={intercept:.2f}µm, slope={slope:.4f}µm/ADC")
        
        return sensor_um, intercept, slope
    
    @staticmethod
    def extract_tf_params_from_csv(time_s, sensor_um, pwm, motor_name):
        """
        Extrae parámetros K y τ directamente del CSV analizando la respuesta al escalón.
        
        Args:
            time_s: Array de tiempo en segundos
            sensor_um: Array de posición en µm
            pwm: Array de señal PWM
            motor_name: "Motor A" o "Motor B"
            
        Returns:
            tuple: (K, tau) donde K es ganancia (µm/s/PWM) y tau es constante de tiempo (s)
        """
        logger.info(f"[{motor_name}] Extrayendo parámetros K y τ del CSV...")
        
        # Detectar escalón de PWM (cambio > 50)
        pwm_diff = np.diff(pwm)
        step_indices = np.where(np.abs(pwm_diff) > 50)[0]
        
        if len(step_indices) == 0:
            logger.warning(f"[{motor_name}] No se detectó escalón de PWM, usando valores por defecto")
            K = 0.9958 if motor_name == "Motor A" else 3.3136
            tau = 0.0350 if motor_name == "Motor A" else 0.0050
            return K, tau
        
        step_start = step_indices[0]
        logger.debug(f"[{motor_name}] Escalón detectado en índice {step_start}")
        
        # Determinar ventana de análisis (5 segundos después del escalón)
        dt_mean = np.mean(np.diff(time_s))
        if dt_mean > 0:
            window_size = int(5.0 / dt_mean)
        else:
            window_size = 500
        
        window_end = min(step_start + window_size, len(time_s))
        
        # Extraer ventana de análisis
        t_window = time_s[step_start:window_end]
        pos_window = sensor_um[step_start:window_end]
        pwm_window = pwm[step_start:window_end]
        
        # Calcular PWM del escalón (promedio después del escalón)
        step_pwm = np.mean(pwm_window[10:min(50, len(pwm_window))])
        logger.debug(f"[{motor_name}] PWM del escalón: {step_pwm:.2f}")
        
        # Calcular velocidad en régimen permanente
        # Usar últimos 20% de datos para estimar régimen permanente
        steady_start = int(len(pos_window) * 0.8)
        if steady_start < len(pos_window) - 10:
            pos_steady = pos_window[steady_start:]
            t_steady = t_window[steady_start:]
            
            # Ajuste lineal para obtener velocidad
            if len(t_steady) > 2:
                coeffs = np.polyfit(t_steady, pos_steady, 1)
                v_ss = abs(coeffs[0])  # Velocidad en µm/s
            else:
                # Fallback: diferencia entre último y primer punto
                v_ss = abs(pos_window[-1] - pos_window[0]) / (t_window[-1] - t_window[0])
        else:
            v_ss = abs(pos_window[-1] - pos_window[0]) / (t_window[-1] - t_window[0])
        
        logger.debug(f"[{motor_name}] Velocidad en régimen permanente: {v_ss:.4f} µm/s")
        
        # Calcular ganancia K = v_ss / PWM
        if abs(step_pwm) > 1:
            K = v_ss / abs(step_pwm)
        else:
            logger.warning(f"[{motor_name}] PWM muy bajo, usando valor por defecto para K")
            K = 0.9958 if motor_name == "Motor A" else 3.3136
        
        logger.info(f"[{motor_name}] Ganancia calculada: K = {K:.4f} µm/s/PWM")
        
        # Calcular constante de tiempo τ (método del 63.2%)
        pos_initial = pos_window[0]
        pos_final = pos_window[-1]
        pos_range = abs(pos_final - pos_initial)
        
        # Valor al 63.2% del cambio total
        if step_pwm > 0:  # Movimiento positivo
            pos_632 = pos_initial + 0.632 * pos_range
        else:  # Movimiento negativo
            pos_632 = pos_initial - 0.632 * pos_range
        
        # Buscar el tiempo donde se alcanza el 63.2%
        t_rel = t_window - t_window[0]  # Tiempo relativo desde el escalón
        
        # Encontrar índice más cercano al 63.2%
        if step_pwm > 0:
            candidates = np.where(pos_window >= pos_632)[0]
        else:
            candidates = np.where(pos_window <= pos_632)[0]
        
        if len(candidates) > 0:
            idx_632 = candidates[0]
            tau = t_rel[idx_632]
            logger.info(f"[{motor_name}] Constante de tiempo: τ = {tau:.4f} s (método 63.2%)")
        else:
            # Fallback: usar 10% del tiempo total de la ventana
            tau = (t_window[-1] - t_window[0]) * 0.1
            logger.warning(f"[{motor_name}] No se encontró punto 63.2%, usando τ = {tau:.4f} s (estimado)")
        
        return K, tau
    
    @staticmethod
    def validate_tf_params(K, tau, motor):
        """
        Valida que los parámetros K y τ estén en rangos razonables.
        
        Args:
            K: Ganancia (µm/s/PWM)
            tau: Constante de tiempo (s)
            motor: 'A' o 'B'
            
        Returns:
            bool: True si los parámetros son válidos
        """
        # Rangos esperados basados en logs históricos
        if motor == 'A':
            K_range = (0.5, 2.0)      # µm/s/PWM
            tau_range = (0.01, 0.1)   # segundos
        else:  # Motor B
            K_range = (2.0, 5.0)
            tau_range = (0.001, 0.05)
        
        valid = True
        
        if not (K_range[0] <= K <= K_range[1]):
            logger.warning(f"Motor {motor}: K={K:.4f} fuera de rango esperado {K_range}")
            valid = False
        
        if not (tau_range[0] <= tau <= tau_range[1]):
            logger.warning(f"Motor {motor}: τ={tau:.4f} fuera de rango esperado {tau_range}")
            valid = False
        
        if valid:
            logger.info(f"✅ Motor {motor}: Parámetros validados - K={K:.4f}, τ={tau:.4f}")
        
        return valid
    
    @staticmethod
    def generate_analysis_figure(time_s, sensor_adc, sensor_um, pwm, motor_name, K, tau, intercept, slope):
        """
        Genera figura de análisis completo con 4 gráficas.
        
        Returns:
            Figure: Figura de matplotlib
        """
        fig, axes = plt.subplots(2, 2, figsize=(18, 12))
        fig.suptitle(f'Análisis de Calibración - {motor_name}', fontsize=16, fontweight='bold', y=0.995)
        
        # ============================================================
        # GRÁFICA 1: Sensor ADC vs Tiempo
        # ============================================================
        ax1 = axes[0, 0]
        ax1.plot(time_s, sensor_adc, 'b-', linewidth=1.5, alpha=0.8, label='Sensor ADC')
        ax1.set_xlabel('Tiempo (s)', fontweight='bold', fontsize=11)
        ax1.set_ylabel('Sensor (ADC)', fontweight='bold', fontsize=11)
        ax1.set_title('Respuesta del Sensor (ADC)', fontweight='bold', fontsize=12)
        ax1.legend(loc='best')
        ax1.grid(True, alpha=0.4)
        
        ax1.text(0.02, 0.98, f'Rango: {sensor_adc.min():.0f} - {sensor_adc.max():.0f} ADC\n'
                              f'Media: {sensor_adc.mean():.1f} ADC\n'
                              f'σ: {sensor_adc.std():.2f} ADC',
                 transform=ax1.transAxes, fontsize=9, verticalalignment='top',
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # ============================================================
        # GRÁFICA 2: Posición con Homogeneidad y Barras de Error
        # ============================================================
        ax2 = axes[0, 1]
        
        ax2.plot(time_s, sensor_um, 'g-', linewidth=3, alpha=0.9, label='Posición medida', zorder=2)
        
        z = np.polyfit(time_s, sensor_um, 1)
        p = np.poly1d(z)
        ideal_um = p(time_s)
        ax2.plot(time_s, ideal_um, 'r--', linewidth=4, alpha=0.7, 
                label='Homogeneidad ideal', zorder=3)
        
        deviation = sensor_um - ideal_um
        ax2.fill_between(time_s, ideal_um - np.abs(deviation).mean(), 
                         ideal_um + np.abs(deviation).mean(), 
                         color='red', alpha=0.15, label=f'Desviación: ±{np.abs(deviation).mean():.1f}µm')
        
        window_size = len(time_s) // 15
        for i in range(0, len(time_s), window_size):
            end_idx = min(i + window_size, len(time_s))
            t_window = time_s[i:end_idx].mean()
            um_mean = sensor_um[i:end_idx].mean()
            um_std = sensor_um[i:end_idx].std()
            ax2.errorbar(t_window, um_mean, yerr=um_std*2, fmt='o', color='darkorange', 
                        markersize=8, capsize=5, capthick=2, alpha=0.8, linewidth=2, zorder=4)
        
        ax2.set_xlabel('Tiempo (s)', fontweight='bold', fontsize=11)
        ax2.set_ylabel('Posición (µm)', fontweight='bold', fontsize=11)
        ax2.set_title('Posición con Línea de Homogeneidad y Barras de Error', fontweight='bold', fontsize=12)
        ax2.legend(loc='best', fontsize=9, framealpha=0.9)
        ax2.grid(True, alpha=0.4)
        
        ax2.text(0.02, 0.02, f'Calibración: µm = {intercept:.2f} - {slope:.4f} × ADC',
                 transform=ax2.transAxes, fontsize=9, verticalalignment='bottom',
                 bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
        
        # ============================================================
        # GRÁFICA 3: PWM vs Tiempo
        # ============================================================
        ax3 = axes[1, 0]
        ax3.plot(time_s, pwm, 'purple', linewidth=1.5, alpha=0.8, label='PWM aplicado')
        ax3.axhline(y=100, color='r', linestyle='--', alpha=0.5, label='PWM máximo')
        ax3.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
        ax3.set_xlabel('Tiempo (s)', fontweight='bold', fontsize=11)
        ax3.set_ylabel('PWM', fontweight='bold', fontsize=11)
        ax3.set_title('Señal de Control (PWM)', fontweight='bold', fontsize=12)
        ax3.legend(loc='best')
        ax3.grid(True, alpha=0.4)
        
        # ============================================================
        # GRÁFICA 4: RESPUESTA AL ESCALÓN - Predicción vs Real (FITTED)
        # ============================================================
        ax4 = axes[1, 1]
        
        pwm_diff = np.diff(pwm)
        step_idx = np.where(np.abs(pwm_diff) > 50)[0]
        
        if len(step_idx) > 0:
            step_start = step_idx[0]
            
            dt_mean = np.mean(np.diff(time_s))
            if dt_mean > 0:
                window_end = min(step_start + int(5.0 / dt_mean), len(time_s))
            else:
                window_end = min(step_start + 500, len(time_s))
            
            t_window = time_s[step_start:window_end] - time_s[step_start]
            pos_window = sensor_um[step_start:window_end]
            pwm_window = pwm[step_start:window_end]
            pos_initial = pos_window[0]
            step_pwm = pwm_window[min(10, len(pwm_window)-1)]
            
            # FIT del modelo a datos reales
            def position_model(t, K_fit, tau_fit, p0_fit):
                return p0_fit + K_fit * step_pwm * (t + tau_fit * np.exp(-t/tau_fit) - tau_fit)
            
            try:
                p0_bounds = ([K*0.1, tau*0.1, pos_initial*0.9], 
                            [K*10, tau*10, pos_initial*1.1])
                
                popt, pcov = curve_fit(
                    position_model, 
                    t_window, 
                    pos_window,
                    p0=[K, tau, pos_initial],
                    bounds=p0_bounds,
                    maxfev=5000
                )
                
                K_fitted, tau_fitted, p0_fitted = popt
                pos_predicted = position_model(t_window, K_fitted, tau_fitted, p0_fitted)
                
                residuals = pos_window - pos_predicted
                ss_res = np.sum(residuals**2)
                ss_tot = np.sum((pos_window - np.mean(pos_window))**2)
                r_squared = 1 - (ss_res / ss_tot)
                
                logger.info(f"  {motor_name} Fit: K={K_fitted:.4f}, τ={tau_fitted:.4f}, R²={r_squared:.4f}")
                
            except Exception as e:
                logger.warning(f"  Fit falló para {motor_name}: {e}")
                pos_predicted = position_model(t_window, K, tau, pos_initial)
                K_fitted, tau_fitted = K, tau
                r_squared = 0.0
            
            velocity_steady = K * step_pwm
            
            ax4.plot(t_window, pos_window, 'g-', linewidth=3, alpha=0.9, 
                    label='Posición Real Medida', zorder=2)
            
            ax4.plot(t_window, pos_predicted, 'r--', linewidth=3, alpha=0.8, 
                    label=f'Predicción Modelo G(s)', zorder=3)
            
            pos_real_mark = 0
            pos_pred_mark = 0
            if len(t_window) > 100:
                t_mark = 1.0
                idx_mark = np.argmin(np.abs(t_window - t_mark))
                pos_real_mark = pos_window[idx_mark]
                pos_pred_mark = pos_predicted[idx_mark]
                
                ax4.plot(t_mark, pos_real_mark, 'go', markersize=12, 
                        markeredgecolor='darkgreen', markeredgewidth=2, zorder=5,
                        label=f'Real @ t={t_mark}s: {pos_real_mark:.1f}µm')
                ax4.plot(t_mark, pos_pred_mark, 'ro', markersize=12, 
                        markeredgecolor='darkred', markeredgewidth=2, zorder=5,
                        label=f'Predicción @ t={t_mark}s: {pos_pred_mark:.1f}µm')
                
                ax4.axvline(x=t_mark, color='blue', linestyle=':', alpha=0.5, linewidth=2)
            
            ax4.set_xlabel('Tiempo desde escalón (s)', fontweight='bold', fontsize=11)
            ax4.set_ylabel('Posición (µm)', fontweight='bold', fontsize=11)
            ax4.set_title(f'Respuesta al Escalón: Predicción vs Real\nPWM={step_pwm:.0f}', 
                         fontweight='bold', fontsize=12)
            ax4.legend(loc='best', fontsize=9, framealpha=0.9)
            ax4.grid(True, alpha=0.4)
            
            error_pred = abs(pos_real_mark - pos_pred_mark) if pos_real_mark != 0 else 0
            tf_text = f"""Función de Transferencia (FITTED):
G(s) = {K_fitted:.4f} / ({tau_fitted:.4f}s + 1)

Original: K={K:.4f}, τ={tau:.4f}
R² = {r_squared:.4f}

Escalón aplicado: PWM = {step_pwm:.0f}
Velocidad en régimen: {velocity_steady:.2f} µm/s

Error predicción @ t=1s:
{error_pred:.2f} µm"""
            
            ax4.text(0.02, 0.98, tf_text, transform=ax4.transAxes, fontsize=9,
                    verticalalignment='top', family='monospace',
                    bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
        else:
            ax4.text(0.5, 0.5, 'No se detectó escalón de PWM en los datos',
                    transform=ax4.transAxes, ha='center', va='center', fontsize=12)
            ax4.set_title('Respuesta al Escalón: No disponible', fontweight='bold')
        
        plt.tight_layout()
        
        return fig
    
    @classmethod
    def generate_calibration_analysis(cls, motor_a_csv=None, motor_b_csv=None):
        """
        Genera análisis completo de calibración para ambos motores.
        
        Args:
            motor_a_csv: Ruta al CSV de Motor A (None = auto-detectar)
            motor_b_csv: Ruta al CSV de Motor B (None = auto-detectar)
            
        Returns:
            dict: {'motor_a': Figure, 'motor_b': Figure, 'success': bool, 'message': str}
        """
        try:
            # Auto-detectar archivos si no se proporcionan
            if motor_a_csv is None or motor_b_csv is None:
                files = cls.find_calibration_files()
                if files is None:
                    return {
                        'success': False,
                        'message': 'No se encontraron archivos de calibración CSV.\n'
                                   'Asegúrate de tener MotorA_Sensor2*.csv y MotorB_Sensor1*.csv en el directorio del proyecto.'
                    }
                motor_a_csv = files['motor_a']
                motor_b_csv = files['motor_b']
            
            logger.info("="*70)
            logger.info("GENERANDO ANÁLISIS DE CALIBRACIÓN")
            logger.info("="*70)
            
            # Motor A - Cargar datos primero
            logger.info("\nMotor A - Sensor 2 (Eje X)")
            time_a, sensor_adc_a, pwm_a = cls.load_motor_data(motor_a_csv, "Motor A")
            sensor_um_a, intercept_a, slope_a = cls.calculate_calibration(sensor_adc_a, "Motor A")
            
            # NUEVO: Extraer K y τ directamente del CSV
            K_a, tau_a = cls.extract_tf_params_from_csv(time_a, sensor_um_a, pwm_a, "Motor A")
            
            # Validar parámetros
            if not cls.validate_tf_params(K_a, tau_a, 'A'):
                logger.warning("Motor A: Parámetros fuera de rango, usando valores por defecto")
                K_a, tau_a = 0.9958, 0.0350
            
            fig_a = cls.generate_analysis_figure(
                time_a, sensor_adc_a, sensor_um_a, pwm_a, 
                "Motor A (Eje X)", K_a, tau_a, intercept_a, slope_a
            )
            
            # Motor B - Cargar datos primero
            logger.info("\nMotor B - Sensor 1 (Eje Y)")
            time_b, sensor_adc_b, pwm_b = cls.load_motor_data(motor_b_csv, "Motor B")
            sensor_um_b, intercept_b, slope_b = cls.calculate_calibration(sensor_adc_b, "Motor B")
            
            # NUEVO: Extraer K y τ directamente del CSV
            K_b, tau_b = cls.extract_tf_params_from_csv(time_b, sensor_um_b, pwm_b, "Motor B")
            
            # Validar parámetros
            if not cls.validate_tf_params(K_b, tau_b, 'B'):
                logger.warning("Motor B: Parámetros fuera de rango, usando valores por defecto")
                K_b, tau_b = 3.3136, 0.0050
            
            fig_b = cls.generate_analysis_figure(
                time_b, sensor_adc_b, sensor_um_b, pwm_b,
                "Motor B (Eje Y)", K_b, tau_b, intercept_b, slope_b
            )
            
            logger.info("\n✅ Análisis de calibración completado")
            
            return {
                'success': True,
                'message': 'Análisis completado exitosamente',
                'motor_a': fig_a,
                'motor_b': fig_b
            }
            
        except FileNotFoundError as e:
            error_msg = f"Archivo no encontrado: {e}"
            logger.error(error_msg)
            return {'success': False, 'message': error_msg}
        except Exception as e:
            error_msg = f"Error en análisis: {str(e)}"
            logger.error(error_msg, exc_info=True)
            return {'success': False, 'message': error_msg}

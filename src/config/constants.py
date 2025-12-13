"""Constantes del sistema físico y configuración serial."""

# --- CONFIGURACIÓN ---
# Ajusta el puerto a tu configuración
SERIAL_PORT = 'COM5' 
BAUD_RATE = 1000000
# Buffer reducido para máxima velocidad (100 puntos = ~1 segundo @ 100Hz)
PLOT_LENGTH = 100

# --- Constantes del Sistema Físico ---
ADC_MAX = 1023.0
RECORRIDO_UM = 20000.0
FACTOR_ESCALA = RECORRIDO_UM / ADC_MAX  # Aprox. 24.4379 µm/unidad_ADC
# --------------------

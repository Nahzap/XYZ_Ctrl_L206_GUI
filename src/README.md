# 🎛️ Sistema de Control y Análisis - Plataforma Microscópica L206

![Version](https://img.shields.io/badge/version-2.2-blue.svg)
![Python](https://img.shields.io/badge/python-3.8+-green.svg)
![License](https://img.shields.io/badge/license-Open%20Source-orange.svg)
![Standards](https://img.shields.io/badge/standards-IEEE-red.svg)

**Sistema completo de control en tiempo real, adquisición de datos, análisis de función de transferencia y diseño de controladores H∞ para motores DC con driver L206.**

---

## 📋 Tabla de Contenidos

- [Descripción General](#-descripción-general)
- [Características Principales](#-características-principales)
- [Arquitectura del Sistema](#-arquitectura-del-sistema)
- [Requisitos del Sistema](#-requisitos-del-sistema)
- [Instalación](#-instalación)
- [Uso](#-uso)
- [Módulos y Componentes](#-módulos-y-componentes)
- [Optimizaciones de Rendimiento](#-optimizaciones-de-rendimiento)
- [Estructura del Código](#-estructura-del-código)
 - [Fundamentos Matemáticos de Control](#-fundamentos-matemáticos-de-control)
- [Contribuir](#-contribuir)
- [Licencia](#-licencia)

---

## 🎯 Descripción General

Este sistema proporciona una solución completa para el control y análisis de motores DC utilizando el driver L206. La aplicación integra:

- **Control en tiempo real** de motores DC duales (Motor A y Motor B)
- **Adquisición de datos** de sensores analógicos de alta velocidad
- **Visualización en tiempo real** con gráficos optimizados
- **Análisis de función de transferencia** utilizando métodos numéricos avanzados
- **Diseño de controladores H∞** (H-infinity) con síntesis robusta
- **Grabación de experimentos** en formato CSV para análisis posterior
- **Integración con cámara Thorlabs** para reconocimiento de imagen
- **Buffer optimizado con NumPy** para alto rendimiento

---

## ✨ Características Principales

### 🎮 Control de Motores
- **Modos de operación:**
  - Modo MANUAL: Control directo por teclado/interfaz
  - Modo AUTO: Control automático con valores programables
- **Control dual:** Manejo simultáneo de Motor A y Motor B
- **Potencia ajustable:** Rango -255 a +255 (PWM)
- **Comunicación serial:** Baudrate 115200 para baja latencia

### 📊 Visualización en Tiempo Real
- **Gráficos PyQtGraph optimizados:**
  - Potencia Motor A (Cian)
  - Potencia Motor B (Magenta)
  - Sensor 1 (Amarillo)
  - Sensor 2 (Verde)
- **Buffer circular NumPy:**
  - Reducción de 90% en uso de memoria
  - Sin copias innecesarias de datos
  - Rendering controlado por FPS (1-120 Hz)
- **Estadísticas en vivo:**
  - Uso de memoria del buffer
  - Conteo de datos y renders
  - Eficiencia de rendering
  - Frames saltados

### 🔬 Análisis de Sistema
- **Identificación de función de transferencia:**
  - Modelo experimental de segundo orden:
    
    ```math
    G(s) = \frac{K}{(\tau_1 s + 1)(\tau_2 s + 1)}
    ```
    
    donde \(\tau_1\) es el polo rápido (dinámica dominante en el rango de
    interés) y \(\tau_2\) es un polo lento (dinámica muy lenta que se
    desprecia en la síntesis H∞/H2 para evitar mal condicionamiento).
  - Modelo de diseño (dinámica rápida equivalente):
    
    ```math
    G_\text{fast}(s) = \frac{K}{\tau s + 1}
    ```
    
    Este es el modelo que usa `hinf_service.py` para la síntesis robusta.
  - Cálculo automático de ganancia K y constante de tiempo τ
  - Calibración con distancia real medida
- **Análisis de respuesta al escalón:**
  - Tiempo de establecimiento
  - Sobrepaso máximo
  - Error en estado estacionario
- **Visualización de resultados:**
  - Gráficos de respuesta temporal
  - Diagrama de Bode
  - Comparación modelo vs datos reales

### 🎛️ Diseño de Controladores H∞
- **Síntesis H-infinity:**
  - Control robusto con rechazo a perturbaciones
  - Funciones de ponderación personalizables
  - Análisis de norma H∞
- **Prueba de controladores:**
  - Control dual Motor A/Motor B
  - Secuencias de pasos programables
  - Simulación de control en lazo cerrado
- **Transferencia de controladores:**
  - Desde módulo de diseño a módulo de prueba
  - Almacenamiento de múltiples diseños

---

## 📐 Fundamentos Matemáticos de Control

Esta sección resume la formulación matemática que implementa el módulo
`core/services/hinf_service.py`, siguiendo el enfoque estándar de
"mixed-sensitivity" descrito en *Essentials of Robust Control* (Zhou,
Doyle, Glover).

### Modelo de planta

El sistema motor–sensor se modela inicialmente como una planta de
segundo orden identificada experimentalmente:

```math
G(s) = \frac{K}{(\tau_1 s + 1)(\tau_2 s + 1)}
```

Cuando existe separación fuerte de tiempos (\(\tau_2 \gg \tau_1\)), la
síntesis robusta se realiza sobre la **dinámica rápida equivalente**:

```math
G_\text{fast}(s) = \frac{K}{\tau s + 1}
```

En el código (`synthesize_hinf_controller`) se usa este modelo de primer
orden para evitar problemas numéricos en las ecuaciones de Riccati.

### Formulación H∞ de sensibilidad mixta

Se define el lazo abierto, la sensibilidad y la sensibilidad
complementaria:

```math
L(s) = G(s) K(s)
```

```math
S(s) = \frac{1}{1 + L(s)},
\qquad
T(s) = \frac{L(s)}{1 + L(s)}.
```

El problema H∞ que resuelve el software es el de **sensibilidad mixta**:

```math
\min_{K(s)} \; \gamma
\quad \text{sujeto a} \quad
\left\|\begin{bmatrix}
W_1(s) S(s) \\
W_2(s) K(s) S(s) \\
W_3(s) T(s)
\end{bmatrix}\right\|_\infty < \gamma,
```

donde \(\|\cdot\|_\infty\) es la norma H∞.

En `hinf_service.py` se construyen las ponderaciones con las formas
estándar (Zhou, Doyle, Glover):

- **Peso de performance** (error de seguimiento):
  
  ```math
  W_1(s) = \frac{\tfrac{1}{M_s} s + \omega_b}{s + \omega_b \, \varepsilon},
  ```
  
  donde \(M_s\) es el pico máximo de sensibilidad admitido,
  \(\omega_b\) el ancho de banda deseado y \(\varepsilon\) controla el
  error en régimen permanente.

- **Peso de esfuerzo de control**:
  
  ```math
  W_2(s) = \frac{k_u}{\tfrac{1}{\omega_{b_u}} s + 1},
  \qquad
  k_u = \frac{1}{U_\text{max}},\; \omega_{b_u} = \frac{\omega_b}{10}.
  ```

- **Peso de robustez** (sensibilidad complementaria):
  
  ```math
  W_3(s) = \frac{s + \omega_T \varepsilon_T}{\varepsilon_T s + \omega_T},
  ```
  
  donde \(\omega_T\) es la frecuencia asociada a la incertidumbre de
  modelo y \(\varepsilon_T\) gobierna el decaimiento en alta frecuencia.

Después de sintetizar el controlador, el código verifica
numéricamente:

```math
\|W_1 S\|_\infty,\; \|W_2 K S\|_\infty,\; \|W_3 T\|_\infty
```

y calcula \(\gamma_\text{verificado} = \max\{\|W_1 S\|_\infty,
\|W_2 K S\|_\infty, \|W_3 T\|_\infty\}\), que se muestra en la
interfaz junto con los márgenes clásicos de ganancia y fase.

### Formulación H2

Como alternativa, el sistema puede realizar síntesis H2 utilizando
`control.augw` y `control.h2syn`. El sistema aumentado \(P\) se
construye automáticamente con `augw(G, W1, W2, W3)` y se resuelve:

```math
K_\text{H2},\; \text{CL} = \operatorname{h2syn}(P, n_\text{meas}, n_\text{con}),
```

con una sola entrada medida (posición) y una sola señal de control.

### Controlador resultante

El controlador resultante se reduce típicamente a una estructura PI:

```math
K(s) = K_p + \frac{K_i}{s} = \frac{K_p s + K_i}{s},
```

cuyos parámetros \(K_p, K_i\) se extraen de la función de transferencia
resultante y se exportan tanto en forma continua como en código Arduino
discreto (sección `export_controller` de `hinf_service.py`).

Esta sección del README sirve como referencia teórica para defender el
procedimiento de diseño y análisis ante revisiones académicas o
ingenieriles.

### 📹 Grabación de Experimentos
- **Formato CSV estructurado:**
  ```
  tiempo,power_a,power_b,sensor_1,sensor_2
  ```
- **Timestamp preciso:** Resolución de milisegundos
- **Nomenclatura personalizable**
- **Exportación automática**

### 🎥 Integración con Cámara Thorlabs
- **Captura de video en tiempo real**
- **Control de parámetros:**
  - Exposición ajustable
  - Frame rate configurable
  - Tamaño de buffer
- **Vista flotante redimensionable**
- **Reconocimiento de imagen**

### 🔬 Autofoco Multi-Objeto con U2-Net (Planificado)
- **Detección de objetos salientes:**
  - U2-Net para segmentación sin calibración previa
  - Detección de múltiples objetos por frame
  - Filtrado por área mínima y probabilidad
- **Autofoco individual por objeto:**
  - Pre-detección antes de captura
  - Búsqueda de Z óptimo por cada objeto (Golden Section Search)
  - Score de enfoque basado en Varianza del Laplaciano (ROI)
- **Generación eficiente de BBDD:**
  - N imágenes por punto (una por objeto detectado)
  - Cada imagen enfocada en su objeto específico
  - Metadata JSON con coordenadas Z, scores y bounding boxes
- **Documentación:** Ver `docs/AUTOFOCUS_INTEGRATION_PLAN.md`

---

## 🏗️ Arquitectura del Sistema

```
┌─────────────────────────────────────────────────────────────┐
│                    ArduinoGUI (QMainWindow)                 │
│  ┌───────────┬───────────┬───────────┬───────────┬────────┐ │
│  │  Control  │ Grabación │ Análisis  │ H∞ Design │ Prueba │ │
│  └───────────┴───────────┴───────────┴───────────┴────────┘ │
└─────────────────────────────────────────────────────────────┘
          │                    │                    │
          ▼                    ▼                    ▼
┌──────────────────┐  ┌──────────────────┐  ┌─────────────────┐
│ SerialReaderThread│  │ OptimizedSignal │  │  CameraWorker  │
│                  │  │     Window      │  │                 │
│ • Lectura async  │  │ • Buffer NumPy  │  │ • Thread async  │
│ • Baudrate 115k  │  │ • FPS control   │  │ • Thorlabs SDK  │
│ • Signal emit    │  │ • Estadísticas  │  │ • Live preview  │
└──────────────────┘  └──────────────────┘  └─────────────────┘
          │                    │                    │
          ▼                    ▼                    ▼
┌─────────────────────────────────────────────────────────────┐
│                    Arduino + L206 Driver                    │
│  Motor A ←→ Sensor 1     │     Motor B ←→ Sensor 2         │
└─────────────────────────────────────────────────────────────┘
```

---

## 💻 Requisitos del Sistema

### Hardware
- **Microcontrolador:** Arduino compatible (Uno, Mega, etc.)
- **Driver de motores:** L206 dual H-bridge
- **Sensores:** 2x sensores analógicos (ADC 10-bit)
- **Puerto serial:** USB o UART
- **Cámara (opcional):** Thorlabs compatible con SDK

### Software
- **Sistema operativo:** Windows 10/11 (para Thorlabs SDK)
- **Python:** 3.8 o superior
- **Espacio en disco:** 100 MB mínimo

### Dependencias Python

```txt
# Core GUI
PyQt5>=5.15.0
pyqtgraph>=0.12.0

# Análisis y control
numpy>=1.20.0
pandas>=1.3.0
scipy>=1.7.0
control>=0.9.0

# Visualización
matplotlib>=3.4.0

# Comunicación serial
pyserial>=3.5

# Cámara Thorlabs (opcional)
pylablib>=1.4.0
```

---

## 🚀 Instalación

### 1. Clonar el repositorio

```bash
git clone https://github.com/tu-usuario/XYZ_Ctrl_L206_GUI.git
cd XYZ_Ctrl_L206_GUI
```

### 2. Crear entorno virtual (recomendado)

```bash
python -m venv venv
# Windows
venv\Scripts\activate
# Linux/Mac
source venv/bin/activate
```

### 3. Instalar dependencias

```bash
pip install -r requirements.txt
```

### 4. Configurar puerto serial

Editar `src/main.py`, línea 164:

```python
SERIAL_PORT = 'COM5'  # Cambiar al puerto de tu Arduino
BAUD_RATE = 115200
```

### 5. Ejecutar la aplicación

```bash
python src/main.py
```

---

## 📖 Uso

### Inicio Rápido

1. **Conectar Arduino** al puerto USB configurado
2. **Ejecutar** `python src/main.py`
3. **Abrir ventana de señales:** Botón "📊 Abrir Señales de Control"
4. **Configurar buffer optimizado:**
   - Ajustar tamaño de buffer (50-2000 muestras)
   - Configurar FPS de rendering (1-120 Hz)
   - Aplicar configuración

### Flujo de Trabajo Típico

#### 1️⃣ Grabación de Experimento

```
Pestaña "Grabación"
  ↓
Ingresar nombre de archivo (ej: experimento_escalon.csv)
  ↓
Clic "Iniciar Grabación"
  ↓
Enviar comando al motor (Modo AUTO)
  ↓
Esperar respuesta estabilizada
  ↓
Clic "Detener Grabación"
```

#### 2️⃣ Análisis de Función de Transferencia

```
Pestaña "Análisis"
  ↓
Seleccionar archivo CSV
  ↓
Configurar motor y sensor
  ↓
Definir rango de tiempo
  ↓
Ingresar distancias reales (min/max mm)
  ↓
Clic "Analizar Tramo"
  ↓
Obtener G(s) = K / (s·(τs + 1))
```

#### 3️⃣ Diseño de Controlador H∞

```
Pestaña "H∞ Synthesis"
  ↓
Ingresar función de transferencia G(s)
  ↓
Configurar funciones de ponderación
  ↓
Clic "Diseñar Controlador H∞"
  ↓
Analizar norma H∞ y estabilidad
  ↓
Transferir a pestaña "Prueba"
```

#### 4️⃣ Prueba de Controladores

```
Pestaña "Prueba"
  ↓
Verificar controladores transferidos (Motor A/B)
  ↓
Configurar secuencia de pasos
  ↓
Ejecutar simulación/control real
  ↓
Analizar resultados
```

#### 5️⃣ Microscopía con Autofoco Multi-Objeto (Planificado)

```
Pestaña "ImgRec" → Sección Microscopía
  ↓
Configurar trayectoria XY (desde TestTab)
  ↓
Habilitar "Autofoco Multi-Objeto"
  ↓
Configurar parámetros:
  - Rango Z: 100 µm
  - Tolerancia: 1 µm
  - Área mínima objeto: 100 px²
  ↓
Clic "Iniciar Microscopía"
  ↓
Para cada punto XY:
  1. Pre-detectar objetos con U2-Net
  2. Para cada objeto detectado:
     - Buscar Z óptimo (Golden Section)
     - Capturar imagen enfocada
  3. Generar: Clase_XXXXX_objYY.png
  ↓
Resultado: BBDD con N×M imágenes
(N puntos × M objetos/punto)
```

---

## 🧩 Módulos y Componentes

### Clases Principales

#### `OptimizedSignalBuffer`
Buffer circular optimizado con NumPy para almacenamiento eficiente de señales.

**Métodos:**
- `append_data(power_a, power_b, sensor_1, sensor_2)`: Agregar datos
- `get_signal_data(signal_name)`: Obtener señal específica
- `get_all_signals()`: Obtener todas las señales
- `clear()`: Limpiar buffer
- `get_memory_usage()`: Calcular uso de memoria

#### `OptimizedSignalWindow`
Ventana de visualización con control de frecuencia de rendering.

**Características:**
- Rendering a FPS configurable
- Estadísticas de rendimiento en tiempo real
- Control individual de visibilidad de señales
- Downsampling automático para alto rendimiento

#### `SerialReaderThread`
Thread asíncrono para lectura serial sin bloqueo de UI.

**Señales:**
- `data_received(str)`: Emite datos recibidos

#### `ArduinoGUI`
Interfaz principal con sistema de pestañas.

**Pestañas:**
1. **Control:** Modos MANUAL/AUTO, configuración de buffer
2. **Grabación:** Registro de experimentos
3. **Análisis:** Identificación de función de transferencia
4. **H∞ Synthesis:** Diseño de controladores robustos
5. **Prueba:** Validación de controladores
6. **ImgRec:** Integración con cámara Thorlabs

#### `CameraWorker`
Thread para manejo de cámara Thorlabs sin bloqueo.

**Funciones:**
- Conexión/desconexión automática
- Captura de frames en tiempo real
- Control de parámetros (exposición, FPS, buffer)

#### `SmartFocusScorer`
Evaluador de enfoque usando U2-Net para Salient Object Detection.

**Ubicación:** `src/img_analysis/smart_focus_scorer.py`

**Pipeline:**
1. Segmentación del objeto saliente usando U2-Net (deep learning)
2. Binarización de la máscara de probabilidad
3. Extracción de bounding box y centroide de **TODOS** los objetos
4. Cálculo de enfoque (Laplaciano) **por cada objeto individual**

**Métodos principales:**
- `assess_image(image)`: Evalúa imagen, retorna `FocusResult` con lista de objetos
- `_find_all_objects()`: Detecta todos los objetos válidos con sus scores
- `_calculate_masked_focus()`: Calcula enfoque solo en ROI del objeto

**Dataclasses:**
- `ObjectInfo`: Información de un objeto (bbox, centroid, area, focus_score)
- `FocusResult`: Resultado con status, score principal y lista de `objects`

#### `MultiObjectAutofocusController` (Planificado)
Controlador de autofoco multi-objeto para microscopía automatizada.

**Ubicación:** `src/core/autofocus/multi_object_autofocus.py`

**Flujo:**
1. `predetect_objects()`: Pre-detecta objetos usando SmartFocusScorer
2. `focus_single_object()`: Busca Z óptimo para un objeto específico
3. `capture_all_objects()`: Enfoca y captura cada objeto individualmente

---

## ⚡ Optimizaciones de Rendimiento

### Buffer Circular NumPy

**Antes (deque):**
```python
self.data = {
    'power_a': deque([0] * 200, maxlen=200),
    # ... conversión a list en cada render
}
```

**Después (NumPy):**
```python
self.data = np.zeros((4, 200), dtype=np.float32)
# Sin copias, acceso directo por vista
```

**Mejoras:**
| Métrica | Antes | Después | Ganancia |
|---------|-------|---------|----------|
| **Memoria** | ~2.24 MB/s | ~0.2 MB/s | **90% ↓** |
| **Copias de datos** | 4× por frame | 0 | **100% ↓** |
| **Latencia render** | Variable | Constante | **Estable** |

### Control de Frecuencia de Rendering

```python
# Evita saturación de CPU con datos de alta frecuencia
render_interval = 1.0 / render_fps  # 30 FPS por defecto
if current_time - last_render_time >= render_interval:
    self.render_plots()  # Renderizar solo cuando sea necesario
```

**Beneficios:**
- Uso de CPU reducido en 60-70%
- UI responsiva incluso a alta tasa de datos
- Estadísticas de eficiencia (renders vs datos)

### Downsampling Automático

```python
self.plot_widget.setDownsampling(mode='peak')
self.plot_widget.setClipToView(True)
```

Mejora el rendimiento con grandes datasets sin pérdida de información visual.

---

## 📁 Estructura del Código

```
XYZ_Ctrl_L206_GUI/
│
├── src/
│   ├── main.py                 # Aplicación principal
│   └── README.md              # Este archivo
│
├── logs/
│   └── motor_control_YYYYMMDD.log  # Logs IEEE format
│
├── data/
│   └── experimento_*.csv      # Archivos de grabación
│
├── requirements.txt           # Dependencias Python
└── README.md                  # Documentación principal
```

### Organización del Código en `main.py`

```python
# 1. Importaciones y configuración
# 2. Buffer optimizado (líneas 52-119)
# 3. Sistema de logging IEEE (líneas 145-160)
# 4. Constantes del sistema (líneas 164-172)
# 5. Tema oscuro personalizado (líneas 175-219)
# 6. Thread serial asíncrono (líneas 221-276)
# 7. Ventanas auxiliares (líneas 280-437)
# 8. Worker de cámara Thorlabs (líneas 441-827)
# 9. Interfaz principal ArduinoGUI (líneas 912+)
```

---

## 🔧 Configuración Avanzada

### Ajustar Parámetros del Buffer

```python
# En ArduinoGUI.__init__()
self.signal_buffer_size = 500  # 200-2000 muestras
self.signal_render_fps = 60    # 1-120 FPS
```

### Modificar Constantes Físicas

```python
# Calibración ADC → Distancia
ADC_MAX = 1023.0              # Resolución 10-bit
RECORRIDO_UM = 25000.0        # Recorrido en micrómetros
FACTOR_ESCALA = 24.4379       # μm/unidad_ADC
```

### Habilitar/Deshabilitar Módulos

```python
# Deshabilitar cámara Thorlabs
THORLABS_AVAILABLE = False

# Ajustar nivel de logging
logger.setLevel(logging.INFO)  # DEBUG, INFO, WARNING, ERROR
```

---

## 🐛 Solución de Problemas

### Error: Puerto serial no encontrado

```
ERROR: Puerto COM5 no encontrado
```

**Solución:**
1. Verificar conexión física del Arduino
2. Identificar puerto en Administrador de Dispositivos (Windows)
3. Actualizar `SERIAL_PORT` en `main.py`

### Error: Cámara Thorlabs no detectada

```
WARNING: pylablib no está instalado
```

**Solución:**
1. Instalar Thorlabs SDK desde: [thorlabs.com](https://www.thorlabs.com)
2. Verificar ruta del SDK en línea 128
3. `pip install pylablib`

### UI lenta o congelada

**Solución:**
1. Reducir FPS de rendering (10-30 Hz)
2. Disminuir tamaño de buffer (<500 muestras)
3. Cerrar ventanas auxiliares no utilizadas

---

## 📊 Formato de Datos

### Archivo CSV de Grabación

```csv
tiempo,power_a,power_b,sensor_1,sensor_2
0.000,0,0,512,487
0.023,100,-50,520,475
0.045,100,-50,535,460
...
```

**Columnas:**
- `tiempo`: Timestamp en segundos (float)
- `power_a`: Potencia Motor A [-255, 255]
- `power_b`: Potencia Motor B [-255, 255]
- `sensor_1`: Lectura ADC Sensor 1 [0, 1023]
- `sensor_2`: Lectura ADC Sensor 2 [0, 1023]

### Protocolo Serial Arduino → PC

```
Formato: POWER_A,POWER_B,SENSOR_1,SENSOR_2\n
Ejemplo: 100,-50,520,475\n
```

---

## 🤝 Contribuir

Las contribuciones son bienvenidas. Por favor:

1. Fork el repositorio
2. Crear rama para feature (`git checkout -b feature/nueva-funcionalidad`)
3. Commit cambios (`git commit -am 'Agregar nueva funcionalidad'`)
4. Push a la rama (`git push origin feature/nueva-funcionalidad`)
5. Crear Pull Request

### Estándares de Código

- **Estilo:** PEP 8
- **Documentación:** Docstrings en español
- **Logging:** Formato IEEE
- **Testing:** Incluir pruebas unitarias cuando aplique

---

## 📝 Notas de Versión

### v2.2 (Actual)
- ✅ Buffer circular optimizado con NumPy
- ✅ Control de frecuencia de rendering (FPS configurable)
- ✅ Estadísticas de rendimiento en tiempo real
- ✅ Panel de configuración dinámica de buffer
- ✅ Gestión de memoria mejorada (90% reducción)
- ✅ Integración con cámara Thorlabs
- ✅ Diseño de controladores H∞

### v2.1
- Análisis de función de transferencia
- Grabación de experimentos CSV
- Visualización en tiempo real con PyQtGraph

### v2.0
- Interfaz gráfica con PyQt5
- Sistema de pestañas
- Control dual de motores

---

## 📄 Licencia

Este proyecto es Open Source y se distribuye bajo una licencia permisiva.

```
Copyright (c) 2024 Sistema de Control L206
Se permite el uso, copia, modificación y distribución libre
```

---

## 👨‍💻 Autor

**Sistema de Control L206**

---

## 📚 Referencias

- **Python Control Systems Library:** [python-control.org](https://python-control.org)
- **PyQtGraph Documentation:** [pyqtgraph.org](http://www.pyqtgraph.org)
- **H-infinity Control Theory:** Zhou & Doyle (1998)
- **Thorlabs SDK Documentation**

---

## 🔗 Enlaces Útiles

- [Documentación NumPy](https://numpy.org/doc/)
- [PyQt5 Tutorial](https://www.riverbankcomputing.com/static/Docs/PyQt5/)
- [Arduino Serial Communication](https://www.arduino.cc/reference/en/language/functions/communication/serial/)
- [Control Systems Primer](https://python-control.readthedocs.io/)

---

<div align="center">

**⭐ Si este proyecto te fue útil, considera darle una estrella en GitHub ⭐**

</div>

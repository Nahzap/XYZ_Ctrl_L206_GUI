# Auditoría de Conexión Arduino - XYZ_Ctrl_L206_GUI

**Fecha:** 2025-12-16  
**Versión del Sistema:** 2.5  
**Estado:** � CORRECCIONES IMPLEMENTADAS

---

## 1. Resumen Ejecutivo

El sistema no puede conectarse al Arduino porque **el puerto COM5 no existe** en el sistema. El log muestra múltiples intentos de conexión fallidos con el error:

```
FileNotFoundError(2, 'El sistema no puede encontrar el archivo especificado.', None, 2)
```

### Problemas Identificados

| # | Problema | Severidad | Estado |
|---|----------|-----------|--------|
| 1 | Puerto COM5 hardcodeado no existe | 🔴 Crítico | ✅ CORREGIDO |
| 2 | No hay detección automática de puertos disponibles | 🟡 Medio | ✅ CORREGIDO |
| 3 | Baudrate por defecto inconsistente (115200 vs 1000000) | 🟡 Medio | ✅ CORREGIDO |
| 4 | No hay feedback visual claro cuando falla la conexión inicial | 🟢 Bajo | ✅ CORREGIDO |

---

## 2. Análisis del Flujo de Conexión

### 2.1 Arquitectura Actual

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  constants.py   │────▶│    main.py       │────▶│ SerialHandler   │
│  SERIAL_PORT    │     │  ArduinoGUI()    │     │   (QThread)     │
│  = 'COM5'       │     │                  │     │                 │
│  BAUD_RATE      │     │ serial_thread =  │     │ self.ser =      │
│  = 1000000      │     │ SerialHandler()  │     │ serial.Serial() │
└─────────────────┘     └──────────────────┘     └─────────────────┘
                               │
                               ▼
                        ┌──────────────────┐
                        │   ControlTab     │
                        │ (UI de conexión) │
                        │                  │
                        │ port_combo       │
                        │ baudrate_combo   │
                        └──────────────────┘
```

### 2.2 Secuencia de Inicialización

1. `main.py:177` - Crea `SerialHandler(SERIAL_PORT, BAUD_RATE)` con valores de `constants.py`
2. `main.py:217` - Pasa `serial_handler` a `ControlTab`
3. `main.py:346` - Llama `serial_thread.start()` para iniciar conexión
4. `serial_handler.py:48` - Intenta abrir `serial.Serial(port=COM5, ...)`
5. **FALLA** - COM5 no existe → `SerialException`

### 2.3 Flujo de Reconexión (Manual)

1. Usuario selecciona puerto en `ControlTab.port_combo`
2. Usuario hace clic en "🔌 Conectar / Reconectar"
3. `ControlTab._request_reconnect()` emite señal con puerto/baudrate
4. `main.py._on_serial_reconnect()` crea nuevo `SerialHandler`
5. Nuevo thread intenta conectar

---

## 3. Archivos Involucrados

### 3.1 `src/config/constants.py`
```python
SERIAL_PORT = 'COM5'   # ⚠️ HARDCODEADO - puede no existir
BAUD_RATE = 1000000    # ✅ Correcto para Arduino Due/Teensy
```

### 3.2 `src/models/system_config.py`
```python
serial_port: str = 'COM5'      # ⚠️ Duplicado
baud_rate: int = 115200        # ⚠️ INCONSISTENTE con constants.py
```

### 3.3 `src/gui/tabs/control_tab.py`
```python
# Línea 85-86: Lista estática de puertos
self.port_combo.addItems(['COM1', 'COM2', ..., 'COM10'])
self.port_combo.setCurrentText('COM5')  # ⚠️ Por defecto

# Línea 93-94: Baudrate por defecto
self.baudrate_combo.setCurrentText('115200')  # ⚠️ INCONSISTENTE
```

### 3.4 `src/core/communication/serial_handler.py`
- ✅ Maneja errores correctamente
- ✅ Emite señal `data_received` con mensajes de error
- ⚠️ No valida si el puerto existe antes de intentar abrir

---

## 4. Problemas Detallados

### 4.1 🔴 Puerto COM5 No Existe

**Causa raíz:** El Arduino no está conectado o está en otro puerto.

**Evidencia del log:**
```
2025-12-16 11:19:23 | ERROR | serial_handler | run:93 | 
Error al abrir puerto COM5: could not open port 'COM5': 
FileNotFoundError(2, 'El sistema no puede encontrar el archivo especificado.', None, 2)
```

**Solución:** Implementar detección automática de puertos disponibles.

### 4.2 🟡 No Hay Detección Automática de Puertos

**Estado actual:** `ControlTab` muestra lista estática COM1-COM10.

**Problema:** El usuario debe adivinar qué puerto usar.

**Solución:** Usar `serial.tools.list_ports` para detectar puertos disponibles.

### 4.3 🟡 Baudrate Inconsistente

| Archivo | Valor |
|---------|-------|
| `constants.py` | 1000000 |
| `system_config.py` | 115200 |
| `control_tab.py` (UI default) | 115200 |

**Problema:** La UI muestra 115200 pero el sistema usa 1000000.

**Solución:** Sincronizar todos los valores a 1000000 (o el que use el Arduino).

### 4.4 🟢 Feedback Visual Insuficiente

**Estado actual:** El usuario ve "❌ Desconectado" pero no sabe por qué.

**Solución:** Mostrar mensaje específico del error (ej: "Puerto COM5 no encontrado").

---

## 5. Plan de Corrección

### Fase 1: Detección Automática de Puertos (Crítico)

1. **Modificar `control_tab.py`:**
   - Agregar método `_scan_ports()` usando `serial.tools.list_ports`
   - Llamar al inicializar y agregar botón "🔄 Escanear"
   - Mostrar descripción del dispositivo (ej: "COM3 - Arduino Mega")

2. **Modificar `constants.py`:**
   - Cambiar `SERIAL_PORT = None` (auto-detectar)
   - O usar función `get_default_port()` que detecte Arduino

### Fase 2: Sincronizar Baudrate

1. **Unificar en `constants.py`:**
   ```python
   BAUD_RATE = 1000000  # Mantener como fuente única
   ```

2. **Actualizar `control_tab.py`:**
   ```python
   self.baudrate_combo.setCurrentText('1000000')
   ```

3. **Eliminar duplicado en `system_config.py`** o sincronizar.

### Fase 3: Mejorar Feedback de Errores

1. **Modificar `control_tab.py`:**
   - Agregar `set_connection_error(message)` para mostrar error específico

2. **Modificar `main.py`:**
   - Pasar mensaje de error desde `update_data()` a `ControlTab`

---

## 6. Código de Corrección Propuesto

### 6.1 Detección de Puertos en `control_tab.py`

```python
import serial.tools.list_ports

def _scan_ports(self):
    """Escanea puertos seriales disponibles."""
    self.port_combo.clear()
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        self.port_combo.addItem("No hay puertos disponibles")
        return
    
    for port in ports:
        # Mostrar puerto con descripción
        display = f"{port.device} - {port.description}"
        self.port_combo.addItem(display, port.device)
    
    # Seleccionar primer Arduino encontrado
    for i, port in enumerate(ports):
        if 'Arduino' in port.description or 'CH340' in port.description:
            self.port_combo.setCurrentIndex(i)
            break
```

### 6.2 Sincronizar Baudrate en UI

```python
# En _create_serial_config_group():
from config.constants import BAUD_RATE

self.baudrate_combo.setCurrentText(str(BAUD_RATE))
```

---

## 7. Verificación Post-Corrección

### Checklist

- [ ] Conectar Arduino físicamente
- [ ] Verificar puerto en Administrador de Dispositivos
- [ ] Ejecutar aplicación
- [ ] Verificar que el puerto correcto aparece en el combo
- [ ] Verificar que baudrate es 1000000
- [ ] Hacer clic en "Conectar"
- [ ] Verificar estado "✅ Conectado (COMx)"
- [ ] Verificar datos de sensores actualizándose

### Comando de Diagnóstico

```python
# Ejecutar en Python para ver puertos disponibles:
import serial.tools.list_ports
for p in serial.tools.list_ports.comports():
    print(f"{p.device}: {p.description}")
```

---

## 8. Conclusión

El problema principal es que **el Arduino no está conectado** o está en un puerto diferente a COM5. Las correcciones propuestas:

1. ✅ Agregan detección automática de puertos
2. ✅ Sincronizan baudrate en toda la aplicación
3. ✅ Mejoran feedback de errores al usuario

**Prioridad:** Implementar Fase 1 (detección de puertos) inmediatamente.

---

## 9. Correcciones Implementadas (2025-12-16)

### 9.1 Archivos Modificados

| Archivo | Cambios |
|---------|---------|
| `src/gui/tabs/control_tab.py` | Detección automática de puertos, botón escanear, baudrate sincronizado |
| `src/main.py` | Método `_detect_arduino_port()` para auto-detectar Arduino al inicio |
| `src/models/system_config.py` | Baudrate corregido a 1000000, límite de validación aumentado |

### 9.2 Nuevas Funcionalidades

1. **Detección Automática de Puertos:**
   - `ControlTab._scan_ports()` escanea puertos disponibles
   - Detecta Arduino por descripción (Arduino, CH340, CH341, FTDI, USB Serial)
   - Botón 🔄 para re-escanear manualmente

2. **Auto-detección al Inicio:**
   - `ArduinoGUI._detect_arduino_port()` detecta Arduino antes de crear SerialHandler
   - Si no encuentra Arduino, usa el primer puerto disponible
   - Fallback a COM5 si no hay puertos

3. **Baudrate Sincronizado:**
   - Todos los archivos usan 1000000 bps
   - UI muestra el valor correcto por defecto

### 9.3 Cómo Probar

1. **Conectar Arduino físicamente**
2. **Ejecutar la aplicación:**
   ```bash
   cd src
   python main.py
   ```
3. **Verificar en el log:**
   - "Arduino detectado automáticamente en: COMx"
   - "INFO: Conectado exitosamente."
4. **Verificar en la UI:**
   - Combo de puertos muestra puertos disponibles con descripción
   - Estado: "✅ Conectado (COMx)"
   - Datos de sensores actualizándose

### 9.4 Si el Arduino NO está conectado

El sistema ahora:
- Muestra "No hay puertos disponibles" en el combo
- Log indica "No se encontraron puertos seriales disponibles"
- Usuario puede conectar Arduino y hacer clic en 🔄 para escanear

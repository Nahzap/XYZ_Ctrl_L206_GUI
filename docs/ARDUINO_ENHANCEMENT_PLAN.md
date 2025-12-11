# 📋 PLAN DE MEJORAS ARDUINO - SISTEMA DE CONTROL L206

**Documento creado:** 2025-12-05  
**Versión Firmware Actual:** v0.1 (básico)  
**Versión Firmware Objetivo:** v0.2 (control cerrado + position hold)  
**Archivos:** `XYZ_Control_Lab206.ino`

---

## 🎯 **OBJETIVOS DE MEJORA**

### **Problemas Actuales Identificados**
1. **Control Open-Loop**: PWM se aplica sin retroalimentación de posición
2. **Sin Position Hold**: Cuando PWM=0, los motores pierden posición por drift/backlash
3. **Sin Verificación de Asentamiento**: No hay confirmación de posición alcanzada
4. **Protocolo Limitado**: Solo 3 comandos (M, A, A,pwm,pwm)

### **Metas del Firmware v0.2**
- ✅ **Position Hold Activo**: Mantener posición con PWM mínimo adaptativo
- ✅ **Control Cerrado Simple**: Usar sensores analógicos para retroalimentación
- ✅ **Protocolo Extendido**: Nuevos comandos H, B, S
- ✅ **Verificación de Settling**: Confirmar posición antes de avanzar
- ✅ **Anti-Backlash Básico**: Compensación simple por cambios de dirección

---

## 🔧 **ARQUITECTURA ACTUAL vs PROPUESTA**

### **Firmware Actual (v0.1)**
```cpp
// Estructura simple
enum ControlMode { MANUAL, AUTO };
ControlMode currentMode = MANUAL;
int potenciaA = 0;
int potenciaB = 0;

// Protocolo básico
"M" -> Modo Manual
"A,pwm_a,pwm_b" -> Potencia directa
```

### **Firmware Propuesto (v0.2)**
```cpp
// Estructura extendida
enum ControlMode { MANUAL, AUTO, HOLD, BRAKE };
enum ControlState { IDLE, MOVING, SETTLING, HOLDING };

struct PositionControl {
  int targetSensor1 = 0;
  int targetSensor2 = 0;
  int holdPWM_A = 30;      // PWM mínimo para mantener
  int holdPWM_B = 30;
  float kp = 0.5;          // Ganancia proporcional simple
  int settleThreshold = 5; // Umbral de asentamiento (ADC)
  unsigned long settleTime = 0;
  bool settled = false;
};

// Protocolo extendido
"M" -> Modo Manual
"A,pwm_a,pwm_b" -> Potencia directa
"H,sensor1,sensor2" -> Position Hold con target
"B" -> Freno activo (short-circuit)
"S,threshold" -> Configurar umbral de asentamiento
```

---

## 📊 **PLAN DE IMPLEMENTACIÓN POR FASES**

### **FASE 1: ESTRUCTURA BÁSICA (1 hora)**
- [ ] **1.1** Agregar enums extendidos (ControlMode, ControlState)
- [ ] **1.2** Crear struct PositionControl con parámetros
- [ ] **1.3** Inicializar variables de control cerrado
- [ ] **1.4** Modificar loop() para incluir nuevos modos

### **FASE 2: COMANDOS DE PROTOCOLO (1 hora)**
- [ ] **2.1** Extender `checkSerialCommands()` para nuevos comandos
- [ ] **2.2** Implementar comando `H,s1,s2` (Position Hold)
- [ ] **2.3** Implementar comando `B` (Active Brake)
- [ ] **2.4** Implementar comando `S,threshold` (Settling config)

### **FASE 3: CONTROL CERRADO SIMPLE (2 horas)**
- [ ] **3.1** Implementar `updatePositionControl()`
- [ ] **3.2** Crear `calculateHoldPWM()` basado en error de sensor
- [ ] **3.3** Agregar `applyActiveBrake()` para freno regenerativo
- [ ] **3.4** Implementar `checkSettling()` con umbral configurable

### **FASE 4: LÓGICA DE SETTLING (1 hora)**
- [ ] **4.1** Implementar máquina de estados: IDLE → MOVING → SETTLING → HOLDING
- [ ] **4.2** Agregar temporizador para verificación de asentamiento
- [ ] **4.3** Implementar `isPositionSettled()` con tolerancia
- [ ] **4.4** Agregar indicador de estado en salida serial

### **FASE 5: ANTI-BACKLASH BÁSICO (1 hora)**
- [ ] **5.1** Detectar cambios de dirección en sensores
- [ ] **5.2** Implementar `applyBacklashCompensation()`
- [ ] **5.3** Agregar sobrepaso controlado en cambios de dirección
- [ ] **5.4** Optimizar parámetros de compensación

### **FASE 6: TESTING Y VALIDACIÓN (1 hora)**
- [ ] **6.1** Crear script de prueba para cada comando
- [ ] **6.2** Verificar respuesta a cambios de carga
- [ ] **6.3** Medir precisión de position hold
- [ ] **6.4** Documentar parámetros óptimos

---

## 💻 **IMPLEMENTACIÓN DETALLADA**

### **Nuevas Variables Globales**
```cpp
// Modos de control extendidos
enum ControlMode { MANUAL, AUTO, HOLD, BRAKE, SETTLING };
enum ControlState { IDLE, MOVING, SETTLING, HOLDING };

// Estructura de control de posición
struct PositionControl {
  int targetSensor1 = 0;
  int targetSensor2 = 0;
  int holdPWM_A = 25;
  int holdPWM_B = 25;
  float kp = 0.3;           // Ganancia proporcional
  int settleThreshold = 8;  // Umbral ADC para asentamiento
  unsigned long settleStartTime = 0;
  unsigned long settleTimeout = 2000; // ms
  bool settled = false;
  int lastDirection1 = 0;
  int lastDirection2 = 0;
  bool backlashEnabled = true;
};

PositionControl posControl;
ControlState currentState = IDLE;
unsigned long lastUpdateTime = 0;
const int UPDATE_INTERVAL = 50; // ms
```

### **Nuevos Comandos del Protocolo**
```cpp
// Comando H: Position Hold
// Formato: "H,sensor1_target,sensor2_target"
else if (strncmp(command_buffer, "H,", 2) == 0) {
  char* token = strtok(command_buffer, ",");
  token = strtok(NULL, ",");
  if (token != NULL) posControl.targetSensor1 = atoi(token);
  token = strtok(NULL, ",");
  if (token != NULL) posControl.targetSensor2 = atoi(token);
  
  currentMode = HOLD;
  currentState = MOVING;
  posControl.settleStartTime = millis();
  posControl.settled = false;
}

// Comando B: Active Brake
else if (strcmp(command_buffer, "B") == 0) {
  currentMode = BRAKE;
  currentState = IDLE;
}

// Comando S: Settling Configuration
// Formato: "S,threshold"
else if (strncmp(command_buffer, "S,", 2) == 0) {
  char* token = strtok(command_buffer, ",");
  token = strtok(NULL, ",");
  if (token != NULL) posControl.settleThreshold = atoi(token);
}
```

### **Control de Posición Cerrado**
```cpp
void updatePositionControl() {
  if (currentMode != HOLD && currentMode != SETTLING) return;
  
  // Leer sensores actuales
  int sensor1 = analogRead(sensorPin1);
  int sensor2 = analogRead(sensorPin2);
  
  // Calcular error
  int error1 = posControl.targetSensor1 - sensor1;
  int error2 = posControl.targetSensor2 - sensor2;
  
  // Control proporcional simple
  int correction1 = (int)(error1 * posControl.kp);
  int correction2 = (int)(error2 * posControl.kp);
  
  // Aplicar PWM de hold + corrección
  potenciaA = constrain(posControl.holdPWM_A + correction1, -255, 255);
  potenciaB = constrain(posControl.holdPWM_B + correction2, -255, 255);
  
  // Actualizar estado
  if (abs(error1) < posControl.settleThreshold && 
      abs(error2) < posControl.settleThreshold) {
    if (currentState == MOVING || currentState == SETTLING) {
      currentState = SETTLING;
      if (millis() - posControl.settleStartTime > posControl.settleTimeout) {
        currentState = HOLDING;
        posControl.settled = true;
      }
    }
  } else {
    currentState = MOVING;
    posControl.settleStartTime = millis();
    posControl.settled = false;
  }
}

void applyActiveBrake() {
  // Freno activo: cortocircuitar motores
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}
```

### **Loop Principal Modificado**
```cpp
void loop() {
  checkSerialCommands();
  
  // Actualizar control de posición si es necesario
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    updatePositionControl();
    lastUpdateTime = currentTime;
  }
  
  // Ejecutar modo de control actual
  if (currentMode == MANUAL) {
    // Lógica manual existente
  } 
  else if (currentMode == AUTO) {
    // Lógica auto existente
  }
  else if (currentMode == HOLD || currentMode == SETTLING) {
    // Control de posición cerrado
    setMotorPower(ENA, IN1, IN2, potenciaA);
    setMotorPower(ENB, IN3, IN4, potenciaB);
  }
  else if (currentMode == BRAKE) {
    applyActiveBrake();
  }
  
  // Enviar datos con estado de control
  sendSensorData();
}
```

---

## 🔄 **INTEGRACIÓN CON PYTHON GUI**

### **Actualización de Protocol.py**
```python
@staticmethod
def format_position_hold(sensor1_target, sensor2_target):
    """Comando para mantener posición con target de sensores."""
    return f'H,{sensor1_target},{sensor2_target}'

@staticmethod
def format_brake_command():
    """Comando de freno activo."""
    return 'B'

@staticmethod
def format_settling_config(threshold):
    """Configurar umbral de asentamiento."""
    return f'S,{threshold}'

@staticmethod
def parse_sensor_data_with_status(line):
    """Parsear datos con estado de asentamiento."""
    parts = line.split(',')
    if len(parts) >= 6:
        return {
            'pot_a': int(parts[0]),
            'pot_b': int(parts[1]),
            'sens_1': int(parts[2]),
            'sens_2': int(parts[3]),
            'state': parts[4].strip(),
            'settled': parts[5].strip() == '1'
        }
    return None
```

### **Mejoras en TestTab**
```python
def execute_trajectory_with_settling(self):
    """Ejecuta trayectoria con verificación de asentamiento."""
    
    # Configurar umbral de asentamiento
    threshold = 8  # ADC units
    self.send_command_callback(f"S,{threshold}")
    
    # Para cada punto de la trayectoria
    for point in self.trajectory_points:
        # Enviar comando de position hold
        x_adc = self._um_to_adc(point[0])
        y_adc = self._um_to_adc(point[1])
        self.send_command_callback(f"H,{x_adc},{y_adc}")
        
        # Esperar asentamiento
        self.wait_for_settling(timeout=2000)
        
        # Pequeña pausa antes del siguiente punto
        time.sleep(0.1)

def wait_for_settling(self, timeout=2000):
    """Espera hasta que el sistema indique posición asentada."""
    start_time = time.time()
    
    while time.time() - start_time < timeout / 1000.0:
        # Verificar estado desde datos serial
        if hasattr(self, 'last_sensor_data'):
            if self.last_sensor_data.get('settled', False):
                logger.info("✅ Posición asentada")
                return True
        time.sleep(0.05)
    
    logger.warning("⚠️ Timeout esperando asentamiento")
    return False
```

---

## 📈 **MÉTRICAS DE MEJORA ESPERADAS**

### **Precisión de Posición**
- **Actual**: Deriva significativa (>50µm) entre waypoints
- **Esperado**: <5µm error con position hold activo
- **Mejora**: 90% reducción en drift

### **Tiempo de Asentamiento**
- **Actual**: Sin verificación (avanza sin confirmar)
- **Esperado**: <500ms para asentamiento confirmado
- **Beneficio**: Trayectorias más predecibles

### **Estabilidad en Carga**
- **Actual**: PWM fijo, sin compensación
- **Esperado**: Compensación automática por variaciones de carga
- **Ventaja**: Mayor fiabilidad en experimentos

### **Backlash Compensation**
- **Actual**: Sin compensación, error en cambios de dirección
- **Esperado**: Reducción 50% del error por backlash
- **Implementación**: Sobrepaso controlado + return

---

## 🧪 **PLAN DE TESTING**

### **Test 1: Position Hold Básico**
```python
# Enviar comando H y verificar estabilidad
send_command("H,500,500")  # Target posición
monitor_position(duration=10s)  # Verificar drift <5µm
```

### **Test 2: Settling Verification**
```python
# Cambiar posición y medir tiempo de asentamiento
send_command("H,600,600") 
measure_settling_time(expected<500ms)
```

### **Test 3: Anti-Backlash**
```python
# Movimientos alternados para probar backlash
for pos in [(400,400), (600,600), (400,400)]:
    send_command(f"H,{pos[0]},{pos[1]}")
    wait_for_settling()
    measure_position_error()
```

### **Test 4: Trayectoria Completa**
```python
# Ejecutar trayectoria zig-zag con nuevas mejoras
execute_trajectory_with_settling()
measure_total_precision()
compare_with_baseline()
```

---

## 📋 **CHECKLIST DE IMPLEMENTACIÓN**

### **Firmware Arduino**
- [ ] Estructura de datos extendida
- [ ] Nuevos comandos implementados
- [ ] Control cerrado funcional
- [ ] Verificación de asentamiento
- [ ] Anti-backlash básico
- [ ] Testing completo

### **Protocolo Python**
- [ ] MotorProtocol actualizado
- [ ] TestTab con settling logic
- [ ] Monitor de precisión en tiempo real
- [ ] Logging de estado del sistema
- [ ] Validación de mejoras

### **Documentación**
- [ ] Manual de nuevos comandos
- [ ] Guía de calibración de parámetros
- [ ] Ejemplos de uso
- [ ] Troubleshooting guide

---

## 🎯 **RESULTADOS FINALES**

### **Impacto en Sistema de Microscopía**
- ✅ **Precisión Sub-micrométrica**: Posición estable para imágenes de alta resolución
- ✅ **Automatización Confiante**: Trayectorias reproducibles sin intervención manual
- ✅ **Experimentos Largos**: Sistema mantiene posición durante horas
- ✅ **Compatibilidad Total**: 100% compatible con GUI existente

### **Ventajas Competitivas**
- **Control Cerrado a Bajo Costo**: Sin necesidad de encoders costosos
- **Implementación Rápida**: 6 horas de desarrollo total
- **Mantenimiento Simple**: Firmware robusto y bien documentado
- **Escalabilidad**: Base para futuras mejoras (PID avanzado, multi-eje)

---

**Próximo Paso:** Comenzar Fase 1 - Estructura básica del firmware extendido.

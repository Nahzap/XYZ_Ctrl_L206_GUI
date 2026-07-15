# Auditoría de compatibilidad: estrategias vs código actual

| Campo | Valor |
|-------|-------|
| **Fecha** | 2026-07-14 00:00 (UTC-4) |
| **Alcance** | Verificar si el código vigente es compatible con (1) lazo máquina-rápido + UI 30 Hz, (2) control micrométrico / LUT / mini-pulso en STM32F767ZI, (3) uso de memorias ITCM/DTCM/AXI/Flash |
| **Firmware** | `MycoViT_XY_Controller` |
| **Host** | `XYZ_Ctrl_L206_GUI` |
| **Método** | Inspección de fuentes (linker, `app_*`, timers GUI, serial, FOV) — sin cambios de código |
| **Planes de referencia** | `plan_arquitectura_control_rapido_ui30hz_2026-07-13.md`, `plan_precision_fov_um_2026-07-13.md` |

---

## 0. Veredicto ejecutivo

| Pregunta | Respuesta |
|----------|-----------|
| ¿El F767 + firmware actual *pueden* soportar la estrategia? | **Sí, como plataforma** (CPU, 1 MHz RT, 1 kHz motores, Flash/RAM sobran). |
| ¿El código *ya implementa* la estrategia? | **No.** |
| ¿Hay blockers estructurales? | **Sí, en host y en contrato de mando** — no en “falta de MHz”. |
| ¿Hay piezas reutilizables? | **Sí:** backbone RT, hook vacío, `SensorBuffer`, H∞ en µm, FOV pulse FSM (aunque con dwell en ticks). |

**Clasificación usada**

- **AS-IS** — el código ya cumple.
- **EXTENSIÓN** — compatible si se añade/cambia módulo sin tirar la base.
- **BLOCKER** — comportamiento actual contradice la estrategia; hay que rediseñar ese eslabón.

---

## 1. Matriz global (claim → código)

| # | Claim de estrategia | Firmware | GUI | Nota |
|---|---------------------|----------|-----|------|
| 1 | Tres planos; UI ≠ reloj de control | EXTENSIÓN (planos MCU sí; UI N/A) | **BLOCKER** | Control = `QTimer(10)` en hilo Qt |
| 2 | UI ~30 Hz solo snapshots | N/A | **BLOCKER** | Telemetría → `update_data` sin throttle 30 Hz |
| 3 | Worker máquina drena serial + control | EXTENSIÓN (RX ISR existe) | **BLOCKER** | Solo `SerialHandler` RX; control en timers GUI |
| 4 | Mini-pulso / timing fino en MCU | **BLOCKER** as-is | EXTENSIÓN (FOV intenta pulsos) | MCU **mantiene** PWM de `A,a,b` |
| 5 | LUT de átomos en memoria MCU | **BLOCKER** as-is | N/A | No hay LUT ni regiones ITCM/DTCM en linker |
| 6 | H∞ / error en µm | N/A (MCU no cierra posición) | **AS-IS** (ley) | Host: `error_um`; dual legacy aún usa ADC en parte |
| 7 | SysTick 1 kHz + RT 1 MHz + hook | **AS-IS** | N/A | Base lista para EXTENSIÓN |
| 8 | PC no comanda grueso en fine | **BLOCKER** (API solo PWM continuo) | **BLOCKER** approach / H∞ live | FOV parcial: pulso pero ~300 ms |

---

## 2. Firmware — hechos verificados

### 2.1 Memorias (linker)

`STM32F767xx_FLASH.ld` define únicamente:

```text
RAM   0x20000000  512K
FLASH 0x08000000  2048K
```

| Recurso silicio F767ZI | Uso en estrategia | Uso en código |
|------------------------|-------------------|---------------|
| Flash 2 MB | LUT const, calibración | Solo código/rodata genérico; **sin LUT** |
| ITCM 16 KB | ISR / hook hot path | **No particionado** |
| DTCM 128 KB | estado RT, EMA, staging CCR | Todo `.data/.bss` en RAM plana desde `0x20000000` |
| AXI SRAM | buffers grandes / DMA | ADC RT **sin DMA**; sin región AXI explícita |
| Backup 4 KB | opcional cal persistente | No usado en app |

**Compatibilidad:** el silicio **sí**; el mapa de linkage **no**. Estrategia de “LUT en Flash + estado en DTCM + hot code ITCM” = **EXTENSIÓN de linker + colocación**, no de HW.

### 2.2 Planos temporales en MCU — AS-IS parcial

| Plano | Implementación | Tasa |
|-------|----------------|------|
| Supervisión | `main`: serial process + `printf` CSV | streaming (`TELEMETRY_INTERVAL_MS=0`) |
| Actuación | `SysTick_Handler` → `app_control_tick_1ms()` | **1 kHz** |
| RT | TIM6 → ADC1/2 → `app_rt_isr` → `app_rt_control_hook` | **~1 MHz** |

Hook: **weak vacío** — punto oficial de extensión para C(z)/LUT/fine.

### 2.3 Contrato AUTO — BLOCKER para “PC no PWM grueso”

Comportamiento verificado en `app_serial.c` / `app_control.c`:

1. `A,<a>,<b>` → `app_control_set_auto` guarda potencias.
2. Cada 1 ms se **reaplican** las mismas potencias (`CTRL_AUTO`).
3. No hay duración, no hay one-shot, no hay setpoint en µm.

Es decir: el MCU **ayuda** a no reenviar el mismo `A` sin cambio, pero **no sustituye** el lazo de posición. Mientras el PC mande PWM distinto cada tick, el modelo sigue siendo **potencia continua comandada por el host**.

Parser: buffer 50 B; si `s_cmd_ready==1`, **líneas nuevas se descartan** (no cola). Proceso de comando en `main`, detrás de TX bloqueante → latencia de mando no es RT.

### 2.4 Actuador — límites digitales

| Elemento | Hecho |
|----------|-------|
| API potencia | Entero **−255…255** |
| PWM | TIM1/TIM8 ARR=4319 → CCR = \|p\|·4319/255 → **256 niveles** expuestos (no todo el ARR) |
| Portadora | ~50 kHz continuo |
| Mini-pulso temporal | **Ausente** |

Una LUT que indexe a `{pwm, t_on}` **cabe** en Flash/DTCM, pero **hoy no existe** el ejecutor de `t_on`.

### 2.5 ADC RT y caché — favorable a EXTENSIÓN

- I-cache / D-cache ON.
- `app_rt` fuerza ADC **sin DMA** y lee `DR` en ISR → evita coherencia D-cache en el path crítico.
- Conviene mantener estado RT en DTCM si se añaden buffers/LUT calientes.

---

## 3. GUI — hechos verificados

### 3.1 Reloj de control — BLOCKER

| Temporizador | Periodo | Función |
|--------------|---------|---------|
| `_dual_timer` | `start(10)` → **100 Hz** | Dual → `A,pwm_a,pwm_b` |
| `_trajectory_timer` | `start(10)` → **100 Hz** | Trayectoria / `StepController.tick()` |
| `control_timer` (H∞) | `start(10)` → **100 Hz** | H∞ live → `A,...` |

Coincide exactamente con el anti-patrón del plan de arquitectura §2.

### 3.2 Plano UI 30 Hz — BLOCKER (ausente)

- `SerialHandler` (QThread) drena RX a máxima velocidad disponible.
- Emite `data_received` **por línea** → `main.update_data` (hilo GUI): parse, labels, `SensorBuffer`, plots.
- **No** hay decimación obligatoria a ~30 Hz para UI.
- Parte del control dual/H∞ lee sensores vía **texto de QLabel**, no solo buffer → acoplamiento UI↔control.

### 3.3 Worker máquina — BLOCKER (ausente)

Existe hilo RX. **No** existe worker que:

- posea el lazo H∞/FOV,
- decida mando,
- escriba TX,

independiente del timer Qt de 10 ms.

Reutilizable: `SensorBuffer` (thread-safe), `StepController.tick()` (FSM invocable), leyes H∞ en µm.

### 3.4 FOV “pulso” — EXTENSIÓN parcial / BLOCKER de timing

| Aspecto | Código | Efecto |
|---------|--------|--------|
| Idea pulso→brake→medir | Presente en FOV_VERIFY | Alineada en espíritu |
| `fov_pulse_dwell_ticks = 30` | Contado en ticks del timer 100 Hz | **≈ 300 ms** ON |
| Settles | Ya en ms (`perf_counter`) | Compatible con Fase C parcial |

Conclusión: el fine **intenta** no ser PWM continuo, pero la unidad de pulso sigue atada al reloj de 100 Hz → plank temporal.

### 3.5 H∞ en µm — AS-IS (ley)

- Path `hinf_native` / `HinfActuator` / live H∞: error en **µm**.
- Path dual/trayectoria **legacy** (sin step): aún fragmentos en **cuentas ADC**.

Estrategia “H∞ siempre en µm”: **casi AS-IS** en step/H∞; limpiar legacy = EXTENSIÓN menor.

---

## 4. Compatibilidad de la LUT micrométrica (análisis)

| Requisito LUT | ¿Código listo? |
|---------------|----------------|
| Flash para tabla const | Capacidad sí; sección/objeto **no** |
| DTCM para índice/estado ISR | Capacidad sí; colocación **no** |
| Hook 1 MHz o tick 1 kHz para disparar | Hook vacío / SysTick **sí** |
| Ejecutor `t_on` (timer one-shot / cuenta ms) | **No** |
| Protocolo “ref µm / átomo” distinto de `A,pwm` sostenido | **No** (solo `M`/`A`/`B`) |
| Calibración A/B por eje | Host tiene slopes/ID; MCU **no** las usa para actuar |

**Veredicto LUT:** estrategia **compatible con el silicio y el backbone**, **incompatible con el firmware de aplicación actual** hasta EXTENSIÓN (memoria + ejecutor + protocolo).

---

## 5. Mapa blocker → fase del plan

| Blocker verificado en código | Fase plan arquitectura |
|------------------------------|------------------------|
| `QTimer(10)` = reloj dual/traj/H∞ | **B** (worker M) + sacar control del Qt timer |
| UI sin plano 30 Hz; `update_data` por línea | **B** |
| Lectura de sensores desde QLabel en algunos paths | **B** |
| `fov_pulse_dwell_ticks` @ 100 Hz (~300 ms) | **C** (pulso en ms reales) |
| MCU solo hold PWM continuo; sin `t_on` | **D** |
| Linker sin ITCM/DTCM; sin LUT | **D** (prep memoria + tabla) |
| API −255…255 como único mando fine | **D** (átomo / CCR fino opcional) |
| Drop de comandos si `s_cmd_ready` lleno + TX blocking | Endurecer en **D**/protocolo |

---

## 6. Qué SÍ se puede afirmar con rigor

1. El **STM32F767ZI tiene recursos de memoria y tiempo** de sobra para control micrométrico con LUT + mini-pulso; eso **no está refutado** por el código.
2. El **código vigente no realiza** ese control: el lazo de posición vive en el PC a **100 Hz** mandando **PWM**, y el MCU **repite duty**.
3. Separar UI 30 Hz del lazo **no está hecho**; solo el RX está en hilo propio.
4. Las estrategias documentadas (`plan_arquitectura_…`, FOV ±8 µm, LUT) son **hoja de ruta compatible**, no descripción del software actual.
5. El camino de menor riesgo verificado por el código:  
   **B (host worker + UI snapshot) → C (dwell en ms) → D (átomo/LUT/timer en MCU + memoria)**  
   sin reescribir el backbone 1 MHz ya validado.

---

## 7. Criterio de “compatible” para no autoengañarse

Hasta que existan **tests S1** (FPS UI no cambia residual) y un **pulso fine con `t_on` medido en MCU o en worker con ms wall-clock**, decir “ya somos compatibles con control micrométrico máquina-rápido” sería **falso respecto al código auditado el 2026-07-14**.

Hoy: **base compatible / producto aún en modelo PC-PWM @ 100 Hz.**

---

*Auditoría: 2026-07-14. Fuentes: linker, `app_rt`/`app_control`/`app_serial`/`app_motor`, `test_service`, `hinf_service`, `step_controller`/`step_config`, `serial_handler`, `main.update_data`.*

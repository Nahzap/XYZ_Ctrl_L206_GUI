# Estrategia: lazo máquina-rápido + UI a ~30 Hz (sin interferir)

| Campo | Valor |
|-------|-------|
| **Fecha** | 2026-07-13 |
| **Hora** | 23:54 (UTC-4) |
| **Proyectos** | `XYZ_Ctrl_L206_GUI` + `MycoViT_XY_Controller` |
| **Principio de producto** | El humano usa el programa y exige exactitud. No gestiona COM ni tasas. Las máquinas miden y controlan lo más rápido posible. |
| **Documento hermano** | `plan_precision_fov_um_2026-07-13.md` (meta ±8 µm) |
| **Estado** | Estrategia aprobada para diseño — pendiente de ejecución |
| **Auditoría vs código** | `Docs_auditoria_compatibilidad_estrategia_2026-07-14.md` (2026-07-14) — base HW/FW sí; producto aún PC-PWM @ 100 Hz |
| **Plan de implementación** | `Docs/20260714_0032_Plan_Implementacion_Control_Micrometrico_Rapido.md` (2026-07-14 00:32) |

---

## 1. Objetivo

**Quitar el techo de ~100 Hz como reloj de medida/control**, y dejar la interfaz en un plano aparte (~30 Hz) que **solo observa** snapshots, sin participar en el lazo.

Metas de sistema:

| ID | Meta | Criterio medible |
|----|------|------------------|
| **S1** | UI ≠ reloj de control | Cambiar FPS UI (15/30/60) **no** cambia residual, `t_verify`, ni duración de mini-pulso |
| **S2** | Máquinas al máximo útil | Tasa de decisión/actuación delimitada por planta/MCU/enlace, no por `QTimer(10)` |
| **S3** | Exactitud de producto | Compatible con meta FOV ±8 µm ≥ 300 ms (`plan_precision_fov_um_…`) |
| **S4** | Experiencia humana | Operador no configura baud/COM más allá de “conectar”; ve µm y OK/FAIL |

---

## 2. Diagnóstico: por qué 100 Hz es inaceptable como lazo

Hoy (de facto):

```text
QTimer 10 ms (100 Hz)
  → lee última telemetría
  → H∞ / StepController en µm
  → send A,a,b
  → (FOV) dwell N ticks del mismo timer  →  pulso de cientos de ms
```

| Hecho | Consecuencia |
|-------|----------------|
| El timer de trayectoria/dual/H∞ es **10 ms** | El periodo de **decisión de potencia** ≈ 100 Hz |
| FOV `dwell` cuenta ticks de ese timer | Un “pulso fino” ≈ N×10 ms (p.ej. 30× → ~300 ms) |
| UI y control comparten presión de hilo/cola | Refresco/gráficas pueden retrasar o ensuciar el lazo |
| MCU mide @ **1 MHz** y aplica motores @ **1 kHz** | Capacidad de máquina **no** está en el camino AUTO del PC |
| Serial @ 1 Mbps sobra para comandos cortos | **COM no es el cuello**; la arquitectura PC-timer sí |

**Conclusión:** el eslabón lento a eliminar no es “el USB”. Es **usar el ritmo de la aplicación de escritorio como señal de control**.

---

## 3. Principio de arquitectura (innegociable)

Tres planos separados. Ninguno es reloj de otro.

```text
┌─────────────────────────────────────────────────────────┐
│  PLANO H — Humano / UI (~30 Hz)                         │
│  Plots, botones, estado, tolerancias, trayectoria       │
│  Solo LEE snapshots. Nunca genera PWM ni corta pulsos.  │
└──────────────────────────▲──────────────────────────────┘
                           │ cola / shared state (decimado)
┌──────────────────────────┴──────────────────────────────┐
│  PLANO M — Máquinas (lo más rápido posible)             │
│  Medida + control + telemetría cruda + mando            │
│  Host worker y/o MCU. Independiente del FPS de UI.      │
└──────────────────────────▲──────────────────────────────┘
                           │ serial / futuro enlace
┌──────────────────────────┴──────────────────────────────┐
│  PLANO RT — Firmware                                    │
│  ADC @ 1 MHz · actuación PWM · mini-pulso · (C(z) fine) │
└─────────────────────────────────────────────────────────┘
```

Reglas:

1. **H no manda potencia.** Manda intención: modo, refs en µm, start/stop, FOV size, tol de cierre.
2. **M ejecuta** la ley en µm y/o despacha átomos al MCU a la tasa máxima sostenible.
3. **RT cronometra** pulsos y muestreo; es dueño del tiempo fino (ms → µs).
4. Bajar UI a 15 Hz o subirla a 60 Hz **no debe** cambiar S2/S3.

---

## 4. Estrategia de comunicación máquina–máquina

Objetivo: **máxima tasa útil**, transparente al humano.

| Prioridad | Qué viaja | Ritmo objetivo | Notas |
|-----------|-----------|----------------|-------|
| P0 | Muestras / estado para control | Máximo que el lazo consuma sin pérdida | Hoy CSV streaming; decimar solo hacia UI |
| P0 | Comandos de actuación / refs | Lo más rápido que el ejecutor acepte | No atado a 30 Hz ni a 100 Hz de Qt |
| P1 | Snapshot hacia UI | **~30 Hz** | Última posición µm, PWM, modo, flags |
| P2 | Logs / métricas de punto | Por evento | `POINT_METRICS`, no por frame de plot |

Política de enlace (evolutiva):

1. **Corto plazo:** mismos cables/COM; separar **hilos** y **consumidores** (control vs UI).
2. **Medio plazo:** pulsos/átomos y timing fino en **MCU** (el enlace deja de transportar PWM continuo a 100 Hz).
3. **Opcional:** framing binario si CSV limita el plano M; la UI sigue a 30 Hz igual.

El baud/COM es detalle de M↔RT. El producto no lo expone como “forma de controlar”.

---

## 5. Dónde vive cada reloj (decidido)

| Función | Reloj dueño | No dueño |
|---------|-------------|----------|
| Muestreo láser | MCU 1 MHz | UI, QTimer |
| Mini-step / duración de pulso | MCU (ms/µs) | Ticks de GUI |
| Ley H∞ en µm (supervisor) | Worker M @ tasa máxima útil en host, **o** MCU | `QTimer` de pantalla |
| Settling / hold temporal | Reloj monotónico de M o RT | FPS del plot |
| Actualizar labels/gráficas | UI ~30 Hz | — |

**H∞ permanece en micrómetros** (norma ya aplicada). Solo cambia **quién** integra el tiempo y **quién** convierte \(u\) → átomo de motor.

---

## 6. Plan por fases (ejecución)

### Fase A — Contrato de planos (diseño cerrado, 0.5 día)

**Entregable:** este documento + checklist de “quién es dueño del tiempo” revisado.

**DoD:** equipo alineado: UI 30 Hz ≠ control; meta S1 escrita como test de aceptación.

---

### Fase B — Desacoplar UI del lazo en el PC (impacto alto, 1–2 días)

**Qué**
- Plano H: timer / paint **~30 Hz** solo para widgets y plots (decimar).
- Plano M: **worker dedicado** (hilo) que:
  - drena el puerto a máxima velocidad;
  - actualiza estado de control;
  - ejecuta Step/H∞ / FOV;
  - envía comandos sin esperar al repaint.
- La UI se suscribe a **snapshots** (cola lock-free / signals throttled), nunca al byte stream crudo como reloj.

**DoD / test S1**
- Misma trayectoria con UI a 15 Hz vs 30 Hz vs 60 Hz → distribución de residuales y `t_verify` estadísticamente igual.
- Matar o pausar updates de plot **no** detiene el lazo M.

**No hacer en B:** aún no migrar H∞ al MCU; primero romper el acoplamiento Qt.

---

### Fase C — Quitar 100 Hz como unidad de pulso (1–2 días)

**Qué**
- Duración de mini-pulso en **tiempo real (ms)**, no en “N ticks del timer 10 ms”.
- Worker M puede correr ≥ control rate actual; el corte de pulso no depende del FPS UI.
- Reducir dwell gordo de FOV hacia el átomo calibrado a mano (cerca de 0, suave).

**DoD**
- Un pulso fino de duración objetivo T_ms (± jitter corto) medible en log.
- Misma amplitud, menor T → menos LSB movidos (mapear autoridad).

**Puente a precisión FOV:** alinea con fine tras 2–4 locks y autoridad ≤ banda ±8 µm.

---

### Fase D — Migrar el tiempo fino al MCU (2–4 días) — elimina el techo host

**Qué (comando de intención, ejecución local)**
- Host envía refs / “corrija Δµm” / modo fine-hold (protocolo extendido o reuso creativo de `A` + params).
- MCU genera **mini-pulso** con SysTick o timer dedicado (ms → sub-ms), midiendo con buffer del lazo 1 MHz.
- Host deja de sostener `A,pwm` como PWM continuo a ritmo de aplicación.

**DoD**
- Latencia mando→movimiento y duración de pulso **insensibles** a carga de UI y a 30 Hz.
- Capacidad de átomo ~1–3 LSB demostrada en banco (calibración manual → tabla por eje).

**Stretch D2:** banda fine / C(z) parcial en `app_rt_control_hook` (1 MHz para estimar; actuación a tasa sensata).

---

### Fase E — Producto / invisibilidad COM (0.5–1 día)

**Qué**
- Conectar = autodetectar / última COM; baud fijo de fábrica en ambos lados.
- UI habla µm, modos, trayectoria; no “tasa de control”.
- Telemetría de debug oculta o pestaña avanzada.

**DoD:** un operador nuevo completa una fila FOV sin tocar baud ni entender 100 Hz.

---

## 7. Mapa de dependencia con el plan FOV ±8 µm

```text
plan_arquitectura (este)          plan_precision_fov
        │                                  │
        ├─ Fase B: UI ≠ lazo ──────────────┤
        ├─ Fase C: pulso en ms ────────────┼─► fine sin plank 300 ms
        └─ Fase D: átomo en MCU ───────────┴─► sostener ±8 µm
```

Sin este plan, el de precisión ajusta knobs **sobre un reloj lento**.  
Sin el de precisión, este plan hace el lazo rápido pero **sigue aceptando mal**.  
Ambos son necesarios; **este desbloquea la velocidad**; el FOV desbloquea el criterio de exactitud.

---

## 8. Riesgos

| Riesgo | Mitigación |
|--------|------------|
| Worker M satura PyQt con signals | Throttle a UI 30 Hz; batch |
| Subir solo el `QTimer` a 1 ms sin desacoplar | Prohibido como “solución”; no cumple S1 |
| Protocolo nuevo rompe GUI legacy | Mantener `A,a,b` para approach; fine en extensión versionada |
| MCU pulse sin calibración de átomo | Fase C/D exige mapa manual potencia×tiempo→LSB |
| Telemetría full-rate come CPU | Plano M consume; UI decima; opcional binario después |

---

## 9. Criterios de aceptación globales (release gate)

1. **S1** verificado (FPS UI no altera control).  
2. Control/telemetría de lazo **no** pasan por el timer de pintura.  
3. Duración de mini-pulso definida en tiempo máquina, no en frames UI.  
4. Meta FOV ±8 µm sigue siendo el KPI de exactitud (plan hermano).  
5. Operador no necesita conocer la tasa del lazo para operar.

---

## 10. Seguimiento

| Fase | Estado | Notas |
|------|--------|-------|
| A Contrato de planos | **Documentado** | 2026-07-13 23:54 |
| B Desacoplar UI 30 Hz | Pendiente | Primer trabajo de código en host |
| C Pulso en tiempo real (host) | Pendiente | Elimina dwell-por-tick-100Hz |
| D Mini-step / fine en MCU | Pendiente | Quita techo del PC de raíz |
| E COM invisible | Pendiente | Producto |

**Orden de ataque:** A (hecho) → **B** → **C** → D → E.  
No empezar por “subir baud” ni por “QTimer a 1 ms en el hilo UI”.

---

## 11. Resumen ejecutivo

| Pregunta | Respuesta |
|----------|-----------|
| ¿Qué hay que conseguir? | Medida/control máquina-rápidos; UI ~30 Hz solo visual |
| ¿Qué se elimina? | 100 Hz (`QTimer` 10 ms) como reloj del lazo y de los pulsos |
| ¿Qué ve el humano? | Exactitud en µm; no la fontanería COM |
| ¿Dónde empieza el trabajo? | Fase B: hilo/worker M + snapshots a UI |
| ¿Dónde muere el techo de verdad? | Fase D: timing de actuación en el STM32 |

---

*Documento creado: 2026-07-13 23:54 (UTC-4).*  
*Motivo: petición explícita de estrategia ante la rechazo del lazo @ ~100 Hz y la separación UI vs máquinas.*

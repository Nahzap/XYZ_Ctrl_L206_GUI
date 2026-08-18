# Auditoría: latencia y fiabilidad del autofoco en microscopía

**Fecha:** 2026-08-13
**Hora:** 18:50 (UTC-4)
**Punto de evidencia:** 1586/5292, SAMPLE_002 QUILACO_2026
**Ciclo medido:** T_AF = 62.43 s (COARSE+FINE 56.11 s; 22+39 mediciones S)
**Estado:** plan implementado (Fases A, B, C) el 2026-08-14; ver sección 12.
Fase D (I/O) pendiente. Las puertas A–C sólo se cierran midiendo en la máquina.

Este informe responde a un ciclo real de ~62 s por semilla. Con 5292 puntos, el
método actual no es viable: la sesión puede tomar días. El BPoF de este punto
cayó cerca del pico óptico (52 µm vs 51 µm del COARSE), pero el coste y la
inestabilidad de S impiden confiar en que eso se repita a escala de muestra.

Referencias previas (contrato, no latencia):

- `informe_auditoria_autofoco_coarse_fine_bpof_2026-08-06_1308.md`
- `informe_auditoria_autofoco_clahe_bpof_2026-08-06_1230.md`

---

## 1. Qué ocurrió en el log (cronómetro)

Punto 1586, 1 objeto (95617 px). Origen calibrado Z_cmd = 43.07 µm.

| Fase | Reloj | Duración | Planos | s/plano | Notas |
|---|---|---|---|---|---|
| Viaje XY | 18:45:18 → 18:45:21 | ~3.0 s | — | — | avance de trayectoria |
| Espera pre-captura | 18:45:21 → 18:45:22 | 1.5 s | — | — | `delay_before` = 1500 ms |
| Detección + origen | 18:45:22 → 18:45:23 | ~1.0 s | — | — | U2-Net + recalibración Z |
| COARSE | 18:45:23 → 18:45:42 | 19.0 s | 22/29 | 0.86 | early-stop sí actuó |
| FINE | 18:45:42 → 18:46:19 | 37.0 s | 39/39 | 0.95 | **sin early-stop** |
| Confirmación BPoF | 18:46:19 → 18:46:23 | 4.0 s | 4 | 1.00 | S cayó 29.7 % → vecinos |
| 3 fotografías | 18:46:23 → 18:46:25 | 2.0 s | 3 | 0.67 | reutiliza curva; 0 probes extra |
| Guardado + origen | 18:46:25 → 18:46:32 | 7.0 s | — | — | PNG 16-bit + retorno Z |
| **Ciclo AF interno** | — | **62.43 s** | 61+4+3 | — | log `Ciclo completo` |
| **Punto de muestra** | 18:45:18 → 18:46:32 | **~74 s** | — | — | lo que paga la trayectoria |

Desglose del tiempo útil vs desperdicio dentro del AF:

```
COARSE útil (subida 39→63 µm):     ~9 s
COARSE desperdicio (0→36 µm plano): ~11 s
FINE útil (entorno 46→56 µm):       ~10 s
FINE desperdicio (32→45 y 57→70):   ~27 s
Confirmación por S inestable:        4 s
Fotos + park:                        2 s
────────────────────────────────────────
Total AF                             62 s
```

Más de la mitad del ciclo (≈31 s) midió planos que la curva ya había descartado
como fuera de foco.

---

## 2. Proyección de sesión (por qué puede tomar días)

Parámetros de sesión (`test_parameters_template.json`):

- 5292 puntos, FOV 160 × 120 µm
- `full_scan` = true
- `n_fine_planes` = 39, `z_step_fine` = 1.0 µm, `z_scan_range` = 22.0 µm
- `z_step_coarse` = 3.0 µm
- `delay_before` = 1.5 s, `delay_after` = 0.5 s
- aprendizaje activo, 50 muestras objetivo (no reduce el Z-scan)

Sea `p` la fracción de puntos con objeto que dispara AF.

```
T_punto ≈ p · 74 s + (1 − p) · 5 s
T_sesion ≈ 5292 · T_punto
```

| p (hit-rate) | T_punto | T_sesión | Comentario |
|---|---|---|---|
| 10 % | 11.9 s | 17.5 h | optimista |
| 30 % | 25.7 s | 37.8 h | plausible en miel |
| 50 % | 39.5 s | 58.1 h | ~2.4 días |
| 100 % | 74.0 s | 108.7 h | ~4.5 días |

El usuario está en 1586/5292 (30 % de la trayectoria). Si el hit-rate real se
parece al de este tramo, la sesión no cierra en un turno de laboratorio.

Objetivo de diseño (sección 7): T_AF ≤ 15 s y T_punto con objeto ≤ 22 s.
Con p = 30 % eso baja T_sesión a ≈ 14 h; con p = 50 % a ≈ 18 h.

---

## 3. Contrato actual del algoritmo (lo que el código hace)

Cadena en `AutofocusService._optimize_focus_simple` + `_focus_surface_sync`:

```
origen Z calibrado (obligatorio)
  → COARSE: Z_min_hw → Z_max_hw, paso_c, early-stop
  → Z_c* = near_max(S_coarse) cerca del origen
  → reanclaje en Z_c*
  → FINE: N capas, paso_f, limitadas por ±Δ
  → BPoF = argmax(S_fine)
  → confirmación (re-medida + vecinos si S cae)
  → stack de N fotos desde la curva FINE (0 probes extra)
  → park en BPoF
```

Configuración que produjo el log (`camera_tab.autofocus`):

```
full_scan            = true
z_step_coarse        = 3.0 µm
z_step_fine          = 1.0 µm
n_fine_planes        = 39
z_scan_range (Δ)     = 22.0 µm
z_arrive_tol         = 1.0 µm
n_captures           = 3
capture_s_drop       = 7.5 %
roi_margin           = 15 px
score_samples/plano  = 1
```

Ventana FINE real (`build_fine_z_planes`):

```
half     = (39 − 1) / 2 = 19
Δ_req    = min(22.0, 1.0 · 19) = 19.0 µm
rango    = [51.04 − 19, 51.04 + 19] = [32.04, 70.04] µm
paso_eff = 1.000 µm
N        = 39
```

FINE no es un refinamiento: es un segundo barrido de 38 µm, casi el viaje
completo del piezo (el COARSE ya cubrió 0 → 63 µm).

---

## 4. Causas raíz (errores y desperdicios)

Se agrupan en tres familias: **tiempo estructural**, **S no repetible**,
**plan fotográfico contaminado**. No son independientes: la S inestable obliga
a ventanas FINE enormes y a confirmaciones extra.

### 4.1 FINE es un re-escaneo, no un refine

Evidencia en el log:

```
ENLACE COARSE→FINE: Z_c*=51.04 µm S=323.3
  → rango=[32.04, 70.04] µm (39 capas, paso_UI=1.000, Δ_max_UI=22.00)
COARSE+FINE completado en 56.11 s (22+39 mediciones S)
```

El COARSE ya localizó el pico con claridad:

```
Z=39.04  S=242.7   (baseline)
Z=42.04  S=258.7
Z=48.04  S=268.6
Z=51.04  S=323.3   ← Z_c*
Z=54.04  S=317.2
Z=60.04  S=278.1
```

Un refine de ±4 a ±6 µm (9–13 planos a 1.0 o 0.5 µm) basta para decidir entre
50, 51 y 52 µm. Los 26 planos restantes (32–45 y 57–70) son fondo plano
(S ≈ 242–293) y consumen ~27 s.

No hay early-stop en FINE. El de COARSE (`coarse_early_stop_patience` = 4,
`drop_rel` = 0.03) sí recortó 7/29 planos. FINE recorre siempre N entero.

Orden de visita FINE (`fine_sequence`):

```
[Z_c*] + [todos los demás en orden creciente de Z]
```

En el log: 51.04 → salto a 32.04 → 33, 34, … 70. El piezo hace un salto de
19 µm y luego barre 38 µm. Eso maximiza tiempo de settle y empeora el primer
S del ancla (ver 4.4).

### 4.2 COARSE parte siempre de Z ≈ 0 aunque el origen esté en 43 µm

`use_full_range` / checkbox «Escaneo Completo» fuerza:

```
z_min, z_max = z_min_hw, z_max_hw     # log: 0.04 → 86.10 µm
n_steps = int((z_max − z_min) / paso_c) + 1   # 29
```

Los 13 primeros planos (0.04–36.04 µm) tienen S = 237–242, meseta sin
estructura. El pico aparece al pasar el origen (43 µm). El early-stop exige
además `z_current >= z_center`, así que aunque el pico estuviera abajo del
origen el barrido seguiría hasta el centro.

`z_max_recorded` existe en el servicio y **nunca se usa**. El modo aprendizaje
de microscopía (50 muestras) no alimenta un prior Z. Cada semilla paga un
COARSE de viaje completo.

El tooltip de la GUI dice «desmarcar para Golden Section Search». Eso es
falso: desmarcar solo recorta a `Z_actual ± Δ`. Golden Section vive en
`multi_object_autofocus.py` y no es la ruta de microscopía.

### 4.3 Coste por plano ≈ 0.9 s (no es el cálculo de S)

Cada `evaluate_s_at_z` ejecuta el contrato:

```
MOVE → Z_STATIC (3 lecturas en banda, tol=1.0 µm)
     → OPTICAL flush (max(3, ceil(t_exp · fps) + 3) frames)
     → MEASURE (acquire_scientific_frame 2590×1942 RAW12)
     → RoiTracker.update() = U2-Net en el frame completo
     → S = CLAHE-HF-v4 sobre ROI
```

Con exposición 50 ms y 14 fps:

```
n_flush = max(3, ceil(0.05 · 14) + 3) = 4 frames  ≈ 0.29 s
```

A eso se suma el movimiento del piezo, 3 lecturas Z, la toma científica y
**U2-Net en cada plano**. El tracker está en `static_window=True` (la ventana
de medida no se mueve), pero `update()` igual llama `_detect()` para chequear
contención. El log lo confirma:

```
ROI estático: segmentación contenida en los 61 planos; S comparable
```

61 inferencias U2-Net + 4 de confirmación + 3 de foto ≈ 68 pases del detector
por semilla. En CPU, U2-Net sobre 5 Mpx es compatible con 200–400 ms/plano:
eso solo puede explicar 14–27 s del ciclo.

`score_samples_per_plane` = 1. No hay mediana. Un frame malo entra directo a
la tabla y puede ganar el argmax.

### 4.4 S no es repetible en la misma Z (error de fiabilidad)

Misma física, distintas pasadas:

| Z (µm) | COARSE | FINE 1ª | FINE 2ª / ancla | Confirm | Foto |
|---|---|---|---|---|---|
| 48.04 | 268.6 | 344.2 | — | — | 282.5 (a 47.05) |
| 51.04 | 323.3 | 266.85 | 277.55 | — | — |
| 52.04 | — | 362.4 | — | 280.0 | 298.4 |

Caídas relativas:

```
S_FINE(51.04) / S_COARSE(51.04) = 277.55 / 323.3  → −14 %
S_confirm(52.04) / S_FINE(52.04) = 280.0 / 362.4   → −29.7 %
S_foto(52.06) / S_FINE(52.04)   = 298.4 / 362.4    → −17.7 %
```

El umbral de alarma del propio código es 12 % en el ancla y 8 % en
confirmación. Ambos se dispararon. El BPoF se mantuvo solo porque ningún
vecino superó +3 %.

Agujero no físico en la curva FINE:

```
Z=50.04  S=353.5
Z=51.04  S=277.6   ← hueco de −23 % en 1 µm
Z=52.04  S=362.4   ← BPoF
```

Una PSF de microscopio no abre un pozo de 80 puntos S en 1 µm y lo cierra en
el siguiente. Ese 51.04 es la primera toma FINE, justo después del salto
63 → 51 µm. El frame o el flush no representaron el plano estático.

Causas en `focus_metric.py` (CLAHE-HF-v4):

1. El 90 % de S es la media del 0.5 % de gradientes más fuertes
   (percentil 99.5). Unos pocos píxeles (polvo, borde residual, flicker)
   mueven S decenas de puntos.
2. CLAHE se recalcula por plano. El histograma local cambia con Z y con el
   frame; no es una escala absoluta.
3. Una sola muestra por Z (`score_samples_per_plane` = 1).
4. Rolling shutter + 50 ms de exposición: un flush corto tras un salto de
   19 µm deja energía de movimiento en el RAW.
5. U2-Net re-segmenta cada plano. Con `static_window` la máscara de medida
   está congelada, pero el overlay y el chequeo de contención siguen
   dependiendo de una silueta que crece/encoge con el desenfoque.

El BPoF 52.04 µm de este punto está a 1 µm del Z_c* y es creíble. La
confianza «OK» del log (`span_rel=0.4195`, `prom_rel=0.2614`) mide contraste
de la curva, no repetibilidad. Una curva ruidosa puede tener span alto y
seguir eligiendo un outlier.

### 4.5 El plan de 3 fotos no logra el ΔS pedido

Objetivo GUI: caída S = 7.5 % respecto al BPoF.

El plan se construye sobre la curva FINE (S_pico = 362.4). 7.5 % de 362.4 es
335. Por eso eligió:

```
FOTO 1  Z=47.04 µm  offset=−5.0 µm   (S_FINE=331.5 ≈ ΔS 8.5 %)
FOTO 2  Z=52.04 µm  offset=+0.0 µm   (S_FINE=362.4)
FOTO 3  Z=53.04 µm  offset=+1.0 µm   (S_FINE=319.9 ≈ ΔS 11.7 %)
```

S real al fotografiar:

```
FOTO 1  S=282.5
FOTO 2  S=298.4
FOTO 3  S=299.1
```

Las tres imágenes tienen S indistinguible (ΔS foto 2 vs 3 = +0.2 %). El
lado +Z no se alejó ópticamente; el lado −Z se alejó 5 µm porque la curva
FINE infló el pico. El stack no es un bracket de 7.5 %: son tres tomas
casi al mismo foco, una de ellas 5 µm abajo por un S no repetible.

### 4.6 Overheads de microscopía que no son AF pero suman

- `delay_before` = 1500 ms en cada punto, también cuando luego el AF vuelve
  a mover Z durante 56 s.
- Recalibración de origen **antes** del AF y **después** del guardado.
- Guardado PNG 16-bit ≈ 7 s (18:46:25 → 18:46:32). En 5292 hits eso es horas
  solo de I/O.
- `z_arrive_stable_reads` = 3 en cada plano. Correcto para no medir en
  movimiento; caro si se visitan 61 planos.

---

## 5. Diagnóstico de este BPoF (¿acertó?)

| Indicador | Valor | Lectura |
|---|---|---|
| Z_c* COARSE | 51.04 µm, S=323.3 | pico interior, subida clara desde 39 µm |
| BPoF FINE | 52.04 µm, S=362.4 | 1 µm del COARSE; dentro de 1 paso fino |
| Confirmación | S=280.0 (−29.7 %) | no desplaza Z; S no es comparable |
| Foto BPoF | S=298.4 | tampoco recupera 362 |
| Vecinos FINE | 50.04=353.5, 53.04=319.9 | el máximo 52 es plausible |
| Agujero 51.04 | S=277.6 | medición inválida, no óptica |

Conclusión: **en este punto el Z es aceptable (±1 µm)**. El método no es
aceptable porque (a) tarda 62 s en decidir 1 µm, (b) S no se puede usar como
escala para el stack, (c) un outlier en otro grano puede ganar el argmax.

No se debe «abrir más FINE» para compensar el ruido. Eso es lo que ya hace
N=39 y es exactamente el problema.

---

## 6. Estrategias de mitigación (fiables, por fases)

Principio: **localizar barato, refinar poco, medir S solo cuando el piezo
está quieto y el frame es fresco**. No se propone Golden Section a ciegas
sobre 86 µm: el COARSE actual ya funciona si se acota.

### Fase A — Parámetros (hoy, sin tocar código)

Cambio en GUI / JSON. Riesgo bajo. Efecto inmediato.

| Parámetro | Actual | Propuesto | Por qué |
|---|---|---|---|
| n_fine_planes | 39 | 9 | refine, no re-scan |
| z_step_fine | 1.0 µm | 0.5 µm | resuelve 50/51/52 sin 38 µm de span |
| z_scan_range | 22.0 µm | 6.0 µm | tope de seguridad, no ventana de trabajo |
| z_step_coarse | 3.0 µm | 3.0 µm | ya localiza; no tocar |
| full_scan | true | true (A) / false (B) | ver Fase B |
| capture_s_drop | 7.5 % | 7.5 % | no sirve hasta que S sea repetible |
| delay_before | 1500 ms | 300 ms | el AF ya espera Z_STATIC |

Ventana FINE resultante:

```
half  = 4
span  = min(6.0, 0.5 · 4) · 2 = 4.0 µm
N     = 9
rango ≈ [Z_c* − 2.0, Z_c* + 2.0] µm
```

Ahorro estimado vs este log: 39 → 9 planos FINE ≈ **−28 s**. T_AF ≈ 34 s.
Aún alto, pero la sesión deja de ser de días si el hit-rate no es 100 %.

Criterio de aceptación Fase A: en 10 semillas, |BPoF − Z_c*| ≤ 2.0 µm y
ningún BPoF en el borde de la ventana FINE.

### Fase B — Algoritmo de búsqueda (código, riesgo medio)

B1. Early-stop FINE, simétrico al COARSE.

```
si i − i_pico ≥ patience_fine (3)
   y S ≤ S_pico · (1 − drop_rel)     # drop_rel = 0.05
   y el pico no está en el borde
→ terminar FINE
```

B2. Visitar FINE desde el centro hacia afuera (no 51 → 32 → 70):

```
Z_c*, Z_c*+h, Z_c*−h, Z_c*+2h, Z_c*−2h, ...
```

El early-stop corta al bajar ambos lados. El ancla no sufre un salto de 19 µm
antes de medirse.

B3. COARSE acotado con prior.

```
si hay historial de k≥5 BPoF de la sesión:
    Z0 = mediana(BPoF)
    σ  = max(4 µm, 2 · desv_abs_mediana)
    COARSE en [Z0 − σ, Z0 + σ] ∩ hardware
si no:
    COARSE en [origen − 20 µm, origen + 25 µm] ∩ hardware
```

`z_max_recorded` debe pasar de campo muerto a este prior. El primer punto de
una muestra puede seguir siendo full-scan; los siguientes no.

B4. No confirmar BPoF si el pico FINE es interior y `prom_rel` ≥ umbral.
Ahorro: 4 s. Confirmar solo si el ancla discrepa >12 % o el pico está a 1
plano del borde.

B5. U2-Net no es parte de S en ventana estática.

Con `static_window=True` la medida no usa la silueta viva. Detectar en el
plano 0 (referencia) y cada K=8 planos (o solo al final) basta para el
chequeo de contención. Ahorro esperado: 0.2–0.4 s × ~50 planos ≈ **10–20 s**.

Estimación Fase A+B sobre este log:

```
COARSE acotado 43±20 µm, paso 3:   ~14 planos × 0.55 s  ≈  8 s
  (0.55 s si se quita U2-Net/plano)
FINE 9 planos, visita radial:                   ≈  5 s
Confirmación omitida (pico interior):           ≈  0 s
Fotos 3:                                        ≈  2 s
────────────────────────────────────────────────────────
T_AF objetivo                                   ≈ 15 s
```

### Fase C — Fiabilidad de S (código, riesgo alto si se toca la fórmula)

C1. Rechazar agujeros de 1 plano antes del argmax.

```
si S(Z) < 0.85 · min(S(Z−h), S(Z+h))
→ re-medir Z una vez; si sigue, marcar inválido y no votar
```

Eso habría re-medido 51.04 (277 vs vecinos 353/362) y evitado el ancla
sucia. No cambia la fórmula.

C2. Mediana de 2 tomas **solo** en el ancla FINE y en el BPoF candidato.
No en los 39 (o 9) planos: el coste se paga donde decide Z.

C3. Bajar la fragilidad del 0.5 % más fuerte, sin reescribir v4 de golpe:

- percentil 99.5 → 98 (más píxeles, menos un pixel de polvo)
- o peso RAW 0.90 / CLAHE 0.10 → 0.70 / 0.30

Validar con 20 curvas de la misma semilla (repetir AF 5 veces, misma XY).
Métrica de éxito: CV de S en el mismo Z ≤ 5 %; hoy está en 15–25 %.

C4. El stack fotográfico debe usar S de las fotos, no S_FINE inflado.
Si |S_foto(BPoF) − S_FINE| / S_FINE > 10 %, recalcular offsets con la S
de la foto 2 (o aceptar offsets fijos ±2–3 µm hasta que C1–C3 cierren).
Objetivo: las 3 PNG deben cumplir ΔS ≈ 7.5 % **en el archivo guardado**,
no en la tabla FINE.

### Fase D — I/O y microscopía (después de A–C)

- Guardado asíncrono (cola) o TIFF sin predictores pesados. Meta: T_save ≤ 1 s.
- No volver al origen entre puntos si el siguiente COARSE usa prior local.
- `delay_after` 500 ms solo si XY arranca antes de terminar el park Z.

No priorizar D mientras T_AF sea 62 s: el I/O es 7 s, el AF es 62 s.

---

## 7. Indicadores de avance (qué medir en cada semilla)

Instrumentar en el log una línea `AF_KPI` por ciclo. Sin esto no hay forma
de saber si una fase mejoró o solo movió el tiempo.

### 7.1 Tiempo y presupuesto

| KPI | Fórmula | Este log | Meta Fase A | Meta A+B |
|---|---|---|---|---|
| T_AF | t_park − t_inicio_scan | 62.43 s | ≤ 35 s | ≤ 15 s |
| T_COARSE | t_fin_coarse − t_ini | 19 s | ≤ 16 s | ≤ 8 s |
| T_FINE | t_fin_fine − t_ini | 37 s | ≤ 10 s | ≤ 5 s |
| T_confirm | t_plan − t_fin_fine | 4 s | ≤ 4 s | ≤ 1 s |
| T_save | t_origen_final − t_park | 7 s | 7 s | ≤ 1 s (D) |
| T_punto | t_avance_siguiente − t_avance | 74 s | ≤ 48 s | ≤ 22 s |
| T_plano | T_AF / N_S | 0.91 s | ≤ 0.90 s | ≤ 0.55 s |

### 7.2 Trabajo de búsqueda

| KPI | Fórmula | Este log | Meta |
|---|---|---|---|
| N_coarse | planos COARSE medidos | 22 / 29 | ≤ 16; recorte ≥ 30 % |
| N_fine | planos FINE medidos | 39 / 39 | ≤ 9; recorte ≥ 0 si N=9 |
| N_S | COARSE+FINE+confirm+fotos | 68 | ≤ 20 |
| span_FINE | Z_fine_max − Z_fine_min | 38.0 µm | ≤ 6.0 µm |
| early_stop_fine | 1 si cortó | 0 | 1 cuando el pico es interior |
| U2Net_por_ciclo | llamadas detect() | 61+ | 2–8 |

### 7.3 Calidad de Z (no negociable al acelerar)

| KPI | Fórmula | Este log | Meta | Abortar si |
|---|---|---|---|---|
| ΔZ_cf | \|BPoF − Z_c*\| | 1.00 µm | ≤ 2.0 µm | > 4.0 µm |
| borde_FINE | pico en idx 0 o N−1 | no | no | sí → ampliar 1 vez ±2 µm |
| span_rel | (S_max − S_min) / S_mediana | 0.4195 | ≥ 0.05 | < 0.015 (ya existe) |
| prom_rel | (S_max − S_mediana) / S_mediana | 0.2614 | ≥ 0.03 | < 0.003 |
| agujero_1px | S < 0.85 · min(vecinos) | sí (51.04) | 0 | > 0 sin re-medida |

### 7.4 Repetibilidad de S (el error que hoy se ignora)

| KPI | Fórmula | Este log | Meta |
|---|---|---|---|
| ε_ancla | \|S_FINE(Z_c*) − S_COARSE(Z_c*)\| / S_COARSE | 14.1 % | ≤ 8 % |
| ε_confirm | (S_FINE(BPoF) − S_conf) / S_FINE | 29.7 % | ≤ 8 % |
| ε_foto | \|S_foto(BPoF) − S_FINE\| / S_FINE | 17.7 % | ≤ 10 % |
| CV_S(Z*) | desv / media en 3 tomas del mismo Z | ~0.13 | ≤ 0.05 |
| ΔS_stack_real | 1 − min(S_fotos) / S_foto(BPoF) | ≈ 5.3 % (282/298) | 7.5 % ± 2.5 % |
| asimetría_stack | \|offset+ / offset−\| | 1 / 5 = 0.20 | 0.5–2.0 |

`ε_confirm` = 29.7 % es la alarma principal de este ciclo. Si A+B bajan
T_AF pero `ε_confirm` sigue > 15 %, el Z acelerado no es de fiar.

### 7.5 Progreso de sesión

| KPI | Fórmula | Uso |
|---|---|---|
| p_hat | n_AF / n_puntos_vistos | hit-rate empírico |
| T_punto_med | media móvil 50 puntos | ETA estable |
| ETA | (N − n) · T_punto_med | decisión de parar/seguir |
| n_BPoF_borde | acumulado | si crece, Δ FINE es corto |
| n_reconfirm | ciclos con ε_confirm > 8 % | si no baja, Fase C es bloqueante |

Línea de log propuesta (una por semilla):

```
AF_KPI T_AF=62.43 T_c=19.0 T_f=37.0 T_k=4.0 N_c=22/29 N_f=39/39
       Zc=51.04 Z*=52.04 dZ=1.00 e_ancla=0.141 e_conf=0.297
       e_foto=0.177 spanF=38.0 agujero=1 borde=0
```

Parseable, una línea, comparable entre commits.

---

## 8. Plan de implementación (orden y puertas)

No implementar todo a la vez. Cada fase tiene puerta cuantitativa.

### Paso 0 — Instrumentar KPI (0.5 día)

Añadir `AF_KPI` y un acumulador de sesión (p_hat, ETA, medianas de T_AF y
ε_confirm). Sin esto, A–D se evalúan a ojo.

Puerta: 5 ciclos de microscopía escriben la línea y el ETA en UI.

### Paso 1 — Fase A, solo JSON/GUI (mismo día)

Bajar N_fine a 9, paso fino 0.5 µm, Δ = 6 µm, delay_before 300 ms.
Correr 15 puntos de la misma muestra.

Puerta A:

- mediana T_AF ≤ 35 s
- mediana |BPoF − Z_c*| ≤ 2 µm
- 0 BPoF en borde FINE
- ε_ancla no empeora vs este log (≤ 15 %)

Si hay BPoF en borde: N=11 o Δ=8, no volver a N=39.

### Paso 2 — Fase B1+B2 (early-stop + visita radial) (1 día)

Puerta B12:

- N_fine medido mediana ≤ 7 (con N planificado 9)
- T_FINE mediana ≤ 6 s
- misma puerta de Z que A

### Paso 3 — Fase B3+B5 (prior COARSE + U2-Net cada K planos) (1–2 días)

Primer punto de muestra: full-scan. A partir del 5º BPoF válido: COARSE
local. Tracker estático: detect() en plano 0 y cada 8.

Puerta B35:

- T_plano ≤ 0.60 s
- T_COARSE ≤ 8 s desde el punto 6
- U2Net_por_ciclo ≤ 8
- T_AF mediana ≤ 15 s
- p_hat · 22 + (1−p_hat) · 5 proyecta T_sesión < 20 h a 5292

### Paso 4 — Fase C (S) (2 días, con banco de 20 repeticiones)

C1 obligatorio (agujeros). C2 en ancla/BPoF. C3 solo si C1+C2 no bajan
ε_confirm por debajo de 10 %.

Puerta C:

- ε_ancla mediana ≤ 8 %
- ε_confirm mediana ≤ 8 %
- CV_S(Z*) ≤ 5 % en 5 repeticiones de la misma XY
- ΔS_stack_real = 7.5 % ± 2.5 % en las PNG
- 0 agujeros sin re-medida

### Paso 5 — Fase D (I/O) cuando T_AF ≤ 15 s

Puerta D: T_save ≤ 1 s, sin pérdida de 16-bit ni de JSON de posición.

---

## 9. Qué no hacer

- No subir N_fine ni Δ «por si el pico se escapa». El pico no se escapó;
  FINE midió 38 µm de fondo.
- No desactivar Z_STATIC ni el flush óptico para ganar 200 ms. El agujero
  en 51.04 muestra que el flush ya es justo en saltos grandes.
- No mezclar S_COARSE y S_FINE en el mismo argmax (el código ya lo evita;
  mantenerlo).
- No usar Golden Section sobre 86 µm: un mínimo local en la meseta 0–36 µm
  (S≈238) es el riesgo clásico, y esa meseta es larga.
- No confirmar con una sola toma para **desplazar** el BPoF. El margen +3 %
  actual es correcto; el problema es que se confirma siempre y S cae 30 %.
- No reescribir CLAHE-HF-v4 en el mismo commit que el early-stop FINE.
  Si Z y S cambian juntos, no se sabe qué rompió el BPoF.

---

## 10. Mapa de código (para la implementación)

| Pieza | Archivo | Qué tocar |
|---|---|---|
| Bucle COARSE / FINE | `src/core/services/autofocus_service.py` `_optimize_focus_simple` | early-stop FINE, orden radial, KPI, prior |
| Ventana FINE | `src/core/autofocus/bpof_candidates.py` `build_fine_z_planes` | no cambiar fórmula; N y Δ vienen de GUI |
| Rango COARSE | `_resolve_scan_range` | prior / origen ± ventana |
| Confirmación | `_confirm_bpof_before_stack` | omitir si pico interior sano |
| Stack | `_build_plan_from_measured_focus_curve` | ΔS sobre S_foto si ε_foto alto |
| S | `src/core/autofocus/focus_metric.py` | solo Fase C; agujero en el servicio, no aquí |
| U2-Net/plano | `src/core/autofocus/roi_tracker.py` `update` | skip detect si static_window y plano % K ≠ 0 |
| Defaults GUI | `src/gui/utils/camera_tab_ui_builder.py` | N=9, Δ=6, tooltip GSS falso |
| Persistencia | `src/config/test_parameters_template.json` `camera_tab.autofocus` | mismos defaults |
| Delay punto | `src/core/services/microscopy_service.py` | delay_before 300 ms |
| Config dataclass | `src/core/models/autofocus_config.py` | `get_search_info` estima 0.15 s/plano: **obsoleto** (realidad 0.9 s) |

`AutofocusConfig.get_search_info` predice `total_steps · 0.15 s`. Con 22+39
anunciaría 9 s; el ciclo real fue 62 s. Esa estimación no debe usarse para
ETA de microscopía.

---

## 11. Resumen ejecutivo

El autofoco acierta Z en este punto (52 µm) y fracasa en tiempo y en escala S.

Causa dominante de tiempo: FINE de 39 planos a 1 µm (±19 µm) sin recorte,
más U2-Net en los 61 planos, más COARSE desde 0 µm. Causa dominante de
error: S no se repite (ancla −14 %, confirmación −29.7 %, foto −18 %),
porque el 90 % de la métrica es el 0.5 % de bordes más fuertes y hay una
sola toma por Z. El stack de 3 fotos no cumple el 7.5 % de ΔS en disco.

Mitigación fiable:

1. Encoger FINE a 9 × 0.5 µm (hoy, sin código).
2. Visita radial + early-stop FINE + COARSE con prior + U2-Net cada K planos.
3. Rechazar agujeros de 1 plano y no fiar el stack a un S_FINE inflado.
4. Medir las 14 KPI de la sección 7 en cada semilla.

Puerta de éxito de la línea: T_AF ≤ 15 s, |BPoF − Z_c*| ≤ 2 µm,
ε_confirm ≤ 8 %, ΔS real en PNG = 7.5 % ± 2.5 %, ETA < 20 h a 5292 puntos
con el hit-rate de la muestra.

---

## 12. Implementación (2026-08-14)

Se implementaron los pasos 0 a 4 del plan (KPI, Fase A, Fase B1–B5, Fase C1–C4).
La Fase D (I/O de guardado) queda fuera: no se toca el guardado 16-bit mientras
la sesión del usuario está en curso.

### 12.1 Código nuevo

| Archivo | Responsabilidad |
|---|---|
| `src/core/autofocus/af_kpi.py` | `AfCycleKpi` (14 KPI por ciclo) y `AfSessionKpi` (medianas, p̂, ETA) |
| `src/core/autofocus/fine_scan_plan.py` | `center_out_sequence` (visita radial) y `RingDeclineStop` (early-stop por anillos) |
| `src/core/autofocus/z_prior.py` | `BpofPrior`: acota COARSE con el historial de BPoF; `bootstrap_window` para los primeros puntos |
| `src/core/autofocus/stack_plan.py` | `stack_asymmetry_ratio` y `rebalance_symmetric` para el bracket de las 3 fotos |
| `src/core/autofocus/persisted_params.py` | Repara el formulario guardado que reintroducía Δ=22 µm / 39 planos / tol=1 µm |

### 12.2 Cambios sobre código existente

| Archivo | Cambio |
|---|---|
| `autofocus_service.py` | KPI por ciclo; FINE centro-afuera con early-stop; prior COARSE; confirmación omitida si el pico FINE es interior y sano; mediana en el plano que decide; rechazo de agujeros de 1 plano; simetrización del stack |
| `bpof_candidates.py` | `find_isolated_dips`, `count_at_z`, `invalidate_z`, `isolated_dip_planes` |
| `focus_metric.py` | Percentil de bordes, peso de la rama RAW y CLAHE (clip/tile) parametrizables por llamada |
| `roi_tracker.py` | `detect_interval`: con ventana estática, U2-Net cada K planos; contadores de inferencias y saltos |
| `microscopy_service.py` | Línea de KPI de sesión con ETA por punto visitado |
| `autofocus_config.py` | Defaults de Fase A; `estimated_time_s` pasa de 0.15 a 0.9 s/plano (el 0.15 anunciaba 9 s para un ciclo de 62 s) |
| `camera_tab_ui_builder.py` | Defaults y tooltips: N=9, paso fino 0.5 µm, Δ=6 µm, tol 0.25 µm, delay 0.3 s |
| `camera_tab.py` | Aplica `sanitize_autofocus_form` al restaurar el formulario y avisa en el log |

### 12.3 Hallazgo no previsto: la Fase A no llegaba a la máquina

`camera_tab._load_default_parameters` restaura `camera_tab.autofocus` del JSON
**tal cual** y pisa los defaults del builder. Cambiar el builder y el JSON no
bastaba: la aplicación en ejecución reescribió
`test_parameters_template.json` durante esta entrega y devolvió los valores del
ciclo auditado (Δ=22 µm, `n_fine_planes`=39, paso fino 1.0 µm, tol 1.0 µm). Es
decir, al reiniciar se habría vuelto a los 61 planos.

`persisted_params.sanitize_autofocus_form` corrige al cargar sólo las dos
combinaciones que hacen mentir a la curva, y explica cada cambio en el log:

- **N_fine**: la ventana FINE no puede exceder ±1 paso grueso. Fuera de ahí FINE
  no refina el plano COARSE ganador, repite el barrido. 39 → 7 planos con paso
  grueso 3 µm y fino 1 µm.
- **tol_Z**: con tolerancia ≥ paso fino/2, dos planos FINE distintos pueden
  medirse en la misma Z real. 1.0 → 0.5 µm.

Los defaults de Fase A (Δ=6, coarse 3, fino 0.5, N=9, tol 0.25) pasan sin
cambios. El formulario reparado se reescribe en el JSON en el arranque, así que
la corrección ocurre una sola vez.

### 12.4 Pruebas

Intérprete: `CTRL_ENV` (el del proyecto; el intérprete base no tiene `pyqtgraph`
ni `control`).

| Alcance | Resultado |
|---|---|
| Área de autofoco (21 archivos) | **143 passed**, 0 failed |
| Suite completa (430 tests) | 399 passed, 2 skipped, 24 failed, 5 errors |

Los 24 fallos y 5 errores son **preexistentes y ajenos al autofoco**. Se verificó
comparando contra un worktree limpio en HEAD (`git worktree add … HEAD`), donde
falla exactamente el mismo conjunto:

| Módulo | Fallos | Causa |
|---|---|---|
| `test_step_controller.py` | 11 | El fake no tiene `_fov_approach_allows_pwm`, `_apply_fov_verify_actuator`, `_run_kz_closure`, `_fov_secondary_axis` |
| `test_fov_unique_resilience.py` | 6 | El fake no tiene `_infer_mesh_step_um` ni `_fov_step_coverage_ok`; flags CZ |
| `test_integration_complete.py` | 5 (errors) | Fixtures `K`, `tau`, `result`, `test_service` inexistentes |
| `test_camera_service_integration.py` | 2 | Worker mockeado no se limpia |
| `test_host_approach.py` | 2 | Tolerancia/deadzone del aproximador |
| `test_basler_worker.py` | 1 | `worker.camera` es `None` al cerrar |
| `test_step_config.py` | 1 | `effective_tol` bajo deadzone |
| `test_scientific_camera_config.py` | 1 | Exposición inconsistente en el JSON: `camera`=0.015 s vs `camera_tab`=0.05 s |

**Crash del proceso, también preexistente.** Ejecutar la suite en un solo proceso
mata el intérprete (exit 3, sin traza) al entrar en
`tests/test_hinf_with_real_data.py` después de cualquier test que cargue U2-Net:
conflicto nativo entre `torch` y `control`/`slycot`. Reproducido idéntico en
HEAD con `pytest tests/test_fov_unique_resilience.py
tests/test_hinf_with_real_data.py`. Mientras no se aísle, la suite se corre en
dos procesos:

```
pytest tests --ignore=tests/test_hinf_with_real_data.py
pytest tests/test_hinf_with_real_data.py
```

Aislado, ese módulo no mata el proceso pero da 1 passed y 2 errors, del mismo
tipo preexistente que `test_integration_complete.py`: usa `result`, `K` y `tau`
como fixtures y no existen.

Pruebas nuevas del área (todas en verde):

| Archivo | Qué fija |
|---|---|
| `test_af_kpi.py` | Línea AF_KPI, medianas y ETA de sesión |
| `test_fine_scan_plan.py` | Orden centro-afuera y parada por anillos |
| `test_z_prior.py` | Ventana COARSE del prior; un BPoF malo no ensancha la ventana |
| `test_stack_plan.py` | Asimetría y rebalanceo del bracket |
| `test_autofocus_fine_refinement.py` | FINE centro-afuera, early-stop y mediana en el ancla, en el servicio |
| `test_autofocus_dip_rejection.py` | Re-medida e invalidación de agujeros de 1 plano |
| `test_autofocus_confirm_and_stack_kpi.py` | Confirmación omitida con pico sano; ε_foto y ΔS_stack |
| `test_autofocus_stack_symmetry.py` | Caso real del log: bracket −5/+1 µm → ±1 µm sin sondear planos nuevos |
| `test_roi_tracker_detect_interval.py` | U2-Net cada K planos sin mover la ventana de medida |
| `test_persisted_autofocus_params.py` | El formulario auditado se repara; los defaults de Fase A no se tocan |

### 12.5 Lo que queda en manos del usuario

1. **`delay_before` sigue en 1.0 s** en el formulario guardado (la app lo
   reescribió). La migración no lo toca porque es una preferencia legítima de
   asentamiento, no un error de medida. Bajarlo a 0.3 s en la pestaña Cámara
   ahorra ~2 s por punto con objeto.
2. **Exposición inconsistente en el JSON** (`camera`=0.015 s vs
   `camera_tab`=0.05 s). Es el fallo preexistente de
   `test_scientific_camera_config.py`. Conviene unificarla antes de la próxima
   sesión: decide qué exposición es la científica.
3. **Puertas A, B12, B35 y C** exigen microscopía real. Al arrancar, la línea
   `AF_KPI` por ciclo y el resumen de sesión con ETA ya salen en el log; con
   15 puntos de la misma muestra se pueden evaluar todas.

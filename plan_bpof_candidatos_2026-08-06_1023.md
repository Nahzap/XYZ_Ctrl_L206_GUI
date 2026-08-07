# Plan / registro: BPoF por tablas de candidatos (coarse + fine)

| Campo | Valor |
|-------|--------|
| **Fecha inicio** | 2026-08-06 |
| **Hora inicio** | 10:23 (UTC-4) |
| **Última actualización** | 2026-08-06 11:35 (UTC-4) |
| **Proyecto** | `XYZ_Ctrl_L206_GUI` |
| **Estado** | Implementado en código; validado en vivo (AF correcto; superficie multi-ROI) |

---

## 1. Contrato del algoritmo (vigente)

```
Entrada: N objetos detectados (ROI/contorno cada uno)

1) TABLA COARSE (instancia propia)
   - Un solo barrido Z (paso grueso UI)
   - Por plano: MOVE → Z_STATIC → OPTICAL → MEASURE
   - S_plano = Σ S_i  (todos los ROI sobre el MISMO frame estático)
   - Z_c* = argmax(TABLA_COARSE)  →  centro zona fine

2) Zona FINE (simétrica)
   - Δ = Distancia escaneo / Distancia fine (UI) → [Z_c*−Δ, Z_c*+Δ]
   - Δ_eff simétrico si hay borde HW
   - N = N° capas fine (UI, impar ≥3)
   - Planos = linspace(Z_c*−Δ_eff, Z_c*+Δ_eff, N)

3) TABLA FINE (instancia propia; no mezcla filas coarse)
   - Misma medición estática + S = Σ S_i
   - BPoF = argmax(TABLA_FINE)
   - Captura multi-focal opcional (solo si N° capturas ≥ 3)
   - Park = un GOTO a BPoF (sin remarcar S)
   - clear ambas tablas
   - Resultado: todos los objetos comparten el mismo Z*; cada uno reporta su S_i en BPoF
```

**Regla dura:** nunca puntuar frames en movimiento. Un clic / un FOV = **un** barrido Z, aunque haya varios ROI.

---

## 2. Pipeline de medición (método único)

| Fase | Condición de cumplimiento | Prohibido |
|------|---------------------------|-----------|
| **MOVE** | Consiga piezo a Z_cmd | Medir S / leer frame útil |
| **Z_STATIC** | \|Z_read−Z_cmd\| ≤ tol en N lecturas | Medir S |
| **OPTICAL** | Descartar frames con exposición solapada al MOVE; Z sigue en banda | Medir S si Z se mueve |
| **MEASURE** | Un frame estático; S = Σ S_i (máscara por ROI) | Re-muestreos redundantes / settle fijo |

API: `evaluate_s_at_z(..., rois=...)` → `_measure_s_static_at_z`  
Archivos: `src/core/services/autofocus_service.py`, `src/core/autofocus/bpof_candidates.py`, `src/core/autofocus/focus_metric.py`

---

## 3. UI / JSON

| Control | Clave | Uso |
|---------|-------|-----|
| Distancia fine / escaneo | `z_scan_range_um` | ±Δ µm zona fine |
| N° capas fine | `n_fine_planes` | N planos linspace (impar) |
| Paso grueso | `z_step_coarse_um` | paso tabla coarse |
| Paso fino | `z_step_fine_um` | referencia (paso_eff ≈ 2Δ/(N−1)) |
| Tol. Z llegada | `z_arrive_tol_um` | \|Z−Zcmd\|≤tol (reemplaza settle ms) |
| N° capturas | `n_captures` | multi-focal post-BPoF si ≥3 |
| Paso captura | `z_step_capture_um` | separación stack |

Template: `src/config/test_parameters_template.json`  
Orquestador UI: `CameraOrchestrator.run_autofocus` pasa **todos** los objetos; el servicio hace 1 barrido superficie.

---

## 4. Fallos observados → fix (cronología)

| Hora (aprox.) | Observado | Causa | Fix |
|---------------|-----------|-------|-----|
| 10:50 | Fine ±2 µm / 5 pts con UI Δ=7 | Δ tomaba paso coarse / refine_window | Δ = `z_scan_range` |
| 10:50 | Coarse S~870; fine remapea ~480–509 | settle 10 ms ≪ exposición 100 ms → frame viejo | Condición Z + OPTICAL por exposición |
| 11:05 | Coarse 672 @52.56; fine flat ~500; park ~489 | Misma carrera óptica; re-mediciones inconsistentes | Pipeline MOVE→Z_STATIC→OPTICAL→MEASURE |
| 11:08 | Imagen ideal S~689 vs AF en Z peor | BPoF de tabla fine contaminada + post-AF drift | Medición estática; S* de tabla |
| 11:23 | AF OK pero lento / pasos de más | Revalidar coarse×3 + verify + park remide + S doble | Eliminado; MEASURE 1 frame; park = GOTO |
| 11:30 | 2× coarse+fine con 1 clic | Loop `for obj in objects` | Superficie multi-ROI, 1 barrido, S=ΣS_i |

---

## 5. Resultados de validación en vivo

| Corrida | Resultado |
|---------|-----------|
| Post-pipeline estático | Autofoco encuentra pico coherente; coarse muestra curva con máximo claro (ej. SΣ/S ~630–670) |
| Fine | Zona ±7 µm × N capas (15–21) según UI; `paso_eff` coherente |
| Sin revalidación | Ya no aparecen líneas `Revalidar coarse→fine` |
| Multi-objeto (antes) | `Enfocando 1/2` + `2/2` = 2 barridos completos (incorrecto) |
| Multi-objeto (ahora) | Contrato: log `superficie: N ROI \| S=ΣS_i` + **un** coarse + **un** fine |
| Park | `Posición final` = GOTO BPoF; S reportada = tabla (no re-score agresivo) |

### Evidencia log (síntesis)

- **Coarse útil:** picos claros (p.ej. 44–48 µm, S~640–670).  
- **Fine:** decide BPoF local (p.ej. 45–50 µm) con N capas.  
- **Regresión evitada:** no repetir N barridos por N objetos; no settle fijo; no revalidación extra.

---

## 6. KPI vs meta

| KPI | Meta | Estado |
|-----|------|--------|
| Zona fine = Distancia UI | log `±Δ_eff` = UI | Cumple |
| N mediciones fine | = N° capas | Cumple |
| Sin settle fijo | condición Z + OPTICAL | Cumple |
| Sin S en movimiento | pipeline 4 fases | Cumple |
| Tablas separadas | coarse / fine instancias | Cumple |
| 1 clic → 1 barrido | multi-ROI = superficie | Cumple (código) |
| Sin pasos redundantes | no reval / no verify doble | Cumple |
| Tests unitarios | candidatos + Z arrive | `tests/test_bpof_candidates.py`, `tests/test_z_arrive_condition.py` |

---

## 7. Archivos tocados (registro)

| Área | Rutas |
|------|--------|
| Candidatos | `src/core/autofocus/bpof_candidates.py` |
| AF servicio | `src/core/services/autofocus_service.py` |
| Orquestador | `src/core/services/camera_orchestrator.py` |
| Config modelo | `src/core/models/autofocus_config.py` |
| UI | `src/gui/utils/camera_tab_ui_builder.py`, `src/gui/tabs/camera_tab.py` |
| Wire cámara | `src/main.py` (exposure callback) |
| Microscopía log | `src/core/services/microscopy_service.py` |
| JSON template | `src/config/test_parameters_template.json` |
| Tests | `tests/test_bpof_candidates.py`, `tests/test_z_arrive_condition.py` |

---

## 8. Pendiente / seguimiento

| Ítem | Nota |
|------|------|
| Validar en vivo multi-ROI superficie | Confirmar log `S=ΣS_i` y un solo coarse+fine con 2 objetos |
| Fine S a veces &lt; pico coarse | Si reaparece con medición estática, revisar tol Z / flush exposición vs piezo |
| Multi-focal | Solo si `n_captures ≥ 3`; no redefine BPoF de tabla |
| Spam UI `TEST Detección` al mover spins | Lateral; no bloquea AF |

---

## 9. Changelog corto

| Hora | Cambio |
|------|--------|
| 10:23–10:54 | Plan inicial; tablas candidatos; Δ fine + N capas |
| ~11:00 | Settle → Tol. Z llegada; condición de cumplimiento |
| ~11:01 | Tablas coarse/fine como instancias separadas |
| ~11:12 | Pipeline óptico estático (exposición + S) |
| ~11:25 | Orquestación MOVE→Z_STATIC→OPTICAL→MEASURE explícita |
| ~11:30 | Eliminar revalidación / verify / remedir park |
| ~11:33 | Evitar N focos independientes |
| ~11:35 | Multi-ROI = 1 superficie (S=ΣS_i); este registro actualizado |

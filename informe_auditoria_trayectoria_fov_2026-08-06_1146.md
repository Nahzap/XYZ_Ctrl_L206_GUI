# Auditoría: trayectoria FOV zig-zag (1 punto → 1 serie)

| Campo | Valor |
|-------|--------|
| **Fecha** | 2026-08-06 |
| **Hora** | 11:46 (UTC-4) |
| **Proyecto** | `XYZ_Ctrl_L206_GUI` |
| **Síntoma** | En 1 posición física se “cumplen” 2–3 puntos de trayectoria |
| **Estado** | Causa identificada; anti-aceptación 11:46 incompleta → ver follow-up `…_1150.md` |

---

## 1. Contrato esperado

```
Trayectoria zig-zag (spacing = FOV_x × FOV_y, overlap 0)
  → 1 punto XY alcanzado (tol segura)
  → 1 serie autofoco (superficie ROI)
  → n imágenes multi-focales en BPoF ± paso Z
  → avanzar al siguiente punto nominal
```

Indicador clave: **1 índice de trayectoria = 1 FOV físico distinto** (Δ_nominal ≈ FOV).

---

## 2. Causa raíz (alta confianza)

| Parámetro | Valor en template / UI típico | FOV malla |
|-----------|------------------------------|-----------|
| `tolerance` | **100 µm** | 162 × 122 µm |
| Regla segura | tol ≤ FOV_min/10 ≈ **12 µm** | — |

**Mecanismo:**

1. Se acepta Pₙ con residual ≤ ±100 µm (puede quedar hasta 100 µm hacia Pₙ₊₁).
2. Distancia residual a Pₙ₊₁ ≈ FOV − 100 → **62 µm (X)** o **22 µm (Y)**.
3. Ambos ≤ tol 100 µm → **Host-stable accept** / FOV_OK **sin mover** al siguiente FOV.
4. Microscopy captura (o salta sin objetos) y hace `resume_trajectory` → índice++ otra vez casi en el mismo XY.

Microscopía **no** aplicaba el guardrail FOV/10 (solo TestTab al ejecutar trayectoria).

---

## 3. Estado máquina (referencia)

| Fase | Quién | Señal / método |
|------|-------|----------------|
| Approach / FOV settle | `TestService` + `StepController` | `point_complete` → `_accept_trajectory_point` |
| Pausa XY | `TestService` | `auto_advance=False` (microscopía) |
| AF + n frames | `MicroscopyService` → `AutofocusService` | `handle_autofocus_complete` |
| Avance índice | StateManager + `resume_trajectory(+1)` | 1 avance por captura/skip |

---

## 4. KPI / métricas a auditar en logs

| KPI | Meta | Cómo verlo |
|-----|------|------------|
| `tol_fov` | ≤ FOV_min/10 | `Política FOV: tol=` / `Tol. trayectoria clamp` |
| `Δ_nominal` entre puntos aceptados | ≈ FOV_x o FOV_y | log `AUDIT FOV accept` |
| `travel` desde prev nominal | ≥ Δ_nominal − tol | mismo log; si travel≪Δ → falso avance |
| Capturas / punto | 1 serie AF + n planos | `Guardando … multi-focales` una vez por `📍 Punto` |
| Host-stable sin viaje | 0 en malla (idx>0) | no debe aparecer con Δ_nominal grande |

### Strings de búsqueda

- `📍 Punto`
- `Host-stable accept`
- `Política FOV: tol=`
- `Tol. trayectoria clamp`
- `AUDIT FOV accept`
- `Captura con autofoco completada`
- `RESUME_TRAJECTORY`
- `Sin objetos en rango`

---

## 5. Correcciones aplicadas (esta sesión)

| Fix | Dónde |
|-----|--------|
| Clamp tol a FOV_min/10 en start microscopía + aviso log/UI | `microscopy_service.py` |
| Exponer FOV_x/FOV_y en params de trayectoria | `test_tab.py` |
| Anti-aceptación (11:46, **incompleta**): `Δ>2·tol` no dispara con FOV=162/tol=100 | `test_service.py` |
| Fix cobertura (11:50): `cov_tol=min(tol,Δ/10)` + tests | `…_1150.md` |
| Log `AUDIT FOV accept` (target, prev, actual, Δ_nom, travel) | `test_service.py` |
| Template `tolerance`: 100 → **12** µm | `test_parameters_template.json` |

---

## 6. Resultados esperados tras fix

| Antes | Después |
|-------|---------|
| 2–3 índices en ~mismo XY | 1 índice por FOV (~162/122 µm) |
| Host-stable en puntos intermedios de malla | Solo si ya se cubrió el paso nominal |
| tol 100 µm silenciosa en microscopía | Clamp + log si UI pide más |

---

## 7. Pendiente / seguimiento

| Ítem | Nota |
|------|------|
| Validar corrida real | Buscar `AUDIT FOV accept` y verificar travel ≈ FOV |
| `_advance_point` legacy | Sigue con `time.sleep`; unificar a resume async (deuda) |
| Skip sin objetos | Avanza índice sin imagen — correcto; no debe coincidir con falso XY |

---

## 8. Archivos

- `src/core/services/test_service.py`
- `src/core/services/microscopy_service.py`
- `src/gui/tabs/test_tab.py`
- `src/config/test_parameters_template.json`
- Este informe: `informe_auditoria_trayectoria_fov_2026-08-06_1146.md`

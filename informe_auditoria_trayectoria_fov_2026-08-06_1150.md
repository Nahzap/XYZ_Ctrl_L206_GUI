# Auditoría trayectoria FOV — follow-up cobertura

| Campo | Valor |
|-------|--------|
| **Fecha** | 2026-08-06 |
| **Hora** | 11:50 (UTC-4) |
| **Precedente** | `informe_auditoria_trayectoria_fov_2026-08-06_1146.md` |
| **Estado** | Bug en anti-aceptación corregido + tests |

---

## 1. Hallazgo en validación del fix 11:46

La regla `Δ_nominal > 2·tol` **no protege** el caso real del usuario:

| Magnitud | Valor |
|----------|-------|
| FOV X | 162 µm |
| tol UI | 100 µm |
| `2·tol` | 200 µm |
| ¿`162 > 200`? | **No** → coverage siempre `ok` |

Con eso, tras aceptar P₀ con residual hacia P₁ (~80 µm), el residual a P₁ (~82 µm) ≤ tol → **Host-stable accept** de P₁ sin cubrir el FOV.

---

## 2. Corrección 11:50

`TestService._fov_step_coverage_ok`:

```
cov_tol = min(tol_ui, Δ_nominal / 10)
exigir travel ≥ Δ_nominal − cov_tol   (si Δ > cov_tol)
rechazar si aún travel ≤ tol_ui en paso de malla grande
```

Ejemplo FOV=162, tol=100:

| actual vs P₀ | cov_tol | min_travel | ¿acepta P₁? |
|--------------|---------|------------|-------------|
| 80 µm | 16.2 | 145.8 | **No** |
| 155 µm | 16.2 | 145.8 | **Sí** |

Clamp microscopía `tol ≤ FOV_min/10` se mantiene (defensa en profundidad).

---

## 3. KPI / métricas (logs)

| KPI | Meta | String |
|-----|------|--------|
| Tol efectiva | ≤ FOV_min/10 | `Tol. trayectoria clamp` / `Política FOV` |
| `travel` | ≥ `min_travel` ≈ 0.9·FOV | `AUDIT FOV accept … travel=… min_travel=` |
| `cov_ok` | `True` solo con FOV cubierto | mismo |
| Host-stable en malla sin viaje | 0 | `Host-stable BLOQUEADO` |
| ACCEPT sin cobertura | 0 | `ACCEPT DENEGADO` |

Tests: `tests/test_fov_step_coverage.py`

---

## 4. Cómo validar en corrida

1. Tol. cierre UI ≤ ~12 µm (o dejar que microscopía clampee).
2. Buscar en log por punto: `AUDIT FOV accept` con `travel ≈ FOV` y `cov_ok=True`.
3. Verificar que no haya 2–3 `📍 Punto` / `Captura con autofoco` con el mismo XY físico.

---

## 5. Archivos

- `src/core/services/test_service.py` — cobertura FOV
- `src/core/services/microscopy_service.py` — clamp tol
- `src/gui/tabs/test_tab.py` — FOV en params
- `src/config/test_parameters_template.json` — tol 12 µm
- `tests/test_fov_step_coverage.py`
- Este informe + `…_1146.md`

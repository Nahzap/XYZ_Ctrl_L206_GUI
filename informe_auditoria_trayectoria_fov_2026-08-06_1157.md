# Auditoría trayectoria FOV — timeout cobertura + clamp malla

| Campo | Valor |
|-------|--------|
| **Fecha** | 2026-08-06 |
| **Hora** | 11:57 (UTC-4) |
| **Síntoma** | P3 en bucle `sin cobertura FOV`; no avanza >6–10s |
| **Estado** | Corregido |

---

## 1. Diagnóstico (logs usuario)

| Dato | Valor |
|------|-------|
| Tol UI | **250 µm** |
| Paso malla | **320 µm** |
| `min_travel` (cov) | ≈ 288 µm (`320 − 32`) |
| Accept host | residual ≤ 250 µm → travel puede quedar en ~70–200 |

Con tol ≥ paso/2 la banda de accept y la cobertura estricta son **incompatibles**: el settle declara OK y el cover deniega → re-arm cada tick → spam y atasco.

---

## 2. Correcciones

| Fix | Detalle |
|-----|---------|
| Clamp en `TestService` | `tol ← min(tol_UI, paso_malla/10)` al start |
| Watchdog **6 s** | Si no hay cobertura → **accept con error** y avanzar |
| Un solo re-arm | No `_prepare_step_transition` en cada deny (evita reset settle) |
| Log throttled | 1 msg/s con `t=elapsed/6s` |

Status de avance con error: `⚠️ cover t/o 6s travel=…/… err=(…,…)`

---

## 3. KPI

| KPI | Meta |
|-----|------|
| Tiempo cazando cobertura | ≤ 6 s |
| Tras timeout | índice++ / pausa captura (según modo) |
| Tol efectiva | ≤ paso/10 (p.ej. 32 µm si paso=320) |
| Spam UI deny | ≤ 1/s |

Strings: `timeout cobertura`, `ACCEPT FORCE`, `Tol. trayectoria clamp`, `cover t/o`.

---

## 4. Archivos

- `src/core/services/test_service.py`
- `tests/test_fov_step_coverage.py`
- Este informe

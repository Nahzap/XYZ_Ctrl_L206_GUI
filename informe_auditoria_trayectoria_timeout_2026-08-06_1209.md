# Auditoría: timeout de punto + cierre approach

| Campo | Valor |
|-------|--------|
| **Fecha** | 2026-08-06 |
| **Hora** | 12:09 (UTC-4) |
| **Síntoma** | Caza FINE en P1 sin accept; timeout 6s no disparaba |
| **Estado** | Corregido |

---

## 1. Por qué no disparaba el timeout

El watchdog solo actuaba en **denegación de cobertura FOV**.  
En P1 la cobertura siempre es OK → si el approach no hace settle, **nunca** había timeout.

Log: residual oscila ~25–75 µm (tol 32), PWM ±114 (piso stiction) → caza sin accept.

---

## 2. Cambios

| Ítem | Detalle |
|------|---------|
| UI Test | Campo **Timeout punto (s)** (default 6) junto a Holgura/Pausa |
| `start_trajectory(point_timeout_s=…)` | Configurable; log: `Timeout punto: Xs` |
| Watchdog global | approach / handoff / fov_verify → `⚠️ point t/o … — avanzo` |
| FINE cerca de tol | No forzar ±u_run (evita bang ±114) |
| Microscopía | Lee `point_timeout_s` de la misma UI |

---

## 3. Strings de log

- `Timeout punto: 6.0s`
- `point t/o` / `POINT TIMEOUT`
- `Approach FINE H∞ Kp…`

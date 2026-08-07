# Plan: latencia live cámara + memoria + SRP CameraTab

| Campo | Valor |
|-------|--------|
| **Fecha** | 2026-08-06 |
| **Hora** | 09:43 (UTC-4) |
| **Proyecto** | `XYZ_Ctrl_L206_GUI` |
| **Síntoma** | Retraso severo al mostrar frames en vivo |

---

## 1. Hipótesis (priorizadas)

| ID | Hipótesis | Indicador |
|----|-----------|-----------|
| H1 | Convert+copia full-res (~15 MB) + QImage cada frame en worker | `preview_bytes` ≪ `full_bytes`; `last_preview_ms` ↓ |
| H2 | Se construye QImage aunque la ventana esté cerrada | `preview_builds_skipped` > 0 cuando UI oculta |
| H3 | Cola Qt / memoria no liberada a tiempo | `frames_dropped_coalesce`, sin backlog creciente |
| H4 | FPS UI (30) > HW (14) genera trabajo inútil | FPS aplicado ≤ 14 |
| H5 | CameraTab (~2500 LOC) viola SRP y ensucia el path live | LOC del bridge live ≪ tab; tests de unidad del bridge |

## 2. Métricas de avance (KPI)

| KPI | Baseline (audit) | Meta | Cómo medir |
|-----|------------------|------|------------|
| Bytes QImage preview / frame | ~15.1 MB (full) | ≤ ~3.8 MB (≤1280 px ancho) | unit test + log métricas |
| Copias full-res por frame live | 2 (ndarray+QImage) | 1 full + 1 preview chico | código + test |
| Drop coalesce (servicio) | ya existe | se mantiene / log cada 5 s | `frames_dropped` |
| Skip preview sin consumidor | 0 | >0 si ventana oculta | flag + contador |
| FPS default UI | 30 (builder) | 14 (datasheet) | UI builder |
| CameraTab LOC en path live | embebido en tab | bridge dedicado + tests | LOC / pytest |

## 3. Pipeline actual (resumen)

```
BaslerWorker → copy full + QImage full → QueuedConnection
  → CameraService coalesce → DirectConnection
  → CameraTab → CameraWindow (throttle 20 FPS + scale)
```

Cuello de botella: hops **antes** del coalesce (convert + dual copy full-res).

## 4. Plan de implementación

1. Módulo puro `live_preview.py` (scale + QImage + métricas estimadas).
2. Worker: preview downscale; skip QImage si `preview_enabled=False`; full `current_frame` solo para AF/captura.
3. Service: API `set_preview_enabled` + exposición de métricas live.
4. Bridge SRP: `CameraLiveBridge` (frame→ventana + visibilidad→preview flag).
5. Defaults FPS=14, buffer≥2.
6. Tests unitarios (bytes, skip, bridge) + actualizar este plan con resultados.

## 5. Checklist

- [x] Plan fechado
- [x] Preview ≤1280 px + tests
- [x] Skip preview sin ventana
- [x] Bridge SRP + tests
- [x] KPI medidos / plan actualizado

## 6. Resultados (2026-08-06 ~09:50 UTC-4)

| KPI | Baseline | Medido post-fix | Estado |
|-----|----------|-------------------|--------|
| Bytes QImage preview | ~15.09 MB (2590×1942×3) | **~3.68 MB** (1280×959×3) | ✅ ~4.1× menos |
| Copias full-res / frame | 2 (ndarray+QImage full) | 1 full ndarray + 1 preview chico | ✅ |
| Skip preview sin ventana | 0 | `preview_builds_skipped` + flag | ✅ (test bridge) |
| FPS default UI | 30 | **14** | ✅ |
| Buffer default UI | 1 | **2** | ✅ |
| SRP path live | embebido en CameraTab | `CameraLiveBridge` + tests | ✅ parcial |
| Tests | — | `8 passed` (`test_live_preview_latency` + coalesce) | ✅ |

### Archivos

- `src/hardware/camera/live_preview.py` — downscale + métricas
- `src/hardware/camera/basler_worker.py` — preview/skip + logs KPI
- `src/core/services/camera_service.py` — `set_preview_enabled` / `get_live_metrics`
- `src/gui/tabs/camera_live_bridge.py` — SRP live
- `src/gui/tabs/camera_tab.py` — delega frames al bridge
- `tests/test_live_preview_latency.py`

### Pendiente (siguiente iteración)

- Extraer Microscopy/C-Focus/Capture de CameraTab (god-object restante ~2.5k LOC).
- Medición en hardware real: revisar log `[BaslerWorker] Live #N … full=…B preview=…B`.

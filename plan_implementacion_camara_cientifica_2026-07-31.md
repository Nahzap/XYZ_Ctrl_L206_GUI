# Plan de implementación: cámara científica rigurosa (Basler acA2500-14uc)

**Inicio:** 2026-07-31 00:34:43 (hora local)  
**Referencia de diagnóstico:** `informe_camara_uso_calibrado_2026-07-31_0028.md`  
**Documento vivo de progreso:** sección 6 (actualizar tras cada fase)

---

## 1. Objetivo

Llevar el software a prácticas estandarizadas y científicamente rigurosas para adquisición con la **Basler acA2500-14uc**, validables **sin hardware de laboratorio** mediante:

- Lógica pura (datasheet, normalización, auditoría)
- Aplicación GenICam mockeable
- Tests unitarios automatizados
- Indicadores cuantitativos de conformidad

---

## 2. Principios (estándar)

| ID | Principio | Fuente |
|---|---|---|
| P1 | Parámetros de adquisición fijos y documentados (exposición, ganancia, FPS) | Basler Optimum Color / machine vision |
| P2 | AutoExposure / AutoGain Off en producción | Basler Gain Auto / User Sets |
| P3 | FPS ≤ límite de datasheet (14 fps @ full frame) | docs.baslerweb.com/aca2500-14uc |
| P4 | Preferir profundidad nativa 12-bit; no fingir 16-bit desde BGR8 | Pixel Format Basler |
| P5 | Resolución nativa por defecto en microscopía | Datasheet 2590×1942 |
| P6 | Configuración UI ↔ hardware coherente | Buenas prácticas de trazabilidad |
| P7 | Separar preview (8-bit) de buffer científico (raw/12-bit) | Separación display/medida |
| P8 | `calibration.json` (XYZ) ≠ calibración óptica/color de cámara | Claridad metrológica |

---

## 3. Fases de implementación

### Fase A — Núcleo puro + auditoría (sin pylon)

| Entrega | Descripción | Criterio de aceptación |
|---|---|---|
| A1 | Módulo `hardware/camera/scientific_config.py` | Datasheet + normalización + auditoría |
| A2 | Tests unitarios sin cámara | ≥ 20 asserts; pytest verde |
| A3 | Defaults JSON alineados (FPS≤14, exposición unificada, res nativa) | Plantilla conforme |

### Fase B — Integración BaslerWorker

| Entrega | Descripción | Criterio de aceptación |
|---|---|---|
| B1 | Cargar UserSet Default (si existe) | Mock aplica `UserSetLoad` |
| B2 | Desactivar GainAuto / ExposureAuto | Nodos Off en mock |
| B3 | Clamp FPS, gain 0 dB, binning 1×1, buffer documentado | Settings resultan en perfil científico |
| B4 | Dual buffer: `current_frame` preview + `current_raw_frame` científico | Tests de grab simulado |

### Fase C — Captura honesta (use_16bit)

| Entrega | Descripción | Criterio de aceptación |
|---|---|---|
| C1 | Resolver frame de guardado sin upscale engañoso 8→16 | Metadata `synthetic=False` solo si hay raw uint16 |
| C2 | Alineación MSB 12→16 para PNG/TIFF | Rango y dtype verificados |
| C3 | Advertencia explícita si no hay profundidad nativa | Flag `warning` no vacío |

### Fase D — Lab (fuera de este sprint offline)

| Entrega | Descripción | Criterio |
|---|---|---|
| D1 | Validar con pylon + cámara real | Smoke test hardware |
| D2 | Color Calibrator / `.pfs` si se requiere colorimetría | Delta E / checklist lab |
| D3 | µm/píxel óptico documentado | Campo en config óptica |

---

## 4. Indicadores y métricas

### 4.1 Conformidad de configuración (score 0–100)

```
score = 100 * (n_checks_ok / n_checks_total)
```

Checks (peso unitario salvo indicación):

| Check ID | Descripción | Peso |
|---|---|---|
| CHK_MODEL | Modelo = acA2500-14uc | 1 |
| CHK_FPS | fps ≤ 14 | 2 |
| CHK_EXP_UNIFIED | Exposición `camera` == `camera_tab` | 2 |
| CHK_GAIN0 | gain_db = 0 | 1 |
| CHK_AUTO_OFF | autos exposición/ganancia Off | 2 |
| CHK_NATIVE_RES | microscopía width/height nativos o 0 | 2 |
| CHK_BITDEPTH_HONEST | use_16bit implica raw uint16 disponible | 3 |
| CHK_BUFFER | buffer documentado y ≥ 1 | 1 |
| CHK_PIXEL_FMT | PixelFormat preferido 12-bit disponible/seleccionado | 2 |

**Umbrales:**

| Score | Estado |
|---|---|
| ≥ 90 | Listo para lab (fase D) |
| 70–89 | Aceptable offline; gaps documentados |
| < 70 | No conforme |

### 4.2 Métricas de proceso de software

| Métrica | Definición | Meta |
|---|---|---|
| M_TESTS | Nº tests unitarios del módulo científico | ≥ 15 |
| M_PASS | % tests pasando | 100 % |
| M_COV_FUNC | Funciones públicas del módulo con ≥1 test | 100 % |
| M_PHASE | Fases A–C cerradas | 3/3 |
| M_WARN | Hallazgos `severity`/`error` en auditoría de plantilla | 0 error; ≤ 2 warning documentados |

### 4.3 Métricas de integridad de imagen (offline)

| Métrica | Definición | Meta |
|---|---|---|
| I_DTYPE | dtype del frame científico mock Bayer12 | uint16 |
| I_MSB | max(frame_msb) ≥ 16× max(frame_12) aproximado por shift | shift = 4 (12→16 MSB) |
| I_NO_FAKE16 | save con use_16bit sin raw → no inventa uint16 silencioso | warning emitido |
| I_PREV8 | preview demosaicado/convertido queda uint8 | dtype uint8 |

---

## 5. Orden de trabajo de este sprint

1. Crear módulo puro + tests  
2. Actualizar plantilla JSON  
3. Integrar BaslerWorker (perfil científico + dual buffer)  
4. Helper de guardado honesto  
5. Ejecutar pytest y volcar métricas en §6  

---

## 6. Registro de progreso (actualizar)

| Timestamp | Fase | Acción | Score | M_TESTS | M_PASS | Notas |
|---|---|---|---|---|---|---|
| 2026-07-31 00:34:43 | — | Plan creado | — | 0 | — | Baseline post-informe |
| 2026-07-31 00:37:49 | A–C | Implementación + pytest | **100.0** | **22** | **100 %** | Score plantilla con `has_raw_uint16_path=True` |

### Baseline (antes del sprint)

Auditoría automática sobre config legacy (informe):

| Check | Baseline | Post-sprint (plantilla) |
|---|---|---|
| CHK_MODEL | OK | OK |
| CHK_FPS | FAIL (30) | OK (14) |
| CHK_EXP_UNIFIED | FAIL (0.015 vs 0.15) | OK (0.015) |
| CHK_GAIN0 | OK (implícito) | OK (0.0 declarado) |
| CHK_AUTO_OFF | FAIL | OK (declarado + apply Off) |
| CHK_NATIVE_RES | FAIL (1920×1080) | OK (2590×1942) |
| CHK_BITDEPTH_HONEST | FAIL | OK (raw path + preserve) |
| CHK_BUFFER | WARN (1 vs 5) | OK (5) |
| CHK_PIXEL_FMT | parcial | OK (BayerGB12 first) |

| Métrica | Baseline | Actual | Meta | Estado |
|---|---|---|---|---|
| Score conformidad | **18.75** / 100 (3/9 checks) | **100.0** / 100 (9/9) | ≥ 90 | Cumple |
| M_TESTS | 0 | 22 | ≥ 15 | Cumple |
| M_PASS | — | 100 % (22/22) | 100 % | Cumple |
| M_COV_FUNC (APIs públicas ejercitadas) | 0 % | 100 % smoke | 100 % | Cumple |
| M_PHASE A–C | 0/3 | **3/3** | 3/3 | Cumple |
| M_WARN (plantilla) | varios error | 0 error / 0 warning | 0 error | Cumple |
| I_DTYPE (mock Bayer12) | n/a | uint16 | uint16 | Cumple |
| I_MSB (shift 4) | n/a | verificado en test | shift=4 | Cumple |
| I_NO_FAKE16 | fallaba en diseño | warning obligatorio | warning | Cumple |
| Delta score | — | **+81.25 pts** (100 − 18.75) | — | — |

Comando de verificación offline:

```bash
.\CTRL_ENV\python.exe -m pytest tests/test_scientific_camera_config.py -v
```

### Pendiente fase D (laboratorio)

| Item | Estado |
|---|---|
| Smoke pylon + cámara real | Pendiente |
| Color Calibrator / `.pfs` si colorimetría | Pendiente |
| µm/píxel óptico documentado en montaje | Pendiente |

---

## 7. Riesgos

| Riesgo | Mitigación |
|---|---|
| Detección U2-Net espera BGR8 | Mantener `current_frame` como preview; raw aparte |
| Packed BayerGB12p difícil offline | Preferir BayerGB12 unpacked en perfil |
| UserSet Load solo idle | Aplicar antes de StartGrabbing |
| Lab no disponible | Todo A–C mockeado; D diferido |
| Canal G solo + use_16bit | No se usa Bayer raw (evita mentir); warning 8-bit | Mitigado en `CameraService` |

---

## 8. Entregables de este sprint

- [x] Este plan con indicadores  
- [x] `src/hardware/camera/scientific_config.py`  
- [x] Integración en `basler_worker.py`  
- [x] Defaults en `test_parameters_template.json`  
- [x] Captura honesta en `camera_service.py` (`resolve_save_frame`)  
- [x] `tests/test_scientific_camera_config.py` (22 passed)  
- [x] Actualización §6 con métricas reales de pytest  

## 9. Archivos tocados

| Archivo | Rol |
|---|---|
| `plan_implementacion_camara_cientifica_2026-07-31.md` | Plan + métricas |
| `src/hardware/camera/scientific_config.py` | Núcleo científico |
| `src/hardware/camera/basler_worker.py` | Perfil + dual buffer |
| `src/hardware/camera/__init__.py` | Exports |
| `src/core/services/camera_service.py` | Guardado honesto |
| `src/config/test_parameters_template.json` | Defaults conformes |
| `tests/test_scientific_camera_config.py` | Tests sin hardware |

# Informe: uso calibrado de la cámara Basler acA2500-14uc

**Fecha y hora:** 2026-07-31 00:28:04 (hora local)  
**Proyecto:** XYZ_Ctrl_L206_GUI  
**Alcance:** Verificar si el modelo declarado y la configuración actual corresponden a un **uso calibrado** según documentación oficial Basler y el código del repositorio.

---

## 1. Veredicto

| Pregunta | Respuesta |
|---|---|
| ¿El modelo del proyecto coincide con el hardware documentado? | **Sí** — Basler ace Classic **acA2500-14uc** |
| ¿La configuración actual corresponde a un uso calibrado (fábrica / colorimétrico / profundidad científica)? | **No** |
| ¿Hay partes razonables para visión artificial / microscopía de detección? | **Parcialmente** (ganancia 0 dB, resolución máxima, Bayer 12-bit en sensor), pero se pierden en el pipeline de software |

**Conclusión:** el modelo está bien identificado y varias decisiones del worker son coherentes con el datasheet; la configuración persistida y el pipeline de captura **no** implementan el flujo de “colores calibrados” ni el de “datos crudos de alta profundidad” que Basler describe para uso calibrado o de mínimo ruido.

---

## 2. Modelo declarado en el proyecto

| Fuente en el repo | Valor |
|---|---|
| `src/config/test_parameters_template.json` → `camera.model` | `Basler acA2500-14uc` |
| `src/hardware/camera/basler_worker.py` (cabecera) | Basler acA2500-14uc, 2590×1942, 12-bit, 14 fps, USB 3.0 |

Al conectar, el worker lee `GetModelName()` / `GetSerialNumber()` del dispositivo real (no hardcodea el S/N en config; `serial_number` está vacío en la plantilla).

---

## 3. Especificaciones oficiales del modelo

Fuentes principales:

- [Documentación de producto acA2500-14uc (Basler)](https://docs.baslerweb.com/aca2500-14uc)
- [Ficha de producto / tienda Basler](https://www.baslerweb.com/en/shop/aca2500-14uc/)
- [Pixel Format (Basler)](https://docs.baslerweb.com/pixel-format)
- [User Sets (Basler)](https://docs.baslerweb.com/user-sets)
- [Achieving Optimum Color Reproduction… (Basler Knowledge)](https://docs.baslerweb.com/knowledge/achieving-optimum-color-reproduction-in-different-use-cases)
- [Color Calibrator (Basler)](https://docs.baslerweb.com/color-calibrator)
- [Electronic Shutter Types / Rolling Shutter (Basler)](https://docs.baslerweb.com/electronic-shutter-types)

| Parámetro | Oficial Basler | Comentario en el proyecto |
|---|---|---|
| Resolución | 2590 × 1942 (5 MP) | Coincide |
| Sensor | onsemi MT9P031, CMOS, **rolling shutter** | Implicado (no documentado en config) |
| Formato óptico | 1/2.5", diagonal efectiva ~7.2 mm | No usado en config |
| Tamaño de píxel | **2.2 × 2.2 µm** | No usado en config óptica |
| Color / mono | Color (visible), filtro IR cut | Coincide (color) |
| Frame rate @ defaults | **14 fps** | Comentario del worker: 14 fps; **config UI/plantilla: 30 fps** |
| Interfaz | USB 3.0 | Coincide |
| Profundidad de píxel | hasta **12 bits** | Comentario: 12-bit nativo |
| Formatos de píxel | Mono8, BayerGB8, BayerGB12, BayerGB12p, YCbCr422_8 | Worker prioriza BayerGB12 / BayerGB12p / BayerGB8 |
| User sets de fábrica (este modelo) | UserSet1–3, **Default**, **HighGain**, **AutoFunctions** | **No** incluye `Color` ni `ColorRaw` |

---

## 4. Configuración actual en el proyecto

### 4.1 Parámetros persistidos (`test_parameters_template.json`)

**Bloque `camera`:**

| Parámetro | Valor actual |
|---|---|
| `frame_rate` | **30** FPS |
| `exposure` | **0.015** s (15 ms) |
| `buffer_size` | **1** frame |

**Bloque `camera_tab` (lo que restaura la UI):**

| Parámetro | Valor actual |
|---|---|
| `exposure` | **0.15** s (150 ms) — distinto del bloque `camera` |
| `fps` | **30** |
| `buffer_frames` | **1** |
| `use_16bit` | **true** |
| `image_format` | PNG |
| Microscopía `img_width` × `img_height` | **1920 × 1080** |
| Canales | solo **G** (verde) |

### 4.2 Lo que realmente aplica `BaslerWorker._configure_camera()`

| Acción en código | Valor / comportamiento |
|---|---|
| `Width` / `Height` | Máximo del sensor (2590×1942) |
| `PixelFormat` | Prioridad BayerGB12 → BayerGB12p → BayerGB8 → … |
| `ExposureTime` | `self.exposure` convertida a µs (default base del worker: 0.02 s hasta que la UI aplique otra) |
| `Gain` | **0.0 dB** (si el nodo existe) |
| `AcquisitionFrameRateEnable` | True; FPS = `min(self.fps, max_fps)` |
| `MaxNumBuffer` | **5** al conectar (ignora el “1” de la plantilla hasta un cambio posterior) |
| Binning | 1×1 |
| Conversor pylon | **`BGR8packed`** + `MsbAligned` en **todos** los frames live/captura |
| User set / LightSourcePreset / BalanceWhite | **No se cargan ni configuran** |
| GainAuto / ExposureAuto | **No se desactivan explícitamente** |

### 4.3 Pipeline de bits (punto crítico)

Aunque la UI marca **16-bit**, el worker convierte cada grab a **BGR8** (`uint8`, 3 canales) vía `ImageFormatConverter`.  
Por tanto, `current_frame` en la ruta Basler es **8-bit demosaicado**, no el Bayer12 nativo del sensor. Las ramas de código que asumen `uint16` (microscopía, z-stack, métricas de foco) **no reciben profundidad 12/16-bit real** desde Basler.

Además, en microscopía se puede **redimensionar** de 2590×1942 → 1920×1080 (`INTER_LINEAR`), lo que degrada resolución espacial nativa.

> Nota: `calibration.json` del repo calibra **ejes XYZ / ADC→µm**, no colorimetría ni escala óptica µm/píxel de la cámara.

---

## 5. Qué entiende Basler por “uso calibrado”

Según el artículo oficial *[Achieving Optimum Color Reproduction in Different Use Cases](https://docs.baslerweb.com/knowledge/achieving-optimum-color-reproduction-in-different-use-cases)*:

### Caso 1 — Colores calibrados (mínimo error colorimétrico)

1. Las cámaras Basler salen de fábrica con **colores calibrados de fábrica**.
2. Seleccionar un **Light Source Preset** adecuado y hacer **balance de blancos**.
3. Si hace falta más precisión: **Color Calibrator** en pylon Viewer con ColorChecker Classic, bajo la misma iluminación/óptica del uso final; guardar preset / parámetros.

### Caso 2 — Mínimo ruido (precisión de color no crítica; típico machine vision)

- En ace Classic / ace U: cargar user set **`ColorRaw`** (datos crudos, sin gamma de sRGB).
- **Importante para este modelo:** en la tabla oficial de User Sets, **acA2500-14uc no lista `Color` ni `ColorRaw`**. Solo Default / HighGain / AutoFunctions (+ UserSet1–3).  
  El equivalente práctico es trabajar en **Bayer12 crudo** y demosaicar/procesar en host de forma controlada, o quedarse en Default con autos apagados y ganancia mínima.

### Buenas prácticas asociadas (docs Basler)

- Para medición / reproducibilidad: **Gain Auto / Exposure Auto en Off** (evitar user set AutoFunctions en producción calibrada).
- Gain bajo (el proyecto ya apunta a 0 dB) → menos ruido.
- Bayer12 aprovecha la profundidad nativa; convertir a BGR8 **antes** de guardar destruye ese beneficio.
- Rolling shutter (MT9P031): con escena **estática** no hay distorsión; con movimiento (etapa XY, vibración, exposición larga) sí puede haber artefacto. Basler recomienda flash sincronizado o escena quieta; sensores lentos (~14 fps) son más sensibles al efecto que sensores rápidos.

---

## 6. Matriz de conformidad

| Criterio de uso calibrado / datasheet | Estado en el proyecto | Severidad |
|---|---|---|
| Modelo correcto (acA2500-14uc) | Cumple | — |
| Resolución nativa 2590×1942 en adquisición | Cumple en worker | — |
| FPS ≤ 14 (límite oficial) | **No** — plantilla/UI piden 30; se clampea al máx. disponible | Media (confusión operativa) |
| Exposición coherente y fija | **No** — 0.015 s vs 0.15 s según bloque; default worker 0.02 s | Media |
| Buffer coherente UI ↔ hardware | **No** — UI/plantilla=1, connect fuerza 5 | Baja |
| Profundidad 12-bit preservada hasta archivo | **No** — conversión forzada a BGR8 | **Alta** |
| Flag `use_16bit=true` refleja realidad Basler | **No** — engañoso en esta ruta | **Alta** |
| Light Source Preset + white balance (caso calibrado color) | **Ausente** | Alta si importa color |
| Color Calibrator / archivo `.pfs` de sesión | **Ausente** | Alta si importa colorimetría |
| User set Default / ColorRaw explícito | **Ausente**; además ColorRaw **no existe** en este SKU | Media |
| GainAuto / ExposureAuto Off explícito | **Ausente** | Media |
| Gain = 0 dB | Cumple (intento en código) | — |
| Microscopía a resolución nativa | **No** — default 1920×1080 | Media–Alta |
| Escala óptica µm/píxel (píxel sensor 2.2 µm × aumento) | **No documentada** en config de cámara | Depende de metrología visual |
| Rolling shutter + waits de asentamiento FOV | Parcialmente mitigado por lógica de settle del sistema; exposición 150 ms aumenta blur si hay movimiento residual | Media |

---

## 7. Interpretación para este laboratorio

El uso dominante del proyecto (detección U2-Net / polen, autofoco, canal G, análisis algorítmico) se acerca más al **Caso 2 de Basler (machine vision, mínimo ruido)** que al **Caso 1 (colorimetría calibrada sRGB)**.

Eso implica:

1. **No es necesario** un ColorChecker completo si el color absoluto no se usa para decisión.
2. **Sí es necesario**, para un uso “calibrado” en sentido científico/metrológico de imagen:
   - Exposición y ganancia **fijas y documentadas**
   - Autos de exposición/ganancia **Off**
   - Preservar **Bayer12** (o al menos 12/16-bit) si se marca 16-bit
   - Capturar a **resolución nativa** si se cuantifica morfología en píxeles
   - FPS de UI ≤ **14**
   - Escena **quieta** al disparar (rolling shutter + exposición larga)

Hoy el sistema **no cumple** ese paquete de forma consistente.

---

## 8. Recomendaciones priorizadas

1. **Alinear FPS** en plantilla/UI a **≤ 14** (valor nominal del datasheet).
2. **Unificar exposición** (`camera` vs `camera_tab`) y fijar un valor de trabajo documentado.
3. Si se mantiene `use_16bit=true`: dejar de convertir a BGR8 en la ruta de captura; guardar Bayer12 o RGB/BGR de 16 bits alineados; demosaicar solo para preview.
4. En `_configure_camera()`: desactivar explícitamente `GainAuto` y `ExposureAuto`; opcionalmente cargar `UserSet` = `Default` al inicio.
5. Microscopía: por defecto `img_width`/`img_height` = resolución nativa (o “0 = nativo”), no 1080p.
6. Si más adelante importa color fiel: calibrar con **Color Calibrator** + ColorChecker bajo la iluminación real; guardar `.pfs` / UserSet1 y cargarlo al conectar.
7. Documentar en config el **µm/píxel** del montaje óptico (no confundir con `calibration.json` de motores).
8. Para FOV/captura: confirmar asentamiento mecánico antes del grab; con rolling shutter + 150 ms, cualquier micro-movimiento degrada nitidez.

---

## 9. Fuentes consultadas

1. Basler Product Documentation — [acA2500-14uc](https://docs.baslerweb.com/aca2500-14uc)  
2. Basler — [ace acA2500-14uc product page](https://www.baslerweb.com/en/shop/aca2500-14uc/)  
3. Basler — [Pixel Format](https://docs.baslerweb.com/pixel-format) (formatos del acA2500-14uc)  
4. Basler — [User Sets](https://docs.baslerweb.com/user-sets) (sets disponibles por modelo)  
5. Basler Knowledge — [Optimum Color Reproduction use cases](https://docs.baslerweb.com/knowledge/achieving-optimum-color-reproduction-in-different-use-cases)  
6. Basler — [Color Calibrator](https://docs.baslerweb.com/color-calibrator)  
7. Basler — [Gain Auto](https://docs.baslerweb.com/gain-auto)  
8. Basler — [Electronic Shutter Types](https://docs.baslerweb.com/electronic-shutter-types)  
9. Basler Learning — [CMOS Rolling Shutter Cameras](https://www.baslerweb.com/en/learning/cmos-rolling-shutter-cameras/)  
10. Código del repo: `basler_worker.py`, `camera_service.py`, `test_parameters_template.json`, `calibration.json`

---

*Informe generado a partir del estado del repositorio en la fecha indicada y de la documentación pública Basler citada arriba.*

# Plan: autofoco multifocal WYSIWYG (preview = PNG) sin interrupción

| Campo | Valor |
|-------|--------|
| **Fecha inicio** | 2026-08-06 |
| **Hora inicio** | 16:14 (UTC-4) |
| **Proyecto** | `XYZ_Ctrl_L206_GUI` |
| **Estado** | Implementado en código (pendiente validación en vivo) |
| **Última actualización** | 2026-08-06 16:16 (UTC-4) |
| **Evidencia fallo** | Log 16:12:12–16:12:33 — `S_ROI COARSE` todo `S=0.00` → `float division by zero` → 0 imágenes |

---

## 1. Objetivo (contrato)

1. **N planos multifocales** (GUI, impar) + `*_focus.json` + `*_position.json` en cada punto con objeto.
2. **Color WYSIWYG**: PNG ≡ preview Basler (mismos píxeles BGR8 pylon+WB empaquetados a uint16 MSB).
3. **El autofoco no se aborta** por fallos internos evitables (S=0 por flush mal cableado, división por cero, fallback silencioso a 1 PNG).

```
Detección → AF COARSE→FINE→BPoF→N fotos (ΔS%) → guardar f0..fN-1 + focus.json + position.json
                                                              ↑
                                                    color = preview
```

---

## 2. Causa raíz del log 16:12

| Síntoma | Causa |
|---------|--------|
| Todos los `S_ROI COARSE` = 0.00 | `_wait_optical_static` espera avance de `current_raw_frame_count` (solo crece al `acquire_scientific_frame`). En el flush **no** se adquiere → timeout (~2 s/plano) → MEASURE no corre → S=0 |
| `float division by zero` | Early-stop COARSE / ratios con `coarse_best_s == 0` |
| `ERROR: Fallo guardar imagen` / 0 PNG | AF lanza excepción → `results=[]` → gate multifocal rechaza stack vacío (correcto tras el bug; no debe ocurrir si S es válido) |

**Fix primario:** `get_frame_count_callback` = contador de grabs **live** (`frame_count`), nunca el id científico.

**Fix secundario:** protecciones `/0` y rechazo claro si toda la tabla COARSE es S=0 (reintento / error explícito, no crash).

---

## 3. Checklist de implementación

- [x] Gate microscopía usa `acquire_scientific_frame_callback` (no `get_frame_callback` legacy)
- [x] Basler WYSIWYG: `scientific_frame_from_preview_bgr8` (preview→MSB16)
- [x] `main.initialize_autofocus`: `get_frame_count_callback` → `worker.frame_count`
- [x] Guardas división por cero en COARSE early-stop y ratios FINE
- [x] Si COARSE todo S≤0: `RuntimeError` explícito (no `float division by zero`)
- [x] OPTICAL timeout + Z estático → medir igual (AF no se interrumpe)
- [x] Tests: frame_count live; early-stop S=0; WYSIWYG preview≡PNG
- [ ] Validación en vivo: 3 PNG `_f*`, `focus.json`, `position.json`, color = preview

---

## 4. No negociable

- No interrumpir el ciclo AF por timeouts ópticos falsos (flush mal cableado).
- No guardar 1 PNG “de consolación” cuando `n_captures ≥ 3` falló el stack.
- No demosaic OpenCV distinto del preview para Basler (color ridículo).

---

## 5. Validación esperada (post-fix)

En log, por objeto:

```
S_ROI COARSE k/N: Z=… S=<positivo>
… BPoF …
FOTO 1/3 … FOTO 2/3 … FOTO 3/3 …
✓ N imagen(es) + posición guardadas
```

En disco:

```
*_f0.png  *_f1.png  *_f2.png  *_focus.json  *_position.json
```

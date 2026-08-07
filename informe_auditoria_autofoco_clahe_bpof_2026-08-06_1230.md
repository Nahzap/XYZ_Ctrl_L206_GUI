# Auditoría y corrección: S CLAHE-HF para BPoF

**Fecha:** 2026-08-06  
**Hora:** 12:30 (UTC-4)  
**Follow-up:** 12:32 (UTC-4), carrera de publicación de frame corregida.  
**Estado:** implementación terminada; tests sintéticos aprobados; falta prueba física del usuario.

## 1. Problema observado

Los logs mostraban una inconsistencia relevante:

- En coarse, `Z=40.04 µm` obtuvo `S=583.6`.
- En fine, al volver alrededor de la misma Z, los valores fueron aproximadamente
  `S=491`.
- El máximo de una sola imagen podía dominar el `argmax`, aunque no representara
  detalle óptico estable.

Esto impedía confiar en que el plano seleccionado fuera realmente el Best Plane
of Focus dentro de la ROI.

## 2. Causas encontradas

### CLAHE no participaba en S

CLAHE se aplicaba durante la detección morfológica, pero la función real usada
por `AutofocusService` para medir cada plano (`focus_metric.py`) evaluaba el
frame sin CLAHE.

### La métrica eliminaba alta frecuencia antes de medirla

Antes de Laplacian/Tenengrad se aplicaba `GaussianBlur(3×3)`. Eso atenuaba
precisamente el detalle fino que debe crecer al acercarse al BPoF.

Además, Laplacian usaba `ksize=5`, menos localizado para las texturas pequeñas
de las semillas.

### El contraste de baja frecuencia contaminaba el resultado

La fórmula incluía `var(gray)/mean`. Ese término puede crecer por iluminación,
sombras o contraste global sin que exista mejor enfoque.

### El borde del contorno podía dominar

Los operadores derivativos se evaluaban hasta el borde de segmentación. La
silueta externa puede producir una respuesta grande y casi constante mientras
el detalle interno sigue desenfocado.

### Un único frame decidía S

Cada Z se añadía a la tabla usando una sola imagen. Ruido, exposición o un frame
atípico podían crear un máximo coarse/fine aislado.

### Carrera `frame_count/current_frame`

Los workers de cámara incrementaban `frame_count` **antes** de copiar el nuevo
frame a `current_frame`. Autofocus esperaba ese contador y podía leer la imagen
anterior creyendo que era nueva. Esto explica directamente diferencias coarse
vs fine al volver al mismo Z.

Se corrigió el orden en Basler, Thorlabs y el worker genérico:

```text
publicar current_frame → incrementar frame_count
```

## 3. Algoritmo implementado

Para cada ROI/contorno fijo:

1. Convertir a gris uint8 con escala reproducible.
2. Aislar el histograma del fondo exterior al contorno.
3. Erosionar ligeramente la máscara para excluir el borde de segmentación.
4. Aplicar **CLAHE** (`clipLimit=2`, grilla `8×8`) dentro de la región.
5. Medir alta frecuencia sin Gaussian previo:
   - Tenengrad/Sobel;
   - varianza de Laplacian `ksize=3`;
   - energía Brenner a 2 píxeles;
   - varianza del residual high-pass/DoG.
6. Fusionar componentes:

```text
S = 0.45·sqrt(Tenengrad)
  + 0.25·sqrt(var(Laplacian))
  + 0.20·sqrt(Brenner)
  + 0.10·sqrt(var(HighPass))
```

7. Tomar al menos **3 frames nuevos y estáticos por Z**.
8. Guardar en la tabla la **mediana S**, no una muestra aislada.
9. BPoF continúa siendo el `argmax` de la tabla fine, ahora sobre mediciones
   CLAHE-HF estables.
10. El contador de cámara solo se publica después de que el frame asociado está
    disponible.

Para múltiples ROIs:

```text
S_plano = Σ S_ROI_i
```

Cada `S_ROI_i` es un promedio/energía normalizada por sus píxeles; una ROI grande
no gana únicamente por tener más área.

## 4. Indicadores nuevos en logs

El inicio del barrido debe mostrar:

```text
S=ΣS_i CLAHE-HF-v2 mediana×3
```

En debug, cada ROI reporta:

```text
S=... CLAHE-HF-v2 | Ten=... Lap=... Brenner=... HP=...
```

La medición por plano reporta:

```text
S estable Z=...: mediana=... n=... min=... max=...
```

## 5. Pruebas

Ejecutadas:

```text
pytest tests/test_focus_metric_clahe.py
       tests/test_autofocus_score_sampling.py
       tests/test_bpof_candidates.py
```

Resultado: **14 tests aprobados**.

Cobertura:

- S decrece monótonamente al aumentar desenfoque.
- Alta frecuencia fuera del contorno no altera S.
- CLAHE reduce dependencia de iluminación global.
- Ruta uint16 preserva el orden de foco.
- Erosión elimina sesgo del borde.
- Una muestra espuria no domina el plano: se usa mediana.
- `argmax_fine` encuentra el BPoF sintético correcto.

Curva sintética obtenida (`Z=0` es el plano enfocado):

```text
Z: -4   -3    -2     -1      0      +1     +2    +3   +4
S: 41.7 72.4 113.6  148.1  169.9  148.1  113.6 72.4 41.7
```

Máximo correcto: `BPoF = Z 0`.

## 6. Archivos modificados

- `src/core/autofocus/focus_metric.py`
- `src/core/services/autofocus_service.py`
- `src/hardware/camera/basler_worker.py`
- `src/hardware/camera/thorlabs_worker.py`
- `src/hardware/camera/camera_worker.py`
- `tests/test_focus_metric_clahe.py`
- `tests/test_autofocus_score_sampling.py`

## 7. Referencias de investigación

- *Quantitative Evaluation of Focus Measure Operators in Optical Microscopy*,
  Sensors 2025: Tenengrad presenta un buen equilibrio entre sensibilidad y
  robustez al ruido.  
  https://www.mdpi.com/1424-8220/25/10/3144
- OpenCV, comparación de focus measures: Tenengrad y Laplacian identifican con
  fiabilidad los frames nítidos; las métricas estadísticas simples son menos
  fiables.  
  https://opencv.org/autofocus-using-opencv-a-comparative-study-of-focus-measures-for-sharpness-assessment/
- Masetti et al., 2023: Laplacian dentro de regiones específicas como estimador
  rápido de contenido espacial de alta frecuencia.  
  https://strathprints.strath.ac.uk/86297/

## 8. Criterio para la prueba física

La siguiente corrida debe cumplir:

- Coarse y fine cerca de la misma Z producen valores S comparables.
- La curva fine presenta un máximo local claro, no un pico aislado.
- Los componentes Ten/Lap/Brenner/HP crecen alrededor del BPoF.
- Tras aparcar, `Z_read` coincide con el BPoF seleccionado dentro de la
  tolerancia Z configurada.


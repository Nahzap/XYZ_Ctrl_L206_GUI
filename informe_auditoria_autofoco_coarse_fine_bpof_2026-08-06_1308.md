# Auditoría: encadenamiento COARSE → FINE → BPoF → stack

**Fecha:** 2026-08-06  
**Hora:** 13:08 (UTC-4)

## Hallazgos

1. `z_step_fine` se sincronizaba desde la GUI, pero no participaba en la
   construcción de los planos FINE. El algoritmo usaba solamente `±Δ` y
   `n_fine_planes`.
2. En escaneo local, el rango COARSE se calculaba alrededor de la posición Z
   anterior y recién después se ordenaba volver al origen calibrado. Por eso
   distintas semillas podían iniciar con referencias físicas diferentes.
3. Un fallo al llegar al origen calibrado o al `Z_coarse*` permitía continuar el
   proceso, debilitando el contrato entre fases.
4. El límite interno de 100 iteraciones podía truncar las 101 capas permitidas
   por la GUI de forma asimétrica.
5. FINE recibía como límites el intervalo del COARSE local. Si `Z_coarse*`
   quedaba en un borde, la ventana FINE podía reducirse o colapsar, aunque el
   C-Focus tuviera recorrido disponible.
6. Los mensajes no mostraban de forma inequívoca qué resultado alimentaba la
   fase siguiente.

## Contrato corregido

```text
origen Z calibrado verificado
  → COARSE sobre el rango resuelto desde ese origen
  → Z_coarse* = argmax(S_coarse)
  → reanclaje verificado en Z_coarse*
  → FINE centrado en Z_coarse*
  → BPoF = argmax(S_fine)
  → n capturas GUI centradas en BPoF con z_step_capture
```

Para FINE:

```text
h_solicitado = z_step_fine · (N_fine − 1) / 2
h_efectivo   = min(h_solicitado, Δ_max_GUI, margen_hardware)
paso_efectivo = h_efectivo / ((N_fine − 1) / 2)
Z_fine(k) = Z_coarse* + k · paso_efectivo
```

Así, `z_step_fine` y `N_fine` son parámetros reales del recorrido. `Δ` queda
como límite máximo de seguridad, no como sustituto silencioso del paso.

## Correcciones aplicadas

- El origen calibrado se alcanza antes de resolver el rango COARSE local.
- Se aborta el autofoco si no se verifica el origen o el reanclaje en
  `Z_coarse*`; FINE ya no puede ejecutarse con una referencia incierta.
- FINE recibe explícitamente `Z_coarse*`, `z_step_fine`, `N_fine` y `Δ_max`.
- FINE usa los límites calibrados del hardware, no los bordes del scan COARSE.
- El primer candidato FINE es exactamente `Z_coarse*`.
- La reducción por límite de iteraciones conserva centro, simetría y N impar.
- El límite interno FINE se alinea con las 101 capas máximas de la GUI.
- Se valida `z_step_fine < z_step_coarse`.
- La interfaz muestra el semirango FINE real, paso, N y el enlace al stack.
- Se agregaron trazas `ENLACE COARSE→FINE` y
  `ENLACE FINE→BPoF→STACK`.

## Verificación

- 27 pruebas del encadenamiento, candidatos BPoF, métrica CLAHE-HF-v2,
  muestreo robusto y configuración: **27/27 correctas**.
- 10 pruebas adicionales de autoridad BPoF, guardado y captura multifocal:
  **10/10 correctas**.
- Compilación de los seis módulos modificados: **correcta**.

Pruebas nuevas:

- el rango COARSE local se resuelve después de volver al origen;
- COARSE no inicia si falla el origen calibrado;
- FINE usa realmente el paso y N de GUI;
- `Δ` limita sin perder simetría ni centro;
- FINE puede refinar más allá de un borde del intervalo COARSE local;
- integración `COARSE Z*=40 → FINE BPoF=41 → stack GUI
  [37, 39, 41, 43, 45]`.

## Indicadores para la próxima prueba física

En el log debe observarse, en este orden:

1. llegada verificada al origen calibrado;
2. `TABLA COARSE` y su `Z_c*`;
3. `ENLACE COARSE→FINE` con `paso_UI`, `paso_eff`, N y `Δ_max_UI`;
4. primer candidato FINE en `Z_c*`;
5. `BPoF TABLA` dentro de la ventana centrada en `Z_c*`;
6. `ENLACE FINE→BPoF→STACK` con exactamente el N y paso de captura de GUI.

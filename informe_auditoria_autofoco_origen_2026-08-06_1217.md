# Auditoría AF: origen calibrado + BPoF

| Campo | Valor |
|-------|--------|
| **Fecha** | 2026-08-06 |
| **Hora** | 12:17 (UTC-4) |
| **Síntoma** | Scan/captura no vuelve al Z de calibración; referencia deriva |
| **Estado** | Corregido |

---

## 1. Contrato

```
Calibración → Z_origen = centro (p.ej. 36.24 µm)
Cada FOV:
  1) GOTO Z_origen (verificado)
  2) Detectar
  3) AF: COARSE → re-ancla Z_c* → FINE → BPoF
  4) Multi-focal en BPoF ± Δ  (park BPoF)
  5) Guardar
  6) GOTO Z_origen (verificado)  ← misma referencia para la siguiente semilla
```

---

## 2. Bugs

| Bug | Efecto |
|-----|--------|
| `_return_cfocus_to_center` usaba `move_z` sin Z_STATIC | A veces no llegaba al centro |
| AF no anclaba al origen antes del scan | Cada run partía de Z residual (BPoF previo) |
| Fine tras coarse en z_max sin re-ancla | Salto 72→25 µm; S fine ≠ S coarse |
| `get_center_position` = z_range/2 HW | Ignoraba centro calibrado |

---

## 3. Strings UI/log

- `Origen calibrado: Z_cmd=… Z_read=…`
- `Re-ancla pre-FINE: Z_c*=…`
- `↩ Origen calibrado Z=… (read=…)µm`
- `Park BPoF` / `Posición final … = BPoF`

---

## 4. Nota sobre BPoF distintos (31 vs 41 µm)

Semillas distintas pueden tener BPoF distinto; eso es correcto.  
Lo que no debe variar es el **origen de partida/retorno** (calibración).

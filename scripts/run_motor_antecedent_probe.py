#!/usr/bin/env python3
"""Sonda de banco en banda 10-20 mm.

1) Lee posicion
2) Kick corto para detectar signo REAL (PWM -> um)
3) Mueve ~2000 um HACIA DENTRO de [10,20] mm (nunca hacia fuera)
4) Guarda CSV/JSON en src/config/motor_antecedent/
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import serial

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from config.constants import (  # noqa: E402
    BAUD_RATE,
    SERIAL_PORT,
    STITION_PWM_MAX,
    STITION_PWM_MIN,
    adc_to_um,
)
from core.control.motor_antecedent import (  # noqa: E402
    AntecedentSample,
    MotorAntecedentResult,
    finalize_antecedent,
)

RANGE_MIN_UM = 10000.0
RANGE_MAX_UM = 20000.0
MARGIN_UM = 300.0


class Telemetry:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.s1 = None
        self.s2 = None
        self.raw = ""
        self.n = 0

    def update(self, line: str) -> None:
        parts = line.strip().split(",")
        if len(parts) < 4:
            return
        try:
            s1 = int(float(parts[2]))
            s2 = int(float(parts[3]))
        except ValueError:
            return
        with self.lock:
            self.s1 = s1
            self.s2 = s2
            self.raw = line.strip()
            self.n += 1

    def get(self, key: str):
        with self.lock:
            return self.s1 if key == "sensor_1" else self.s2 if key == "sensor_2" else None


def reader_loop(ser: serial.Serial, tel: Telemetry, stop: threading.Event) -> None:
    buf = ""
    while not stop.is_set():
        try:
            chunk = ser.read(256)
        except Exception:
            break
        if not chunk:
            continue
        buf += chunk.decode("utf-8", errors="ignore")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            if line.strip():
                tel.update(line.strip())


def send(ser: serial.Serial, cmd: str) -> None:
    ser.write((cmd.strip() + "\n").encode("ascii"))
    ser.flush()


def wait_adc(tel: Telemetry, key: str, timeout_s: float = 2.0):
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < timeout_s:
        v = tel.get(key)
        if v is not None:
            return v
        time.sleep(0.01)
    return None


def apply_pwm(ser: serial.Serial, motor: str, pwm: int) -> None:
    if motor == "A":
        send(ser, f"A,{int(pwm)},0")
    else:
        send(ser, f"A,0,{int(pwm)}")


def read_pos(tel: Telemetry, sensor_key: str, axis: str):
    adc = tel.get(sensor_key)
    if adc is None:
        return None, None
    return float(adc), float(adc_to_um(float(adc), axis=axis))


def interior_direction(pos0: float) -> tuple[int, float, str]:
    """Direccion en um que apunta al INTERIOR de [10,20] mm."""
    lo = RANGE_MIN_UM + MARGIN_UM
    hi = RANGE_MAX_UM - MARGIN_UM
    room_up = max(0.0, hi - pos0)
    room_dn = max(0.0, pos0 - lo)
    # Cerca de 10 mm -> subir; cerca de 20 mm -> bajar
    if room_up >= room_dn:
        return 1, room_up, "hacia 20mm (interior)"
    return -1, room_dn, "hacia 10mm (interior)"


def discover_pwm_sign(
    ser: serial.Serial,
    tel: Telemetry,
    *,
    motor: str,
    sensor_key: str,
    axis: str,
    want_um_dir: int,
    pwm_mag: int,
) -> int:
    """Devuelve +1/-1 de PWM que PRODUCE want_um_dir en um.

    Kick corto (+pwm), mide dU; si no cuadra, usa el signo opuesto.
    """
    send(ser, "A,0,0")
    time.sleep(0.08)
    _, p0 = read_pos(tel, sensor_key, axis)
    if p0 is None:
        return +1 if want_um_dir > 0 else -1

    # Kick +PWM
    apply_pwm(ser, motor, +abs(pwm_mag))
    time.sleep(0.18)
    send(ser, "A,0,0")
    time.sleep(0.10)
    _, p1 = read_pos(tel, sensor_key, axis)
    if p1 is None:
        return +1 if want_um_dir > 0 else -1

    d = p1 - p0
    print(f"  signo-test +PWM={pwm_mag}: dU={d:+.1f}um (de {p0/1000:.2f} a {p1/1000:.2f} mm)")

    if abs(d) < 15.0:
        # No movio: otro intento un poco mas largo
        apply_pwm(ser, motor, +abs(pwm_mag))
        time.sleep(0.35)
        send(ser, "A,0,0")
        time.sleep(0.10)
        _, p2 = read_pos(tel, sensor_key, axis)
        if p2 is not None:
            d = p2 - p0
            print(f"  signo-test retry: dU={d:+.1f}um")

    if abs(d) < 15.0:
        print("  WARN: casi sin movimiento en kick; asumo PWM+ => um+")
        pwm_for_plus_um = +1
    else:
        pwm_for_plus_um = +1 if d > 0 else -1

    # PWM que produce want_um_dir
    return pwm_for_plus_um if want_um_dir > 0 else -pwm_for_plus_um


def probe_motor(
    ser: serial.Serial,
    tel: Telemetry,
    *,
    motor: str,
    sensor_key: str,
    axis: str,
    delta_um: float,
    pwm: int,
    timeout_s: float,
) -> MotorAntecedentResult:
    pwm_mag = max(STITION_PWM_MIN, min(STITION_PWM_MAX, abs(int(pwm))))
    send(ser, "N")
    send(ser, "A,0,0")
    time.sleep(0.05)

    adc0 = wait_adc(tel, sensor_key, 2.0)
    if adc0 is None:
        res = MotorAntecedentResult(
            motor=motor, axis=axis, sensor_key=sensor_key, host_invert=False,
            pwm_cmd=pwm_mag, target_delta_um=abs(delta_um), direction=1,
        )
        res.ok = False
        res.reason = "sin telemetria"
        return finalize_antecedent(res)

    pos0 = float(adc_to_um(float(adc0), axis=axis))
    print(
        f"\n=== Motor {motor} | {sensor_key} | pos0={pos0:.1f}um ({pos0/1000:.2f}mm) ==="
    )

    if pos0 < RANGE_MIN_UM - 800 or pos0 > RANGE_MAX_UM + 800:
        res = MotorAntecedentResult(
            motor=motor, axis=axis, sensor_key=sensor_key, host_invert=False,
            pwm_cmd=pwm_mag, target_delta_um=abs(delta_um), direction=1,
        )
        res.ok = False
        res.reason = f"fuera de 10-20mm (pos={pos0/1000:.2f}mm)"
        print(f"  SKIP: {res.reason}")
        return finalize_antecedent(res)

    um_dir, room, note = interior_direction(pos0)
    planned = min(abs(float(delta_um)), room)
    print(f"  plan: {note} room={room:.0f}um -> mover {um_dir * planned:+.0f}um")

    if planned < 500.0:
        res = MotorAntecedentResult(
            motor=motor, axis=axis, sensor_key=sensor_key, host_invert=False,
            pwm_cmd=pwm_mag, target_delta_um=abs(delta_um), direction=um_dir,
        )
        res.ok = False
        res.reason = "sin margen interior en 10-20mm"
        print(f"  SKIP: {res.reason}")
        return finalize_antecedent(res)

    pwm_sign = discover_pwm_sign(
        ser, tel,
        motor=motor, sensor_key=sensor_key, axis=axis,
        want_um_dir=um_dir, pwm_mag=pwm_mag,
    )
    print(f"  PWM comando = {pwm_sign * pwm_mag:+d} para um_dir={um_dir:+d}")

    # Releer origen tras el kick
    adc1 = wait_adc(tel, sensor_key, 1.0)
    pos0 = float(adc_to_um(float(adc1), axis=axis)) if adc1 is not None else pos0
    # Replan por si el kick nos movio
    um_dir2, room2, _ = interior_direction(pos0)
    if um_dir2 != um_dir:
        um_dir = um_dir2
        pwm_sign = discover_pwm_sign(
            ser, tel,
            motor=motor, sensor_key=sensor_key, axis=axis,
            want_um_dir=um_dir, pwm_mag=pwm_mag,
        )
        adc1 = wait_adc(tel, sensor_key, 1.0)
        pos0 = float(adc_to_um(float(adc1), axis=axis)) if adc1 is not None else pos0
        _, room2, _ = interior_direction(pos0)
    planned = min(abs(float(delta_um)), room2)
    target_signed = um_dir * planned

    res = MotorAntecedentResult(
        motor=motor,
        axis=axis,
        sensor_key=sensor_key,
        host_invert=(pwm_sign * um_dir) < 0,  # True si PWM+ baja um
        pwm_cmd=pwm_mag,
        target_delta_um=planned,
        direction=um_dir,
    )
    t0 = time.perf_counter()
    print(
        f"  RUN pos0={pos0/1000:.2f}mm target={target_signed:+.0f}um "
        f"end~{(pos0 + target_signed)/1000:.2f}mm"
    )

    reason = "timeout"
    pwm_cmd = int(pwm_sign * pwm_mag)
    while True:
        t = time.perf_counter() - t0
        adc, pos = read_pos(tel, sensor_key, axis)
        if adc is None or pos is None:
            time.sleep(0.005)
            continue
        delta = pos - pos0
        # Hard-stop solo si se ALEJA del interior de 10-20 mm.
        # Si arranca un poco bajo 10 mm y sube, dejar entrar a la banda.
        if um_dir > 0 and pos > RANGE_MAX_UM:
            reason = "band_limit"
            break
        if um_dir < 0 and pos < RANGE_MIN_UM:
            reason = "band_limit"
            break
        if um_dir > 0 and pos < RANGE_MIN_UM - 2500:
            reason = "band_limit"
            break
        if um_dir < 0 and pos > RANGE_MAX_UM + 2500:
            reason = "band_limit"
            break
        # Si nos alejamos del objetivo interior, cortar
        if (um_dir > 0 and delta < -80.0) or (um_dir < 0 and delta > 80.0):
            reason = "wrong_way"
            break
        apply_pwm(ser, motor, pwm_cmd)
        res.samples.append(
            AntecedentSample(
                t_s=t, adc=float(adc), pos_um=pos, pwm=pwm_cmd, delta_um=delta
            )
        )
        if len(res.samples) % 20 == 0:
            print(
                f"  t={t*1000:.0f}ms dU={delta:+.1f}um pos={pos/1000:.2f}mm "
                f"PWM={pwm_cmd:+d} adc={adc:.0f}"
            )
        if abs(delta) >= 0.98 * planned:
            reason = "target_reached"
            break
        if t >= timeout_s:
            reason = "timeout"
            break
        time.sleep(0.01)

    send(ser, "A,0,0")
    time.sleep(0.05)
    res.reason = reason
    res = finalize_antecedent(res)
    for line in res.summary_lines():
        print(" ", line.replace("µ", "u").replace("Δ", "d"))
    return res


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=BAUD_RATE)
    ap.add_argument("--delta", type=float, default=2000.0)
    ap.add_argument("--pwm", type=int, default=120)
    ap.add_argument("--timeout", type=float, default=12.0)
    ap.add_argument("--only", choices=("A", "B", "BOTH"), default="BOTH")
    args = ap.parse_args()

    print(f"Abriendo {args.port} @ {args.baud}")
    print(f"Banda: {RANGE_MIN_UM/1000:.0f}-{RANGE_MAX_UM/1000:.0f} mm (interior)")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as e:
        print(f"ERROR COM: {e}")
        return 2

    tel = Telemetry()
    stop = threading.Event()
    threading.Thread(target=reader_loop, args=(ser, tel, stop), daemon=True).start()

    send(ser, "N")
    send(ser, "M")
    send(ser, "A,0,0")
    time.sleep(0.4)
    n0 = tel.n
    time.sleep(0.4)
    print(f"Telemetria: {tel.n - n0} lineas/0.4s last={tel.raw!r}")
    if tel.n == n0:
        print("ERROR: sin telemetria")
        stop.set()
        ser.close()
        return 3

    s1, s2 = tel.get("sensor_1"), tel.get("sensor_2")
    if s1 is not None and s2 is not None:
        xa = adc_to_um(float(s2), "x")
        yb = adc_to_um(float(s1), "y")
        print(f"Ahora: A/X={xa/1000:.2f}mm  B/Y={yb/1000:.2f}mm")

    results = []
    try:
        if args.only in ("A", "BOTH"):
            results.append(
                probe_motor(
                    ser, tel, motor="A", sensor_key="sensor_2", axis="x",
                    delta_um=args.delta, pwm=args.pwm, timeout_s=args.timeout,
                )
            )
            time.sleep(0.5)
        if args.only in ("B", "BOTH"):
            results.append(
                probe_motor(
                    ser, tel, motor="B", sensor_key="sensor_1", axis="y",
                    delta_um=args.delta, pwm=args.pwm, timeout_s=args.timeout,
                )
            )
    finally:
        send(ser, "A,0,0")
        send(ser, "B")
        time.sleep(0.05)
        send(ser, "A,0,0")
        send(ser, "M")
        stop.set()
        time.sleep(0.05)
        ser.close()

    print("\n======== RESUMEN ========")
    for r in results:
        print(
            f"Motor {r.motor}: ok={r.ok} dU={r.delta_um:+.1f}um "
            f"dir={r.direction:+d} invert_eff={r.host_invert} "
            f"K_eff={r.k_eff_um_s_per_pwm:.4f} | {r.reason}"
        )
        print(f"  {r.json_path}")
    return 0 if results and all(r.ok for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())

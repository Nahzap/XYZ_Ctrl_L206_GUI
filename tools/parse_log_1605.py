"""Parse motor_control_20260626.log session 16:05+."""
import re
import statistics as st
from pathlib import Path

LOG = Path(__file__).resolve().parents[1] / "motor_control_20260626.log"
lines = [
    l
    for l in LOG.read_text(encoding="utf-8", errors="replace").splitlines()
    if "2026-06-26 16:0" in l
]

fov_begins = []
fov_ok = []
fov_ticks = []
failures = []
timeouts = []
pwm_cmds = []
handoffs = []

for line in lines:
    if "FOV_VERIFY hacia" in line and "avance=" in line:
        m = re.search(
            r"Punto (\d+).*hacia \(([-\d.]+), ([-\d.]+)\) residual=\(([-\d.]+), ([-\d.]+)\).*avance=([^\s]+) eje=(\w)",
            line,
        )
        if m:
            fov_begins.append(
                {
                    "pt": int(m.group(1)),
                    "nom": (float(m.group(2)), float(m.group(3))),
                    "res": (float(m.group(4)), float(m.group(5))),
                    "avance": m.group(6),
                    "eje0": m.group(7),
                }
            )
    if "FOV_VERIFY OK" in line:
        m = re.search(r"Punto (\d+).*err=\(([-\d.]+), ([-\d.]+)\).*t_verify=(\d+)ms", line)
        if m:
            fov_ok.append(
                {
                    "pt": int(m.group(1)),
                    "err": (float(m.group(2)), float(m.group(3))),
                    "t_ms": int(m.group(4)),
                }
            )
    if "FOV_VERIFY punto" in line and "eje" in line and "contin" in line:
        m = re.search(r"punto (\d+) eje (\w): err=\(([-\d.]+), ([-\d.]+)\).*t=(\d+)ms", line)
        if m:
            fov_ticks.append(
                {
                    "pt": int(m.group(1)),
                    "eje": m.group(2),
                    "err": (float(m.group(3)), float(m.group(4))),
                    "t_ms": int(m.group(5)),
                }
            )
    if "eje" in line and "cumplido" in line:
        m = re.search(r"eje (\w) cumplido \(err=([-\d.]+)", line)
        if m:
            handoffs.append({"eje": m.group(1), "err": float(m.group(2))})
    if "FALL" in line and "Punto" in line:
        failures.append(line.split("|")[-1].strip())
    if "Timeout paso" in line:
        timeouts.append(line)
    if "Comando enviado exitosamente: A," in line:
        m = re.search(r"A,(-?\d+),(-?\d+)", line)
        if m:
            pwm_cmds.append((int(m.group(1)), int(m.group(2))))

print(f"Session 16:0x lines: {len(lines)}")
print(f"\nFOV_VERIFY begins: {len(fov_begins)}")
for b in fov_begins:
    print(
        f"  Pt{b['pt']} nom={b['nom']} res={b['res']} avance={b['avance']} start={b['eje0']}"
    )

print(f"\nFOV_VERIFY OK: {len(fov_ok)}")
for o in fov_ok:
    ex, ey = o["err"]
    print(f"  Pt{o['pt']} err=({ex:+.1f},{ey:+.1f}) t={o['t_ms']}ms max={max(abs(ex),abs(ey)):.1f}")

print(f"\nFailures: {len(failures)}")
for f in failures:
    print(f"  {f}")

print(f"\nStep timeouts: {len(timeouts)}")

# Axis alternation in FOV progress logs per point
print("\n--- FOV axis sequence (progress logs) ---")
by_pt: dict = {}
for t in fov_ticks:
    by_pt.setdefault(t["pt"], []).append(t["eje"])
for pt in sorted(by_pt):
    seq = by_pt[pt]
    switches = sum(1 for i in range(1, len(seq)) if seq[i] != seq[i - 1])
    print(f"  Pt{pt}: {len(seq)} progress logs, {switches} axis switches, last_eje={seq[-1] if seq else '-'}")

# PWM flip-flop X/Y at end
both_zero = sum(1 for a, b in pwm_cmds if a == 0 and b == 0)
x_only = sum(1 for a, b in pwm_cmds if a != 0 and b == 0)
y_only = sum(1 for a, b in pwm_cmds if a == 0 and b != 0)
both = sum(1 for a, b in pwm_cmds if a != 0 and b != 0)
print(f"\nPWM commands: total={len(pwm_cmds)} x_only={x_only} y_only={y_only} both={both} zero={both_zero}")

flips = 0
for i in range(1, len(pwm_cmds)):
    pa, pb = pwm_cmds[i - 1]
    ca, cb = pwm_cmds[i]
    prev_axis = "x" if pa != 0 else ("y" if pb != 0 else "-")
    curr_axis = "x" if ca != 0 else ("y" if cb != 0 else "-")
    if prev_axis != "-" and curr_axis != "-" and prev_axis != curr_axis:
        flips += 1
print(f"PWM axis flips (consecutive): {flips}")

if fov_ok:
    times = [o["t_ms"] for o in fov_ok]
    print(f"\nt_verify mean={st.mean(times):.0f}ms max={max(times)}ms")

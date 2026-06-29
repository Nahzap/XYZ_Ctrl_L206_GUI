"""Parse motor_control log for FOV step session analysis."""
import re
import statistics as st
from pathlib import Path

LOG = Path(__file__).resolve().parents[1] / "motor_control_20260626.log"
lines = LOG.read_text(encoding="utf-8", errors="replace").splitlines()

# Detect sessions by app start
sessions = []
for i, line in enumerate(lines):
    if "INICIANDO SISTEMA DE CONTROL" in line:
        ts = line[:19]
        sessions.append((ts, i))

print("Sessions:", sessions[-5:])

# Use last session (15:01+)
start_idx = sessions[-1][1] if sessions else 0
chunk = lines[start_idx:]
print(f"Analyzing from line {start_idx}, {len(chunk)} lines, session {sessions[-1][0] if sessions else '?'}")

pts = []
res = []
desf = []
prep = []
hinf = []
for line in chunk:
    if "prepare_transition" in line and "StepController" in line:
        prep.append(line)
    if "Homog" in line and "pasos" in line:
        m = re.search(r"Punto (\d+).*?\((\d+) pasos, (\d+)ms\)", line)
        if m:
            pts.append((int(m.group(1)), int(m.group(2)), int(m.group(3))))
    if "residual vs FOV" in line:
        m = re.search(
            r"Punto (\d+).*actual=\(([-\d.]+),([-\d.]+)\) nominal=\(([-\d.]+),([-\d.]+)\) err=\(([-\d.]+),([-\d.]+)\).*tol_fov=([\d.]+)",
            line,
        )
        if m:
            res.append(
                {
                    "pt": int(m.group(1)),
                    "actual": (float(m.group(2)), float(m.group(3))),
                    "nominal": (float(m.group(4)), float(m.group(5))),
                    "err": (float(m.group(6)), float(m.group(7))),
                    "tol": float(m.group(8)),
                }
            )
    if "Desfase posici" in line:
        desf.append(line.split("|")[-1].strip())
    if "Transici" in line and "TestService" in line:
        m = re.search(
            r"Transici[^\d]*\((\d+)\).*actual=\(([-\d.]+),([-\d.]+)\).*FOV=\(([-\d.]+),([-\d.]+)\)",
            line,
        )
        if m:
            hinf.append(
                (
                    int(m.group(1)),
                    float(m.group(2)),
                    float(m.group(3)),
                    float(m.group(4)),
                    float(m.group(5)),
                )
            )
    if "mode=hinf_native" in line or "mode=orchestrated" in line:
        mode_line = line

mode = "unknown"
for line in prep[:3]:
    if "mode=hinf_native" in line:
        mode = "hinf_native"
    elif "mode=orchestrated" in line:
        mode = "orchestrated"

print(f"\nMode: {mode}")
print(f"Points completed: {len(pts)}")
print(f"Desfase warnings: {len(desf)}")

if pts:
    times = [x[2] for x in pts]
    steps = [x[1] for x in pts]
    print(f"t_move: mean={st.mean(times):.0f}ms std={st.pstdev(times):.0f}ms min={min(times)} max={max(times)}")
    print(f"n_steps: mean={st.mean(steps):.1f} min={min(steps)} max={max(steps)}")

print("\n--- Per-point table ---")
print(f"{'Pt':>4} {'steps':>5} {'t_ms':>6} {'nom_x':>8} {'nom_y':>8} {'act_x':>8} {'act_y':>8} {'err_x':>7} {'err_y':>7} {'|e|>tol':>7}")
for p in res:
    ex, ey = p["err"]
    over = abs(ex) > p["tol"] or abs(ey) > p["tol"]
    print(
        f"{p['pt']:4d} "
        f"{'':5s} "
        f"{'':6s} "
        f"{p['nominal'][0]:8.1f} {p['nominal'][1]:8.1f} "
        f"{p['actual'][0]:8.1f} {p['actual'][1]:8.1f} "
        f"{ex:7.1f} {ey:7.1f} {'YES' if over else 'no':>7}"
    )

# Merge with pts timing
pt_map = {p[0]: (p[1], p[2]) for p in pts}
print("\n--- With timing ---")
for p in res:
    stp = pt_map.get(p["pt"], (None, None))
    ex, ey = p["err"]
    print(f"Pt{p['pt']:3d} steps={stp[0]} t={stp[1]}ms err=({ex:.1f},{ey:.1f})")

# FOV nominal vs actual delta analysis for row moves (dy_nom ~ 0)
print("\n--- Row moves (|dY_nom|<1) ---")
for t in hinf:
    idx, ax, ay, nx, ny = t
    if abs(ny) < 1.0 and abs(ax) > 50:
        print(f"  Pt{idx}: dX_actual={ax:.1f} dX_nom={nx:.1f} dY_actual={ay:.1f}")

# Accumulated drift X on a row (same Y_nominal)
if res:
    by_y = {}
    for p in res:
        yn = round(p["nominal"][1])
        by_y.setdefault(yn, []).append(p)
    print("\n--- Residual Y by row (nominal Y) ---")
    for yn in sorted(by_y.keys())[:5]:
        errs = [abs(p["err"][1]) for p in by_y[yn]]
        print(f"  Y_nom={yn}: mean|err_y|={st.mean(errs):.1f} max={max(errs):.1f} n={len(errs)}")

# Nominal FOV spacing check
if len(res) >= 2:
    print("\n--- Nominal FOV spacing (consecutive points) ---")
    for i in range(min(5, len(res) - 1)):
        a, b = res[i], res[i + 1]
        dx = b["nominal"][0] - a["nominal"][0]
        dy = b["nominal"][1] - a["nominal"][1]
        dxa = b["actual"][0] - a["actual"][0]
        dya = b["actual"][1] - a["actual"][1]
        print(
            f"  {a['pt']}→{b['pt']}: nom Δ=({dx:.1f},{dy:.1f}) actual Δ=({dxa:.1f},{dya:.1f}) "
            f"Δerror=({dxa-dx:.1f},{dya-dy:.1f})"
        )

# First prepare_transition sample
if prep:
    print("\n--- First prepare_transition ---")
    print(prep[0][80:200])

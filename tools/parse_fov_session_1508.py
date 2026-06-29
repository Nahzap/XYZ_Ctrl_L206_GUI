"""Parse 15:08 FOV session for canvas overlap analysis."""
import re
import statistics as st
from pathlib import Path

LOG = Path(__file__).resolve().parents[1] / "motor_control_20260626.log"
FOV_X = 162.9  # from log dX_nom
FOV_Y = 122.1  # test_parameters_template.json

lines = [
    l
    for l in LOG.read_text(encoding="utf-8", errors="replace").splitlines()
    if l.startswith("2026-06-26 15:08") or l.startswith("2026-06-26 15:09")
    or l.startswith("2026-06-26 15:10") or l.startswith("2026-06-26 15:11")
    or l.startswith("2026-06-26 15:12") or l.startswith("2026-06-26 15:13")
    or l.startswith("2026-06-26 15:14") or l.startswith("2026-06-26 15:15")
]

pts = []
res = []
trans = []
for line in lines:
    m = re.search(r"Homog[^\d]*\((\d+) pasos, (\d+)ms\) Punto (\d+)", line)
    if m:
        pts.append((int(m.group(3)), int(m.group(1)), int(m.group(2))))
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
    m = re.search(
        r"Transici[^\d]*\((\d+)\).*actual=\(([-\d.]+),([-\d.]+)\).*FOV=\(([-\d.]+),([-\d.]+)\)",
        line,
    )
    if m:
        trans.append(
            {
                "pt": int(m.group(1)),
                "d_actual": (float(m.group(2)), float(m.group(3))),
                "d_nom": (float(m.group(4)), float(m.group(5))),
            }
        )

res.sort(key=lambda x: x["pt"])
pts.sort(key=lambda x: x[0])
pt_time = {p[0]: (p[1], p[2]) for p in pts}

print(f"Session 15:08+ points: {len(res)} residuals, {len(pts)} completed")
if pts:
    times = [p[2] for p in pts]
    print(f"t_move mean={st.mean(times):.0f}ms std={st.pstdev(times):.0f}ms")

if res:
    ex = [abs(p["err"][0]) for p in res]
    ey = [abs(p["err"][1]) for p in res]
    tol = res[0]["tol"]
    print(f"|err_x| mean={st.mean(ex):.1f} max={max(ex):.1f}")
    print(f"|err_y| mean={st.mean(ey):.1f} max={max(ey):.1f}")
    print(f"points over tol_fov {tol}: {sum(1 for p in res if abs(p['err'][0])>tol or abs(p['err'][1])>tol)}/{len(res)}")

# Canvas: effective step between consecutive captures vs FOV
print("\n--- Consecutive capture spacing (actual vs FOV) ---")
gaps_x = []
gaps_y = []
for i in range(len(res) - 1):
    a, b = res[i], res[i + 1]
    dxa = b["actual"][0] - a["actual"][0]
    dya = b["actual"][1] - a["actual"][1]
    dxn = b["nominal"][0] - a["nominal"][0]
    dyn = b["nominal"][1] - a["nominal"][1]
    if abs(dxn) > 50 and abs(dyn) < 5:  # row step
        gaps_x.append(dxa)
        eff_overlap_x = (FOV_X - abs(dxa - FOV_X)) / FOV_X * 100 if abs(dxa) > 0 else 0
    if abs(dyn) > 50 and abs(dxn) < 5:  # row change
        gaps_y.append(dya)
    if i < 8:
        print(
            f"  Pt{a['pt']}->{b['pt']}: nom=({dxn:.0f},{dyn:.0f}) "
            f"actual=({dxa:.0f},{dya:.0f}) err_vs_nom=({dxa-dxn:.0f},{dya-dyn:.0f})"
        )

if gaps_x:
    print(f"\nRow X steps (actual): mean={st.mean(gaps_x):.1f} std={st.pstdev(gaps_x):.1f} (FOV_X={FOV_X})")
    print(f"  Delta from FOV_X: mean={st.mean([g-FOV_X for g in gaps_x]):.1f} max={max(abs(g-FOV_X) for g in gaps_x):.1f}")

# Row internal Y drift at same Y_nom
by_row = {}
for p in res:
    yn = round(p["nominal"][1])
    by_row.setdefault(yn, []).append(p)
print("\n--- Y drift within row (same Y_nominal) ---")
for yn in sorted(by_row.keys()):
    errs_y = [p["err"][1] for p in by_row[yn]]
    spread = max(p["actual"][1] for p in by_row[yn]) - min(p["actual"][1] for p in by_row[yn])
    print(
        f"  Y_nom={yn}: n={len(by_row[yn])} err_y mean={st.mean([abs(e) for e in errs_y]):.1f} "
        f"spread_actual_Y={spread:.1f}um (FOV_Y={FOV_Y})"
    )

# Estimated canvas overlap if tiles placed at actual positions on nominal grid
# Adjacent in X on same row: overlap = FOV_X - |actual_delta_x - FOV_X| ... simplified
print("\n--- Estimated overlap vs FOV (same-row neighbors) ---")
res_by_pt = {p["pt"]: p for p in res}
overlap_pcts = []
for i in range(len(res) - 1):
    a, b = res[i], res[i + 1]
    if abs(a["nominal"][1] - b["nominal"][1]) < 1:
        dxa = abs(b["actual"][0] - a["actual"][0])
        # overlap fraction: 1 - |dxa - FOV_X|/FOV_X clipped
        ov = max(0, 100 * (1 - abs(dxa - FOV_X) / FOV_X))
        overlap_pcts.append(ov)
        if len(overlap_pcts) <= 6:
            print(f"  Pt{a['pt']}->{b['pt']}: dX_actual={dxa:.1f} overlap_X~{ov:.0f}%")
if overlap_pcts:
    print(f"  mean overlap_X~{st.mean(overlap_pcts):.0f}% min={min(overlap_pcts):.0f}%")

# d_actual vs d_nom on row moves
row_trans = [t for t in trans if abs(t["d_nom"][1]) < 1 and abs(t["d_nom"][0]) > 50]
if row_trans:
    err_dx = [t["d_actual"][0] - t["d_nom"][0] for t in row_trans]
    err_dy = [t["d_actual"][1] - t["d_nom"][1] for t in row_trans]
    print(f"\nRow transitions: n={len(row_trans)}")
    print(f"  dX actual-nom: mean={st.mean(err_dx):.1f} std={st.pstdev(err_dx):.1f}")
    print(f"  dY spurious (nom=0): mean={st.mean(err_dy):.1f} max={max(abs(y) for y in err_dy):.1f}")

print("\n--- Per-point (first 15) ---")
for p in res[:15]:
    stp = pt_time.get(p["pt"], (None, None))
    ex, ey = p["err"]
    print(
        f"Pt{p['pt']:3d} steps={stp[0]} t={stp[1]}ms "
        f"err=({ex:+.1f},{ey:+.1f}) |e|>{p['tol']}"
    )

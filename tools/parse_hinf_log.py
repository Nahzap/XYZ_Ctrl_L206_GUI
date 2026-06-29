import re
import statistics as st
from collections import Counter
from pathlib import Path

p = Path(__file__).resolve().parents[1] / "motor_control_20260626.log"
lines = p.read_text(encoding="utf-8", errors="replace").splitlines()

cmds = []
for line in lines:
    if "14:04:" in line or "14:05:0" in line or "14:05:1" in line or "14:05:2" in line:
        m = re.search(r"Comando enviado exitosamente: (A,[-0-9,]+)", line)
        if m:
            cmds.append(m.group(1))

ctr = Counter(cmds)
print("Top PWM commands (hinf session 14:04-14:05):")
for k, v in ctr.most_common(12):
    print(f"  {v:5d}  {k}")
print(f"total A commands: {len(cmds)}")

y_pwms = []
x_pwms = []
for c in cmds:
    m = re.match(r"A,(-?\d+),(-?\d+)", c)
    if m:
        x_pwms.append(int(m.group(1)))
        y_pwms.append(int(m.group(2)))

flips_y = sum(
    1
    for i in range(1, len(y_pwms))
    if y_pwms[i] * y_pwms[i - 1] < 0 and y_pwms[i] != 0 and y_pwms[i - 1] != 0
)
flips_x = sum(
    1
    for i in range(1, len(x_pwms))
    if x_pwms[i] * x_pwms[i - 1] < 0 and x_pwms[i] != 0 and x_pwms[i - 1] != 0
)
print(f"Y pwm sign flips (nonzero): {flips_y}")
print(f"X pwm sign flips (nonzero): {flips_x}")
print(f"pct A,0,0: {100*ctr.get('A,0,0',0)/len(cmds):.1f}%")
sat = sum(v for k, v in ctr.items() if "135" in k)
print(f"pct saturated |pwm|=135: {100*sat/len(cmds):.1f}%")

# per-point table
pts = []
for line in lines:
    if "Homog" in line and ("14:04:" in line or "14:05:" in line):
        m = re.search(r"Punto (\d+).*?\((\d+) pasos, (\d+)ms\)", line)
        if m:
            pts.append((int(m.group(1)), int(m.group(2)), int(m.group(3))))

res = []
for line in lines:
    if "residual vs FOV" in line and ("14:04:" in line or "14:05:" in line):
        m = re.search(
            r"Punto (\d+).*err=\(([-\d.]+),([-\d.]+)\).*tol_fov=([\d.]+)", line
        )
        if m:
            res.append(
                (
                    int(m.group(1)),
                    float(m.group(2)),
                    float(m.group(3)),
                    float(m.group(4)),
                )
            )

print("\nPer-point summary:")
print(f"{'Pt':>3} {'steps':>5} {'t_ms':>6} {'err_x':>7} {'err_y':>7} {'|e|>tol':>8}")
for (pi, ns, tm), (_, ex, ey, tol) in zip(pts, res):
    over = abs(ex) > tol or abs(ey) > tol
    print(f"{pi:3d} {ns:5d} {tm:6d} {ex:7.1f} {ey:7.1f} {'YES' if over else 'no':>8}")

if pts:
    times = [x[2] for x in pts]
    print(f"\nt_move mean={st.mean(times):.0f}ms std={st.pstdev(times):.0f}ms CV={st.pstdev(times)/st.mean(times):.2f}")

import json
import sys
from pathlib import Path

sys.path.insert(0, "src")
from config.constants import lsb_um, mcu_cz_invert, slope_um_per_adc

for m in ("A", "B"):
    d = json.loads(Path(f"src/config/motor_antecedent/latest_{m}.json").read_text(encoding="utf-8"))
    print("===", m, "===")
    print(
        " dU={:.1f} um t={:.0f} ms K_eff={:.4f}".format(
            d["delta_um"], d["duration_s"] * 1000, d["k_eff_um_s_per_pwm"]
        )
    )
    print(
        " v={:.1f} um/s host_invert_eff={} PWM_plus_raises_um={}".format(
            d["mean_vel_um_s"], d["host_invert"], (not d["host_invert"])
        )
    )
    ke = d["k_eff_um_s_per_pwm"]
    print(" v@95={:.0f} um/s | 50ms@95 ≈ {:.0f} um | 200ms@95 ≈ {:.0f} um".format(
        ke * 95, ke * 95 * 0.05, ke * 95 * 0.20
    ))

print("LSB x/y um:", round(lsb_um("x"), 3), round(lsb_um("y"), 3))
print("slope x/y:", slope_um_per_adc("x"), slope_um_per_adc("y"))
print("MCU I with Invert OFF/OFF:", int(mcu_cz_invert("x", False)), int(mcu_cz_invert("y", False)))
print("MCU I with Invert ON/ON :", int(mcu_cz_invert("x", True)), int(mcu_cz_invert("y", True)))
print("tol 8um ≈ ADC gate:", round(8 / lsb_um("x"), 2), round(8 / lsb_um("y"), 2), "counts")

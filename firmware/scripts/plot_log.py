"""Plot motor_log.csv: setpoint vs. measured speed, and motor voltage."""
import csv
import sys

import matplotlib.pyplot as plt

path = sys.argv[1] if len(sys.argv) > 1 else "motor_log.csv"
rows = list(csv.DictReader(open(path)))
t = [int(r["time_ms"]) / 1000 for r in rows]
setpoint = [float(r["setpoint_rpm"]) for r in rows]
measured = [float(r["measured_rpm"]) for r in rows]
voltage = [float(r["voltage"]) for r in rows]

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
ax1.plot(t, setpoint, "--", label="Setpoint")
ax1.plot(t, measured, label="Measured (encoder)")
ax1.set_ylabel("Speed [RPM]")
ax1.set_title("FreeRTOS PID speed control of a simulated DC motor")
for x, text in ((2.0, "load step"), (5.4, "watchdog stop")):
    ax1.axvline(x, color="gray", linewidth=0.8)
    ax1.text(x + 0.05, max(setpoint) * 0.7, text, color="gray")
ax1.legend(loc="upper right")
ax1.grid(alpha=0.3)

ax2.plot(t, voltage, color="tab:red")
ax2.set_ylabel("Motor voltage [V]")
ax2.set_xlabel("Time [s]")
ax2.grid(alpha=0.3)

fig.tight_layout()
out = path.replace(".csv", ".png")
fig.savefig(out, dpi=120)
print(f"Saved plot to {out}")
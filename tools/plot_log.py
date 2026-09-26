"""
plot_log.py - plot an IMU Logger CSV: accelerometer-only vs gyro-only vs filtered angle.

Usage:   python plot_log.py LOG005.CSV
Output:  LOG005.png, saved next to the CSV

Needs:   pip install pandas matplotlib numpy
"""
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ---- Must match the firmware ----
ACCEL_COUNTS_PER_G = 4096.0      # ACCEL_CONFIG = 0x10 (+/-8 g)
GYRO_COUNTS_PER_DPS = 16.4       # GYRO_CONFIG  = 0x18 (+/-2000 deg/s)
ACCEL_OFFSET = (0.095, -0.025, -0.22)   # g, subtracted per axis (same as firmware)

# ---- Colours (validated categorical palette, light mode) ----
FILTER, ACCEL, GYRO = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK_2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"


def wrap180(a):
    """Keep an angle between -180 and +180 degrees."""
    return (a + 180.0) % 360.0 - 180.0


def break_wraps(y):
    """Insert a gap where an angle jumps across +/-180, so the line
    doesn't draw a vertical streak across the whole chart."""
    y = np.asarray(y, dtype=float).copy()
    jumps = np.abs(np.diff(y)) > 180
    y[1:][jumps] = np.nan
    return y


def main(path):
    d = pd.read_csv(path, skipinitialspace=True)
    t = (d.ms - d.ms.iloc[0]) / 1000.0          # seconds since logging started
    dt = d.ms.diff().fillna(10) / 1000.0         # real time between rows

    # Accelerometer in g, with the same offsets the firmware uses
    ax = d.ax / ACCEL_COUNTS_PER_G - ACCEL_OFFSET[0]
    ay = d.ay / ACCEL_COUNTS_PER_G - ACCEL_OFFSET[1]
    az = d.az / ACCEL_COUNTS_PER_G - ACCEL_OFFSET[2]

    # 1) Accelerometer only: direction of gravity (fooled by any motion)
    acc_roll = np.degrees(np.arctan2(ay, az))
    acc_pitch = np.degrees(np.arctan2(-ax, np.sqrt(ay**2 + az**2)))

    # 2) Gyro only: add up rotation rate x time (smooth, but drifts)
    gyro_roll = wrap180(acc_roll.iloc[0] + np.cumsum(d.gx / GYRO_COUNTS_PER_DPS * dt))
    gyro_pitch = acc_pitch.iloc[0] + np.cumsum(d.gy / GYRO_COUNTS_PER_DPS * dt)

    # 3) What the firmware's complementary filter logged
    filt_roll = d.roll_x100 / 100.0
    filt_pitch = d.pitch_x100 / 100.0

    plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK_2,
                         "xtick.color": INK_2, "ytick.color": INK_2})
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True, facecolor=SURFACE)

    panels = [
        (axes[0], "Roll (rotation about X)", acc_roll, gyro_roll, filt_roll, (-180, 180)),
        (axes[1], "Pitch (rotation about Y)", acc_pitch, gyro_pitch, filt_pitch, None),
    ]
    for ax_, title, acc, gyro, filt, ylim in panels:
        ax_.set_facecolor(SURFACE)
        ax_.plot(t, break_wraps(acc), color=ACCEL, lw=1, alpha=0.85, label="Accelerometer only")
        ax_.plot(t, break_wraps(gyro), color=GYRO, lw=1.5, label="Gyro only")
        ax_.plot(t, break_wraps(filt), color=FILTER, lw=2, label="Complementary filter (logged)")
        ax_.set_title(title, loc="left", color=INK, fontsize=11, fontweight="bold")
        ax_.set_ylabel("degrees")
        ax_.grid(True, color=GRID, lw=0.8)
        ax_.spines[["top", "right"]].set_visible(False)
        if ylim:
            ax_.set_ylim(*ylim)

    axes[1].set_xlabel("time since logging started (s)")
    fig.suptitle(f"IMU Logger - {path.split('/')[-1]}", x=0.01, ha="left",
                 color=INK, fontsize=13, fontweight="bold", y=0.99)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper left", bbox_to_anchor=(0.01, 0.955), ncol=3,
               frameon=False, labelcolor=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.92))

    out = path.rsplit(".", 1)[0] + ".png"
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print("saved", out)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: python plot_log.py LOGnnn.CSV")
        sys.exit(1)
    main(sys.argv[1])

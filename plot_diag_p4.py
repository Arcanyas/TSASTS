"""
Diagnostic plot for Phase 4 stability signals.
Run after a simulation: mjpython plot_diag_p4.py
"""

import sys
from pathlib import Path
import numpy as np

try:
    import pandas as pd
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
except ImportError:
    print("Install pandas + matplotlib: pip install pandas matplotlib")
    sys.exit(1)

LOG_PATH = Path(__file__).resolve().parent / "logs" / "diag_p4.csv"

if not LOG_PATH.exists():
    print(f"Log not found: {LOG_PATH}\nRun a simulation first.")
    sys.exit(1)

df = pd.read_csv(LOG_PATH)
df_p4 = df[df["phase"] == 4].copy()

if df_p4.empty:
    print("No Phase 4 data in log — did the simulation reach Phase 4?")
    sys.exit(1)

t = df["time"]
t4 = df_p4["time"]

# Phase boundary times for shading
p3_start = df[df["phase"] == 3]["time"].iloc[0] if (df["phase"] == 3).any() else None
p4_start = t4.iloc[0]

fig = plt.figure(figsize=(16, 18))
fig.suptitle("Phase 3→4 Stability Diagnostics", fontsize=14, fontweight="bold")
gs = gridspec.GridSpec(5, 2, figure=fig, hspace=0.5, wspace=0.35)


def shade_phases(ax):
    ax.axvline(p4_start, color="navy", lw=1.2, ls="--", label="P4 start")
    if p3_start is not None:
        ax.axvspan(p3_start, p4_start, alpha=0.07, color="blue", label="Phase 3")
    ax.axvspan(p4_start, t.iloc[-1], alpha=0.07, color="green", label="Phase 4")


# ── 1. Root pitch & lean ──────────────────────────────────────────────────────
ax = fig.add_subplot(gs[0, 0])
ax.plot(t, df["root_pitch"], label="root_pitch", color="steelblue")
ax.plot(t, df["lean"], label="trunk_lean_rel", color="darkorange", ls="--")
ax.axhline(-0.68, color="red", lw=0.8, ls=":", label="backlean limit (-0.68)")
ax.axhline(-0.52, color="purple", lw=0.8, ls=":", label="P5 guard (-0.52)")
shade_phases(ax)
ax.set_title("Root pitch & trunk lean")
ax.set_xlabel("time (s)")
ax.set_ylabel("rad")
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

# ── 2. flex_extension ─────────────────────────────────────────────────────────
ax = fig.add_subplot(gs[0, 1])
ax.plot(t, df["flex_extension"], label="flex_extension", color="teal")
ax.axhline(-0.20, color="red", lw=0.8, ls=":", label="P4 spine target (-0.20)")
shade_phases(ax)
ax.set_title("Spine joint (flex_extension)")
ax.set_xlabel("time (s)")
ax.set_ylabel("rad")
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

# ── 3. Pelvis height & velocity ───────────────────────────────────────────────
ax = fig.add_subplot(gs[1, 0])
ax.plot(t, df["pelvis_z"], label="pelvis_z", color="darkgreen")
ax.axhline(0.84, color="red", lw=0.8, ls=":", label="P4 target (0.84)")
shade_phases(ax)
ax.set_title("Pelvis height")
ax.set_xlabel("time (s)")
ax.set_ylabel("m")
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

ax = fig.add_subplot(gs[1, 1])
ax.plot(t, df["pelvis_z_vel"], label="pelvis_z_vel", color="olive")
ax.axhline(0.0, color="gray", lw=0.8)
shade_phases(ax)
ax.set_title("Pelvis vertical velocity")
ax.set_xlabel("time (s)")
ax.set_ylabel("m/s")
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

# ── 4. Knee angles ────────────────────────────────────────────────────────────
ax = fig.add_subplot(gs[2, 0])
ax.plot(t, df["knee_r"], label="knee_r", color="crimson")
ax.plot(t, df["knee_l"], label="knee_l", color="salmon", ls="--")
ax.axhline(0.02, color="blue", lw=0.8, ls=":", label="P4 target (0.02)")
ax.axhline(0.0, color="black", lw=0.8, ls=":", label="straight (0)")
shade_phases(ax)
ax.set_title("Knee angles — hyperextension < 0")
ax.set_xlabel("time (s)")
ax.set_ylabel("rad")
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

# ── 5. Hip & ankle ────────────────────────────────────────────────────────────
ax = fig.add_subplot(gs[2, 1])
ax.plot(t, df["hip"], label="hip", color="purple")
ax.plot(t, df["ankle"], label="ankle", color="brown", ls="--")
ax.axhline(-0.18, color="purple", lw=0.8, ls=":", label="P4 hip target (-0.18)")
shade_phases(ax)
ax.set_title("Hip & ankle angles")
ax.set_xlabel("time (s)")
ax.set_ylabel("rad")
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

# ── 6. TORSO muscle signals ───────────────────────────────────────────────────
ax = fig.add_subplot(gs[3, 0])
ax.plot(t, df["stim_TORSO_EXT"], label="stim TORSO_EXT", color="firebrick")
ax.plot(t, df["stim_TORSO_FLEX"], label="stim TORSO_FLEX", color="dodgerblue")
ax.plot(t, df["S_P4_TORSO_EXT"], label="module S_P4_TORSO_EXT", color="firebrick", ls="--", alpha=0.5)
ax.plot(t, df["S_P4_TORSO_FLEX"], label="module S_P4_TORSO_FLEX", color="dodgerblue", ls="--", alpha=0.5)
ax.axhline(0.28, color="firebrick", lw=0.7, ls=":", label="TORSO_EXT floor (0.28)")
shade_phases(ax)
ax.set_title("Torso muscle signals")
ax.set_xlabel("time (s)")
ax.set_ylabel("activation")
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

# ── 7. Leg muscle signals ─────────────────────────────────────────────────────
ax = fig.add_subplot(gs[3, 1])
ax.plot(t, df["stim_GLU"], label="stim GLU", color="darkorange")
ax.plot(t, df["stim_VAS"], label="stim VAS", color="steelblue")
ax.plot(t, df["stim_HAM"], label="stim HAM", color="green")
ax.plot(t, df["stim_HFL"], label="stim HFL", color="violet")
shade_phases(ax)
ax.set_title("Leg muscle signals")
ax.set_xlabel("time (s)")
ax.set_ylabel("activation")
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

# ── 8. Root pitch velocity ────────────────────────────────────────────────────
ax = fig.add_subplot(gs[4, :])
ax.plot(t, df["root_pitch_rel_vel"], label="root_pitch_rel_vel", color="darkred")
ax.axhline(0.0, color="gray", lw=0.8)
shade_phases(ax)
ax.set_title("Root pitch relative velocity (oscillation indicator)")
ax.set_xlabel("time (s)")
ax.set_ylabel("rad/s")
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

out_path = LOG_PATH.parent / "diag_p4.png"
plt.savefig(out_path, dpi=130, bbox_inches="tight")
print(f"Saved: {out_path}")

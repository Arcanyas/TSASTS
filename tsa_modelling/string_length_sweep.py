"""
TSA string-length sweep: contraction, Jacobian, and required torque.

Generates a 3×1 subplot figure for string lengths L in [0.25, 0.70] m
at fixed motor shaft radius r, payload mass m, and a maximum winding
angle of 200 rad.

Physics
-------
Contraction:       X(θ)    = L − √(L² − (θ·r)²)         capped at 0.30·L
Jacobian:          J(θ)    = dX/dθ = θ·r² / √(L² − (θ·r)²)
Required torque:   τ(θ)    = F_payload · J(θ)   (virtual work)

Plots terminate where the string hits its max-contraction limit (30% of L)
or at MAX_ANGLE, whichever comes first.

Run from tsa_modelling/:
    python string_length_sweep.py

Output: images/string_length_sweep.png
"""

from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from pathlib import Path

# ── Constants ────────────────────────────────────────────────────────────────
r       = 0.004   # motor shaft / string radius [m]
m_pay   = 10.0    # payload mass [kg]  (shin-cable equivalent, same as _SHIN_CABLE_MASS)
g       = 9.81    # [m/s²]
F_pay   = m_pay * g                  # constant cable tension [N]
MAX_CONTRACTION_RATIO = 0.30         # string saturates at 30 % of L
MAX_ANGLE = 125.0                    # x-axis upper bound [rad]
N_PTS   = 600                        # points per curve

# ── String-length sweep ───────────────────────────────────────────────────────
L_values = np.round(np.arange(0.25, 0.71, 0.05), 2)   # 0.25, 0.30, … 0.70 m

# ── Colourmap ────────────────────────────────────────────────────────────────
cmap   = cm.viridis
n      = len(L_values)
colors = [cmap(i / (n - 1)) for i in range(n)]

# ── Figure ───────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(10, 13), sharex=True)
fig.suptitle(
    f"TSA String-Length Sweep\n"
    f"r = {r*1e3:.1f} mm   |   payload mass = {m_pay:.0f} kg   |   max θ = {MAX_ANGLE:.0f} rad",
    fontsize=13, fontweight="bold",
)

for L, color in zip(L_values, colors):
    # Angle at which contraction saturates at MAX_CONTRACTION_RATIO × L.
    # From X_max = L − √(L² − (θ_sat·r)²)  →  θ_sat = √(L²−(L−X_max)²) / r
    X_max   = MAX_CONTRACTION_RATIO * L
    theta_sat = np.sqrt(L**2 - (L - X_max)**2) / r     # geometric limit
    theta_end = min(theta_sat, MAX_ANGLE)

    theta = np.linspace(0.0, theta_end, N_PTS)
    arg   = np.maximum(L**2 - (theta * r)**2, 1e-15)   # numerical safety

    # 1. Contraction [mm]
    X_mm = (L - np.sqrt(arg)) * 1e3

    # 2. Jacobian dX/dθ [mm/rad]
    J_mm = (theta * r**2 / np.sqrt(arg)) * 1e3

    # 3. Required motor torque to maintain constant payload tension [N·m]
    tau = F_pay * (theta * r**2 / np.sqrt(arg))

    label = f"L = {L:.2f} m"
    axes[0].plot(theta, X_mm, color=color, lw=1.6, label=label)
    axes[1].plot(theta, J_mm, color=color, lw=1.6)
    axes[2].plot(theta, tau,  color=color, lw=1.6)

# ── Stall-torque reference on panel 3 ────────────────────────────────────────
TAU_STALL = 0.1668   # N·m  (1.7 kg·cm motor used in simulation)
axes[2].axhline(TAU_STALL, color="red", lw=1.2, ls="--",
                label=f"Stall torque ({TAU_STALL:.3f} N·m)")

# ── Axis labels and formatting ────────────────────────────────────────────────
panel_cfg = [
    ("Contraction  X  (mm)",        "String contraction vs motor angle"),
    ("Jacobian  dX/dθ  (mm / rad)", "Contraction rate (Jacobian) vs motor angle"),
    ("Required torque  (N·m)",      "Motor torque required to pull payload vs motor angle"),
]
for ax, (ylabel, title) in zip(axes, panel_cfg):
    ax.set_ylabel(ylabel, fontsize=10)
    ax.set_title(title, fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, MAX_ANGLE)

axes[-1].set_xlabel("Motor angle  θ  (rad)", fontsize=10)

# Legend on panel 1 (contraction); stall-torque legend on panel 3.
axes[0].legend(fontsize=7, ncol=2, loc="upper left", title="String length")
axes[2].legend(fontsize=8, loc="upper left")

plt.tight_layout()

OUT = Path(__file__).resolve().parent / "images" / "string_length_sweep.png"
fig.savefig(str(OUT), dpi=150, bbox_inches="tight")
print(f"Saved → {OUT}")
plt.show()

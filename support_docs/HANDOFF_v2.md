# TSA Exoskeleton STS — Handoff Document v2

## Project Overview

A Twisted String Actuator (TSA) exoskeleton assists knee **extension** during a
MuJoCo sit-to-stand (STS) simulation. Cable tension is computed per-step and
injected as negative `qfrc_applied` at the knee DOF (extension direction).

Key files:
- `tsa_modelling/model_v2.py` — TSASimulator (single-unit TSA physics + EOM)
- `tsa_modelling/actuator.py` — TSAActuator (tension→knee torque, stall logic)
- `ctrl_optim/tsa_integration.py` — bridges both actuators into MuJoCo
- `ctrl_optim/sts_ctrl.py` — 4-phase STS reflex controller (calls TSA integration)
- `plot_tsa_log.py` — log plotter (percentile-scaled y-axes to suppress init spike)
- `logs/tsa_log.csv` / `logs/tsa_log_plot.png` — most recent run

---

## Physical Parameters (current values)

| Parameter | Value | File |
|-----------|-------|------|
| String length L | 0.50 m | `tsa_integration.py` |
| Cable radius r | 4 mm | `tsa_integration.py` |
| Moment arm d | **0.03 m** (placeholder) | `tsa_integration.py` `MOMENT_ARM` |
| Max contraction ratio | 0.30 | `tsa_integration.py` → `model_v2.py` |
| Max contraction X_max | 150 mm | 0.30 × 0.50 |
| Motor stall torque | 0.1275 N·m | Pololu 9.7:1 LP 12 V |
| Motor no-load speed | 60.7 rad/s | 580 RPM × 2π/60 |
| Motor inertia I | 5×10⁻⁵ kg·m² | `tsa_integration.py` |
| Pretension θ₀ | 2π rad | one full turn |
| Resistance scale | 0.002 | `tsa_integration.py` `RESISTANCE_SCALE` |
| TSA torque demand | 10% of full demand | `sts_ctrl.py` line 1056 |

### Moment arm note (important)

`d = 0.03 m` is a **placeholder**. The physical system has:
- Motors mounted anteriorly on the thigh
- Cable routed through tubes down the anterior shin
- Attachment point on the anterior calf, ~0.28 m below the knee centre

The 0.28 m is the attachment distance, NOT the moment arm. The effective moment
arm is the **perpendicular distance from the knee joint axis to the cable line of
action**, which for an anterior routing is approximately the anterior offset of
the cable from the joint centre (~3–6 cm). Use d = 0.03 m for now; verify from
CAD/measurements before running parameter sweeps.

---

## Changes Made This Session (v1 → v2)

### 1 — Fix A removed: EOM throughout (`actuator.py`)

**History:** Fix A was introduced when F_resist ≈ 2 N (old formula), and at the
contraction wall the stall branch suddenly gave T = τ_stall/J_max = 25 N — a 10×
spike. Fix A frozen the pre-wall tau_avail to prevent this.

**Why it was wrong:** With the current RESISTANCE_SCALE formula F_resist ≈ 50 N,
the "spike" at wall becomes 18.7 N → 31.3 N (67% increase), which is physically
correct: the motor hits a hard stop and delivers its stall torque. Fix A was
artificially capping tension at 18.7 N, preventing the correct peak.

**Physics:** At the wall (θ̇ = 0, θ̈ = 0 forced by RK4):
- tau_to_hold = J_max × F_resist = 0.00407 × 50 = 0.204 N·m > τ_stall = 0.1275 N·m
- tau_cmd = clip(T_des × J_max, 0, τ_stall) = 0.1275 N·m
- tau_cmd < tau_to_hold → stall branch fires: T = τ_stall / J_max = **31.3 N** ✓

**Fix:** Remove Fix A entirely. The tension block is now two branches only:

```python
tau_to_hold = self._J * joint_resistance_force

if tau_cmd <= tau_to_hold + 1e-9:
    # Motor stalled (includes at-wall case): T = tau_cmd / J
    T_dynamic = tau_cmd / self._J if self._J > 1e-12 else 0.0
else:
    # Motor advancing: EOM gives dynamic tension
    _, T_dynamic, _, _, _, _, _ = self.tsa._eom(
        self._theta, self._theta_dot, tau_eval, F_ext_override=joint_resistance_force
    )
```

**Result:** During wind-up EOM fires (inertia-dominated, T = m·Ẍ + F_resist ≈ 15–30 N
rising). At the wall stall branch fires: T = τ_stall/J_max = 31.3 N (correct peak).
Tension now naturally peaks at max contraction — the correct TSA characteristic.

**Why RESISTANCE_SCALE cannot be removed:** The user hypothesised that
RESISTANCE_SCALE was only needed because d = 0.28 m was wrong. This is incorrect.
Without scaling: F_resist = (τ_bias + τ_constr) / d = 700 N·m / 0.03 m = 23 333 N.
At pretension J₀ = 0.000201 m/rad: J₀ × F_resist = 4.7 N·m >> τ_stall = 0.1275 N·m.
Motor cannot move at all; EOM branch never fires. RESISTANCE_SCALE physically
represents the fraction of body-weight torque the cable must overcome (muscles take
the rest), and is needed regardless of moment arm.

### 2 — TSA demand scaled to 10% (`sts_ctrl.py` line 1056)

```python
return 0.1 * _demand(obs["knee_r"]), 0.1 * _demand(obs["knee_l"])
```

Controller demand: `tau = clip(10.0 × max(0, knee_angle − 0.08), 0, 20.0) × 0.1`
TSA is zero in Phase 1, active in Phases 2–4.

### 3 — Clean shutdown and reproducible seed (`base_code.py`)

`env.reset(seed=0)` fixes the starting pose. Main loop wrapped in
`try/except KeyboardInterrupt/finally` so Ctrl+C always calls `sts.close()`
(flushes CSV) and `env.close()`.

### 4 — F_resist formula revised (`tsa_integration.py`)

**History of F_resist decisions:**

| Approach | F_resist | Problem |
|----------|----------|---------|
| `bias + constraint` (original) | ~2500 N | body-weight contact dominates → motor permanently stalled |
| `bias + constraint + muscle` (v1 Bug 3 fix) | ≈ 0 N | muscles cancel everything → cable never under tension |
| `SCALE × (bias + constraint)` (current) | ~50 N | motor correctly stalled, delivers T = T_des |

**Rationale for current approach:** During STS with foot contact, the cable
resistance is the full body-weight load transmitted through the knee (qfrc_bias +
qfrc_constraint). This is physically correct but at d = 0.03 m gives ~23 000 N
which no motor can advance against. `RESISTANCE_SCALE = 0.002` brings it to
~50 N — above the motor's stall-tension ceiling (~31 N at max J), so the motor
correctly enters the stall branch and delivers `T = T_des` throughout.
`qfrc_actuator` (muscle force) is excluded: including it drives F_resist to 0.

```python
RESISTANCE_SCALE = 0.002
tau_net = RESISTANCE_SCALE * max(0.0, tau_bias + tau_constr)
F_resist = tau_net / moment_arm
```

### 5 — Moment arm updated from 0.28 m → 0.03 m (`tsa_integration.py`)

At d = 0.28 m:
- Required contraction: 0.28 × 1.57 = 440 mm >> 150 mm (impossible)
- Required motor speed: (0.28/0.004) × θ̇_knee >> 60.7 rad/s (impossible)

At d = 0.03 m:
- Required contraction: 0.03 × 1.57 = 47 mm ✓ (within 150 mm)
- Motor reaches ~35–40 rad/s and hits wall cleanly ✓

### 6 — Plot y-axis clipping (`plot_tsa_log.py`)

Y-axes now use 1st–99th percentile of each signal so the single-step
initialisation spike on the left leg (t ≈ 0.05 s, contact transient) does not
compress the scale and hide the overall trend.

---

## Current Behaviour (latest log)

| Quantity | Value | Notes |
|----------|-------|-------|
| Peak cable tension | ~30–35 N | tracks T_des = tau_demand / d |
| Peak torque delivered (pre-wall) | ~0.5–0.9 N·m | below tau_demand; EOM branch |
| Torque delivered (post-wall) | = tau_demand exactly | stall branch T = T_des |
| Contraction at wall | 150 mm | both motors hit wall at t ≈ 2.82–2.83 s |
| Motor speed at wall-hit | ~24 rad/s | tau_avail locked at ~0.076 N·m |
| Effective payload mass | ~530–570 kg | M_kk / d² at d = 0.03 m — see below |

---

## Known Inconsistencies / Outstanding Issues

### Issue 1 — Pre-wall torque is significantly below tau_demand (most important)

**Symptom:** During the motor wind-up phase (t = 0–2.82 s), torque delivered
tracks well below the demanded torque (e.g., 0.49 N·m vs 1.55 N·m at t = 0.04 s).
`sat_r = 0` throughout, so this is NOT motor torque saturation.

**Root cause:** The stall branch (`T = T_des`) only fires when
`tau_cmd ≤ J × F_resist`. At small J (near pretension, J ≈ 0.0002 m/rad),
`tau_to_hold = J × F_resist` is small. `tau_cmd = T_des × J` is also small but
often exceeds `tau_to_hold`, so the **EOM branch fires** and tension is set by
cable dynamics (`T ≈ F_resist + m·Ẍ`), not by demand. The delivered torque only
converges to tau_demand after the motor has spun up and the stall branch reliably
fires.

**Status:** Unresolved. The EOM branch is physically correct but produces
sub-demand torque during wind-up. The transition to clean tracking happens
naturally as J grows (motor winds the string) and contraction increases.

**Potential fix (not applied):** Force the stall branch throughout by removing the
EOM branch entirely — replace with `T = tau_cmd / J` always. This is less
physically rigorous but ensures `torque = tau_demand` from the first step.

### Issue 2 — Effective payload mass is unrealistically large

`payload_mass = M_kk / d²`. At d = 0.03 m and M_kk ≈ 0.48 kg·m²:
`payload_mass ≈ 533 kg`.

This dominates the EOM inertia term (`D_θ = I_motor + m·J²`) and inflates the
inertial cable tension during wind-up. It does not cause incorrect post-wall
behaviour (stall branch ignores m), but it distorts the EOM dynamics pre-wall.

At d = 0.03 m, reflecting the full knee inertia to cable space via `M_kk / d²`
is likely inappropriate — the cable doesn't couple to the full knee inertia at
this moment arm. The effective cable-side inertia should probably be estimated
from the shin/foot segment inertia alone (~0.25 kg·m² / 0.03² = 278 kg), or
the inertia reflection formula reconsidered for the specific routing geometry.

**Status:** Unresolved. Revisit once d is confirmed from hardware geometry.

### Issue 3 — Hard motor speed clamp at wall

`theta_dot` drops from ~24 rad/s to 0 in a single 10 ms timestep when the
contraction wall is hit. In a real system this would be a damaging mechanical
impact. The simulation does not model any deceleration ramp or compliance.

**Status:** Accepted for now. Would need a compliant wall model (spring-damper
at X = X_max) for realistic impact dynamics.

### Issue 4 — RESISTANCE_SCALE is a tuning hack, not physics

`RESISTANCE_SCALE = 0.002` was chosen empirically to keep F_resist ≈ 50 N.
Once the real moment arm d is confirmed, this should be rederived or replaced
with a physical model of what fraction of the body-weight load the cable faces.

### Issue 5 — Initialisation spike on left leg at t ≈ 0.05 s

A single-step contact transient causes F_resist_l to jump to ~8 N, briefly
firing the stall branch and delivering full T_des for one step. Visible in the
raw CSV; suppressed in plots by percentile y-axis scaling. Harmless but
asymmetric.

---

## Next Steps (Priority Order)

1. **Confirm d from hardware geometry.** Measure the perpendicular offset of the
   cable from the knee joint axis in the physical or CAD model. Update
   `MOMENT_ARM` and re-tune `RESISTANCE_SCALE`.

2. **Fix pre-wall torque tracking (Issue 1).** Evaluate whether to replace the
   EOM branch with a stall-branch-only model (`T = tau_cmd / J` always) to
   guarantee `torque = tau_demand` from first step.

3. **Revisit payload_mass formula (Issue 2).** At d = 0.03 m the reflected
   inertia is ~533 kg. Consider using shin-segment inertia directly instead of
   reflecting M_kk through d.

4. **4-motor string length parameter sweep.** Planned but not started. Sweep L
   over a range and evaluate contraction, speed, and torque delivery. Use the
   confirmed d value.

5. **Torque demand scaling.** Currently 10% of full demand as a safe starting
   point. Once mechanics are validated, increase toward 50–100%.

---

## Key Code Locations

| Concept | File | Key lines |
|---------|------|-----------|
| Tension branch (stall / EOM) | `tsa_modelling/actuator.py` | `step()` ~lines 168–189 |
| RK4 with hard wall stop | `tsa_modelling/actuator.py` | `_rk4_step()` ~lines 270–305 |
| Moment arm constant | `ctrl_optim/tsa_integration.py` | `MOMENT_ARM` line 52 |
| Resistance scale constant | `ctrl_optim/tsa_integration.py` | `RESISTANCE_SCALE` line 60 |
| F_resist formula | `ctrl_optim/tsa_integration.py` | `_get_resistance()` ~line 207 |
| Payload mass update per step | `ctrl_optim/tsa_integration.py` | `step()` line 184 |
| TSA torque injection | `ctrl_optim/tsa_integration.py` | `step()` line 195 |
| TSA demand (10% scale) | `ctrl_optim/sts_ctrl.py` | line 1056 |
| TSA controller params | `ctrl_optim/sts_ctrl.py` | lines 293–295 |
| Plot percentile y-axis | `plot_tsa_log.py` | inside axis loop ~line 63 |

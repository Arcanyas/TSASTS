# TSA Exoskeleton STS — Handoff Document v3

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
- `logs/tsa_log_v7.csv` — most recent run log

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

---

## Tension Branch Logic (final — `actuator.py` `step()`)

Three mutually exclusive cases, evaluated after RK4:

### (a) At hard wall (`X >= X_max`)

On the **first** step hitting the wall, the pre-wall tension is latched into
`_wall_tension`. Subsequent at-wall steps hold that value constant:

```python
if at_wall:
    if not self._was_at_wall:
        self._wall_tension = self.last_tension   # latch on transition
    T_dynamic = self._wall_tension if tau_desired > 1e-12 else 0.0
```

**Why:** In a real TSA the string stays wound once fully contracted. Tension is
held by the wound geometry, not by motor command. Reducing torque demand does
not unwind the string — tension remains constant until demand is entirely zero
(motor actively stopped) or the string is back-driven.

**No spike:** latching `last_tension` (the final pre-wall EOM value) ensures
a smooth continuation rather than jumping to `τ_stall/J_max`.

State variables added to `__init__` and `reset()`:
- `self._was_at_wall = False`
- `self._wall_tension = 0.0`
- `self._was_at_wall` updated at end of each step: `self._was_at_wall = at_wall`

### (b) Motor stalled by load before wall (`tau_cmd ≤ J·F_resist`)

```python
T_dynamic = tau_cmd / self._J
```

### (c) Motor advancing — EOM branch

```python
_, T_dynamic, _, _, _, _, _ = self.tsa._eom(
    self._theta, self._theta_dot, tau_eval,
    F_ext_override=joint_resistance_force,
)
```

**Note on pre-wall torque tracking:** During wind-up J is small, so
`tau_to_hold = J × F_resist` is small. `tau_cmd` often exceeds it, so the EOM
branch fires and T is inertia-dominated (`m·Ẍ ≈ 14 N >> F_resist ≈ 1 N`).
Delivered torque converges to demand only as J grows toward wall. This is
physically correct but produces sub-demand torque early. See Outstanding Issues.

---

## Key Code Locations

| Concept | File | Lines |
|---------|------|-------|
| Tension branch (wall / stall / EOM) | `tsa_modelling/actuator.py` | `step()` ~168–208 |
| Wall latch state | `tsa_modelling/actuator.py` | `__init__` lines 100–101, `reset` |
| RK4 with hard wall stop | `tsa_modelling/actuator.py` | `_rk4_step()` ~280–315 |
| Moment arm constant | `ctrl_optim/tsa_integration.py` | `MOMENT_ARM` line 52 |
| Resistance scale constant | `ctrl_optim/tsa_integration.py` | `RESISTANCE_SCALE` line 60 |
| F_resist formula | `ctrl_optim/tsa_integration.py` | `_get_resistance()` |
| Payload mass update per step | `ctrl_optim/tsa_integration.py` | `step()` — `act.tsa.m = M_kk / d²` |
| Cable-slack (X_geometric) check | `ctrl_optim/tsa_integration.py` | `step()` — after `act.step()` |
| TSA torque injection | `ctrl_optim/tsa_integration.py` | `step()` — `qfrc_applied[dadr] -= result['torque']` |
| TSA demand function | `ctrl_optim/sts_ctrl.py` | `_tsa_knee_torque_demand()` line 1036 |
| TSA demand 10% scale | `ctrl_optim/sts_ctrl.py` | line 1056 |
| TSA controller params | `ctrl_optim/sts_ctrl.py` | lines 293–295 |
| Plot percentile y-axis | `plot_tsa_log.py` | inside axis loop ~line 63 |

---

## What Changed This Session (v2 → v3)

### 1 — At-wall tension: latch-and-hold (final behaviour)

**History of at-wall tension decisions:**

| Version | Behaviour | Problem |
|---------|-----------|---------|
| v1 (Fix A) | Froze `tau_avail` at pre-wall speed | Capped T at 18.7 N (wrong peak) |
| v2 (remove Fix A) | 3-branch: at_wall → `T = tau_cmd / J` | Tension tracked demand and tapered — wrong for fixed string |
| v2b | at_wall → `T = min(T_des, τ_stall/J)` | Dropped toward zero as demand fell — physically wrong |
| v2c | at_wall → `T = τ_stall/J` while demand > 0 | Correct constant, but **spike** from pre-wall EOM to stall ceiling |
| **v3 (current)** | Latch `last_tension` on wall contact, hold constant | No spike, physically correct constant tension |

### 2 — No other changes from v2

All other mechanics (RESISTANCE_SCALE, X_geometric slack check, EOM throughout
pre-wall, clean shutdown, reproducible seed) remain exactly as described in
HANDOFF_v2.md.

---

## Outstanding Issues (unchanged from v2)

### Issue 1 — Pre-wall torque below tau_demand (most important)

During wind-up (EOM branch), delivered torque tracks well below demanded torque
because the EOM is inertia-dominated (`m·Ẍ` at m = 533 kg, Ẍ ≈ 0.026 m/s²
gives ~14 N vs F_resist ≈ 1 N). The stall branch (`T = T_des`) only fires when
`tau_cmd ≤ J·F_resist`, which happens reliably only near and at the wall.

**Potential fix (not applied):** Force stall branch throughout by removing EOM
branch entirely — `T = tau_cmd / J` always. Less physically rigorous but
guarantees `torque = tau_demand` from step 1.

### Issue 2 — Effective payload mass is unrealistically large

`payload_mass = M_kk / d²`. At d = 0.03 m and M_kk ≈ 0.48 kg·m²:
`payload_mass ≈ 533 kg`. This inflates inertial tension during wind-up.
Post-wall behaviour is unaffected (wall latch branch ignores m).

Consider using shin-segment inertia (~0.25 kg·m²) / d² directly rather than
reflecting the full knee DOF inertia.

### Issue 3 — Hard motor speed clamp at wall

`theta_dot` drops from ~24 rad/s to 0 in a single 10 ms timestep. A real system
would need a compliant wall model (spring-damper at X = X_max).

### Issue 4 — RESISTANCE_SCALE is a tuning hack, not physics

`RESISTANCE_SCALE = 0.002` was chosen empirically. Should be rederived once the
real moment arm d is confirmed from hardware.

### Issue 5 — Initialisation spike on left leg at t ≈ 0.05 s

Single-step contact transient causes F_resist_l to jump briefly, firing the
stall branch for one step. Suppressed in plots by percentile y-axis scaling.

---

## Next Steps (Priority Order)

1. **Confirm d from hardware geometry.** Measure perpendicular offset of the
   cable from the knee joint axis. Update `MOMENT_ARM` and re-tune
   `RESISTANCE_SCALE`.

2. **Fix pre-wall torque tracking (Issue 1).** Evaluate stall-branch-only model
   (`T = tau_cmd / J` always) vs keeping EOM for physical accuracy.

3. **Revisit payload_mass formula (Issue 2).** Replace `M_kk / d²` with
   shin-segment inertia directly.

4. **4-motor string length parameter sweep.** Sweep L over a range; evaluate
   contraction, speed, and torque delivery. Use confirmed d value.

5. **Increase TSA torque demand.** Currently capped at 10% of full demand.
   Once mechanics are validated, increase toward 50–100%.

---

## F_resist Design Summary (do not simplify without reading this)

| Approach | F_resist | Problem |
|----------|----------|---------|
| `bias + constraint` (original) | ~2500 N | body-weight contact → motor permanently stalled |
| `bias + constraint + muscle` | ≈ 0 N | muscles cancel load → cable never under tension |
| `SCALE × (bias + constraint)` (current) | ~50 N | motor correctly stalled, delivers T = T_des |

`RESISTANCE_SCALE` is NOT removable by fixing d. Without it:
- F_resist = 700 N·m / 0.03 m = 23 333 N
- J₀ × F_resist = 4.7 N·m >> τ_stall = 0.1275 N·m
- Motor cannot move at all; EOM branch never fires.

`RESISTANCE_SCALE` physically represents the fraction of body-weight torque the
cable must overcome (muscles bear the rest). It is load-sharing, not a hack per
se, but its value (0.002) is empirical and should be re-derived from hardware.

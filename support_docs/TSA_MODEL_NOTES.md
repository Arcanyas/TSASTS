# TSA Model — Logic, Geometry, and Tension Branching

This document traces the full computational path from MuJoCo joint state to
injected knee torque, layer by layer.  The goal is to identify where assumptions
may break down and verify the branching logic.

---

## 1 · String Kinematics (`model_v2.py`)

### 1.1 Forward kinematics — contraction X(θ)

A single twisted string of untwisted length L and bundle radius r contracts when
the motor winds by angle θ from its untwisted state:

```
X(θ) = L − √(L² − θ²r²)          [eq. 1.3]
```

Key properties:
- X = 0 when θ = 0 (untwisted, no contraction)
- X increases with θ (more winding → more shortening)
- X → L asymptotically as θ → L/r (geometric singularity)
- Hard cap: `X_max = max_contraction_ratio × L` (30% of L by default)

**Current parameters (L = 0.5 m, r = 0.004 m):**

| θ (rad) | turns | X (mm) | J (mm/rad) |
|---------|-------|--------|-----------|
| 2π (pretension was here) | 1 | 0.6 | 0.20 |
| 20π (pretension now) | 10 | 68 | 2.33 |
| wall | ~89 | 150 | 4.08 |

The Jacobian grows significantly with winding — this is why pretension matters.

### 1.2 Task-space Jacobian J(X)

J maps motor angular velocity to cable contraction velocity:
```
Ẋ = J · θ̇

J(X) = r · √[(2L − X) · X] / (L − X)   [Section 1.4]
```

Properties:
- J = 0 at X = 0 (no contraction → no mechanical advantage)
- J increases monotonically with X (more wound → faster contraction per unit RPM)
- J → ∞ as X → L (geometric singularity, hit before this by X_max cap)

**Implication:** A motor starting from low X is slow and has no mechanical
advantage.  Pretensioning to a high X₀ is essential for useful contraction rates.

### 1.3 Jacobian rate J̇

```
J̇ = (L²r²) / (L−X)³ · (1/J) · Ẋ
```

Used in the EOM to account for the changing gear ratio while the motor is
spinning.

---

## 2 · Motor Torque-Speed Curve (`model_v2._tau_available`)

A DC motor with linear torque-speed characteristic:

```
τ_avail(θ̇) = τ_stall · max(0,  1 − |θ̇| / ω₀)
```

| Parameter | Value | Source |
|-----------|-------|--------|
| τ_stall (max_motor_torque) | 0.1668 N·m | MP motor spec: 1.7 kg·cm |
| ω₀ (no_load_speed) | 83.8 rad/s | 800 RPM |

At θ̇ = 0: τ_avail = τ_stall = 0.1668 N·m (maximum torque, motor stalled)
At θ̇ = ω₀: τ_avail = 0 (no-load, zero torque, maximum speed)

**Implication:** There is a direct trade-off — the motor produces maximum torque
only when stationary.  As it spins up, available torque falls linearly.  A motor
running near no-load speed produces near-zero tension.

---

## 3 · Equations of Motion (`model_v2._eom`)

### 3.1 Coupled system

Two coupled equations govern the motor+payload system (eq. 1.32):

```
Motor:    I · θ̈  =  τ_cmd  −  J · T  −  b_θ · θ̇
Payload:  T       =  m · Ẍ  +  F_ext  +  b_X · Ẋ
Kinematics: Ẍ    =  J · θ̈  +  J̇ · θ̇
```

where:
- I = motor rotational inertia (5×10⁻⁵ kg·m²)
- m = cable-side payload mass (updated from MuJoCo M diagonal per step)
- F_ext = external force opposing contraction (= F_resist, the joint resistance)
- b_θ, b_X = viscous damping coefficients

### 3.2 Eliminating the algebraic loop

Substituting Ẍ and T into the motor equation:

```
(I + m·J²) · θ̈  =  τ_cmd  −  J·(m·J̇·θ̇ + F_ext + b_X·J·θ̇)  −  b_θ·θ̇
```

The effective inertia is `D_θ = I + m·J²`.  This is explicit — no iteration
needed.  Once θ̈ is known, T follows immediately:

```
T  =  m · (J·θ̈ + J̇·θ̇)  +  F_ext  +  b_X · J · θ̇
T  =  max(T, 0)   ← strings cannot push
```

**Critical observation:** T depends linearly on F_ext.  If F_ext ≈ 0
(no resistance), then at steady state (θ̈ ≈ 0, θ̇ ≈ constant):

```
T_steady  ≈  b_X · Ẋ  ≈  0    (b_X = 0 in current config)
```

**A motor with no external load generates near-zero string tension.**
This is physically correct (a loose string cannot be under tension if nothing
is pulling the other end) but has major implications for the simulation — see
Section 6.

---

## 4 · Integration Layer (`actuator.py` — `TSAActuator.step`)

Each call to `step(t, dt, desired_tension, joint_resistance_force)` executes:

### Step 1 — Convert demand to torque
```
τ_desired  =  T_des × J(current)
```
Virtual work principle: to produce tension T_des, the motor must produce torque
J×T_des.

### Step 2 — Clamp to motor capability
```
τ_cmd  =  clip(τ_desired,  0,  τ_avail(θ̇))
```
In `full_power` mode, `T_des = 1×10⁶ N` (sentinel), so this always gives
`τ_cmd = τ_avail` — the motor runs at maximum torque for its current speed.

### Step 3 — RK4 integration
Advances (θ, θ̇) forward by dt using the EOM.  Constraints enforced:
- `θ ≥ θ_pretension` (no unwinding below pre-tension)
- `θ̇ ≥ 0` (one-way drive only)
- If `X(θ) ≥ X_max`: θ̇ clamped to 0 (hard wall)

### Step 4 — Synchronise kinematics
```
X    = _contraction(θ_new)
J    = _jacobian(X)
Ẋ    = J · θ̇
J̇   = _jacobian_dot(J, X, Ẋ)
```

### Step 5 — Tension branching (see Section 5)

### Step 6 — Knee torque
```
knee_torque  =  T_dynamic × d_eff
```

---

## 5 · Tension Branching Logic (Step 5 of `step()`)

This is the most complex part.  After RK4 advances θ, there are **three
mutually exclusive cases** for computing the achieved string tension:

```
at_wall     ← X ≥ X_max − ε
tau_to_hold ← J · F_resist          (torque needed just to hold the load stationary)
load_stalled ← τ_avail ≤ tau_to_hold
```

### Case A — Motor at hard wall (`at_wall = True`)

The RK4 has clamped θ̇ = 0.  Motor cannot wind further.

**Formula (after the recent fix):**
```
τ_stall  =  τ_avail(0)  =  max_motor_torque   (full stall torque at θ̇ = 0)
T        =  τ_stall / J
```

This gives the maximum tension the motor can hold statically at the wall.

For current parameters at X = 150 mm (wall, L = 0.5 m):
```
J_wall   = 4.08 mm/rad
T_wall   = 0.1668 / 0.00408  =  40.9 N  per motor
τ_knee   = 40.9 × 0.045      =  1.84 N·m per motor
4 motors: 4 × 1.84            =  7.36 N·m per leg
```

> **Before the fix:** wall tension was latched at `last_tension` on first wall
> contact.  Because the motor typically arrives at the wall after free-winding
> with F_resist ≈ 0 (cable slack during the run-up), `last_tension ≈ 0`.
> This caused permanent zero torque even though the cable was taut.

### Case B — Load stall (`tau_cmd ≤ tau_to_hold`)

Motor torque is insufficient to advance against the external load (even at
zero speed).  Motor holds position.

```
T  =  τ_cmd / J
```

This is a static equilibrium: whatever torque the motor is producing is
balanced by the cable tension acting at radius J.

**When this fires:** F_resist is large enough that even at stall torque the
motor cannot move forward.  In our STS setup this rarely fires because
F_resist = RESISTANCE_SCALE × (bias + constraint) / d is very small.

### Case C — Motor advancing (EOM branch)

Motor is moving (θ̇ > 0) and not at the wall, and has enough torque to
overcome the load.

```
T  =  m · (J·θ̈ + J̇·θ̇)  +  F_resist  +  b_X · Ẋ
```

**The critical problem:** This is the dominant branch during free-winding.
If F_resist ≈ 0 (which happens when the cable is slack — the integration
layer correctly passes f_res = 0 for slack motors), then:

```
T  ≈  m · Ẍ  (inertial term only)
```

At steady-state winding speed (Ẍ → 0), tension → 0.  Even during acceleration,
T is only the inertial term needed to accelerate the cable-side mass.  With
m_eff = M_kk / d² (from MuJoCo inertia), this can be non-zero briefly, which
explains the initial spike in the tension plot — but it decays once θ̇ stabilises.

---

## 6 · Geometric Slack Check (`tsa_integration_full.py`)

This is a **second, independent layer** of slack detection applied after the
actuator physics have run.

### 6.1 Geometric contraction threshold

As the knee extends from its seated angle θ_seated by Δθ (decrease in knee
angle), the cable path on the anterior knee shortens by:

```
ΔX_path  =  MOMENT_ARM · Δθ  =  0.045 · (θ_seated − θ_current)
```

The motor must have contracted at least this much (plus the pretension offset)
for the cable to remain taut:

```
X_geom  =  X₀  +  0.045 · max(0,  θ_seated − θ_current)
```

where X₀ = `_contraction(θ_pretension)` is the contraction at the pretension
angle.

### 6.2 Slack decision

```python
if result['X'] < X_geom:      # strict: X == X_geom means taut
    result['tension'] = 0.0
    result['torque']  = 0.0
```

If the motor's actual contraction has not kept pace with the geometric demand,
the cable is slack — the string has excess length on the anterior knee and
cannot transmit force regardless of what the motor is doing internally.

`X == X_geom` is the taut boundary: the cable is just taut with zero excess
length, so tension can exist.  Using `<` (strict) keeps this case in the taut
branch.

### 6.3 Load decision for next step

```python
is_slack_now = motor.actuator.X < X_geom   # strict: taut at X == X_geom
f_res = 0.0 if is_slack_now else F_resist_per
```

If the cable was slack at the **start** of this timestep, the motor has nothing
to pull against, so F_resist = 0 is passed to the actuator.  This correctly
allows the motor to free-spin toward ω₀ (the physically correct behaviour for
an unloaded motor).  A motor exactly at X_geom is taut and receives full
F_resist — it should not free-spin.

**Important:** the check uses the state from the **previous** timestep.  There
is a one-step lag: the first step after the cable becomes taut still passes
f_res = 0, and the first step after going slack still passes f_res = F_resist.
At dt = 0.01 s this is acceptable.

---

## 7 · F_resist — Role and Current Value

```
F_resist_total  =  RESISTANCE_SCALE × max(0, τ_bias + τ_constraint) / d_nominal
F_resist_per    =  F_resist_total / N_active
```

| Symbol | Description |
|--------|-------------|
| τ_bias | Gravity + Coriolis torque at the knee DOF (from MuJoCo qfrc_bias) |
| τ_constraint | Constraint forces including GRF at knee DOF (qfrc_constraint) |
| RESISTANCE_SCALE | 0.002 (dimensionless scale factor) |
| d_nominal | 0.045 m |

**At peak seated load:** τ_bias + τ_constraint ≈ 100–500 N·m.
F_resist_total = 0.002 × 300 / 0.045 ≈ **13 N**.

This is the resistance force each active motor sees on the cable side.

**Implication for tension:**  In Case C (advancing motor), at steady state:
```
T_steady  =  F_resist_per  ≈  13 / N_active   N
```
For N_active = 1: T ≈ 13 N → knee torque ≈ 13 × 0.045 = 0.6 N·m.  Small.

The motor only produces meaningful tension when it is **stalled against the
load** (Case B) or **at the hard wall** (Case A).  The EOM advancing branch
produces tension proportional to F_resist, which is small by design.

---

## 8 · End-to-End Sequence for One STS (what the simulation actually does)

**Assumptions:** L = 0.5 m, 20π pretension (X₀ = 68 mm), full_power mode.

| Time | Event | Tension | Reason |
|------|-------|---------|--------|
| t = 0 | M0 activates. X = X_geom = 68 mm. `is_slack_now = True`. | 0 | First step: f_res = 0 (slack check on prev state, equal boundary treated as slack) |
| t = dt | X_motor > X_geom (motor wound one step). `is_slack_now = False`. | EOM branch: T ≈ m·Ẍ + F_resist | Motor accelerating, brief inertial spike |
| t ≈ 0.05–0.1 s | Motor near ω₀. Ẍ → 0. | T ≈ F_resist ≈ 13 N | Steady state advancing — small but non-zero |
| t ≈ 0.2–0.3 s | Knee starts extending rapidly. X_geom rising at ~100+ mm/s. Motor at ~195 mm/s max. If peak knee velocity exceeds motor capability, X_motor < X_geom. | 0 | Geometric slack. f_res → 0. Motor free-spins. |
| t ≈ 0.33 s | Motor hits wall (X = 150 mm). `at_wall = True`. X_geom ≈ 100 mm < 150 mm. Slack check: 150 > 100 → taut. | **40.9 N** (after fix) | Case A: T = τ_stall / J_wall. Motor held at stall torque. |
| t = 0.33 s → STS end | X_motor = 150 mm. X_geom rises to ≤ 147 mm. Always: X_motor > X_geom. | **40.9 N sustained** | Case A maintained. Full stall torque delivered. |
| t = 0.5 s | M1 activates. X = 68 mm, X_geom ≈ 110 mm. Slack. | 0 | M1 free-spins from 68 mm toward wall (~0.33 s to reach wall). |
| t ≈ 0.83 s | M1 hits wall. STS ≈ complete. | 40.9 N (briefly) | Too late to contribute much. |

---

## 9 · Identified Issues and Their Status

### Issue 1 — Wall tension latched at zero [FIXED in actuator.py]

**Was:** `wall_tension = last_tension` latched on first wall contact.
Motor arrives at wall while free-winding (F_resist ≈ 0) → `last_tension ≈ 0`
→ permanent zero torque at wall even when cable is taut.

**Now:** `T_wall = τ_stall / J` computed fresh at the wall every step.
Correct physics: stalled motor exerts stall torque; tension = τ_stall / J.

### Issue 2 — Stagger timing vs STS window

Default t_stagger = 0.5 s means:
- M0 activates t = 0.0 s, hits wall t ≈ 0.33 s → useful ✓
- M1 activates t = 0.5 s, hits wall t ≈ 0.83 s → STS ends ≈ 0.8 s → marginal
- M2 activates t = 1.0 s → after STS ✗
- M3 activates t = 1.5 s → after STS ✗

Only M0 (and barely M1) contribute during the STS.  The PPO should learn to set
all activation times to ≤ 0.3 s (before the motor hits the wall).

### Issue 3 — F_resist very small → EOM tension very small

RESISTANCE_SCALE = 0.002 was chosen to prevent motor stall.  The trade-off:
in the advancing (EOM) branch, T_steady ≈ F_resist ≈ 13 N per motor → only
0.6 N·m per motor.  The motor only delivers full torque (Case A/B) when stalled.

**Consequence:** the motor's useful torque is only delivered at the wall.  The
advancing phase produces negligible torque.  This is not a bug — it is the
physics of TSAs — but it means the timing of wall contact is critical.

### Issue 4 — Shorter strings do not help

For any string length L, the geometric demand from the knee is fixed:
```
X_geom_rise  =  0.045 × 1.75  ≈  79 mm   (independent of L)
max_contraction  =  0.3 × L
Max safe X₀  =  0.3L − 79 mm
```

Shorter L → smaller max_contraction → less room for pretension → lower X₀ →
lower starting J → slower contraction rate.  L = 0.5 m with X₀ = 68 mm (20π
pretension) is already near-optimal for this motor.

### Issue 5 — Peak knee velocity may briefly exceed motor rate

At peak knee extension (≈ 4–5 rad/s), X_geom rises at 180–225 mm/s.
Motor maximum: J × ω₀ = 2.33 × 83.8 = 195 mm/s at pretension.
Motor falls briefly behind at peak → goes slack → free-spins to wall.
After the wall tension fix (Issue 1), this slack episode is benign: the motor
hits the wall and delivers 40.9 N from then on.

---

## 10 · Parameters Quick Reference

```
L                    = 0.50 m
r                    = 0.004 m   (4 mm radius)
pretension_theta     = 20π rad  (10 turns)
X₀                   = 68 mm
max_contraction_ratio = 0.30
max_contraction      = 150 mm
τ_stall              = 0.1668 N·m   (MP motor 1.7 kg·cm)
ω₀                   = 83.8 rad/s  (800 RPM)
MOMENT_ARM (d)       = 0.045 m    (4.5 cm anterior knee)
RESISTANCE_SCALE     = 0.002
N_motors_per_leg     = 4   (M0–M3)
t_stagger (default)  = 0.5 s
```

```
X_geom(knee_angle)  =  X₀  +  0.045 · max(0,  1.75 − knee_angle)
X_geom_at_seated    =  68 mm
X_geom_at_standing  =  68 + 0.045 × 1.75  =  147 mm   (< 150 mm wall ✓)
T_wall (per motor)  =  τ_stall / J_wall  =  0.1668 / 0.00408  =  40.9 N
τ_knee (per motor)  =  40.9 × 0.045  =  1.84 N·m
τ_knee (4 motors)   =  4 × 1.84      =  7.36 N·m per leg
```

# TSA Exoskeleton STS — Simulation Observations

Data source: `logs/tsa_log_v7.csv`

---

## Observation 1 — Cable slack during early extension phase

### What was seen
In the torque delivered plot (and cable tension plot), the actuator shows:
1. A brief spike in torque/tension at the very start (~t = 0.03–0.10 s)
2. An abrupt drop to **zero** at t ≈ 0.11 s
3. A long dead zone of zero assistance lasting ~0.62 seconds (t = 0.11–0.73 s)
4. A sudden jump to ~1 N·m torque / 33.6 N tension at t ≈ 0.74 s

### Root cause
The exoskeleton cable becomes **geometrically slack** during early knee extension.

The slack condition is defined as:

```
X_wound < X_geometric
where X_geometric = d × max(0, θ_knee_initial − θ_knee_current)
```

`X_wound` is the total string contraction from motor winding. `X_geometric` is the cable shortening the routing geometry demands as the knee extends from its initial seated angle. If `X_wound < X_geometric`, the cable is slack and delivers zero tension.

### Data confirming this

| Time (s) | Knee angle (rad) | X wound (mm) | X geometric (mm) | State |
|----------|------------------|--------------|------------------|-------|
| 0.10 | 1.554 | 0.72 | 0.48 | TAUT — tension = 18.6 N |
| 0.11 | 1.517 | 0.74 | 1.58 | **SLACK — tension = 0** |
| 0.73 | 1.262 | 9.23 | 9.24 | SLACK — tension = 0 |
| 0.74 | 1.260 | 9.51 | 9.30 | **TAUT — tension = 33.6 N** |

Between t = 0.11 s and t = 0.74 s the motor is continuously winding (~35 rad/s), but the knee extends faster than the motor can shorten the cable.

### Physical interpretation
The motor winds at approximately 4–5 mm/s of cable shortening (35 rad/s × small Jacobian), but the knee rotation demands cable shortening at ~13 mm/s during the rapid early extension phase. The cable therefore runs slack and provides no assistive force until the motor has wound enough cable to catch up with the knee geometry.

The "sudden spike" at t = 0.74 s is not a numerical artefact — it is the cable transitioning from completely slack to fully taut in a single timestep (10 ms), after which the EOM delivers full tension.

### Design implication
The TSA provides **zero assistance for the first ~0.7 s of the extension phase**, which is when the knee torque demand is highest (demand ≈ 1.4 N·m at t = 0.11 s vs ≈ 1.18 N·m at t = 0.74 s). This is a parameter mismatch, not a control failure.

Potential remedies:
- **Increase pretension angle** (more pre-wound turns) to give the cable a larger initial buffer
- **Reduce string length L** (shorter L → faster contraction per motor radian → smaller Jacobian lag)
- **Increase moment arm d** raises X_geometric and worsens slack; decreasing d reduces the geometric demand and could reduce slack onset
- **Motor speed optimisation** — the 4-motor string length parameter sweep (next step in project) should be evaluated against this slack metric

---

## Observation 2 — Pre-wall torque tracks well below demand (EOM branch)

### What was seen
Once the cable becomes taut (t > 0.74 s), delivered torque rises to ~1 N·m and stays roughly constant while the motor winds toward max contraction. However, the **demanded torque at this point is ~1.18 N·m**, so the delivered torque underperforms by ~15–20%.

### Root cause
During motor wind-up the EOM (equation of motion) branch fires, not the stall branch. The stall branch (`T = T_desired`) only activates when the motor load torque exceeds its available torque (`J × F_resist ≥ τ_avail`), which happens reliably only at the contraction wall. Before the wall, cable tension is determined by inertial dynamics:

```
T = m·Ẍ + F_resist
```

With the effective payload mass m ≈ 533 kg (reflected from MuJoCo's knee inertia diagonal M_kk / d²), the inertial term dominates and the delivered tension follows cable dynamics rather than the commanded demand.

### Design implication
Closed-loop tension control (feedback on actual cable tension or knee torque) would be needed to guarantee torque tracking during wind-up. The current open-loop command structure cannot enforce `T = T_desired` while the motor is advancing.

---

## Observation 3 — Constant tension hold at max contraction

### What was seen
Once the motor reaches the maximum contraction wall (X = 150 mm, t ≈ 2.8–2.9 s), tension is held constant at the value present just before wall contact, rather than spiking or decaying.

### Implementation
On first wall contact, the pre-wall tension (the last EOM-computed value) is latched and held constant as long as any torque demand exists:

```python
if not self._was_at_wall:
    self._wall_tension = self.last_tension   # latch on transition
T_dynamic = self._wall_tension if tau_desired > 1e-12 else 0.0
```

Tension releases to zero only when the controller fully removes demand (`tau_desired → 0`).

### Physical interpretation
In a real TSA the string remains wound once fully contracted. The geometric twist state holds the cable at a fixed length regardless of the motor command level. Reducing the motor command does not unwind the string; the string only releases with active back-drive or demand fully removed. The latch-and-hold behaviour replicates this.

---

## Observation 4 — Effective payload mass is unrealistically large

### What was seen
The payload mass used in the TSA equation of motion is computed as:

```
m_eff = M_kk / d²
```

where M_kk is the (knee, knee) diagonal of MuJoCo's joint-space inertia matrix (~0.48 kg·m²) and d = 0.03 m. This gives **m_eff ≈ 533 kg**, which is physically unrealistic for the shin/foot segment (~3–5 kg physically).

### Effect
The large m inflates the inertial tension term during wind-up (`m·Ẍ` dominates over `F_resist`). This is responsible for the initial ~18 N tension spike at t = 0.03 s (EOM inertial surge when motor starts from rest). Post-wall behaviour is unaffected because the wall latch branch does not use m.

### Note
Reflecting the full knee DOF inertia through d = 0.03 m is likely physically incorrect — the cable does not couple to the full knee inertia at a 30 mm moment arm. A better estimate would use the shin/foot segment inertia alone (~0.25 kg·m²), giving m_eff = 0.25 / 0.03² ≈ 278 kg (still large but more defensible). Confirmed d from hardware geometry is required before revisiting this.

---

## Observation 5 — Left-leg initialisation transient at t ≈ 0.05 s

### What was seen
A single-step spike in `F_resist_l` (left leg joint resistance force) at t ≈ 0.05 s causes the stall branch to fire briefly, delivering full `T = T_desired` for one timestep. This is visible as an asymmetry between left and right legs in the early tension plot.

### Root cause
A contact transient in MuJoCo's `qfrc_constraint` on the left side at ground contact initialisation. The spike in `F_resist_l` momentarily satisfies the stall condition. It is suppressed in the log plots using 1st–99th percentile y-axis scaling and is functionally harmless.

---

## Observation 6 — Motor winding speed is insufficient to track knee geometry during extension

### What was seen
The TSA motor delivers significant torque (~4.45 Nm per side) only during Phase 1 (lean-forward, knee nearly stationary). By Phase 2, delivered torque drops to ~0.13 Nm, and by Phase 3 it is zero. This is not a timing or pretension issue — it is a fundamental speed mismatch.

### Root cause
The geometric demand rate during active knee extension is:

```
ẋ_geom = r × dθ/dt ≈ 0.045 m × 1 rad/s ≈ 45 mm/s
```

The motor winds cable at approximately **12 mm/s**. Because `v_motor < ẋ_geom`, the cable path shortens faster than the motor can compensate, and the cable goes slack as soon as the knee begins extending rapidly (Phase 2 onset).

### Phase 1 is the only viable window
During Phase 1 (lean-forward), `dθ/dt ≈ 0` — the knee is nearly stationary. The geometric demand rate is negligible, so the pretension (X₀ ≈ 0.63 mm) is sufficient to keep the cable taut and deliver torque against a static knee load.

### Design implication
The motor winding speed must exceed `r × dθ/dt ≈ 45 mm/s` to deliver torque during Phase 2–3. This is a hardware constraint that cannot be resolved by tuning string length L or activation timing. Until motor speed is increased or the cable routing geometry is changed (reducing effective r), Phase 1 is the only assistive window available.

---

## Observation 7 — Late motor activation cannot shift torque delivery into Phase 2

### What was seen
Activating motors later (during Phase 2) does not produce torque during Phase 2. The motor arrives in Phase 2 with zero headroom and a geometric deficit that grows faster than it can close.

### Proof
For a motor activated at `t_activate`, the time-to-taut `τ` satisfies:

```
τ = r × Δθ(t_activate) / (v_motor − r × dθ/dt)
```

where `Δθ(t_activate)` is the knee extension already accumulated since seat-off. During Phase 2, `dθ/dt ≈ 1 rad/s`, so:

```
denominator = 12 − 45 = −33 mm/s  (negative)
```

A negative denominator means **no finite positive τ exists** — the motor can never catch up with the geometry once Phase 2 extension is underway. Activating later also increases `Δθ(t_activate)` (larger initial gap), making the problem strictly worse.

### Conclusion
Activation timing cannot be used to shift torque delivery from Phase 1 into Phase 2. Timing parameters (t0–t3) provide no gradient over the muscle relief reward because the motor's taut window is fixed by the kinematics, not by when it is switched on.

---

## Observation 8 — Muscle relief reward (R_muscle) has no gradient over TSA parameters

### What was seen
Grid search over 25 configurations (5 × L, 5 × t_stagger) with `w_muscle = 0.5` yields:

| Signal | Range across full grid |
|--------|----------------------|
| R_torque | 0.017 (weak gradient over L, none over t_stagger) |
| R_muscle | 0.000033 (floating-point noise — no gradient) |

`extract_muscle_log.py` confirms the physical cause: the TSA delivers torque only in Phase 1 when baseline quad activation is ~0.075. Phase 2–3, where activation peaks at ~0.43, receives essentially zero assistance. The resulting muscle reduction is **0.15%** of baseline — below any usable gradient.

### Consequence for PPO
Setting `w_muscle > 0` adds a near-constant offset (~−0.013) to every reward without introducing any gradient over the action space. This adds noise to the value function without helping the policy. `w_muscle` should remain at 0 until the motor speed limitation is resolved and the TSA can deliver meaningful torque during Phase 2–3.

### What useful R_muscle signal would look like
If the TSA reduced Phase 2 quad activation from 0.43 toward 0.30 (a ~30% reduction), R_muscle would vary by ~0.1 across configurations and provide actionable gradient. This requires `v_motor > 45 mm/s` or a different mechanical arrangement.

---

## Parameter summary at time of observations

| Parameter | Value |
|-----------|-------|
| String length L | 0.50 m |
| Moment arm d | 0.03 m (placeholder) |
| Max contraction X_max | 150 mm |
| Motor stall torque | 0.1275 N·m |
| Motor no-load speed | 60.7 rad/s |
| Pretension | 2π rad (1 turn) |
| RESISTANCE_SCALE | 0.002 |
| TSA demand scale | 10% of full demand |
| Effective payload mass | ~533 kg |

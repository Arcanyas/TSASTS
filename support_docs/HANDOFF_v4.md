# TSA Exoskeleton STS — Handoff Document v4

## Stage Summary

v4 picks up from the validated 4-motor-per-leg simulation (v3 mechanics, full staggered
activation, `tsa_integration_full.py`) and moves into **RL-based parameter optimisation**.
The optimiser will find the best:

1. **String length L** — one scalar, shared across all 8 motors (4 per leg)
2. **Motor activation times** — up to 8 scalars, one per motor per leg

The reflex-based STS controller (`sts_ctrl.py`) is **not retrained** — it stays fixed.
The RL policy only tunes TSA hardware parameters.

---

## What Is Being Optimised

### Parameter vector

```
θ = [L, t_r0, t_r1, t_r2, t_r3, t_l0, t_l1, t_l2, t_l3]
```

| Symbol | Meaning | Current default | Suggested bounds |
|--------|---------|-----------------|-----------------|
| L      | Untwisted string length [m] | 0.50 | [0.25, 0.80] |
| t_r0–t_r3 | Activation times for right leg M0–M3 [s] | 0.0, 0.5, 1.0, 1.5 | [0.0, 3.0] each |
| t_l0–t_l3 | Activation times for left leg M0–M3 [s] | 0.0, 0.5, 1.0, 1.5 | [0.0, 3.0] each |

**Dimensionality reduction option:** if left/right symmetry is assumed, share
`t_ri = t_li` and reduce to 5 parameters. Preferred for early experiments.

### Why L matters

Shorter L → larger Jacobian J = dX/dθ at any given θ → more contraction per
motor radian → faster cable take-up → less slack dead-zone early in extension.
But shorter L also reduces X_max (since X_max = 0.30 × L), so the motor hits the
wall earlier and saturates sooner. The optimum L trades off slack penalty vs
wall-contact timing.

See `tsa_modelling/model_v2.py` for the exact kinematics:

```
X(θ) = L - sqrt(L² - (r·θ)²)
J(θ) = r²·θ / sqrt(L² - (r·θ)²)
```

---

## Where Parameters Live in Code

### String length L

- Stored in `STSReflexParams.tsa_string_length` (`sts_ctrl.py` line 302)
- Passed to `TSAIntegrationFull.__init__` via `sim, L=self.params.tsa_string_length`
  (`sts_ctrl.py` line 395)
- Propagates to every `TSASimulator` via `_build_tsa_sim(id, L)` in
  `tsa_integration_full.py` line 176–190

### Activation times

- Produced by `build_default_motor_configs(t_stagger, alpha_deg)` in
  `tsa_integration_full.py` line 144–168
- The four `MotorConfig.activation_time` values are: `0·t_stagger, 1·t_stagger,
  2·t_stagger, 3·t_stagger`
- Currently uniform — both legs receive the same four configs (`sts_ctrl.py` line 394)
- `MotorUnit.try_activate(t)` checks `t >= activation_time` each step —
  activation is one-way until `full_reset()` is called

---

## Required Code Changes for RL

### 1 — Make `SitToStandSim` accept θ at construction time

Add a `tsa_motor_configs` keyword to `SitToStandSim.__init__` so the RL loop can
pass per-run motor configs without modifying `STSReflexParams`:

```python
# sts_ctrl.py — proposed change
def __init__(self, sim, env, params=None, debug=True,
             use_tsa=False, use_tsa_full=False,
             tsa_motor_configs=None):  # NEW — list of 4 MotorConfig or None

    ...
    if use_tsa_full:
        configs = tsa_motor_configs if tsa_motor_configs is not None \
                  else build_default_motor_configs(t_stagger=0.5, alpha_deg=8.0)
        self.tsa = TSAIntegrationFull(sim, L=self.params.tsa_string_length,
                                      motor_configs=configs, control_mode='full_power')
```

And accept L from `params.tsa_string_length` (already there) — so the RL loop
sets `params.tsa_string_length = L_candidate` before constructing `SitToStandSim`.

### 2 — Per-leg activation times

The current `build_default_motor_configs` returns one list used for **both** legs.
To allow per-leg variation, pass two separate lists:

```python
# tsa_integration_full.py — proposed signature change
class TSAIntegrationFull:
    def __init__(self, sim, L=0.50,
                 motor_configs=None,          # shared (existing)
                 motor_configs_r=None,        # right-specific (NEW)
                 motor_configs_l=None,        # left-specific (NEW)
                 control_mode='full_power'):
        ...
        configs_r = motor_configs_r or motor_configs or build_default_motor_configs()
        configs_l = motor_configs_l or motor_configs or configs_r  # fallback to right
        self.legs = {
            'r': MultiMotorLeg('right', configs_r, L, control_mode),
            'l': MultiMotorLeg('left',  configs_l, L, control_mode),
        }
```

### 3 — Disable CSV logging during RL runs

The per-step CSV write is a bottleneck during rollout collection (thousands of
episodes). Add a `log_to_csv=False` flag to `SitToStandSim.__init__` to skip
opening the file entirely when called from the RL training loop. Retain logging
for evaluation-only runs.

---

## Suggested RL Wrapper Design

```python
# ppo_wrapper.py (new file — sketch, not final)
import numpy as np
from sts_ctrl import SitToStandSim, STSReflexParams
from tsa_integration_full import MotorConfig

class TSAOptimEnv:
    """
    Single-episode rollout env for PPO.

    Action: θ = [L, t_r0..t_r3, t_l0..t_l3]  (9-D continuous)
             or [L, t0..t3]                     (5-D symmetric)
    Episode: reset MuJoCo, run max_steps of fixed STS reflex, return scalar reward.
    """

    BOUNDS_L    = (0.25, 0.80)   # m
    BOUNDS_T    = (0.0,  3.0)    # s  per activation time

    def __init__(self, env_mujoco, symmetric=True, max_steps=5000):
        self.env      = env_mujoco
        self.sym      = symmetric
        self.max_steps = max_steps

    def reset(self, theta):
        L = float(np.clip(theta[0], *self.BOUNDS_L))
        t_raw = np.clip(theta[1:], *self.BOUNDS_T)
        # Build motor configs
        if self.sym:
            t_r = t_l = t_raw[:4]
        else:
            t_r, t_l = t_raw[:4], t_raw[4:]

        cfgs_r = [MotorConfig(off, t, f"M{i}") for i, (off, t) in
                  enumerate(zip([0,0,8,-8], t_r))]
        cfgs_l = [MotorConfig(off, t, f"M{i}") for i, (off, t) in
                  enumerate(zip([0,0,8,-8], t_l))]

        params = STSReflexParams(tsa_string_length=L)
        self.env.reset(seed=0)
        set_seated_pose(self.env.sim)     # same helper as base_code.py

        self.sts = SitToStandSim(
            self.env.sim, self.env,
            params=params, debug=False,
            use_tsa_full=True,
            tsa_motor_configs_r=cfgs_r,
            tsa_motor_configs_l=cfgs_l,
            log_to_csv=False,             # no CSV during training
        )
        self.sts.reset_filters()
        self.sts.get_observation()
        self.sts.capture_phase1_hold_pose()
        self.sts.reset_phase(1)

    def rollout(self):
        total_torque = 0.0
        steps        = 0
        for _ in range(self.max_steps):
            obs = self.sts.get_observation()
            phase = self.sts.get_phase()
            _, _, done, _, _ = self.sts.step(None, phase)
            # Accumulate torque from last_state
            s = self.sts.tsa.last_state
            total_torque += (s['r'].get('torque', 0.0) +
                             s['l'].get('torque', 0.0))
            steps += 1
            if done:
                break
        return total_torque, steps
```

---

## Reward Function Design

The reward should capture **useful assistance** rather than raw torque, since a
motor that fires too early or at max contraction doesn't help.

### Recommended: integral of delivered torque during high-demand phase

```
R = ∫ min(τ_delivered(t), τ_demand(t)) dt
```

This is maximised when the TSA tracks demand closely during Phase 2–3 of STS
(the extension phase where knee torque demand is highest). Capped at demand to
prevent reward hacking from over-torquing.

**Implementation:** accumulate `min(result['torque'], tau_demand)` per step for
both legs, multiply by `dt` (= 0.01 s per MuJoCo step).

### Optional penalty terms

| Penalty | Reason |
|---------|--------|
| `−λ · slack_fraction` | Penalise motors that activate before cable is taut |
| `−λ · N_motors_at_wall` | Penalise premature wall contact (wasted capacity) |
| `−λ · (L − L_ref)²` | Regularise L near mechanical safe range |

Start without penalties; add only if the policy exploits obvious failure modes.

### What to avoid

- **Total torque** as sole reward — drives activation at t=0 for all motors,
  always, because the integral is trivially maximised by early, maximal
  contraction (whether or not it helps the user during peak demand).
- **Episode success flag** — too sparse, gradient vanishes early in training.

---

## Key Implementation Notes

### Resetting between episodes

`SitToStandSim.reset_phase(1)` resets the phase counter but does **not** reset
TSA actuator state. Call `self.sts.tsa.reset()` after `reset_phase(1)` to
re-latch `knee_angle_initial` and clear all motor physics.

The full reset sequence for each episode:

```python
env.reset(seed=0)
set_seated_pose(env.sim)
sts.reset_filters()
sts.get_observation()
sts.capture_phase1_hold_pose()
sts.reset_phase(1)
sts.tsa.reset()           # ← required: re-latches knee_angle_initial, clears X/θ
```

### Fixed seed

`env.reset(seed=0)` ensures reproducible qpos noise — essential for fair
episode-to-episode comparisons during training.

### MuJoCo timestep

Default is **dt = 0.01 s** (100 Hz). All `activation_time` bounds should be
interpreted relative to simulation seconds, not wall-clock time.
A typical STS episode runs ~3–5 s = 300–500 steps.

---

## Physical Constraints to Enforce

| Constraint | Reason |
|------------|--------|
| `t_{i+1} ≥ t_i` (per leg) | Avoid policy discovering reverse-order activation, which has no physical motivation |
| `L ≤ 0.80 m` | String must fit within calf-to-thigh routing; beyond 0.80 m X_max becomes irrelevant within a normal STS range |
| `L ≥ 0.25 m` | Below 0.25 m the motor hits X_max (75 mm at max_contraction_ratio=0.30) before knee reaches standing angle |
| `t_i ≤ 2.5 s` | Phase 3 (main extension) is typically done by ~3 s; activating after that gives no benefit |

Enforce as action clipping or as projected-gradient constraints in the PPO
implementation. Avoid hard rejection (episodic restarts for invalid θ cause high
variance).

---

## Outstanding Issues That Affect RL Quality (from v3)

### Issue 1 — Pre-wall torque under-delivers (most impactful for reward)

During wind-up (EOM branch), delivered torque tracks well below demand because
`m_eff ≈ 533 kg` makes the inertial term dominate. The reward signal will
correctly penalise this, but the policy may compensate by preferring earlier
activation (so more motors are at the wall during peak demand) rather than
fixing the underlying physical under-delivery.

**Action:** Before running RL at scale, evaluate stall-branch-only mode
(`T = tau_cmd / J` always, no EOM). This is toggled by removing the EOM branch
in `actuator.py step()`. It guarantees `τ_delivered = τ_demand` during wind-up
and gives cleaner reward gradients.

### Issue 2 — m_eff = 533 kg artefact (initial tension spike)

Affects M0 at t ≈ 0 s and any motor on first activation. The spike is a
simulation artefact, not physical. During RL rollouts, the spike will not
accumulate significantly into the reward because it lasts < 5 steps, but it
adds noise to tension/torque readouts. Low priority for RL, but avoid reading
per-motor tension at t=0 as a proxy for "motor working correctly."

### Issue 3 — MOMENT_ARM = 0.03 m is a placeholder

All torque values, and therefore all reward magnitudes, scale as d². If d is
later confirmed to be e.g. 0.05 m, the reward scale changes by (0.05/0.03)² ≈ 2.8×.
Normalise the reward by some reference torque (e.g., peak demand ~20 N·m) to
make results portable across d changes.

---

## File Index

| File | Role in RL pipeline |
|------|---------------------|
| `ctrl_optim/sts_ctrl.py` | Fixed STS controller — modify `__init__` to accept `tsa_motor_configs_r/l` and `log_to_csv` |
| `ctrl_optim/tsa_integration_full.py` | 4-motor integration — modify `TSAIntegrationFull.__init__` to accept per-leg configs |
| `ctrl_optim/tsa_integration.py` | Single-motor path — leave untouched |
| `tsa_modelling/actuator.py` | TSA physics — consider stall-branch-only mode for cleaner RL gradients |
| `tsa_modelling/model_v2.py` | TSA kinematics — L enters here; no changes needed |
| `base_code.py` | Manual run entrypoint — unchanged for RL |
| `plot_tsa_log.py` | Evaluation plotting — run after evaluation episodes, not during training |
| `logs/full/tsa_log_full.csv` | Full 4-motor log — disable during training, enable for evaluation |
| *(new)* `ppo_wrapper.py` | RL episode wrapper — to be created in `ctrl_optim/` |
| *(new)* `train_ppo.py` | PPO training loop — to be created at repo root |

---

## Recommended First Experiment

Run a grid search over (L, t_stagger) before PPO to sanity-check the reward
landscape and confirm monotonicity assumptions:

```python
for L in [0.30, 0.40, 0.50, 0.60, 0.70]:
    for t_stagger in [0.0, 0.25, 0.50, 0.75, 1.0]:
        # symmetric 5-param: M0 at 0, M1 at t_stagger, M2 at 2*t_stagger, M3 at 3*t_stagger
        theta = [L, 0, t_stagger, 2*t_stagger, 3*t_stagger]
        env.reset(theta)
        reward, steps = env.rollout()
        print(L, t_stagger, reward, steps)
```

This gives a 5×5 = 25 point landscape without any ML overhead, verifying the
reward signal is informative before committing to PPO.

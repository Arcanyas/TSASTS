# TSA Exoskeleton STS — Handoff Document v5

## Stage Summary

v5 closes the gap between the completed PPO pipeline (v4) and actually running it
productively. Two things happened this session:

1. **Geometric slack model corrected** — both TSA integration files now use the
   physically correct formula derived from `L(θ) = L₀ + r·θ`.
2. **Flat reward landscape diagnosed** — the corrected model is accurate but does
   not resolve the flat landscape; the root cause is that `pretension_theta = 2π`
   gives only X₀ ≈ 0.63 mm of initial contraction, far less than the ~34 mm the
   motor needs to close before it can pull taut during STS.

The PPO pipeline is **fully built and runnable** (`train_ppo.py`, `ppo_wrapper.py`,
`ppo_config.py`). Before launching a real training run, the flat reward landscape
must be resolved so PPO sees a gradient over both L and timing.

---

## Change Made This Session: Geometric Slack Model

### What changed

**`ctrl_optim/tsa_integration_full.py`** — `MultiMotorLeg.step()`, lines ~378–392:

```python
# OLD — used d_eff (torque arm, varies with α); no pretension offset
X_geom = motor.d_eff * max(0.0, knee_angle_initial - knee_angle)

# NEW — L(θ) = L₀ + r·θ; anatomical path radius r = MOMENT_ARM (constant)
X0_motor = motor.actuator.tsa._contraction(motor.actuator.tsa.theta_pretension)
X_geom   = X0_motor + MOMENT_ARM * max(0.0, knee_angle_initial - knee_angle)
if result['X'] <= X_geom:
    result['tension'] = 0.0
    result['torque']  = 0.0
else:
    result['torque'] = result['tension'] * motor.d_eff   # geometry correction
```

**`ctrl_optim/tsa_integration.py`** — `TSAIntegration.step()`, lines ~221–226:

```python
# OLD
X_geometric = d * max(0.0, self._knee_angle_initial[side] - knee_angle)

# NEW
X0          = act.tsa._contraction(act.tsa.theta_pretension)
X_geometric = X0 + d * max(0.0, self._knee_angle_initial[side] - knee_angle)
```

The module-level docstring in `tsa_integration_full.py` (eq. 2 block, lines ~29–38)
was also updated to document the corrected model.

### Why the model is correct

`L(θ) = L₀ + r·θ` describes the cable path on the anterior knee, where r = 0.045 m
is the anatomical radius and θ is flexion (0 = fully extended, 1.75 rad seated).

Calibrated at seated (start of motion), the cable is just taut after pretension:
`X_motor = X₀ = _contraction(2π) ≈ 0.63 mm`. As the knee extends (θ decreases by
Δθ = θ_seated − θ_current), the path shortens by r·Δθ; the motor must wind that
shortening on top of the pretension offset to stay taut.

Hence: **taut iff X_motor ≥ X₀ + r·Δθ**.

Key distinction: the path radius `r = MOMENT_ARM = 0.045 m` is anatomical and
constant for all motors regardless of lateral offset α. The torque arm
`d_eff = MOMENT_ARM × cos(α)` is only used for the torque calculation (eq. 1).

### Why the reward is still flat

At pretension_theta = 2π, X₀ = 0.63 mm. The geometric demand at mid-extension
(Δθ ≈ 0.75 rad) is 0.63 + 0.045 × 0.75 ≈ 34 mm. The motor starts winding from
X = 0.63 mm at activation. At early wind-up speeds (~12 mm/s) it cannot close a
34 mm deficit before Phase 3 ends. Result: `result['X'] ≤ X_geom` for all timing
configs → zero torque, zero reward gradient over t_stagger.

**The grid search confirms this**: reward is identical for all t_stagger > 0
(see `logs/grid_search_results.csv`). Only L shows a weak gradient.

---

## Outstanding Issue: Flat Reward Landscape

The motor must close ~34 mm of cable deficit before it can contribute torque.
This needs a physical fix before PPO is useful. Three options were identified
previously; **none were implemented this session** (explicitly deferred by user):

| Option | Mechanism | Tradeoff |
|--------|-----------|----------|
| **A. Per-motor knee latch** | Each motor latches `knee_angle_initial` at its own activation time, not at seat-off | Rejected by user: physically wrong — string length is fixed, so a later-activated motor cannot suddenly become taut at a shallower angle |
| **B. Free-travel offset** | Add a `cable_slack_offset` (e.g. 30 mm) that the motor must first wind before contributing | No comfort impact while seated; cable is pre-run-out at attachment. Most compatible with fixed-L constraint |
| **C. Increase pretension** | Raise `pretension_theta` from 2π to ~16π → X₀ ≈ 32 mm, bridging the gap | Changes seated comfort (cable pre-tensioned against skin); changes all TSASimulator physics |

**Recommendation before starting PPO**: implement **Option B** (free-travel offset
`cable_slack_offset = 0.030` m) in `tsa_integration_full.py`. Change is one line:

```python
# In MultiMotorLeg.step(), replace:
X_geom = X0_motor + MOMENT_ARM * max(0.0, knee_angle_initial - knee_angle)

# With (free-travel offset added to pretension slack):
CABLE_SLACK_OFFSET = 0.030  # m — cable pre-run-out at calf attachment
X_geom = (X0_motor - CABLE_SLACK_OFFSET) + MOMENT_ARM * max(0.0, knee_angle_initial - knee_angle)
```

This shifts the taut threshold down by 30 mm, so the motor is effectively
already taut at activation and contributes torque from the first step.

---

## PPO Pipeline Status

All files are complete and tested via grid search:

| File | Status |
|------|--------|
| `ctrl_optim/ppo_wrapper.py` | Complete — one-step Gymnasium env wrapping full STS episode |
| `ctrl_optim/ppo_config.py` | Complete — `TSAOptimConfig` with env/reward/PPO sub-configs |
| `train_ppo.py` | Complete — SB3 PPO training loop with checkpoints and TensorBoard |
| `ctrl_optim/grid_search.py` | Complete — 5×5 landscape validation |
| `ctrl_optim/tsa_integration_full.py` | Complete (geometric model updated this session) |
| `ctrl_optim/tsa_integration.py` | Complete (geometric model updated this session) |

### To run a training experiment

```bash
# Quick sanity check (25 episodes, grid search):
mjpython ctrl_optim/grid_search.py

# Full PPO run (default 5000 episodes, ~30 min):
mjpython train_ppo.py

# Override timesteps and output dir:
mjpython train_ppo.py --timesteps 2000 --out logs/ppo_test_v1
```

Outputs land in `logs/ppo_<timestamp>/`:
- `config.json` — the exact config used
- `checkpoints/ppo_tsa_<N>_steps.zip` — SB3 checkpoint every 200 episodes
- `ppo_tsa_final.zip` — best policy at end of training
- `tensorboard/` — reward/loss curves

### Reward function (current)

```
R = w_torque × R_torque  +  w_muscle × R_muscle  +  w_time × R_time

R_torque = −∫|τ_delivered − α·τ_demand| dt  /  (τ_ref × T)
R_muscle = −∫mean_quad_activation dt  /  T
R_time   = −t_final / t_max_episode
```

Current weights (in `ppo_config.py`):
- `w_torque = 1.0`, `w_muscle = 0.0`, `w_time = 0.0`
- `support_fraction α = 0.30` (TSA targets 30% of knee demand)
- `torque_ref = 20.0 N·m`

**Start with just torque reward** (`w_muscle = w_time = 0`). Add muscle and time
terms only after verifying the torque signal has a gradient.

### Action space

```
θ = [L, t0, t1, t2, t3]   (symmetric 5-D, same for both legs)

L  ∈ [0.25, 0.70] m
ti ∈ [0.0,  2.5]  s,   sorted so t0 ≤ t1 ≤ t2 ≤ t3
```

Lateral offsets are fixed at `[0°, 0°, +8°, −8°]` (motor geometry).

---

## Key Physical Constants

| Symbol | Value | Location | Notes |
|--------|-------|----------|-------|
| `MOMENT_ARM` | 0.045 m | `tsa_integration_full.py:109` | Anatomical anterior-knee radius r; used for path geometry AND torque arm (for α=0 motors) |
| `RESISTANCE_SCALE` | 0.002 | `tsa_integration_full.py:112` | Scales raw joint torque to ~50 N cable resistance; keeps motor in stall branch |
| `pretension_theta` | 2π rad | `tsa_integration_full.py:188` | Gives X₀ ≈ 0.63 mm; key parameter behind flat landscape |
| `max_contraction_ratio` | 0.30 | `tsa_integration_full.py:194` | X_max = 0.30 × L; motor saturates past this |
| `r_bundle` | 0.004 m | `tsa_integration_full.py:182` | TSA string bundle radius |

---

## File Index

| File | Role |
|------|------|
| `ctrl_optim/sts_ctrl.py` | Fixed STS reflex controller — **do not modify** for PPO |
| `ctrl_optim/tsa_integration_full.py` | 4-motor integration — geometric model updated v5 |
| `ctrl_optim/tsa_integration.py` | Single-motor integration — geometric model updated v5 |
| `ctrl_optim/ppo_wrapper.py` | Gymnasium env wrapping STS episode |
| `ctrl_optim/ppo_config.py` | All hyperparameters — tune here before training runs |
| `train_ppo.py` | Entry point: `mjpython train_ppo.py` |
| `ctrl_optim/grid_search.py` | Landscape sanity check: `mjpython ctrl_optim/grid_search.py` |
| `tsa_modelling/model_v2.py` | TSASimulator kinematics — `_contraction(theta)` used for X₀ |
| `tsa_modelling/actuator.py` | TSAActuator physics — EOM + stall branches |
| `logs/grid_search_results.csv` | Flat landscape result confirming the pending reward issue |
| `plot_tsa_log.py` | Diagnostic plots — run after evaluation, not during training |

---

## Recommended Next Steps (in order)

1. **Fix the flat landscape** — implement Option B (free-travel offset, one line change
   in `tsa_integration_full.py`) and re-run grid search to confirm gradient appears.
2. **Run PPO smoke test** — `mjpython train_ppo.py --timesteps 200` and inspect
   TensorBoard to confirm reward is moving.
3. **Scale up** — full 5000-episode run with `w_torque=1.0`; check convergence.
4. **Add muscle reward** — once torque reward converges, set `w_muscle=0.2` and
   retrain from scratch to introduce quadriceps-relief objective.
5. **Validate best θ** — extract best action from final model, run with
   `base_code.py`, plot `plot_tsa_log.py` to inspect per-motor torque delivery.

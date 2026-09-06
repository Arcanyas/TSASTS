# TSA Simulation Codebase — Handover Summary

## Overview

This codebase models a **Twisted String Actuator (TSA)** for use as an assistive knee actuator in an exoskeleton sit-to-stand simulation. The physics follow the mathematical models in:

> *"Modeling of Twisted String Actuation Systems"*, Chapter 1, Section 1.4

There are two files: `tsa_model_v2.py` (the physics engine) and `actuator.py` (the controller-facing wrapper). They are designed to be imported into a higher-level `SitToStandSim` controller loop.

---

## File 1: `tsa_model_v2.py` — `TSASimulator`

### Purpose
Low-level TSA physics. Given a motor torque input `τ(t)`, integrates the motor dynamics forward in time and returns all kinematic and dynamic quantities.

### Key equations implemented

**Forward kinematics** (eq. 1.3):
```
X(θ) = L - sqrt(L² - θ²r²)
```

**Task-space Jacobian** (Section 1.4):
```
J(X) = r * sqrt((2L - X) * X) / (L - X)
```

**Equations of motion** (eq. 1.22 / 1.32) — coupled motor + payload, substituted to eliminate algebraic loop:
```
(I + m·J²)·θ̈ = τ_cmd - J·(m·J̇·θ̇ + F_ext + b_X·J·θ̇) - b_θ·θ̇
```

**Dynamic string tension** (eq. 1.32 bottom row):
```
T = m·Ẍ + F_ext + b_X·Ẋ     where Ẍ = J·θ̈ + J̇·θ̇
```

**Motor torque-speed curve** (linear characteristic):
```
τ_available(θ̇) = τ_stall * max(0, 1 - |θ̇| / ω_noload)
```

### Constructor parameters

| Parameter | Type | Description |
|---|---|---|
| `id` | int | Actuator index |
| `L` | float | Untwisted string length [m] |
| `radius` | float | Effective bundle radius [m]. For 2 strings of manufacturer diameter `d`, use `r ≈ 2d/3` |
| `payload_mass` | float | Mass the cable sees [kg]. For exo: `I_leg / moment_arm²` |
| `I_motor` | float | Rotor moment of inertia [kg·m²] |
| `pretension_theta` | float | Initial motor angle [rad]. Avoids kinematic singularity at θ=0 |
| `max_motor_torque` | float | Stall torque [N·m] |
| `no_load_speed` | float | No-load angular speed [rad/s] |
| `b_theta` | float | Motor viscous friction [N·m·s/rad] |
| `b_X` | float | Payload viscous friction [N·s/m] |
| `gravity_along_string` | bool | Whether F_ext = m·g acts along cable. **False for exo**, True for hanging-mass bench test |
| `max_contraction_ratio` | float | Maximum contraction as fraction of L (default 0.30) |

### Key methods

| Method | Returns | Notes |
|---|---|---|
| `simulate(tau_input, t_end, dt)` | dict of arrays | Batch simulation given callable `τ(t)`. Uses RK4 internally |
| `_eom(theta, theta_dot, tau_cmd)` | `(θ̈, T, X, J, Ẋ, J̇, Ẍ)` | Core physics. Takes `tau_cmd` directly — **no internal clamping**. Used by actuator |
| `_contraction(theta)` | float | Forward kinematics |
| `_jacobian(X)` | float | Task-space Jacobian |
| `_jacobian_dot(J, X, X_dot)` | float | Time derivative of J |
| `_tau_available(theta_dot)` | float | Available motor torque at current speed |

### Important design note
`_eom()` does **not** clamp `tau_cmd` internally. The caller is responsible for clamping. This was a deliberate design choice to keep `actuator.py` in full control of the torque command.

---

## File 2: `actuator.py` — `TSAActuator`

### Purpose
Controller-facing wrapper around `TSASimulator`. Exposes a per-step interface that takes `desired_tension` and `joint_resistance_force`, integrates one timestep, and returns knee torque and full state.

### Constructor parameters

| Parameter | Type | Description |
|---|---|---|
| `tsa` | TSASimulator | Configured simulator instance |
| `side` | str | `'right'` or `'left'` |
| `moment_arm` | float | `d` [m]. `knee_torque = T * d` |
| `name` | str | Label for logging |

### `step()` interface

```python
result = actuator.step(
    t,                        # current sim time [s]
    dt,                       # timestep [s]
    desired_tension,          # target cable tension [N]
    joint_resistance_force,   # joint force opposing contraction [N]
                              # = tau_knee_external / moment_arm
)
```

**Returns dict with:**

| Key | Units | Description |
|---|---|---|
| `torque` | N·m | Knee torque = T_dynamic × d |
| `tension` | N | Dynamic string tension T from EOM |
| `tension_cmd` | N | Tension implied by clamped tau_cmd |
| `tau_cmd` | N·m | Actual motor torque sent to EOM |
| `tau_available` | N·m | Motor torque available at current speed |
| `X` | m | String contraction |
| `X_dot` | m/s | Contraction velocity |
| `theta` | rad | Motor angle |
| `theta_dot` | rad/s | Motor angular velocity |
| `J` | m/rad | Task-space Jacobian |
| `torque_saturated` | bool | True if tau_cmd was clamped by motor limit |

### Internal control flow per step

```
1. tau_desired = T_des * J              (virtual work: τ·dθ = T·dX)
2. tau_cmd = clip(tau_desired, 0, tau_available)
3. Override TSA F_ext = joint_resistance_force for this step
4. RK4 integrate motor ODE over dt
5. Recompute X, J, Xdot, Jdot at new state
6. Re-evaluate _eom() to get T_dynamic
7. knee_torque = T_dynamic * moment_arm
```

### Why `tension != desired_tension`

`desired_tension` is a feedforward motor torque target, not a closed-loop guarantee. The achieved dynamic tension `T = m·Ẍ + F_ext + b_X·Ẋ` will differ because:
- Motor inertia means `θ̈` is not instantaneous
- Coriolis and friction absorb some torque
- If `tau_cmd` is clamped, less torque reaches the string

To track tension precisely, close a PID loop around `result['tension']` in your controller.

---

## Physics notes critical for integration

### Gravity convention
The `gravity_along_string` flag determines what `F_ext` means inside `_eom()`:
- `True` → `F_ext = m * g`. Motor must overcome payload weight to contract. Used for **bench tests** with a hanging mass.
- `False` → `F_ext = 0` from the TSA's perspective. The external load is supplied **per-step** via `joint_resistance_force` in `step()`. Use this for **exo**.

### `payload_mass` for the exo
This is **not** a hanging mass — it is the reflected inertia of the leg segment:
```
payload_mass = I_leg / moment_arm²
```
For example: shin+foot inertia ≈ 0.25 kg·m², moment_arm = 0.05 m → `payload_mass = 100 kg`.

### `joint_resistance_force` per step
This is the force the knee joint exerts back on the cable opposing contraction:
```
joint_resistance_force = tau_knee_external / moment_arm
```
Your musculoskeletal model should supply this each timestep from the joint torque. When `T_des > joint_resistance_force`, the cable shortens. When `T_des < joint_resistance_force`, the motor stalls and holds position.

### Kinematic singularity at θ = 0
At zero twisting angle, `J = 0` and the Jacobian is undefined. Always initialise with `pretension_theta > 0` (default: `2π` = 1 full rotation). The textbook recommends 3–5 mm of pre-contraction as a minimum.

### Motor stall behaviour
Stall is **soft saturation**, not a hard freeze. When `tau_available < tau_desired`:
- `tau_cmd` is clamped to `tau_available`
- The motor decelerates naturally under the load
- `torque_saturated = True` in the returned dict
- Recovery is automatic when demand drops

### String radius accuracy
The kinematic model is more sensitive to errors in `radius` than in `L` (see Section 1.1.4, eq. 1.13). For a bundle of two strings with manufacturer diameter `d`:
```
r ≈ 2d/3
```
Consider adaptive radius estimation (eq. 1.17) if you observe kinematic drift over time.

---

## Recommended setup for SitToStandSim

```python
from model_v2 import TSASimulator
from actuator import TSAActuator

sim = TSASimulator(
    id=0,
    L=0.25,                     # string length to optimise later
    radius=0.004,               # 4 mm bundle radius
    payload_mass=100.0,         # I_leg / d^2 = 0.25 / 0.05^2
    I_motor=5e-5,
    pretension_theta=2*np.pi,
    max_motor_torque=0.1275,    # Pololu 9.7:1 LP 12V
    no_load_speed=520.0,
    b_theta=1e-4,
    b_X=0.0,
    gravity_along_string=False, # exo: load supplied per step
    max_contraction_ratio=0.30,
)

actuator = TSAActuator(sim, side='right', moment_arm=0.05)

# Inside your controller loop:
for each timestep:
    F_resist = knee_torque_from_model / actuator.moment_arm
    result = actuator.step(t, dt, desired_tension, F_resist)
    apply result['torque'] to knee joint
```

---

## Planned next steps

- [ ] Optimise `L` across 4 motors per leg for sit-to-stand assist profile
- [ ] Replace fixed `moment_arm` with geometry-dependent `d(joint_angle)`
- [ ] Add closed-loop tension PID around `result['tension']`
- [ ] Adaptive radius estimation (eq. 1.17) for long-duration wear
- [ ] Extend to multi-TSA coordination (4 motors, bilateral)
# TSAExoskeletonSTS Environment Notes

This file is a practical map of this repository: what each folder is for, how to run things on macOS, where custom environments are registered, and what can break.

## 1) Repository at a glance

Top level folders:

- `myoassist/`: Main Python package and working simulation/training code.
- `combined-sts-helpers/`: Helper scripts and notes for integrating custom TSA + torso sit-to-stand (STS) environments.
- `TSAmodel/`: XML model files from project-specific TSA work.
- `torso/`: Torso exosuit XML + assets (mirrors files that are also present under myosuite simhive).
- `exovenv/`: Existing local Python virtual environment in this workspace.

The active development center is `myoassist/`.

## 2) Main technology stack

- Python >= 3.11 (`setup.py` enforces this).
- MuJoCo 3.3.3 (`requirements.txt`).
- `gymnasium==0.29.1` for environment APIs.
- Stable-Baselines3 + PyTorch for RL.
- CMA-ES (`cma`) and custom reflex controllers for optimization.

Core package layout inside `myoassist/`:

- `myosuite/`: bundled/forked MyoSuite code and env registry.
- `rl_train/`: RL environments, training scripts, analyzers, configs.
- `ctrl_optim/`: reflex controller simulation and optimization pipeline.
- `models/`: leg/exoskeleton MuJoCo models and meshes.
- `docs/`: detailed docs for getting started, RL, optimization, and modeling.

## 3) How environments are registered

Environment registration is automatic when `myosuite` is imported.

- `myosuite/__init__.py` imports MyoSuite suites and triggers registration.
- Custom IDs relevant to this project are in:
  - `myoassist/myosuite/envs/myo/myobase/__init__.py`

Custom IDs found:

- `myoTSA-v0`
  - entry point: `myosuite.envs.myo.myobase.base_TSA:BaseTSAV0`
  - model path: `models/mesh/TSAmodel/human_exo.xml`
- `TorsoLegs`
  - entry point: `myosuite.envs.myo.myobase.base_torso:BaseTorso`
  - model path: `myosuite/simhive/myo_sim/torso/myotorso_exosuit.xml` (resolved using `curr_dir` in file)

## 4) Most useful scripts to run

From `myoassist/` directory:

### TSA environment quick run

- `base_code_TSA.py`
  - creates `gym.make('myoTSA-v0')`
  - sets a simple action and steps with render

### Sit-to-stand (torso + legs)

- `base_sts.py`
  - creates `gym.make('TorsoLegs')`
  - builds staged sit-to-stand keyframes (`q_sit` -> `q_stand`)
  - auto-detects free-root z indices and candidate foot sites/bodies
  - lifts root if feet penetrate floor
  - plays STS phases via smoothed interpolation

### RL minimal simulation

- `rl_train/run_sim_minimal.py`
  - directly constructs `MyoAssistLegBase`
  - random actions for short sanity simulation

### RL training entry point

- `rl_train/run_train.py`
  - requires `--config_file_path`
  - loads json config, creates environment/model, trains or realtime evaluates

Example config files:

- `rl_train/train/train_configs/imitation_tutorial_22_separated_net_partial_obs.json`
- `rl_train/train/train_configs/imitation_tutorial_22_separated_net_full_obs.json`

### Reflex controller minimal run

- `ctrl_optim/run_ctrl_minimal.py`
  - random control params
  - short sim
  - prints walking duration

### Reflex evaluation launcher

- `ctrl_optim/run_eval.py`
  - thin launcher into `results/evaluation/eval.py`

## 5) macOS execution notes (important)

For MuJoCo viewer/render on macOS, run scripts with `mjpython` when needed.

This repo docs explicitly call this out, and terminal history here already shows:

- `mjpython base_sts.py` succeeded (exit code 0)

So for rendering-heavy scripts prefer:

- `mjpython base_sts.py`
- `mjpython base_code_TSA.py`
- `mjpython rl_train/run_sim_minimal.py`
- `mjpython rl_train/run_train.py ... --flag_rendering`

## 6) Setup for this workspace

### Option A: use existing virtual environment

There is already `exovenv/` at repository root.

From `TSAExoskeletonSTS/`:

```bash
source exovenv/bin/activate
cd myoassist
pip install -e .
python test_setup.py
```

### Option B: fresh venv

From `myoassist/`:

```bash
python3.11 -m venv .my_venv
source .my_venv/bin/activate
pip install -e .
python test_setup.py
```

`test_setup.py` validates imports, MuJoCo stepping, RL environment init, reflex environment init, and minimal controller script behavior.

## 7) Data and model locations

Primary model roots:

- `myoassist/models/22muscle_2D/`
- `myoassist/models/26muscle_3D/`
- `myoassist/models/80muscle/`
- `myoassist/models/mesh/`
- `myoassist/models/TSAmodel/` (contains TSA-specific XMLs in this workspace)

Torso model used by `TorsoLegs`:

- `myoassist/myosuite/simhive/myo_sim/torso/myotorso_exosuit.xml`
- with related assets in `myoassist/myosuite/simhive/myo_sim/torso/assets/`

## 8) Known quirks and pitfalls

1. Duplicate helper scripts exist.
- `combined-sts-helpers/base_sts.py` and `myoassist/base_sts.py` are effectively same STS script.
- `combined-sts-helpers/base_code.py` and `myoassist/base_code_TSA.py` are similar quick demos.

2. Integration README in `combined-sts-helpers/` appears historical.
- It describes manual copying into MyoSuite paths.
- In this workspace, custom files are already present in the bundled `myoassist/myosuite/...` tree.

3. One suspicious registration block exists in `myobase/__init__.py`.
- ID `myoHandReachRandom-v0` is re-registered with `entry_point='myosuite.envs.myo.base_v0:BaseTSAV0.py'`.
- This looks malformed/out of place (contains `.py` in class path and likely wrong module path).
- Treat as likely stale unless intentionally used.

4. Relative model paths are sensitive to working directory and package layout.
- Prefer running from `myoassist/` for scripts that reference `models/...`.

## 9) Suggested daily workflow

1. Activate environment.
2. `cd myoassist`.
3. Sanity check:
   - `mjpython base_sts.py` (STS env and rendering)
   - `mjpython base_code_TSA.py` (TSA env)
4. For RL experiments:
   - run `rl_train/run_train.py` with one of `rl_train/train/train_configs/*.json`
5. For reflex/controller tests:
   - run `ctrl_optim/run_ctrl_minimal.py` then `run_ctrl.py` / `run_eval.py`.

## 10) Quick command snippets

From `myoassist/`:

```bash
# STS simulation
mjpython base_sts.py

# TSA simulation
mjpython base_code_TSA.py

# RL quick simulation
mjpython rl_train/run_sim_minimal.py

# RL train (example)
mjpython rl_train/run_train.py \
  --config_file_path rl_train/train/train_configs/imitation_tutorial_22_separated_net_partial_obs.json \
  --config.total_timesteps 12 \
  --config.env_params.num_envs 1 \
  --config.ppo_params.n_steps 4 \
  --config.ppo_params.batch_size 4 \
  --config.logger_params.logging_frequency 1 \
  --config.logger_params.evaluate_frequency 1 \
  --flag_rendering

# Reflex minimal
python ctrl_optim/run_ctrl_minimal.py
```

## 11) If something fails

- Run `python test_setup.py` first.
- Confirm you are inside `myoassist/` when running scripts with relative model paths.
- On macOS rendering errors, switch `python` to `mjpython`.
- Check that editable install was done (`pip install -e .`) in the active environment.


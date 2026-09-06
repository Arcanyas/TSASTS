# elbow movement
import sys
from pathlib import Path

MYOASSIST_DIR = Path(__file__).resolve().parent / "myoassist"
if str(MYOASSIST_DIR) not in sys.path:
    sys.path.insert(0, str(MYOASSIST_DIR))

CTRL_OPTIM_DIR = Path(__file__).resolve().parent / "ctrl_optim"
if str(CTRL_OPTIM_DIR) not in sys.path:
    sys.path.insert(0, str(CTRL_OPTIM_DIR))

import myosuite.envs.myo.myobase  # noqa: F401 — registers TorsoLegs
from myosuite.utils import gym
import numpy as np
from sts_ctrl_ari import SitToStandSim

env = gym.make('TorsoLegs')
env.reset()
var = np.zeros(91)
print(var)

import time
# def lerp(a, b, t): return (1.0 - t) * a + t * b
# def deg(x): return np.deg2rad(x)

# def make_pose(q_base, hip_deg, knee_deg, ankle_deg):
#     q = q_base.copy()
#     q[HFR] = deg(hip_deg);  q[HFL] = deg(hip_deg)
#     q[KFR] = deg(knee_deg); q[KFL] = deg(knee_deg)
#     q[AFR] = deg(ankle_deg);q[AFL] = deg(ankle_deg)
#     return q

# q_sit  = make_pose(q0, hip_deg=95,  knee_deg=105, ankle_deg=-10)  # sitting-ish crouch
# q_lean = make_pose(q0, hip_deg=115, knee_deg=105, ankle_deg=-15)  # lean forward (hip hinge)
# q_up   = make_pose(q0, hip_deg=10,  knee_deg=10,  ankle_deg=0)


datap0 = {
    "qpos": np.fromstring("-0.4 0.1 0.7 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -0.4 0.1 0.7 1 0 0 0 0.5 0 0 0 0 1.3 0 0 0 0 0 0 0 0 0.5 0 0 0 0 1.3 0 0 0 0 0 0 0 0", sep=' '),
    "qvel": np.fromstring("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", sep=' '),
    "act": np.fromstring("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", sep=' '),
    "ctrl": np.fromstring("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", sep=' '),
    "site_pos": env.sim.model.site_pos[:].copy(),
    "site_quat": env.sim.model.site_quat[:].copy(),
    "body_pos": env.sim.model.body_pos[:].copy(),
    "body_quat": env.sim.model.body_quat[:].copy() 
}

datap1 = {
    "qpos": np.fromstring("-5.23 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -0.025 0.1 1 0.707388 0 0 -0.706825 1.3 0 0 0 0 1.3 0 0 0 0 0 0 0 0 1.3 0 0 0 0 1.3 0 0 0 0 0 0 0 0", sep=' '),
    "qvel": np.fromstring("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", sep=' '),
    "act": np.fromstring("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", sep=' '),
    "ctrl": np.fromstring("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", sep=' '),
    "site_pos": env.sim.model.site_pos[:].copy(),
    "site_quat": env.sim.model.site_quat[:].copy(),
    "body_pos": env.sim.model.body_pos[:].copy(),
    "body_quat": env.sim.model.body_quat[:].copy() 
}

datap2 = {
    "qpos": np.fromstring("-0.174 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -0.025 0.1 1 0.707388 0 0 -0.706825 0.785398 0 0 0 0 1 0 0 0 0 0 0 0 0 0.785398 0 0 0 0 1 0 0 0 0 0 0 0 0", sep=' '),
    "qvel": np.fromstring("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", sep=' '),
    "act": np.fromstring("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", sep=' '),
    "ctrl": np.fromstring("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", sep=' '),
    "site_pos": env.sim.model.site_pos[:].copy(),
    "site_quat": env.sim.model.site_quat[:].copy(),
    "body_pos": env.sim.model.body_pos[:].copy(),
    "body_quat": env.sim.model.body_quat[:].copy() 
}

datap3 = {
    "qpos": np.fromstring("-0.1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -0.025 0.1 1 0.707388 0 0 -0.706825 0.26 0 0 0 0 0.26 0 0 0 0 0 0 0 0 0.26 0 0 0 0 0.26 0 0 0 0 0 0 0 0", sep=' '),
    "qvel": np.fromstring("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", sep=' '),
    "act": np.fromstring("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", sep=' '),
    "ctrl": np.fromstring("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", sep=' '),
    "site_pos": env.sim.model.site_pos[:].copy(),
    "site_quat": env.sim.model.site_quat[:].copy(),
    "body_pos": env.sim.model.body_pos[:].copy(),
    "body_quat": env.sim.model.body_quat[:].copy() 
}

datap4 = {
    "qpos": np.fromstring("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -0.025 0.1 1 0.707388 0 0 -0.706825 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", sep=' '),
    "qvel": np.fromstring("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", sep=' '),
    "act": np.fromstring("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", sep=' '),
    "ctrl": np.fromstring("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", sep=' '),
    "site_pos": env.sim.model.site_pos[:].copy(),
    "site_quat": env.sim.model.site_quat[:].copy(),
    "body_pos": env.sim.model.body_pos[:].copy(),
    "body_quat": env.sim.model.body_quat[:].copy() 
}

def set_seated_pose(sim):
    model = sim.model
    data = sim.data

    name2jid = {model.joint(i).name: i for i in range(model.njnt)}

    def set_joint(name, value):
        jid = name2jid[name]
        qadr = model.jnt_qposadr[jid]
        data.qpos[qadr] = value

    # Root position relative to human body pos.
    set_joint("root_x", 0.0)
    set_joint("root_z", 0.0)
    set_joint("root_pitch", 0.0)

    # These values are initial guesses; signs may need flipping.
    set_joint("hip_flexion_r", 1.57)
    set_joint("hip_flexion_l", 1.57)

    set_joint("knee_angle_r", 1.57)
    set_joint("knee_angle_l", 1.57)

    set_joint("ankle_angle_r", 0)
    set_joint("ankle_angle_l", 0)

    data.qvel[:] = 0.0
    sim.forward()


data = datap0

count = 0

sts = SitToStandSim(env.sim, env, debug=True)
# sts.settle_initial_contacts()
# print(env.sim.model.geom('pelvis').id)
max_steps = 50000


# set_seated_pose(env.sim)
sts.reset_filters()
sts.get_observation()
sts.capture_phase1_hold_pose()
sts.reset_phase(1)

for step_i in range(max_steps):
    env.mj_render()
    obs = sts.get_observation()
    # sts.debug_contacts()
    phase = sts.get_phase()
    feet_grounded = (
    bool(obs.get("left_foot_contact", False))
    and bool(obs.get("right_foot_contact", False))
    )  
    
    # print(feet_grounded) 
    if phase ==1:

        obs_env, reward, _, _,_ = sts.step(None, phase)
    if phase ==2:
            obs_env, reward, _, _,_ = sts.step(None, phase)
            
    if phase ==3:

        obs_env, reward, _, _,_ = sts.step(None, phase)
    
    # if phase ==4:

    #     obs_env, reward, _, _,_ = sts.step(None, phase)

    
    
    
    # print(u_control)
    # if _ == 1:
    #     data["time"] = env.sim.data.time
    #     env.set_env_state(data)
    # if data["qpos"][25] > data_comp["qpos"][25]: 
    #     # data["qpos"][0] = data["qpos"][0] - 0.001
    #     data["qpos"][25] = data["qpos"][25] - 0.001
    #     data["qpos"][30] = data["qpos"][30] - 0.001
    #     data["qpos"][39] = data["qpos"][39] - 0.001
    #     data["qpos"][44] = data["qpos"][44] - 0.001
    # else: 
    #     count+= 1 
    #     if count == 1:
    #         data = datap1
    #         data_comp = datap2
    #     elif count == 2:
    #         data = datap2
    #         data_comp = datap3
    #     elif count == 3:
    #         data = datap3
    #         data_comp = datap4
    # data["time"] = env.sim.data.time
    # env.set_env_state(data)
    
env.close()


# from myosuite.utils import gym
# import numpy as np

# # ============================================================
# # Environment
# # ============================================================
# env = gym.make("TorsoLegs")
# env.reset()
# sim = env.sim
# model = sim.model
# data = sim.data

# print("model nq  =", model.nq)
# print("model nv  =", model.nv)
# print("model nu  =", model.nu)
# print("model njnt=", model.njnt)
# print("model nsite=", model.nsite)
# print("model nbody=", model.nbody)

# # ============================================================
# # Utilities
# # ============================================================
# def lerp(a, b, t):
#     return (1.0 - t) * a + t * b

# def smoothstep(x):
#     x = np.clip(x, 0.0, 1.0)
#     return 3 * x**2 - 2 * x**3

# def deg(x):
#     return np.deg2rad(x)

# # Joint mapping
# name2jid = {model.joint(i).name: i for i in range(model.njnt)}
# qadr = model.jnt_qposadr

# def has_joint(name):
#     return name in name2jid

# def qidx(name):
#     return int(qadr[name2jid[name]])

# def set_joint(q, name, value):
#     if has_joint(name):
#         q[qidx(name)] = value

# def clamp_limits(q):
#     for i in range(model.njnt):
#         if model.jnt_limited[i]:
#             adr = int(model.jnt_qposadr[i])
#             lo, hi = model.jnt_range[i]
#             q[adr] = np.clip(q[adr], lo, hi)
#     return q

# # ============================================================
# # Parse qpos safely
# # ============================================================
# def parse_qpos_string(qpos_str, nq, label):
#     vals = np.fromstring(qpos_str, sep=" ")
#     if len(vals) == nq:
#         return vals

#     # common copy/paste issue: one extra trailing zero
#     if len(vals) == nq + 1 and abs(vals[-1]) < 1e-12:
#         print(f"[warn] {label}: had {len(vals)} values, trimming trailing zero to match nq={nq}")
#         return vals[:-1]

#     raise ValueError(
#         f"{label} has length {len(vals)}, but model.nq = {nq}. "
#         f"Please fix the qpos string."
#     )

# # ============================================================
# # Your sit / stand qpos
# # NOTE:
# # - Sitting qpos is from your Python snippet
# # - Standing qpos is from your XML keyframe
# # - If one has one extra trailing zero, parser trims it
# # ============================================================
# q_sit_str = """
# -0.4 0.1 0.84 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -0.4 0.1 0.84 0 0 0 0 1.57 0 0 0 0 1.3 0 0 0 0 0 0 0 0 1.57 0 0 0 0 1.3 0 0 0 0 0 0 0 0
# """

# q_stand_str = """
# 0.025 0.1 0.935 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -0.025 0.1 0.935 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
# """

# q_sit = parse_qpos_string(q_sit_str, model.nq, "q_sit")
# q_stand = parse_qpos_string(q_stand_str, model.nq, "q_stand")

# # print("len(q_sit)   =", len(q_sit))
# # print("len(q_stand) =", len(q_stand))

# # ============================================================
# # Root z indices for floating-base lift correction
# # We detect all free joints and store their z positions.
# # For a free joint, qpos layout is [x y z qw qx qy qz].
# # ============================================================
# FREE_JOINT = 0  # MuJoCo mjJNT_FREE

# root_z_indices = []

# print(model.njnt, "joints in model")
# for j in range(model.njnt):
#     if model.jnt_type[j] == FREE_JOINT:
        
#         adr = int(model.jnt_qposadr[j])
#         root_z_indices.append(adr + 2)

# # print("Detected free-joint z indices:", root_z_indices)

# if not root_z_indices:
#     # fallback: assume first free root z is qpos[2]
#     root_z_indices = [2]
#     # print("[warn] No free joint detected from model; falling back to qpos[2] for root z correction")

# # ============================================================
# # Optional torso / lumbar shaping
# # Negative values bend forward in your model
# # ============================================================
# def add_forward_lean(q, torso_deg=None, lumbar_total_deg=None):
#     q = q.copy()

#     if torso_deg is not None and has_joint("flex_extension"):
#         set_joint(q, "flex_extension", deg(torso_deg))

#     if lumbar_total_deg is not None:

#         lumbar_joints = ["L1_L2_FE", "L2_L3_FE", "L3_L4_FE", "L4_L5_FE"]
#         each = deg(lumbar_total_deg) / 4.0
#         for jn in lumbar_joints:
#             if has_joint(jn):
#                 set_joint(q, jn, each)

#     return clamp_limits(q)

# # ============================================================
# # Candidate foot sites / bodies for floor penetration detection
# # Auto-detect by name substrings so you don't have to hardcode blindly
# # ============================================================
# site_names = [model.site(i).name for i in range(model.nsite)]
# body_names = [model.body(i).name for i in range(model.nbody)]


# def find_candidate_sites():
#     wanted = ["heel", "toe", "foot", "ankle", "mtp"]
#     out = []
#     for s in site_names:
#         sl = s.lower()
#         if any(w in sl for w in wanted):
#             out.append(s)
#     return out

# def find_candidate_bodies():
#     wanted = ["talus", "calcn", "foot", "toe", "ankle", "mtp"]
#     out = []
#     for b in body_names:
#         bl = b.lower()
#         if any(w in bl for w in wanted):
#             out.append(b)
#     return out

# foot_site_candidates = find_candidate_sites()
# foot_body_candidates = find_candidate_bodies()



# # ============================================================
# # Foot penetration correction
# # - evaluates current pose
# # - finds lowest foot site/body z
# # - lifts free root(s) if below floor
# # ============================================================
# def get_min_foot_z():
#     min_z = np.inf

#     # Prefer sites if present
#     for s in foot_site_candidates:
#         try:
#             sid = model.site_name2id(s)
#             z = data.site_xpos[sid][2]
#             min_z = min(min_z, z)
#         except Exception:
#             pass

#     # Fallback to body origins if no useful sites
#     if not np.isfinite(min_z):
#         for b in foot_body_candidates:
#             try:
#                 bid = model.body_name2id(b)
#                 z = data.xpos[bid][2]
#                 min_z = min(min_z, z)
#             except Exception:
#                 pass

#     return min_z

# def lift_root_if_feet_penetrate(state, floor_z=0.0, margin=0.003):
#     env.set_env_state(state)
#     sim.forward()

#     min_z = get_min_foot_z()

#     if np.isfinite(min_z) and min_z < floor_z + margin:
#         dz = (floor_z + margin) - min_z
#         for idx in root_z_indices:
#             state["qpos"][idx] += dz

#     return state

# # ============================================================
# # Build 4 STS phase keyframes
# # We interpolate the FULL qpos to include translation.
# # Then we add torso/lumbar shaping for realism.
# # ============================================================
# u1 = 0.20   # end phase 1: momentum forward
# u2 = 0.38   # end phase 2: momentum transfer / seat-off
# u3 = 0.88   # end phase 3: extension
# u4 = 1.00   # end phase 4: stabilisation

# q_phase0 = q_sit.copy()
# q_phase1 = lerp(q_sit, q_stand, u1)
# q_phase2 = lerp(q_sit, q_stand, u2)
# q_phase3 = lerp(q_sit, q_stand, u3)
# q_phase4 = q_stand.copy()

# # Forward-lean shaping for upper body (negative = forward)
# q_phase1 = add_forward_lean(q_phase1, torso_deg=-15, lumbar_total_deg=-12)
# q_phase2 = add_forward_lean(q_phase2, torso_deg=-30, lumbar_total_deg=-20)
# q_phase3 = add_forward_lean(q_phase3, torso_deg=-8,  lumbar_total_deg=-6)
# q_phase4 = add_forward_lean(q_phase4, torso_deg=0,   lumbar_total_deg=0)

# # Optional: keep initial upright sit posture exactly upright in torso
# q_phase0 = add_forward_lean(q_phase0, torso_deg=0, lumbar_total_deg=0)

# # Clamp any limited joints
# for q in [q_phase0, q_phase1, q_phase2, q_phase3, q_phase4]:
#     clamp_limits(q)

# # ============================================================
# # Animation helper
# # ============================================================
# state = env.get_env_state()
# state["qvel"] = np.zeros_like(state["qvel"])

# def play_segment(q_start, q_end, n_frames, floor_z=0.0, margin=0.003):
#     for t in range(n_frames):
#         u = smoothstep(t / (n_frames - 1))
#         state["qpos"] = lerp(q_start, q_end, u)
#         state["qvel"] = np.zeros_like(state["qvel"])
#         arr =  np.zeros(model.na)
#         state["act"] = arr
#         print(arr)

#         # Prevent feet sinking into floor
#         state_copy = {
#             "qpos": state["qpos"].copy(),
#             "qvel": state["qvel"].copy(),
#             "act": state["act"].copy() if "act" in state else np.zeros(model.na),
#             "ctrl": state["ctrl"].copy() if "ctrl" in state else np.zeros(model.nu),
#             "time": state["time"] if "time" in state else 0.0,
#             "site_pos": state["site_pos"].copy() if "site_pos" in state else model.site_pos.copy(),
#             "site_quat": state["site_quat"].copy() if "site_quat" in state else model.site_quat.copy(),
#             "body_pos": state["body_pos"].copy() if "body_pos" in state else model.body_pos.copy(),
#             "body_quat": state["body_quat"].copy() if "body_quat" in state else model.body_quat.copy(),
#         }

#         state_copy = lift_root_if_feet_penetrate(state_copy, floor_z=floor_z, margin=margin)
#         env.set_env_state(state_copy)
#         sim.forward()
#         env.mj_render()

# # ============================================================
# # Run 4-phase sit-to-stand
# # Slowed down a bit
# # ============================================================
# # Start seated
# state["qpos"] = q_phase0.copy()
# state["qvel"] = np.zeros_like(state["qvel"])
# env.set_env_state(state)
# sim.forward()

# # Hold seated briefly
# for _ in range(150):
#     env.mj_render()

# # Phase 1: momentum forward
# play_segment(q_phase0, q_phase1, n_frames=500)

# # Phase 2: momentum transfer / seat-off
# play_segment(q_phase1, q_phase2, n_frames=450)

# # Phase 3: extension
# play_segment(q_phase2, q_phase3, n_frames=1100)

# # Phase 4: stabilisation
# play_segment(q_phase3, q_phase4, n_frames=600)

# # Hold standing
# for _ in range(300):
#     env.mj_render()

# env.close()


# from myosuite.utils import gym
# import myosuite

# print("myosuite version:", getattr(myosuite, "__version__", "unknown"))
# print("myosuite path:", myosuite.__file__)

# # Gymnasium-style registry
# try:
#     ids = sorted([spec.id for spec in gym.envs.registry.values()])
# except Exception:
#     ids = sorted(list(gym.envs.registry.keys()))

# # Show only MyoSuite-ish envs
# myo_ids = [i for i in ids if i.lower().startswith("myo")]
# print("Total myo envs:", len(myo_ids))
# print("\n".join(myo_ids))

# # Filter torso-related
# torso_ids = [i for i in myo_ids if "torso" in i.lower()]
# print("\n--- torso envs ---")
# print("\n".join(torso_ids) if torso_ids else "None found")
# #hand and finger movement 
# import numpy as np
# action = np.ones(71)
# action[-1] = 0
# action[-2] = 0

# from myosuite.utils import gym
# env = gym.make('TorsoLegs')

# env.reset()
# add_term = 0.5
# # env.get_randomized_initial_state()
# # print(env.sim.model.key_qpos[0])
# losses = np.ones(len(action))
# for _ in range(1000000):
#     env.mj_render()
#     # env.step(action)
#     # for i in range (0,len(action)):
#     #     current_loss = abs(env.sim.data.qpos.ravel().copy()[i+7] - env.sim.model.key_qpos[0][i+7])
#     #     if losses[i] > current_loss:
#     #         add_term = add_term*-1
#     #     losses[i] = current_loss
#     #     action[i] = action[i] + add_term
#     # env.get_randomized_initial_state()
# env.close()


# import numpy as np
# action = np.zeros(71)
# action[-1] = 1
# action[-2] = 1
# from myosuite.utils import gym
# env = gym.make('myoLegStandRandom-v0')
# env.reset()
# # env.get_randomized_initial_state()
# for _ in range(10000):
#     env.mj_render()
#     # env.step(action) 
#     # env.get_randomized_initial_state()
# env.close()


# import numpy as np
# action = np.zeros(71)
# action[-1] = 1
# action[-2] = 1
# from myosuite.utils import gym
# env = gym.make('myoLegWalk-vexo')
# env.reset()
# # env.get_randomized_initial_state()
# for _ in range(10000):
#     env.mj_render()
#     # env.step(action) 
#     # env.get_randomized_initial_state()
# env.close()


# import mujoco
# import numpy as np

# xml_path = "/Users/ninastidham/myoassist/myosuite/simhive/myo_sim/torso/myotorso_abdomen.xml"
# m = mujoco.MjModel.from_xml_path(xml_path)
# d = mujoco.MjData(m)

# print("njnt:", m.njnt, "nq:", m.nq, "nu:", m.nu)
# for j in range(m.njnt):
#     print(j, mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j))

# print("\nActuators:")
# for a in range(m.nu):
#     print(a, mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a))
# import numpy as np
# from myosuite.utils import gym
# env = gym.make('myoSarcChallengeChaseTagP1-v0')
# env.reset()
# # env.get_randomized_initial_state()
# for _ in range(10000):
#     env.mj_render()
#     # env.step(env.action_space.sample()) # take a random action
# env.close()


# from myosuite.utils import gym
# import deprl
# from deprl import env_wrappers

# # we can pass arguments to the environments here
# env = gym.make('myoLegWalk-v0')
# env = env_wrappers.GymWrapper(env)
# policy = deprl.load_baseline(env)
# env.reset()
# for i in range(1000):
#     env.mj_render()
#     action = policy(obs)
#     obs, *_ = env.step(action)
# env.close()
# #test policy loading and simulating

# from myosuite.utils import gym
# policy = "iterations/best_policy.pickle"

# import pickle
# pi = pickle.load(open(policy, 'rb'))

# env = gym.make('myoElbowPose1D6MRandom-v0')
# env.reset()
# for _ in range(1000):
#     env.mj_render()
#     env.step(env.action_space.sample()) # take a random action

#muscle fatigue test

# from myosuite.utils import gym
# env = gym.make('myoElbowPose1D6MRandom-v0')
# env.reset()
# for _ in range(1000):
#     env.mj_render()
#     env.step(env.action_space.sample()) # take a random action

# # Add muscle fatigue
# env = gym.make('myoFatiElbowPose1D6MRandom-v0')
# env.reset()
# for _ in range(1000):
#     env.mj_render()
#     env.step(env.action_space.sample()) # take a random action
# env.close()

#Sarcopenia is a muscle disorder that occurs commonly in the elderly population - reduces grip strength and muscle mass in simulation

# from myosuite.utils import gym
# env = gym.make('myoElbowPose1D6MRandom-v0')
# env.reset()
# for _ in range(1000):
#     env.mj_render()
#     env.step(env.action_space.sample()) # take a random action

# # Add muscle weakness
# env = gym.make('myoSarcElbowPose1D6MRandom-v0')
# env.reset()
# for _ in range(1000):
#     env.mj_render()
#     env.step(env.action_space.sample()) # take a random action
# env.close()

#leg move:

# from myosuite.utils import gym
# import deprl
# from deprl import env_wrappers

# # we can pass arguments to the environments here
# env = gym.make('myoLegWalk-v0', reset_type='random')
# env = env_wrappers.GymWrapper(env)
# policy = deprl.load_baseline(env)
# obs = env.reset()
# for i in range(1000):
#     env.mj_render()
#     action = policy(obs)
#     obs, *_ = env.step(action)
# env.close()

'''mjpython python test_moving.py'''

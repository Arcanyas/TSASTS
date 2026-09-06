# """Launch the combined TSA sit-to-stand simulation."""

import os
import signal
import sys
from pathlib import Path

MYOASSIST_DIR = Path(__file__).resolve().parent / "myoassist"
if str(MYOASSIST_DIR) not in sys.path:
    sys.path.insert(0, str(MYOASSIST_DIR))

CTRL_OPTIM_DIR = Path(__file__).resolve().parent / "ctrl_optim"
if str(CTRL_OPTIM_DIR) not in sys.path:
    sys.path.insert(0, str(CTRL_OPTIM_DIR))

import myosuite.envs.myo.myobase  # noqa: F401
from myosuite.utils import gym
import numpy as np
from sts_ctrl import SitToStandSim
env = gym.make('TorsoLegs')
env.reset(seed=0)   # fixed seed → reproducible qpos noise
var = np.zeros(91)
print(var)

import time


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

    set_joint("knee_angle_r", 1.75)
    set_joint("knee_angle_l", 1.75)

    set_joint("ankle_angle_r", -0.15)
    set_joint("ankle_angle_l", -0.15)

    data.qvel[:] = 0.0
    sim.forward()



# use_tsa_full for all 8 motors
# use_tsa for simplified bilateral model
sts = SitToStandSim(env.sim, env, debug=True, use_tsa_full=True, use_tsa=False)

max_steps = 50000


set_seated_pose(env.sim)
sts.reset_filters()
sts.get_observation()
sts.capture_phase1_hold_pose()
sts.reset_phase(1)


def _shutdown(_sig=None, _frame=None):
    # Flush the TSA CSV log before exiting.
    try:
        sts.close()
    except Exception:
        pass
    # os._exit bypasses mjpython's event loop — sys.exit raises SystemExit
    # which mjpython catches and uses to restart the script instead of quitting.
    os._exit(0)


signal.signal(signal.SIGINT,  _shutdown)
signal.signal(signal.SIGTERM, _shutdown)

try:
    for step_i in range(max_steps):
        env.mj_render()
        obs = sts.get_observation()
        phase = sts.get_phase()

        obs_env, reward, _, _, _ = sts.step(None, phase)

except (KeyboardInterrupt, SystemExit):
    pass

_shutdown()


'''mjpython python test_moving.py'''
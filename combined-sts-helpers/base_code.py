# elbow movement
from myosuite.utils import gym
import numpy as np
from ctrl_optim.sts_ctrl import SitToStandSim
env = gym.make('TorsoLegs')
env.reset()
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

    set_joint("knee_angle_r", 1.57)
    set_joint("knee_angle_l", 1.57)

    set_joint("ankle_angle_r", 0)
    set_joint("ankle_angle_l", 0)

    data.qvel[:] = 0.0
    sim.forward()




sts = SitToStandSim(env.sim, env, debug=True)

max_steps = 50000


set_seated_pose(env.sim)
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
            
    # if phase ==3:

    #     obs_env, reward, _, _,_ = sts.step(None, phase)
    
    # if phase ==4:

    #     obs_env, reward, _, _,_ = sts.step(None, phase)

    
env.close()


'''mjpython python test_moving.py'''

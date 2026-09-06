register_env_with_variants(id='myoHandReachRandom-v0',
    entry_point='myosuite.envs.myo.base_v0:BaseTSAV0.py',
    max_episode_steps=100,
    kwargs={
        'model_path': '../../../../models/mesh/TSAmodel/human_exo.xml'
    }
    
)

register_env_with_variants(id='TorsoLegs',
    entry_point='myosuite.envs.myo.base_torso:BaseTorso',
    max_episode_steps=100,
    kwargs={
        'model_path': 'myosuite/simhive/myo_sim/torso/myotorso_exosuit.xml',
        'normalize_act': True,
        'min_height':0.8,    # minimum center of mass height before reset
        'max_rot':0.8,       # maximum rotation before reset
        'hip_period':100,    # desired periodic hip angle movement
        'reset_type':'init', # none, init, random
        'target_x_vel':0.0,  # desired x velocity in m/s
        'target_y_vel':1.2,  # desired y velocity in m/s
        'target_rot': None,   # if None then the initial root pos will be taken, otherwise provide quat
        'terrain':'stairs',
        'variant':'fixed'
    }
)
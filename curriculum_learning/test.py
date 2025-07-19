import gymnasium as gym
from stable_baselines3.ppo import PPO
from gymnasium import Env
import time
import mujoco
import numpy as np

DURATION = 500


xml = 'franka_emika_panda/mjx_single_cube.xml'
rl_model_path = "ppo_reach_model.zip"
cube_link_name = "box"
end_effector_link_name = "hand"
left_finger_link_name = "left_finger"
right_finger_link_name = "right_finger"


model = mujoco.MjModel.from_xml_path(xml)
data = mujoco.MjData(model)

cube_id = model.body(cube_link_name).id
end_effector_id = model.body(end_effector_link_name).id
left_finger_id = model.body(left_finger_link_name).id
right_finger_id = model.body(right_finger_link_name).id

rl_model = PPO.load(rl_model_path)

with mujoco.viewer.launch_passive(model, data) as viewer:
  start = time.time()
  while viewer.is_running() and time.time() - start < DURATION:
    step_start = time.time()

    cube_pose = data.body(cube_id).xpos
    end_effector_pose = data.body(end_effector_id).xpos

    cube_pos = data.body(cube_id).xpos
    ee_pos = data.body(end_effector_id).xpos
    left_finger_pos = data.body(left_finger_id).xpos
    right_finger_pos = data.body(right_finger_id).xpos
        
    obs = np.concatenate([
            data.qpos,
            data.qvel,
            cube_pos,
            ee_pos,
            left_finger_pos,
            right_finger_pos
        ]).astype(np.float32)
    
    action, _states = rl_model.predict(obs, deterministic=True)
    data.ctrl[:] = action

    mujoco.mj_step(model, data)
    viewer.sync()

    time_until_next_step = model.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)

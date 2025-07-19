import os
import time
import mujoco
import mujoco.viewer

MENAGERIE_PATH = "/Users/timii/Developer/repositories/mujoco_menagerie"
robot_path = '/Users/timii/Developer/repositories/mujoco_menagerie/franka_emika_panda/mjx_scene.xml'

model = mujoco.MjModel.from_xml_path(robot_path)
data = mujoco.MjData(model)

DURATION = 10.0

with mujoco.viewer.launch_passive(model, data) as viewer:
  start = time.time()
  while viewer.is_running() and time.time() - start < DURATION:
    step_start = time.time()
    # data.ctrl = ...

    mujoco.mj_step(model, data)
    viewer.sync()

    time_until_next_step = model.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
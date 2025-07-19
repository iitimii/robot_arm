import os
import mujoco
import mujoco.viewer

MENAGERIE_PATH = "/Users/timii/Developer/repositories/mujoco_menagerie"
robot_path = os.path.join(MENAGERIE_PATH, 'boston_dynamics_spot/scene_arm.xml')

model = mujoco.MjModel.from_xml_path(robot_path)
viewer = mujoco.viewer.launch(model)
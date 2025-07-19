import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("box.xml")
viewer = mujoco.viewer.launch(model)
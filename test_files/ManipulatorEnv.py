# import gymnasium as gym
# import numpy as np
# import transforms3d as tf3d
# import pybullet as p
# import pybullet_data
# import time

# class ManipulatorEnv(gym.Env):
#     def __init__(self, manipulator="kuka_iiwa/model.urdf"):
#         # observations = rgb image, xyz, joint angles
#         physicClient = p.connect(p.GUI)

#         # Options
#         p.setAdditionalSearchPath(pybullet_data.getDataPath())
#         p.setGravity(0, 0, -9.8)
#         p.setRealTimeSimulation(1)
#         # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # Optionally disable PyBullet's GUI overlay
#         # p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)  # Enable shadows

#         # Load Models
#         self.plane_id = p.loadURDF("plane.urdf")
#         table_top_height = 0.625
#         self.table_id = p.loadURDF("table/table.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))
#         self.box1_id = p.loadURDF("models/box.xml", [0.4, 0.23, table_top_height + 0.01], p.getQuaternionFromEuler([0, 0, 0]))
#         self.box2_id = p.loadURDF("models/box.xml", [-0.4, 0.23, table_top_height + 0.01], p.getQuaternionFromEuler([0, 0, 0]))
#         self.manipulator_id = p.loadURDF("kuka_iiwa/model.urdf", [0, -0.35, table_top_height], p.getQuaternionFromEuler([0, 0, 0]))   
#         self.num_manipulator_joints = p.getNumJoints(self.manipulator_id)  

#         self.circular_segment_id = p.loadURDF("models/wooden_blocks/circular_segment.urdf", [0.4, 0.23, table_top_height + 0.1])
#         self.cube_id = p.loadURDF("models/wooden_blocks/cube.urdf", [0.4, 0.23, table_top_height + 0.1])
#         self.cuboid0_id = p.loadURDF("models/wooden_blocks/cuboid0.urdf", [0.4, 0.23, table_top_height + 0.1])
#         self.cuboid1_id = p.loadURDF("models/wooden_blocks/cuboid1.urdf", [0.4, 0.23, table_top_height + 0.1])
#         self.cylinder_id = p.loadURDF("models/wooden_blocks/cylinder.urdf", [0.4, 0.23, table_top_height + 0.1])
#         self.traingle_id = p.loadURDF("models/wooden_blocks/triangle.urdf", [0.4, 0.23, table_top_height + 0.1])

#     def step(self, action):
#         joint_states = p.getJointStates(self.manipulator_id, range(self.num_manipulator_joints))
#         joint_positions = [state[0] for state in joint_states]

#         target_joints_pose = p.calculateInverseKinematics(self.manipulator_id, self.num_manipulator_joints-1, action)
#         p.setJointMotorControlArray(self.manipulator_id, range(self.num_manipulator_joints), p.POSITION_CONTROL, target_joints_pose)


#         p.stepSimulation()
#         time.sleep(1. / 240.)
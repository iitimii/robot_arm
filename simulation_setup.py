import pybullet as p
import pybullet_data
import time

# Set the URDF file for the robot arm
robot_arm_urdf = r"robot_arm_new\robots_arm.urdf"  # Replace with the path to your robot arm URDF file

urdf_file = r"robot_arm_new\box.urdf"  # Your box file

# Initialize PyBullet simulation
physics_client = p.connect(p.GUI)

# Set the additional search path for PyBullet data
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity
p.setGravity(0, 0, -9.8)

# Load the ground plane
plane_id = p.loadURDF("plane.urdf")

# Set table top height
table_top_height = 0.625

# Load the table
table_position = [0, 0, 0]
table_orientation = p.getQuaternionFromEuler([0, 0, 0])
table_id = p.loadURDF("table/table.urdf", table_position, table_orientation)

# Load the first box on the table
box1_position = [0.2, 0, table_top_height + 0.05]
box1_orientation = p.getQuaternionFromEuler([0, 0, 0])
box1_id = p.loadURDF(urdf_file, box1_position, box1_orientation)

# Load the second box on the table
box2_position = [-0.4, 0, table_top_height + 0.05]
box2_orientation = p.getQuaternionFromEuler([0, 0, 0])
box2_id = p.loadURDF(urdf_file, box2_position, box2_orientation)

# Load the robot arm on the table
robot_arm_position = [0, 0, table_top_height + 0.1]  # Adjust the height of the robot arm
robot_arm_orientation = p.getQuaternionFromEuler([0, 0, 0])  # Set the orientation
robot_arm_id = p.loadURDF(robot_arm_urdf, robot_arm_position, robot_arm_orientation)

# Run the simulation
try:
    while True:
        p.stepSimulation()
        time.sleep(1. / 240.)  # Run at 240Hz
except KeyboardInterrupt:
    p.disconnect()

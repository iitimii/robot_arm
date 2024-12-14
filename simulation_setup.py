import pybullet as p
import pybullet_data
import time


urdf_file = r"C:\Users\akinw\Downloads\box\box.urdf.xml"

physics_client = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the ground plane
p.setGravity(0, 0, -9.8)  # Set gravity
plane_id = p.loadURDF("plane.urdf")

table_top_height = 0.625 
# Load the table
table_position = [0, 0, 0]
table_orientation = p.getQuaternionFromEuler([0, 0, 0])
table_id = p.loadURDF("table/table.urdf", table_position, table_orientation)

# Load the first box on the table
box1_position = [0.2, 0,table_top_height + 0.05]  # Slightly to the right of center
box1_orientation = p.getQuaternionFromEuler([0, 0, 0])
box1_id = p.loadURDF(urdf_file, box1_position, box1_orientation)

# Load the second box on the table
box2_position = [-0.4, 0, table_top_height+0.05]  # Slightly to the left of center
box2_orientation = p.getQuaternionFromEuler([0, 0, 0])
box2_id = p.loadURDF(urdf_file, box2_position, box2_orientation)

# Run the simulation
try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)  # Run at 240Hz
except KeyboardInterrupt:
    p.disconnect()

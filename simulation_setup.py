import pybullet as p
import pybullet_data
import time


physics_client = p.connect(p.GUI) 

p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the simulation environment
p.setGravity(0, 0, -9.8)  # Set gravity
plane_id = p.loadURDF("plane.urdf") 

# Load the table
table_position = [0, 0, 0] 
table_orientation = p.getQuaternionFromEuler([0, 0, 0]) 
table_id = p.loadURDF("table/table.urdf", table_position, table_orientation)

try:
    while True:
        time.sleep(1.0) 
except KeyboardInterrupt:
    p.disconnect()  
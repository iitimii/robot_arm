import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class RobotSimulation:
    def __init__(self):
        # Connect to PyBullet in GUI mode
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Load environment
        self.planeId = p.loadURDF("plane.urdf")
        self.robotId = p.loadURDF("kuka_iiwa/model.urdf")
        self.cubeId = p.loadURDF("cube.urdf", [1, 1, 0.5])
        
        # Set gravity
        p.setGravity(0, 0, -9.8)
        
        # Prepare visualization
        self.setup_visualization()
        
        # Joint and end-effector tracking
        self.joint_angles = []
        self.end_effector_positions = []
        
    def setup_visualization(self):
        # Suppress matplotlib interactive warning
        plt.rcParams.update({'figure.max_open_warning': 0})
        
        # Set up matplotlib for non-blocking plotting
        plt.ion()  # Turn on interactive mode
        
        # Create figure and store references to prevent garbage collection
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(5, 15))
        
        # Try to keep the plot window in the background
        plt.pause(0.1)
        plt.show(block=False)
        
        # Joint Angles Plot
        self.ax1.set_title('Joint Angles Over Time')
        self.ax1.set_xlabel('Time Step')
        self.ax1.set_ylabel('Angle (radians)')
        
        # End Effector Position Plot
        self.ax2 = plt.subplot(122, projection='3d')
        self.ax2.set_title('End Effector Path')
        self.ax2.set_xlabel('X')
        self.ax2.set_ylabel('Y')
        self.ax2.set_zlabel('Z')
        
    def get_joint_states(self):
        # Get current joint states
        joint_states = p.getJointStates(self.robotId, range(p.getNumJoints(self.robotId)))
        return [state[0] for state in joint_states]
    
    def inverse_kinematics(self, target_position):
        # Simple IK to move end effector
        joint_poses = p.calculateInverseKinematics(
            self.robotId, 
            p.getNumJoints(self.robotId) - 1,  # Last joint (end effector)
            target_position
        )
        return joint_poses
    
    def pick_and_place_cube(self):
        # Approach positions
        pick_pos = [0.5, 0.5, 0.6]  # Above the cube
        place_pos = [0.7, 0.7, 0.6]  # Destination
        
        # Pick up sequence
        pick_joints = self.inverse_kinematics(pick_pos)
        p.setJointMotorControlArray(
            self.robotId, 
            range(p.getNumJoints(self.robotId)), 
            p.POSITION_CONTROL, 
            targetPositions=pick_joints
        )
        
        # Tracking and visualization loop
        for step in range(500):
            p.stepSimulation()
            time.sleep(1./240.)
            
            # Get and track joint states
            current_joints = self.get_joint_states()
            self.joint_angles.append(current_joints)
            
            # Get end effector state
            link_state = p.getLinkState(self.robotId, p.getNumJoints(self.robotId) - 1)
            self.end_effector_positions.append(link_state[0])
            
            # Update visualizations periodically (e.g., every 10 steps)
            if step % 10 == 0:
                self.update_plots()
        
    def update_plots(self):
        # Clear previous plots
        self.ax1.clear()
        self.ax2.clear()
        
        # Recreate plot titles and labels
        self.ax1.set_title('Joint Angles Over Time')
        self.ax1.set_xlabel('Time Step')
        self.ax1.set_ylabel('Angle (radians)')
        self.ax2.set_title('End Effector Path')
        self.ax2.set_xlabel('X')
        self.ax2.set_ylabel('Y')
        self.ax2.set_zlabel('Z')
        
        # Plot joint angles
        joint_angles_array = np.array(self.joint_angles)
        if joint_angles_array.size > 0:
            for i in range(joint_angles_array.shape[1]):
                self.ax1.plot(joint_angles_array[:, i], label=f'Joint {i+1}')
            self.ax1.legend()
        
        # Plot end effector path
        positions_array = np.array(self.end_effector_positions)
        if positions_array.size > 0:
            self.ax2.plot3D(
                positions_array[:, 0], 
                positions_array[:, 1], 
                positions_array[:, 2], 
                'blue'
            )
        
        plt.tight_layout()
        
        # Update the figure without blocking
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def run(self):
        try:
            self.pick_and_place_cube()
            plt.ioff()  # Turn off interactive mode
            plt.show()  # Keep the final plot open
        finally:
            p.disconnect()

# Run the simulation
if __name__ == "__main__":
    sim = RobotSimulation()
    sim.run()

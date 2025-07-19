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
        
        # Camera parameters
        self.camera_width = 640
        self.camera_height = 480
        self.camera_fov = 60  # Field of View
        self.camera_aspect = self.camera_width / self.camera_height
        self.camera_near = 0.01
        self.camera_far = 5.0
        
    def setup_visualization(self):
        # Suppress matplotlib interactive warning
        plt.rcParams.update({'figure.max_open_warning': 0})
        
        # Set up matplotlib for non-blocking plotting
        plt.ion()  # Turn on interactive mode
        
        # Create figure and store references to prevent garbage collection
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(10, 5))
        
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
        
    #
    def get_camera_view_matrix(self):
        # Get the end-effector link state (last link)
        link_state = p.getLinkState(self.robotId, p.getNumJoints(self.robotId) - 1)
        cam_position = link_state[0]  # Position of the link (last link)
        cam_orientation = link_state[1]  # Orientation of the link (quaternion)

        # Convert orientation to rotation matrix
        cam_rotation = p.getMatrixFromQuaternion(cam_orientation)
        
        # Define the camera target as a point in front of the end effector
        # For example, place the target 0.1 meters in front of the camera along the camera's forward direction
        cam_target = [
            cam_position[0] + cam_rotation[0] * 0.1,
            cam_position[1] + cam_rotation[3] * 0.1,
            cam_position[2] + cam_rotation[6] * 0.1
        ]
        
        # Define the "up" vector for the camera (aligned with the link's "up" vector)
        cam_up = [cam_rotation[2], cam_rotation[5], cam_rotation[8]]

        # Compute the view matrix
        view_matrix = p.computeViewMatrix(cam_position, cam_target, cam_up)
        return view_matrix


    def get_camera_image(self):
        # Compute the view and projection matrices
        view_matrix = self.get_camera_view_matrix()
        proj_matrix = p.computeProjectionMatrixFOV(
            self.camera_fov, self.camera_aspect, self.camera_near, self.camera_far
        )

        # Capture the camera image
        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
            self.camera_width, self.camera_height, view_matrix, proj_matrix
        )

        # Reshape the RGB image and normalize to 0-1
        rgb_array = np.reshape(rgbImg, (height, width, 4)) / 255.0
        return rgb_array
    
    def run(self):
        try:
            # self.pick_and_place_cube()
            # plt.ioff()  # Turn off interactive mode
            # plt.show()  # Keep the final plot open
            
            plt.ion()  # Enable interactive mode for Matplotlib
            fig, ax = plt.subplots(figsize=(8, 6))
            
            for _ in range(1000):
                p.stepSimulation()
                time.sleep(1.0 / 240.0)

                # Capture and display camera view
                camera_image = self.get_camera_image()

                # Display the image
                ax.clear()
                ax.imshow(camera_image)
                ax.axis("off")
                plt.pause(0.01)  # Update the plot
        finally:
            plt.ioff()  # Turn off interactive mode
            plt.show()  # Keep the last frame displayed
            p.disconnect()


if __name__ == "__main__":
    sim = RobotSimulation()
    sim.run()

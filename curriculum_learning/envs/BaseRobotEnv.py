from gymnasium import spaces, Env
import numpy as np

import mujoco

TARGET_HEIGHT = 3.0
EPISODE_DURATION = 8
GRASP_THRESHOLD = 0.05  # Distance threshold for successful grasp
LIFT_THRESHOLD = 0.1    # Height threshold to consider cube lifted

cube_link_name = "box"
end_effector_link_name = "hand"
left_finger_link_name = "left_finger"
right_finger_link_name = "right_finger"

class BaseRobotEnv(Env):
    """Base environment for robotic manipulation tasks"""
    
    def __init__(self, xml):
        super(BaseRobotEnv, self).__init__()
        self.model = mujoco.MjModel.from_xml_path(xml)
        self.data = mujoco.MjData(self.model)
        
        # Get body IDs
        self.cube_id = self.model.body(cube_link_name).id
        self.end_effector_id = self.model.body(end_effector_link_name).id
        self.left_finger_id = self.model.body(left_finger_link_name).id
        self.right_finger_id = self.model.body(right_finger_link_name).id
        
        # Control setup
        ctrl_range = self.model.actuator_ctrlrange
        low = ctrl_range[:, 0]
        high = ctrl_range[:, 1]
        self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)
        
        obs_dim = (len(self.data.qpos) + len(self.data.qvel) + 
                  3 + 3 + 3 + 3)  # cube_pos + ee_pos + left_finger_pos + right_finger_pos
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, 
                                          shape=(obs_dim,), dtype=np.float32)
        
        self.initial_cube_height = None
        self.episode_step = 0
        self.max_episode_steps = int(EPISODE_DURATION / self.model.opt.timestep)
        self.target_height = TARGET_HEIGHT
        
    def reset(self, seed=None):
        if seed is not None:
            np.random.seed(seed)
            
        mujoco.mj_resetData(self.model, self.data)
        
        # Randomize initial joint positions
        joint_range = self.model.jnt_range
        for i in range(len(self.data.qpos)):
            if i < len(joint_range):
                low, high = joint_range[i]
                if low < high:
                    self.data.qpos[i] = np.random.uniform(low * 0.8, high * 0.8)

            # Randomize cube position slightly
            cube_joint_idx = 9
            self.data.qpos[cube_joint_idx:cube_joint_idx+3] = [
                np.random.uniform(-0.2, 0.2),  # x
                np.random.uniform(-0.2, 0.2),  # y  
                0.03  # z
            ]
        
        mujoco.mj_forward(self.model, self.data)
        
        # # Store initial cube height
        self.initial_cube_height = self.data.body(self.cube_id).xpos[2]
        self.episode_step = 0
        
        return self._get_obs(), {}
    
    def _get_obs(self):
        """Get observation vector"""
        cube_pos = self.data.body(self.cube_id).xpos
        ee_pos = self.data.body(self.end_effector_id).xpos
        left_finger_pos = self.data.body(self.left_finger_id).xpos
        right_finger_pos = self.data.body(self.right_finger_id).xpos
        
        return np.concatenate([
            self.data.qpos,
            self.data.qvel,
            cube_pos,
            ee_pos,
            left_finger_pos,
            right_finger_pos
        ]).astype(np.float32)
    
    def _get_distance_to_cube(self):
        """Calculate distance from gripper mid point to cube"""
        cube_pos = self.data.body(self.cube_id).xpos
        gripper_mid_pos = (self.data.body(self.left_finger_id).xpos + self.data.body(self.right_finger_id).xpos) / 2
        return np.linalg.norm(gripper_mid_pos - cube_pos)

    def _get_grasp_quality(self):
        """Calculate grasp quality based on finger positions relative to cube"""
        cube_pos = self.data.body(self.cube_id).xpos
        left_finger_pos = self.data.body(self.left_finger_id).xpos
        right_finger_pos = self.data.body(self.right_finger_id).xpos
        
        # Distance from each finger to cube
        left_dist = np.linalg.norm(left_finger_pos - cube_pos)
        right_dist = np.linalg.norm(right_finger_pos - cube_pos)
        
        # Finger separation (for grasp stability)
        finger_dist = np.linalg.norm(left_finger_pos - right_finger_pos)
        
        # Ideal finger separation for grasping
        ideal_separation = 0.02  # Adjust based on cube size
        separation_penalty = abs(finger_dist - ideal_separation)
        
        return 1.0 / (1.0 + left_dist + right_dist + separation_penalty)
    
    def _is_grasping(self):
        """Check if the robot is successfully grasping the cube"""
        # cube_pos = self.data.body(self.cube_id).xpos
        # left_finger_pos = self.data.body(self.left_finger_id).xpos
        # right_finger_pos = self.data.body(self.right_finger_id).xpos
        
        # left_dist = np.linalg.norm(left_finger_pos - cube_pos)
        # right_dist = np.linalg.norm(right_finger_pos - cube_pos)
        
        # return (left_dist < GRASP_THRESHOLD and right_dist < GRASP_THRESHOLD)
        # Use contact or collision information to determine grasping
        contacts = self.data.contact
        for contact in contacts:
            if (contact.geom1 == self.left_finger_id and contact.geom2 == self.cube_id) or \
               (contact.geom1 == self.right_finger_id and contact.geom2 == self.cube_id):
                return True
        return False
    
    def _get_contact_info(self):
        """Get contact information for the cube"""
        contacts = self.data.contact
        contact_info = []
        for contact in contacts:
            if contact.geom1 == self.cube_id or contact.geom2 == self.cube_id:
                contact_info.append({
                    'geom1': contact.geom1,
                    'geom2': contact.geom2,
                    'pos': contact.pos,
                    'dist': contact.dist
                })
        return contact_info

    
    def step(self, action):
        self.data.ctrl[:] = action
        mujoco.mj_step(self.model, self.data)
        self.episode_step += 1
        
        obs = self._get_obs()
        reward = self._get_reward()
        
        # Episode termination conditions
        cube_height = self.data.body(self.cube_id).xpos[2]
        success = cube_height > TARGET_HEIGHT
        timeout = self.episode_step >= self.max_episode_steps
        
        terminated = True if success else False
        truncated = True if timeout else False
        
        info = {
            'success': success,
            'cube_height': cube_height,
            'distance_to_cube': self._get_distance_to_cube(),
            'is_grasping': self._is_grasping(),
            'grasp_quality': self._get_grasp_quality()
        }
        
        return obs, reward, terminated, truncated, info
    
    def _get_reward(self):
        """Base reward function - to be overridden by subclasses"""
        return 0.0
    
    def render(self, mode='human'):
        pass
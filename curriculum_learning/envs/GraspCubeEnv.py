from .BaseRobotEnv import BaseRobotEnv


class GraspCubeEnv(BaseRobotEnv):
    """Environment focused on grasping the cube"""
    
    def _get_reward(self):
        distance = self._get_distance_to_cube()
        grasp_quality = self._get_grasp_quality()
        is_grasping = self._is_grasping()
        
        # Reward for reaching
        reach_reward = 5.0 / (1.0 + distance)
        
        # Reward for good grasp positioning
        grasp_reward = 10.0 * grasp_quality
        
        # Bonus for successful grasp
        grasp_bonus = 20.0 if is_grasping else 0.0
        
        # Small lift reward to encourage lifting while grasping
        cube_height = self.data.body(self.cube_id).xpos[2]
        lift_reward = 0.0
        if is_grasping:
            height_diff = cube_height - self.initial_cube_height
            lift_reward = 5.0 * max(0, height_diff)
        
        # Time penalty
        time_penalty = -0.01 * self.episode_step / self.max_episode_steps
        
        return reach_reward + grasp_reward + grasp_bonus + lift_reward + time_penalty
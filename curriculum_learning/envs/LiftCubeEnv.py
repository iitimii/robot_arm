from .BaseRobotEnv import BaseRobotEnv


class LiftCubeEnv(BaseRobotEnv):
    """Environment focused on lifting the cube to target height"""
    
    def _get_reward(self):
        distance = self._get_distance_to_cube()
        grasp_quality = self._get_grasp_quality()
        is_grasping = self._is_grasping()
        cube_height = self.data.body(self.cube_id).xpos[2]
        
        # Basic reach reward (lower weight since this should be learned)
        reach_reward = 2.0 / (1.0 + distance)
        
        # Grasp reward
        grasp_reward = 5.0 * grasp_quality
        grasp_bonus = 10.0 if is_grasping else 0.0
        
        # Height reward - main objective
        height_diff = cube_height - self.initial_cube_height
        height_reward = 15.0 * max(0, height_diff)
        
        # Success bonus
        success_bonus = 100.0 if cube_height > self.target_height else 0.0
        
        # Penalty for dropping (cube falling below initial height while not grasping)
        drop_penalty = 0.0
        if cube_height < self.initial_cube_height - 0.05 and not is_grasping:
            drop_penalty = -20.0
        
        # Time penalty
        time_penalty = -0.02 * self.episode_step / self.max_episode_steps
        
        return (reach_reward + grasp_reward + grasp_bonus + height_reward + 
                success_bonus + drop_penalty + time_penalty)
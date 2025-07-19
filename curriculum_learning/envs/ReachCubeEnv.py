from .BaseRobotEnv import BaseRobotEnv
import numpy as np


class ReachCubeEnv(BaseRobotEnv):
    """Environment focused on reaching the cube"""
    
    def _get_reward(self):
        distance = self._get_distance_to_cube()
        
        # Dense reward for getting closer to cube
        # reach_reward = 10.0 / (1.0 + distance)
        # Exponential decay for distance
        reach_reward = 10.0 * np.exp(-2*distance)

        
        # Bonus for getting very close
        
        # Small penalty for time to encourage efficiency
        time_penalty = -0.01 * self.episode_step / self.max_episode_steps
        
        return reach_reward # + time_penalty

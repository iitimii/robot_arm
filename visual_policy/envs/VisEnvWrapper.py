from gymnasium import spaces, Env
import numpy as np

import mujoco
from dm_control import manipulation


EPISODE_DURATION = 10



class VisEnvWrapper(Env):
    """Environment for robotic manipulation tasks"""
    
    def __init__(self):
        super(VisEnvWrapper, self).__init__()
        self.env = manipulation.load('lift_brick_vision')
        self.episode_step = 0
        self.max_episode_steps = int(EPISODE_DURATION / self.env.control_timestep())

        low = self.env.action_spec().minimum
        high = self.env.action_spec().maximum
        self.action_space = spaces.Box(low=low, high=high, dtype=np.float64)
        
        obs_dim = self.env.observation_spec()['front_close'].shape[1:]
        obs_dim = (obs_dim[2], obs_dim[1], obs_dim[0])
        obs_low = np.zeros(obs_dim, dtype=np.float32)
        obs_high = np.ones(obs_dim, dtype=np.float32)
        self.observation_space = spaces.Box(low=obs_low, high=obs_high, shape=obs_dim, dtype=np.float32)
    

        
    def reset(self, seed=None):
        if seed is not None:
            np.random.seed(seed)

        self.episode_step = 0

        timestep = self.env.reset()
        obs = timestep.observation['front_close'].squeeze().astype(np.float32) / 255.0
        obs = np.transpose(obs, (2, 1, 0))
        
        return obs, {}

    
    def step(self, action):
        timestep = self.env.step(action)
        self.episode_step += 1
        
        obs = timestep.observation['front_close'].squeeze().astype(np.float32) / 255.0
        obs = np.transpose(obs, (2, 1, 0))

        reward = timestep.reward
        
        timeout = self.episode_step >= self.max_episode_steps
        
        terminated = False
        truncated = True if timeout else False
        
        info = {
        }
        
        return obs, reward, terminated, truncated, info
    
    
    def render(self, mode='human'):
        pass


if __name__ == "__main__":
    from stable_baselines3 import PPO
    from stable_baselines3.common.env_checker import check_env
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold

    print("=== Training Start ===")
    env = VisEnvWrapper()
    check_env(env)
    
    model = PPO(
    policy='CnnPolicy',
    env=env,
    verbose=1,
    learning_rate=3e-4,
    n_steps=2048,
    batch_size=64,
    n_epochs=10,
    policy_kwargs=dict(
        normalize_images=False
    )
)

    total_timesteps=int(1e6)
    model.learn(total_timesteps=total_timesteps)
    model.save('ppo_model')
    print("Model saved!")
import gymnasium as gym
from gymnasium import spaces, Env
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold
import mujoco
from envs.ReachCubeEnv import ReachCubeEnv
from envs.GraspCubeEnv import GraspCubeEnv
from envs.LiftCubeEnv import LiftCubeEnv



def train_progressive_learning(robot_path, total_timesteps_per_stage=1e5):
    """Train the robot progressively through different stages"""
    
    print("=== Stage 1: Learning to Reach ===")
    reach_env = ReachCubeEnv(robot_path)
    check_env(reach_env)
    
    reach_model = PPO(
        policy='MlpPolicy',
        env=reach_env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
    )
    
    reach_model.learn(total_timesteps=total_timesteps_per_stage)
    reach_model.save('ppo_reach_model')
    print("Reach model saved!")
    
    print("=== Stage 2: Learning to Grasp ===")
    grasp_env = GraspCubeEnv(robot_path)
    check_env(grasp_env)
    
    # Initialize grasp model with reach model weights
    grasp_model = PPO.load('ppo_reach_model', env=grasp_env)
    grasp_model.learn(total_timesteps=total_timesteps_per_stage)
    grasp_model.save('ppo_grasp_model')
    print("Grasp model saved!")
    
    print("=== Stage 3: Learning to Lift ===")
    lift_env = LiftCubeEnv(robot_path)
    check_env(lift_env)
    
    # Initialize lift model with grasp model weights
    lift_model = PPO.load('ppo_grasp_model', env=lift_env)
    lift_model.learn(total_timesteps=total_timesteps_per_stage)
    lift_model.save('ppo_lift_model')
    print("Lift model saved!")
    
    return lift_model


def evaluate_model(model, env, num_episodes=10):
    """Evaluate the trained model"""
    successes = 0
    total_rewards = []
    
    for episode in range(num_episodes):
        obs, info = env.reset()
        episode_reward = 0
        
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            episode_reward += reward
            
            if terminated or truncated:
                if info.get('success', False):
                    successes += 1
                total_rewards.append(episode_reward)
                print(f"Episode {episode + 1}: Reward = {episode_reward:.2f}, "
                      f"Success = {info.get('success', False)}, "
                      f"Final Height = {info.get('cube_height', 0):.3f}")
                break
    
    success_rate = successes / num_episodes
    avg_reward = np.mean(total_rewards)
    
    print(f"\nEvaluation Results:")
    print(f"Success Rate: {success_rate:.2%}")
    print(f"Average Reward: {avg_reward:.2f}")
    
    return success_rate, avg_reward


if __name__ == '__main__':
    robot_path = 'franka_emika_panda/mjx_single_cube.xml'
    
    # Test individual environments
    print("Testing ReachCubeEnv...")
    reach_env = ReachCubeEnv(robot_path)
    check_env(reach_env)
    
    print("Testing GraspCubeEnv...")
    grasp_env = GraspCubeEnv(robot_path)
    check_env(grasp_env)
    
    print("Testing LiftCubeEnv...")
    lift_env = LiftCubeEnv(robot_path)
    check_env(lift_env)
    
    print("All environments passed compliance check!")
    
    # Quick test
    print("\nRunning quick test...")
    obs, info = lift_env.reset()
    print(f"Initial observation shape: {obs.shape}")
    
    for i in range(5):
        action = lift_env.action_space.sample()
        obs, reward, terminated, truncated, info = lift_env.step(action)
        print(f"Step {i}: reward={reward:.3f}, distance={info['distance_to_cube']:.3f}, "
              f"grasping={info['is_grasping']}, height={info['cube_height']:.3f}")
        
        if terminated or truncated:
            obs, info = lift_env.reset()
            print("Episode reset")
    
    # Progressive training
    print("\nStarting progressive training...")
    final_model = train_progressive_learning(robot_path, total_timesteps_per_stage=1e7)
    
    # Final evaluation
    print("\n=== Final Evaluation ===")
    evaluate_model(final_model, LiftCubeEnv(robot_path), num_episodes=5)
    
    print("Training completed!")
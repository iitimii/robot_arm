import torch
from simulation.kuka_envs.kukaGymEnv import KukaGymEnv
from rl_algorithms.REINFORCE import REINFORCE

env = KukaGymEnv()

model = REINFORCE(env, learning_rate=1e-3)
model.learn(15_000)

env = env = KukaGymEnv(renders=True)
for _ in range(5):
    Rewards = []
    obs, _ = env.reset()
    obs = torch.tensor(obs, dtype=torch.float32)
    done = False
    env.render()

    while not done:
        probs = model.predict(obs)
        action = torch.argmax(probs)

        obs_, reward, terminated, truncated, _ = env.step(action.item())
        done = terminated or truncated

        obs = torch.tensor(obs_, dtype=torch.float32)
        Rewards.append(reward)

    print(f'Reward: {sum(Rewards)}')
env.close()
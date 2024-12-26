import torch
from torch import nn, optim
from torch.distributions import Categorical
import gymnasium as gym

y = 0.99
lr = 0.005
max_steps = 10_000

env = gym.make("CartPole-v1")
net = nn.Sequential(
    nn.Linear(4, 64),
    nn.ReLU(),
    nn.Linear(64, 64),
    nn.ReLU(),
    nn.Linear(64, env.action_space.n),
    nn.Softmax(dim=-1)
)

optim = optim.Adam(net.parameters(), lr=lr)

step = 0
while step < max_steps:
    obs, _ = env.reset()
    obs = torch.tensor(obs, dtype=torch.float32)
    done = False
    Actions, States, Rewards = [], [], []

    while not done:
        probs = net(obs)
        dist = Categorical(probs)
        action = dist.sample()

        obs_, reward, terminated, truncated, _ = env.step(action.item())
        done = terminated or truncated

        Actions.append(action)
        States.append(obs)
        Rewards.append(reward)

        obs = torch.tensor(obs_, dtype=torch.float32)
        step += 1

    for t in range(len(Rewards)):
        G = 0.0
        for k, r in enumerate(Rewards[t:]):
            G += (y ** k) * r

        state = States[t]
        action = Actions[t]

        probs = net(state)
        dist = Categorical(probs)
        log_prob = dist.log_prob(action)

        loss = - log_prob * G

        optim.zero_grad()
        loss.backward()
        optim.step()

env.close()

env = gym.make("CartPole-v1", render_mode='human')
for _ in range(5):
    Rewards = []
    obs, _ = env.reset()
    obs = torch.tensor(obs, dtype=torch.float32)
    done = False
    env.render()

    while not done:
        probs = net(obs)
        dist = Categorical(probs)
        action = dist.sample()

        obs_, reward, terminated, truncated, _ = env.step(action.item())
        done = terminated or truncated

        obs = torch.tensor(obs_, dtype=torch.float32)
        Rewards.append(reward)

    print(f'Reward: {sum(Rewards)}')
env.close()


class REINFORCE():
    def __init__(self, env: gym.Env)
        pass
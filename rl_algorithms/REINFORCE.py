import torch
from torch import nn, optim
from torch.distributions import Categorical
import gymnasium as gym

class REINFORCE():
    def __init__(self, env: gym.Env, policy: nn.Module = None, learning_rate=5e-3, discount_factor=0.99):
        self.env = env
        if policy is None:
            self.policy = nn.Sequential(
            nn.Linear(self.env.observation_space.shape[0], 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, self.env.action_space.n),
            nn.Softmax(dim=-1))
        else:
            self.policy = policy

        self.discount_factor = discount_factor
        self.learning_rate = learning_rate

        self.Actions, self.States, self.Rewards = [], [], []
        self.optim = optim.Adam(self.policy.parameters(), lr=self.learning_rate)

    def train(self):
        self.policy.train()
        for t in range(len(self.Rewards)):
            G = 0.0
            for k, r in enumerate(self.Rewards[t:]):
                G += (self.discount_factor ** k) * r

            state = self.States[t]
            action = self.Actions[t]

            probs = self.policy(state)
            dist = Categorical(probs)
            log_prob = dist.log_prob(action)

            loss = - log_prob * G

            self.optim.zero_grad()
            loss.backward()
            self.optim.step()
        

    def learn(self, total_timesteps=10_000):
        step = 0
        while step < total_timesteps:
            obs, _ = self.env.reset()
            obs = torch.tensor(obs, dtype=torch.float32)
            done = False
            self.Actions, self.States, self.Rewards = [], [], []

            while not done:
                probs = self.policy(obs)
                dist = Categorical(probs)
                action = dist.sample()

                obs_, reward, terminated, truncated, _ = self.env.step(action.item())
                done = terminated or truncated

                self.Actions.append(action)
                self.States.append(obs)
                self.Rewards.append(reward)

                obs = torch.tensor(obs_, dtype=torch.float32)
                step += 1

            self.train()
            print(f"timestep: {step}/{total_timesteps}")

        self.env.close()
        return self
    
    def predict(self, obs):
        obs = torch.tensor(obs, dtype=torch.float32)
        return self.policy(obs)

from actor import Actor
from critic import Critic
from gym import Env

class Trainer:
    def __init__(self, actor:Actor, critic:Critic, env:Env):
        self.actor = actor
        self.critic = critic
        self.env = env
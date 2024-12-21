from simulation.kuka_envs.kukaGymEnv import KukaGymEnv
from simulation.kuka_envs.kukaCamGymEnv import KukaCamGymEnv
from simulation.kuka_envs.kuka_diverse_object_gym_env import KukaDiverseObjectEnv

env = KukaGymEnv(renders=True)

env.reset()
while True:
    _ = env.step(env.action_space.sample())

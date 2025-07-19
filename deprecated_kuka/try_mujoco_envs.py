from simulation.mujoco.gym_manipulator_envs import ReacherBulletEnv, StrikerBulletEnv, PusherBulletEnv, ThrowerBulletEnv

env = ThrowerBulletEnv(render=True)

env.reset()
while True:
    _ = env.step(env.action_space.sample())
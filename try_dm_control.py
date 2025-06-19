import numpy as np
from dm_control import mujoco
from dm_control import manipulation
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from IPython.display import HTML

def display_video(frames, framerate=30):
    height, width, _ = frames[0].shape
    dpi = 70
    orig_backend = matplotlib.get_backend()
    matplotlib.use('Agg')  # Switch to headless 'Agg' to inhibit figure rendering.
    fig, ax = plt.subplots(1, 1, figsize=(width / dpi, height / dpi), dpi=dpi)
    matplotlib.use(orig_backend)  # Switch back to the original backend.
    ax.set_axis_off()
    ax.set_aspect('equal')
    ax.set_position([0, 0, 1, 1])
    im = ax.imshow(frames[0])
    def update(frame):
      im.set_data(frame)
      return [im]
    interval = 1000/framerate
    anim = animation.FuncAnimation(fig=fig, func=update, frames=frames,
                                   interval=interval, blit=True, repeat=False)
    return HTML(anim.to_html5_video())

env = manipulation.load('lift_brick_vision')

time_step = env.reset()
frames = []
timestep = env.reset()
frames.append(timestep.observation['front_close'])

for _ in range(100):
    action = np.random.uniform(-1, 1, env.action_spec().shape)
    time_step = env.step(action)
    print(f"Reward: {time_step.reward}, Last: {time_step.last()}")
    frames.append(timestep.observation['front_close'])

all_frames = np.concatenate(frames, axis=0)
display_video(all_frames, 30)
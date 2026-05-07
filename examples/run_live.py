#!/usr/bin/env python3
"""Run the RectWorldEnv and show a live matplotlib window if available.

Behavior:
- If a display is available, opens a live matplotlib window and updates frames.
- If no display, falls back to storing frames and writing a GIF at /tmp/rect_world_sim.gif.

Usage:
  PYTHONPATH=. python3 examples/run_live.py
"""
import os
import time
import math
from pathlib import Path
from rl_envs.rect_world_env import RectWorldEnv

def try_live_run(out_gif='/tmp/rect_world_sim.gif', frames_dir='/tmp/rect_world_frames'):
    env = RectWorldEnv()
    obs = env.reset()
    steps = 0
    max_steps = 300

    # prepare frames dir
    Path(frames_dir).mkdir(parents=True, exist_ok=True)

    use_live = True
    try:
        import matplotlib.pyplot as plt
        plt.ion()
        fig, ax = plt.subplots()
    except Exception:
        use_live = False

    frames = []

    try:
        while steps < max_steps:
            # simple policy: move toward goal
            gx,gy = env.goal
            x,y,theta = env.state
            ang_to_goal = math.atan2(gy-y, gx-x)
            diff = (ang_to_goal - theta + math.pi) % (2*math.pi) - math.pi
            if abs(diff) < 0.3:
                a = 0  # forward
            elif diff > 0:
                a = 1  # fwd_left
            else:
                a = 2  # fwd_right

            obs, r, done, info = env.step(a)
            steps += 1

            # render frame to numpy image
            try:
                import matplotlib.pyplot as plt
                from io import BytesIO
                buf = BytesIO()
                env.render(show=False, filepath=None)
            except Exception:
                pass

            # Save a PNG by calling render to a file
            fname = os.path.join(frames_dir, f'frame_{steps:04d}.png')
            env.render(show=False, filepath=fname)
            frames.append(fname)

            if use_live:
                try:
                    img = plt.imread(fname)
                    ax.clear()
                    ax.imshow(img)
                    ax.axis('off')
                    fig.canvas.draw()
                    plt.pause(0.02)
                except Exception:
                    use_live = False

            if done:
                break

        # finished, produce GIF if not live or even if live for sharing
        try:
            import imageio
            imgs = [imageio.v2.imread(f) for f in frames]
            imageio.mimsave(out_gif, imgs, fps=10)
            print('Wrote GIF to', out_gif)
        except Exception:
            # try pillow
            try:
                from PIL import Image
                imgs = [Image.open(f).convert('RGBA') for f in frames]
                imgs[0].save(out_gif, save_all=True, append_images=imgs[1:], duration=100, loop=0)
                print('Wrote GIF to', out_gif)
            except Exception as e:
                print('Could not write GIF:', e)

    finally:
        if use_live:
            try:
                import matplotlib.pyplot as plt
                plt.ioff()
            except Exception:
                pass

if __name__ == '__main__':
    print('Starting live run (will fall back to gif if no display)')
    try_live_run()

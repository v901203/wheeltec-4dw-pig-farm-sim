#!/usr/bin/env python3
"""Render a single preview image of the RectWorldEnv to /tmp/rect_world_pens_rectcar.png"""
from rl_envs.rect_world_env import RectWorldEnv

def main():
    env = RectWorldEnv()
    env.reset()
    env.render(show=False, filepath='/tmp/rect_world_pens_rectcar.png')
    print('Wrote /tmp/rect_world_pens_rectcar.png')

if __name__ == '__main__':
    main()

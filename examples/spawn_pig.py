#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
import os
import subprocess
import argparse


def main():
    parser = argparse.ArgumentParser(description='Show or spawn the farm_pig model')
    parser.add_argument('--spawn', action='store_true', help='Call scripts/spawn_robot.sh to spawn the pig')
    args = parser.parse_args()

    pig_sdf_path = os.path.join(get_package_share_directory('pig_model'), 'models', 'farm_pig', 'model.sdf')
    print(pig_sdf_path)

    if args.spawn:
        cmd = ['./scripts/spawn_robot.sh', '--file', pig_sdf_path, '--model', 'farm_pig', '--pos', '-7', '0', '0.01', '--yaw', '0']
        print('Running:', ' '.join(cmd))
        subprocess.run(cmd)


if __name__ == '__main__':
    main()

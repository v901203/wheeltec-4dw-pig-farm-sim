#!/usr/bin/env python3
"""Stop only the M10P driver before a DDS migration; run inside robot-dev."""
import argparse
from pathlib import Path
import os
import signal
import time


def command(pid):
    try:
        return Path('/proc', str(pid), 'cmdline').read_bytes().split(b'\0')[:-1]
    except (OSError, ProcessLookupError):
        return []


def is_lidar(args):
    tokens = [a.decode(errors='replace') for a in args]
    launch = any(Path(a).name == 'ros2' for a in tokens) and all(
        x in tokens for x in ('launch', 'lslidar_driver', 'lslidar_m10p_launch.py'))
    driver = bool(tokens) and Path(tokens[0]).name == 'lslidar_driver_node' and any(
        a == '__ns:=/x10' for a in tokens)
    return launch or driver


def identity(pid):
    try:
        return Path('/proc', str(pid), 'stat').read_text().rsplit(')', 1)[1].split()[19]
    except (OSError, IndexError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stop', action='store_true', help='Stop the listed driver processes')
    args = parser.parse_args()
    targets = []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit() or int(path.name) == os.getpid():
            continue
        pid = int(path.name)
        cmd = command(pid)
        names = {Path(a.decode(errors='replace')).name for a in cmd}
        if names & {'fsm_lidar_patrol.py', 'control_modes.py', 'wheeltec_uart_bridge_node'}:
            raise SystemExit('仍有小車控制程序，請先停止巡邏服務及手動控制入口。')
        if is_lidar(cmd):
            targets.append((pid, cmd, identity(pid)))
    for pid, cmd, _ in targets:
        print(pid, b' '.join(cmd).decode(errors='replace'), flush=True)
    if not args.stop:
        print('唯讀列出；加 --stop 才會停止上述雷達程序。')
        return
    def alive(pid, cmd, born):
        return born is not None and identity(pid) == born and command(pid) == cmd
    for sig, timeout in ((signal.SIGTERM, 5.), (signal.SIGKILL, 2.)):
        for pid, cmd, born in targets:
            if alive(pid, cmd, born):
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and any(alive(*t) for t in targets):
            time.sleep(.1)
    if any(alive(*t) for t in targets):
        raise SystemExit('部分雷達程序尚未退出，請檢查後再啟動。')
    print('舊雷達程序已停止（已結束的殭屍程序不占用 socket）。未啟動巡邏或其他節點。')


if __name__ == '__main__':
    main()

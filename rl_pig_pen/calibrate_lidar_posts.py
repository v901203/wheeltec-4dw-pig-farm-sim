#!/usr/bin/env python3
"""Read-only raw LiDAR capture; candidate post angles require physical verification."""
import argparse
import gzip
import json
import math
from pathlib import Path
import time
import numpy as np


class PostSurvey:
    def __init__(self, yaw=0., near=.08, far=.50, bin_degrees=.25):
        self.yaw, self.near, self.far, self.width = yaw, near, far, bin_degrees
        self.exposure = np.zeros(round(360 / bin_degrees), dtype=int)
        self.hits = np.zeros_like(self.exposure)
        self.ranges = [[] for _ in self.hits]
        self.frames = 0

    def add(self, scan):
        raw = np.asarray(scan.ranges, dtype=float)
        if (raw.ndim != 1 or not len(raw) or not math.isfinite(scan.angle_min)
                or not math.isfinite(scan.angle_increment) or scan.angle_increment == 0):
            raise ValueError('Invalid LaserScan angles')
        angles = (np.degrees(scan.angle_min + np.arange(len(raw)) * scan.angle_increment
                             + self.yaw) + 180) % 360
        bins = np.minimum((angles / self.width).astype(int), len(self.hits) - 1)
        # One vote per angular bin per scan, regardless of laser resolution.
        self.exposure[np.unique(bins)] += 1
        valid = (np.isfinite(raw) & (raw >= max(self.near, scan.range_min))
                 & (raw <= min(self.far, scan.range_max)))
        self.hits[np.unique(bins[valid])] += 1
        for b, r in zip(bins[valid], raw[valid]):
            self.ranges[b].append(float(r))
        self.frames += 1

    def report(self, fraction=.2, min_frames=10, padding=1.):
        ratio = np.divide(self.hits, self.exposure, out=np.zeros(len(self.hits)),
                          where=self.exposure > 0)
        active = (self.hits >= min_frames) & (ratio >= fraction)
        groups = []
        for index in np.flatnonzero(active):
            if not groups or index != groups[-1][-1] + 1:
                groups.append([])
            groups[-1].append(int(index))
        candidates = []
        for group in groups:
            values = [r for i in group for r in self.ranges[i]]
            low = -180 + group[0] * self.width
            high = -180 + (group[-1] + 1) * self.width
            # Corner search windows only. Other close returns remain in report.
            middle = (low + high) / 2
            corner = any(a <= middle <= b for a, b in
                         ((20, 70), (110, 160), (-160, -110), (-70, -20)))
            candidates.append(dict(low_deg=low, high_deg=high, corner_candidate=corner,
                padded_deg=[max(-180, low-padding), min(180, high+padding)],
                minimum_frame_fraction=float(min(ratio[group])),
                median_range_m=float(np.median(values)),
                range_p10_p90_m=list(map(float, np.quantile(values, [.1, .9])))))
        return dict(frames=self.frames, near_m=self.near, far_m=self.far,
            lidar_yaw_rad=self.yaw, bin_degrees=self.width,
            candidates=candidates, exposure=self.exposure.tolist(), hits=self.hits.tolist(),
            warning='候選可能是牆、線材或其他物體；不是已確認角柱。不可直接套用全部候選。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scan-topic', default='/x10/scan')
    parser.add_argument('--duration', type=float, default=60.)
    parser.add_argument('--near', type=float, default=.08)
    parser.add_argument('--far', type=float, default=.50)
    parser.add_argument('--fraction', type=float, default=.2)
    parser.add_argument('--padding', type=float, default=1.)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('patrol_config.json'))
    parser.add_argument('--output', type=Path, default=Path('logs/lidar_posts') / time.strftime('%Y%m%d_%H%M%S'))
    args = parser.parse_args()
    if not (all(math.isfinite(v) for v in (args.duration, args.near, args.far, args.fraction, args.padding))
            and args.duration > 0 and 0 < args.near < args.far <= 2
            and 0 < args.fraction <= 1 and 0 <= args.padding <= 5):
        parser.error('Invalid capture duration, ranges, fraction or padding')
    config = json.loads(args.config.read_text())
    yaw = float(config.get('lidar_yaw', 0.))
    if not math.isfinite(yaw):
        parser.error('lidar_yaw must be finite')
    survey = PostSurvey(yaw, args.near, args.far)
    args.output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    node = rclpy.create_node('lidar_post_calibration')
    started = None
    previous = None
    last_progress = time.monotonic()
    with gzip.open(args.output / 'raw_scans.jsonl.gz', 'wt', encoding='utf-8') as raw_file:
        def receive(scan):
            nonlocal started, previous
            stamp = (scan.header.stamp.sec, scan.header.stamp.nanosec)
            if previous is not None and stamp <= previous:
                return
            survey.add(scan)
            previous = stamp
            if started is None:
                started = time.monotonic()
            # Preserve raw returns including inf/nan for replay, without old masks.
            raw_file.write(json.dumps(dict(stamp=stamp, angle_min=scan.angle_min,
                angle_increment=scan.angle_increment, range_min=scan.range_min,
                range_max=scan.range_max, ranges=list(scan.ranges))) + '\n')
        node.create_subscription(LaserScan, args.scan_topic, receive, qos_profile_sensor_data)
        wait_start = time.monotonic()
        try:
            print('唯讀量測開始；請保持車子不動、周圍 1 公尺清空，不發布任何速度指令。', flush=True)
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.2)
                now = time.monotonic()
                if started is None and now - wait_start > 15:
                    print('15 秒未收到新掃描，請確認雷達節點與 topic。', flush=True)
                    break
                if now - last_progress > 5:
                    print(f'已收集 {survey.frames} 幀', flush=True)
                    last_progress = now
                if started is not None and now - started >= args.duration:
                    break
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    report = survey.report(args.fraction, padding=args.padding)
    report['existing_mask'] = config.get('lidar_ignore_sectors_deg', [])
    report['requested_duration_s'] = args.duration
    report['observed_duration_s'] = 0. if started is None else time.monotonic() - started
    (args.output / 'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(f'收集 {survey.frames} 幀，報告：{args.output / "report.json"}')
    for c in report['candidates']:
        print(f'{c["low_deg"]:.2f}～{c["high_deg"]:.2f}° '
              f'出現率至少 {c["minimum_frame_fraction"]:.0%} '
              f'距離中位數 {c["median_range_m"]:.3f} m '
              f'{"角柱候選" if c["corner_candidate"] else "其他近距離回波"}')
    print(report['warning'])
    if survey.frames < 100:
        raise SystemExit('掃描不足 100 幀，請排查後重新收集。')


if __name__ == '__main__':
    main()

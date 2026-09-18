import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from navigation import load_config, scan_features


class LidarChecker(Node):
    def __init__(self):
        super().__init__('lidar_checker_node')
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.cb, qos_profile_sensor_data
        )
        self.received = False

    def cb(self, msg):
        cfg = load_config()
        feat = scan_features(msg, cfg)
        print("\n" + "=" * 45)
        print("【雷達感測與幾何診斷結果】")
        print(f"正前方距離 (front_m):     {feat.front_m:.2f} m")
        print(f"左側距離   (left_edge_m): {feat.left_edge_m:.2f} m")
        print(f"右側距離   (right_edge_m): {feat.right_edge_m:.2f} m")
        print(f"走道平行誤差 (parallel_err): {math.degrees(feat.wall_parallel_error):.2f}° ({feat.wall_parallel_error:.3f} rad)")
        print(f"走道擬合信心度 (confidence): {feat.wall_alignment_confidence:.2f}")
        print("=" * 45 + "\n")
        self.received = True


def main():
    rclpy.init()
    node = LidarChecker()
    print("正在等待 /scan 話題資料（請確保 Gazebo 與模擬小車正在運行）...")
    while rclpy.ok() and not node.received:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


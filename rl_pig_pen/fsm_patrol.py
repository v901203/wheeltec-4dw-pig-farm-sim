import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from navigation import load_config, scan_features, MAX_LIN, MAX_ANG


class PigPenStraightLineController(Node):
    def __init__(self):
        super().__init__('pig_pen_straight_controller')
        self.cfg = load_config()

        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data
        )
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.latest_scan = None
        self.log_counter = 0

        self.timer = self.create_timer(self.cfg.control_dt, self.control_loop)
        self.get_logger().info("純平行與前瞻置中控制器已啟動")

    def scan_cb(self, msg):
        self.latest_scan = msg

    def control_loop(self):
        if self.latest_scan is None:
            return

        feat = scan_features(self.latest_scan, self.cfg)

        # 1. 走道盡頭煞停
        if feat.front_m < 0.55:
            cmd = Twist()
            self.pub_cmd.publish(cmd)
            self.get_logger().warn(f"走道盡頭抵達 ({feat.front_m:.2f}m)，煞停")
            return

        # 2. 前瞻左右扇區 (改取左前 35°~65° 與 右前 -65°~-35°)
        angles = feat.angles
        ranges = feat.ranges

        left_mask = (angles >= math.radians(35)) & (angles <= math.radians(65))
        right_mask = (angles >= math.radians(-65)) & (angles <= math.radians(-35))

        forward_left = float(np.min(ranges[left_mask])) if np.any(left_mask) else feat.left_edge_m
        forward_right = float(np.min(ranges[right_mask])) if np.any(right_mask) else feat.right_edge_m

        # 3. 前瞻置中誤差
        center_error = forward_left - forward_right

        # 4. 控制增益
        kp_heading = 1.4
        kp_center = 1.0

        wz = (kp_heading * feat.wall_parallel_error) + (kp_center * center_error)

        # 5. 限速輸出
        max_turn = min(0.3, MAX_ANG)
        vx = float(np.clip(float(self.cfg.cruise_speed), 0.0, MAX_LIN))
        wz = float(np.clip(wz, -max_turn, max_turn))

        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        self.pub_cmd.publish(cmd)

        # 6. 印出除錯日誌
        self.log_counter += 1
        if self.log_counter % 12 == 0:
            self.get_logger().info(
                f"左前距:{forward_left:.2f}m 右前距:{forward_right:.2f}m | "
                f"平行誤:{math.degrees(feat.wall_parallel_error):.1f}° | 置中差:{center_error:.2f}m -> wz:{wz:.2f}"
            )


def main():
    rclpy.init()
    node = PigPenStraightLineController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
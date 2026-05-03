#!/usr/bin/env python3
"""Republish Gazebo model odom to /odom for ROS compatibility."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class RepublishOdom(Node):
    def __init__(self):
        super().__init__('republish_odom')
        self.sub = self.create_subscription(
            Odometry, '/model/wheeltec_mini/odometry', self.cb, 10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.get_logger().info('Republishing /model/wheeltec_mini/odometry -> /odom')

    def cb(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RepublishOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

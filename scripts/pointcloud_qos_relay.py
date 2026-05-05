#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class PointCloudQosRelay(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_qos_relay')

        in_topic = '/camera/depth/points'
        out_topic = '/camera/depth/points_reliable'

        # Match producer QoS to receive pointcloud from depth_image_proc.
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publish with RELIABLE QoS so RViz default subscription can consume it.
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(PointCloud2, out_topic, pub_qos)
        self._sub = self.create_subscription(PointCloud2, in_topic, self._on_msg, sub_qos)
        self.get_logger().info(f'Relaying {in_topic} -> {out_topic} (BEST_EFFORT -> RELIABLE)')

    def _on_msg(self, msg: PointCloud2) -> None:
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = PointCloudQosRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

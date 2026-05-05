#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

class RobotDescriptionPublisher(Node):
    def __init__(self, urdf_path: str):
        super().__init__('robot_description_publisher')
        self.pub = self.create_publisher(String, '/robot_description', 10)
        self.urdf = ''
        if os.path.isfile(urdf_path):
            with open(urdf_path, 'r', encoding='utf-8') as f:
                self.urdf = f.read()
        else:
            self.get_logger().error(f'URDF not found: {urdf_path}')
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if self.urdf:
            msg = String()
            msg.data = self.urdf
            self.pub.publish(msg)

def main():
    import sys
    if len(sys.argv) < 2:
        print('Usage: publish_robot_description.py /path/to/robot.urdf')
        return
    urdf_path = sys.argv[1]
    rclpy.init()
    node = RobotDescriptionPublisher(urdf_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

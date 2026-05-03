#!/usr/bin/env python3
"""ROS2 node: convert /cmd_vel -> left/right wheel angular velocities

Publishes:
 - /wheeltec_mini/left_wheel_velocity (std_msgs/Float64)
 - /wheeltec_mini/right_wheel_velocity (std_msgs/Float64)

Params (ROS2 node params or CLI overrides):
 - wheel_radius (m)
 - wheel_base (m)  # distance between left and right wheels
 - model_name (string) prefix for topics (default: wheeltec_mini)

This node does not assume any specific ros_gz bridge mapping; use
ros_gz_bridge to map these topics to Gazebo if desired.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class CmdVelToWheels(Node):
    def __init__(self):
        super().__init__('cmdvel_to_wheels')
        # parameters
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.30)
        self.declare_parameter('model_name', 'wheeltec_mini')

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.model_name = self.get_parameter('model_name').value

        left_topic = f'/{self.model_name}/left_wheel_velocity'
        right_topic = f'/{self.model_name}/right_wheel_velocity'

        self.pub_left = self.create_publisher(Float64, left_topic, 10)
        self.pub_right = self.create_publisher(Float64, right_topic, 10)

        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmdvel_cb, 10)

        self.get_logger().info(f'Node started: wheel_radius={self.wheel_radius} m, '
                               f'wheel_base={self.wheel_base} m, model={self.model_name}')

    def cmdvel_cb(self, msg: Twist):
        v = msg.linear.x
        omega = msg.angular.z

        # differential drive kinematics
        v_r = v + (omega * self.wheel_base / 2.0)
        v_l = v - (omega * self.wheel_base / 2.0)

        # convert linear wheel velocities to angular velocities (rad/s)
        try:
            w_r = v_r / self.wheel_radius
            w_l = v_l / self.wheel_radius
        except Exception as e:
            self.get_logger().error(f'Error computing wheel velocities: {e}')
            return

        m_left = Float64()
        m_right = Float64()
        m_left.data = float(w_l)
        m_right.data = float(w_r)

        self.pub_left.publish(m_left)
        self.pub_right.publish(m_right)

        self.get_logger().debug(f'cmd_vel v={v:.3f} omega={omega:.3f} -> w_l={w_l:.3f} w_r={w_r:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToWheels()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

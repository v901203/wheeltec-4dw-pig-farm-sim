from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    config = Path(get_package_share_directory("wheeltec_uart_bridge")) / "config" / "uart_bridge.yaml"
    return LaunchDescription([
        Node(
            package="wheeltec_uart_bridge",
            executable="wheeltec_uart_bridge_node",
            name="wheeltec_uart_bridge",
            output="screen",
            parameters=[str(config)],
        )
    ])

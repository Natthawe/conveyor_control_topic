"""Launch file for the odometry node."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """ Launches the odometry node with parameters from a YAML file. """
    config = os.path.join(
        get_package_share_directory('conveyor_control_topic'),
        'config',
        'ack_led_hook.yaml'
    )

    # Check if the config file exists before launching the node
    if not os.path.isfile(config):
        print(f"Config file not found at: {config}")

    return LaunchDescription([
        Node(
            package='conveyor_control_topic',
            executable='ack_udp_node',
            name='ack_udp_node',
            output='screen',
            parameters=[config]
        )
    ])

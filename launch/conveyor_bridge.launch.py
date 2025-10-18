from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='conveyor_control_topic',
            executable='bridge_node',
            name='conveyor_bridge',
            output='screen',
            parameters=['config/params.yaml']             
        )
    ])

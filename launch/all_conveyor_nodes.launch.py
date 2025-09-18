# all_conveyor_nodes.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('conveyor_control_topic')
    launch_dir = os.path.join(pkg_share, 'launch')

    enable_conveyor = LaunchConfiguration('enable_conveyor')
    enable_led      = LaunchConfiguration('enable_led')
    enable_hook     = LaunchConfiguration('enable_hook')

    return LaunchDescription([
        # arguments
        DeclareLaunchArgument('enable_conveyor', default_value='false',
                              description='Launch conveyor UDP node'),
        DeclareLaunchArgument('enable_led', default_value='true',
                              description='Launch LED UDP node'),
        DeclareLaunchArgument('enable_hook', default_value='true',
                              description='Launch Hook/Relay ACK UDP node'),

        # include: conveyor
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'conveyor_udp_node.launch.py')
            ),
            condition=IfCondition(enable_conveyor),
        ),

        # include: LED
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'led_udp_node.launch.py')
            ),
            condition=IfCondition(enable_led),
        ),

        # include: hook/relay ACK
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'ack_hook_relay_udp_node.launch.py')
            ),
            condition=IfCondition(enable_hook),
        ),
    ])

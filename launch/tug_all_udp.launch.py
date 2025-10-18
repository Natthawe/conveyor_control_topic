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

    enable_control_relay = LaunchConfiguration('enable_control_relay')
    enable_conveyor_sensor = LaunchConfiguration('enable_conveyor_sensor')

    return LaunchDescription([
        # arguments
        DeclareLaunchArgument('enable_control_relay', default_value='true',
                              description='Launch Control Relay UDP node'),
        DeclareLaunchArgument('enable_conveyor_sensor', default_value='true',
                              description='Launch Conveyor Sensor UDP node'),

        # include: control relay
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'tug_nano_control_relay.launch.py')
            ),
            condition=IfCondition(enable_control_relay),
        ),

        # include: conveyor sensor
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'tug_nano_conveyor_sensor.launch.py')
            ),
            condition=IfCondition(enable_conveyor_sensor),
        ),

    ])

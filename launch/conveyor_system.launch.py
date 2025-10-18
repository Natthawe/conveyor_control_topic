from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    target_ip         = DeclareLaunchArgument('target_ip', default_value='10.1.100.222')
    port_conveyor_rx  = DeclareLaunchArgument('port_conveyor_rx', default_value='30001')
    port_relay_rx     = DeclareLaunchArgument('port_relay_rx', default_value='30002')
    port_conveyor_ack = DeclareLaunchArgument('port_conveyor_ack', default_value='31001')
    port_relay_ack    = DeclareLaunchArgument('port_relay_ack', default_value='31002')
    bind_ip           = DeclareLaunchArgument('bind_ip', default_value='0.0.0.0')
    telemetry_port    = DeclareLaunchArgument('telemetry_port', default_value='32001')

    lc_target_ip         = LaunchConfiguration('target_ip')
    lc_port_conveyor_rx  = LaunchConfiguration('port_conveyor_rx')
    lc_port_relay_rx     = LaunchConfiguration('port_relay_rx')
    lc_port_conveyor_ack = LaunchConfiguration('port_conveyor_ack')
    lc_port_relay_ack    = LaunchConfiguration('port_relay_ack')
    lc_bind_ip           = LaunchConfiguration('bind_ip')
    lc_telemetry_port    = LaunchConfiguration('telemetry_port')

    # bridge = Node(
    #     package='conveyor_control_topic',
    #     executable='bridge_node',
    #     name='conveyor_bridge',
    #     output='screen',
    #     parameters=[{
    #         'target_ip': lc_target_ip,
    #         'port_conveyor_rx': lc_port_conveyor_rx,
    #         'port_relay_rx': lc_port_relay_rx,
    #         'port_conveyor_ack': lc_port_conveyor_ack,
    #         'port_relay_ack': lc_port_relay_ack,
    #         'bind_ip': lc_bind_ip,
    #         'telemetry_port': lc_telemetry_port,
    #     }]
    # )

    # ถ้าคุณใช้ status_node ของผมด้วย (ปล่อย latency/timestamps): เปิดส่วนนี้
    status = Node(
        package='conveyor_control_topic',
        executable='conveyor_status_node',
        name='conveyor_status_node',
        output='screen',
        parameters=[{
            'enable_udp_telemetry': True,   # หรือ False ถ้าอยากใช้เฉพาะ /conveyor_ack
            'target_ip': lc_target_ip,
            'port_conveyor_rx': lc_port_conveyor_rx,
            'port_relay_rx': lc_port_relay_rx,
            'port_conveyor_ack': lc_port_conveyor_ack,
            'port_relay_ack': lc_port_relay_ack,
            'bind_ip': lc_bind_ip,
            'telemetry_port': lc_telemetry_port,
        }]
    )

    return LaunchDescription([
        target_ip, 
        port_conveyor_rx, 
        port_relay_rx, 
        port_conveyor_ack, 
        port_relay_ack,
        bind_ip, 
        telemetry_port,
        # bridge,
        status
    ])

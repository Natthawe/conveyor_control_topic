# conveyor_control_topic

This ROS 2 package provides a simple UDP node, split into three control boards and listening for different topics, keeping all the nodes separate to prevent clutter, and sending data to an Arduino or microcontroller via UDP.

### Launch Conveyor UDP
    ros2 launch conveyor_control_topic conveyor_udp_node.launch.py

### Launch LED UDP
    ros2 launch conveyor_control_topic led_udp_node.launch.py

### Launch Hook UDP
    ros2 launch conveyor_control_topic ack_hook_relay_udp_node.launch.py
#!/usr/bin/env python3
import socket
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class LedUDPNode(Node):
    def __init__(self):
        super().__init__('led_udp_node')

        # Subscribe Int32 จาก /stopRobotX
        self.subscription = self.create_subscription(
            Int32, '/stopRobotX', self.listener_callback, 10
        )

        # พารามิเตอร์ IP/Port (แก้ตอนรันได้)
        self.declare_parameter('udp_ip', '10.1.100.210')
        self.declare_parameter('udp_port', 8001)

        self.udp_ip = self.get_parameter('udp_ip').get_parameter_value().string_value
        self.udp_port = int(self.get_parameter('udp_port').get_parameter_value().integer_value)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.get_logger().info('LED UDP Node (Int32) started.')
        self.get_logger().info(f'UDP target: {self.udp_ip}:{self.udp_port}')

    def listener_callback(self, msg: Int32):
        val = int(msg.data)
        try:
            # ส่งเป็น Int32 little-endian 4 ไบต์ => ต้องตรงกับฝั่ง Arduino ที่ใช้ memcpy(&val, buf, 4)
            payload = struct.pack('<i', val)
            self.sock.sendto(payload, (self.udp_ip, self.udp_port))
            self.get_logger().info(f'Sent Int32 {val} -> {self.udp_ip}:{self.udp_port}')
        except Exception as e:
            self.get_logger().error(f'UDP send failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LedUDPNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

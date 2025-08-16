#!/usr/bin/env python3
import socket
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

def parse_targets(target_list):
    """รับลิสต์สตริง ['ip:port', ...] -> คืนลิสต์ [(ip, port), ...]"""
    targets = []
    for t in target_list:
        t = t.strip()
        if not t:
            continue
        if ':' not in t:
            raise ValueError(f"Invalid target '{t}', expected 'IP:PORT'")
        ip, port_s = t.split(':', 1)
        ip = ip.strip()
        port = int(port_s.strip())
        
        socket.inet_aton(ip)
        if not (0 < port < 65536):
            raise ValueError(f"Invalid port '{port}' in target '{t}'")
        targets.append((ip, port))
    return targets

class ConveyorUDPNode(Node):
    def __init__(self):
        super().__init__('conveyor_udp_node')

        # ---- Parameters ----
        self.declare_parameter('udp_targets', ['10.1.100.210:8000', '10.1.100.211:8001'])
        self.declare_parameter('send_ascii', False)  # False = ส่งเป็นไบต์เดียว

        targets_param = self.get_parameter('udp_targets').get_parameter_value().string_array_value
        self.send_ascii = self.get_parameter('send_ascii').get_parameter_value().bool_value

        try:
            self.targets = parse_targets(targets_param)
        except Exception as e:
            raise RuntimeError(f"Invalid udp_targets: {e}")

        # ---- UDP socket ----
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ---- ROS sub ----
        self.subscription = self.create_subscription(Int8, '/conveyor_state', self.listener_callback, 10)

        # ---- Logs ----
        self.get_logger().info('Conveyor UDP Node started.')
        self.get_logger().info(f"Targets: {', '.join([f'{ip}:{port}' for ip,port in self.targets])}")
        self.get_logger().info(f"Mode: {'ASCII' if self.send_ascii else 'RAW int8'}")

    def listener_callback(self, msg: Int8):
        v = int(msg.data)
        if v not in (-1, 0, 1, 2, -2):
            self.get_logger().warn(f"Unexpected command {v}, will still forward.")
        try:
            if self.send_ascii:
                payload = str(v).encode('utf-8')  # ส่งเป็นข้อความ เช่น b"-1"
            else:
                # ส่งเป็น 1 ไบต์ signed: -1 -> 0xFF, 0 -> 0x00, 1 -> 0x01, 2 -> 0x02, ...
                payload = struct.pack('b', v)

            for ip, port in self.targets:
                self.sock.sendto(payload, (ip, port))
            self.get_logger().info(f"Forwarded {v} to {len(self.targets)} targets "
                                   f"({'ASCII' if self.send_ascii else 'RAW'})")
        except Exception as e:
            self.get_logger().error(f"UDP send failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorUDPNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

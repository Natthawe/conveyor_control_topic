#!/usr/bin/env python3
import socket
import select
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Int32, String, Int32MultiArray


def make_udp_sock(bind_ip: str, bind_port: int, recv_buf: int = 0, reuse: bool = True):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if reuse:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((bind_ip, bind_port))
    if recv_buf > 0:
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, recv_buf)
        except Exception:
            pass
    s.setblocking(False)
    return s


class ConveyorRelayBridge(Node):
    def __init__(self):
        super().__init__('conveyor_relay_bridge')

        # ===== Parameters =====
        self.declare_parameter('device_ip', '10.1.100.222')
        self.declare_parameter('bind_ip', '0.0.0.0')  # listen all NICs
        self.declare_parameter('conv_cmd_port', 30001)
        self.declare_parameter('conv_status_port', 30002)
        self.declare_parameter('relay_cmd_port', 30010)
        self.declare_parameter('relay_status_port', 30011)
        self.declare_parameter('poll_hz', 50.0)
        self.declare_parameter('recv_buf_size', 8192)
        self.declare_parameter('send_binary', False)  # ถ้าจะส่งแบบ 4 ไบต์ int32 → True (ตอนนี้ใช้ ASCII พิมพ์ง่าย)

        self.device_ip: str = self.get_parameter('device_ip').get_parameter_value().string_value
        self.bind_ip: str = self.get_parameter('bind_ip').get_parameter_value().string_value
        self.conv_cmd_port: int = self.get_parameter('conv_cmd_port').get_parameter_value().integer_value
        self.conv_status_port: int = self.get_parameter('conv_status_port').get_parameter_value().integer_value
        self.relay_cmd_port: int = self.get_parameter('relay_cmd_port').get_parameter_value().integer_value
        self.relay_status_port: int = self.get_parameter('relay_status_port').get_parameter_value().integer_value
        self.poll_hz: float = float(self.get_parameter('poll_hz').get_parameter_value().double_value)
        self.recv_buf_size: int = self.get_parameter('recv_buf_size').get_parameter_value().integer_value
        self.send_binary: bool = self.get_parameter('send_binary').get_parameter_value().bool_value

        # ===== UDP sockets =====
        # 1) สถานะ (บอร์ด → ROS2)
        self.conv_status_sock = make_udp_sock(self.bind_ip, self.conv_status_port, self.recv_buf_size)
        self.relay_status_sock = make_udp_sock(self.bind_ip, self.relay_status_port, self.recv_buf_size)

        # 2) ส่งคำสั่ง (ROS2 → บอร์ด) + รอ ACK บนซ็อกเก็ตเดียวกัน
        self.conv_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.conv_cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.conv_cmd_sock.bind((self.bind_ip, 0))  # ผูกพอร์ตแบบสุ่ม เพื่อให้บอร์ดตอบ ACK กลับมาที่ซ็อกเก็ตนี้
        self.conv_cmd_sock.setblocking(False)

        self.relay_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.relay_cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.relay_cmd_sock.bind((self.bind_ip, 0))
        self.relay_cmd_sock.setblocking(False)

        # ===== ROS pubs/subs =====
        qos = QoSProfile(depth=10)

        # Sub: รับคำสั่งจาก ROS2
        self.create_subscription(Int32, '/conveyor_state', self.cb_conveyor_cmd, qos)
        self.create_subscription(Int32, '/relay_cmd', self.cb_relay_cmd, qos)

        # Pub: ACK
        self.pub_conv_ack = self.create_publisher(String, '/conveyor_ack', qos)
        self.pub_relay_ack = self.create_publisher(String, '/relay_ack', qos)

        # Pub: Conveyor status
        self.pub_sensors = self.create_publisher(Int32MultiArray, '/conveyor/sensors', qos)   # [L, R]
        self.pub_motor_dir = self.create_publisher(Int32, '/conveyor/motor_dir', qos)         # -1/0/1

        # Pub: Relay status
        self.pub_relay_status = self.create_publisher(Int32MultiArray, '/relay/status', qos)  # [r1, r2]

        # Timer: poll sockets
        self.timer = self.create_timer(1.0 / max(self.poll_hz, 1.0), self.poll_sockets)

        self.get_logger().info(f'ConveyorRelayBridge up. Device={self.device_ip}')
        self.get_logger().info(f'Listen conv_status={self.bind_ip}:{self.conv_status_port}, relay_status={self.bind_ip}:{self.relay_status_port}')
        self.get_logger().info(f'Send conv_cmd -> {self.device_ip}:{self.conv_cmd_port}')
        self.get_logger().info(f'Send relay_cmd -> {self.device_ip}:{self.relay_cmd_port}')

    # ========== Command callbacks ==========
    def cb_conveyor_cmd(self, msg: Int32):
        val = int(msg.data)
        try:
            if self.send_binary:
                self.conv_cmd_sock.sendto(val.to_bytes(4, byteorder='little', signed=True),
                                          (self.device_ip, self.conv_cmd_port))
            else:
                # ส่ง ASCII ตรงตามโค้ดบอร์ด (อ่านได้ทั้ง ASCII/4-byte)
                self.conv_cmd_sock.sendto(str(val).encode('ascii'), (self.device_ip, self.conv_cmd_port))
        except Exception as ex:
            self.get_logger().error(f'Conveyor send error: {ex}')

    def cb_relay_cmd(self, msg: Int32):
        val = int(msg.data)
        try:
            if self.send_binary:
                self.relay_cmd_sock.sendto(val.to_bytes(4, byteorder='little', signed=True),
                                           (self.device_ip, self.relay_cmd_port))
            else:
                self.relay_cmd_sock.sendto(str(val).encode('ascii'), (self.device_ip, self.relay_cmd_port))
        except Exception as ex:
            self.get_logger().error(f'Relay send error: {ex}')

    # ========== Poll sockets ==========
    def poll_sockets(self):
        # เร็ว/กินซีพียูน้อย: ใช้ select แบบ timeout 0
        rlist = [self.conv_status_sock, self.relay_status_sock, self.conv_cmd_sock, self.relay_cmd_sock]
        readable, _, _ = select.select(rlist, [], [], 0.0)

        for s in readable:
            try:
                data, (rip, rport) = s.recvfrom(2048)
            except BlockingIOError:
                continue
            except Exception as ex:
                self.get_logger().warn(f'UDP recv error: {ex}')
                continue

            if s is self.conv_status_sock:
                self.handle_conv_status(data)
            elif s is self.relay_status_sock:
                self.handle_relay_status(data)
            elif s is self.conv_cmd_sock:
                # ACK กลับจากบอร์ดหลังส่งคำสั่งสายพาน
                text = data.decode('utf-8', errors='ignore').strip()
                msg = String()
                msg.data = text
                self.pub_conv_ack.publish(msg)
            elif s is self.relay_cmd_sock:
                # ACK กลับจากบอร์ดหลังส่งคำสั่งรีเลย์
                text = data.decode('utf-8', errors='ignore').strip()
                msg = String()
                msg.data = text
                self.pub_relay_ack.publish(msg)

    # ========== Parsers ==========
    def handle_conv_status(self, data: bytes):
        """
        โค้ดบอร์ดส่ง:
          - b"S,<L>,<R>"
          - b"M,<dir>"
        """
        text = data.decode('utf-8', errors='ignore').strip()
        if not text:
            return

        # รองรับกรณีรวมหลายบรรทัด (ปกติบอร์ดส่งบรรทัดเดียว/แพ็กเก็ต)
        for line in text.splitlines():
            parts = line.split(',')
            if not parts:
                continue
            tag = parts[0].strip().upper()
            if tag == 'S' and len(parts) >= 3:
                try:
                    l = int(parts[1]); r = int(parts[2])
                    arr = Int32MultiArray()
                    arr.data = [l, r]
                    self.pub_sensors.publish(arr)
                except ValueError:
                    pass
            elif tag == 'M' and len(parts) >= 2:
                try:
                    d = int(parts[1])
                    m = Int32(); m.data = d
                    self.pub_motor_dir.publish(m)
                except ValueError:
                    pass
            else:
                # ถ้ามีฟอร์แมตอื่น ๆ ก็แค่ log
                self.get_logger().debug(f'Unhandled conv line: {line}')

    def handle_relay_status(self, data: bytes):
        """
        โค้ดบอร์ดส่ง:
          - b"R,<r1>,<r2>"
        """
        text = data.decode('utf-8', errors='ignore').strip()
        if not text:
            return

        for line in text.splitlines():
            parts = line.split(',')
            if parts and parts[0].strip().upper() == 'R' and len(parts) >= 3:
                try:
                    r1 = int(parts[1]); r2 = int(parts[2])
                    arr = Int32MultiArray()
                    arr.data = [r1, r2]
                    self.pub_relay_status.publish(arr)
                except ValueError:
                    pass
            else:
                self.get_logger().debug(f'Unhandled relay line: {line}')


def main():
    rclpy.init()
    node = ConveyorRelayBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


def to_ms() -> int:
    return int(time.time() * 1000)


class UdpBridge(Node):
    """
    ส่งคำสั่งไป Nano และรับ ACK/Telemetry กลับ → ปล่อยเป็นท็อปิก ROS:
      ส่ง: /conveyor_state (Int32) → UDP 10.1.100.222:30001
           /relay_state     (Int32) → UDP 10.1.100.222:30002
      รับ: conv ACK จาก 0.0.0.0:31001 → /conveyor_ack (String)
           relay ACK จาก 0.0.0.0:31002 → /relay_ack (String)
      รับ: telemetry จาก 0.0.0.0:32001 → /conveyor_telemetry (String)  (optional, เผื่อ debug เพิ่ม)

    หมายเหตุ:
      - Nano ฝั่งคุณตั้ง USE_FIXED_ACK_IP=true แล้วจะยิง ACK กลับมาที่ IP เครื่อง ROS:PORT ที่นี่ bind ไว้
      - พารามิเตอร์เปลี่ยนจาก launch ได้
    """

    def __init__(self):
        super().__init__('conveyor_bridge')

        # ==== Parameters ====
        self.declare_parameter('target_ip', '10.1.100.222')
        self.declare_parameter('port_conveyor_rx', 30001)   # ส่งคำสั่ง conveyor ไปที่พอร์ตนี้ของ Nano
        self.declare_parameter('port_relay_rx', 30002)      # ส่งคำสั่ง relay ไปที่พอร์ตนี้ของ Nano
        self.declare_parameter('port_conveyor_ack', 31001)  # รอฟัง ACK คอนเวเยอร์
        self.declare_parameter('port_relay_ack', 31002)     # รอฟัง ACK รีเลย์
        self.declare_parameter('bind_ip', '0.0.0.0')
        self.declare_parameter('telemetry_port', 32001)     # รอฟังเทเลเมทรี (optional)

        self.target_ip = self.get_parameter('target_ip').get_parameter_value().string_value
        self.port_conv_tx = int(self.get_parameter('port_conveyor_rx').value)  # TX to Nano
        self.port_relay_tx = int(self.get_parameter('port_relay_rx').value)    # TX to Nano
        self.port_conv_ack = int(self.get_parameter('port_conveyor_ack').value)
        self.port_relay_ack = int(self.get_parameter('port_relay_ack').value)
        self.bind_ip = self.get_parameter('bind_ip').get_parameter_value().string_value
        self.telemetry_port = int(self.get_parameter('telemetry_port').value)

        # ==== Publishers ====
        self.pub_conv_ack = self.create_publisher(String, '/conveyor_ack', 10)
        self.pub_relay_ack = self.create_publisher(String, '/relay_ack', 10)
        self.pub_tel = self.create_publisher(String, '/conveyor_telemetry', 10)

        # ==== Subscribers ====
        self.sub_conv = self.create_subscription(Int32, '/conveyor_state', self.cb_conv_cmd, 10)
        self.sub_relay = self.create_subscription(Int32, '/relay_state', self.cb_relay_cmd, 10)

        # ==== UDP sockets ====
        # ส่ง (ไม่ต้อง bind)
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_send.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # รับ ACK (2 ช่อง)
        self.sock_conv_ack = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_conv_ack.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_conv_ack.bind((self.bind_ip, self.port_conv_ack))
        self.sock_conv_ack.settimeout(0.5)

        self.sock_relay_ack = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_relay_ack.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_relay_ack.bind((self.bind_ip, self.port_relay_ack))
        self.sock_relay_ack.settimeout(0.5)

        # รับ Telemetry (optional)
        self.sock_tel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_tel.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_tel.bind((self.bind_ip, self.telemetry_port))
        self.sock_tel.settimeout(0.5)

        # ==== Threads ====
        self._stop = threading.Event()
        self.t_conv_ack = threading.Thread(target=self._loop_conv_ack, daemon=True)
        self.t_relay_ack = threading.Thread(target=self._loop_relay_ack, daemon=True)
        self.t_tel = threading.Thread(target=self._loop_tel, daemon=True)

        self.t_conv_ack.start()
        self.t_relay_ack.start()
        self.t_tel.start()

        self.get_logger().info(f"UDP bridge ready.")
        self.get_logger().info(f"Target Nano: {self.target_ip} (conv={self.port_conv_tx}, relay={self.port_relay_tx})")
        self.get_logger().info(f"Listening ACK on conv={self.port_conv_ack}, relay={self.port_relay_ack}")
        self.get_logger().info(f"Telemetry listening on {self.bind_ip}:{self.telemetry_port}")

    # ===== Send commands =====
    def cb_conv_cmd(self, msg: Int32):
        data = str(msg.data).encode('ascii')
        self.sock_send.sendto(data, (self.target_ip, self.port_conv_tx))
        self.get_logger().info(f"[SEND CONV] {msg.data} -> {self.target_ip}:{self.port_conv_tx}")

    def cb_relay_cmd(self, msg: Int32):
        # รีเลย์ฝั่งคุณรับทั้ง ASCII/ไบนารีได้ แต่เราส่ง ASCII เพื่ออ่านง่าย
        data = str(msg.data).encode('ascii')
        self.sock_send.sendto(data, (self.target_ip, self.port_relay_tx))
        self.get_logger().info(f"[SEND RELAY] {msg.data} -> {self.target_ip}:{self.port_relay_tx}")

    # ===== Receive loops =====
    def _loop_conv_ack(self):
        self.get_logger().info(f"Bound conveyor ACK on {self.bind_ip}:{self.port_conv_ack}")
        while not self._stop.is_set():
            try:
                data, addr = self.sock_conv_ack.recvfrom(2048)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"Conv ACK recv error: {e}")
                continue
            text = data.decode(errors='replace').strip()
            self.pub_conv_ack.publish(String(data=text))
            # (debug) self.get_logger().info(f"[ACK CONV] {text}")

    def _loop_relay_ack(self):
        self.get_logger().info(f"Bound relay ACK on {self.bind_ip}:{self.port_relay_ack}")
        while not self._stop.is_set():
            try:
                data, addr = self.sock_relay_ack.recvfrom(2048)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"Relay ACK recv error: {e}")
                continue
            text = data.decode(errors='replace').strip()
            self.pub_relay_ack.publish(String(data=text))
            # (debug) self.get_logger().info(f"[ACK RELAY] {text}")

    def _loop_tel(self):
        while not self._stop.is_set():
            try:
                data, addr = self.sock_tel.recvfrom(2048)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"Telemetry recv error: {e}")
                continue
            text = data.decode(errors='replace').strip()
            # ปล่อยทั้งสตริงไว้ก่อน (status_node ที่ผมให้ก่อนหน้าก็อ่านตรงนี้ได้ถ้าคุณ remap ไป)
            self.pub_tel.publish(String(data=text))

    def destroy_node(self):
        self._stop.set()
        try:
            self.sock_send.close()
        except Exception:
            pass
        for s in [self.sock_conv_ack, self.sock_relay_ack, self.sock_tel]:
            try:
                s.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

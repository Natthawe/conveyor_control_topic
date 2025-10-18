#!/usr/bin/env python3
import socket
import struct
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class ConveyorRelayUDPBridge(Node):
    def __init__(self):
        super().__init__('conveyor_relay_udp_bridge')

        # ========= Parameters =========
        self.declare_parameter('target_ip', '10.1.100.222')   # Nano IP
        self.declare_parameter('port_conveyor_rx', 30001)     # ไปที่ Nano (รับคำสั่งคอนเวเยอร์)
        self.declare_parameter('port_relay_rx', 30002)        # ไปที่ Nano (รับคำสั่งรีเลย์)
        self.declare_parameter('port_conveyor_ack', 31001)    # เราเปิดรอฟัง ACK ที่นี่
        self.declare_parameter('port_relay_ack', 31002)       # เราเปิดรอฟัง ACK ที่นี่
        self.declare_parameter('use_binary_relay', True)      # ส่งรีเลย์เป็น int32 4 ไบต์ (little-endian)
        self.declare_parameter('bind_ip', '0.0.0.0')          # ผูกพอร์ต ACK ที่ IP นี้

        self.target_ip         = self.get_parameter('target_ip').value
        self.port_conv_rx      = int(self.get_parameter('port_conveyor_rx').value)
        self.port_relay_rx     = int(self.get_parameter('port_relay_rx').value)
        self.port_conv_ack     = int(self.get_parameter('port_conveyor_ack').value)
        self.port_relay_ack    = int(self.get_parameter('port_relay_ack').value)
        self.use_binary_relay  = bool(self.get_parameter('use_binary_relay').value)
        self.bind_ip           = self.get_parameter('bind_ip').value

        # ========= Publishers / Subscribers =========
        self.pub_conv_ack = self.create_publisher(String, '/conveyor_ack', 10)
        self.pub_relay_ack = self.create_publisher(String, '/relay_ack', 10)

        self.sub_conv = self.create_subscription(Int32, '/conveyor_state', self.cb_conv, 10)
        self.sub_relay = self.create_subscription(Int32, '/relay_state', self.cb_relay, 10)

        # ========= Sockets (send) =========
        self.sock_conv_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_relay_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_conv_send.settimeout(0.5)
        self.sock_relay_send.settimeout(0.5)

        # ========= Sockets (ACK listeners) =========
        self.sock_conv_ack = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_relay_ack = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_conv_ack.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_relay_ack.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            self.sock_conv_ack.bind((self.bind_ip, self.port_conv_ack))
            self.get_logger().info(f'Bound conveyor ACK on {self.bind_ip}:{self.port_conv_ack}')
        except Exception as e:
            self.get_logger().error(f'Bind conveyor ACK failed: {e}')

        try:
            self.sock_relay_ack.bind((self.bind_ip, self.port_relay_ack))
            self.get_logger().info(f'Bound relay ACK on {self.bind_ip}:{self.port_relay_ack}')
        except Exception as e:
            self.get_logger().error(f'Bind relay ACK failed: {e}')

        # ========= Start listener threads =========
        self._stop = threading.Event()
        self.th_conv = threading.Thread(target=self._ack_loop,
                                        args=(self.sock_conv_ack, self.pub_conv_ack, 'CONV'),
                                        daemon=True)
        self.th_relay = threading.Thread(target=self._ack_loop,
                                         args=(self.sock_relay_ack, self.pub_relay_ack, 'RELAY'),
                                         daemon=True)
        self.th_conv.start()
        self.th_relay.start()

        self.get_logger().info('UDP bridge ready.')
        self.get_logger().info(f'Target Nano: {self.target_ip} (conv={self.port_conv_rx}, relay={self.port_relay_rx})')
        self.get_logger().info(f'Listening ACK on conv={self.port_conv_ack}, relay={self.port_relay_ack}')
        self.get_logger().info(f'use_binary_relay={self.use_binary_relay}')

    # ===== Senders =====
    def cb_conv(self, msg: Int32):
        try:
            # Conveyor firmware expects ASCII (ตามโค้ด Nano)
            data = f'{msg.data}'.encode()
            self.sock_conv_send.sendto(data, (self.target_ip, self.port_conv_rx))
            self.get_logger().info(f'[SEND CONV] {msg.data} -> {self.target_ip}:{self.port_conv_rx}')
        except Exception as e:
            self.get_logger().error(f'CONV send failed: {e}')

    def cb_relay(self, msg: Int32):
        try:
            if self.use_binary_relay:
                data = struct.pack('<i', int(msg.data))
            else:
                data = f'{msg.data}'.encode()
            self.sock_relay_send.sendto(data, (self.target_ip, self.port_relay_rx))
            self.get_logger().info(f'[SEND RELAY] {msg.data} -> {self.target_ip}:{self.port_relay_rx} (binary={self.use_binary_relay})')
        except Exception as e:
            self.get_logger().error(f'RELAY send failed: {e}')

    # ===== ACK listeners =====
    def _ack_loop(self, sock: socket.socket, publisher, tag: str):
        sock.settimeout(0.5)
        while not self._stop.is_set():
            try:
                data, addr = sock.recvfrom(1500)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'[{tag}] recv error: {e}')
                continue

            text = data.decode(errors='replace')
            msg = String()
            msg.data = f'{addr[0]}:{addr[1]} {text}'
            publisher.publish(msg)
            self.get_logger().info(f'[ACK {tag}] {msg.data}')

    def destroy_node(self):
        self._stop.set()
        try:
            self.sock_conv_ack.close()
            self.sock_relay_ack.close()
            self.sock_conv_send.close()
            self.sock_relay_send.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorRelayUDPBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

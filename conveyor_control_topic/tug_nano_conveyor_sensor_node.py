#!/usr/bin/env python3
import socket
import select
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Int32, String, Int32MultiArray


def _parse_reliability(s: str) -> ReliabilityPolicy:
    return ReliabilityPolicy.RELIABLE if (s or "").lower() == "reliable" else ReliabilityPolicy.BEST_EFFORT


def _parse_durability(s: str) -> DurabilityPolicy:
    return DurabilityPolicy.TRANSIENT_LOCAL if (s or "").lower() == "transient_local" else DurabilityPolicy.VOLATILE


class ConveyorUDPNode(Node):
    def __init__(self):
        super().__init__('conveyor_udp_node')

        # ==== Parameters ====
        # Target (Nano)
        self.declare_parameter('cmd_ip', '10.1.100.222')     # Nano IP
        self.declare_parameter('cmd_port', 30001)            # Nano command port

        # Status listener (PC LAN only)
        self.declare_parameter('listen_ip', '10.1.100.100')  # PC IP on LAN
        self.declare_parameter('listen_port', 30002)         # board → PC (S,L,R / M,dir)

        # Topics
        self.declare_parameter('topic_cmd', '/conveyor_state')
        self.declare_parameter('topic_ack', '/conveyor_ack')
        self.declare_parameter('topic_sensors', '/conveyor/sensors')
        self.declare_parameter('topic_motor', '/conveyor/motor_dir')

        # QoS
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('qos_reliability', 'reliable')
        self.declare_parameter('qos_durability', 'volatile')

        # Polling
        self.declare_parameter('poll_hz', 100.0)

        # Send format (ASCII-only per your Nano sketch)
        self.declare_parameter('send_ascii', True)

        # ==== Read params ====
        self.cmd_ip = self.get_parameter('cmd_ip').get_parameter_value().string_value
        self.cmd_port = int(self.get_parameter('cmd_port').get_parameter_value().integer_value)

        self.listen_ip = self.get_parameter('listen_ip').get_parameter_value().string_value
        self.listen_port = int(self.get_parameter('listen_port').get_parameter_value().integer_value)

        topic_cmd = self.get_parameter('topic_cmd').get_parameter_value().string_value
        topic_ack = self.get_parameter('topic_ack').get_parameter_value().string_value
        topic_sensors = self.get_parameter('topic_sensors').get_parameter_value().string_value
        topic_motor = self.get_parameter('topic_motor').get_parameter_value().string_value

        qos_depth = int(self.get_parameter('qos_depth').get_parameter_value().integer_value)
        qos_reliability = _parse_reliability(self.get_parameter('qos_reliability').get_parameter_value().string_value)
        qos_durability = _parse_durability(self.get_parameter('qos_durability').get_parameter_value().string_value)
        self.qos = QoSProfile(depth=qos_depth, reliability=qos_reliability, durability=qos_durability)

        self.poll_hz = float(self.get_parameter('poll_hz').get_parameter_value().double_value)
        self.send_ascii = bool(self.get_parameter('send_ascii').get_parameter_value().bool_value)

        # ==== UDP sockets ====
        # TX: ไม่ bind → OS เลือก ephemeral source-port (คงที่ตลอดอายุซ็อกเก็ต)
        self.sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_tx.setblocking(False)

        # RX: ฟังสถานะจากบอร์ด @ 30002
        self.sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_rx.bind((self.listen_ip, self.listen_port))
        self.sock_rx.setblocking(False)

        # ==== ROS pubs/subs ====
        self.sub_cmd = self.create_subscription(Int32, topic_cmd, self.cb_cmd, self.qos)

        self.pub_ack = self.create_publisher(String, topic_ack, self.qos)
        self.pub_sensors = self.create_publisher(Int32MultiArray, topic_sensors, self.qos)  # [L,R]
        self.pub_motor = self.create_publisher(Int32, topic_motor, self.qos)                # -1/0/1

        # Poll timer
        self.timer = self.create_timer(1.0 / max(1.0, self.poll_hz), self._poll_udp)

        self.get_logger().info(
            f'Conveyor UDP Node started.\n'
            f'  → TX target  : {self.cmd_ip}:{self.cmd_port} (ephemeral source port)\n'
            f'  → RX status  : {self.listen_ip}:{self.listen_port} (S,L,R / M,dir)\n'
            f'  Topics: cmd={topic_cmd}, ack={topic_ack}, sensors={topic_sensors}, motor={topic_motor}\n'
            f'  QoS(depth={qos_depth}, rel={qos_reliability.name}, dur={qos_durability.name})'
        )

    # ==== Send command callback ====
    def cb_cmd(self, msg: Int32):
        val = int(msg.data)
        try:
            # Nano ฝั่ง conveyor อ่านเป็น ASCII (atoi)
            payload = str(val).encode('ascii') if self.send_ascii else str(val).encode('ascii')
            self.sock_tx.sendto(payload, (self.cmd_ip, self.cmd_port))
            self.get_logger().info(f'Sent CMD {val} -> {self.cmd_ip}:{self.cmd_port}')
        except Exception as e:
            self.get_logger().error(f'UDP send failed: {e}')

    # ==== Poll both sockets ====
    def _poll_udp(self):
        rlist, _, _ = select.select([self.sock_tx, self.sock_rx], [], [], 0.0)

        for s in rlist:
            try:
                data, (rip, rport) = s.recvfrom(2048)
            except BlockingIOError:
                continue
            except Exception as e:
                self.get_logger().warn(f'UDP recv error: {e}')
                continue

            if s is self.sock_tx:
                # ACK/echo ที่บอร์ดตอบกลับมาที่ซ็อกเก็ตส่ง
                text = data.decode('utf-8', errors='ignore').strip()
                self.pub_ack.publish(String(data=text))
                self.get_logger().debug(f'ACK: {text}')

            elif s is self.sock_rx:
                # สถานะจากบอร์ด @ 30002, รูปแบบ:
                #   "S,<L>,<R>"  หรือ  "M,<dir>"
                text = data.decode('utf-8', errors='ignore').strip()
                for line in text.splitlines():
                    if not line:
                        continue
                    parts = [p.strip() for p in line.split(',')]
                    if not parts:
                        continue
                    tag = parts[0].upper()
                    if tag == 'S' and len(parts) >= 3:
                        try:
                            l = int(parts[1])
                            r = int(parts[2])
                            arr = Int32MultiArray()
                            arr.data = [l, r]
                            self.pub_sensors.publish(arr)
                        except ValueError:
                            self.get_logger().debug(f'Bad S-line: {line}')
                    elif tag == 'M' and len(parts) >= 2:
                        try:
                            d = int(parts[1])
                            self.pub_motor.publish(Int32(data=d))
                        except ValueError:
                            self.get_logger().debug(f'Bad M-line: {line}')
                    else:
                        self.get_logger().debug(f'Unhandled line: {line}')


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

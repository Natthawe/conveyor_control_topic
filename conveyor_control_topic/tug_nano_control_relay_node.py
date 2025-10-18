#!/usr/bin/env python3
import socket
import struct
import select
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Int32, Float32, String, Int32MultiArray
from gmr_msgs.msg import ControllerStatus


def _parse_reliability(s: str) -> ReliabilityPolicy:
    return ReliabilityPolicy.RELIABLE if (s or "").lower() == "reliable" else ReliabilityPolicy.BEST_EFFORT


def _parse_durability(s: str) -> DurabilityPolicy:
    return DurabilityPolicy.TRANSIENT_LOCAL if (s or "").lower() == "transient_local" else DurabilityPolicy.VOLATILE


class LedUDPNode(Node):
    def __init__(self):
        super().__init__('led_relay_udp_node')

        # ===== Declare params =====
        # Target (Nano)
        self.declare_parameter('udp_ip', '10.1.100.222')   # Nano IP
        self.declare_parameter('udp_port', 30010)          # Nano listen port

        # Local LAN (PC) — ฟังเฉพาะ interface นี้
        self.declare_parameter('listen_ip', '10.1.100.100')    # PC IP (LAN)
        self.declare_parameter('listen_port', 30011)           # รับสถานะจากบอร์ด (R,r1,r2)

        # Topics
        self.declare_parameter('topic_stop', '/stopRobotX')
        self.declare_parameter('topic_status', '/gmr_controllers/status')
        self.declare_parameter('topic_distance', '/closest_object_distance')

        # Distance logic
        self.declare_parameter('distance_threshold', 1.0)
        self.declare_parameter('alert_value', -1)
        self.declare_parameter('clear_value', 0)
        self.declare_parameter('distance_send_clear_on_edge_up', True)

        # Emergency logic
        self.declare_parameter('suppress_alert_when_emergency_active', True)
        self.declare_parameter('rearm_delay_sec', 2.0)

        # QoS
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('qos_reliability', 'reliable')
        self.declare_parameter('qos_durability', 'volatile')

        # Misc
        self.declare_parameter('dedupe_same_value', True)
        self.declare_parameter('require_status_before_distance', True)

        # Polling (อ่าน UDP แบบ non-blocking)
        self.declare_parameter('poll_hz', 100.0)

        # ===== Read params =====
        self.udp_ip = self.get_parameter('udp_ip').get_parameter_value().string_value
        self.udp_port = int(self.get_parameter('udp_port').get_parameter_value().integer_value)

        self.listen_ip = self.get_parameter('listen_ip').get_parameter_value().string_value
        self.listen_port = int(self.get_parameter('listen_port').get_parameter_value().integer_value)

        self.topic_stop = self.get_parameter('topic_stop').get_parameter_value().string_value
        self.topic_status = self.get_parameter('topic_status').get_parameter_value().string_value
        self.topic_distance = self.get_parameter('topic_distance').get_parameter_value().string_value

        self.distance_threshold = float(self.get_parameter('distance_threshold').get_parameter_value().double_value)
        self.alert_value = int(self.get_parameter('alert_value').get_parameter_value().integer_value)
        self.clear_value = int(self.get_parameter('clear_value').get_parameter_value().integer_value)
        self.distance_send_clear_on_edge_up = bool(self.get_parameter('distance_send_clear_on_edge_up').get_parameter_value().bool_value)

        self.suppress_alert_when_emergency_active = bool(self.get_parameter('suppress_alert_when_emergency_active').get_parameter_value().bool_value)
        self.rearm_delay_sec = float(self.get_parameter('rearm_delay_sec').get_parameter_value().double_value)

        qos_depth = int(self.get_parameter('qos_depth').get_parameter_value().integer_value)
        qos_reliability = _parse_reliability(self.get_parameter('qos_reliability').get_parameter_value().string_value)
        qos_durability = _parse_durability(self.get_parameter('qos_durability').get_parameter_value().string_value)
        self.qos = QoSProfile(depth=qos_depth, reliability=qos_reliability, durability=qos_durability)

        self.dedupe_same_value = bool(self.get_parameter('dedupe_same_value').get_parameter_value().bool_value)
        self.require_status_before_distance = bool(self.get_parameter('require_status_before_distance').get_parameter_value().bool_value)

        self.poll_hz = float(self.get_parameter('poll_hz').get_parameter_value().double_value)

        # ===== UDP sockets =====
        # TX socket: ไม่ bind → ให้ OS เลือกพอร์ตต้นทาง (ephemeral) และคงที่ตลอดอายุซ็อกเก็ต
        self.sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_tx.setblocking(False)

        # RX socket สำหรับสถานะรีเลย์จากบอร์ด (R,r1,r2) @ listen_port
        self.sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_rx.bind((self.listen_ip, self.listen_port))
        self.sock_rx.setblocking(False)

        # ===== Subscriptions =====
        self.sub_stop = self.create_subscription(Int32, self.topic_stop, self.stop_callback, self.qos)
        self.sub_status = self.create_subscription(ControllerStatus, self.topic_status, self.status_callback, self.qos)
        self.sub_distance = self.create_subscription(Float32, self.topic_distance, self.distance_callback, self.qos)

        # ===== Publishers =====
        self.pub_relay_status = self.create_publisher(Int32MultiArray, '/relay_status', self.qos)  # [r1, r2]
        self.pub_relay_ack = self.create_publisher(String, '/relay_ack', self.qos)                 # ข้อความ ACK

        # ===== States =====
        self._last_emergency_val = None
        self._emergency_state = None
        self._status_initialized = False
        self._last_sent_val = None
        self._distance_under_threshold = False
        self._delay_timer = None

        # ===== Poll sockets by timer =====
        self.timer = self.create_timer(1.0 / max(1.0, self.poll_hz), self._poll_udp)

        self.get_logger().info(
            f'LED UDP Node started.\n'
            f'  → TX target  : {self.udp_ip}:{self.udp_port} (PC -> NANO)\n'
            f'  → RX status  : {self.listen_ip}:{self.listen_port} (NANO -> PC)\n'
            f'  QoS(depth={qos_depth}, rel={qos_reliability.name}, dur={qos_durability.name})\n'
            f'  Distance(th={self.distance_threshold}, alert={self.alert_value}, clear={self.clear_value}, '
            f'rearm={self.rearm_delay_sec}s, require_status_before_distance={self.require_status_before_distance})'
        )

    # ===== Helpers =====
    def _send_udp_int32(self, val: int):
        ival = int(val)
        if self.dedupe_same_value and self._last_sent_val == ival:
            return
        try:
            payload = struct.pack('<i', ival)  # บอร์ดรองรับ int32 LE
            self.sock_tx.sendto(payload, (self.udp_ip, self.udp_port))
            self._last_sent_val = ival
            self.get_logger().info(f'Sent Int32 {ival} -> {self.udp_ip}:{self.udp_port}')
        except Exception as e:
            self.get_logger().error(f'UDP send failed: {e}')

        # ถ้าส่ง 1 ให้ยกเลิก timer หน่วง alert ทันที
        if ival == 1:
            self._cancel_delay_timer()

    def _emergency_active(self) -> bool:
        return bool(self._emergency_state) is True

    def _cancel_delay_timer(self):
        if self._delay_timer is not None:
            try:
                self._delay_timer.cancel()
            except Exception:
                pass
            self._delay_timer = None
            self.get_logger().debug('Cancelled pending alert timer.')

    def _schedule_delayed_alert(self):
        self._cancel_delay_timer()
        self.get_logger().debug(f'Scheduling alert ({self.alert_value}) in {self.rearm_delay_sec}s.')
        self._delay_timer = self.create_timer(self.rearm_delay_sec, self._delayed_alert_cb)

    def _delayed_alert_cb(self):
        self._cancel_delay_timer()
        if self._distance_under_threshold and not self._emergency_active():
            self._send_udp_int32(self.alert_value)
        else:
            self.get_logger().debug('Skip delayed alert (conditions not met).')

    # ===== Callbacks =====
    def stop_callback(self, msg: Int32):
        ival = int(msg.data)
        if ival == 1:
            self._emergency_state = True
            self._status_initialized = True
        elif ival == 0:
            self._emergency_state = False
            self._status_initialized = True

        self._send_udp_int32(ival)

        if ival == 0 and self._distance_under_threshold:
            self._schedule_delayed_alert()

    def status_callback(self, msg: ControllerStatus):
        val = 1 if msg.emergency_status else 0

        self._emergency_state = (val == 1)
        self._status_initialized = True

        if self._last_emergency_val is None:
            self._last_emergency_val = val
            self.get_logger().info(f'Initial emergency_status = {val} (stored, not sent).')
            return

        if val != self._last_emergency_val:
            self._last_emergency_val = val
            self._send_udp_int32(val)

            if val == 0 and self._distance_under_threshold:
                self._schedule_delayed_alert()

    def distance_callback(self, msg: Float32):
        if self.require_status_before_distance and not self._status_initialized:
            under_probe = (msg.data < self.distance_threshold)
            self._distance_under_threshold = under_probe
            self.get_logger().debug('Distance ignored until emergency status is initialized.')
            return

        under = (msg.data < self.distance_threshold)

        if under and not self._distance_under_threshold:
            self._distance_under_threshold = True
            if self.suppress_alert_when_emergency_active and self._emergency_active():
                self.get_logger().debug('Distance <threshold suppressed (emergency active).')
            else:
                self._send_udp_int32(self.alert_value)
            return

        if (not under) and self._distance_under_threshold:
            self._distance_under_threshold = False
            self._cancel_delay_timer()
            if self.distance_send_clear_on_edge_up and not self._emergency_active():
                self._send_udp_int32(self.clear_value)
            else:
                self.get_logger().debug('Edge-up clear suppressed (emergency active or disabled).')
            return

    # ===== UDP polling =====
    def _poll_udp(self):
        """อ่านทั้ง 'ACK จากพอร์ตต้นทางของซ็อกเก็ตส่ง' และ 'สถานะ R,r1,r2 จากพอร์ต 30011'"""
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
                # ACK จากบอร์ด (บอร์ด echo กลับข้อความ) → publish /relay_ack
                try:
                    text = data.decode('utf-8', errors='ignore').strip()
                except Exception:
                    text = f'<{len(data)} bytes>'
                msg = String()
                msg.data = text
                self.pub_relay_ack.publish(msg)
                self.get_logger().debug(f'ACK: {text}')

            elif s is self.sock_rx:
                # สถานะรีเลย์จากบอร์ด: "R,1,0"
                try:
                    text = data.decode('utf-8', errors='ignore').strip()
                    for line in text.splitlines():
                        if not line:
                            continue
                        if line.startswith('R,'):
                            parts = line.split(',')
                            if len(parts) >= 3:
                                r1 = int(parts[1])
                                r2 = int(parts[2])
                                arr = Int32MultiArray()
                                arr.data = [r1, r2]
                                self.pub_relay_status.publish(arr)
                                # self.get_logger().info(f'Relay status: R1={r1}, R2={r2}')
                        else:
                            self.get_logger().debug(f'Unhandled line on status port: {line}')
                except Exception as e:
                    self.get_logger().warn(f'Parse status error: {e}')


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

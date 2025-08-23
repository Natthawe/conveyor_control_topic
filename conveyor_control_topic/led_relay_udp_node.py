#!/usr/bin/env python3
import socket
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Int32, Float32
from gmr_msgs.msg import ControllerStatus


def _parse_reliability(s: str) -> ReliabilityPolicy:
    return ReliabilityPolicy.RELIABLE if (s or "").lower() == "reliable" else ReliabilityPolicy.BEST_EFFORT

def _parse_durability(s: str) -> DurabilityPolicy:
    return DurabilityPolicy.TRANSIENT_LOCAL if (s or "").lower() == "transient_local" else DurabilityPolicy.VOLATILE


class LedUDPNode(Node):
    def __init__(self):
        super().__init__('led_relay_udp_node')

        # ===== Declare params =====
        self.declare_parameter('udp_ip', '10.1.100.210')
        self.declare_parameter('udp_port', 8001)

        self.declare_parameter('topic_stop', '/stopRobotX')
        self.declare_parameter('topic_status', '/gmr_controllers/status')
        self.declare_parameter('topic_distance', '/closest_object_distance')

        self.declare_parameter('distance_threshold', 1.0)
        self.declare_parameter('alert_value', -1)
        self.declare_parameter('clear_value', 0)
        self.declare_parameter('distance_send_clear_on_edge_up', True)

        self.declare_parameter('suppress_alert_when_emergency_active', True)
        self.declare_parameter('rearm_delay_sec', 2.0)

        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('qos_reliability', 'reliable')
        self.declare_parameter('qos_durability', 'volatile')

        self.declare_parameter('dedupe_same_value', True)

        # ใหม่: บังคับให้ได้สถานะฉุกเฉินก่อน จึงประมวลผล distance (กันบัคตอนสตาร์ท)
        self.declare_parameter('require_status_before_distance', True)

        # ===== Read params =====
        self.udp_ip = self.get_parameter('udp_ip').get_parameter_value().string_value
        self.udp_port = int(self.get_parameter('udp_port').get_parameter_value().integer_value)

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

        # ===== UDP socket =====
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ===== Subscriptions =====
        self.sub_stop = self.create_subscription(Int32, self.topic_stop, self.stop_callback, self.qos)
        self.sub_status = self.create_subscription(ControllerStatus, self.topic_status, self.status_callback, self.qos)
        self.sub_distance = self.create_subscription(Float32, self.topic_distance, self.distance_callback, self.qos)

        # ===== States =====
        self._last_emergency_val = None          # ค่าจาก status ล่าสุด (0/1)
        self._emergency_state = None             # True/False: “สถานะฉุกเฉินที่รู้จริง”
        self._status_initialized = False         # ได้รับสถานะฉุกเฉินอย่างน้อย 1 ครั้งแล้วหรือยัง
        self._last_sent_val = None               # ค่าที่ส่ง UDP ล่าสุด (กันสแปม)
        self._distance_under_threshold = False   # อยู่ < threshold ล่าสุดไหม (สำหรับ edge)
        self._delay_timer = None                 # Timer หน่วงส่ง alert หลังปลดฉุกเฉิน

        self.get_logger().info(
            f'LED UDP Node started. UDP target: {self.udp_ip}:{self.udp_port} | '
            f'th={self.distance_threshold}, alert={self.alert_value}, clear={self.clear_value}, '
            f'rearm_delay={self.rearm_delay_sec}s | require_status_before_distance={self.require_status_before_distance}'
        )
        self.get_logger().info(
            f'Topics: stop={self.topic_stop}, status={self.topic_status}, distance={self.topic_distance} | '
            f'QoS(depth={qos_depth}, rel={qos_reliability.name}, dur={qos_durability.name})'
        )

    # ===== Helpers =====
    def _send_udp_int32(self, val: int):
        ival = int(val)
        if self.dedupe_same_value and self._last_sent_val == ival:
            return
        try:
            payload = struct.pack('<i', ival)
            self.sock.sendto(payload, (self.udp_ip, self.udp_port))
            self._last_sent_val = ival
            self.get_logger().info(f'Sent Int32 {ival} -> {self.udp_ip}:{self.udp_port}')
        except Exception as e:
            self.get_logger().error(f'UDP send failed: {e}')

        # ถ้าส่ง 1 ให้ยกเลิก timer หน่วง alert ทันที
        if ival == 1:
            self._cancel_delay_timer()

    def _emergency_active(self) -> bool:
        """Emergency active ตามสถานะ 'ที่รับมา' ไม่ใช่ค่าที่ส่งออก"""
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
        """หน่วงเวลาส่ง alert_value ถ้ายัง <threshold และไม่อยู่ในฉุกเฉิน"""
        self._cancel_delay_timer()
        self.get_logger().debug(f'Scheduling alert ({self.alert_value}) in {self.rearm_delay_sec}s.')
        self._delay_timer = self.create_timer(self.rearm_delay_sec, self._delayed_alert_cb)

    def _delayed_alert_cb(self):
        # one-shot
        self._cancel_delay_timer()
        if self._distance_under_threshold and not self._emergency_active():
            self._send_udp_int32(self.alert_value)
        else:
            self.get_logger().debug('Skip delayed alert (conditions not met).')

    # ===== Callbacks =====
    def stop_callback(self, msg: Int32):
        """ส่งทุกค่าจาก /stopRobotX ตามเดิม และอัปเดตสถานะฉุกเฉิน"""
        ival = int(msg.data)
        # อัปเดตสถานะจริง
        if ival == 1:
            self._emergency_state = True
            self._status_initialized = True
        elif ival == 0:
            self._emergency_state = False
            self._status_initialized = True

        self._send_udp_int32(ival)

        # ถ้าปลดฉุกเฉินเป็น 0 และยัง <threshold → ตั้งหน่วงส่ง alert
        if ival == 0 and self._distance_under_threshold:
            self._schedule_delayed_alert()

    def status_callback(self, msg: ControllerStatus):
        """Edge-trigger เฉพาะตอน status เปลี่ยน และอัปเดตสถานะฉุกเฉิน"""
        val = 1 if msg.emergency_status else 0

        # อัปเดตสถานะจริง + mark initialized
        self._emergency_state = (val == 1)
        self._status_initialized = True

        if self._last_emergency_val is None:
            self._last_emergency_val = val
            self.get_logger().info(f'Initial emergency_status = {val} (stored, not sent).')
            return

        if val != self._last_emergency_val:
            self._last_emergency_val = val
            self._send_udp_int32(val)

            # ถ้าพึ่งปลดฉุกเฉิน (0) และยังอยู่ <threshold → หน่วงส่ง alert
            if val == 0 and self._distance_under_threshold:
                self._schedule_delayed_alert()

    def distance_callback(self, msg: Float32):
        # ถ้าต้องการให้แน่ใจว่าได้รับสถานะฉุกเฉินก่อน จึงยอมประมวลผล distance
        if self.require_status_before_distance and not self._status_initialized:
            # อัปเดตสถานะ edge ภายในเพื่อความถูกต้องของรอบถัดไป แต่ "ไม่ส่งค่า"
            under_probe = (msg.data < self.distance_threshold)
            self._distance_under_threshold = under_probe
            self.get_logger().debug('Distance ignored until emergency status is initialized.')
            return

        under = (msg.data < self.distance_threshold)

        # Edge-down: เพิ่งต่ำกว่า threshold
        if under and not self._distance_under_threshold:
            self._distance_under_threshold = True
            if self.suppress_alert_when_emergency_active and self._emergency_active():
                self.get_logger().debug('Distance <threshold suppressed (emergency active).')
            else:
                self._send_udp_int32(self.alert_value)
            return

        # Edge-up: เพิ่งกลับขึ้น >= threshold
        if (not under) and self._distance_under_threshold:
            self._distance_under_threshold = False
            self._cancel_delay_timer()
            if self.distance_send_clear_on_edge_up and not self._emergency_active():
                self._send_udp_int32(self.clear_value)
            else:
                self.get_logger().debug('Edge-up clear suppressed (emergency active or disabled).')
            return


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

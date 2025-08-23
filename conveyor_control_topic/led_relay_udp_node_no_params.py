#!/usr/bin/env python3
import socket
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from gmr_msgs.msg import ControllerStatus


class LedUDPNode(Node):
    def __init__(self):
        super().__init__('led_relay_udp_node')

        # ===== Params =====
        self.declare_parameter('udp_ip', '10.1.100.210')
        self.declare_parameter('udp_port', 8001)

        self.udp_ip = self.get_parameter('udp_ip').get_parameter_value().string_value
        self.udp_port = int(self.get_parameter('udp_port').get_parameter_value().integer_value)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ===== Subscriptions =====
        self.sub_stop = self.create_subscription(Int32, '/stopRobotX', self.stop_callback, 10)
        self.sub_status = self.create_subscription(ControllerStatus, '/gmr_controllers/status', self.status_callback, 10)
        self.sub_distance = self.create_subscription(Float32, '/closest_object_distance', self.distance_callback, 10)

        # ===== States =====
        self._last_emergency_val = None        # emergency_status ล่าสุด (0/1)
        self._last_sent_val = None             # ค่าที่ส่ง UDP ล่าสุด (ป้องกันส่งซ้ำ)
        self._distance_under_threshold = False # เคยอยู่ < 1.0 อยู่หรือไม่
        self._delay_timer = None               # Timer หน่วงส่ง -1 หลังปลดฉุกเฉิน

        self.get_logger().info(f'LED UDP Node started. UDP target: {self.udp_ip}:{self.udp_port}')
        self.get_logger().info(
            'Rules: stopRobotX -> ส่งทุกครั้ง | '
            'gmr_controllers/status -> ส่งตอนเปลี่ยน (1/0) | '
            'distance: <1.0 -> -1 (ครั้งเดียว), ≥1.0 -> 0 (ครั้งเดียว, เมื่อไม่อยู่ใน emergency); '
            'ปลดฉุกเฉินเป็น 0 ขณะยัง <1.0 -> หน่วง 2s แล้วส่ง -1'
        )

    # ===== Helpers =====
    def _send_udp_int32(self, val: int):
        """ส่ง Int32 (little-endian) พร้อมกันส่งซ้ำซ้อน"""
        ival = int(val)
        if self._last_sent_val == ival:
            return
        try:
            payload = struct.pack('<i', ival)
            self.sock.sendto(payload, (self.udp_ip, self.udp_port))
            self._last_sent_val = ival
            self.get_logger().info(f'Sent Int32 {ival} -> {self.udp_ip}:{self.udp_port}')
        except Exception as e:
            self.get_logger().error(f'UDP send failed: {e}')

        # ถ้าส่ง 1 ให้ยกเลิก timer หน่วง -1 ทันที
        if ival == 1:
            self._cancel_delay_timer()

    def _emergency_active(self) -> bool:
        """ถือว่า emergency active ถ้าค่าสุดท้ายที่ส่งออกคือ 1"""
        return self._last_sent_val == 1

    def _cancel_delay_timer(self):
        if self._delay_timer is not None:
            try:
                self._delay_timer.cancel()
            except Exception:
                pass
            self._delay_timer = None
            self.get_logger().debug('Cancelled pending -1 timer.')

    def _schedule_delayed_minus1(self, delay_sec: float = 2.0):
        """หน่วงเวลาสend -1 ถ้ายัง <1.0 และไม่อยู่ในฉุกเฉิน"""
        self._cancel_delay_timer()
        self.get_logger().debug(f'Scheduling -1 in {delay_sec}s (if still <1.0 and no emergency).')
        self._delay_timer = self.create_timer(delay_sec, self._delayed_minus1_cb)

    def _delayed_minus1_cb(self):
        # one-shot: ยกเลิกตัวเองทันทีที่เข้า callback
        self._cancel_delay_timer()
        # เงื่อนไขยังคงอยู่หรือไม่?
        if self._distance_under_threshold and not self._emergency_active():
            self._send_udp_int32(-1)
        else:
            self.get_logger().debug('Skip delayed -1 (conditions not met).')

    # ===== Callbacks =====
    def stop_callback(self, msg: Int32):
        """ส่งทุกค่าจาก /stopRobotX ตามเดิม"""
        ival = int(msg.data)
        self._send_udp_int32(ival)

        # ถ้าส่ง 0 เพื่อปลดฉุกเฉิน และยังอยู่ <1.0 → หน่วง 2s แล้วส่ง -1
        if ival == 0 and self._distance_under_threshold:
            self._schedule_delayed_minus1()

    def status_callback(self, msg: ControllerStatus):
        """ส่งเฉพาะตอน emergency_status เปลี่ยน (True->1 / False->0)"""
        val = 1 if msg.emergency_status else 0
        if self._last_emergency_val is None:
            self._last_emergency_val = val
            self.get_logger().info(f'Initial emergency_status = {val}, stored but not sent.')
            return

        if val != self._last_emergency_val:
            self._last_emergency_val = val
            self._send_udp_int32(val)

            # ถ้าพึ่งปลดฉุกเฉิน (0) และยังอยู่ <1.0 → หน่วง 2s แล้วส่ง -1
            if val == 0 and self._distance_under_threshold:
                self._schedule_delayed_minus1()

    def distance_callback(self, msg: Float32):
        under = (msg.data < 1.0)

        # Edge-down: เพิ่งต่ำกว่า 1.0
        if under and not self._distance_under_threshold:
            self._distance_under_threshold = True
            # ถ้าฉุกเฉิน active → ไม่ส่ง -1 ตอนนี้ (รอปลดเป็น 0 แล้วค่อยหน่วงส่ง -1)
            if self._emergency_active():
                self.get_logger().debug('Distance <1.0 suppressed (emergency active).')
            else:
                self._send_udp_int32(-1)
            return

        # Edge-up: เพิ่งกลับขึ้น ≥ 1.0
        if not under and self._distance_under_threshold:
            self._distance_under_threshold = False
            self._cancel_delay_timer()
            # แก้จุดบั๊ก: ถ้า emergency ยัง active อยู่ ห้ามส่ง 0 มากดทับ 1
            if not self._emergency_active():
                self._send_udp_int32(0)
            else:
                self.get_logger().debug('Edge-up >=1.0 but emergency active -> suppress 0.')
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

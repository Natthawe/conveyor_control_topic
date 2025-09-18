#!/usr/bin/env python3
import socket
import struct
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String

ACK_HDR = 0xAC

def parse_targets(target_list):
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
        self.declare_parameter('udp_targets', ['10.1.100.210:8000'])
        self.declare_parameter('send_ascii', False)
        self.declare_parameter('listen_port', 9000)
        self.declare_parameter('ack_timeout_ms', 300)
        self.declare_parameter('ack_retries', 2)
        self.declare_parameter('require_completed', False)
        self.declare_parameter('ack_success_policy', 'all')  # 'all' or 'any'
        # NEW: หน้าต่างรอรับ ACK ชุดเดียวกันต่อ (มิลลิวินาที)
        self.declare_parameter('ack_settle_ms', 150)
        # (ออปชัน) เผยรายละเอียด ACK ทาง topic
        self.declare_parameter('publish_detail', False)

        # ---- Load params ----
        targets_param         = self.get_parameter('udp_targets').get_parameter_value().string_array_value
        self.send_ascii       = self.get_parameter('send_ascii').get_parameter_value().bool_value
        self.listen_port      = int(self.get_parameter('listen_port').get_parameter_value().integer_value)
        self.ack_timeout_ms   = int(self.get_parameter('ack_timeout_ms').get_parameter_value().integer_value)
        self.ack_retries      = int(self.get_parameter('ack_retries').get_parameter_value().integer_value)
        self.require_completed = self.get_parameter('require_completed').get_parameter_value().bool_value
        self.ack_success_policy = self.get_parameter('ack_success_policy').get_parameter_value().string_value.lower()
        self.ack_settle_ms    = int(self.get_parameter('ack_settle_ms').get_parameter_value().integer_value)
        self.publish_detail   = self.get_parameter('publish_detail').get_parameter_value().bool_value

        try:
            self.targets = parse_targets(targets_param)
        except Exception as e:
            raise RuntimeError(f"Invalid udp_targets: {e}")

        # ---- UDP socket ----
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.listen_port))
        self.sock.settimeout(self.ack_timeout_ms / 1000.0)

        # ---- ROS ----
        self.subscription = self.create_subscription(Int8, '/conveyor_state', self.listener_callback, 10)
        self.pub_status = self.create_publisher(Int8, '/conveyor_ack_status', 10)
        self.pub_ack_detail = self.create_publisher(String, '/conveyor_ack', 10) if self.publish_detail else None

        self.seq = 0

        self.get_logger().info('Conveyor UDP Node with ACK started.')
        self.get_logger().info(f"Targets: {', '.join([f'{ip}:{port}' for ip,port in self.targets])}")
        self.get_logger().info(
            f"Mode: {'ASCII' if self.send_ascii else 'RAW int8+seq'} "
            f"listen_port={self.listen_port}, require_completed={self.require_completed}, "
            f"ack_success_policy={self.ack_success_policy}, ack_settle_ms={self.ack_settle_ms}"
        )

    def _next_seq(self):
        self.seq = (self.seq + 1) & 0xFF
        if self.seq == 0:
            self.seq = 1
        return self.seq

    def _wait_ack_once(self, expected_ip: str, seq: int):
        """รอจนได้ ACK (ชุดแรก) จาก expected_ip และ seq ตรง ภายใน ack_timeout_ms"""
        deadline = time.monotonic() + (self.ack_timeout_ms / 1000.0)
        last_detail = ""
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False, -1, last_detail or f"ACK timeout (seq={seq})"
            try:
                self.sock.settimeout(remaining)
                data, addr = self.sock.recvfrom(64)
                src_ip, _ = addr

                if len(data) < 6 or data[0] != ACK_HDR:
                    last_detail = f"Invalid ACK frame len={len(data)} from {src_ip}"
                    continue

                recv_seq = data[1]
                status   = data[2]
                cmd      = struct.unpack('b', data[3:4])[0]
                mirror   = data[4]
                moving   = data[5]

                if src_ip != expected_ip or recv_seq != seq:
                    last_detail = f"ACK not match (from {src_ip}, seq={recv_seq})"
                    continue

                detail = f"ACK {src_ip} seq={recv_seq} status={status} cmd={cmd} mirror={mirror} moving={moving}"
                return True, status, detail

            except socket.timeout:
                return False, -1, last_detail or f"ACK timeout (seq={seq})"

    def _drain_acks_for_seq(self, expected_ip: str, seq: int, settle_ms: int, current_best_status: int, current_detail: str):
        """
        หลังจากได้ ACK แรกแล้ว ให้ดูด ACK ของ seq เดียวกันต่อไปอีก settle_ms มิลลิวินาที
        แล้วคืน 'สถานะสูงสุด' ที่พบ (จัดอันดับ: 0<1<2<3 โดย 3=aborted สำคัญสุด)
        """
        rank = {0:0, 1:1, 2:2, 3:3}
        best_status = current_best_status
        best_detail = current_detail
        deadline = time.monotonic() + (settle_ms / 1000.0)

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                self.sock.settimeout(remaining)
                data, addr = self.sock.recvfrom(64)
                src_ip, _ = addr
                if len(data) < 6 or data[0] != ACK_HDR:
                    continue
                recv_seq = data[1]
                status   = data[2]
                cmd      = struct.unpack('b', data[3:4])[0]
                mirror   = data[4]
                moving   = data[5]

                if src_ip != expected_ip or recv_seq != seq:
                    # ACK ของเครื่องอื่น/seq อื่น — ข้ามไป
                    continue

                # อัปเดตถ้า "สถานะสูงกว่า"
                if rank.get(status, -1) >= rank.get(best_status, -1):
                    best_status = status
                    best_detail = f"ACK {src_ip} seq={recv_seq} status={status} cmd={cmd} mirror={mirror} moving={moving}"

                # ถ้าเจอ aborted(3) หรือ completed(2) แล้ว ถือว่าได้สถานะสุดท้าย ตัดจบเร็วได้
                if status in (2, 3):
                    break

            except socket.timeout:
                break

        return best_status, best_detail

    def _wait_until_completed(self, expected_ip: str, seq: int):
        """รอจน status เป็น completed(2) หรือ aborted(3) โดย retry ตาม ack_retries"""
        tries = self.ack_retries + 1
        last_detail = ""
        while tries > 0:
            ok, status, detail = self._wait_ack_once(expected_ip, seq)
            last_detail = detail
            if ok and status in (2, 3):
                return (status == 2), detail
            tries -= 1
        return False, last_detail or "No completed/aborted ACK"

    def _send_with_ack(self, ip: str, port: int, cmd: int, seq: int):
        """ส่งไปยังเป้าหมายเดียว + รอ ACK + (ออปชัน) drain เพื่อตามสถานะสูงสุด หรือรอจน completed"""
        if self.send_ascii:
            payload = str(cmd).encode('utf-8')
        else:
            payload = struct.pack('bB', cmd, seq)

        attempts = self.ack_retries + 1
        last_detail = ""
        for attempt in range(1, attempts + 1):
            try:
                self.sock.sendto(payload, (ip, port))
            except Exception as e:
                last_detail = f"UDP send failed to {ip}:{port}: {e}"
                return False, last_detail

            # รอ ACK แรก
            ok, status, detail = self._wait_ack_once(ip, seq) if not self.send_ascii else (True, 0, f"ASCII sent {ip}:{port}")
            last_detail = detail
            if not ok:
                self.get_logger().warn(f"[{ip}] attempt {attempt}/{attempts}: {detail}")
                continue

            # ถ้าบังคับต้อง complete จริง ให้รอจนเจอ 2/3
            if self.require_completed and not self.send_ascii:
                ok2, detail2 = self._wait_until_completed(ip, seq)
                last_detail = detail2
                return ok2, last_detail

            # ไม่ require_completed → drain ภายใน ack_settle_ms เพื่ออัปเดตสถานะสูงสุด
            if not self.send_ascii and self.ack_settle_ms > 0:
                status, detail = self._drain_acks_for_seq(ip, seq, self.ack_settle_ms, status, detail)
                last_detail = detail

            # กรณีสั่งรีเลย์ (2/-2) — ปกติ Arduino จะส่ง completed ติด ๆ กับ received
            # drain ข้างบนจะอัพเดตให้เป็น status=2 อยู่แล้ว (ถ้าทันหน้าต่างเวลา)
            return True, last_detail

        return False, last_detail

    def listener_callback(self, msg: Int8):
        cmd = int(msg.data)
        if cmd not in (-2, -1, 0, 1, 2):
            self.get_logger().warn(f"Unexpected command {cmd}, forwarding anyway.")

        seq = self._next_seq()
        results = []
        ok_flags = []

        for ip, port in self.targets:
            ok, detail = self._send_with_ack(ip, port, cmd, seq)
            ok_flags.append(ok)
            tag = "OK" if ok else "FAIL"
            results.append(f"{ip}:{port} {tag} {detail}")

        # รวมผลเป็น 0/1
        if self.ack_success_policy == 'any':
            success = 1 if any(ok_flags) else 0
        else:
            success = 1 if all(ok_flags) else 0

        summary = " | ".join(results)
        self.get_logger().info(f"Send result (cmd={cmd}, seq={seq}): {summary}  => STATUS={success}")
        if self.pub_ack_detail is not None:
            self.pub_ack_detail.publish(String(data=summary))
        self.pub_status.publish(Int8(data=success))

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

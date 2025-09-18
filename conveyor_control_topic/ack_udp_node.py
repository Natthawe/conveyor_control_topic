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
        # NEW: รวมผลเป็น 0/1 แบบไหน? 'all' หรือ 'any'
        self.declare_parameter('ack_success_policy', 'all')

        targets_param        = self.get_parameter('udp_targets').get_parameter_value().string_array_value
        self.send_ascii      = self.get_parameter('send_ascii').get_parameter_value().bool_value
        self.listen_port     = int(self.get_parameter('listen_port').get_parameter_value().integer_value)
        self.ack_timeout_ms  = int(self.get_parameter('ack_timeout_ms').get_parameter_value().integer_value)
        self.ack_retries     = int(self.get_parameter('ack_retries').get_parameter_value().integer_value)
        self.require_completed = self.get_parameter('require_completed').get_parameter_value().bool_value
        self.ack_success_policy = self.get_parameter('ack_success_policy').get_parameter_value().string_value.lower()

        try:
            self.targets = parse_targets(targets_param)
        except Exception as e:
            raise RuntimeError(f"Invalid udp_targets: {e}")

        # ---- UDP socket (bind เพื่อรับ ACK) ----
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.listen_port))
        self.sock.settimeout(self.ack_timeout_ms / 1000.0)

        # ---- ROS ----
        self.subscription = self.create_subscription(Int8, '/conveyor_state', self.listener_callback, 10)
        # self.pub_ack = self.create_publisher(String, '/conveyor_ack', 10)         # รายละเอียดข้อความเดิม
        self.pub_status = self.create_publisher(Int8, '/conveyor_ack_status', 10) # 0/1 สรุปผล

        self.seq = 0

        self.get_logger().info('Conveyor UDP Node with ACK started.')
        self.get_logger().info(f"Targets: {', '.join([f'{ip}:{port}' for ip,port in self.targets])}")
        self.get_logger().info(
            f"Mode: {'ASCII' if self.send_ascii else 'RAW int8+seq'} "
            f"listen_port={self.listen_port}, require_completed={self.require_completed}, "
            f"ack_success_policy={self.ack_success_policy}"
        )

    def _next_seq(self):
        self.seq = (self.seq + 1) & 0xFF
        if self.seq == 0:
            self.seq = 1
        return self.seq

    def _wait_ack_once(self, expected_ip: str, seq: int):
        """รอจนกว่าได้ ACK จาก expected_ip และ seq ตรง ภายใน ack_timeout_ms (อ่านหลายแพ็กเก็ตจนหมดเวลา)"""
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

    def _wait_until_completed(self, expected_ip: str, seq: int):
        """รอจนกว่า status จะเป็น completed(2) หรือ aborted(3) ภายในรอบ retry เดิม"""
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
        """ส่งไปยังเป้าหมายเดียว + รอ ACK + retry; ถ้าตั้ง require_completed=True จะรอต่อจนจบ"""
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

            ok, status, detail = self._wait_ack_once(ip, seq) if not self.send_ascii else (True, 0, f"ASCII sent {ip}:{port}")
            last_detail = detail
            if not ok:
                self.get_logger().warn(f"[{ip}] attempt {attempt}/{attempts}: {detail}")
                continue

            # คำสั่งรีเลย์ (2/-2): ถือว่าจบเมื่อได้ ACK แรก
            if cmd in (2, -2):
                return True, last_detail + " (relay)"

            if not self.require_completed or self.send_ascii:
                return True, last_detail

            # ต้องรอจน completed/aborted
            ok2, detail2 = self._wait_until_completed(ip, seq)
            last_detail = detail2
            return ok2, last_detail

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

        # สรุปผลรวมเป็น 0/1 ตามนโยบาย
        if self.ack_success_policy == 'any':
            success = 1 if any(ok_flags) else 0
        else:
            # 'all' เป็นค่า default
            success = 1 if all(ok_flags) else 0

        summary = " | ".join(results)
        self.get_logger().info(f"Send result (cmd={cmd}, seq={seq}): {summary}  => STATUS={success}")
        # self.pub_ack.publish(String(data=summary))
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
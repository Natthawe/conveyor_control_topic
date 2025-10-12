#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int32MultiArray
import socket
import threading

class ConveyorUDPNode(Node):
    def __init__(self):
        super().__init__('conveyor_udp_node')

        # ---- Params ----
        self.declare_parameter('udp_ip', '10.1.100.222')   # IP ของ Nano
        self.declare_parameter('udp_port', 30001)          # พอร์ตปลายทางบน Nano
        self.declare_parameter('listen_port', 30002)       # พอร์ตฝั่ง ROS ที่จะ bind เพื่อรับสถานะ
        self.udp_ip = self.get_parameter('udp_ip').get_parameter_value().string_value
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.listen_port = self.get_parameter('listen_port').get_parameter_value().integer_value

        # ---- UDP socket (ทั้งส่ง/รับ) ----
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', self.listen_port))   # ใช้พอร์ตคงที่ เพื่อให้นาโน reply กลับมาที่พอร์ตนี้
        self.sock.setblocking(False)

        # ---- ROS I/O ----
        self.subscription = self.create_subscription(
            Int8, '/conveyor_state', self.listener_callback, 10
        )
        self.pub_sensor_arr = self.create_publisher(Int32MultiArray, '/conveyor/sensor', 10)
        self.pub_sensor_L   = self.create_publisher(Int8, '/conveyor/sensor_left', 10)
        self.pub_sensor_R   = self.create_publisher(Int8, '/conveyor/sensor_right', 10)
        self.pub_motor_dir  = self.create_publisher(Int8, '/conveyor/motor_dir', 10)

        # Timer โพลล์ UDP เข้ามา (1kHz ก็ยังสบาย แต่ตั้ง 100Hz พอ)
        self.create_timer(0.01, self.poll_udp)

        self.get_logger().info('Conveyor UDP Node Started.')
        self.get_logger().info(f'Send  → {self.udp_ip}:{self.udp_port}')
        self.get_logger().info(f'Listen on UDP port {self.listen_port}')

    # ส่งคำสั่งไป Nano: 1 / -1 / 0 (ตามโค้ดเดิม)
    def listener_callback(self, msg: Int8):
        command = str(int(msg.data))
        self.get_logger().info(f'Sending UDP command: {command}')
        try:
            self.sock.sendto(command.encode('ascii'), (self.udp_ip, self.udp_port))
        except Exception as e:
            self.get_logger().error(f'UDP send error: {e}')

    # รับสถานะจาก Nano
    def poll_udp(self):
        try:
            while True:
                data, addr = self.sock.recvfrom(512)
                line = data.decode('ascii', errors='ignore').strip()
                if not line:
                    continue
                # รองรับหลายบรรทัดในแพ็กเก็ตเดียว (กันกรณีส่งติด ๆ กัน)
                for part in line.splitlines():
                    self.handle_line(part.strip())
        except BlockingIOError:
            pass
        except Exception as e:
            self.get_logger().warn(f'UDP recv error: {e}')

    def handle_line(self, line: str):
        # รูปแบบ:
        #   S,<L>,<R>    เช่น S,1,0
        #   M,<dir>      เช่น M,1 | M,0 | M,-1
        if not line:
            return
        if line.startswith('S,'):
            try:
                _, rest = line.split('S,', 1)
                L_str, R_str = rest.split(',', 1)
                L = 1 if int(L_str) != 0 else 0
                R = 1 if int(R_str) != 0 else 0
                # publish array
                arr = Int32MultiArray()
                arr.data = [L, R]
                self.pub_sensor_arr.publish(arr)
                # publish รายตัว
                mL, mR = Int8(), Int8()
                mL.data = L
                mR.data = R
                self.pub_sensor_L.publish(mL)
                self.pub_sensor_R.publish(mR)
                self.get_logger().debug(f'RX Sensors L={L} R={R}')
            except Exception as e:
                self.get_logger().warn(f'Parse S line error [{line}]: {e}')
        elif line.startswith('M,'):
            try:
                _, d = line.split('M,', 1)
                dir_val = int(d)
                msg = Int8()
                msg.data = dir_val
                self.pub_motor_dir.publish(msg)
                self.get_logger().debug(f'RX Motor dir={dir_val}')
            except Exception as e:
                self.get_logger().warn(f'Parse M line error [{line}]: {e}')
        else:
            # ข้อความอื่น ๆ จาก Nano (เช่น log, "Command received:")
            self.get_logger().debug(f'UDP: {line}')

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorUDPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

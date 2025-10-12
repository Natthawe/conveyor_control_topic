# tray_bridge_node.py
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

STATUS_MAP = {
    0: "IDLE_DISABLED",
    1: "RUNNING",
    2: "SOFTSTOPPING",
    3: "HARD_STOPPED",
    4: "TARGET_REACHED",
    6: "ENABLED_IDLE",
    5: "ERROR",
}

class TrayBridgeNode(Node):
    def __init__(self):
        super().__init__('tray_bridge')

        # ----- Parameters -----
        self.declare_parameter('udp_cmd_ip',   '10.1.100.222')  # Nano IP
        self.declare_parameter('udp_cmd_port', 30001)           # Nano cmd port
        self.declare_parameter('status_listen_port', 30002)     # local port to receive status
        self.declare_parameter('status_topic', '/tray_status')
        self.declare_parameter('cmd_topic',    '/tray_cmd')

        self.udp_cmd_ip   = self.get_parameter('udp_cmd_ip').get_parameter_value().string_value
        self.udp_cmd_port = int(self.get_parameter('udp_cmd_port').value)
        self.status_port  = int(self.get_parameter('status_listen_port').value)
        status_topic      = self.get_parameter('status_topic').get_parameter_value().string_value
        cmd_topic         = self.get_parameter('cmd_topic').get_parameter_value().string_value

        # ----- ROS pubs/subs -----
        self.pub_status = self.create_publisher(Int32, status_topic, 10)
        self.sub_cmd    = self.create_subscription(Int32, cmd_topic, self.cmd_cb, 10)

        # ----- UDP sockets -----
        # Sender (commands -> Nano)
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Receiver (status <- Nano)
        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx_sock.bind(('', self.status_port))
        self.rx_sock.setblocking(False)  # non-blocking

        self.get_logger().info(
            f'TrayBridge started. CMD -> {self.udp_cmd_ip}:{self.udp_cmd_port}, '
            f'STATUS <- UDP:{self.status_port}'
        )
        self.get_logger().info(f'Topics: cmd={cmd_topic} (Int32), status={status_topic} (Int32)')

        # poll status at 50 Hz
        self.timer = self.create_timer(0.02, self.poll_status)

    # ----- Callbacks -----
    def cmd_cb(self, msg: Int32):
        # forward integer as ASCII string ("-1","0","1","2")
        val = int(msg.data)
        payload = str(val).encode()
        try:
            self.tx_sock.sendto(payload, (self.udp_cmd_ip, self.udp_cmd_port))
            self.get_logger().info(
                f'Sent UDP cmd → {self.udp_cmd_ip}:{self.udp_cmd_port}  data={val}'
            )
        except Exception as e:
            self.get_logger().error(f'UDP send error: {e}')

    def poll_status(self):
        # drain all pending datagrams
        while True:
            try:
                data, addr = self.rx_sock.recvfrom(64)
            except BlockingIOError:
                break
            except Exception as e:
                self.get_logger().error(f'UDP recv error: {e}')
                break

            txt = data.decode(errors='ignore').strip()
            try:
                val = int(txt)
                self.pub_status.publish(Int32(data=val))
                label = STATUS_MAP.get(val, "UNKNOWN")
                self.get_logger().info(f'Got status {val} ({label}) from {addr}')
            except ValueError:
                self.get_logger().warn(f'Non-int status payload: "{txt}" from {addr}')

def main(args=None):
    rclpy.init(args=args)
    node = TrayBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

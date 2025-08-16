import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import socket

class ConveyorUDPNode(Node):
    def __init__(self):
        super().__init__('conveyor_udp_node')
        self.subscription = self.create_subscription(
            Int8,
            '/stopRobotX',
            self.listener_callback,
            10
        )
        self.declare_parameter('udp_ip', '10.1.100.222')
        self.declare_parameter('udp_port', 30001)

        self.udp_ip = self.get_parameter('udp_ip').get_parameter_value().string_value
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info('Conveyor UDP Node Started.')
        self.get_logger().info(f'UDP IP: {self.udp_ip}')
        self.get_logger().info(f'UDP Port: {self.udp_port}')

    def listener_callback(self, msg):
        command = str(msg.data)
        self.get_logger().info(f'Sending UDP command: {command}')
        self.sock.sendto(command.encode(), (self.udp_ip, self.udp_port))

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorUDPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

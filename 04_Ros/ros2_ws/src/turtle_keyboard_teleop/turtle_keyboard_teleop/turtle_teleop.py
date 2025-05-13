import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class TurtleTeleop(Node):
    def __init__(self):
        super().__init__('turtle_teleop')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("Controla con W/A/S/D. Ctrl+C para salir.")

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                key = self.get_key()
                msg = Twist()

                if key == 'w':
                    msg.linear.x = 2.0
                elif key == 's':
                    msg.linear.x = -2.0
                elif key == 'a':
                    msg.angular.z = 2.0
                elif key == 'd':
                    msg.angular.z = -2.0
                elif key == '\x03':  # Ctrl+C
                    break

                self.publisher.publish(msg)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
        return key

def main(args=None):
    rclpy.init(args=args)
    node = TurtleTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

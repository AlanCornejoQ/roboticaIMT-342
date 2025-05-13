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
        self.linear_scale = 1.0
        self.angular_scale = 1.0
        self.get_logger().info("Control con W/A/S/D | Velocidad: 1–9 | Ctrl+C para salir")

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                key = self.get_key()
                msg = Twist()

                # Movimiento
                if key == 'w':
                    msg.linear.x = self.linear_scale
                    print(f"Avanzar [{self.linear_scale}]")
                elif key == 's':
                    msg.linear.x = -self.linear_scale
                    print(f"Retroceder [{self.linear_scale}]")
                elif key == 'a':
                    msg.angular.z = self.angular_scale
                    print(f"Girar izquierda [{self.angular_scale}]")
                elif key == 'd':
                    msg.angular.z = -self.angular_scale
                    print(f"Girar derecha [{self.angular_scale}]")

                # Velocidad
                elif key in '123456789':
                    factor = int(key) / 5.0  # escalar entre 0.2 y 1.8
                    self.linear_scale = round(factor, 2)
                    self.angular_scale = round(factor, 2)
                    print(f"⚙ Velocidad actualizada: lineal = {self.linear_scale}, angular = {self.angular_scale}")
                    continue  # no publica movimiento, solo cambia escala

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


# keyboard_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import sys
import termios
import tty

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.pub = self.create_publisher(Int8, 'motor_cmd', 10)
        self.timer = self.create_timer(0.1, self.read_key)

    def read_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
            msg = Int8()

            if ch == '\x1b':  # arrow keys
                next1, next2 = sys.stdin.read(2)
                if next2 == 'A':
                    msg.data = 1  # up
                elif next2 == 'B':
                    msg.data = -1  # down
                else:
                    msg.data = 0
                self.pub.publish(msg)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main():
    rclpy.init()
    node = KeyboardController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

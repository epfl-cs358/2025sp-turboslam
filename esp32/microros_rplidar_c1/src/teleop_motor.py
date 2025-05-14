# teleop_motor.py
import sys, tty, termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_motor')
        self.pub = self.create_publisher(Int8, 'motor_cmd', 10)
        self.get_logger().info('Use ↑/↓ to drive motor')
        self.orig = termios.tcgetattr(sys.stdin)
    def run(self):
        tty.setraw(sys.stdin)
        while True:
            ch = sys.stdin.read(1)
            if ch == '\x1b':  # start of arrow key
                sys.stdin.read(1)
                c2 = sys.stdin.read(1)
                msg = Int8()
                msg.data = 1 if c2=='A' else -1 if c2=='B' else 0
                self.pub.publish(msg)
            elif ch == 'q':
                break
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig)

def main():
    rclpy.init()
    t = Teleop()
    t.run()
    rclpy.shutdown()

if __name__=='__main__':
    main()

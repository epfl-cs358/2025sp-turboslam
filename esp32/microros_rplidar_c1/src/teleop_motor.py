# # teleop_motor.py
# import sys, tty, termios
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int8

# class Teleop(Node):
#     def __init__(self):
#         super().__init__('teleop_motor')
#         self.pub = self.create_publisher(Int8, 'motor_cmd', 10)
#         self.get_logger().info('Use ↑/↓ to drive motor')
#         self.orig = termios.tcgetattr(sys.stdin)
#     def run(self):
#         tty.setraw(sys.stdin)
#         while True:
#             ch = sys.stdin.read(1)
#             if ch == '\x1b':  # start of arrow key
#                 sys.stdin.read(1)
#                 c2 = sys.stdin.read(1)
#                 msg = Int8()
#                 msg.data = 1 if c2=='A' else -1 if c2=='B' else 0
#                 self.pub.publish(msg)
#             elif ch == 'q':
#                 break
#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig)

# def main():
#     rclpy.init()
#     t = Teleop()
#     t.run()
#     rclpy.shutdown()

# if __name__=='__main__':
#     main()

#!/usr/bin/env python3
import sys, tty, termios, select, time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int32

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        # Publishers
        self.motor_pub = self.create_publisher(Int8,  'motor_cmd',      10)
        self.steer_pub = self.create_publisher(Int32, 'servo_dir/angle', 10)

        # Teleop state
        self.drive_power = 1     # Int8: Motor driving power, constrained to -1 (reverse), 0 (stop), or 1 (forward)
        self.angle       = 90    # starting at center
        self.step_deg    = 5

        # Key‐hold tracking
        self.w_held = False
        self.s_held = False
        self.last_drive_ts = time.time()
        self.drive_timeout = 0.2  # seconds to wait before assuming release

        # Terminal setup
        self.orig_attrs = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)

        self.get_logger().info("Use W/S to drive, A/D to steer, X to stop, Q to quit. Key releases are detected after a timeout of 0.2 seconds.")

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_attrs)

    def publish_drive(self, val:int):
        msg = Int8()
        msg.data = val
        self.motor_pub.publish(msg)

    def publish_steer(self):
        msg = Int32()
        msg.data = self.angle
        self.steer_pub.publish(msg)

    def run(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

                # 1) Non-blocking key read
                if select.select([sys.stdin], [], [], 0)[0]:
                    c = sys.stdin.read(1).lower()
                    if c == 'q':
                        break
                    elif c == 'w':
                        self.w_held = True
                        self.s_held = False
                        self.last_drive_ts = time.time()
                        self.publish_drive(self.drive_power)
                    elif c == 's':
                        self.s_held = True
                        self.w_held = False
                        self.last_drive_ts = time.time()
                        self.publish_drive(-self.drive_power)
                    elif c == 'x':
                        self.w_held = self.s_held = False
                        self.publish_drive(0)
                    elif c == 'a':
                        self.angle = max(0, self.angle - self.step_deg)
                        self.publish_steer()
                    elif c == 'd':
                        self.angle = min(180, self.angle + self.step_deg)
                        self.publish_steer()

                # 2) Detect release (timeout)
                now = time.time()
                if (self.w_held or self.s_held) and (now - self.last_drive_ts > self.drive_timeout):
                    # assume key released
                    self.w_held = self.s_held = False
                    self.publish_drive(0)

        finally:
            self.restore_terminal()

def main():
    rclpy.init()
    node = Teleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

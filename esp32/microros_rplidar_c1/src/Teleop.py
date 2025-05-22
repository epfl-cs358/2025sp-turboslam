#!/usr/bin/env python3
import sys, tty, termios, select, time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int32, Bool
import sys, termios, tty, select

obstacle = False

def obstacle_callback(msg):
        global obstacle
        obstacle = msg.data    

class Teleop(Node):


    def __init__(self):
        super().__init__('teleop_keyboard')
        # Publishers
        self.motor_pub = self.create_publisher(Int8,  'motor_cmd',      10)
        self.steer_pub = self.create_publisher(Int32, 'servo_dir/angle', 10)

        #Subscriber
        self.create_subscription(Bool, '/emergency_stop', obstacle_callback,10)

        # Teleop state
        self.drive_power = 1   # Motor driving power, limited to -0.3 to 0.3
        self.angle       = 90    # starting at center (90 degrees)
        self.step_deg    = 5
        self.min_angle   = 60    # minimum steering angle (30 degrees left of center)
        self.max_angle   = 120   # maximum steering angle (30 degrees right of center)

        # Key‐hold tracking
        self.w_held = False
        self.s_held = False
        self.last_drive_ts = time.time()
        self.drive_timeout = 0.1  # seconds to wait before assuming release

        # Terminal setup
        self.fd = sys.stdin.fileno()
        self.orig_attrs = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        self.get_logger().info("Use W/S to drive, A/D to steer, X to stop, Q to quit. Key releases are detected after a timeout of 0.2 seconds.")
        self.get_logger().info(f"Steering limited to {self.min_angle}° - {self.max_angle}° (±30° from center)")
        self.get_logger().info(f"Drive power limited to ±{self.drive_power}")


    def restore_terminal(self):
        termios.tcsetattr(sys.fd, termios.TCSADRAIN, self.orig_attrs)

    def publish_drive(self, val:float):
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
                        if obstacle:
                            self.w_held = self.s_held = False
                            self.publish_drive(0)
                        else:
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
                        self.angle = max(self.min_angle, self.angle - self.step_deg)
                        self.publish_steer()
                        if self.w_held or self.s_held:
                            self.last_drive_ts = time.time()
                            self.publish_drive(self.drive_power if self.w_held else -self.drive_power)

                    elif c == 'd':
                        self.angle = min(self.max_angle, self.angle + self.step_deg)
                        self.publish_steer()
                        if self.w_held or self.s_held:
                            self.last_drive_ts = time.time()
                            self.publish_drive(self.drive_power if self.w_held else -self.drive_power)


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
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import sys
import termios
import tty

class AckermannKeyboard(Node):
    def __init__(self):
        super().__init__('ackermann_keyboard')
        self.pub = self.create_publisher(
            AckermannDriveStamped,
            '/arkermann_car/cmd_ackermann',
            10
        )

        self.speed = 0.0
        self.steering = 0.0

        self.speed_step = 0.2
        self.steering_step = 0.1

        self.get_logger().info("""
Ackermann Keyboard Teleop
------------------------
W / S : increase / decrease speed
A / D : steer left / right
SPACE : stop
CTRL+C to quit
""")

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def run(self):
        while rclpy.ok():
            key = self.get_key()

            if key == 'w':
                self.speed += self.speed_step
            elif key == 's':
                self.speed -= self.speed_step
            elif key == 'a':
                self.steering += self.steering_step
            elif key == 'd':
                self.steering -= self.steering_step
            elif key == ' ':
                self.speed = 0.0
                self.steering = 0.0
            elif key == '\x03':
                break

            msg = AckermannDriveStamped()
            msg.drive.speed = self.speed
            msg.drive.steering_angle = self.steering

            self.pub.publish(msg)

def main():
    rclpy.init()
    node = AckermannKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

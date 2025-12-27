#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

class CmdVelSwitch(Node):

    def __init__(self):
        super().__init__('cmd_vel_switch')

        self.mode = 0  # 0=MANUAL, 1=AUTO
        self.teleop_cmd = Twist()
        self.auto_cmd = Twist()

        self.create_subscription(Twist, '/cmd_vel_teleop', self.teleop_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_auto', self.auto_cb, 10)
        self.create_subscription(Int8, '/control_mode', self.mode_cb, 10)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.05, self.publish_cmd)

        self.get_logger().info("CMD_VEL switch started")

    def teleop_cb(self, msg):
        self.teleop_cmd = msg

    def auto_cb(self, msg):
        self.auto_cmd = msg

    def mode_cb(self, msg):
        self.mode = msg.data

    def publish_cmd(self):
        if self.mode == 0:
            self.pub.publish(self.teleop_cmd)
        else:
            self.pub.publish(self.auto_cmd)

def main():
    rclpy.init()
    node = CmdVelSwitch()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

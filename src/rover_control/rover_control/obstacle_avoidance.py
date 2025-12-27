#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_fsm')

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher (ONLY cmd_vel)
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Parameters
        self.safe_dist = 0.6
        self.forward_speed = 0.18
        self.reverse_speed = -0.15
        self.turn_speed = 0.7

        self.stop_time = 0.3
        self.reverse_time = 2.0
        self.turn_time = 2.0

        # FSM state
        self.state = "FORWARD"
        self.state_start_time = time.time()

        # Lidar data
        self.front_min = float('inf')
        self.left_min = float('inf')
        self.right_min = float('inf')

        self.get_logger().info("FSM Obstacle Avoidance Node Started")

    # -------------------------
    # LIDAR CALLBACK
    # -------------------------
    def scan_callback(self, msg):
        ranges = msg.ranges
        n = len(ranges)

        def clean(data):
            return [d for d in data if not math.isinf(d) and not math.isnan(d)]

        front = clean(ranges[int(0.45*n):int(0.55*n)])
        left  = clean(ranges[int(0.65*n):int(0.85*n)])
        right = clean(ranges[int(0.15*n):int(0.35*n)])

        self.front_min = min(front) if front else float('inf')
        self.left_min  = min(left) if left else float('inf')
        self.right_min = min(right) if right else float('inf')

    # -------------------------
    # FSM CONTROL LOOP
    # -------------------------
    def control_loop(self):
        cmd = Twist()
        now = time.time()

        # -------- FORWARD --------
        if self.state == "FORWARD":
            if self.front_min < self.safe_dist:
                self.state = "STOP"
                self.state_start_time = now
            else:
                cmd.linear.x = self.forward_speed

        # -------- STOP --------
        elif self.state == "STOP":
            if now - self.state_start_time > self.stop_time:
                self.state = "REVERSE"
                self.state_start_time = now

        # -------- REVERSE --------
        elif self.state == "REVERSE":
            cmd.linear.x = self.reverse_speed
            if now - self.state_start_time > self.reverse_time:
                self.state = "SCAN"
                self.state_start_time = now

        # -------- SCAN --------
        elif self.state == "SCAN":
            if self.left_min > self.right_min:
                self.state = "TURN_LEFT"
            else:
                self.state = "TURN_RIGHT"
            self.state_start_time = now

        # -------- TURN LEFT --------
        elif self.state == "TURN_LEFT":
            cmd.angular.z = self.turn_speed
            if now - self.state_start_time > self.turn_time:
                self.state = "FORWARD"

        # -------- TURN RIGHT --------
        elif self.state == "TURN_RIGHT":
            cmd.angular.z = -self.turn_speed
            if now - self.state_start_time > self.turn_time:
                self.state = "FORWARD"

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time


class FSMObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('fsm_obstacle_avoidance')

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        # ---------------- PARAMETERS ----------------
        self.front_limit = 0.7
        self.side_limit = 1.0

        self.forward_speed = 0.18
        self.reverse_speed = -0.15
        self.turn_speed = 0.7

        self.reverse_time = 1.0
        self.turn_min_time = 0.8
        self.escape_time = 3.0

        # ---------------- FSM ----------------
        self.state = "FORWARD"
        self.state_start = time.time()
        self.turn_dir = None   # LEFT or RIGHT

        # ---------------- LIDAR ----------------
        self.front = float('inf')
        self.left = float('inf')
        self.right = float('inf')

        self.get_logger().info("FSM Obstacle Avoidance (Beginner Safe) Started")

    # ---------------- LIDAR ----------------
    def scan_callback(self, msg):
        ranges = msg.ranges
        n = len(ranges)

        def clean(data):
            return [d for d in data if not math.isnan(d) and not math.isinf(d)]

        front = clean(ranges[int(0.47*n):int(0.53*n)])
        left  = clean(ranges[int(0.65*n):int(0.85*n)])
        right = clean(ranges[int(0.15*n):int(0.35*n)])

        self.front = min(front) if front else float('inf')
        self.left  = sum(left)/len(left) if left else float('inf')
        self.right = sum(right)/len(right) if right else float('inf')

    # ---------------- FSM ----------------
    def control_loop(self):
        cmd = Twist()
        now = time.time()

        # -------- FORWARD --------
        if self.state == "FORWARD":
            if self.front < self.front_limit:
                self.state = "REVERSE"
                self.state_start = now
            else:
                cmd.linear.x = self.forward_speed

        # -------- REVERSE --------
        elif self.state == "REVERSE":
            cmd.linear.x = self.reverse_speed

            if now - self.state_start > self.reverse_time:
                # Choose turn direction ONCE
                self.turn_dir = "LEFT" if self.left > self.right else "RIGHT"
                self.state = "TURN"
                self.state_start = now

        # -------- TURN --------
        elif self.state == "TURN":

            if self.turn_dir == "LEFT":
                cmd.angular.z = self.turn_speed
                side_clear = self.left
            else:
                cmd.angular.z = -self.turn_speed
                side_clear = self.right

            # ✔ Ignore front completely
            if side_clear > self.side_limit and \
               now - self.state_start > self.turn_min_time:
                self.state = "FORWARD"

            # ESCAPE if turning too long
            elif now - self.state_start > self.escape_time:
                self.state = "ESCAPE"
                self.state_start = now

        # -------- ESCAPE --------
        elif self.state == "ESCAPE":
            cmd.linear.x = self.reverse_speed
            cmd.angular.z = self.turn_speed

            if now - self.state_start > 1.5:
                self.state = "FORWARD"

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = FSMObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

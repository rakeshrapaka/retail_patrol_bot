#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class PatrolManager(Node):
    def __init__(self):
        super().__init__('patrol_manager')

        # Publishers / Subscribers
        self.pub_state = self.create_publisher(String, '/patrol_state', 10)
        self.sub_person = self.create_subscription(Int32, '/person_count', self.person_cb, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State variables
        self.state = 'idle'
        self.persons = 0
        self.min_scan = None
        self.obstacle_detected = False
        self.safe_distance = 0.4  # meters

        # Timers
        self.timer = self.create_timer(0.5, self.timer_cb)

        self.get_logger().info('✅ Patrol manager with obstacle/person awareness started')

    def person_cb(self, msg: Int32):
        self.persons = msg.data
        self.get_logger().info(f"[PersonCB] Updated person_count = {self.persons}")

    def scan_cb(self, msg: LaserScan):
        if not msg.ranges:
            return
        valid = [r for r in msg.ranges if not math.isinf(r) and r > 0.0]
        if valid:
            self.min_scan = min(valid)
            self.obstacle_detected = self.min_scan < self.safe_distance
        else:
            self.min_scan = None
            self.obstacle_detected = False

        self.get_logger().info(
            f"[ScanCB] min_scan = {self.min_scan:.2f} m" if self.min_scan else "[ScanCB] No valid scan data"
        )

    def timer_cb(self):
        tw = Twist()

        if self.persons > 0:
            # Stop if person detected
            self.state = 'alert'
            tw.linear.x = 0.0
            tw.angular.z = 0.0

        elif self.obstacle_detected:
            # Rotate if obstacle ahead
            self.state = 'avoid'
            tw.linear.x = 0.0
            tw.angular.z = 0.4

        else:
            # Move forward
            self.state = 'patrolling'
            tw.linear.x = 0.1
            tw.angular.z = 0.0

        # Publish state
        m = String()
        m.data = self.state
        self.pub_state.publish(m)
        self.cmd_pub.publish(tw)

        # Debug log
        self.get_logger().info(
            f"[Decision] state={self.state}, person_count={self.persons}, "
            f"min_scan={self.min_scan}, cmd_vel=({tw.linear.x:.2f}, {tw.angular.z:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PatrolManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

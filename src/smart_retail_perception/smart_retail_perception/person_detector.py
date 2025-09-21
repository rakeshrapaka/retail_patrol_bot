#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import time


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')

        # ROS interfaces
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 5)
        self.pub = self.create_publisher(Int32, '/person_count', 10)

        # HOG person detector
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # Debug/watchdog
        self.last_frame_time = time.time()
        self.watchdog_timer = self.create_timer(2.0, self.watchdog_cb)  # check every 2s

        self.get_logger().info('✅ Person detector started, waiting for camera frames...')

    def image_cb(self, msg: Image):
        self.last_frame_time = time.time()  # update watchdog

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        boxes, _ = self.hog.detectMultiScale(gray, winStride=(8, 8), padding=(8, 8), scale=1.05)
        count = len(boxes)

        # Publish result
        m = Int32()
        m.data = count
        self.pub.publish(m)

        # Debug log
        self.get_logger().info(f'Frame received, detected {count} persons')

    def watchdog_cb(self):
        now = time.time()
        if now - self.last_frame_time > 5.0:
            self.get_logger().warn('⚠️ No camera frames received in the last 5 seconds!')
        else:
            self.get_logger().debug('Camera is active')


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

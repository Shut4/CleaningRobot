#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
import numpy as np


class TrashDetectorEvent(Node):
    def __init__(self):
        super().__init__("trash_detector_event")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("publish_topic", "/trash_detected")
        self.declare_parameter("min_area", 500)
        self.declare_parameter("cooldown_frames", 10)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.publish_topic = str(self.get_parameter("publish_topic").value)
        self.min_area = int(self.get_parameter("min_area").value)
        self.cooldown_frames = int(self.get_parameter("cooldown_frames").value)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Bool, self.publish_topic, 10)
        self.sub = self.create_subscription(Image, self.image_topic, self.cb, 10)

        self._cooldown = 0

        self.get_logger().info(
            f"trash_detector_event: image={self.image_topic}, pub={self.publish_topic}"
        )

    def cb(self, msg: Image):
        if self._cooldown > 0:
            self._cooldown -= 1
            return

        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.detect_red(bgr):
            self.pub.publish(Bool(data=True))
            self.get_logger().info("trash detected -> publish True")
            self._cooldown = self.cooldown_frames

    def detect_red(self, bgr) -> bool:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        red1 = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255))
        red2 = cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
        mask = cv2.bitwise_or(red1, red2)
        area = int(cv2.countNonZero(mask))
        return area > self.min_area


def main():
    rclpy.init()
    node = TrashDetectorEvent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

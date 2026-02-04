import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge

import cv2
import numpy as np


class TrashMarkerTest(Node):
    def __init__(self):
        super().__init__("trash_marker_test")

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, "/image_raw", self.cb_image, 10)
        self.marker_pub = self.create_publisher(Marker, "/visualization_marker", 10)

        self.marker_id = 0
        self.cooldown = 0  # 連続検知でマーカー増えすぎないように

    def cb_image(self, msg: Image):
        if self.cooldown > 0:
            self.cooldown -= 1
            return

        # ROS Image -> OpenCV (bgr8) 変換（講義資料のとおり）:contentReference[oaicite:4]{index=4}
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        found = self.detect_red_or_blue(cv_image)
        if not found:
            return

        self.publish_marker_base_link()
        self.get_logger().info("trash detected -> marker published")
        self.cooldown = 10  # 約10フレーム休む

    def detect_red(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # 赤はHSVで2レンジ必要
        red1 = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255))
        red2 = cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))

        mask = cv2.bitwise_or(red1, red2)
        area = int(cv2.countNonZero(mask))

        # 閾値はとりあえず小さめ。必要なら調整
        return area > 500

    def publish_marker_base_link(self):
        m = Marker()
        m.header.frame_id = "base_link"  # ★まずここで動作確認
        m.header.stamp = self.get_clock().now().to_msg()

        m.ns = "trash"
        m.id = self.marker_id
        self.marker_id += 1

        m.type = Marker.SPHERE
        m.action = Marker.ADD

        # base_link前方に出す（「出た」ことの確認が目的）
        m.pose.position.x = 0.17
        m.pose.position.y = 0.0
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0

        m.scale.x = 0.15
        m.scale.y = 0.15
        m.scale.z = 0.15

        m.color.a = 1.0
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2

        m.lifetime.sec = 0  # 消さない
        self.marker_pub.publish(m)


def main():
    rclpy.init()
    node = TrashMarkerTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

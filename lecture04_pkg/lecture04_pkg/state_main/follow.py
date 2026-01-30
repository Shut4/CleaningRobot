#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: navigation.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibikino-Musashi@Home）
"""

# モジュールのインポート(外部)
import numpy as np
import cv2

# モジュールのインポート(ROS2関連)
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import State
from yasmin import Blackboard


class FollowState(State):
    """FollowStateクラス（Stateクラスの継承）
    赤色領域を取得して追従する
    """

    def __init__(self, node: Node):
        """クラスの初期化メソッド"""
        # 継承したStateクラスのコンストラクタをオーバーライド
        # 引数のoutcomesには，ステートが完了したときに返す可能性のある結果を文字列で指定
        super().__init__(outcomes=["outcome"])
        self.node = node

        self.bridge = CvBridge()
        self.image_pub = self.node.create_publisher(
            msg_type=Image, topic="masked_image", qos_profile=10
        )
        self.image_sub = self.node.create_subscription(
            msg_type=Image, topic="image_raw", callback=self.callback, qos_profile=10
        )

        self.vel_pub = self.node.create_publisher(msg_type=Twist, topic="cmd_vel", qos_profile=10)

        self.cmd_vel = Twist()
        self.detect_log = "stop"

    def callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.node.get_logger().info(e)

        # ========================= state =====================================
        # 赤色のマスク処理
        # =====================================================================
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, w, ch = hsv.shape
        hsv1 = hsv

        # 赤色の範囲1
        hsv_min = np.array([0, 150, 150])
        hsv_max = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

        # 赤色の範囲2
        hsv_min = np.array([160, 150, 150])
        hsv_max = np.array([179, 255, 255])
        mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

        # 赤色領域のマスク
        mask = mask1 + mask2
        masked_hsv = cv2.bitwise_and(hsv1, hsv1, mask=mask)

        # 赤色領域の計算
        ones = np.ones((h, w))
        masked = cv2.bitwise_and(ones, ones, mask=mask)

        # 左、中、右の赤色領域を計算
        ones_left = sum(sum(masked[0:h, 0 : int(w / 3)]))
        ones_center = sum(sum(masked[0:h, int(w / 3) : int(2 * w / 3)]))
        ones_right = sum(sum(masked[0:h, int(2 * w / 3) : w]))

        # cmd_vel の設定
        cmd_vel = Twist()
        if (ones_left > ones_center) and (ones_left > ones_right):
            detect_log = "Left side"
            cmd_vel.linear.x = 0.00
            cmd_vel.angular.z = 0.20
        elif (ones_center > ones_left) and (ones_center > ones_right):
            detect_log = "Center"
            cmd_vel.linear.x = 0.05
            cmd_vel.angular.z = 0.00
        elif (ones_right > ones_left) and (ones_right > ones_center):
            detect_log = "Right side"
            cmd_vel.linear.x = 0.00
            cmd_vel.angular.z = -0.20
        else:
            detect_log = "stop"
            cmd_vel.linear.x = 0.00
            cmd_vel.angular.z = 0.00

        self.detect_log = detect_log
        self.cmd_vel = cmd_vel

        # 結果のパブリッシュ
        try:
            img_cv = cv2.cvtColor(masked_hsv, cv2.COLOR_HSV2BGR)
            img_msg = self.bridge.cv2_to_imgmsg(img_cv, "bgr8")
            self.image_pub.publish(img_msg)
        except CvBridgeError as e:
            self.node.get_logger().info(e)

    def execute(self, blackboard: Blackboard) -> str:
        """
        Followステートの実行メソッド

        Args:
            blackboard (CustomBlackboard): CustomBlackboardオブジェクト

        Returns:
            str: outcomesの文字列
        """
        self.node.get_logger().info("Follow")
        self.node.get_logger().info("Start!!")
        cnt = 0
        CNT_MAX = 50
        while rclpy.ok():  #正しくノードが動作している間ループを実行
            self.node.get_logger().info(self.detect_log)
            self.vel_pub.publish(self.cmd_vel)
            self.node.get_clock().sleep_for(Duration(seconds=1))

            cnt += 1
            rclpy.spin_once(self.node)
            if cnt > CNT_MAX:
                break
        # ロボットの停止コマンドを送信
        self.vel_pub.publish(Twist())
        self.node.get_logger().info("Stop!!")
        self.node.get_clock().sleep_for(Duration(seconds=1))
        return "outcome"

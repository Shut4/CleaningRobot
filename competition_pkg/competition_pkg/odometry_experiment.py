#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class OdomController(Node):

    def __init__(self):
        super().__init__('turtlebot_odom_controller')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # ロボットの現在位置・姿勢
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # 制御ステート
        self.state = 0  # 0:前進1, 1:回転, 2:前進2, 3:停止
        self.start_x = None
        self.start_y = None
        self.start_yaw = 0

        # タイマー: 10Hz で制御ループ実行
        self.timer = self.create_timer(0.1, self.control_loop)

    # =============================
    # オドメトリコールバック
    # =============================
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # 姿勢をクォータニオンからヨー角へ変換
        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, self.yaw = euler_from_quaternion(quaternion)

    # =============================
    # メイン制御ループ
    # =============================
    def control_loop(self):
        msg = Twist()

        # --- 状態0：最初の前進0.3m ---
        if self.state == 0:
            if self.start_x is None:
                self.start_x = self.x
                self.start_y = self.y

            target_yaw = self.start_yaw + math.pi/2  # 右方向 (-)
            # 角度差を [-pi, pi] に正規化
            angle_error = math.atan2(math.sin(target_yaw - self.yaw),math.cos(target_yaw - self.yaw))

            if abs(angle_error) > 0.05:
                msg.angular.z = -0.5  # 右回転（マイナス）
            else:
                self.state = 1
                self.start_x = self.x
                self.start_y = self.y

        # --- 状態1：90°右回転 ---
        elif self.state == 1:
            target_yaw = self.start_yaw - math.pi/2  # 右方向 (-)
            # 角度差を [-pi, pi] に正規化
            angle_error = math.atan2(math.sin(target_yaw - self.yaw),math.cos(target_yaw - self.yaw))

            if abs(angle_error) > 0.05:
                msg.angular.z = -0.5  # 右回転（マイナス）
            else:
                self.state = 2
                self.start_x = self.x
                self.start_y = self.y

        # --- 状態2：もう一度前進0.3m ---
        elif self.state == 2:
            distance = math.sqrt((self.x - self.start_x)**2 + (self.y - self.start_y)**2)

            if distance < 0.3:
                msg.linear.x = 0.15
            else:
                self.state = 3

        # --- 状態3：停止 ---
        elif self.state == 3:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            self.get_logger().info("Finished movement.")
            return

        # 速度の送信
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
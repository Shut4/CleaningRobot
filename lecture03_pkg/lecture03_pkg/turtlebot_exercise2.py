#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: turtlebot_exercise2.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibikino-Musashi@Home）
"""

# モジュールのインポート(標準)
import math

# モジュールのインポート(ROS2関連)
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist


class TurtlebotController(Node):
    """TurtlebotControllerクラス（Nodeクラスを継承）
    TurtleBot3移動制御メッセージトピックをパブリッシュするROS2ノードクラス

    Attributes:
        publisher (Publisher): メッセージをパブリッシュするための ROS2 パブリッシャ
        timer (Timer): 指定された周期でコールバックメソッドを呼び出すためのタイマ
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # ノード名の宣言
        super().__init__("turtlebot_exercise2")

        # パブリッシャの宣言
        self.publisher = self.create_publisher(msg_type=Twist, topic="cmd_vel", qos_profile=10)

        # メッセージの定義
        msg = Twist()

        # ======================================== #
        # TODO
        # ======================================== #
        # 30[cm]（0.3[m]）前進するプログラム
        msg.linear.x = 0.15 # [m/s]   # 並進速度
        msg.angular.z = 0.0  # [rad/s] # 旋回速度
        # ログを表示
        self.get_logger().info(str(msg))
        # メッセージをパブリッシュ
        self.publisher.publish(msg)
        self.get_clock().sleep_for(Duration(seconds=2))
        # 90度（1/2π）右旋回するプログラム
        msg.linear.x = 0.0 # [m/s]   # 並進速度
        msg.angular.z = -1.0  # [rad/s] # 旋回速度
        # ログを表示
        self.get_logger().info(str(msg))
        # メッセージをパブリッシュ
        self.publisher.publish(msg)
        self.get_clock().sleep_for(Duration(seconds=1.57))

        # 30[cm]（0.3[m]）前進するプログラム
        msg.linear.x = 0.15 # [m/s]
        msg.angular.z = 0.0  # [rad/s] # 旋回速度
        self.get_logger().info(str(msg))
        self.publisher.publish(msg)
        self.get_clock().sleep_for(Duration(seconds=2))
        # 停止するプログラム
        msg.linear.x = 0.0 # [m/s]   # 並進速度
        msg.angular.z = 0.0  # [rad/s] # 旋回速度
        self.get_logger().info(str(msg))
        self.publisher.publish(msg)


def main(args=None):
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # TurtlebotControllerクラスのインスタント生成
    node = TurtlebotController()

    # ノードが終了するまで指定したノードを実行
    rclpy.spin(node)

    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

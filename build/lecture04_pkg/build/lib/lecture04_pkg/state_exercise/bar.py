#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: bar.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibikino-Musashi@Home）
"""

# モジュールのインポート(ROS2関連)
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import State
from yasmin import Blackboard


class BarState(State):
    """BarStateクラス（Stateクラスの継承）"""

    def __init__(self, node: Node):
        """クラスの初期化メソッド

        Args:
            node (Node): Nodeクラスのオブジェクト
        """
        # 継承したStateクラスのコンストラクタをオーバーライド
        # 引数のoutcomesには，ステートが完了したときに返す可能性のある結果を文字列で指定
        super().__init__(outcomes=["outcome2"])

        # Nodeオブジェクトのインスタンスを生成
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        """
        BARステートの実行メソッド

        Args:
            blackboard (Blackboard): Blackboardクラスのオブジェクト

        Returns:
            str: outcomesの文字列
        """
        # ログを表示
        self.node.get_logger().info("Executing state BAR")

        # 1秒待機
        self.node.get_clock().sleep_for(Duration(seconds=1))

        # ログを表示
        self.node.get_logger().info(blackboard.foo_str)

        return "outcome2"

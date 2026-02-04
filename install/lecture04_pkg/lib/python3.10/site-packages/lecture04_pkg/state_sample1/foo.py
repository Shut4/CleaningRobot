#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: foo.py
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


class FooState(State):
    """FooStateクラス（Stateクラスの継承）"""

    def __init__(self, node: Node):
        """クラスの初期化メソッド

        Args:
            node (Node): Nodeクラスのオブジェクト
        """
        # 継承したStateクラスのコンストラクタをオーバーライド
        # 引数のoutcomesには，ステートが完了したときに返す可能性のある結果を文字列で指定
        super().__init__(outcomes=["outcome1", "outcome2"])

        # Nodeオブジェクトのインスタンスを生成
        self.node = node

        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        """
        FOOステートの実行メソッド

        Args:
            blackboard (CustomBlackboard): CustomBlackboardオブジェクト

        Returns:
            str: outcomesの文字列
        """
        # ログを表示
        self.node.get_logger().info("Executing state FOO")

        # 3秒待機
        self.node.get_clock().sleep_for(Duration(seconds=3))

        if self.counter < 3:
            self.counter += 1
            blackboard.foo_str = f"Counter: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"
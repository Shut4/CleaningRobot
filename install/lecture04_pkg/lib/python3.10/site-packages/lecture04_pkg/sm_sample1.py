#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibikino-Musashi@Home）
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_sample1 import foo, bar


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(引数は，'ノード名')
        super().__init__("sm_sample1")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="FOO",  # ステート名
            state=foo.FooState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞書
                "outcome1": "BAR",  # "outcome1"が返ってきたらBARステートに遷移
                "outcome2": "EXIT",  # "outcome2"が返ってきたらEXITに遷移
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="BAR",  # ステート名
            state=bar.BarState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞書
                "outcome2": "FOO"  # "outcome2"が返ってきたらFOOステートに遷移
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_SAMPLE1", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with outcome: " + outcome)


def main(args=None):
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

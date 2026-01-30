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
from yasmin import StateMachine #ステートマシンを定義するのに用いるクラスをインポート
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_main import mr1, mr2, search


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(引数は，'ノード名')
        super().__init__("sm_main")

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT","FAILED"])

        # ステートマシンにFOOステートを追加
        sm.add_state(
            name="MR1",  # ステート名
            state=mr1.MR1State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞書
                "succeed": "SEARCH",  # "outcome1"が返ってきたらBARステートに遷移
                "failed": "FAILED"
            },
        )

        # ステートマシンにBARステートを追加
        sm.add_state(
            name="MR2",  # ステート名
            state=mr2.MR2State(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞書
                "succeed": "SEARCH",  # "outcome2"が返ってきたらFOOステートに遷移
                "failed": "FAILED"
            },
        )

        sm.add_state(
            name="SEARCH",  # ステート名
            state=search.SearchState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞書
                "found": "EXIT",  # "outcome2"が返ってきたらFOOステートに遷移
                "notfound1": "MR2",
                "notfound2": "EXIT"
            },
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_MAIN", fsm=sm)

        # ステートマシンを実行
        outcome = sm()

        # ログを表示
        self.get_logger().info("State Machine finished with outcome: " + outcome)


def main(args=None):
    """Main関数"""
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    node = StateMachineNode()

    rclpy.spin(node)
    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
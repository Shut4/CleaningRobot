#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_exercise.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibikino-Musashi@Home）
"""

# モジュールのインポート（ROS2関連）
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_exercise import foo, bar, hoge


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(引数は，'ノード名')
        super().__init__("sm_exercise")

        # 有限状態機械（FSM）を作成
        sm = StateMachine(outcomes=["EXIT"])

        # FSMにFOOステートを追加
        sm.add_state(
            name="FOO",  # ステート名
            state=foo.FooState(self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞書
                "outcome1": "BAR",  # "outcome1"が返ってきたらBARステートに遷移
                "outcome2": "EXIT",  # "outcome2"が返ってきたらEXITに遷移
            },
        )

        # TODO: ステートマシンにBARステートを追加
        sm.add_state(
            name="BAR",
            state=bar.BarState(self),
            transitions={"outcome2": "HOGE"},
        )

        # TODO: ステートマシンにHOGEステートを追加
        sm.add_state(
            name="HOGE",  # ステート名
            state=hoge.HogeState(node=self),  # 実行するステート
            transitions={  # 次に遷移するステートを定義する辞書
                "outcome3": "FOO",  # "outcome1"が返ってきたらBARステートに遷移
                "outcome4": "BAR",  # "outcome2"が返ってきたらEXITに遷移
            },
        )

        # Yasmin ViewerにFSM情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_EXERCISE", fsm=sm)

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

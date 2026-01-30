#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: sm_main.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibikino-Musashi@Home）

"""

# モジュールのインポート(ROS2関連)
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist


# モジュールのインポート（YASMIN関連）
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub

# モジュールのインポート（自作:各ステート）
from .state_main import follow, navigation


class StateMachineNode(Node):
    """StateMachineNodeクラス（Nodeクラスを継承）
    ステートマシンを実行するノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        super().__init__("sm_main")

        self.get_logger().info("\033[43m\033[30m\033[1m<< PLEASE ENTER TO START >>\033[0m")
        input()
        self.get_logger().info("Task Start!!")

        self.vel_pub = self.create_publisher(msg_type=Twist, topic="cmd_vel", qos_profile=10)

        # StateMachineクラスのインスタンを生成
        sm = StateMachine(outcomes=["EXIT"])

        # ステートマシンにFollowStateを追加
        sm.add_state(
            name="Follow",
            state=follow.FollowState(node=self),
            transitions={"outcome": "Navigation"},
        )

        # ステートマシンにNavigationStateを追加
        sm.add_state(
            name="Navigation",
            state=navigation.NavigationState(node=self),
            transitions={"succeed": "EXIT", "failed": "Follow"},
        )

        # Yasmin Viewerにステートマシンの情報をパブリッシュ
        YasminViewerPub(fsm_name="SM_MAIN", fsm=sm)

        # ステートマシンを実行
        outcome = sm()
        self.get_logger().info("State Machine finished with outcome: " + outcome)


def shutdown(node: Node):
    """シャットダウン関数
    終了時にTurtleBot3を停止させる

    Args:
        node (Node): Nodeオブジェクト
    """
    node.get_logger().info("Follow State Cleanup!!")
    pub = node.create_publisher(Twist, "cmd_vel", 10)
    pub.publish(Twist())
    node.get_clock().sleep_for(Duration(nanoseconds=100))
    node.destroy_publisher(pub)


def main(args=None):
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # StateMachineNodeクラスのインスタント生成
    try:
        node = StateMachineNode()
    except KeyboardInterrupt:
        pass
    finally:
        shutdown(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

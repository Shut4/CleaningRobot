#!/usr/bin/env python3
# -*- encoding: UTF-8 -*-

"""
File: navigation_sample2.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibikino-Musashi@Home）
"""

# モジュールのインポート(ROS2関連)
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.action._follow_waypoints import (
    FollowWaypoints_GetResult_Response,
    FollowWaypoints_Feedback,
    FollowWaypoints_FeedbackMessage,
)
from action_msgs.msg import GoalStatus
import tf_transformations


class WaypointsFollower(Node):
    """WaypointsFollowerクラス（Nodeクラスを継承）
    複数指定した目的地にTurtleBot3を移動させるROS2ノードクラス
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(引数は，'ノード名')
        super().__init__("navigation_sample2")

        # インスタンス変数の初期化
        self._goal_handle: ClientGoalHandle = None
        self._result_future: FollowWaypoints_GetResult_Response = None
        self._feedback: FollowWaypoints_Feedback = None
        self._status: int = None

        # ActionClientのインスタンスを生成（接続先のアクションサーバー名を指定）
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, "follow_waypoints")

        # アクションサーバーが起動するまで待機
        self.follow_waypoints_client.wait_for_server()

    def setWaypoints(self, waypoints: list):
        """複数目的地を設定するメソッド

        Args:
            waypoints (list): 目的地のリスト
        """
        # 目的地のリストを初期化
        self.waypoints = []

        # 指定した目的地の数だけ繰り返す
        for wp in waypoints:
            # PoseStampedメッセージを作成し，各目的地の座標と姿勢を設定
            pose = PoseStamped()

            ## 目的地の座標を設定
            pose.header.frame_id = "map"
            pose.pose.position.x = wp[0]  # 目的地のX座標[m]
            pose.pose.position.y = wp[1]  # 目的地のY座標[m]
            pose.pose.position.z = 0.0

            ## 目的地での姿勢を設定
            ### オイラー角からクォータニオンへの変換
            quat = tf_transformations.quaternion_from_euler(0, 0, wp[2])
            pose.pose.orientation.x = quat[1]
            pose.pose.orientation.y = quat[2]
            pose.pose.orientation.z = quat[3]
            pose.pose.orientation.w = quat[0]

            ## 目的地をリストに追加
            self.waypoints.append(pose)

    def followWaypoints(self) -> bool:
        """目的地のリストの先頭から順に各目的地に移動させるメソッド

        Returns:
            bool: 'FollowWaypointsサーバーからの返答．True（承認），False（拒否）
        """
        self.get_logger().debug("Waiting for 'FollowWaypoints' action server")

        # FollowWaypointsサーバーと接続できるまで待機
        while not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'FollowWaypoints' action server not available, waiting...")

        # FollowWaypointsアクションのGoalメッセージを作成
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints

        # 目的地のリストをアクションサーバーに非同期で送信し，フィードバックメッセージ受信時に呼ぶコールバックメソッドを登録
        send_goal_future = self.follow_waypoints_client.send_goal_async(
            goal_msg, feedback_callback=self._feedbackcallback
        )
        # Goalメッセージの送信が完了するまで待機
        rclpy.spin_until_future_complete(self, send_goal_future)

        # Goalメッセージの送信結果を取得
        self._goal_handle = send_goal_future.result()

        # FollowWaypointsサーバーの承認結果を確認
        if not self._goal_handle.accepted:
            self.get_logger().error(
                f"Following {len(self.waypoints)} waypoints request was rejected!"
            )
            return False

        self._result_future = self._goal_handle.get_result_async()
        return True

    def cancelNav(self):
        """実行中のナビゲーションをキャンセルするメソッド"""
        self.get_logger().info("Canceling current task.")
        if self._result_future:
            future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)

    def isNavComplete(self) -> bool:
        """ナビゲーションの完了状態を確認するメソッド

        Returns:
            bool: ステータスを返す．True（キャンセル，完了），False（タイムアウト，処理中，未完了）
        """
        if not self._result_future:
            return True
        rclpy.spin_until_future_complete(self, self._result_future, timeout_sec=0.10)
        if self._result_future.result():
            self._status = self._result_future.result().status
            if self._status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().debug(f"Task with failed with status code: {self._status}")
                return True
        else:
            return False
        self.get_logger().debug(f"Navigation succeeded!")
        return True

    def getResult(self) -> int:
        """保留中のアクションの結果メッセージを取得するメソッド"""
        return self._status

    def getFeedback(self) -> FollowWaypoints_Feedback:
        """保留中のアクションのフィードバックメッセージを取得するメソッド

        Returns:
            FollowWaypoints_Feedback: FollowWaypoints_Feedbackオブジェクトを返す
        """
        return self._feedback

    def _feedbackcallback(self, msg: FollowWaypoints_FeedbackMessage):
        """サーバーからのフィードバックを受信したときに呼ばれるコールバックメソッド

        Args:
            feedback_msg (FollowWaypoints_FeedbackMessage): nav2_msgs/action/FollowWaypoints型のFeedbackメッセージ
        """
        # フィードバックを取得
        self.get_logger().debug("Received action feedback message")
        self._feedback = msg.feedback


def main(args=None):
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # TODO 各目的地の座標と目的地での姿勢をリストで定義
    waypoints = [
        (111, 121, 111),  # 目的地1（X [m], Y [m], Yaw角 [rad]）
        (123, 123, 234),  # 目的地2（X [m], Y [m], yaw角 [rad]）
        (321, 123, 123),  # 目的地3（X [m], Y [m], yaw角 [rad]）
    ]

    # WaypointsFollowerクラスのインスタンス生成
    nav = WaypointsFollower()

    # 目的地のリストを設定
    nav.setWaypoints(waypoints)

    # 目的地のリストの先頭から順に移動
    nav.followWaypoints()

    # ナビゲーションは完了するまで待機
    while not nav.isNavComplete():
        # フィールドバックメッセージを取得
        feedback = nav.getFeedback()
        # 現在のウェイポイントを表示
        nav.get_logger().info(f"Current waypoint: {feedback.current_waypoint}")

    # ナビゲーション結果を取得し，表示
    result = nav.getResult()
    match result:
        case GoalStatus.STATUS_SUCCEEDED:
            nav.get_logger().info("Nvigation succeeded!")
        case GoalStatus.STATUS_CANCELED:
            nav.get_logger().info("Nvigation was canceled!")
        case GoalStatus.STATUS_ABORTED:
            nav.get_logger().error("Nvigation failed!")
        case _:
            nav.get_logger().error("Unknown error!")

    # 終了処理
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

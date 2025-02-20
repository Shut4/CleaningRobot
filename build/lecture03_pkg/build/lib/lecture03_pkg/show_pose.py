#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: show_pose.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibikino-Musashi@Home）
"""

# モジュールのインポート(ROS2関連)
import rclpy
from rclpy.node import Node
import rclpy.time
import tf2_ros  # TF2のモジュール
from tf2_ros import TransformListener
from tf2_ros import TransformException  # TF2の例外処理
import tf_transformations


class ShowPose(Node):
    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(引数は，'ノード名')
        super().__init__("show_pose")

        # tf2_ros.Bufferクラスのインスタンスを生成
        self.tf_buffer = tf2_ros.Buffer()

        # TransformListenerクラスのインスタンスのインスタンスを生成
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 割り込み用タイマの定義（指定した周期[s]でコールバックメソッドを呼ぶ）
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """割り込み用タイマのコールバックメソッド"""
        try:
            # ソースフレームから見たターゲットフレームの相対座標を取得
            # フレームはターゲット→ソースの順に記述する
            # 引数timeはtfを取得する時間．rclpy.time.Time()を設定すると最新のtfを取得する.
            trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except TransformException:
            return

        # 座標データの取得
        translation = trans.transform.translation

        # 回転データの取得
        rotation = trans.transform.rotation

        # 回転データをtransforms3dのクォータニオンの形式（リスト）に変換
        quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]

        # クォータニオンをオイラー角に変換
        euler = tf_transformations.euler_from_quaternion(quaternion)

        # ログを表示
        self.get_logger().info(f"x: {translation.x}, y: {translation.y}, yaw: {euler[2]}")


def main(args=None):
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # ShowPoseクラスのインスタント生成
    node = ShowPose()

    # ノードが終了するまで指定したノードを実行
    rclpy.spin(node)

    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

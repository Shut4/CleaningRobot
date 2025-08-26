#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: launch_sample1.launch.py
Author: Tomoaki Fujino
"""

# モジュールのインポート(ROS2関連)
from launch import LaunchDescription
from launch_ros.actions import Node


# launchモジュールをインポート
def generate_launch_description():
    """launchの記述情報を生成するメソッド

    Returns:
        launch.LaunchDescription: launchの記述情報を含むオブジェクト
    """
    return LaunchDescription(
        [
            # lecture01_pkgのtalkerノードの設定
            Node(
                package="lecture01_pkg",  # ノードが属するパッケージ
                executable="talker",  # 実行するノードの実行ファイル
                name="talker",  # ノードの名前
                output="screen",  # 出力先
            ),
            # lecture01_pkgのlistenerノードの設定
            Node(
                package="lecture01_pkg",  # ノードが属するパッケージ
                executable="listener",  # 実行するノードの実行ファイル
                name="listener",  # ノードの名前
                output="screen",  # 出力先
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()

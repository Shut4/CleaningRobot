#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: launch_sample2.launch.py
Author: Tomoaki Fujino
"""

# モジュールのインポート(標準)
import os

# モジュールのインポート(ROS2関連)
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


# launchモジュールをインポート
def generate_launch_description():
    """複数のノードを起動するためのlaunchの記述情報を生成するメソッド

    Returns:
        launch.LaunchDescription: launchの記述情報を含むオブジェクト
    """

    # LaunchConfigの設定(map)
    map_file = LaunchConfiguration(
        variable_name="map",  # 変数名
        # デフォルト値（map.yamlの絶対パス）
        default=os.path.join(
            os.getenv("HOME"),
            "ros2_lecture_ws",
            "map.yaml"
        )
    )

    # LaunchConfigの設定(use_sim_time)
    use_sim_time = LaunchConfiguration(
        variable_name="use_sim_time",  # 変数名
        default="false"  # デフォルト値
    )

    return LaunchDescription(
        [
            # 引数の定義(map)
            DeclareLaunchArgument(
                name="map",  # キー名
                default_value=os.path.join(
                    os.getenv("HOME"),
                    "ros2_lecture_ws",
                    "map.yaml"
                ),
                description="Full path to map file to load",  # 説明
            ),
            # 引数の定義(use_sim_time)
            DeclareLaunchArgument(
                name="use_sim_time",  # キー名
                default_value="false",  # デフォルト値
                description="Use simulation (Gazebo) clock if true",  # 説明
            ),
            # navigation2.launch.pyの設定
            IncludeLaunchDescription(
                launch_description_source = PythonLaunchDescriptionSource(
                    # 実行するlaunchファイルのパス
                    os.path.join(
                        get_package_share_directory("turtlebot3_navigation2"),
                        "launch",
                        "navigation2.launch.py"
                    )
                ),
                launch_arguments={
                    "map": map_file,  # 地図ファイルのパス
                    "use_sim_time": use_sim_time,  # シミュレータ時間の使用有無
                }.items(),
            ),
            # yasmin_viewer_nodeノードの設定
            Node(
                package="yasmin_viewer",  # ノードが属するパッケージ
                executable="yasmin_viewer_node",  # 実行するノードの実行ファイル
                name="yasmin_viewer_node",  # ノードの名前
                parameters=[{"use_sim_time": use_sim_time}],  # パラメータの設定
                output="screen",  # 出力先
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros

from yasmin import StateMachine

from lecture04_pkg.state_comp.init_home_state import InitHomeState
from lecture04_pkg.state_comp.follow_waypoints_state import FollowWaypointsState
from lecture04_pkg.state_comp.return_home_state import ReturnHomeState


def main():
    rclpy.init()
    node = Node("fsm_main")

    # TF
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    # --- StateMachine ---
    sm = StateMachine(outcomes=["finished", "failed"])

    # blackboard（共有データ）
    bb = sm.get_blackboard() if hasattr(sm, "get_blackboard") else sm.blackboard  # 保険
    # 巡回点（map座標[m], yaw[rad]）
    bb["waypoints"] = [
        (-0.644, 0.797, -2.067),  # 目的地1（X [m], Y [m], Yaw角 [rad]）
        (-1.366, 1.036, -2.062),  # 目的地2（X [m], Y [m], yaw角 [rad]）
        (-1.274, 0.335, -0.999),  # 目的地3（X [m], Y [m], yaw角 [rad]）
        (0.164, -0.559, -0.607),  # 目的地4（X [m], Y [m], Yaw角 [rad]）
        (0.693, -0.046, 2.692),  # 目的地5（X [m], Y [m], yaw角 [rad]）
        (-0.168, 0.336, -1.961),  # 目的地6（X [m], Y [m], yaw角 [rad]）
    ]

    sm.add(
        "INIT_HOME",
        InitHomeState(node, tf_buffer),
        transitions={"done": "FOLLOW"},
    )

    sm.add(
        "FOLLOW",
        FollowWaypointsState(
            node,
            tf_buffer,
            follow_action_name="follow_waypoints",
            trash_topic="/trash_detected",
            marker_topic="/visualization_marker",
        ),
        transitions={"done": "RETURN", "failed": "RETURN"},  # 巡回失敗でも帰る
    )

    sm.add(
        "RETURN",
        ReturnHomeState(node, nav_action_name="/navigate_to_pose"),
        transitions={"done": "finished", "failed": "failed"},
    )

    outcome = sm.execute()
    node.get_logger().info(f"FSM finished with outcome: {outcome}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

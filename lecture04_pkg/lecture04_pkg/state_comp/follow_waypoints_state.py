import math
import time
from typing import List, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.time import Time

import tf2_ros
import tf_transformations

from yasmin import State
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from nav2_msgs.action import FollowWaypoints


def quat_from_yaw(yaw: float):
    # returns (x, y, z, w)
    return tf_transformations.quaternion_from_euler(0.0, 0.0, float(yaw))


class FollowWaypointsState(State):
    def __init__(
        self,
        node,
        tf_buffer: tf2_ros.Buffer,
        follow_action_name: str = "follow_waypoints",
        trash_topic: str = "/trash_detected",
        marker_topic: str = "/visualization_marker",
        marker_ns: str = "trash_map",
        marker_scale: float = 0.15,
        goal_timeout_s: float = 300.0,
    ):
        super().__init__(outcomes=["done", "failed"])
        self.node = node
        self.tf_buffer = tf_buffer

        self.follow_client = ActionClient(node, FollowWaypoints, follow_action_name)

        self.marker_pub = node.create_publisher(Marker, marker_topic, 10)
        self.marker_ns = marker_ns
        self.marker_scale = float(marker_scale)
        self.marker_id = 0

        self.goal_timeout_s = float(goal_timeout_s)

        self._trash_flag = False
        node.create_subscription(Bool, trash_topic, self._cb_trash, 10)

    def _cb_trash(self, msg: Bool):
        if msg.data:
            self._trash_flag = True

    def _lookup_map_base(self):
        t = self.tf_buffer.lookup_transform("map", "base_link", Time())
        x = float(t.transform.translation.x)
        y = float(t.transform.translation.y)
        q = t.transform.rotation
        yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return (x, y, yaw)

    def _publish_marker_here(self):
        try:
            x, y, _ = self._lookup_map_base()
        except Exception:
            self.node.get_logger().warn("[FOLLOW] TF not ready, cannot publish marker.")
            return

        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = self.marker_ns
        m.id = self.marker_id
        self.marker_id += 1

        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0

        s = self.marker_scale
        m.scale.x = s
        m.scale.y = s
        m.scale.z = s

        m.color.a = 1.0
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2

        self.marker_pub.publish(m)
        self.node.get_logger().info(
            f"[FOLLOW] marker published at x={x:.2f}, y={y:.2f}"
        )

    def _build_poses(
        self, waypoints: List[Tuple[float, float, float]]
    ) -> List[PoseStamped]:
        poses = []
        now = self.node.get_clock().now().to_msg()
        for x, y, yaw in waypoints:
            p = PoseStamped()
            p.header.frame_id = "map"
            p.header.stamp = now
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            q = quat_from_yaw(yaw)
            # (x,y,z,w) の順で代入
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            poses.append(p)
        return poses

    def execute(self, blackboard):
        # ここに巡回点を入れる（map座標[m], yaw[rad]）
        waypoints: List[Tuple[float, float, float]] = getattr(
            blackboard, "waypoints", []
        )
        if not waypoints:
            self.node.get_logger().error("[FOLLOW] blackboard['waypoints'] is empty.")
            return "failed"

        self.node.get_logger().info("[FOLLOW] waiting FollowWaypoints server...")
        if not self.follow_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("[FOLLOW] FollowWaypoints server not ready.")
            return "failed"

        goal = FollowWaypoints.Goal()
        goal.poses = self._build_poses(waypoints)

        self.node.get_logger().info(f"[FOLLOW] send goal N={len(goal.poses)}")
        send_future = self.follow_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
        if not send_future.done():
            self.node.get_logger().error("[FOLLOW] send_goal_async timed out.")
            return "failed"

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("[FOLLOW] goal rejected.")
            return "failed"

        result_future = goal_handle.get_result_async()
        start = time.time()

        # 実行中ループ：検知フラグを見て marker を出す
        while rclpy.ok():
            # trash event
            if self._trash_flag:
                self._trash_flag = False
                self._publish_marker_here()

            # result check
            if result_future.done():
                res = result_future.result()
                status = res.status
                self.node.get_logger().info(f"[FOLLOW] finished status={status}")
                # SUCCEEDEDなら done、失敗でも次へ進みたいなら done扱いにしてもOK
                return "done" if status == 4 else "failed"

            # timeout
            if (time.time() - start) > self.goal_timeout_s:
                self.node.get_logger().warn("[FOLLOW] timeout -> cancel.")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(
                    self.node, cancel_future, timeout_sec=2.0
                )
                return "failed"

            rclpy.spin_once(self.node, timeout_sec=0.1)

        return "failed"

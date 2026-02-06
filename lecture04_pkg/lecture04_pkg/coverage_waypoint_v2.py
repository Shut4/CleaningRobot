#!/usr/bin/env python3
import math
import time
from typing import List, Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    HistoryPolicy,
)

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose

# RVizでwaypoint可視化したいならMarkerArrayを有効化
try:
    from visualization_msgs.msg import Marker, MarkerArray

    HAS_MARKERS = True
except Exception:
    HAS_MARKERS = False


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


class CoverageWaypointNodeV2(Node):
    def __init__(self):
        super().__init__("coverage_waypoint_node")

        # ---- Params (小マップ2.5m角を想定してデフォルトを控えめ) ----
        self.declare_parameter("pitch_m", 0.45)  # 行間（縦方向）
        self.declare_parameter("segment_mode", "center")  # "center" or "multi"
        self.declare_parameter("step_m", 0.80)  # multi用（横方向の間隔）
        self.declare_parameter("margin_m", 0.08)  # 障害物から離す距離（小さめ）
        self.declare_parameter("occ_th", 65)  # >= occ_th を障害物扱い（少し強め）
        self.declare_parameter("treat_unknown_as_obstacle", True)
        self.declare_parameter("goal_timeout_s", 60.0)
        self.declare_parameter("max_retries", 0)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("action_name", "navigate_to_pose")  # namespaceズレ対策

        self.pitch_m = float(self.get_parameter("pitch_m").value)
        self.segment_mode = str(self.get_parameter("segment_mode").value)
        self.step_m = float(self.get_parameter("step_m").value)
        self.margin_m = float(self.get_parameter("margin_m").value)
        self.occ_th = int(self.get_parameter("occ_th").value)
        self.treat_unknown = bool(self.get_parameter("treat_unknown_as_obstacle").value)
        self.goal_timeout_s = float(self.get_parameter("goal_timeout_s").value)
        self.max_retries = int(self.get_parameter("max_retries").value)
        self.publish_markers = bool(self.get_parameter("publish_markers").value)
        self.action_name = str(self.get_parameter("action_name").value)

        # ---- QoS: /map は Transient Local (latched) を取り逃さない ----
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.on_map, map_qos
        )
        self.nav_client = ActionClient(self, NavigateToPose, self.action_name)

        if HAS_MARKERS and self.publish_markers:
            self.marker_pub = self.create_publisher(
                MarkerArray, "/coverage_waypoints", 10
            )
        else:
            self.marker_pub = None

        self._map_msg: Optional[OccupancyGrid] = None
        self._started = False

        self.create_timer(0.5, self.kick)

        self.get_logger().info(
            "coverage_waypoint_node_v2 ready. Waiting for /map and Nav2 action..."
        )
        self.get_logger().info(
            f"Params: pitch_m={self.pitch_m}, margin_m={self.margin_m}, "
            f"occ_th={self.occ_th}, unknown_as_obs={self.treat_unknown}, "
            f"segment_mode={self.segment_mode}, step_m={self.step_m}, action_name={self.action_name}"
        )

    def on_map(self, msg: OccupancyGrid):
        # 1回受け取れば十分（地図更新追従したいなら変える）
        if self._map_msg is None:
            self._map_msg = msg
            w = msg.info.width
            h = msg.info.height
            res = msg.info.resolution
            self.get_logger().info(f"Received /map: {w}x{h}, res={res}")

            # map統計（原因調査用）
            a = np.array(msg.data, dtype=np.int16)
            self.get_logger().info(
                f"Map stats: min={int(a.min())}, max={int(a.max())}, "
                f"unknown={int((a < 0).sum())}, zero={int((a == 0).sum())}, "
                f"occ(>= {self.occ_th})={int((a >= self.occ_th).sum())}, total={a.size}"
            )

    def kick(self):
        if self._started:
            return
        if self._map_msg is None:
            return
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            return

        self._started = True
        self.get_logger().info("Nav2 action server is ready. Building waypoints...")
        waypoints = self.build_waypoints_from_map(self._map_msg)

        if len(waypoints) == 0:
            self.get_logger().error("No waypoints generated. Check map / parameters.")
            self.get_logger().error(
                "Try: -p margin_m:=0.05  -p treat_unknown_as_obstacle:=false  -p occ_th:=80"
            )
            return

        self.get_logger().info(
            f"Generated {len(waypoints)} waypoints. Start navigating..."
        )
        if self.marker_pub is not None:
            self.publish_waypoint_markers(waypoints)

        self.run_waypoints(waypoints)
        self.get_logger().info("Coverage navigation done.")

    # ---------------------------
    # Waypoint Generation
    # ---------------------------
    def build_waypoints_from_map(
        self, m: OccupancyGrid
    ) -> List[Tuple[float, float, float]]:
        w = m.info.width
        h = m.info.height
        res = float(m.info.resolution)
        ox = float(m.info.origin.position.x)
        oy = float(m.info.origin.position.y)

        data = np.array(m.data, dtype=np.int16).reshape((h, w))  # row=y, col=x

        # obstacle mask
        obs = data >= self.occ_th
        if self.treat_unknown:
            obs = np.logical_or(obs, data < 0)

        margin_cells = int(math.ceil(self.margin_m / res))
        pitch_cells = max(1, int(round(self.pitch_m / res)))
        step_cells = max(1, int(round(self.step_m / res)))

        # SAFE_FREE
        safe = np.ones((h, w), dtype=bool)
        safe[obs] = False

        # 近傍に障害物があるセルを落とす
        if margin_cells > 0:
            obs_ys, obs_xs = np.where(obs)
            for yy, xx in zip(obs_ys.tolist(), obs_xs.tolist()):
                y0 = max(0, yy - margin_cells)
                y1 = min(h - 1, yy + margin_cells)
                x0 = max(0, xx - margin_cells)
                x1 = min(w - 1, xx + margin_cells)
                safe[y0 : y1 + 1, x0 : x1 + 1] = False

        ys, xs = np.where(safe)
        if len(xs) == 0:
            return []

        x_min, x_max = int(xs.min()), int(xs.max())
        y_min, y_max = int(ys.min()), int(ys.max())

        # 小マップだと「短区間除外」が厳しすぎると全消しになるので、閾値を緩める
        #   - 最低3セル
        #   - ただしmarginが大きいときだけ少し増やす（上限を小さく）
        min_seg = max(3, min(2 * margin_cells + 1, 9))

        way_xy: List[Tuple[float, float]] = []

        line_idx = 0
        for yy in range(y_min, y_max + 1, pitch_cells):
            row = safe[yy, x_min : x_max + 1]
            segments = self.find_segments(row)

            line_points: List[Tuple[float, float]] = []

            for a, b in segments:
                seg_len = b - a + 1
                if seg_len < min_seg:
                    continue

                if self.segment_mode == "multi":
                    for ix_local in range(a, b + 1, step_cells):
                        ix = x_min + ix_local
                        wx = ox + (ix + 0.5) * res
                        wy = oy + (yy + 0.5) * res
                        line_points.append((wx, wy))
                else:
                    # center
                    cx = (a + b) // 2
                    ix = x_min + cx
                    wx = ox + (ix + 0.5) * res
                    wy = oy + (yy + 0.5) * res
                    line_points.append((wx, wy))

            if (line_idx % 2) == 1:
                line_points.reverse()
            way_xy.extend(line_points)
            line_idx += 1

        # yaw を次点方向に合わせる
        waypoints: List[Tuple[float, float, float]] = []
        for i, (x, y) in enumerate(way_xy):
            if i < len(way_xy) - 1:
                nx, ny = way_xy[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = waypoints[-1][2] if len(waypoints) > 0 else 0.0
            waypoints.append((x, y, yaw))

        return waypoints

    @staticmethod
    def find_segments(bool_row: np.ndarray) -> List[Tuple[int, int]]:
        segments: List[Tuple[int, int]] = []
        in_seg = False
        start = 0
        for i, v in enumerate(bool_row.tolist()):
            if v and not in_seg:
                in_seg = True
                start = i
            elif (not v) and in_seg:
                segments.append((start, i - 1))
                in_seg = False
        if in_seg:
            segments.append((start, len(bool_row) - 1))
        return segments

    # ---------------------------
    # Nav2 Execution
    # ---------------------------
    def run_waypoints(self, wps: List[Tuple[float, float, float]]):
        for i, (x, y, yaw) in enumerate(wps):
            ok = self.try_goal(x, y, yaw, i, len(wps))
            if not ok:
                self.get_logger().warn(
                    f"[{i + 1}/{len(wps)}] skip waypoint due to failure."
                )

    def try_goal(self, x: float, y: float, yaw: float, i: int, n: int) -> bool:
        for attempt in range(self.max_retries + 1):
            self.get_logger().info(
                f"[{i + 1}/{n}] Goal send (attempt {attempt + 1}): x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
            )
            result = self.send_goal_and_wait(x, y, yaw, timeout_s=self.goal_timeout_s)
            if result is True:
                return True
        return False

    def send_goal_and_wait(
        self, x: float, y: float, yaw: float, timeout_s: float
    ) -> bool:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = yaw_to_quat(yaw)

        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2.")
            return False

        result_future = goal_handle.get_result_async()

        start = time.time()
        while rclpy.ok():
            if result_future.done():
                res = result_future.result()
                status = res.status
                if status == 4:  # SUCCEEDED
                    self.get_logger().info("Goal succeeded.")
                    return True
                else:
                    self.get_logger().warn(
                        f"Goal finished but not success. status={status}"
                    )
                    return False

            if (time.time() - start) > timeout_s:
                self.get_logger().warn("Goal timeout -> cancel.")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                return False

            rclpy.spin_once(self, timeout_sec=0.1)

        return False

    # ---------------------------
    # Markers
    # ---------------------------
    def publish_waypoint_markers(self, wps: List[Tuple[float, float, float]]):
        if self.marker_pub is None:
            return
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, (x, y, yaw) in enumerate(wps):
            mk = Marker()
            mk.header.frame_id = "map"
            mk.header.stamp = now
            mk.ns = "coverage_waypoints"
            mk.id = i
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD
            mk.pose.position.x = float(x)
            mk.pose.position.y = float(y)
            mk.pose.position.z = 0.05
            mk.pose.orientation = yaw_to_quat(yaw)
            mk.scale.x = 0.07
            mk.scale.y = 0.07
            mk.scale.z = 0.07
            mk.color.a = 1.0
            mk.color.r = 0.1
            mk.color.g = 0.8
            mk.color.b = 0.1
            ma.markers.append(mk)
        self.marker_pub.publish(ma)
        self.get_logger().info("Published waypoint markers to /coverage_waypoints")


def main():
    rclpy.init()
    node = CoverageWaypointNodeV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

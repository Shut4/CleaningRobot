#!/usr/bin/env python3
import math
import time
from typing import List, Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose

# もしRVizでwaypoint可視化したいなら有効化
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


class CoverageWaypointNode(Node):
    def __init__(self):
        super().__init__("coverage_waypoint_node")

        # ---- Params ----
        self.declare_parameter("pitch_m", 0.4)  # ライン間隔
        self.declare_parameter("step_m", 0.8)  # multiモード用（今は未使用）
        self.declare_parameter("segment_mode", "center")  # "center" 推奨（安定）
        self.declare_parameter("margin_m", 0.22)  # 障害物から離す距離
        self.declare_parameter("occ_th", 50)  # >= occ_th を障害物扱い
        self.declare_parameter("treat_unknown_as_obstacle", True)
        self.declare_parameter("goal_timeout_s", 60.0)
        self.declare_parameter("max_retries", 0)  # まずは0（スキップ優先）
        self.declare_parameter("publish_markers", True)

        self.pitch_m = float(self.get_parameter("pitch_m").value)
        self.margin_m = float(self.get_parameter("margin_m").value)
        self.occ_th = int(self.get_parameter("occ_th").value)
        self.treat_unknown = bool(self.get_parameter("treat_unknown_as_obstacle").value)
        self.goal_timeout_s = float(self.get_parameter("goal_timeout_s").value)
        self.max_retries = int(self.get_parameter("max_retries").value)
        self.segment_mode = str(self.get_parameter("segment_mode").value)
        self.publish_markers = bool(self.get_parameter("publish_markers").value)

        # ---- Subs / Action ----
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.on_map, 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        if HAS_MARKERS and self.publish_markers:
            self.marker_pub = self.create_publisher(
                MarkerArray, "/coverage_waypoints", 10
            )
        else:
            self.marker_pub = None

        self._map_msg: Optional[OccupancyGrid] = None
        self._started = False

        # 起動後少し待ってから main を走らせる（/map待ち、Nav2待ちをログに出すため）
        self.create_timer(0.5, self.kick)

        self.get_logger().info(
            "coverage_waypoint_node ready. Waiting for /map and Nav2 action..."
        )

    def on_map(self, msg: OccupancyGrid):
        # 1回受け取れば十分（マップ更新追従したいならここを変更）
        if self._map_msg is None:
            self._map_msg = msg
            self.get_logger().info(
                f"Received /map: {msg.info.width}x{msg.info.height}, res={msg.info.resolution}"
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
            return

        self.get_logger().info(
            f"Generated {len(waypoints)} waypoints. Start navigating..."
        )
        if self.marker_pub is not None:
            self.publish_waypoint_markers(waypoints)

        # ここから順次実行（同期）
        self.run_waypoints(waypoints)
        self.get_logger().info("Coverage navigation done.")
        # ノード終了したいならここで shutdown してもOK
        # rclpy.shutdown()

    # ---------------------------
    # Waypoint Generation
    # ---------------------------
    def build_waypoints_from_map(
        self, m: OccupancyGrid
    ) -> List[Tuple[float, float, float]]:
        """
        Returns list of (x, y, yaw) in map frame.
        """
        w = m.info.width
        h = m.info.height
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y

        data = np.array(m.data, dtype=np.int16).reshape((h, w))  # (row=y, col=x)

        # obstacle mask
        obs = data >= self.occ_th
        if self.treat_unknown:
            obs = np.logical_or(obs, data < 0)

        margin_cells = int(math.ceil(self.margin_m / res))
        pitch_cells = max(1, int(round(self.pitch_m / res)))

        # SAFE_FREE: obsの近傍(margin)を削る（素朴な近傍走査）
        safe = np.ones((h, w), dtype=bool)
        safe[obs] = False

        # 近傍に障害物があるセルを落とす
        if margin_cells > 0:
            # 重くなりすぎないよう、障害物座標だけ走査
            obs_ys, obs_xs = np.where(obs)
            for yy, xx in zip(obs_ys.tolist(), obs_xs.tolist()):
                y0 = max(0, yy - margin_cells)
                y1 = min(h - 1, yy + margin_cells)
                x0 = max(0, xx - margin_cells)
                x1 = min(w - 1, xx + margin_cells)
                safe[y0 : y1 + 1, x0 : x1 + 1] = False

        safe_free = safe  # Trueなら安全に踏める候補

        # 外接矩形（safe_freeが存在する範囲）
        ys, xs = np.where(safe_free)
        if len(xs) == 0:
            return []
        x_min, x_max = int(xs.min()), int(xs.max())
        y_min, y_max = int(ys.min()), int(ys.max())

        way_xy: List[Tuple[float, float]] = []

        # ラウンモア走査：yをpitchで刻み、各ラインは連続区間ごとに点を置く
        line_idx = 0
        for yy in range(y_min, y_max + 1, pitch_cells):
            row = safe_free[yy, x_min : x_max + 1]
            segments = self.find_segments(row)  # [(start,end)] in sliced coords
            # segment_mode="center"：各区間の中央だけ
            for a, b in segments:
                # 走れそうにない短区間は捨てる（最低でも数セル）
                if (b - a + 1) < max(3, 2 * margin_cells + 1):
                    continue
                cx = (a + b) // 2
                ix = x_min + cx
                # cell center -> world
                wx = ox + (ix + 0.5) * res
                wy = oy + (yy + 0.5) * res
                way_xy.append((wx, wy))

            # 往復：奇数ラインは逆順にする（移動ムダ減）
            if (line_idx % 2) == 1:
                # このラインで追加した分だけを逆順にしたいが、簡易に全体の末尾区間を逆にする
                # （厳密にやるなら lineごとにlist作ってextendする）
                # ここではline単位で作る
                pass

            line_idx += 1

        # lineごとの往復をちゃんとやる版（上の簡易を置き換え）
        way_xy = []
        line_idx = 0
        for yy in range(y_min, y_max + 1, pitch_cells):
            row = safe_free[yy, x_min : x_max + 1]
            segments = self.find_segments(row)
            line_points: List[Tuple[float, float]] = []
            for a, b in segments:
                if (b - a + 1) < max(3, 2 * margin_cells + 1):
                    continue
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
        """Trueが連続する区間を返す（rowは1D bool array）"""
        segments = []
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
        # 1点ずつ送る
        for i, (x, y, yaw) in enumerate(wps):
            ok = self.try_goal(x, y, yaw, i, len(wps))
            if not ok:
                self.get_logger().warn(
                    f"[{i + 1}/{len(wps)}] skip waypoint due to failure."
                )

    def try_goal(self, x: float, y: float, yaw: float, i: int, n: int) -> bool:
        for attempt in range(self.max_retries + 1):
            self.get_logger().info(
                f"[{i + 1}/{n}] Goal send (attempt {attempt + 1}): x={x:.2f}, y={y:.2f}"
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
            # 結果が来たら終了
            if result_future.done():
                res = result_future.result()
                status = res.status
                # 4=SUCCEEDED が一般的（Nav2のaction status）
                if status == 4:
                    self.get_logger().info("Goal succeeded.")
                    return True
                else:
                    self.get_logger().warn(
                        f"Goal finished but not success. status={status}"
                    )
                    return False

            # timeout
            if (time.time() - start) > timeout_s:
                self.get_logger().warn("Goal timeout -> cancel.")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                return False

            # 少しspin
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
    node = CoverageWaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

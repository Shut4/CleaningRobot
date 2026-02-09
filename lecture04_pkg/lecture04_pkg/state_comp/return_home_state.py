import time
import rclpy
from rclpy.action import ActionClient
from yasmin import State

import tf_transformations
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def quat_from_yaw(yaw: float):
    return tf_transformations.quaternion_from_euler(0.0, 0.0, float(yaw))  # (x,y,z,w)


class ReturnHomeState(State):
    def __init__(
        self,
        node,
        nav_action_name: str = "/navigate_to_pose",
        goal_timeout_s: float = 120.0,
    ):
        super().__init__(outcomes=["done", "failed"])
        self.node = node
        self.nav_client = ActionClient(node, NavigateToPose, nav_action_name)
        self.goal_timeout_s = float(goal_timeout_s)

    def execute(self, blackboard):
        home = getattr(blackboard, "home_pose", None)
        if home is None:
            self.node.get_logger().error("[RETURN] home_pose not set in blackboard.")
            return "failed"

        x, y, yaw = home
        self.node.get_logger().info(
            f"[RETURN] go home: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("[RETURN] NavigateToPose server not ready.")
            return "failed"

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        q = quat_from_yaw(yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        goal = NavigateToPose.Goal()
        goal.pose = pose

        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
        if not send_future.done():
            self.node.get_logger().error("[RETURN] send_goal_async timed out.")
            return "failed"

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("[RETURN] goal rejected.")
            return "failed"

        result_future = goal_handle.get_result_async()
        start = time.time()

        while rclpy.ok():
            if result_future.done():
                res = result_future.result()
                status = res.status
                self.node.get_logger().info(f"[RETURN] finished status={status}")
                return "done" if status == 4 else "failed"

            if (time.time() - start) > self.goal_timeout_s:
                self.node.get_logger().warn("[RETURN] timeout -> cancel.")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(
                    self.node, cancel_future, timeout_sec=2.0
                )
                return "failed"

            rclpy.spin_once(self.node, timeout_sec=0.1)

        return "failed"

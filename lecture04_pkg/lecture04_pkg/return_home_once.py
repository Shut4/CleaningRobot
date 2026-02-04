#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

import tf2_ros
import tf_transformations

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class ReturnHomeOnce(Node):
    def __init__(self):
        super().__init__("return_home_once")

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Nav2 Action
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.home = None  # (x,y,yaw)

    def wait_and_save_home(self):
        self.get_logger().info("Waiting TF: map -> base_link to save HOME...")
        while rclpy.ok():
            try:
                t = self.tf_buffer.lookup_transform("map", "base_link", Time())
                x = float(t.transform.translation.x)
                y = float(t.transform.translation.y)

                q = t.transform.rotation
                yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

                self.home = (x, y, yaw)
                self.get_logger().info(
                    f"HOME saved: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
                )
                return
            except Exception as e:
                self.get_logger().info(f"TF not ready yet: {e}")
                rclpy.spin_once(self, timeout_sec=0.2)

    def send_goal(self, x, y, yaw):
        self.get_logger().info("Waiting /navigate_to_pose action server...")
        while not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("...still waiting")

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)

        q = tf_transformations.quaternion_from_euler(0.0, 0.0, float(yaw))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f"Sending goal HOME: ({x:.3f}, {y:.3f}, {yaw:.3f})")
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        self.get_logger().info("Navigating...")
        while rclpy.ok():
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.2)
            if result_future.done():
                break

        result = result_future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("SUCCEEDED: Returned HOME.")
            return True
        else:
            self.get_logger().error(f"FAILED: status={status}")
            return False


def main():
    rclpy.init()
    node = ReturnHomeOnce()

    # 1) 初期位置保存（ここでRVizの2D Pose Estimate後にやると安定）
    input(">> ロボットを初期位置に置いて、自己位置を合わせたら Enter（HOME保存）: ")
    node.wait_and_save_home()

    # 2) 適当に動かしてから戻す（テスト用）
    input(">> 手動でロボットを動かす/少し走らせたら Enter（HOMEへ戻る）: ")
    x, y, yaw = node.home
    node.send_goal(x, y, yaw)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

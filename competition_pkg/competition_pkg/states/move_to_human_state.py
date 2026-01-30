# move_to_human_state.py
from yasmin import State
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import tf_transformations
import rclpy

class MoveToHumanState(State):
    def __init__(self, node):
        super().__init__(outcomes=["succeed", "failed"])
        self.node = node
        self.client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self._result_future = None
        self._goal_handle = None

    def execute(self, blackboard):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("アクションサーバが応答しません")
            return "failed"

        pose = PoseStamped()
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = 2.4118
        pose.pose.position.y = 0.0679
        pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, 0.0365)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.node.get_logger().info("人の位置に移動します")

        future = self.client.send_goal_async(goal_msg)        self.sub.destroy()
        rclpy.spin_until_future_complete(self.node, future)

        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            return "failed"

        self._result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, self._result_future)

        result = self._result_future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            return "succeed"
        else:
            return "failed"
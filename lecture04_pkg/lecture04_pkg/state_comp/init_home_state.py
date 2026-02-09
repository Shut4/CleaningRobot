import tf2_ros
import tf_transformations
from rclpy.time import Time
from yasmin import State


class InitHomeState(State):
    def __init__(self, node, tf_buffer: tf2_ros.Buffer):
        super().__init__(outcomes=["done"])
        self.node = node
        self.tf_buffer = tf_buffer

    def execute(self, blackboard):
        self.node.get_logger().info("[INIT_HOME] trying to save home pose...")

        while True:
            try:
                t = self.tf_buffer.lookup_transform("map", "base_link", Time())
                x = float(t.transform.translation.x)
                y = float(t.transform.translation.y)
                q = t.transform.rotation
                yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

                blackboard["home_pose"] = (x, y, yaw)
                self.node.get_logger().info(
                    f"[INIT_HOME] home saved: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
                )
                return "done"
            except Exception:
                # TF待ち
                import rclpy

                rclpy.spin_once(self.node, timeout_sec=0.1)

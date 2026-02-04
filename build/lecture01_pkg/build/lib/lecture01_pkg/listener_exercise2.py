#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: listener_exercise2.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibikino-Musashi@Home）
"""


# Import modules (ROS2 related)
import rclpy
from rclpy.node import Node

# TODO: Import modules (geometry_msgs/Twist message type)
from geometry_msgs.msg import Twist


class Listener(Node):
    """Listener class (inherits from Node class)
    ROS2 node class that subscribes to a specific topic

    Attributes:
        subscription (Subscription): ROS2 subscriber to subscribe to a specific topic
    """

    def __init__(self):
        """Class initialization method"""
        # Override the constructor of the inherited Node class (argument is 'node name')
        super().__init__("listener_exercise2")

        # TODO: Declare subscriber
        self.subscription = self.create_subscription(
            msg_type=Twist, topic="aiu", callback=self.listener_callback, qos_profile=10
        )

        # Prevent warning (not required)
        self.subscription

    def listener_callback(self, msg: Twist):
        """Callback method called when a subscribed message is received

        Args:
            msg (Twist): Received message
        """
        # Display log
        self.get_logger().info(f"I heard: '{msg}'")


def main(args=None):
    """Main function"""
    # Initialize ROS2 Python client library
    rclpy.init(args=args)

    # Create instance of Listener class
    listener = Listener()

    # Run the specified node until it terminates
    rclpy.spin(listener)

    # Cleanup
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

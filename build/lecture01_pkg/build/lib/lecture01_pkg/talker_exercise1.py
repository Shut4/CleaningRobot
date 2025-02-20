#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: talker_exercise1.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibikino-Musashi@Home）
"""

# Import modules (ROS2 related)
import rclpy
from rclpy.node import Node

# TODO: Import modules (std_msgs/Int64 message type)
from std_msgs.msg import Int64


class Talker(Node):
    """Talker class (inherits from Node class)
    ROS2 node class that publishes messages to a specific topic at specified intervals

    Attributes:
        publisher (Publisher): ROS2 publisher to publish messages
        timer (Timer): Timer to call callback method at specified intervals
    """

    def __init__(self):
        """Class initialization method"""
        # Override the constructor of the inherited Node class (argument is 'node name')
        super().__init__("talker_exercise1")

        # TODO: Declare publisher
        self.publisher = self.create_publisher(msg_type=Int64, topic="aiu", qos_profile=10)



        # Define interrupt timer (calls callback method at specified period [s])
        self.timer = self.create_timer(timer_period_sec=0.5, callback=self.timer_callback)

    def timer_callback(self):
        """Callback method for interrupt timer"""
        # TODO: Define message
        msg = Int64()
        msg.data = 2025

        # Publish message
        self.publisher.publish(msg)

        # Display log
        self.get_logger().info(str(msg.data))


def main(args=None):
    """Main function"""
    # Initialize ROS2 Python client library
    rclpy.init(args=args)

    # Create instance of Talker class
    talker = Talker()

    # Run the specified node until it terminates
    rclpy.spin(talker)

    # Cleanup
    talker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: talker.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibikino-Musashi@Home）
"""

# Import modules (ROS2 related)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    """Talker class (inherits from Node class)
    ROS2 node class that publishes messages to a specific topic at specified intervals (simple publisher example)

    Attributes:
        publisher (Publisher): ROS2 publisher to publish messages
        timer (Timer): Timer to call callback method at specified intervals
    """

    def __init__(self):
        """Class initialization method"""
        # Override the constructor of the inherited Node class (argument is 'node name')
        super().__init__("talker")

        # Declare publisher
        self.publisher = self.create_publisher(msg_type=String, topic="chatter", qos_profile=10)

        # Define interrupt timer (calls callback method at specified period [s])
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        """Callback method for interrupt timer"""
        # Define message
        msg = String()
        msg.data = f"hello world {self.get_clock().now()}"

        # Publish message
        self.publisher.publish(msg)

        # Display log
        self.get_logger().info(msg.data)


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

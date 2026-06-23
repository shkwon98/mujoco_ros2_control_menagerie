#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__("robot_description_publisher")
        self.declare_parameter("robot_description", "")

        self._message = String()
        self._message.data = (
            self.get_parameter("robot_description")
            .get_parameter_value()
            .string_value
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(String, "robot_description", qos)
        self._publish_robot_description()

    def _publish_robot_description(self):
        self._publisher.publish(self._message)


def main():
    rclpy.init()
    node = RobotDescriptionPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

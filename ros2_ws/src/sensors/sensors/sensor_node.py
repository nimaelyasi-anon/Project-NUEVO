from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SensorNode(Node):
    def __init__(self) -> None:
        super().__init__("sensor_node")

        self.ultrasonic_pub = self.create_publisher(Float32, "/ultrasonic_distance", 10)
        self.timer = self.create_timer(1.0, self.publish_test_distance)

        self.get_logger().info("sensor node started")

    def publish_test_distance(self) -> None:
        msg = Float32()
        msg.data = 12.0
        self.ultrasonic_pub.publish(msg)
        self.get_logger().info(f"Published ultrasonic distance: {msg.data} cm")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

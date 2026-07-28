import rclpy
from rclpy.node import Node


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__("obstacle_detector")

        self.get_logger().info(
            "Obstacle Detector baseline node started."
        )

        self.timer = self.create_timer(
            5.0,
            self.timer_callback
        )

    def timer_callback(self):
        self.get_logger().info(
            "Waiting for sensors (Phase 2)..."
        )


def main(args=None):
    rclpy.init(args=args)

    node = ObstacleDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
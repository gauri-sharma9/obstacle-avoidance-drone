import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

SAFE_DISTANCE_M = 3.0


class AvoidanceNode(Node):
    def __init__(self):
        super().__init__('avoidance_node')
        self.create_subscription(String, '/fusion/obstacle_summary', self.callback, 10)
        self.publisher_ = self.create_publisher(String, '/avoidance/command', 10)
        self.get_logger().info('Avoidance decision node started')

    def callback(self, msg):
        data = json.loads(msg.data)
        radar_dist = data.get("radar_closest_m")
        camera_hits = data.get("camera_detections", 0)

        cmd = String()
        if radar_dist is not None and radar_dist < SAFE_DISTANCE_M:
            cmd.data = f"AVOID: radar obstacle at {radar_dist}m — hold position"
            self.get_logger().warn(cmd.data)
        elif camera_hits > 0:
            cmd.data = f"CAUTION: camera detected {camera_hits} object(s) — reduce speed"
            self.get_logger().warn(cmd.data)
        else:
            cmd.data = "CLEAR: no obstacles detected — proceed"
            self.get_logger().info(cmd.data)
        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
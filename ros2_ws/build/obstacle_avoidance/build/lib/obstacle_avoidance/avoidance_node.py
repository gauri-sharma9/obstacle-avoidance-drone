"""avoidance_node.py — High-level avoidance decision node

Subscribes to:
  /fusion/obstacle_summary  (std_msgs/String, JSON)

Publishes:
  /avoidance/command  (std_msgs/String, plain-text command)

Decision logic:
  - If any fused sector reports an obstacle < CRITICAL_DISTANCE_M → STOP
  - If closest obstacle < WARN_DISTANCE_M → CAUTION + bearing
  - If camera edge_threat confirmed → additional CAUTION
  - Otherwise → CLEAR

Note: ArduPilot handles actual avoidance autonomously via OBSTACLE_DISTANCE
      (sent by mavlink_obstacle_node). This node provides human-readable
      logs and an optional override channel for custom controllers.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

# Distances in metres
CRITICAL_DISTANCE_M = 1.5   # Stop / emergency hold
WARN_DISTANCE_M     = 3.0   # Slow down / caution


class AvoidanceNode(Node):
    def __init__(self):
        super().__init__('avoidance_node')
        self.create_subscription(
            String, '/fusion/obstacle_summary', self.callback, 10
        )
        self.publisher_ = self.create_publisher(String, '/avoidance/command', 10)
        self.get_logger().info(
            f'Avoidance decision node started '
            f'(critical={CRITICAL_DISTANCE_M}m, warn={WARN_DISTANCE_M}m)'
        )

    def callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse obstacle_summary JSON: {e}')
            return

        closest_m        = data.get('closest_m', float('inf'))
        closest_sector   = data.get('closest_sector_deg', -1)
        camera_confirm   = data.get('camera_confirming', False)

        cmd = String()

        if closest_m <= CRITICAL_DISTANCE_M:
            cmd.data = (
                f'CRITICAL: obstacle at {closest_m:.2f}m '
                f'(sector {closest_sector}°) — emergency hold'
            )
            self.get_logger().error(cmd.data)

        elif closest_m <= WARN_DISTANCE_M:
            suffix = ' [camera confirmed]' if camera_confirm else ''
            cmd.data = (
                f'CAUTION: obstacle at {closest_m:.2f}m '
                f'(sector {closest_sector}°){suffix} — reducing speed'
            )
            self.get_logger().warn(cmd.data)

        elif camera_confirm:
            cmd.data = (
                f'CAUTION: camera sees obstacle (sensors clear @ {closest_m:.2f}m) '
                '— slowing'
            )
            self.get_logger().warn(cmd.data)

        else:
            cmd.data = f'CLEAR: nearest obstacle at {closest_m:.2f}m — proceed'
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
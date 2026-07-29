import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import math
import random

# Match these to your actual demo_world.sdf obstacle positions.
# Check with: grep -A 3 "obstacle_box\|obstacle_cyl" simulation/worlds/demo_world.sdf
SIMULATED_OBSTACLES = [
    {"id": "obstacle_box_1", "x": 3.0, "y": 1.0, "z": 0.5},
    {"id": "obstacle_cyl_1", "x": 5.0, "y": -2.0, "z": 0.5},
    {"id": "obstacle_box_2", "x": -2.0, "y": 3.0, "z": 0.5},
    {"id": "obstacle_cyl_2", "x": -4.0, "y": -1.0, "z": 0.5},
]

RADAR_ORIGIN = (0.0, 0.0, 0.2)   # drone's approximate spawn point
RADAR_MAX_RANGE = 15.0            # meters
RANGE_NOISE_STDDEV = 0.05         # meters, simulates radar measurement noise


class RadarNode(Node):
    def __init__(self):
        super().__init__('radar_node')
        self.publisher_ = self.create_publisher(MarkerArray, '/radar/detections', 10)
        self.timer = self.create_timer(0.5, self.scan_callback)  # 2 Hz radar sweep
        self.get_logger().info('Simulated radar node started, publishing on /radar/detections')

    def scan_callback(self):
        marker_array = MarkerArray()
        ox, oy, oz = RADAR_ORIGIN
        detections_in_range = 0

        for i, obs in enumerate(SIMULATED_OBSTACLES):
            dx = obs["x"] - ox
            dy = obs["y"] - oy
            dz = obs["z"] - oz
            true_range = math.sqrt(dx**2 + dy**2 + dz**2)

            if true_range > RADAR_MAX_RANGE:
                continue

            noisy_range = true_range + random.gauss(0, RANGE_NOISE_STDDEV)
            detections_in_range += 1

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'radar'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = obs["x"]
            marker.pose.position.y = obs["y"]
            marker.pose.position.z = obs["z"]
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker_array.markers.append(marker)

            self.get_logger().info(
                f'Radar detection: {obs["id"]} at range {noisy_range:.2f} m'
            )

        self.publisher_.publish(marker_array)
        if detections_in_range == 0:
            self.get_logger().info('Radar sweep complete: no obstacles in range')


def main(args=None):
    rclpy.init(args=args)
    node = RadarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
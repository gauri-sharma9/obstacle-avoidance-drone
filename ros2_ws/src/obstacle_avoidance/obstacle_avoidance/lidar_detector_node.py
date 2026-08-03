import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Float32
import math

LIDAR_TOPIC = '/lidar/points'


class LidarDetectorNode(Node):
    def __init__(self):
        super().__init__('lidar_detector_node')
        self.subscription = self.create_subscription(
            PointCloud2, LIDAR_TOPIC, self.scan_callback, 10
        )
        self.closest_pub = self.create_publisher(Float32, '/lidar/closest_obstacle_m', 10)
        self.get_logger().info(f'LiDAR detector node started, subscribed to {LIDAR_TOPIC}')

    def scan_callback(self, msg):
        closest = float('inf')
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            dist = math.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            if dist < closest:
                closest = dist

        out = Float32()
        if closest < float('inf'):
            out.data = closest
            self.get_logger().info(f'LiDAR (live): closest obstacle at {closest:.2f} m')
        else:
            out.data = -1.0
        self.closest_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
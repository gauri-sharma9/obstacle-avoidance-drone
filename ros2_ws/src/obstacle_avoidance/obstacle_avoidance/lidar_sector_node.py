import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray
import math

LIDAR_TOPIC = '/lidar/points'
NUM_SECTORS = 72          # 360 / 5 degrees per sector, ArduPilot's expected layout
MAX_RANGE_CM = 1000.0     # 10m max range, matches sensor spec


class LidarSectorNode(Node):
    def __init__(self):
        super().__init__('lidar_sector_node')
        self.subscription = self.create_subscription(
            PointCloud2, LIDAR_TOPIC, self.scan_callback, 10
        )
        self.publisher_ = self.create_publisher(Float32MultiArray, '/lidar/sector_distances_cm', 10)
        self.get_logger().info(f'LiDAR sector node started, subscribed to {LIDAR_TOPIC}')

    def scan_callback(self, msg):
        sectors = [MAX_RANGE_CM] * NUM_SECTORS
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y = point[0], point[1]
            dist_cm = math.sqrt(x**2 + y**2) * 100.0
            if dist_cm > MAX_RANGE_CM or dist_cm < 20.0:  # Ignore self-reflections on drone body (<20cm)
                continue
            angle_deg = math.degrees(math.atan2(y, x))
            if angle_deg < 0:
                angle_deg += 360.0
            sector_idx = int(angle_deg / 5.0) % NUM_SECTORS
            if dist_cm < sectors[sector_idx]:
                sectors[sector_idx] = dist_cm

        out = Float32MultiArray()
        out.data = sectors
        self.publisher_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import json
import time

NUM_SECTORS = 72
MAX_RANGE_CM = 1500.0
CAMERA_TIMEOUT_S = 3.0


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.lidar_sectors = [MAX_RANGE_CM] * NUM_SECTORS
        self.radar_sectors = [MAX_RANGE_CM] * NUM_SECTORS
        self.camera_last_time = None
        self.camera_count = 0

        self.create_subscription(Float32MultiArray, '/lidar/sector_distances_cm', self.lidar_cb, 10)
        self.create_subscription(Float32MultiArray, '/radar/sector_distances_cm', self.radar_cb, 10)
        self.create_subscription(String, '/camera/detection_status', self.camera_cb, 10)

        self.fused_pub = self.create_publisher(Float32MultiArray, '/fused/obstacle_sectors_cm', 10)
        self.summary_pub = self.create_publisher(String, '/fusion/obstacle_summary', 10)
        self.timer = self.create_timer(0.5, self.fuse_and_publish)
        self.get_logger().info('Fusion node started: lidar + radar + camera -> 72-sector array')

    def lidar_cb(self, msg):
        self.lidar_sectors = list(msg.data)

    def radar_cb(self, msg):
        self.radar_sectors = list(msg.data)

    def camera_cb(self, msg):
        data = json.loads(msg.data)
        self.camera_last_time = time.time()
        self.camera_count = data.get("count", 0)

    def fuse_and_publish(self):
        fused = [
            min(l, r) for l, r in zip(self.lidar_sectors, self.radar_sectors)
        ]

        camera_active = (
            self.camera_last_time is not None
            and (time.time() - self.camera_last_time) < CAMERA_TIMEOUT_S
        )
        # Camera acts as a confidence tightener: if it visually confirms an object,
        # shrink the safety margin slightly on the forward sectors (0 and nearby),
        # since a visual confirmation raises confidence there's really something there.
        if camera_active and self.camera_count > 0:
            for i in list(range(0, 3)) + list(range(NUM_SECTORS - 3, NUM_SECTORS)):
                fused[i] = max(fused[i] * 0.9, 30.0)

        out = Float32MultiArray()
        out.data = fused
        self.fused_pub.publish(out)

        closest_cm = min(fused)
        closest_sector = fused.index(closest_cm)
        summary = {
            "closest_m": round(closest_cm / 100.0, 2),
            "closest_sector_deg": closest_sector * 5,
            "camera_confirming": camera_active and self.camera_count > 0,
        }
        s = String()
        s.data = json.dumps(summary)
        self.summary_pub.publish(s)
        self.get_logger().info(f'Fusion: {summary}')


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
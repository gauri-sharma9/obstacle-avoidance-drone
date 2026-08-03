import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
import json
import time

CAMERA_TIMEOUT_S = 3.0  # how long camera signal stays "recent"


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.radar_closest = None
        self.camera_last_frame_time = None
        self.camera_detection_count = 0

        self.create_subscription(MarkerArray, '/radar/detections', self.radar_cb, 10)
        self.create_subscription(String, '/camera/detection_status', self.camera_cb, 10)
        self.publisher_ = self.create_publisher(String, '/fusion/obstacle_summary', 10)
        self.timer = self.create_timer(1.0, self.fuse_and_publish)
        self.get_logger().info('Fusion node started (radar + camera)')

    def radar_cb(self, msg):
        if not msg.markers:
            self.radar_closest = None
            return
        closest = min(
            (m.pose.position.x**2 + m.pose.position.y**2 + m.pose.position.z**2) ** 0.5
            for m in msg.markers
        )
        self.radar_closest = closest

    def camera_cb(self, msg):
        data = json.loads(msg.data)
        self.camera_last_frame_time = time.time()
        self.camera_detection_count = data.get("count", 0)

    def fuse_and_publish(self):
        camera_active = (
            self.camera_last_frame_time is not None
            and (time.time() - self.camera_last_frame_time) < CAMERA_TIMEOUT_S
        )
        result = {
            "radar_closest_m": round(self.radar_closest, 2) if self.radar_closest else None,
            "camera_active": camera_active,
            "camera_detections": self.camera_detection_count if camera_active else 0,
        }
        msg = String()
        msg.data = json.dumps(result)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Fusion: {result}')


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
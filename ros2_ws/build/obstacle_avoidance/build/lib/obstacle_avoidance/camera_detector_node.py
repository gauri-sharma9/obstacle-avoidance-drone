import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import json
import cv2
import numpy as np

CAMERA_TOPIC = '/camera'


class CameraDetectorNode(Node):
    def __init__(self):
        super().__init__('camera_detector_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            10
        )
        self.status_pub = self.create_publisher(String, '/camera/detection_status', 10)
        self.get_logger().info(f'Camera detector node started, subscribed to {CAMERA_TOPIC}')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_lower = np.array([0, 100, 100])
        red_upper = np.array([10, 255, 255])
        blue_lower = np.array([100, 100, 100])
        blue_upper = np.array([130, 255, 255])

        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
        combined_mask = cv2.bitwise_or(red_mask, blue_mask)

        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = 0
        for c in contours:
            area = cv2.contourArea(c)
            if area > 200:
                x, y, w, h = cv2.boundingRect(c)
                detections += 1
                self.get_logger().info(
                    f'Camera detection #{detections}: bbox=({x},{y},{w},{h}), area={area:.0f}'
                )

        if detections == 0:
            self.get_logger().info('Camera frame processed: no obstacles detected')

        status = String()
        status.data = json.dumps({"count": detections})
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = CameraDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
"""camera_detector_node.py — Camera-based obstacle detector

Subscribes to:
  /camera  (sensor_msgs/Image, R8G8B8 color feed from Gazebo)

Publishes:
  /camera/detection_status  (std_msgs/String, JSON)

Detection strategy:
  1. Color detection — looks for obstacle colors (red boxes, blue cylinders,
     orange/yellow/purple obstacles placed in the Gazebo world).
  2. Edge-density analysis — counts dense edge regions in the image center,
     indicating near obstacles even without known colors.
  3. Reports obstacle count, bearing angles, and a binary "threat" flag.

The fusion node uses this as a third sensor modality to raise confidence
when a visual confirmation of an obstacle exists.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import json
import cv2
import numpy as np

CAMERA_TOPIC = '/camera'
# Full horizontal FOV ≈ 86 degrees (1.5 rad) — matches model.sdf
HFOV_DEG = 86.0
# Minimum contour area (px²) to count as a real detection
MIN_CONTOUR_AREA = 200
# Edge density in ROI that triggers "close obstacle" warning (0-1)
EDGE_DENSITY_THRESHOLD = 0.30
# Central ROI fraction for edge density (ignore periphery)
ROI_FRACTION = 0.5


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
        self.get_logger().info(
            f'Camera detector node started, subscribed to {CAMERA_TOPIC}'
        )

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _color_detections(self, frame: np.ndarray, img_w: int) -> list:
        """Return list of dicts for each color-matched obstacle contour."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Obstacle color ranges: red (wraps), blue, orange, yellow, purple
        masks = {
            'red':    cv2.bitwise_or(
                        cv2.inRange(hsv, np.array([0, 70, 50]),   np.array([10, 255, 255])),
                        cv2.inRange(hsv, np.array([170, 70, 50]), np.array([180, 255, 255]))
                      ),
            'blue':   cv2.inRange(hsv, np.array([100, 70, 50]), np.array([130, 255, 255])),
            'orange': cv2.inRange(hsv, np.array([10, 100, 100]), np.array([20, 255, 255])),
            'yellow': cv2.inRange(hsv, np.array([20, 80, 50]),  np.array([35, 255, 255])),
            'purple': cv2.inRange(hsv, np.array([135, 70, 50]), np.array([165, 255, 255])),
        }
        combined = masks['red']
        for m in list(masks.values())[1:]:
            combined = cv2.bitwise_or(combined, m)

        # Morphological cleanup — remove noise, fill holes
        kernel = np.ones((5, 5), np.uint8)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_CONTOUR_AREA:
                continue
            x, y, w, h = cv2.boundingRect(c)
            center_x = x + w / 2.0
            # Map pixel center_x to bearing angle in degrees
            bearing_deg = ((center_x / float(img_w)) - 0.5) * HFOV_DEG
            # Determine which color matched
            color_name = 'unknown'
            cx_i, cy_i = int(center_x), int(y + h / 2.0)
            if 0 <= cy_i < frame.shape[0] and 0 <= cx_i < frame.shape[1]:
                for name, mask in masks.items():
                    if mask[cy_i, cx_i] > 0:
                        color_name = name
                        break
            detections.append({
                'bbox':        [x, y, w, h],
                'area':        float(area),
                'bearing_deg': round(bearing_deg, 2),
                'color':       color_name,
            })
            self.get_logger().info(
                f'Camera [{color_name}]: bbox=({x},{y},{w},{h}), '
                f'bearing={bearing_deg:.1f}°, area={area:.0f}px²'
            )
        return detections

    def _edge_threat(self, frame: np.ndarray) -> bool:
        """Return True if edge density in the central ROI exceeds threshold.

        This catches grey/brown obstacles that have no distinct color but
        create sharp edges when close to the camera.
        """
        h, w = frame.shape[:2]
        roi_x0 = int(w * (1 - ROI_FRACTION) / 2)
        roi_y0 = int(h * (1 - ROI_FRACTION) / 2)
        roi = frame[roi_y0:h - roi_y0, roi_x0:w - roi_x0]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        density = float(np.count_nonzero(edges)) / (edges.shape[0] * edges.shape[1])
        return density > EDGE_DENSITY_THRESHOLD

    # ── Main callback ─────────────────────────────────────────────────────────

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        img_w = frame.shape[1]
        detections = self._color_detections(frame, img_w)
        edge_threat = self._edge_threat(frame)

        if not detections and not edge_threat:
            self.get_logger().debug('Camera: no obstacles detected')

        status = String()
        status.data = json.dumps({
            'count':       len(detections),
            'detections':  detections,
            'edge_threat': edge_threat,
        })
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = CameraDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
"""fusion_node.py — Sensor fusion: LiDAR + Radar + Camera → single obstacle map

Subscribes to:
  /lidar/sector_distances_cm   Float32MultiArray  72 sectors, cm
  /radar/sector_distances_cm   Float32MultiArray  72 sectors, cm
  /camera/detection_status     String (JSON)

Publishes at 2 Hz:
  /fused/obstacle_sectors_cm   Float32MultiArray  72 sectors, cm  (→ mavlink_obstacle_node)
  /fusion/obstacle_summary     String (JSON)                       (→ avoidance_node)

Fusion strategy:
  1. Per-sector minimum of lidar and radar.
  2. If camera reports a bearing-anchored detection, shrink the corresponding
     sector(s) by a configurable factor to raise priority.
  3. If camera reports an edge-threat (generalized proximity), shrink the
     forward 5 sectors as a conservative penalty.
  4. Publish closest distance/sector in summary JSON.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import json
import time

NUM_SECTORS     = 72        # 360° / 5° per sector
MAX_RANGE_CM    = 1500.0    # 15 m — radar/lidar max
CAMERA_TIMEOUT_S = 3.0      # Discard stale camera data after this
# How much to shrink a sector when camera confirms an object there
CAMERA_CONFIDENCE_FACTOR = 0.85
# Minimum floor so we never send 0 cm (ArduPilot ignores 0)
MIN_FLOOR_CM = 30.0


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        self.lidar_sectors: list[float] = [MAX_RANGE_CM] * NUM_SECTORS
        self.radar_sectors: list[float] = [MAX_RANGE_CM] * NUM_SECTORS
        self.camera_last_time: float | None = None
        self.camera_count: int = 0
        self.camera_bearings: list[float] = []   # bearing_deg per detection
        self.camera_edge_threat: bool = False

        self.create_subscription(
            Float32MultiArray, '/lidar/sector_distances_cm', self.lidar_cb, 10
        )
        self.create_subscription(
            Float32MultiArray, '/radar/sector_distances_cm', self.radar_cb, 10
        )
        self.create_subscription(
            String, '/camera/detection_status', self.camera_cb, 10
        )

        self.fused_pub  = self.create_publisher(
            Float32MultiArray, '/fused/obstacle_sectors_cm', 10
        )
        self.summary_pub = self.create_publisher(
            String, '/fusion/obstacle_summary', 10
        )
        # 2 Hz publish timer
        self.timer = self.create_timer(0.5, self.fuse_and_publish)
        self.get_logger().info(
            'Fusion node started: lidar + radar + camera → 72-sector array'
        )

    # ── Subscriptions ─────────────────────────────────────────────────────────

    def lidar_cb(self, msg: Float32MultiArray):
        if len(msg.data) == NUM_SECTORS:
            self.lidar_sectors = list(msg.data)
        else:
            self.get_logger().warn(
                f'LiDAR sector array size mismatch: got {len(msg.data)}, expected {NUM_SECTORS}'
            )

    def radar_cb(self, msg: Float32MultiArray):
        if len(msg.data) == NUM_SECTORS:
            self.radar_sectors = list(msg.data)

    def camera_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.camera_last_time  = time.time()
        self.camera_count      = data.get('count', 0)
        self.camera_edge_threat = data.get('edge_threat', False)
        self.camera_bearings   = [
            d['bearing_deg'] for d in data.get('detections', [])
        ]

    # ── Fusion ────────────────────────────────────────────────────────────────

    def _bearing_to_sector(self, bearing_deg: float) -> int:
        """Convert camera bearing (−43..+43°) to sector index (0..71)."""
        # bearing 0° = straight ahead = sector 0
        angle = bearing_deg % 360.0
        return int(angle / 5.0) % NUM_SECTORS

    def fuse_and_publish(self):
        # 1. Min-fusion of lidar and radar per sector
        fused: list[float] = [
            min(l, r) for l, r in zip(self.lidar_sectors, self.radar_sectors)
        ]

        # 2. Camera integration
        camera_active = (
            self.camera_last_time is not None
            and (time.time() - self.camera_last_time) < CAMERA_TIMEOUT_S
        )
        camera_confirming = camera_active and (self.camera_count > 0 or self.camera_edge_threat)

        if camera_active:
            # a) Bearing-anchored shrink for each detected object
            for bearing in self.camera_bearings:
                idx = self._bearing_to_sector(bearing)
                # Affect that sector and its two immediate neighbours
                for offset in (-1, 0, 1):
                    s = (idx + offset) % NUM_SECTORS
                    fused[s] = max(fused[s] * CAMERA_CONFIDENCE_FACTOR, MIN_FLOOR_CM)

            # b) General edge-threat → shrink forward cone (sectors 0–4 and 68–71)
            if self.camera_edge_threat:
                forward_sectors = list(range(0, 5)) + list(range(NUM_SECTORS - 4, NUM_SECTORS))
                for s in forward_sectors:
                    fused[s] = max(fused[s] * CAMERA_CONFIDENCE_FACTOR, MIN_FLOOR_CM)

        # 3. Enforce minimum floor
        fused = [max(v, MIN_FLOOR_CM) for v in fused]

        # 4. Publish fused array
        out = Float32MultiArray()
        out.data = fused
        self.fused_pub.publish(out)

        # 5. Publish summary
        closest_cm     = min(fused)
        closest_sector = fused.index(closest_cm)
        summary = {
            'closest_m':          round(closest_cm / 100.0, 2),
            'closest_sector_deg': closest_sector * 5,
            'camera_confirming':  camera_confirming,
        }
        s = String()
        s.data = json.dumps(summary)
        self.summary_pub.publish(s)

        self.get_logger().info(
            f'Fusion: closest={summary["closest_m"]}m @ '
            f'{summary["closest_sector_deg"]}° | cam={camera_confirming}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
"""radar_node.py — Simulated radar obstacle detector

Since the Iris drone does not have a real radar sensor in the Gazebo model,
this node simulates a radar by computing distances from the drone's current
position to a list of known obstacle locations (matching those in demo_world.sdf).

Drone position is tracked via the /mavros/local_position/pose topic if
available, otherwise the node defaults to origin.

Publishes:
  /radar/sector_distances_cm  (std_msgs/Float32MultiArray, 72 sectors)

Radar characteristics vs LiDAR:
  - Longer range (15 m vs 10 m)
  - Gaussian noise on measurements (simulates real RF behaviour)
  - Wider beam — smears readings across neighbouring sectors
  - Unaffected by optical interference (works through dust, smoke)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import random

# ── Obstacle map — must match simulation/worlds/demo_world.sdf ────────────────
KNOWN_OBSTACLES = [
    {'id': 'obstacle_box_1',  'x':  6.0, 'y':  1.0,  'z': 1.0, 'radius': 0.7},
    {'id': 'obstacle_cyl_1',  'x': 10.0, 'y': -1.5,  'z': 1.0, 'radius': 0.5},
    {'id': 'obstacle_box_2',  'x': 15.0, 'y':  0.5,  'z': 1.5, 'radius': 0.9},
    {'id': 'obstacle_cyl_2',  'x': 20.0, 'y': -1.0,  'z': 1.0, 'radius': 0.6},
]

NUM_SECTORS   = 72
MAX_RANGE_CM  = 1500.0   # 15 m radar max range
NOISE_STD_CM  = 15.0     # Gaussian noise std dev (cm)
BEAM_WIDTH    = 2        # ± sectors to spread radar return into
PUBLISH_HZ    = 2.0      # Hz


class RadarNode(Node):
    def __init__(self):
        super().__init__('radar_node')

        self.drone_x: float = 0.0
        self.drone_y: float = 0.0

        self.publisher_ = self.create_publisher(
            Float32MultiArray, '/radar/sector_distances_cm', 10
        )

        # 1. MAVLink position tracking (daemon thread)
        import threading
        self._mav_thread = threading.Thread(target=self._mavlink_position_loop, daemon=True)
        self._mav_thread.start()

        # 2. Optional: subscribe to drone local position from ROS topic if available
        try:
            from geometry_msgs.msg import PoseStamped
            self.create_subscription(
                PoseStamped,
                '/mavros/local_position/pose',
                self._pose_callback,
                10,
            )
        except Exception:
            pass

        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self._scan_callback)
        self.get_logger().info(
            f'Radar node started ({PUBLISH_HZ} Hz, max range {MAX_RANGE_CM/100:.0f} m, MAVLink position listener active)'
        )

    def _mavlink_position_loop(self):
        """Listen to MAVLink LOCAL_POSITION_NED from SITL on UDP 14551."""
        try:
            from pymavlink import mavutil
            mav_conn = mavutil.mavlink_connection('udpin:0.0.0.0:14551', source_system=254, source_component=190)
            while rclpy.ok():
                msg = mav_conn.recv_match(type=['LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT'], blocking=True, timeout=1.0)
                if msg is not None:
                    if msg.get_type() == 'LOCAL_POSITION_NED':
                        self.drone_x = msg.x
                        self.drone_y = msg.y
        except Exception as e:
            self.get_logger().debug(f'MAVLink position loop exception: {e}')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pose_callback(self, msg):
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y

    def _scan_callback(self):
        sectors = [MAX_RANGE_CM] * NUM_SECTORS

        for obs in KNOWN_OBSTACLES:
            dx = obs['x'] - self.drone_x
            dy = obs['y'] - self.drone_y
            # Distance to obstacle edge (subtract radius)
            dist_m = max(math.sqrt(dx ** 2 + dy ** 2) - obs['radius'], 0.1)
            dist_cm = dist_m * 100.0

            if dist_cm > MAX_RANGE_CM:
                continue

            # Add Gaussian noise
            noisy_cm = dist_cm + random.gauss(0.0, NOISE_STD_CM)
            noisy_cm = max(noisy_cm, 10.0)   # Never below 10 cm

            # Compute sector
            angle_deg = math.degrees(math.atan2(dy, dx))
            if angle_deg < 0:
                angle_deg += 360.0
            center_idx = int(angle_deg / 5.0) % NUM_SECTORS

            # Spread across beam width
            for offset in range(-BEAM_WIDTH, BEAM_WIDTH + 1):
                s = (center_idx + offset) % NUM_SECTORS
                # Attenuate slightly for off-centre sectors
                attenuation = 1.0 + abs(offset) * 0.05
                effective_cm = noisy_cm * attenuation
                if effective_cm < sectors[s]:
                    sectors[s] = effective_cm

            self.get_logger().debug(
                f'Radar: {obs["id"]} @ sector {center_idx} = {noisy_cm/100:.2f} m'
            )

        out = Float32MultiArray()
        out.data = sectors
        self.publisher_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RadarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
"""mavlink_obstacle_node.py — MAVLink OBSTACLE_DISTANCE streamer

Reads fused sector data from /fused/obstacle_sectors_cm and forwards it
to ArduPilot SITL as MAVLink OBSTACLE_DISTANCE messages (msg ID 330).
ArduPilot's Object Avoidance (OA_TYPE, PRX1_TYPE=2) uses these to
autonomously reroute the drone around detected obstacles.

Connection:
  udpin:127.0.0.1:14550  (MAVProxy must be forwarding to this port)
  Falls back to direct connection if MAVProxy is not present.

Component ID:
  196 = MAV_COMP_ID_OBSTACLE_AVOIDANCE

Message format: OBSTACLE_DISTANCE
  - 72 sectors, 5° per sector, sector 0 = forward (body-FRD)
  - Distances in cm, uint16
  - Streamed at up to 10 Hz (limited by fused data arrival ~2 Hz)
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from pymavlink import mavutil

# ── Constants ──────────────────────────────────────────────────────────────────
NUM_SECTORS   = 72
MAX_RANGE_CM  = 1500          # 15 m (radar max range)
MIN_RANGE_CM  = 10            # 10 cm minimum
MAVLINK_ADDR  = 'udpin:0.0.0.0:14550'
HEARTBEAT_HZ  = 2
HB_TIMEOUT_S  = 5.0
COMP_ID       = 196           # MAV_COMP_ID_OBSTACLE_AVOIDANCE


class MavlinkObstacleNode(Node):
    """ROS2 node: fused sectors → OBSTACLE_DISTANCE → ArduPilot SITL."""

    def __init__(self):
        super().__init__('mavlink_obstacle_node')
        self.get_logger().info(
            f'MAVLink obstacle node starting on {MAVLINK_ADDR}...'
        )

        # ── MAVLink connection ─────────────────────────────────────────────────
        self._master       = None
        self._connected    = False
        self._last_hb_time = 0.0
        self._lock         = threading.Lock()
        self._stop_evt     = threading.Event()

        self._connect()

        # Heartbeat send thread
        self._hb_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True, name='mav-hb'
        )
        self._hb_thread.start()

        # ── ROS subscription ───────────────────────────────────────────────────
        self.create_subscription(
            Float32MultiArray,
            '/fused/obstacle_sectors_cm',
            self._sectors_callback,
            10,
        )
        self.get_logger().info('Waiting for MAVLink heartbeat from ArduPilot...')

    # ── Connection ─────────────────────────────────────────────────────────────

    def _connect(self):
        """Try to open MAVLink UDP socket. Non-blocking."""
        try:
            self._master = mavutil.mavlink_connection(
                MAVLINK_ADDR,
                source_system=255,
                source_component=COMP_ID,
            )
            self.get_logger().info(f'MAVLink socket bound on {MAVLINK_ADDR}')
        except Exception as e:
            self.get_logger().error(f'MAVLink connect failed: {e}')
            self._master = None

    def _heartbeat_loop(self):
        """Send GCS heartbeat and watch for autopilot heartbeat."""
        while not self._stop_evt.is_set():
            if self._master is None:
                time.sleep(1.0)
                continue

            # Send our GCS heartbeat
            try:
                self._master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )
            except Exception:
                pass

            # Check for incoming heartbeat from autopilot
            try:
                msg = self._master.recv_match(type='HEARTBEAT', blocking=False)
                if msg is not None and msg.get_srcSystem() == 1:
                    with self._lock:
                        if not self._connected:
                            self.get_logger().info(
                                'ArduPilot heartbeat received — '
                                'streaming OBSTACLE_DISTANCE'
                            )
                        self._connected     = True
                        self._last_hb_time  = time.time()
            except Exception:
                pass

            # Warn if heartbeat lost
            with self._lock:
                if (self._connected
                        and (time.time() - self._last_hb_time) > HB_TIMEOUT_S):
                    self.get_logger().warn('ArduPilot heartbeat timeout!')
                    self._connected = False

            time.sleep(1.0 / HEARTBEAT_HZ)

    # ── ROS callback ───────────────────────────────────────────────────────────

    def _sectors_callback(self, msg: Float32MultiArray):
        with self._lock:
            connected = self._connected

        if not connected or self._master is None:
            return

        try:
            # Clamp and convert to uint16 cm values
            raw = list(msg.data)
            if len(raw) < NUM_SECTORS:
                raw += [MAX_RANGE_CM] * (NUM_SECTORS - len(raw))
            distances = [
                int(min(max(d, 0.0), 65534.0)) for d in raw[:NUM_SECTORS]
            ]

            self._master.mav.obstacle_distance_send(
                int(time.time() * 1e6),                         # time_usec
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,      # sensor_type
                distances,                                       # distances[72] uint16 cm
                0,                                               # increment (ignored if increment_f set)
                MIN_RANGE_CM,                                    # min_distance (cm)
                MAX_RANGE_CM,                                    # max_distance (cm)
                5.0,                                             # increment_f (degrees per sector)
                0.0,                                             # angle_offset (sector 0 = forward)
                mavutil.mavlink.MAV_FRAME_BODY_FRD,             # frame
            )

            closest = min(distances)
            self.get_logger().info(
                f'OBSTACLE_DISTANCE sent: closest={closest}cm '
                f'({closest/100:.2f}m)'
            )

        except Exception as e:
            self.get_logger().warn(f'Failed to send OBSTACLE_DISTANCE: {e}')

    # ── Cleanup ────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._stop_evt.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
from pymavlink import mavutil

KNOWN_OBSTACLES = [
    {"id": "obstacle_box_1", "x": 3.0, "y": 1.0, "z": 0.5},
    {"id": "obstacle_cyl_1", "x": 5.0, "y": -2.0, "z": 0.5},
    {"id": "obstacle_box_2", "x": -2.0, "y": 3.0, "z": 0.5},
    {"id": "obstacle_cyl_2", "x": -4.0, "y": -1.0, "z": 0.5},
]
NUM_SECTORS = 72
MAX_RANGE_CM = 1500.0  # radar has longer range than lidar


class RadarNode(Node):
    def __init__(self):
        super().__init__('radar_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/radar/sector_distances_cm', 10)
        self.drone_pos = (0.0, 0.0, 0.2)

        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.get_logger().info('Radar node waiting for MAVLink heartbeat...')
        self.master.wait_heartbeat()
        self.get_logger().info('Radar node connected to MAVLink')

        self.timer = self.create_timer(0.5, self.scan_callback)

    def update_drone_position(self):
        msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg is not None:
            self.drone_pos = (msg.x, msg.y, -msg.z)

    def scan_callback(self):
        self.update_drone_position()
        ox, oy, _ = self.drone_pos
        sectors = [MAX_RANGE_CM] * NUM_SECTORS

        for obs in KNOWN_OBSTACLES:
            dx, dy = obs["x"] - ox, obs["y"] - oy
            dist_cm = math.sqrt(dx**2 + dy**2) * 100.0
            if dist_cm > MAX_RANGE_CM:
                continue
            angle_deg = math.degrees(math.atan2(dy, dx))
            if angle_deg < 0:
                angle_deg += 360.0
            sector_idx = int(angle_deg / 5.0) % NUM_SECTORS
            if dist_cm < sectors[sector_idx]:
                sectors[sector_idx] = dist_cm
            self.get_logger().info(f'Radar: {obs["id"]} sector {sector_idx} at {dist_cm/100:.2f} m')

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
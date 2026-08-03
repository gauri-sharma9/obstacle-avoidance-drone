import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from pymavlink import mavutil
import time

NUM_SECTORS = 72
MAX_RANGE_CM = 1500


class MavlinkObstacleNode(Node):
    def __init__(self):
        super().__init__('mavlink_obstacle_node')
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.get_logger().info('Waiting for MAVLink heartbeat...')
        self.master.wait_heartbeat()
        self.get_logger().info('MAVLink connected; streaming OBSTACLE_DISTANCE to ArduPilot')
        self.create_subscription(Float32MultiArray, '/fused/obstacle_sectors_cm', self.callback, 10)

    def callback(self, msg):
        distances = [int(min(d, 65534)) for d in msg.data]  # uint16, capped
        self.master.mav.obstacle_distance_send(
            int(time.time() * 1e6),           # time_usec
            mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
            distances,                         # 72 sector distances in cm
            0,                                  # increment (0 = use increment_f)
            10,                                 # min_distance (cm)
            MAX_RANGE_CM,                       # max_distance (cm)
            5.0,                                 # increment_f (degrees per sector)
            0.0,                                 # angle_offset (sector 0 = forward)
            mavutil.mavlink.MAV_FRAME_BODY_FRD
        )
        self.get_logger().info(f'Sent OBSTACLE_DISTANCE: closest={min(distances)}cm')


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import time


class MissionCommanderNode(Node):
    def __init__(self):
        super().__init__('mission_commander_node')
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.get_logger().info('Waiting for heartbeat...')
        self.master.wait_heartbeat()
        self.get_logger().info('Connected. Starting mission in 3 seconds...')
        time.sleep(3)
        self.run_mission()

    def set_mode(self, mode_name):
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        self.get_logger().info(f'Mode set to {mode_name}')

    def arm(self):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().info('Arm command sent')

    def takeoff(self, alt_m):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt_m
        )
        self.get_logger().info(f'Takeoff to {alt_m}m sent')

    def goto(self, x, y, z):
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # position only
            x, y, -z,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().info(f'GOTO ({x}, {y}, {z}m) sent — flying through obstacle field')

    def run_mission(self):
        self.set_mode('GUIDED')
        time.sleep(2)
        self.arm()
        time.sleep(3)
        self.takeoff(3.0)
        time.sleep(8)
        # This waypoint deliberately crosses your obstacle cluster —
        # BendyRuler should visibly deviate around obstacle_box_1/cyl_1/box_2/cyl_2
        self.goto(6.0, 0.0, 3.0)
        self.get_logger().info('Mission commands sent. Watch Gazebo for avoidance behavior.')


def main(args=None):
    rclpy.init(args=args)
    node = MissionCommanderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
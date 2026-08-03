import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pymavlink import mavutil


class MavlinkCommandNode(Node):
    def __init__(self):
        super().__init__('mavlink_command_node')
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.get_logger().info('Waiting for MAVLink heartbeat...')
        self.master.wait_heartbeat()
        self.get_logger().info('MAVLink connected; ready to relay avoidance commands')
        self.create_subscription(String, '/avoidance/command', self.command_callback, 10)

    def command_callback(self, msg):
        if msg.data.startswith("AVOID"):
            self.master.mav.set_position_target_local_ned_send(
                0, self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111000111,
                0, 0, 0,
                0, 0, 0,
                0, 0, 0, 0, 0
            )
            self.get_logger().warn('MAVLink: HOLD command sent to flight controller')
        else:
            self.get_logger().info('MAVLink: no hold needed')


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
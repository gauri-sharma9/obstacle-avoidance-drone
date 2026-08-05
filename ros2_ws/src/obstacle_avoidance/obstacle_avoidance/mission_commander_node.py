import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import time


class MissionCommanderNode(Node):
    def __init__(self):
        super().__init__('mission_commander_node')
        self.get_logger().info('Mission commander node starting...')
        self.master = None
        self.connected = False
        self.mission_started = False
        self.timer = self.create_timer(1.0, self.check_connection_and_run)

    def try_connect(self):
        targets = [
            'udpin:0.0.0.0:14551',
            'udpin:0.0.0.0:14550',
            'tcp:127.0.0.1:5760',
            'tcp:127.0.0.1:5762',
            'udp:127.0.0.1:14550'
        ]
        for target in targets:
            try:
                conn = mavutil.mavlink_connection(target, source_system=255, source_component=191)
                # Send heartbeat ping first
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                msg = conn.recv_match(type=['HEARTBEAT', 'ATTITUDE', 'SYS_STATUS'], blocking=True, timeout=0.3)
                if msg is not None:
                    self.master = conn
                    self.connected = True
                    self.master.target_system = msg.get_srcSystem() if msg.get_srcSystem() > 0 else 1
                    self.master.target_component = msg.get_srcComponent() if msg.get_srcComponent() > 0 else 1
                    self.get_logger().info(
                        f'Connected to SITL on {target} (System {self.master.target_system})!'
                    )
                    return
            except Exception as e:
                self.get_logger().debug(f'Target {target} attempt failed: {e}')

    def check_connection_and_run(self):
        if not self.connected:
            self.try_connect()
            if not self.connected:
                self.get_logger().info('Waiting for MAVLink heartbeat from SITL/MAVProxy...')
                return

        if not self.mission_started:
            self.mission_started = True
            self.get_logger().info('=== STARTING AUTOMATIC TAKEOFF & FLIGHT SEQUENCE ===')
            self.configure_avoidance_parameters()
            time.sleep(1)
            self.set_mode('GUIDED')
            time.sleep(1)
            self.arm()
            time.sleep(2)
            self.takeoff(3.0)
            time.sleep(4)
            self.goto(25.0, 0.0, 3.0)
        else:
            # Periodically re-publish GOTO and ARM to ensure drone flies through obstacle field
            self.send_heartbeat_ping()
            self.set_mode('GUIDED')
            self.arm()
            self.goto(25.0, 0.0, 3.0)


def main(args=None):
    rclpy.init(args=args)
    node = MissionCommanderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
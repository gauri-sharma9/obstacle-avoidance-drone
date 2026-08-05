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
        targets = ['udpin:0.0.0.0:14551', 'udp:127.0.0.1:14551', 'tcp:127.0.0.1:5760', 'udpin:0.0.0.0:14550']
        for target in targets:
            try:
                conn = mavutil.mavlink_connection(target, source_system=255, source_component=191)
                self.master = conn
                self.get_logger().info(f'Trying MAVLink target {target}...')
                return
            except Exception as e:
                self.get_logger().debug(f'Target {target} failed: {e}')

    def set_parameter(self, param_name, param_value):
        if self.master is None:
            return
        try:
            self.master.mav.param_set_send(
                self.master.target_system if self.master.target_system > 0 else 1,
                self.master.target_component if self.master.target_component > 0 else 1,
                param_name.encode('utf-8'),
                float(param_value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            self.get_logger().info(f'Parameter set: {param_name} = {param_value}')
        except Exception as e:
            self.get_logger().warn(f'Failed to set parameter {param_name}: {e}')

    def configure_avoidance_parameters(self):
        self.get_logger().info('Configuring ArduPilot BendyRuler avoidance parameters...')
        self.set_parameter('PRX1_TYPE', 2)    # MAVLink Proximity
        self.set_parameter('AVOID_ENABLE', 7) # Avoidance enabled
        self.set_parameter('OA_TYPE', 1)       # BendyRuler path planning
        self.set_parameter('AVOID_MARGIN', 2.0)# 2.0m avoidance margin

    def set_mode(self, mode_name):
        if self.master is None:
            return
        try:
            mode_id = self.master.mode_mapping()[mode_name]
            target_sys = self.master.target_system if self.master.target_system > 0 else 1
            self.master.mav.set_mode_send(
                target_sys,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            self.get_logger().info(f'Mode set command sent: {mode_name}')
        except Exception as e:
            self.get_logger().warn(f'Failed to set mode {mode_name}: {e}')

    def arm(self):
        if self.master is None:
            return
        try:
            target_sys = self.master.target_system if self.master.target_system > 0 else 1
            target_comp = self.master.target_component if self.master.target_component > 0 else 1
            self.master.mav.command_long_send(
                target_sys, target_comp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
            )
            self.get_logger().info('ARM command sent!')
        except Exception as e:
            self.get_logger().warn(f'Arm failed: {e}')

    def takeoff(self, alt_m):
        if self.master is None:
            return
        try:
            target_sys = self.master.target_system if self.master.target_system > 0 else 1
            target_comp = self.master.target_component if self.master.target_component > 0 else 1
            self.master.mav.command_long_send(
                target_sys, target_comp,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt_m
            )
            self.get_logger().info(f'TAKEOFF command sent (target altitude: {alt_m}m)!')
        except Exception as e:
            self.get_logger().warn(f'Takeoff failed: {e}')

    def goto(self, x, y, z):
        if self.master is None:
            return
        try:
            target_sys = self.master.target_system if self.master.target_system > 0 else 1
            target_comp = self.master.target_component if self.master.target_component > 0 else 1
            self.master.mav.set_position_target_local_ned_send(
                0, target_sys, target_comp,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,  # position only
                x, y, -z,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            self.get_logger().info(f'GOTO waypoint ({x}m, {y}m, {z}m) sent!')
        except Exception as e:
            self.get_logger().warn(f'GOTO failed: {e}')

    def send_heartbeat_ping(self):
        if self.master is not None:
            try:
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
            except Exception:
                pass

    def check_connection_and_run(self):
        if not self.connected:
            if self.master is None:
                self.try_connect()
            if self.master is not None:
                self.send_heartbeat_ping()
                msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
                if msg is not None:
                    self.connected = True
                    self.master.target_system = msg.get_srcSystem()
                    self.master.target_component = msg.get_srcComponent()
                    self.get_logger().info(
                        f'MAVLink heartbeat received from SITL (System {self.master.target_system})!'
                    )
                else:
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
            time.sleep(5)
            self.goto(25.0, 0.0, 3.0)
        else:
            # Periodically re-publish GOTO and ARM to ensure flight progress
            self.send_heartbeat_ping()
            self.goto(25.0, 0.0, 3.0)


def main(args=None):
    rclpy.init(args=args)
    node = MissionCommanderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
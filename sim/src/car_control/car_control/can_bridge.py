import math
import os

import can
import cantools
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def wheel_rpms_from_twist(v, w, track, radius, gear):
    """Differential drive: robot speed (m/s, rad/s) -> motor speeds (rpm)."""
    v_left = v - w * track / 2.0
    v_right = v + w * track / 2.0
    to_rpm = 60.0 / (2.0 * math.pi * radius) * gear
    return v_left * to_rpm, v_right * to_rpm


def twist_from_wheel_rpms(rpm_left, rpm_right, track, radius, gear):
    """Inverse: measured motor speeds (rpm) -> robot speed (m/s, rad/s)."""
    to_mps = 2.0 * math.pi * radius / 60.0 / gear
    v_left, v_right = rpm_left * to_mps, rpm_right * to_mps
    return (v_left + v_right) / 2.0, (v_right - v_left) / track


class CanBridge(Node):
    """Connects ROS 2 to the motor ECUs on the CAN bus.

    /cmd_vel_safe (Twist)  -> WHEEL_CMD (0x100) every 50 ms
    MOTOR_STATUS 0x201/0x202 -> /cmd_vel (Twist) for the Gazebo car
    """

    def __init__(self):
        super().__init__('can_bridge')
        self.declare_parameter('can_interface', 'vcan0')
        self.declare_parameter('dbc_file', os.path.expanduser('~/autonomous-robot-car/can/robot_car.dbc'))
        self.declare_parameter('track_width', 0.181)   # m, distance between the wheels
        self.declare_parameter('wheel_radius', 0.033)  # m
        self.declare_parameter('gear_ratio', 30.0)     # motor turns per wheel turn
        self.declare_parameter('status_timeout', 0.2)  # s without status -> stop
        p = lambda name: self.get_parameter(name).value
        self.track, self.radius, self.gear = p('track_width'), p('wheel_radius'), p('gear_ratio')
        self.status_timeout = p('status_timeout')

        self.db = cantools.database.load_file(p('dbc_file'))
        self.cmd_msg = self.db.get_message_by_name('WHEEL_CMD')
        self.bus = can.Bus(interface='socketcan', channel=p('can_interface'))

        self.target = Twist()
        self.counter = 0
        self.status = {'left': None, 'right': None}     # (time, decoded signals)
        self.ok = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'motor_state', 10)
        self.create_subscription(Twist, 'cmd_vel_safe', self.on_cmd, 10)
        self.create_timer(0.05, self.send_command)      # 20 Hz heartbeat
        self.create_timer(0.01, self.read_bus)          # read status frames
        self.get_logger().info(f"CAN bridge on {p('can_interface')}, gear ratio {self.gear}")

    def on_cmd(self, msg):
        self.target = msg

    def send_command(self):
        left, right = wheel_rpms_from_twist(self.target.linear.x, self.target.angular.z,
                                            self.track, self.radius, self.gear)
        clamp = lambda x: int(max(-5000, min(5000, round(x))))
        data = self.cmd_msg.encode({'SetpointLeft': clamp(left), 'SetpointRight': clamp(right),
                                    'AliveCounter': self.counter})
        self.counter = (self.counter + 1) % 256
        try:
            self.bus.send(can.Message(arbitration_id=self.cmd_msg.frame_id, data=data,
                                      is_extended_id=False))
        except can.CanError as e:
            self.get_logger().warn(f'CAN send failed: {e}', throttle_duration_sec=2.0)

    def read_bus(self):
        now = self.get_clock().now().nanoseconds / 1e9
        while True:
            frame = self.bus.recv(timeout=0.0)
            if frame is None:
                break
            if frame.arbitration_id == 0x201:
                side = 'left'
            elif frame.arbitration_id == 0x202:
                side = 'right'
            else:
                continue
            signals = self.db.decode_message(frame.arbitration_id, frame.data, decode_choices=False)
            self.status[side] = (now, signals)
        self.publish_motion(now)

    def publish_motion(self, now):
        fresh = all(s is not None and now - s[0] < self.status_timeout for s in self.status.values())
        healthy = fresh and all(s[1]['State'] == 0 for s in self.status.values())

        out = Twist()
        if fresh:
            # The car moves with the speed the motors REALLY have, not the requested one
            v, w = twist_from_wheel_rpms(self.status['left'][1]['SpeedRpm'],
                                         self.status['right'][1]['SpeedRpm'],
                                         self.track, self.radius, self.gear)
            out.linear.x, out.angular.z = v, w
        self.cmd_pub.publish(out)

        if healthy != self.ok:
            if healthy:
                self.get_logger().info('Both motor ECUs OK')
            elif not fresh:
                missing = [k for k, s in self.status.items() if s is None or now - s[0] >= self.status_timeout]
                self.get_logger().warn(f'No status from motor ECU(s): {", ".join(missing)} -> car stops')
            else:
                self.get_logger().warn('Motor ECU in WATCHDOG state')
            self.ok = healthy
        self.state_pub.publish(String(data='OK' if healthy else 'FAULT'))


def main():
    rclpy.init()
    node = CanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bus.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
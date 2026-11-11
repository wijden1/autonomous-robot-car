import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String

from car_control.lane_logic import STOP, LaneSteeringController, SignStateMachine


class LaneController(Node):
    """PD lane following plus a state machine for stop and slow signs.

    The logic lives in lane_logic.py (unit-tested); this node only connects
    it to ROS topics.
    """

    def __init__(self):
        super().__init__('lane_controller')
        self.declare_parameter('speed', 0.15)            # m/s
        self.declare_parameter('kp', 1.5)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('max_turn', 1.5)          # rad/s
        self.declare_parameter('stop_time', 3.0)         # s to wait at a stop sign
        self.declare_parameter('slow_time', 5.0)         # s to drive slowly
        self.declare_parameter('ignore_stop_time', 4.0)  # s to ignore the same stop sign
        self.declare_parameter('ignore_slow_time', 5.0)  # s to ignore the same slow sign
        p = lambda name: self.get_parameter(name).value

        self.speed = p('speed')
        self.steering = LaneSteeringController(p('kp'), p('kd'), p('max_turn'))
        self.signs = SignStateMachine(p('stop_time'), p('slow_time'),
                                      p('ignore_stop_time'), p('ignore_slow_time'))

        self.dt = 0.05  # 20 Hz control loop
        self.offset = math.nan
        self.last_msg_time = None
        self.line_lost = True

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        self.state_pub = self.create_publisher(String, 'car_state', 10)
        self.create_subscription(Float32, 'lane/offset', self.on_offset, 10)
        self.create_subscription(String, 'sign', self.on_sign, 10)
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            f'Lane controller running: speed={self.speed}, kp={p("kp")}, kd={p("kd")}')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def log_transition(self, change):
        if change:
            self.get_logger().info(f'State: {change[0]} -> {change[1]}')

    def on_offset(self, msg):
        self.offset = msg.data
        self.last_msg_time = self.now()

    def on_sign(self, msg):
        self.log_transition(self.signs.on_sign(msg.data, self.now()))

    def control_loop(self):
        now = self.now()
        self.log_transition(self.signs.update(now))
        self.state_pub.publish(String(data=self.signs.state))
        cmd = Twist()

        fresh = self.last_msg_time is not None and now - self.last_msg_time < 0.5
        if not fresh or math.isnan(self.offset):
            if not self.line_lost:
                self.get_logger().warn('Line lost - stopping')
                self.line_lost = True
            self.steering.reset()
            self.cmd_pub.publish(cmd)  # zero speed
            return
        if self.line_lost:
            self.get_logger().info('Line found - driving')
            self.line_lost = False

        if self.signs.state == STOP:
            self.steering.reset()
            self.cmd_pub.publish(cmd)  # stand still at the stop sign
            return

        cmd.linear.x = self.speed * self.signs.speed_factor
        cmd.angular.z = self.steering.update(self.offset, self.dt)
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = LaneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String

DRIVE, SLOW, STOP = 'DRIVE', 'SLOW', 'STOP'


class LaneController(Node):
    """PD lane following plus a state machine for stop and slow signs."""

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
        self.speed, self.kp, self.kd = p('speed'), p('kp'), p('kd')
        self.max_turn = p('max_turn')
        self.stop_time, self.slow_time = p('stop_time'), p('slow_time')
        self.ignore_stop_time = p('ignore_stop_time')
        self.ignore_slow_time = p('ignore_slow_time')

        self.dt = 0.05  # 20 Hz control loop
        self.offset = math.nan
        self.last_msg_time = None
        self.prev_error = 0.0
        self.line_lost = True

        self.state = DRIVE
        self.state_until = 0.0
        self.ignore_stop_until = 0.0
        self.ignore_slow_until = 0.0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        self.state_pub = self.create_publisher(String, 'car_state', 10)
        self.create_subscription(Float32, 'lane/offset', self.on_offset, 10)
        self.create_subscription(String, 'sign', self.on_sign, 10)
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            f'Lane controller running: speed={self.speed}, kp={self.kp}, kd={self.kd}')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def set_state(self, new_state, duration=0.0):
        if new_state != self.state:
            self.get_logger().info(f'State: {self.state} -> {new_state}')
        self.state = new_state
        self.state_until = self.now() + duration

    def on_offset(self, msg):
        self.offset = msg.data
        self.last_msg_time = self.get_clock().now()

    def on_sign(self, msg):
        t = self.now()
        if msg.data == 'stop' and self.state != STOP and t > self.ignore_stop_until:
            self.set_state(STOP, self.stop_time)
        elif msg.data == 'slow' and self.state == DRIVE and t > self.ignore_slow_until:
            self.set_state(SLOW, self.slow_time)

    def update_state(self):
        t = self.now()
        if self.state == STOP and t > self.state_until:
            self.ignore_stop_until = t + self.ignore_stop_time
            self.set_state(DRIVE)
        elif self.state == SLOW and t > self.state_until:
            self.ignore_slow_until = t + self.ignore_slow_time
            self.set_state(DRIVE)

    def control_loop(self):
        self.update_state()
        self.state_pub.publish(String(data=self.state))
        cmd = Twist()

        fresh = (self.last_msg_time is not None and
                 (self.get_clock().now() - self.last_msg_time).nanoseconds < 0.5e9)
        if not fresh or math.isnan(self.offset):
            if not self.line_lost:
                self.get_logger().warn('Line lost - stopping')
                self.line_lost = True
            self.prev_error = 0.0
            self.cmd_pub.publish(cmd)  # zero speed
            return
        if self.line_lost:
            self.get_logger().info('Line found - driving')
            self.line_lost = False

        if self.state == STOP:
            self.prev_error = 0.0
            self.cmd_pub.publish(cmd)  # stand still at the stop sign
            return

        # Line on the right (positive offset) -> turn right (negative angular z)
        error = self.offset
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        turn = -(self.kp * error + self.kd * derivative)
        turn = max(-self.max_turn, min(self.max_turn, turn))

        cmd.linear.x = self.speed * (0.5 if self.state == SLOW else 1.0)
        cmd.angular.z = turn
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
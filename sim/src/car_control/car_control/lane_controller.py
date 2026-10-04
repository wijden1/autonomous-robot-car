import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class LaneController(Node):
    """PD controller: turns the lane offset into a steering command."""

    def __init__(self):
        super().__init__('lane_controller')
        self.declare_parameter('speed', 0.15)     # m/s
        self.declare_parameter('kp', 1.5)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('max_turn', 1.5)   # rad/s
        self.speed = self.get_parameter('speed').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.max_turn = self.get_parameter('max_turn').value

        self.dt = 0.05  # 20 Hz control loop
        self.offset = math.nan
        self.last_msg_time = None
        self.prev_error = 0.0
        self.line_lost = True

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        self.create_subscription(Float32, 'lane/offset', self.on_offset, 10)
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f'Lane controller running: speed={self.speed}, kp={self.kp}, kd={self.kd}')

    def on_offset(self, msg):
        self.offset = msg.data
        self.last_msg_time = self.get_clock().now()

    def control_loop(self):
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

        # Line on the right (positive offset) -> turn right (negative angular z)
        error = self.offset
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        turn = -(self.kp * error + self.kd * derivative)
        turn = max(-self.max_turn, min(self.max_turn, turn))

        cmd.linear.x = self.speed
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
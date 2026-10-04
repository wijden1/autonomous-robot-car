import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SafetyNode(Node):
    """Passes driving commands through, but blocks forward driving near obstacles."""

    def __init__(self):
        super().__init__('safety_node')
        self.declare_parameter('stop_distance', 0.3)
        self.stop_distance = self.get_parameter('stop_distance').value

        self.min_range = math.inf
        self.last_scan_time = None
        self.last_cmd = Twist()
        self.blocked = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel_raw', self.on_cmd, 10)
        self.create_subscription(LaserScan, 'front_scan', self.on_scan, 10)
        self.create_timer(0.05, self.publish_safe_cmd)  # 20 Hz

        self.get_logger().info(f'Safety node running, stop distance {self.stop_distance} m')

    def on_scan(self, msg):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        self.min_range = min(valid) if valid else math.inf
        self.last_scan_time = self.get_clock().now()

    def on_cmd(self, msg):
        self.last_cmd = msg

    def publish_safe_cmd(self):
        scan_age_ok = (self.last_scan_time is not None and
                       (self.get_clock().now() - self.last_scan_time).nanoseconds < 0.5e9)
        blocked = (not scan_age_ok) or self.min_range < self.stop_distance

        if blocked != self.blocked:
            if blocked:
                reason = 'no sensor data' if not scan_age_ok else f'obstacle at {self.min_range:.2f} m'
                self.get_logger().warn(f'STOP: {reason}')
            else:
                self.get_logger().info('Path clear')
            self.blocked = blocked

        out = Twist()
        out.linear.x = self.last_cmd.linear.x
        out.angular.z = self.last_cmd.angular.z
        if blocked and out.linear.x > 0.0:
            out.linear.x = 0.0
        self.cmd_pub.publish(out)


def main():
    rclpy.init()
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
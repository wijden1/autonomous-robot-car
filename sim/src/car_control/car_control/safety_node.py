import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from car_control.safety_logic import SafetyMonitor, min_valid_range


class SafetyNode(Node):
    """Passes driving commands through, but blocks forward driving near obstacles.

    The decision itself lives in safety_logic.SafetyMonitor (unit-tested);
    this node only connects it to ROS topics.
    """

    def __init__(self):
        super().__init__('safety_node')
        self.declare_parameter('stop_distance', 0.3)
        self.monitor = SafetyMonitor(stop_distance=self.get_parameter('stop_distance').value)

        self.last_cmd = Twist()
        self.blocked = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel_raw', self.on_cmd, 10)
        self.create_subscription(LaserScan, 'front_scan', self.on_scan, 10)
        self.create_timer(0.05, self.publish_safe_cmd)  # 20 Hz

        self.get_logger().info(f'Safety node running, stop distance {self.monitor.stop_distance} m')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_scan(self, msg):
        self.monitor.on_scan(min_valid_range(msg.ranges, msg.range_min, msg.range_max), self.now())

    def on_cmd(self, msg):
        self.last_cmd = msg

    def publish_safe_cmd(self):
        blocked, reason = self.monitor.evaluate(self.now())

        if blocked != self.blocked:
            if blocked:
                self.get_logger().warn(f'STOP: {reason}')
            else:
                self.get_logger().info('Path clear')
            self.blocked = blocked

        out = Twist()
        out.linear.x, out.angular.z = SafetyMonitor.filter_command(
            self.last_cmd.linear.x, self.last_cmd.angular.z, blocked)
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

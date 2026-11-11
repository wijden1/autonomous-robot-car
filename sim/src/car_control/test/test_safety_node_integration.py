"""Integration test: start the real safety_node and talk to it over ROS 2 topics."""
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


@pytest.mark.launch_test
def generate_test_description():
    safety_node = launch_ros.actions.Node(
        package='car_control',
        executable='safety_node',
        parameters=[{'stop_distance': 0.3}],
    )
    return launch.LaunchDescription([safety_node, launch_testing.actions.ReadyToTest()])


class TestSafetyNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('safety_node_tester')
        self.received = []
        self.node.create_subscription(Twist, 'cmd_vel', self.received.append, 10)
        self.cmd_pub = self.node.create_publisher(Twist, 'cmd_vel_raw', 10)
        self.scan_pub = self.node.create_publisher(LaserScan, 'front_scan', 10)

    def tearDown(self):
        self.node.destroy_node()

    def send_for(self, seconds, distance=None):
        """Send 'drive 0.15 m/s' (and a scan, unless distance is None) at about 20 Hz."""
        cmd = Twist()
        cmd.linear.x = 0.15
        end = time.time() + seconds
        while time.time() < end:
            if distance is not None:
                scan = LaserScan()
                scan.range_min = 0.05
                scan.range_max = 4.0
                scan.ranges = [distance] * 5
                self.scan_pub.publish(scan)
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def speeds_during(self, seconds, distance=None):
        """Warm up first, then collect the speeds the safety node publishes."""
        self.send_for(1.5, distance)      # time for discovery and for the node to react
        self.received.clear()
        self.send_for(seconds, distance)
        self.assertGreater(len(self.received), 5, 'safety_node published nothing on /cmd_vel')
        return [msg.linear.x for msg in self.received]

    def test_clear_path_passes_the_command(self):
        speeds = self.speeds_during(1.0, distance=2.0)
        for v in speeds:
            self.assertAlmostEqual(v, 0.15, places=3)

    def test_obstacle_stops_the_car(self):
        speeds = self.speeds_during(1.0, distance=0.2)
        self.assertEqual(max(speeds), 0.0)

    def test_silent_sensor_stops_the_car(self):
        speeds = self.speeds_during(1.0, distance=None)
        self.assertEqual(max(speeds), 0.0)


@launch_testing.post_shutdown_test()
class TestSafetyNodeShutdown(unittest.TestCase):

    def test_node_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

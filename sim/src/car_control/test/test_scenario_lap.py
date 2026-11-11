"""Scenario test: start the full simulation and check the car's behaviour on one lap.

World layout (see make_track.py): the car spawns at (-1, -1) heading +x on the
bottom straight, passes the stop sign (x=1), drives the right curve, then the
top straight (y=1) in -x direction past the slow sign, and must stop before
the obstacle at (-1, 1).

/odom starts at (0, 0) where the car spawns, so world = odom + (-1, -1).
"""
import math
import os
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
from std_msgs.msg import String

SPAWN_X, SPAWN_Y = -1.0, -1.0
OBSTACLE_X = -1.0           # obstacle_1 centre, size 0.2 m
SCENARIO_TIMEOUT = 300.0    # s of wall time for the whole lap


@pytest.mark.launch_test
def generate_test_description():
    sim = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('car_description'), 'launch', 'sim.launch.py')),
        launch_arguments={'auto': 'true', 'use_can': 'false'}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


class TestLapScenario(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('scenario_tester')
        self.x = self.y = self.speed = 0.0
        self.have_odom = False
        self.state = 'UNKNOWN'
        self.node.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.node.create_subscription(String, 'car_state', self.on_state, 10)

    def tearDown(self):
        self.node.destroy_node()

    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x + SPAWN_X
        self.y = msg.pose.pose.position.y + SPAWN_Y
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.have_odom = True

    def on_state(self, msg):
        self.state = msg.data

    def wait_until(self, condition, timeout, what):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if condition():
                return
        self.fail(f'Timeout after {timeout:.0f} s waiting for: {what} '
                  f'(state={self.state}, x={self.x:.2f}, y={self.y:.2f}, v={self.speed:.3f})')

    def max_speed_for(self, seconds):
        top = 0.0
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            top = max(top, self.speed)
        return top

    def test_full_lap(self):
        start = time.time()
        remaining = lambda: max(1.0, SCENARIO_TIMEOUT - (time.time() - start))

        # 1. The car starts driving by itself
        self.wait_until(lambda: self.have_odom and self.speed > 0.05, 60, 'car starts driving')
        print(f'[scenario] driving, at x={self.x:.2f} y={self.y:.2f}')

        # 2. Stop sign on the bottom straight: state STOP and really standing still
        self.wait_until(lambda: self.state == 'STOP', remaining(), 'STOP at the stop sign')
        self.assertLess(self.y, -0.5, 'STOP should happen on the bottom straight')
        self.assertTrue(-0.5 < self.x < 1.3, f'STOP at unexpected x={self.x:.2f}')
        self.max_speed_for(0.5)   # give the car time to brake
        self.assertLess(self.max_speed_for(1.5), 0.02, 'car did not stand still at the stop sign')
        print(f'[scenario] stopped at stop sign, x={self.x:.2f} y={self.y:.2f}')

        # 3. Slow sign on the top straight
        self.wait_until(lambda: self.state == 'SLOW', remaining(), 'SLOW at the slow sign')
        self.assertGreater(self.y, 0.5, 'SLOW should happen on the top straight')
        print(f'[scenario] slow zone, x={self.x:.2f} y={self.y:.2f}')

        # 4. Obstacle: the car stops on the top straight and stays stopped
        self.wait_until(lambda: self.state == 'DRIVE' and self.speed < 0.01 and self.x < 0.3,
                        remaining(), 'standing still in front of the obstacle')
        self.assertLess(self.max_speed_for(3.0), 0.02, 'car did not stay stopped at the obstacle')
        gap = self.x - (OBSTACLE_X + 0.1)   # car centre to the obstacle's near face
        print(f'[scenario] stopped before obstacle, x={self.x:.2f}, gap to box {gap:.2f} m')
        self.assertGreater(self.y, 0.5, 'obstacle stop should be on the top straight')
        self.assertGreater(gap, 0.2, 'car got too close to the obstacle')
        self.assertLess(gap, 0.8, 'car stopped far too early')

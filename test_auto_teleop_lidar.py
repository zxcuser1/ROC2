#!/usr/bin/env python3
"""Unit tests for the current ROS 2 auto_teleop_lidar node.

Put this file in ~/ros2_ws and run:
  source /opt/ros/$ROS_DISTRO/setup.bash
  source ~/ros2_ws/install/setup.bash
  cd ~/ros2_ws
  python3 test_auto_teleop_lidar.py -v

Tests instantiate the real node but intercept publish(), so no velocity
commands are sent to the robot.
"""

import importlib.util
import math
import unittest
from importlib.machinery import SourceFileLoader
from pathlib import Path

import rclpy
from sensor_msgs.msg import LaserScan


def load_robot_class():
    root = Path(__file__).resolve().parent.parent
    matches = sorted(
        p for p in root.rglob('*')
        if p.is_file() and p.name in ('auto_teleop_lidar', 'auto_teleop_lidar.py')
    )
    if len(matches) != 1:
        raise RuntimeError(
            f'Expected one auto_teleop_lidar script in {root}, found: {matches}. '
            'Remove duplicates or specify the path in this test file.'
        )
    loader = SourceFileLoader('auto_lidar_under_test', str(matches[0]))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module.auto_teleop_lidar


class TestAutoLidar(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.RobotClass = load_robot_class()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.robot = self.RobotClass()
        self.commands = []
        # Nothing is sent to /controller/reference during tests.
        self.robot.publish = lambda: self.commands.append((
            self.robot.target_linear_vel_x,
            self.robot.target_linear_vel_y,
            self.robot.target_angular_vel,
        ))

    def tearDown(self):
        self.robot.destroy_node()

    def set_scan(self, points):
        now = self.robot.get_clock().now().nanoseconds
        self.robot.scan = (list(points), now)

    @staticmethod
    def wall_on_side(side):
        # Dense arc 0.45 m away, from +10..+90 (left) or -10..-90 (right).
        return [
            (0.45 * math.cos(math.radians(deg) * side),
             0.45 * math.sin(math.radians(deg) * side))
            for deg in range(10, 91, 5)
        ]

    def test_front_blocked(self):
        clearance = self.robot.path_clearance([(0.40, 0.0)], 0.0, 0.9)
        self.assertAlmostEqual(clearance, 0.40)

    def test_obstacle_outside_robot_width(self):
        clearance = self.robot.path_clearance([(0.40, 0.40)], 0.0, 0.9)
        self.assertAlmostEqual(clearance, 0.9)

    def test_empty_path_moves_forward(self):
        self.set_scan([])  # All lidar rays report infinity / no obstacles.
        self.robot.update_velocity()
        x, y, angular = self.commands[-1]
        self.assertGreater(x, 0)
        self.assertEqual(y, 0)
        self.assertEqual(angular, 0)

    def test_obstacle_on_left_prefers_right(self):
        self.set_scan([(0.4, 0.0)] + self.wall_on_side(1))
        self.robot.update_velocity()
        self.assertEqual(self.robot.turn_direction, -1)
        self.assertLess(self.commands[-1][2], 0)
        self.assertEqual(self.commands[-1][0], 0)

    def test_obstacle_on_right_prefers_left(self):
        self.set_scan([(0.4, 0.0)] + self.wall_on_side(-1))
        self.robot.update_velocity()
        self.assertEqual(self.robot.turn_direction, 1)
        self.assertGreater(self.commands[-1][2], 0)
        self.assertEqual(self.commands[-1][0], 0)

    def test_turn_side_remains_after_free_path(self):
        # First obstacle: right is preferred.
        self.set_scan([(0.4, 0.0)] + self.wall_on_side(1))
        self.robot.update_velocity()
        self.assertEqual(self.robot.turn_direction, -1)

        # Straight segment must not reset chosen direction.
        self.set_scan([])
        self.robot.update_velocity()
        self.assertGreater(self.commands[-1][0], 0)
        self.assertEqual(self.robot.turn_direction, -1)

        # New obstacle with *opposite* open side: direction stays right.
        self.set_scan([(0.4, 0.0)] + self.wall_on_side(-1))
        self.robot.update_velocity()
        self.assertEqual(self.robot.turn_direction, -1)
        self.assertLess(self.commands[-1][2], 0)

    def test_no_scan_stops(self):
        self.robot.scan = None
        self.robot.update_velocity()
        self.assertEqual(self.commands[-1], (0.0, 0.0, 0.0))

    def test_stale_scan_stops(self):
        now = self.robot.get_clock().now().nanoseconds
        self.robot.scan = ([], now - 2_000_000_000)
        self.robot.update_velocity()
        self.assertEqual(self.commands[-1], (0.0, 0.0, 0.0))

    def test_lidar_angle_sign(self):
        msg = LaserScan()
        msg.angle_min = 0.0
        msg.angle_increment = 0.015747331082820892
        msg.range_min = 0.05
        msg.range_max = 7.0
        msg.ranges = [float('inf')] * 400
        msg.ranges[0] = 0.4      # Front
        msg.ranges[100] = 0.5    # Approx. 90 degrees left
        msg.ranges[300] = 0.6    # Approx. 270 degrees right

        self.robot.lidar_callback(msg)
        self.assertIsNotNone(self.robot.scan)
        points, _ = self.robot.scan
        self.assertEqual(len(points), 3)
        self.assertAlmostEqual(points[0][0], 0.4, places=2)
        self.assertGreater(points[1][1], 0.49)
        self.assertLess(points[2][1], -0.59)


if __name__ == '__main__':
    unittest.main(verbosity=2)

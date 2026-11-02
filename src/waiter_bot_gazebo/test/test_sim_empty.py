# Copyright (c) 2026 Pavan Patil
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""N1: the robot spawns in the empty world, publishes its sensors and drives as commanded."""
import math
import os
import random
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
import tf2_ros

# Private ROS domain and Gazebo partition, so the test never meets another running simulation.
os.environ['ROS_DOMAIN_ID'] = str(random.randint(100, 200))
os.environ['GZ_PARTITION'] = f'waiter_bot_test_{os.getpid()}'


def generate_test_description():
    """Launch the simulator headless with the camera off, and tell the test it can start."""
    share = get_package_share_directory('waiter_bot_gazebo')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sim.launch.py')),
        launch_arguments={'headless': 'true', 'camera': 'false'}.items())
    return LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


def yaw_of(orientation):
    """Return the heading angle of a quaternion that only rotates about the vertical axis."""
    return math.atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y),
                      1 - 2 * (orientation.y ** 2 + orientation.z ** 2))


def angle_difference(a, b):
    """Return the signed difference a - b wrapped to (-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class TestEmptyWorld(unittest.TestCase):
    """Checks that need the running simulation, in the order a user would meet them."""

    @classmethod
    def setUpClass(cls):
        """Start a probe node that listens to the robot's topics and sends velocity commands."""
        rclpy.init()
        cls.node = rclpy.create_node('sim_probe')
        cls.scans, cls.odoms, cls.truth, cls.clocks = [], [], [], []
        for msg_type, topic, store in ((LaserScan, 'scan', cls.scans),
                                       (Odometry, 'odom', cls.odoms),
                                       (PoseStamped, 'ground_truth/pose', cls.truth),
                                       (Clock, 'clock', cls.clocks)):
            cls.node.create_subscription(msg_type, topic, store.append, qos_profile_sensor_data)
        cls.cmd = cls.node.create_publisher(Twist, 'cmd_vel', 10)
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node)

    @classmethod
    def tearDownClass(cls):
        """Shut the probe down."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, seconds):
        """Let the probe receive messages for a while."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait_for(self, condition, timeout, what):
        """Wait until a condition holds, or fail the test with a message naming what is missing."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if condition():
                return
            self.spin(0.1)
        self.fail(f'timed out waiting for {what}')

    def sim_now(self):
        """Return the simulated time of the latest ground-truth pose, in seconds."""
        stamp = self.truth[-1].header.stamp
        return stamp.sec + stamp.nanosec * 1e-9

    def drive(self, linear, angular, seconds):
        """Command a velocity for some simulated seconds and return what truth and odometry saw."""
        self.spin(1.0)
        truth0, odom0, start = self.truth[-1], self.odoms[-1], self.sim_now()
        twist = Twist()
        twist.linear.x, twist.angular.z = linear, angular
        while self.sim_now() - start < seconds:
            self.cmd.publish(twist)
            self.spin(0.05)
        self.cmd.publish(Twist())
        self.spin(2.0)
        truth1, odom1 = self.truth[-1], self.odoms[-1]

        def moved(a, b):
            return math.hypot(b.position.x - a.position.x, b.position.y - a.position.y)

        return {
            'true_distance': moved(truth0.pose, truth1.pose),
            'odom_distance': moved(odom0.pose.pose, odom1.pose.pose),
            'true_yaw': angle_difference(yaw_of(truth1.pose.orientation),
                                         yaw_of(truth0.pose.orientation)),
            'odom_yaw': angle_difference(yaw_of(odom1.pose.pose.orientation),
                                         yaw_of(odom0.pose.pose.orientation)),
        }

    def test_1_sensors_clock_and_tf(self):
        """N1: the lidar runs at 10 Hz, time advances, and the frames connect within a minute."""
        self.wait_for(lambda: self.truth and self.odoms and len(self.scans) > 5, 90,
                      'the robot to spawn and publish')
        self.spin(3.0)
        stamps = [s.header.stamp.sec + s.header.stamp.nanosec * 1e-9 for s in self.scans[-30:]]
        rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        self.assertAlmostEqual(rate, 10.0, delta=2.0, msg='lidar rate in simulated time')
        self.assertEqual(self.scans[-1].header.frame_id, 'lidar_link')
        self.assertGreater(len(self.scans[-1].ranges), 300)
        self.assertGreater(self.clocks[-1].clock.sec, self.clocks[0].clock.sec)
        self.wait_for(lambda: self.tf_buffer.can_transform('odom', 'lidar_link',
                                                           rclpy.time.Time()),
                      30, 'the TF chain odom -> base_footprint -> lidar_link')
        self.assertAlmostEqual(self.truth[-1].pose.position.z, 0.0, delta=0.02,
                               msg='the robot rests on the floor')

    def test_2_drives_straight_as_odometry_says(self):
        """The robot moves forward in a straight line and odometry agrees within 5 percent."""
        result = self.drive(0.3, 0.0, 3.0)
        self.assertGreater(result['true_distance'], 0.7)
        self.assertAlmostEqual(result['true_distance'], result['odom_distance'],
                               delta=0.05 * result['odom_distance'])
        self.assertLess(abs(result['true_yaw']), 0.05)

    def test_3_turns_as_odometry_says(self):
        """The robot turns on the spot and the true turn is within 10 percent of odometry's."""
        result = self.drive(0.0, 1.0, 3.0)
        self.assertGreater(result['odom_yaw'], 2.0)
        self.assertAlmostEqual(result['true_yaw'], result['odom_yaw'],
                               delta=0.10 * result['odom_yaw'])
        self.assertLess(result['true_distance'], 0.05)

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

"""N2 to N5: Nav2 becomes active, AMCL converges, every café location is reached, cancel works."""
from collections import deque
import math
import os
import time
import unittest

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import (qos_profile_sensor_data, QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)
import yaml

# Private ROS domain and Gazebo partition, so the test never meets another running simulation.
os.environ['ROS_DOMAIN_ID'] = str(os.getpid() % 100 + 1)
os.environ['GZ_PARTITION'] = f'waiter_bot_nav_test_{os.getpid()}'

NAV2_NODES = ['amcl', 'map_server', 'controller_server', 'planner_server', 'smoother_server',
              'route_server', 'behavior_server', 'bt_navigator', 'waypoint_follower',
              'velocity_smoother', 'collision_monitor', 'docking_server']
POSITION_TOLERANCE = 0.30
YAW_TOLERANCE = 0.35
TRIP_LIMIT_SEC = 60.0
ROUNDS = 3


def generate_test_description():
    """Launch the café simulation and Nav2 headless, and tell the test it can start."""
    gazebo = get_package_share_directory('waiter_bot_gazebo')
    navigation = get_package_share_directory('waiter_bot_navigation')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'sim.launch.py')),
        launch_arguments={'world': 'cafe', 'headless': 'true', 'camera': 'false',
                          'ground_truth': 'true'}.items())
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation, 'launch', 'navigation.launch.py')))
    return LaunchDescription([sim, nav, launch_testing.actions.ReadyToTest()])


def yaw_of(q):
    """Return the heading angle of a quaternion that only rotates about the vertical axis."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def angle_difference(a, b):
    """Return the signed difference a - b wrapped to (-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class TestNavigation(unittest.TestCase):
    """Navigation checks in the order a delivery needs them."""

    @classmethod
    def setUpClass(cls):
        """Start a probe node, load the café locations and listen to the robot's poses."""
        rclpy.init()
        cls.node = rclpy.create_node('nav_probe')
        path = os.path.join(get_package_share_directory('waiter_bot_core'), 'config', 'cafe.yaml')
        with open(path) as f:
            cls.locations = yaml.safe_load(f)['butler_task_manager']['ros__parameters'][
                'locations']
        cls.truth, cls.amcl = deque(maxlen=200), deque(maxlen=20)
        cls.node.create_subscription(PoseStamped, 'ground_truth/pose',
                                     lambda m: cls.truth.append(m), qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        cls.node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                     lambda m: cls.amcl.append(m), latched)
        cls.initial = cls.node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        cls.client = ActionClient(cls.node, NavigateToPose, 'navigate_to_pose')
        # Each test needs the robot publishing and the action server up, whatever ran before it.
        end = time.monotonic() + 120
        while time.monotonic() < end and not (cls.truth and cls.client.server_is_ready()):
            rclpy.spin_once(cls.node, timeout_sec=0.1)
        if not (cls.truth and cls.client.server_is_ready()):
            raise RuntimeError('the robot or navigate_to_pose did not come up within 120 s')

    @classmethod
    def tearDownClass(cls):
        """Shut the probe down."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, seconds):
        """Let the probe receive messages for a while."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_for(self, condition, timeout, what):
        """Wait until a condition holds, or fail naming what is missing."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if condition():
                return
            self.spin(0.1)
        self.fail(f'timed out waiting for {what}')

    def lifecycle_state(self, name):
        """Return the lifecycle state id of a Nav2 node, or None if it does not answer yet."""
        client = self.node.create_client(GetState, f'/{name}/get_state')
        try:
            if not client.wait_for_service(timeout_sec=0.5):
                return None
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            return future.result().current_state.id if future.result() else None
        finally:
            self.node.destroy_client(client)

    def settle(self):
        """Wait until the robot has stopped (under 1 cm in 0.5 s), as the success test needs."""
        for _ in range(40):
            a = self.truth[-1].pose.position
            self.spin(0.5)
            b = self.truth[-1].pose.position
            if math.hypot(b.x - a.x, b.y - a.y) < 0.01:
                return
        self.fail('the robot never came to rest')

    def goal_for(self, name):
        """Build a NavigateToPose goal for a named café location."""
        pose = self.locations[name]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = pose['x']
        goal.pose.pose.position.y = pose['y']
        goal.pose.pose.orientation.z = math.sin(pose['yaw'] / 2)
        goal.pose.pose.orientation.w = math.cos(pose['yaw'] / 2)
        return goal

    def send(self, name):
        """Send a goal and return its handle once Nav2 has accepted it."""
        future = self.client.send_goal_async(self.goal_for(name))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        handle = future.result()
        self.assertIsNotNone(handle, f'no answer to the goal for {name}')
        self.assertTrue(handle.accepted, f'goal for {name} was rejected')
        return handle

    def error_to(self, name):
        """Return the true distance and heading error between the robot and a location."""
        pose = self.locations[name]
        true = self.truth[-1].pose
        return (math.hypot(true.position.x - pose['x'], true.position.y - pose['y']),
                abs(angle_difference(yaw_of(true.orientation), pose['yaw'])))

    def test_1_lifecycle_nodes_active(self):
        """N2: every Nav2 lifecycle node reaches `active`."""
        end = time.monotonic() + 60
        states = {}
        while time.monotonic() < end:
            states = {n: self.lifecycle_state(n) for n in NAV2_NODES}
            if all(s == State.PRIMARY_STATE_ACTIVE for s in states.values()):
                return
            self.spin(1.0)
        self.fail(f'not all active: {states}')

    def test_2_amcl_converges(self):
        """N3: the AMCL pose has a small covariance and sits near the true pose."""
        # Tell AMCL the true pose with a loose covariance, drive a short trip so it has motion to
        # correct against, then require a fresh, tight estimate near the true pose.
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose = self.truth[-1].pose
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        self.amcl.clear()
        self.initial.publish(msg)
        self.wait_for(lambda: self.amcl, 30, 'a fresh AMCL pose')
        self.assertGreater(self.amcl[-1].pose.covariance[0], 0.1, 'the loose start was ignored')
        handle = self.send('table1')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=TRIP_LIMIT_SEC)
        self.assertTrue(result.done(), 'the convergence trip did not finish')
        self.wait_for(lambda: self.amcl[-1].pose.covariance[0] < 0.1
                      and self.amcl[-1].pose.covariance[7] < 0.1, 15, 'a converged AMCL pose')
        est, true = self.amcl[-1].pose.pose.position, self.truth[-1].pose.position
        self.assertLess(math.hypot(est.x - true.x, est.y - true.y), 0.3)

    def test_3_trips_reach_every_location(self):
        """N4: three rounds of kitchen, each table and home all meet the success definition."""
        failures = []
        route = ['kitchen', 'table1', 'table2', 'table3', 'home']
        for round_number in range(ROUNDS):
            for name in route:
                start = time.monotonic()
                handle = self.send(name)
                result = handle.get_result_async()
                rclpy.spin_until_future_complete(self.node, result, timeout_sec=TRIP_LIMIT_SEC)
                took = time.monotonic() - start
                if not result.done():
                    handle.cancel_goal_async()
                    self.spin(2.0)
                    failures.append(f'round {round_number} {name}: over {TRIP_LIMIT_SEC} s')
                    continue
                self.settle()
                distance, heading = self.error_to(name)
                status = result.result().status
                print(f'round {round_number} {name}: status {status} {took:.1f}s '
                      f'distance {distance:.3f} heading {heading:.3f}')
                if (status != GoalStatus.STATUS_SUCCEEDED or distance > POSITION_TOLERANCE
                        or heading > YAW_TOLERANCE):
                    failures.append(f'round {round_number} {name}: status {status}, '
                                    f'distance {distance:.2f}, heading {heading:.2f}')
        self.assertEqual(failures, [])

    def test_4_cancel_stops_the_robot(self):
        """N5: cancelling a goal in motion stops the robot within 2 s."""
        start = self.truth[-1].pose.position
        handle = self.send('kitchen')
        self.spin(4.0)
        moving = self.truth[-1].pose.position
        self.assertGreater(math.hypot(moving.x - start.x, moving.y - start.y), 0.2, 'never moved')
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, cancel, timeout_sec=5.0)
        self.assertEqual(cancel.result().return_code, 0, 'the cancel was not accepted')
        self.spin(2.0)
        a = self.truth[-1].pose.position
        self.spin(1.0)
        b = self.truth[-1].pose.position
        self.assertLess(math.hypot(b.x - a.x, b.y - a.y), 0.02, 'still moving 2 s after cancel')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=5.0)
        self.assertEqual(result.result().status, GoalStatus.STATUS_CANCELED)

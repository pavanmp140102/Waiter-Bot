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

"""Node tests TN1 to TN7: the task manager against a fake Nav2 server, with no simulator."""
import os
import random
import threading
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from waiter_bot_core.task_manager import ButlerTaskManager
from waiter_bot_interfaces.action import DeliverOrders
from waiter_bot_interfaces.msg import ButlerState
from waiter_bot_interfaces.srv import CancelTable, Confirm

POSES = {'home': (0.0, 0.0, 0.0), 'kitchen': (4.0, 0.0, 0.0), 'table1': (1.5, 4.0, 1.57),
         'table2': (-1.5, 4.0, 1.57), 'table3': (-4.5, 4.0, 1.57)}
HOME, KITCHEN, TABLE1 = (0.0, 0.0), (4.0, 0.0), (1.5, 4.0)
NO_WAITING = {'require_kitchen_confirmation': False, 'require_table_confirmation': False}


def parameters(**overrides):
    """Build the café parameters as a node would load them, with some values replaced."""
    values = {'tables': ['table1', 'table2', 'table3'], 'confirmation_timeout_sec': 30.0}
    for name, (x, y, yaw) in POSES.items():
        values.update({f'locations.{name}.x': x, f'locations.{name}.y': y,
                       f'locations.{name}.yaw': yaw})
    values.update(overrides)
    return [Parameter(name, value=value) for name, value in values.items()]


def until(predicate, timeout=15.0):
    """Wait for a condition, failing the test if it never becomes true."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError('timed out waiting for a condition')


def wait(future, timeout=15.0):
    """Wait for a ROS future to finish and return its result."""
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    assert event.wait(timeout), 'timed out waiting for a future'
    return future.result()


class FakeNav(Node):
    """Stands in for Nav2: answers each NavigateToPose goal as the script says."""

    def __init__(self):
        """Offer the navigate_to_pose action, recording every goal."""
        super().__init__('fake_nav')
        self.goals = []
        self.script = []
        ActionServer(self, NavigateToPose, 'navigate_to_pose', self._execute,
                     cancel_callback=lambda _: CancelResponse.ACCEPT,
                     callback_group=ReentrantCallbackGroup())

    def _execute(self, goal_handle):
        """Succeed, fail or hold a goal until it is cancelled, as scripted ('ok' by default)."""
        position = goal_handle.request.pose.pose.position
        self.goals.append((round(position.x, 2), round(position.y, 2)))
        mode = self.script.pop(0) if self.script else 'ok'
        end = time.monotonic() + (30.0 if mode == 'hold' else 0.05)
        while time.monotonic() < end:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.01)
        if mode == 'fail':
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return NavigateToPose.Result()


class System:
    """A task manager, a fake Nav2 and a client, all spinning in one process."""

    def __init__(self, params):
        """Start the nodes and wait until the client can reach the task manager and Nav2."""
        self.butler = ButlerTaskManager(parameter_overrides=params)
        self.nav = FakeNav()
        self.client = Node('test_client')
        self.states, self.feedback = [], []
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.client.create_subscription(ButlerState, '/butler/state', self.states.append,
                                        latched)
        self.clock = self.client.create_publisher(Clock, '/clock', 10)
        self.action = ActionClient(self.client, DeliverOrders, '/butler/deliver_orders')
        self.confirm_srv = self.client.create_client(Confirm, '/butler/confirm')
        self.cancel_srv = self.client.create_client(CancelTable, '/butler/cancel_table')
        self.executor = MultiThreadedExecutor(num_threads=12)
        for node in (self.butler, self.nav, self.client):
            self.executor.add_node(node)
        threading.Thread(target=self.executor.spin, daemon=True).start()
        assert self.action.wait_for_server(15.0)
        assert self.butler._nav.wait_for_server(15.0)
        assert self.confirm_srv.wait_for_service(15.0)
        assert self.cancel_srv.wait_for_service(15.0)

    def send(self, *tables):
        """Place an order and return its goal handle, or None if it was rejected."""
        goal = DeliverOrders.Goal(tables=list(tables))
        handle = wait(self.action.send_goal_async(
            goal, feedback_callback=lambda m: self.feedback.append(m.feedback)))
        return handle if handle.accepted else None

    def result(self, handle):
        """Wait for the task to end and return the action result wrapper."""
        return wait(handle.get_result_async())

    def confirm(self, location):
        """Call the confirm service and return its response."""
        return wait(self.confirm_srv.call_async(Confirm.Request(location=location)))

    def cancel_table(self, table):
        """Call the cancel-table service and return its response."""
        return wait(self.cancel_srv.call_async(CancelTable.Request(table=table)))

    def wait_state(self, name):
        """Wait until the latest published state has this name."""
        until(lambda: self.states and self.states[-1].state == name)

    def set_time(self, seconds):
        """Publish simulated time, for a task manager running on /clock."""
        for _ in range(3):
            self.clock.publish(Clock(clock=rclpy.time.Time(seconds=seconds).to_msg()))
            time.sleep(0.05)

    def close(self):
        """Stop spinning and tear the nodes down."""
        self.butler.shutdown()
        self.executor.shutdown()
        for node in (self.butler, self.nav, self.client):
            node.destroy_node()


@pytest.fixture(scope='module', autouse=True)
def ros():
    """Initialise ROS on a private domain so these tests never meet a real system."""
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(100, 200))
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def system():
    """Return a factory that starts a `System` and closes it after the test."""
    started = []

    def make(**overrides):
        started.append(System(parameters(**overrides)))
        return started[-1]
    yield make
    for item in started:
        item.close()


def test_tn1_missing_location_stops_startup():
    """TN1: a table without a pose is named in the error before anything runs."""
    params = [p for p in parameters(tables=['table1', 'table9']) if 'table2' not in p.name]
    with pytest.raises(ValueError, match='table9'):
        ButlerTaskManager(parameter_overrides=params)


def test_tn2_goal_runs_to_completion(system):
    """TN2: an order is accepted, progress is reported, and the result arrives."""
    s = system(**NO_WAITING)
    assert s.send('table9') is None
    handle = s.send('table1')
    result = s.result(handle)
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.outcome == 'completed' and result.result.delivered == ['table1']
    assert s.nav.goals == [KITCHEN, TABLE1, HOME]
    states = [f.state for f in s.feedback]
    assert 'to_kitchen' in states and 'to_table' in states and 'to_home' in states


def test_tn3_services_reach_the_state_machine(system):
    """TN3: confirm and cancel-table behave as in the logic tests (R3b, R7)."""
    s = system()
    handle = s.send('table1', 'table2')
    s.wait_state('at_kitchen')
    wrong = s.confirm('table1')
    assert not wrong.accepted and 'kitchen' in wrong.message
    assert s.confirm('kitchen').accepted
    s.wait_state('at_table')
    assert s.cancel_table('table2').accepted
    assert s.confirm('table1').accepted
    result = s.result(handle).result
    assert result.outcome == 'partial'
    assert result.delivered == ['table1'] and result.cancelled == ['table2']


def test_tn4_state_topic_is_transient_local(system):
    """TN4: a subscriber that starts late still receives the latest state."""
    s = system()
    late = []
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    s.client.create_subscription(ButlerState, '/butler/state', late.append, latched)
    until(lambda: late)
    assert late[-1].state == 'idle' and not late[-1].food_onboard


def test_tn5_navigation_failure_ends_in_error(system):
    """TN5: when Nav2 keeps failing the task aborts and the state says why."""
    s = system(nav_max_retries=0, **NO_WAITING)
    s.nav.script = ['fail', 'fail']
    result = s.result(s.send('table1'))
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.outcome == 'nav_failed' and s.states[-1].state == 'error'
    assert len(s.nav.goals) == 2


def test_tn6_cancel_during_navigation_returns_the_food(system):
    """TN6: cancelling on the way to a table cancels that trip and returns via the kitchen."""
    s = system(**NO_WAITING)
    s.nav.script = ['ok', 'hold']
    handle = s.send('table1')
    until(lambda: len(s.nav.goals) == 2)
    wait(handle.cancel_goal_async())
    result = s.result(handle)
    assert result.status == GoalStatus.STATUS_CANCELED
    assert result.result.outcome == 'cancelled'
    assert s.nav.goals == [KITCHEN, TABLE1, KITCHEN, HOME]


def test_tn7_timeout_follows_simulated_time(system):
    """TN7: with a paused /clock the timeout does not fire; advancing the clock fires it."""
    s = system(confirmation_timeout_sec=5.0, use_sim_time=True)
    s.set_time(100)
    handle = s.send('table1')
    s.wait_state('at_kitchen')
    time.sleep(1.5)
    assert s.states[-1].state == 'at_kitchen'
    s.set_time(106)
    result = s.result(handle)
    assert result.result.outcome == 'kitchen_timeout'

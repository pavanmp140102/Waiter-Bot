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

"""
Butler task manager node: connects ROS (action, services, Nav2) to the `ButlerFsm`.

The node owns no delivery rules. It turns ROS callbacks into events for the state machine and
carries out the commands that come back: drive to a place, start or stop a timer, finish the
task. All state-machine calls are made under one lock, and no callback ever blocks on a future.
"""
import threading

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from waiter_bot_core.butler_fsm import (
    ButlerFsm, Config, Finish, Navigate, StartTimer, StopTimer)
from waiter_bot_core.locations import LocationRegistry
from waiter_bot_interfaces.action import DeliverOrders
from waiter_bot_interfaces.msg import ButlerState
from waiter_bot_interfaces.srv import CancelTable, Confirm

POSE_KEYS = ('x', 'y', 'yaw')


class ButlerTaskManager(Node):
    """Accepts delivery orders, drives them through Nav2 and reports progress."""

    def __init__(self, **node_kwargs):
        """Load the parameters and create the action server, services and Nav2 client."""
        super().__init__('butler_task_manager', **node_kwargs)
        self._lock = threading.Lock()
        self._fsm, self._locations = self._load_config()
        self._frame = self.declare_parameter('map_frame', 'map').value
        self._done = threading.Event()
        self._goal_handle = None
        self._outcome = None
        self._nav_handle = None
        self._nav_token = 0
        self._timer = None
        self._tick = None
        self._timer_end = None

        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                 callback_group=ReentrantCallbackGroup())
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(ButlerState, '/butler/state', latched)
        self._server = ActionServer(
            self, DeliverOrders, '/butler/deliver_orders', self._execute,
            goal_callback=self._on_goal, cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup())
        service_group = MutuallyExclusiveCallbackGroup()
        self.create_service(Confirm, '/butler/confirm', self._on_confirm,
                            callback_group=service_group)
        self.create_service(CancelTable, '/butler/cancel_table', self._on_cancel_table,
                            callback_group=service_group)
        with self._lock:
            self._publish()

    # -- configuration ---------------------------------------------------------------------

    def _load_config(self):
        """Read the parameters and fail at start-up if a location is missing or incomplete."""
        home = self.declare_parameter('home', 'home').value
        kitchen = self.declare_parameter('kitchen', 'kitchen').value
        tables = tuple(self.declare_parameter('tables', ['table1', 'table2', 'table3']).value)
        config = Config(
            tables=tables, home=home, kitchen=kitchen,
            confirmation_timeout_sec=self.declare_parameter(
                'confirmation_timeout_sec', 20.0).value,
            require_kitchen_confirmation=self.declare_parameter(
                'require_kitchen_confirmation', True).value,
            require_table_confirmation=self.declare_parameter(
                'require_table_confirmation', True).value,
            kitchen_return_dwell_sec=self.declare_parameter(
                'kitchen_return_dwell_sec', 0.0).value,
            nav_max_retries=self.declare_parameter('nav_max_retries', 2).value)
        raw = {}
        untyped = ParameterDescriptor(dynamic_typing=True)
        for name in dict.fromkeys((home, kitchen, *tables)):
            values = {}
            for key in POSE_KEYS:
                value = self.declare_parameter(
                    f'locations.{name}.{key}', None, untyped).value
                if value is not None:
                    values[key] = value
            raw[name] = values
        return ButlerFsm(config), LocationRegistry.from_dict(raw, home, kitchen, tables)

    # -- ROS entry points ------------------------------------------------------------------

    def _on_goal(self, request):
        """Accept an order batch if the machine is free and Nav2 is up, and start the task."""
        with self._lock:
            reason = ('nav2_unavailable' if not self._nav.server_is_ready()
                      else self._fsm.check_goal(list(request.tables)))
            if reason:
                self.get_logger().warning(f'order rejected: {reason}')
                return GoalResponse.REJECT
            self._done = threading.Event()
            self._goal_handle = None
            self._apply(self._fsm.start(list(request.tables)))
            return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        """Cancel the whole task: the machine decides between home and kitchen-then-home."""
        with self._lock:
            self._apply(self._fsm.cancel_all())
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        """Hold the action goal open until the machine finishes, then report the result."""
        done = self._done
        with self._lock:
            self._goal_handle = goal_handle
            self._publish()
        done.wait()
        with self._lock:
            snapshot = self._fsm.snapshot()
            outcome = self._outcome or 'cancelled'
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif outcome == 'nav_failed':
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return DeliverOrders.Result(
            outcome=outcome, delivered=list(snapshot.delivered),
            skipped=list(snapshot.skipped), cancelled=list(snapshot.cancelled))

    def _on_confirm(self, request, response):
        """Tell the machine someone attended the robot, if it is waiting at that location."""
        with self._lock:
            reason = self._fsm.can_confirm(request.location)
            if not reason:
                self._apply(self._fsm.confirmed(request.location))
        response.accepted, response.message = not reason, reason
        return response

    def _on_cancel_table(self, request, response):
        """Cancel one table's order so the robot skips it and serves the others."""
        with self._lock:
            reason = self._fsm.can_cancel_table(request.table)
            if not reason:
                self._apply(self._fsm.cancel_table(request.table))
        response.accepted, response.message = not reason, reason
        return response

    # -- carrying out commands (lock held) -------------------------------------------------

    def _apply(self, commands):
        """Carry out what the state machine asked for, then publish the new state."""
        for command in commands:
            if isinstance(command, Navigate):
                self._navigate(command.location)
            elif isinstance(command, StartTimer):
                self._start_timer(command.seconds)
            elif isinstance(command, StopTimer):
                self._stop_timer()
            elif isinstance(command, Finish):
                self._outcome = command.outcome
                self._publish()
                self._goal_handle = None
                self._done.set()
        self._publish()

    def _navigate(self, location):
        """Send a Nav2 goal for a named location, replacing any trip still in progress."""
        self._cancel_nav()
        self._nav_token += 1
        token = self._nav_token
        pose = self._locations.pose(location)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose.x, pose.y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = pose.quaternion()
        self._nav.send_goal_async(goal).add_done_callback(
            lambda future: self._on_nav_goal(future, token))

    def _cancel_nav(self):
        """Cancel the active Nav2 goal, if any, so the robot stops heading there."""
        if self._nav_handle is not None:
            self._nav_handle.cancel_goal_async()
            self._nav_handle = None

    def _on_nav_goal(self, future, token):
        """Handle Nav2's answer to a goal: watch for its result, or treat refusal as failure."""
        handle = future.result()
        with self._lock:
            if token != self._nav_token:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self._apply(self._fsm.nav_failed())
                return
            self._nav_handle = handle
        handle.get_result_async().add_done_callback(
            lambda result: self._on_nav_result(result, token))

    def _on_nav_result(self, future, token):
        """Turn the end of a Nav2 trip into an arrival or a failure event."""
        status = future.result().status
        with self._lock:
            if token != self._nav_token:
                return
            self._nav_handle = None
            self._apply(self._fsm.arrived() if status == GoalStatus.STATUS_SUCCEEDED
                        else self._fsm.nav_failed())

    def _start_timer(self, seconds):
        """Start the one-shot confirmation timer and the one-second countdown tick."""
        self._stop_timer()
        self._timer_end = self.get_clock().now() + Duration(seconds=seconds)
        self._timer = self.create_timer(seconds, self._on_timer,
                                        callback_group=self._timer_group)
        self._tick = self.create_timer(1.0, self._on_tick, callback_group=self._timer_group)

    def _stop_timer(self):
        """Cancel the confirmation timer and the countdown tick."""
        for timer in (self._timer, self._tick):
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
        self._timer = self._tick = self._timer_end = None

    def _on_timer(self):
        """Tell the machine nobody attended the robot in time, unless the wait already ended."""
        with self._lock:
            if self._timer is None:
                return
            self._stop_timer()
            self._apply(self._fsm.timeout())

    def _on_tick(self):
        """Publish the state once a second while a timer runs, so the countdown stays current."""
        with self._lock:
            if self._tick is not None:
                self._publish()

    def _publish(self):
        """Publish the state topic and the action feedback from the machine's snapshot."""
        snap = self._fsm.snapshot()
        remaining = 0.0
        if self._timer_end is not None:
            remaining = max(0.0, (self._timer_end - self.get_clock().now()).nanoseconds * 1e-9)
        self._state_pub.publish(ButlerState(
            stamp=self.get_clock().now().to_msg(), state=snap.state, target=snap.target,
            pending=list(snap.pending), delivered=list(snap.delivered),
            skipped=list(snap.skipped), cancelled=list(snap.cancelled),
            food_onboard=snap.food_onboard, timer_remaining=remaining))
        if self._goal_handle is not None:
            self._goal_handle.publish_feedback(DeliverOrders.Feedback(
                state=snap.state, target=snap.target, pending=list(snap.pending),
                delivered=list(snap.delivered), timer_remaining=remaining))

    def shutdown(self):
        """Stop the robot's trip and wake any waiting goal so the process can exit cleanly."""
        with self._lock:
            self._cancel_nav()
            self._stop_timer()
            self._done.set()


def main(args=None):
    """Run the task manager until it is shut down."""
    rclpy.init(args=args)
    node = ButlerTaskManager()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

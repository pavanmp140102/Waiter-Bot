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
Butler task state machine: ROS-free logic that decides where the robot goes next.

The node turns ROS callbacks into method calls on `ButlerFsm` and executes the commands that
come back. Nothing here reads a clock, starts a thread or imports ROS, so every delivery
situation can be replayed deterministically in a unit test.

One rule covers most situations: the robot goes home only when it is not carrying food. If it
is, it returns to the kitchen first (see `ButlerFsm.food_onboard` and `ButlerFsm._next_leg`).
"""
from dataclasses import dataclass
from enum import Enum


class State(str, Enum):
    """Phases of one delivery task, from waiting at home to coming back home."""

    IDLE = 'idle'
    TO_KITCHEN = 'to_kitchen'
    AT_KITCHEN = 'at_kitchen'
    TO_TABLE = 'to_table'
    AT_TABLE = 'at_table'
    RETURNING_FOOD = 'returning_food'
    TO_HOME = 'to_home'
    ERROR = 'error'


class Status(str, Enum):
    """What happened to one table's order, reported back to whoever placed it."""

    PENDING = 'pending'
    DELIVERED = 'delivered'
    SKIPPED = 'skipped'
    CANCELLED = 'cancelled'


@dataclass(frozen=True)
class Navigate:
    """Command: drive to a named location, replacing any trip in progress."""

    location: str


@dataclass(frozen=True)
class StartTimer:
    """Command: start the one-shot confirmation timer, so a missing person cannot stall a task."""

    seconds: float


@dataclass(frozen=True)
class StopTimer:
    """Command: cancel the confirmation timer because the wait ended another way."""


@dataclass(frozen=True)
class Finish:
    """Command: the task is over and the robot is idle, report this outcome to the client."""

    outcome: str


@dataclass(frozen=True)
class Config:
    """Names, timing and switches that make the machine generic instead of hardcoded."""

    tables: tuple
    home: str = 'home'
    kitchen: str = 'kitchen'
    confirmation_timeout_sec: float = 20.0
    require_kitchen_confirmation: bool = True
    require_table_confirmation: bool = True
    kitchen_return_dwell_sec: float = 0.0
    nav_max_retries: int = 2


@dataclass(frozen=True)
class Snapshot:
    """Read-only view of the task for the state topic and the action feedback."""

    state: str
    target: str
    pending: tuple
    delivered: tuple
    skipped: tuple
    cancelled: tuple
    food_onboard: bool


class ButlerFsm:
    """
    Event-driven state machine for one butler robot.

    Every event method returns the list of commands the caller must carry out.
    """

    def __init__(self, config):
        """Create an idle machine for the given configuration."""
        self.config = config
        self.state = State.IDLE
        self._reset()

    def _reset(self):
        """Forget the previous task so a new batch of orders starts clean."""
        self._orders = {}
        self._todo = []
        self._current = None
        self._loaded = set()
        self._picked_up = False
        self._outcome = None
        self._target = ''
        self._retries = 0
        self._dwelling = False

    # -- queries ---------------------------------------------------------------------------

    @property
    def food_onboard(self):
        """Tell whether loaded food is still undelivered, which decides kitchen before home."""
        return any(self._orders[t] is not Status.DELIVERED for t in self._loaded)

    def snapshot(self):
        """Return the current task as a `Snapshot` for the state topic and feedback."""
        def tables(status):
            return tuple(t for t, s in self._orders.items() if s is status)
        pending = ((self._current,) if self._current else ()) + tuple(self._todo)
        return Snapshot(self.state.value, self._target, pending, tables(Status.DELIVERED),
                        tables(Status.SKIPPED), tables(Status.CANCELLED), self.food_onboard)

    def check_goal(self, tables):
        """Return why a new order batch cannot start, or an empty string when it can."""
        if self.state not in (State.IDLE, State.ERROR):
            return 'busy'
        if not tables:
            return 'no tables given'
        if len(set(tables)) != len(tables):
            return 'duplicate table'
        unknown = [t for t in tables if t not in self.config.tables]
        return f'unknown table {unknown[0]}' if unknown else ''

    def can_confirm(self, location):
        """Return why a confirmation at this location is not wanted, or an empty string."""
        if self.state is State.AT_KITCHEN:
            waiting_at = self.config.kitchen
        elif self.state is State.AT_TABLE:
            waiting_at = self._current
        else:
            return 'not waiting for a confirmation'
        return '' if location == waiting_at else f'waiting at {waiting_at}, not {location}'

    def can_cancel_table(self, table):
        """Return why this table's order cannot be cancelled now, or an empty string."""
        status = self._orders.get(table)
        if status is None:
            return f'no order for {table}'
        if status is not Status.PENDING:
            return f'{table} is already {status.value}'
        return ''

    # -- events ----------------------------------------------------------------------------

    def start(self, tables):
        """Begin a delivery task for these tables, after `check_goal` accepted them."""
        self._reset()
        self._orders = {t: Status.PENDING for t in tables}
        self._todo = list(tables)
        self.state = State.TO_KITCHEN
        return self._go(self.config.kitchen)

    def arrived(self):
        """Handle the robot reaching its target, starting the wait or the next trip."""
        cfg = self.config
        if self.state is State.TO_KITCHEN:
            self.state = State.AT_KITCHEN
            return self._wait() if cfg.require_kitchen_confirmation else self._load()
        if self.state is State.TO_TABLE:
            self.state = State.AT_TABLE
            return self._wait() if cfg.require_table_confirmation else self._deliver()
        if self.state is State.RETURNING_FOOD and not self._dwelling:
            self._loaded = set()  # handed back at the kitchen, so nothing is onboard any more
            if cfg.kitchen_return_dwell_sec > 0:
                self._dwelling = True
                return [StartTimer(cfg.kitchen_return_dwell_sec)]
            return self._go_home()
        if self.state is State.TO_HOME:
            return self._finish()
        return []

    def confirmed(self, location):
        """Handle someone attending the robot: load the food or hand it over."""
        if self.can_confirm(location):
            return []
        if self.state is State.AT_KITCHEN:
            return [StopTimer(), *self._load()]
        return [StopTimer(), *self._deliver()]

    def timeout(self):
        """Handle nobody attending the robot in time, so it can move on or return home."""
        if self.state is State.AT_KITCHEN:
            self._outcome = 'kitchen_timeout'
            self._drop_pending(Status.SKIPPED)
            return self._go_home()
        if self.state is State.AT_TABLE:
            self._orders[self._current] = Status.SKIPPED
            return self._next_leg()
        if self.state is State.RETURNING_FOOD and self._dwelling:
            return self._go_home()
        return []

    def cancel_all(self):
        """Handle the whole task being cancelled: go home, via the kitchen with food onboard."""
        if self.state in (State.IDLE, State.ERROR):
            return []
        self._outcome = 'cancelled'
        if self.state in (State.TO_HOME, State.RETURNING_FOOD):
            return []
        self._drop_pending(Status.CANCELLED)
        return [*self._stop(), *self._next_leg()]

    def cancel_table(self, table):
        """Handle one table's order being cancelled: skip it and carry on with the others."""
        if self.can_cancel_table(table):
            return []
        self._orders[table] = Status.CANCELLED
        if table == self._current:
            return [*self._stop(), *self._next_leg()]
        self._todo.remove(table)
        if not self._picked_up and not self._todo:
            return [*self._stop(), *self._go_home()]
        return []

    def nav_failed(self):
        """Handle a failed trip: retry, then give up in the way that keeps the robot useful."""
        if self.state not in (State.TO_KITCHEN, State.TO_TABLE, State.RETURNING_FOOD,
                              State.TO_HOME):
            return []
        if self._retries < self.config.nav_max_retries:
            self._retries += 1
            return [Navigate(self._target)]
        if self.state is State.TO_KITCHEN:
            self._outcome = self._outcome or 'nav_failed'
            self._drop_pending(Status.SKIPPED)
            return self._go_home()
        if self.state is State.TO_TABLE:
            self._orders[self._current] = Status.SKIPPED
            return self._next_leg()
        if self.state is State.RETURNING_FOOD:
            return self._go_home()
        self.state = State.ERROR
        self._target = ''
        return [Finish('nav_failed')]

    # -- internals -------------------------------------------------------------------------

    def _next_leg(self):
        """Pick the next trip: the next table, else the kitchen if food is onboard, else home."""
        if self._todo:
            self._current = self._todo.pop(0)
            self.state = State.TO_TABLE
            return self._go(self._current)
        self._current = None
        if self.food_onboard:
            self.state = State.RETURNING_FOOD
            return self._go(self.config.kitchen)
        return self._go_home()

    def _go(self, location):
        """Send the robot to a location with a fresh retry budget."""
        self._target = location
        self._retries = 0
        return [Navigate(location)]

    def _go_home(self):
        """Send the robot home, ending the task when it arrives."""
        self.state = State.TO_HOME
        self._dwelling = False
        return self._go(self.config.home)

    def _wait(self):
        """Start waiting for someone to attend the robot at the kitchen or a table."""
        return [StartTimer(self.config.confirmation_timeout_sec)]

    def _stop(self):
        """Cancel the confirmation timer if the robot is currently waiting on one."""
        waiting = self.state in (State.AT_KITCHEN, State.AT_TABLE) or (
            self.state is State.RETURNING_FOOD and self._dwelling)
        return [StopTimer()] if waiting else []

    def _load(self):
        """Take the food on board at the kitchen and set off for the first table."""
        self._picked_up = True
        self._loaded = {t for t, s in self._orders.items() if s is Status.PENDING}
        return self._next_leg()

    def _deliver(self):
        """Mark the current table delivered and move on."""
        self._orders[self._current] = Status.DELIVERED
        return self._next_leg()

    def _drop_pending(self, status):
        """Give every order that was never served this status, so the result accounts for all."""
        for table, current in self._orders.items():
            if current is Status.PENDING:
                self._orders[table] = status
        self._todo.clear()
        self._current = None

    def _finish(self):
        """End the task at home: completed, cancelled, partial or the recorded reason."""
        self.state = State.IDLE
        self._target = ''
        statuses = set(self._orders.values())
        if statuses == {Status.DELIVERED}:
            default = 'completed'
        elif statuses == {Status.CANCELLED}:
            default = 'cancelled'
        else:
            default = 'partial'
        return [Finish(self._outcome or default)]

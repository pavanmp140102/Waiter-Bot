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

"""Test helper: drives a `ButlerFsm` like the node would and records what it was told to do."""
from waiter_bot_core.butler_fsm import (
    ButlerFsm, Config, Finish, Navigate, StartTimer, StopTimer)

TABLES = ('table1', 'table2', 'table3')


class Bot:
    """A machine plus a log of trips, so a test reads like the situation it replays."""

    def __init__(self, tables=TABLES, **options):
        """Create an idle machine; options override the `Config` defaults."""
        self.fsm = ButlerFsm(Config(tables=tuple(tables), **options))
        self.visited = []
        self.outcome = None
        self.timer = False

    def _do(self, commands):
        """Carry out commands the way the node does, checking the timer is used sanely."""
        for command in commands:
            if isinstance(command, Navigate):
                self.visited.append(command.location)
            elif isinstance(command, StartTimer):
                assert not self.timer, 'timer started while one was running'
                self.timer = True
            elif isinstance(command, StopTimer):
                assert self.timer, 'timer stopped while none was running'
                self.timer = False
            elif isinstance(command, Finish):
                assert not self.timer, 'task finished with a timer running'
                self.outcome = command.outcome
        return self

    def start(self, *tables):
        """Place an order, checking the machine accepts it."""
        assert self.fsm.check_goal(tables) == ''
        return self._do(self.fsm.start(tables))

    def arrive(self, times=1):
        """Make the robot reach its target, once or several times in a row."""
        for _ in range(times):
            self._do(self.fsm.arrived())
        return self

    def confirm(self, location):
        """Have someone attend the robot at this location."""
        return self._do(self.fsm.confirmed(location))

    def timeout(self):
        """Let the timer run out because nobody attended the robot."""
        self.timer = False
        return self._do(self.fsm.timeout())

    def cancel_all(self):
        """Cancel the whole task."""
        return self._do(self.fsm.cancel_all())

    def cancel_table(self, table):
        """Cancel one table's order."""
        return self._do(self.fsm.cancel_table(table))

    def nav_failed(self):
        """Fail the current trip."""
        return self._do(self.fsm.nav_failed())

    @property
    def state(self):
        """Return the current state name, for short assertions."""
        return self.fsm.state.value

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

"""Location registry checks: a missing pose must stop start-up with a clear message."""
import math

import pytest

from waiter_bot_core.locations import LocationRegistry

RAW = {
    'home': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    'kitchen': {'x': 4.0, 'y': 0.0, 'yaw': math.pi},
    'table1': {'x': 1.0, 'y': 2.0, 'yaw': 1.57},
}


def test_registry_returns_poses():
    """Each named location gives back the pose it was loaded with."""
    registry = LocationRegistry.from_dict(RAW, 'home', 'kitchen', ['table1'])
    assert registry.pose('table1').y == 2.0
    z, w = registry.pose('kitchen').quaternion()
    assert abs(z - 1.0) < 1e-9 and abs(w) < 1e-9


def test_missing_location_fails_fast():
    """A table without a pose is reported by name before the robot ever moves."""
    with pytest.raises(ValueError, match='table2'):
        LocationRegistry.from_dict(RAW, 'home', 'kitchen', ['table1', 'table2'])


def test_incomplete_pose_fails_fast():
    """A pose without a yaw is reported with the missing key."""
    raw = dict(RAW, table1={'x': 1.0, 'y': 2.0})
    with pytest.raises(ValueError, match='yaw'):
        LocationRegistry.from_dict(raw, 'home', 'kitchen', ['table1'])

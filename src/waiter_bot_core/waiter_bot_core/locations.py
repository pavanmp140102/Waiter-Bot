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

"""Named locations of the café: ROS-free, so the same table can be checked in plain tests."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    """Where the robot should stand: a position in the map frame and a heading in radians."""

    x: float
    y: float
    yaw: float

    def quaternion(self):
        """Return the heading as (z, w) of a quaternion, the form a Nav2 goal pose needs."""
        return math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)


class LocationRegistry:
    """Maps location names (home, kitchen, tables) to poses and checks that none is missing."""

    def __init__(self, poses, home, kitchen, tables):
        """Build the registry, failing at start-up if a required location has no pose."""
        missing = [n for n in (home, kitchen, *tables) if n not in poses]
        if missing:
            raise ValueError(f'no pose for location(s): {", ".join(missing)}')
        self._poses = dict(poses)

    def pose(self, name):
        """Return the pose of a named location so the task manager can send a goal to it."""
        return self._poses[name]

    @classmethod
    def from_dict(cls, raw, home, kitchen, tables):
        """Build a registry from {name: {x, y, yaw}}, as loaded from the parameter file."""
        poses = {}
        for name, values in raw.items():
            absent = [k for k in ('x', 'y', 'yaw') if k not in values]
            if absent:
                raise ValueError(f'location {name} is missing {", ".join(absent)}')
            poses[name] = Pose2D(float(values['x']), float(values['y']), float(values['yaw']))
        return cls(poses, home, kitchen, tables)

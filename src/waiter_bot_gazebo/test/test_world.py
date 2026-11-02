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

"""S3: every world file is valid SDF."""
import glob
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
import pytest

WORLDS = sorted(glob.glob(os.path.join(
    get_package_share_directory('waiter_bot_gazebo'), 'worlds', '*.sdf')))


def test_worlds_exist():
    """At least the empty world is installed."""
    assert any(w.endswith('empty.sdf') for w in WORLDS)


@pytest.mark.parametrize('world', WORLDS)
def test_world_is_valid_sdf(world):
    """`gz sdf -k` accepts the world."""
    out = subprocess.run(['gz', 'sdf', '-k', world], capture_output=True, text=True)
    assert out.returncode == 0 and 'Valid' in out.stdout, out.stdout + out.stderr

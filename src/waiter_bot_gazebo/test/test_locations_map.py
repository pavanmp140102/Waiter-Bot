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

"""S4 and S8: the locations agree with the map and the world, and the repository stays small."""
import math
import os
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

CLEARANCE_M = 0.4
TOLERANCE_M = 0.05
FREE_PIXEL = 254


def _share(package, *parts):
    """Return a path inside a package's share folder, where the installed data files live."""
    return os.path.join(get_package_share_directory(package), *parts)


def _locations():
    """Read the location table the task manager uses, so tests check the real poses."""
    with open(_share('waiter_bot_core', 'config', 'cafe.yaml')) as f:
        params = yaml.safe_load(f)['butler_task_manager']['ros__parameters']
    return params['locations']


def _world_includes():
    """Map each included entity in the café world to its x, y, yaw pose."""
    return {i.findtext('name'): [float(v) for v in i.findtext('pose').split()]
            for i in ET.parse(_share('waiter_bot_gazebo', 'worlds', 'cafe.sdf')).getroot().iter(
                'include')}


def _load_map():
    """Read the saved café map, so the tests can check where the robot is allowed to stop."""
    with open(_share('waiter_bot_navigation', 'maps', 'cafe.yaml')) as f:
        info = yaml.safe_load(f)
    with open(_share('waiter_bot_navigation', 'maps', info['image']), 'rb') as f:
        tokens = []
        while len(tokens) < 4:
            tokens += f.readline().split(b'#')[0].split()
        magic, width, height, _ = tokens[0], int(tokens[1]), int(tokens[2]), tokens[3]
        assert magic == b'P5'
        data = f.read()
    return info, width, height, data


def test_locations_are_free_with_clearance():
    """Each stopping point has 0.4 m of free map around it, so Nav2 can always reach it."""
    info, width, height, data = _load_map()
    res = info['resolution']
    ox, oy = info['origin'][0], info['origin'][1]
    radius = int(math.ceil(CLEARANCE_M / res))
    for name, pose in _locations().items():
        cx = int((pose['x'] - ox) / res)
        cy = height - 1 - int((pose['y'] - oy) / res)
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if math.hypot(dx, dy) * res > CLEARANCE_M:
                    continue
                x, y = cx + dx, cy + dy
                assert 0 <= x < width and 0 <= y < height, name
                assert data[y * width + x] == FREE_PIXEL, '%s not free at %d,%d' % (name, x, y)


def test_locations_match_world_markers():
    """Each location has a world floor marker within 5 cm, so the demo shows the targets."""
    includes = _world_includes()
    for name, pose in _locations().items():
        marker = includes['%s_marker' % name]
        assert math.hypot(marker[0] - pose['x'], marker[1] - pose['y']) <= TOLERANCE_M, name


def test_world_obstacles_are_occupied_in_map():
    """Table, counter and column centres are occupied in the map, so Nav2 plans around them."""
    info, width, height, data = _load_map()
    res = info['resolution']
    checked = 0
    for name, pose in _world_includes().items():
        if name.endswith('_centre') or name == 'kitchen_counter' or name.startswith('column_'):
            x = int((pose[0] - info['origin'][0]) / res)
            y = height - 1 - int((pose[1] - info['origin'][1]) / res)
            assert data[y * width + x] == 0, '%s is not occupied in the map' % name
            checked += 1
    assert checked == 6


def test_repository_under_15_mb_without_blender_files():
    """The repository stays under 15 MB and holds no `.blend` files, so it clones quickly."""
    root = subprocess.run(['git', 'rev-parse', '--show-toplevel'], capture_output=True,
                          text=True, cwd=os.path.dirname(__file__)).stdout.strip()
    if not root or not os.path.isdir(os.path.join(root, '.git')):
        pytest.skip('not a git checkout')
    files = subprocess.run(['git', 'ls-files', '-co', '--exclude-standard'], capture_output=True,
                           text=True, cwd=root).stdout.split('\n')
    files = [f for f in files if f and os.path.isfile(os.path.join(root, f))]
    assert sum(os.path.getsize(os.path.join(root, f)) for f in files) < 15 * 1024 * 1024
    assert not [f for f in files if f.endswith(('.blend', '.blend1'))]

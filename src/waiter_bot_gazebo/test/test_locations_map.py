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
import yaml

LOCATIONS = yaml.safe_load(open(os.path.join(
    get_package_share_directory('waiter_bot_core'), 'config', 'cafe.yaml')))[
        'butler_task_manager']['ros__parameters']['locations']
MAP_DIR = os.path.join(get_package_share_directory('waiter_bot_navigation'), 'maps')
WORLD = os.path.join(get_package_share_directory('waiter_bot_gazebo'), 'worlds', 'cafe.sdf')
CLEARANCE_M = 0.4
TOLERANCE_M = 0.05


def _load_map():
    """Read the saved café map, so the tests can check where the robot is allowed to stop."""
    info = yaml.safe_load(open(os.path.join(MAP_DIR, 'cafe.yaml')))
    with open(os.path.join(MAP_DIR, info['image']), 'rb') as f:
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
    for name, pose in LOCATIONS.items():
        cx = int((pose['x'] - ox) / res)
        cy = height - 1 - int((pose['y'] - oy) / res)
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if math.hypot(dx, dy) * res > CLEARANCE_M:
                    continue
                x, y = cx + dx, cy + dy
                assert 0 <= x < width and 0 <= y < height, name
                assert data[y * width + x] > 0.75 * 255, '%s blocked at %d,%d' % (name, x, y)


def test_locations_match_world_markers():
    """Each location has a world floor marker within 5 cm, so the demo shows the targets."""
    includes = {i.findtext('name'): [float(v) for v in i.findtext('pose').split()]
                for i in ET.parse(WORLD).getroot().iter('include')}
    for name, pose in LOCATIONS.items():
        marker = includes['%s_marker' % name]
        assert math.hypot(marker[0] - pose['x'], marker[1] - pose['y']) <= TOLERANCE_M, name


def test_repository_under_15_mb_without_blender_files():
    """The repository stays under 15 MB and holds no `.blend` files, so it clones quickly."""
    root = subprocess.run(['git', 'rev-parse', '--show-toplevel'], capture_output=True,
                          text=True, cwd=os.path.dirname(__file__)).stdout.strip()
    files = subprocess.run(['git', 'ls-files', '-co', '--exclude-standard'], capture_output=True,
                           text=True, cwd=root).stdout.split('\n')
    files = [f for f in files if f and os.path.isfile(os.path.join(root, f))]
    assert sum(os.path.getsize(os.path.join(root, f)) for f in files) < 15 * 1024 * 1024
    assert not [f for f in files if f.endswith(('.blend', '.blend1'))]

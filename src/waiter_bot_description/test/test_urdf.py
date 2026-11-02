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

"""S3: the robot description turns into a valid URDF with the expected structure."""
import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
import pytest

XACRO = os.path.join(get_package_share_directory('waiter_bot_description'),
                     'urdf', 'waiter_bot.urdf.xacro')
WHEEL_JOINTS = ('lf_wheel_joint', 'rf_wheel_joint', 'lb_wheel_joint', 'rb_wheel_joint')


def urdf(*args):
    """Return the URDF text produced by xacro, with optional xacro arguments."""
    return subprocess.run(['xacro', XACRO, *args], check=True, capture_output=True,
                          text=True).stdout


def test_check_urdf_accepts_the_model(tmp_path):
    """`check_urdf` parses the model and finds one tree rooted at base_footprint."""
    path = tmp_path / 'waiter_bot.urdf'
    path.write_text(urdf())
    out = subprocess.run(['check_urdf', str(path)], check=True, capture_output=True,
                         text=True).stdout
    assert 'Successfully Parsed XML' in out and 'root Link: base_footprint' in out


def test_wheel_joints_and_sensors_exist():
    """The four wheel joints, the lidar and the camera are all present."""
    text = urdf()
    for joint in WHEEL_JOINTS:
        assert f'name="{joint}"' in text
    assert 'name="lidar_link"' in text and 'name="camera_link"' in text


def test_camera_can_be_left_out():
    """With camera:=false no camera link or sensor is generated, for lighter tests."""
    text = urdf('camera:=false')
    assert 'camera_link' not in text and 'front_camera' not in text


def test_masses_are_plausible():
    """The robot weighs tens of kilograms, not the 83 kg of the original export."""
    masses = [float(m) for m in re.findall(r'<mass value="([0-9.e+-]+)"', urdf())]
    assert 10.0 < sum(masses) < 40.0 and all(m > 0 for m in masses)


def test_every_mesh_file_exists():
    """Each mesh the model refers to is installed with the package."""
    share = get_package_share_directory('waiter_bot_description')
    meshes = re.findall(r'package://waiter_bot_description/([^"]+)"', urdf())
    assert meshes
    for mesh in meshes:
        assert os.path.isfile(os.path.join(share, mesh)), mesh


@pytest.mark.parametrize('joint', WHEEL_JOINTS)
def test_wheels_are_driven_by_the_diff_drive_plugin(joint):
    """Every wheel joint is named in the DiffDrive plugin, so none is left free-spinning."""
    text = urdf()
    assert re.search(rf'<(left|right)_joint>{joint}</(left|right)_joint>', text)

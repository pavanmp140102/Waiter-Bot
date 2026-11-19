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

MODELS_DIR = os.path.join(get_package_share_directory('waiter_bot_gazebo'), 'models')
MODELS = sorted(glob.glob(os.path.join(MODELS_DIR, '*', 'model.sdf')))
WORLDS = sorted(glob.glob(os.path.join(
    get_package_share_directory('waiter_bot_gazebo'), 'worlds', '*.sdf')))


def test_worlds_exist():
    """The empty world and the café world are installed, so the simulator has worlds to load."""
    assert any(w.endswith('empty.sdf') for w in WORLDS)
    assert any(w.endswith('cafe.sdf') for w in WORLDS)


@pytest.mark.parametrize('world', WORLDS)
def test_world_is_valid_sdf(world):
    """`gz sdf -k` accepts the world, with its `model://` includes resolved."""
    env = dict(os.environ, SDF_PATH=MODELS_DIR)
    out = subprocess.run(['gz', 'sdf', '-k', world], capture_output=True, text=True, env=env)
    assert out.returncode == 0 and 'Valid' in out.stdout, out.stdout + out.stderr


@pytest.mark.parametrize('model', MODELS)
def test_model_is_valid_sdf(model):
    """`gz sdf -k` accepts every model the café world loads, and it has a model.config."""
    assert os.path.isfile(os.path.join(os.path.dirname(model), 'model.config'))
    out = subprocess.run(['gz', 'sdf', '-k', model], capture_output=True, text=True)
    assert out.returncode == 0 and 'Valid' in out.stdout, out.stdout + out.stderr

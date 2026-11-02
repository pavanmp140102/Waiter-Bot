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

"""S6: the logic modules import only the standard library, so they stay testable without ROS."""
import ast
from pathlib import Path
import sys

import waiter_bot_core

LOGIC_MODULES = ('butler_fsm.py', 'locations.py')


def imported_modules(path):
    """Return the top-level module names a source file imports."""
    names = set()
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Import):
            names.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            names.add(node.module.split('.')[0])
    return names


def test_s6_logic_modules_import_only_the_standard_library():
    """Fail if the state machine or the location registry ever imports ROS or another package."""
    package_dir = Path(waiter_bot_core.__file__).parent
    for name in LOGIC_MODULES:
        outside = imported_modules(package_dir / name) - set(sys.stdlib_module_names)
        assert not outside, f'{name} imports {sorted(outside)}'

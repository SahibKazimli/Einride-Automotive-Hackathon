# Copyright 2025 Einride AB
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Type-check the package, the way test_flake8 and test_pep257 lint it.

There is no ament_mypy in Jazzy, so this drives mypy's own API rather than an
ament wrapper. It skips where mypy is absent - it is a declared test dependency,
but the package must still be testable on a machine that only has the runtime.
"""

import pathlib

import pytest

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parent.parent
SOURCES = [
    str(PACKAGE_ROOT / 'einride_mini_truck_hardware'),
    str(PACKAGE_ROOT / 'test'),
    str(PACKAGE_ROOT / 'setup.py'),
]
CODEC = str(PACKAGE_ROOT / 'einride_mini_truck_hardware' / 'serial_link.py')


def _mypy(tmp_path: pathlib.Path, *arguments: str) -> None:
    """Run mypy and fail with its own report. Skips if mypy is not installed."""
    api = pytest.importorskip('mypy.api', reason='mypy is not installed')
    # Keep the cache out of the source tree; the package is small enough that
    # losing incrementality costs nothing.
    stdout, stderr, status = api.run(
        ['--cache-dir', str(tmp_path / 'mypy_cache'), *arguments])
    assert status == 0, stdout + stderr


@pytest.mark.linter
def test_the_package_is_typed(tmp_path: pathlib.Path) -> None:
    """Every function and method carries annotations, and they agree.

    --ignore-missing-imports because rclpy and the message packages ship no type
    information: without it every ROS import is an error about a missing stub,
    which says nothing about this package.
    """
    _mypy(tmp_path, '--ignore-missing-imports', '--disallow-untyped-defs', *SOURCES)


@pytest.mark.linter
def test_the_codec_is_strictly_typed(tmp_path: pathlib.Path) -> None:
    """serial_link.py has no ROS dependency, so it can be checked in full.

    That is the point of keeping it a pure module - it is the part most likely
    to be reused or ported, and --strict is a stronger guarantee than the node
    can offer while rclpy remains untyped.
    """
    _mypy(tmp_path, '--strict', CODEC)


def test_the_package_declares_it_is_typed() -> None:
    """PEP 561: without this marker, consumers get none of the above."""
    assert (PACKAGE_ROOT / 'einride_mini_truck_hardware' / 'py.typed').is_file()
    assert "package_data={package_name: ['py.typed']}" in \
        (PACKAGE_ROOT / 'setup.py').read_text()

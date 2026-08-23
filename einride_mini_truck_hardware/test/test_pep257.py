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

from ament_pep257.main import main
import pytest

# ament_pep257's shipped 'ament' convention is pydocstyle's *entire* rule set
# minus a short ignore list, which pulls in four checks this package
# deliberately does not follow:
#
#   D213  wants the summary on the second line, i.e. a blank first line. PEP 257
#         itself and the rest of the ROS 2 ecosystem put it on the first line.
#         D212, its mutually exclusive twin, is already in ament's ignore list,
#         so exactly one of the pair has to be turned off here.
#   D406  Google-style section headers end in a colon, not a newline.
#   D407  and take no dashed underline; that is numpy style.
#   D413  and need no trailing blank line after the last section.
#
# Everything else in the convention stays on.
IGNORED = ['D213', 'D406', 'D407', 'D413']


@pytest.mark.pep257
@pytest.mark.linter
def test_pep257() -> None:
    rc = main(argv=['--add-ignore'] + IGNORED + ['--'])
    assert rc == 0, 'Found code style errors / warnings'

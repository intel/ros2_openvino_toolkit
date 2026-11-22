# Copyright (c) 2026 Intel Corporation
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

"""Helpers shared by the openvino_test launch files."""

import os
import sys

from ament_index_python.packages import get_package_share_directory

_node_launch_dir = os.path.join(get_package_share_directory('openvino_node'), 'launch')
if _node_launch_dir not in sys.path:
    sys.path.insert(0, _node_launch_dir)

import launch_helpers  # noqa: E402


def resolve_test_yaml(yaml_filename):
    """Return a copy of a test YAML from openvino_test with its path placeholders resolved."""
    return launch_helpers.resolve_yaml_paths(yaml_filename, package_name='openvino_test')

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

"""
Helper utilities for OpenVINO ROS2 launch files.
Provides path resolution for YAML configuration files.
"""

import os
import re
import sys
import tempfile
from ament_index_python.packages import get_package_share_directory

TOOLKIT_DIR_ENV = 'OPENVINO_TOOLKIT_DIR'


def get_toolkit_dir():
    """
    Return the directory that contains the 'models' folder.

    Lookup order:
    1. $OPENVINO_TOOLKIT_DIR
    2. $XDG_DATA_HOME/openvino_toolkit (default ~/.local/share/openvino_toolkit)
    3. /opt/openvino_toolkit
    The first candidate that contains a 'models' directory wins; when none
    exists, the XDG location is returned (the download script target).
    """
    xdg_data = os.environ.get('XDG_DATA_HOME') or os.path.join(
        os.path.expanduser('~'), '.local', 'share')
    xdg_dir = os.path.join(xdg_data, 'openvino_toolkit')

    env_dir = os.environ.get(TOOLKIT_DIR_ENV)
    if env_dir:
        return os.path.abspath(os.path.expanduser(env_dir))

    for candidate in (xdg_dir, '/opt/openvino_toolkit'):
        if os.path.isdir(os.path.join(candidate, 'models')):
            return candidate
    return xdg_dir


def get_device_override():
    """
    Return the inference device requested on the command line, or None.

    Usage: ros2 launch openvino_node <file>.launch.py device:=GPU
    Falls back to $OPENVINO_DEVICE. Any OpenVINO device string is accepted,
    e.g. CPU, GPU, GPU.1, NPU, AUTO, AUTO:GPU,CPU, HETERO:GPU,CPU.
    When unset, the 'engine:' values from the pipeline YAML are used.
    """
    for arg in sys.argv:
        if arg.startswith('device:='):
            value = arg.split(':=', 1)[1].strip()
            if value:
                return value
    return os.environ.get('OPENVINO_DEVICE') or None


def resolve_yaml_paths(yaml_filename, package_name='openvino_node'):
    """
    Load a YAML file and resolve placeholder paths.

    Replaces the following placeholders:
    - <OPENVINO_NODE_SHARE> -> toolkit directory (see get_toolkit_dir())
    - <OPENVINO_DATA> -> openvino_node share directory + /data (labels, images)
    - <OPENVINO_TEST_DATA> -> share directory of package_name + /data
      (e.g. test videos installed by openvino_test)

    Args:
        yaml_filename: Name of the YAML file (e.g., 'pipeline_people.yaml')
        package_name: ROS2 package that owns the YAML file in its 'param'
            directory (default: 'openvino_node')

    Returns:
        Path to the resolved YAML file (temporary file)
    """
    package_share_dir = get_package_share_directory(package_name)
    node_share_dir = get_package_share_directory('openvino_node')
    original_yaml = os.path.join(package_share_dir, 'param', yaml_filename)
    toolkit_dir = get_toolkit_dir()

    temp_dir = tempfile.mkdtemp(prefix='openvino_')
    resolved_yaml = os.path.join(temp_dir, yaml_filename)

    with open(original_yaml, 'r') as f:
        yaml_content = f.read()

    device = get_device_override()
    if device:
        yaml_content = re.sub(r'^(\s*engine:\s*)\S+', r'\g<1>' + device,
                              yaml_content, flags=re.MULTILINE)
        print('[launch_helpers] Using inference device: %s' % device)

    yaml_content = yaml_content.replace('<OPENVINO_NODE_SHARE>', toolkit_dir)
    yaml_content = yaml_content.replace(
        '<OPENVINO_DATA>', os.path.join(node_share_dir, 'data'))
    yaml_content = yaml_content.replace(
        '<OPENVINO_TEST_DATA>', os.path.join(package_share_dir, 'data'))

    with open(resolved_yaml, 'w') as f:
        f.write(yaml_content)

    # Warn early about missing model files instead of failing inside the node.
    for model in re.findall(r'^\s*model:\s*(\S+)', yaml_content, re.MULTILINE):
        if not os.path.isfile(model):
            print('[launch_helpers] WARNING: model not found: %s\n'
                  '  Set %s or run: ros2 run openvino_node download_models.sh'
                  % (model, TOOLKIT_DIR_ENV))

    return resolved_yaml

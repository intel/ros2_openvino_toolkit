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
import tempfile
from ament_index_python.packages import get_package_share_directory


def resolve_yaml_paths(yaml_filename, package_name='openvino_node'):
    """
    Load a YAML file and resolve placeholder paths.

    Replaces the following placeholders:
    - <OPENVINO_NODE_SHARE> -> /opt/openvino_toolkit (system OpenVINO models)
    - <OPENVINO_DATA> -> Package share directory + /data (labels, images)

    Args:
        yaml_filename: Name of the YAML file (e.g., 'pipeline_people.yaml')
        package_name: ROS2 package name (default: 'openvino_node')

    Returns:
        Path to the resolved YAML file (temporary file)
    """
    # Get package share directory
    package_share_dir = get_package_share_directory(package_name)
    original_yaml = os.path.join(package_share_dir, 'param', yaml_filename)

    # Create a temporary directory for resolved YAML
    temp_dir = tempfile.mkdtemp(prefix='openvino_')
    resolved_yaml = os.path.join(temp_dir, yaml_filename)

    # Read and process the YAML file
    with open(original_yaml, 'r') as f:
        yaml_content = f.read()

    # Replace placeholders with actual paths
    # Models use system OpenVINO installation
    yaml_content = yaml_content.replace('<OPENVINO_NODE_SHARE>', '/opt/openvino_toolkit')
    # Data (labels, images) use package installation
    yaml_content = yaml_content.replace('<OPENVINO_DATA>', os.path.join(package_share_dir, 'data'))

    # Write the resolved YAML
    with open(resolved_yaml, 'w') as f:
        f.write(yaml_content)

    return resolved_yaml

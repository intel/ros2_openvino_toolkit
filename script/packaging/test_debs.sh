#!/usr/bin/env bash
# Copyright (C) 2026 Intel Corporation
#
# SPDX-License-Identifier: Apache-2.0
#
# Install the Debian packages from DIST_DIR and check the installation. Runs as
# root inside the clean container started by `make test-package`.
set -eo pipefail

SRC_DIR="${SRC_DIR:-/src}"

"$SRC_DIR/script/packaging/install_debs.sh"

# shellcheck source=/dev/null
. "/opt/ros/$ROS_DISTRO/setup.bash"
prefix="/opt/ros/$ROS_DISTRO"

for package in object_msgs openvino_msgs openvino_param_lib openvino_wrapper_lib openvino_node; do
  ros2 pkg prefix "$package"
done
ros2 interface show object_msgs/msg/ObjectsInBoxes > /dev/null
ros2 interface show openvino_msgs/srv/PipelineSrv > /dev/null
ros2 pkg executables openvino_node | grep pipeline_with_params
ros2 launch openvino_node pipeline_image.launch.py --show-args

mapfile -t binaries < <(find "$prefix/lib/openvino_node" -type f ! -name '*.sh')
ldd "${binaries[@]}" "$prefix"/lib/lib{openvino_param_lib,openvino_wrapper_lib,composable_pipeline}.so > /tmp/ldd.txt
if grep "not found" /tmp/ldd.txt; then
  echo "Unresolved shared libraries." >&2
  exit 1
fi
echo "Package test passed."

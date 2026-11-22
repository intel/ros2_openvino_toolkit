#!/usr/bin/env bash
# Copyright (C) 2026 Intel Corporation
#
# SPDX-License-Identifier: Apache-2.0
#
# Enable the OpenVINO APT repository and install the Debian packages from
# DIST_DIR, plus the APT packages given as arguments. Runs as root inside the
# containers started by `make test-package`, `make test-functional` and
# `make shell`.
set -euo pipefail

SRC_DIR="${SRC_DIR:-/src}"
DIST_DIR="${DIST_DIR:-/dist}"

export DEBIAN_FRONTEND=noninteractive
"$SRC_DIR/script/packaging/setup_openvino_apt.sh"
apt-get -y -q install --no-install-recommends "$DIST_DIR"/*.deb "$@"

#!/usr/bin/env bash
# Copyright (C) 2026 Intel Corporation
#
# SPDX-License-Identifier: Apache-2.0
#
# Enable the Intel OpenVINO APT repository for the running Ubuntu release.
# Usage: sudo ./setup_openvino_apt.sh
set -euo pipefail

KEY_URL="https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB"
KEYRING="/usr/share/keyrings/intel-openvino-archive-keyring.gpg"
SOURCE_LIST="/etc/apt/sources.list.d/intel-openvino.list"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "This script must be run as root." >&2
  exit 1
fi

# shellcheck source=/dev/null
. /etc/os-release
suite="ubuntu${VERSION_ID%%.*}"

export DEBIAN_FRONTEND=noninteractive
apt-get -qq update
apt-get -qq install -y --no-install-recommends ca-certificates curl gnupg

curl -fsSL "$KEY_URL" | gpg --dearmor --yes -o "$KEYRING"
echo "deb [signed-by=$KEYRING] https://apt.repos.intel.com/openvino $suite main" > "$SOURCE_LIST"
apt-get -qq update

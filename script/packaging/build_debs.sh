#!/usr/bin/env bash
# Copyright (C) 2026 Intel Corporation
#
# SPDX-License-Identifier: Apache-2.0
#
# Build the Debian packages in dependency order. Runs as root inside the
# container started by `make package`: sources in SRC_DIR, packages to OUT_DIR.
set -euo pipefail

SRC_DIR="${SRC_DIR:-/src}"
OUT_DIR="${OUT_DIR:-/out}"
# Each package is installed after it is built, so the order matters.
PACKAGES=(object_msgs openvino_msgs openvino_param_lib openvino_wrapper_lib sample)

export DEBIAN_FRONTEND=noninteractive
apt-get -qq update
apt-get -qq install -y --no-install-recommends build-essential devscripts equivs
"$SRC_DIR/script/packaging/setup_openvino_apt.sh"

# Build in a copy so that the source tree stays free of build artifacts.
work_dir="$(mktemp -d)"
tar -C "$SRC_DIR" --exclude=.git --exclude=./build --exclude=./install \
  --exclude=./log --exclude=./dist --exclude=./.models --exclude=./.colcon-docker \
  -cf - . | tar -C "$work_dir" -xf -

arch="$(dpkg --print-architecture)"
for package in "${PACKAGES[@]}"; do
  cd "$work_dir/$package"
  mk-build-deps --install --remove \
    --tool 'apt-get -y -q --no-install-recommends' debian/control
  dpkg-buildpackage -b -us -uc

  source_name="$(dpkg-parsechangelog --show-field Source)"
  version="$(dpkg-parsechangelog --show-field Version)"
  deb="$work_dir/${source_name}_${version}_${arch}.deb"
  apt-get -y -q install --no-install-recommends "$deb"
  cp "$deb" "$OUT_DIR/"
done

if [[ -n "${HOST_UID:-}" ]]; then
  chown "$HOST_UID:${HOST_GID:-$HOST_UID}" "$OUT_DIR"/*.deb
fi

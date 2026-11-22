# Copyright (C) 2026 Intel Corporation
#
# SPDX-License-Identifier: Apache-2.0

SHELL := bash -eu -o pipefail

ROOT_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

# Fixed per branch so that a sourced ROS environment cannot redirect the build.
override ROS_DISTRO := jazzy
DOCKER_IMAGE := ros:$(ROS_DISTRO)-ros-base
DIST_DIR := $(ROOT_DIR)/dist
# Model cache of `make test`, mounted at /opt/openvino_toolkit in the container.
MODELS_DIR := $(ROOT_DIR)/.models
# colcon output of the container targets, kept apart from a native build/ install/ log/.
COLCON_DIR := .colcon-docker
COLCON_BASES := --build-base $(COLCON_DIR)/build --install-base $(COLCON_DIR)/install
MAKE_JOBS ?= $(shell nproc)
HOST_UID := $(shell id -u)
HOST_GID := $(shell id -g)

# OpenVINO release used by the colcon targets; keep in sync with */debian/control.
OPENVINO_VERSION := 2025.4.0

DOCKER_RUN := docker run --rm -t --platform linux/amd64 \
	-e DEBIAN_FRONTEND=noninteractive \
	-e http_proxy -e https_proxy -e no_proxy

# Inference device of `make test-functional`, for example CPU, GPU or AUTO.
DEVICE ?= CPU
# APT packages added by `make shell`: the GPU driver of the distribution and an image viewer.
SHELL_PACKAGES ?= intel-opencl-icd ros-$(ROS_DISTRO)-rqt-image-view
# Model cache, GPU (/dev/dri) and NPU (/dev/accel) for the containers that run pipelines.
DOCKER_RUNTIME_ARGS := -v $(MODELS_DIR):/opt/openvino_toolkit \
	-e OPENVINO_TOOLKIT_DIR=/opt/openvino_toolkit \
	$(if $(wildcard /dev/dri),--device /dev/dri) $(if $(wildcard /dev/accel),--device /dev/accel)
# X11 display of the host for `make shell`. The X authority entry is bound to the host name.
DOCKER_DISPLAY_ARGS := $(if $(DISPLAY),-e DISPLAY -e QT_X11_NO_MITSHM=1 \
	-v /tmp/.X11-unix:/tmp/.X11-unix --hostname $(shell hostname) \
	$(if $(XAUTHORITY),-e XAUTHORITY=/tmp/.Xauthority -v $(XAUTHORITY):/tmp/.Xauthority:ro))

define CHECK_PACKAGES
@if ! ls $(DIST_DIR)/ros-$(ROS_DISTRO)-openvino-node_*.deb > /dev/null 2>&1; then \
	echo "Error: no packages in $(DIST_DIR). Run 'make package' first"; \
	exit 1; \
fi
endef

COLCON_DEPS := libopencv-dev libyaml-cpp-dev openvino-libraries-dev-$(OPENVINO_VERSION) \
	ros-$(ROS_DISTRO)-ament-cmake-gtest ros-$(ROS_DISTRO)-ament-lint-auto \
	ros-$(ROS_DISTRO)-ament-lint-common ros-$(ROS_DISTRO)-cv-bridge \
	ros-$(ROS_DISTRO)-librealsense2 ros-$(ROS_DISTRO)-yaml-cpp-vendor
COLCON_EVENT_HANDLERS := console_direct- compile_commands- console_cohesion- \
	console_package_list- console_start_end+ console_stderr+ desktop_notification- \
	event_log- log- log_command- status- store_result- summary+ terminal_title-
# The container runs as root: hand the generated directories back to the caller on exit.
COLCON_SETUP := trap 'chown -R $(HOST_UID):$(HOST_GID) /src/$(COLCON_DIR) /opt/openvino_toolkit 2> /dev/null || true' EXIT; \
	/src/script/packaging/setup_openvino_apt.sh && \
	apt-get -qq install -y --no-install-recommends $(COLCON_DEPS) && \
	cd /src && export COLCON_LOG_PATH=$(COLCON_DIR)/log && . /opt/ros/$(ROS_DISTRO)/setup.sh

.DEFAULT_GOAL := help
.PHONY: build clean clean-colcon clean-debian help lint lint-all lint-bash lint-clang \
	lint-githubactions lint-json lint-markdown lint-python lint-yaml package shell test \
	test-functional test-package

build:
	@# Help: Build the workspace with colcon
	$(DOCKER_RUN) -v $(ROOT_DIR):/src $(DOCKER_IMAGE) bash -c "$(COLCON_SETUP) && \
		colcon build $(COLCON_BASES) --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
		--event-handlers $(COLCON_EVENT_HANDLERS)"

test:
	@# Help: Build the workspace, download the models and run the unit tests with colcon
	mkdir -p $(MODELS_DIR)
	$(DOCKER_RUN) -v $(ROOT_DIR):/src -v $(MODELS_DIR):/opt/openvino_toolkit \
		-e OPENVINO_TOOLKIT_DIR=/opt/openvino_toolkit \
		$(DOCKER_IMAGE) bash -c "$(COLCON_SETUP) && \
		apt-get -qq install -y --no-install-recommends psmisc && \
		colcon build $(COLCON_BASES) --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
		--event-handlers $(COLCON_EVENT_HANDLERS) && \
		. $(COLCON_DIR)/install/setup.sh && \
		script/download_models.sh --all && \
		rc=0 && \
		{ colcon test $(COLCON_BASES) --packages-select openvino_test --ctest-args -R '^unittest_' \
		--return-code-on-test-failure --executor sequential \
		--event-handlers $(COLCON_EVENT_HANDLERS) || rc=\$$?; } && \
		{ colcon test-result --verbose --test-result-base $(COLCON_DIR)/build/openvino_test || true; } && \
		exit \$$rc"

package:
	@# Help: Build the Debian packages into dist/
	mkdir -p $(DIST_DIR)
	$(DOCKER_RUN) \
		-v $(ROOT_DIR):/src:ro \
		-v $(DIST_DIR):/out \
		-e DEB_BUILD_OPTIONS='nocheck parallel=$(MAKE_JOBS)' \
		-e HOST_UID=$(HOST_UID) -e HOST_GID=$(HOST_GID) \
		$(DOCKER_IMAGE) /src/script/packaging/build_debs.sh

test-package:
	@# Help: Install the Debian packages from dist/ in a clean container and check them
	$(CHECK_PACKAGES)
	$(DOCKER_RUN) \
		-v $(ROOT_DIR):/src:ro \
		-v $(DIST_DIR):/dist:ro \
		$(DOCKER_IMAGE) /src/script/packaging/test_debs.sh

test-functional:
	@# Help: Install the Debian packages from dist/ in a clean container and run the CI pipelines on DEVICE (default CPU)
	$(CHECK_PACKAGES)
	mkdir -p $(MODELS_DIR)
	$(DOCKER_RUN) $(DOCKER_RUNTIME_ARGS) \
		-v $(ROOT_DIR):/src:ro \
		-v $(DIST_DIR):/dist:ro \
		-e DEVICE='$(DEVICE)' -e HOST_UID=$(HOST_UID) -e HOST_GID=$(HOST_GID) \
		$(DOCKER_IMAGE) /src/script/packaging/test_functional.sh \
		$(if $(findstring GPU,$(DEVICE)),intel-opencl-icd)

shell:
	@# Help: Open a shell in a container with the Debian packages from dist/ installed, for manual tests
	$(CHECK_PACKAGES)
	mkdir -p $(MODELS_DIR)
	docker run --rm -it --platform linux/amd64 \
		-e http_proxy -e https_proxy -e no_proxy \
		$(DOCKER_RUNTIME_ARGS) $(DOCKER_DISPLAY_ARGS) \
		-v $(ROOT_DIR):/src:ro \
		-v $(DIST_DIR):/dist:ro \
		$(DOCKER_IMAGE) bash -c "trap 'chown -R $(HOST_UID):$(HOST_GID) /opt/openvino_toolkit' EXIT; \
		/src/script/packaging/install_debs.sh $(SHELL_PACKAGES) && { bash || true; }"

lint:
	@# Help: Run all sub-linters using super-linter
	VALIDATE_GITHUB_ACTIONS=true \
	VALIDATE_YAML=true \
	VALIDATE_JSON=true \
	VALIDATE_PYTHON_PYLINT=true \
	VALIDATE_PYTHON_FLAKE8=true \
	VALIDATE_BASH=true \
	VALIDATE_MARKDOWN=true \
	VALIDATE_CLANG_FORMAT=true \
		$(MAKE) lint-all

lint-all:
	@# Help: Run super-linter on the files changed against the default branch
	docker run --rm --platform linux/amd64 \
		-e SHELL=/bin/bash \
		-e RUN_LOCAL=true \
		--env-file $(ROOT_DIR)/.github/linters/super-linter.env \
		$(if $(VALIDATE_ALL_CODEBASE),-e VALIDATE_ALL_CODEBASE=$(VALIDATE_ALL_CODEBASE)) \
		$(if $(USE_FIND_ALGORITHM),-e USE_FIND_ALGORITHM=$(USE_FIND_ALGORITHM)) \
		$(if $(FILTER_REGEX_INCLUDE),-e FILTER_REGEX_INCLUDE='$(FILTER_REGEX_INCLUDE)') \
		$(if $(VALIDATE_GITHUB_ACTIONS),-e VALIDATE_GITHUB_ACTIONS=$(VALIDATE_GITHUB_ACTIONS)) \
		$(if $(VALIDATE_YAML),-e VALIDATE_YAML=$(VALIDATE_YAML)) \
		$(if $(VALIDATE_JSON),-e VALIDATE_JSON=$(VALIDATE_JSON)) \
		$(if $(VALIDATE_PYTHON_PYLINT),-e VALIDATE_PYTHON_PYLINT=$(VALIDATE_PYTHON_PYLINT)) \
		$(if $(VALIDATE_PYTHON_FLAKE8),-e VALIDATE_PYTHON_FLAKE8=$(VALIDATE_PYTHON_FLAKE8)) \
		$(if $(VALIDATE_BASH),-e VALIDATE_BASH=$(VALIDATE_BASH)) \
		$(if $(VALIDATE_MARKDOWN),-e VALIDATE_MARKDOWN=$(VALIDATE_MARKDOWN)) \
		$(if $(VALIDATE_CLANG_FORMAT),-e VALIDATE_CLANG_FORMAT=$(VALIDATE_CLANG_FORMAT)) \
		-v $(ROOT_DIR):/tmp/lint \
		ghcr.io/super-linter/super-linter:slim-v8.1.0

lint-bash:
	@# Help: Run Bash linter using super-linter
	VALIDATE_BASH=true $(MAKE) lint-all

lint-clang:
	@# Help: Run clang linter using super-linter
	VALIDATE_CLANG_FORMAT=true $(MAKE) lint-all

lint-githubactions:
	@# Help: Run Github Actions linter using super-linter
	VALIDATE_GITHUB_ACTIONS=true $(MAKE) lint-all

lint-json:
	@# Help: Run JSON linter using super-linter
	VALIDATE_JSON=true $(MAKE) lint-all

lint-markdown:
	@# Help: Run Markdown linter using super-linter
	VALIDATE_MARKDOWN=true $(MAKE) lint-all

lint-python:
	@# Help: Run Python linter using super-linter
	VALIDATE_PYTHON_PYLINT=true VALIDATE_PYTHON_FLAKE8=true $(MAKE) lint-all

lint-yaml:
	@# Help: Run YAML linter using super-linter
	VALIDATE_YAML=true $(MAKE) lint-all

clean-colcon:
	@# Help: Clean up the colcon artifacts of the build and test targets
	rm -rf $(ROOT_DIR)/$(COLCON_DIR)

clean-debian:
	@# Help: Clean up Debian packaging artifacts
	rm -rf $(DIST_DIR)

clean: clean-debian clean-colcon
	@# Help: Clean up all build artifacts

help:
	@printf "%-20s %s\n" "Target" "Description"
	@printf "%-20s %s\n" "------" "-----------"
	@grep -E '^[a-zA-Z0-9_%-]+:|^[[:space:]]+@# Help:' $(firstword $(MAKEFILE_LIST)) | \
	awk '\
		/^[a-zA-Z0-9_%-]+:/ { \
			target = $$1; \
			sub(":", "", target); \
		} \
		/^[[:space:]]+@# Help:/ { \
			if (target != "") { \
				help_line = $$0; \
				sub("^[[:space:]]+@# Help: ", "", help_line); \
				printf "%-20s %s\n", target, help_line; \
				target = ""; \
			} \
		}' | sort -k1,1

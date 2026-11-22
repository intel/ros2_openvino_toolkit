#!/usr/bin/env bash
# Copyright (C) 2026 Intel Corporation
#
# SPDX-License-Identifier: Apache-2.0
#
# Install the Debian packages from DIST_DIR and run the CI pipelines of
# openvino_node on DEVICE. A pipeline passes when it is created and a message
# arrives on each of its topics. Runs as root inside the clean container
# started by `make test-functional`; arguments are additional APT packages.
set -eo pipefail

SRC_DIR="${SRC_DIR:-/src}"
DEVICE="${DEVICE:-CPU}"
STARTUP_TIMEOUT="${STARTUP_TIMEOUT:-120}"
TOPIC_TIMEOUT="${TOPIC_TIMEOUT:-60}"
STOP_TIMEOUT=20

# "<launch file> <topic>...", limited to the models that download_models.sh provides.
PIPELINES=(
  "pipeline_image_ci_test.py /ros2_openvino_toolkit/emotions_recognition /ros2_openvino_toolkit/headposes_estimation /ros2_openvino_toolkit/people/age_genders_Recognition"
  "pipeline_people_ci_test.py /ros2_openvino_toolkit/face_detection /ros2_openvino_toolkit/emotions_recognition /ros2_openvino_toolkit/headposes_estimation /ros2_openvino_toolkit/age_genders_Recognition"
  "pipeline_person_attributes_ci_test.py /ros2_openvino_toolkit/detected_objects /ros2_openvino_toolkit/person_attributes"
  "pipeline_reidentification_ci_test.py /ros2_openvino_toolkit/reidentified_persons"
  "pipeline_segmentation_image_ci_test.py /ros2_openvino_toolkit/segmented_objects"
  "pipeline_vehicle_detection_ci_test.py /ros2_openvino_toolkit/detected_license_plates /ros2_openvino_toolkit/detected_vehicles_attribs"
)
# "<launch file> <model> <topic>...", run for the models that export_yolo26.sh has put into the model cache.
YOLO26_PIPELINES=(
  "pipeline_yolo26_detection_ci_test.py yolo26n /ros2_openvino_toolkit/detected_objects"
  "pipeline_yolo26_segmentation_ci_test.py yolo26n-seg /ros2_openvino_toolkit/segmented_objects"
  "pipeline_yolo26_pose_ci_test.py yolo26n-pose /ros2_openvino_toolkit/detected_keypoints"
  "pipeline_yolo26_obb_ci_test.py yolo26n-obb /ros2_openvino_toolkit/detected_oriented_objects"
  "pipeline_yolo26_classification_ci_test.py yolo26n-cls /ros2_openvino_toolkit/classified_objects"
  "pipeline_yolo26_semantic_ci_test.py yolo26n-sem /ros2_openvino_toolkit/segmented_objects"
  "pipeline_yolo26_depth_ci_test.py yolo26n-depth /ros2_openvino_toolkit/depth_image"
)

"$SRC_DIR/script/packaging/install_debs.sh" "$@"

# shellcheck source=/dev/null
. "/opt/ros/$ROS_DISTRO/setup.bash"
share_dir=$(ros2 pkg prefix --share openvino_node)
log_dir=$(mktemp -d)

# The models are downloaded as root into the mounted cache: hand it back on exit.
toolkit_dir="${OPENVINO_TOOLKIT_DIR:-/opt/openvino_toolkit}"
trap 'chown -R "${HOST_UID:-0}:${HOST_GID:-0}" "$toolkit_dir" 2> /dev/null || true' EXIT
ros2 run openvino_node download_models.sh --all

# Stop the process group of a launch, forcibly when it does not react to SIGINT.
stop_launch() {
  local pid=$1
  kill -INT -- "-$pid" 2> /dev/null || true
  for ((i = 0; i < STOP_TIMEOUT; i++)); do
    kill -0 "$pid" 2> /dev/null || break
    sleep 1
  done
  kill -KILL -- "-$pid" 2> /dev/null || true
  wait "$pid" 2> /dev/null || true
}

# Wait for one message. `ros2 topic echo` exits at once while the topic is not
# discovered yet, so it is repeated until the timeout expires.
wait_for_message() {
  local topic=$1
  local deadline=$((SECONDS + TOPIC_TIMEOUT))
  while ((SECONDS < deadline)); do
    if timeout "$((deadline - SECONDS))" ros2 topic echo --once "$topic" > /dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

run_pipeline() {
  local launch_file=$1
  shift
  local name="${launch_file%_test.py}"
  local log="$log_dir/$name.log"
  local yaml pid topic result=0

  # The CI launch files take the YAML file as it is: resolve its path
  # placeholders and the device with the launch helpers of the package.
  yaml=$(PYTHONPATH="$share_dir/launch:$PYTHONPATH" OPENVINO_DEVICE="$DEVICE" python3 -c \
    "import launch_helpers; print(launch_helpers.resolve_yaml_paths('$name.yaml'))" | tail -n 1)

  : > "$log"
  setsid ros2 launch openvino_node "$launch_file" "yaml_path:=$yaml" >> "$log" 2>&1 &
  pid=$!
  for ((i = 0; i < STARTUP_TIMEOUT; i++)); do
    if grep -q "One Pipeline Created!" "$log" || ! kill -0 "$pid" 2> /dev/null; then
      break
    fi
    sleep 1
  done

  if grep -q "One Pipeline Created!" "$log"; then
    grep -o "Engine created: .*" "$log" | sort | uniq -c | sed 's/^ */  /'
    for topic in "$@"; do
      if wait_for_message "$topic"; then
        echo "  PASS $topic"
      else
        echo "  FAIL $topic: no message within ${TOPIC_TIMEOUT}s"
        result=1
      fi
    done
  else
    echo "  FAIL the pipeline was not created (timeout ${STARTUP_TIMEOUT}s)"
    result=1
  fi
  if grep -q "\[ ERROR \]" "$log"; then
    echo "  FAIL the pipeline reported an error"
    result=1
  fi

  stop_launch "$pid"
  if [[ $result -ne 0 ]]; then
    echo "  --- last lines of the launch log ---"
    tail -n 30 "$log" | sed 's/^/  /'
  fi
  return "$result"
}

failed=0
total=0
for pipeline in "${PIPELINES[@]}"; do
  read -r -a fields <<< "$pipeline"
  echo "== ${fields[0]} on $DEVICE"
  total=$((total + 1))
  run_pipeline "${fields[@]}" || failed=$((failed + 1))
done
for pipeline in "${YOLO26_PIPELINES[@]}"; do
  read -r -a fields <<< "$pipeline"
  if [[ ! -s "$toolkit_dir/models/yolo26/${fields[1]}/FP16/${fields[1]}.xml" ]]; then
    echo "== ${fields[0]} skipped: ${fields[1]} is not exported, see export_yolo26.sh"
    continue
  fi
  echo "== ${fields[0]} on $DEVICE"
  total=$((total + 1))
  run_pipeline "${fields[0]}" "${fields[@]:2}" || failed=$((failed + 1))
done

if [[ $failed -ne 0 ]]; then
  echo "Functional test failed: $failed of $total pipelines." >&2
  exit 1
fi
echo "Functional test passed: $total pipelines on $DEVICE."

#!/usr/bin/env bash
# Get the OpenVINO models of the samples with one command.
#
# Usage:
#   setup_models.sh                                 # core set: all Intel models, SSD object detection, DeepLabv3, YOLO26 detection
#   setup_models.sh --all                           # every model that the scripts provide, incl. Mask R-CNN (large)
#   setup_models.sh --sample pipeline_segmentation  # models of one sample (launch file or YAML name, repeatable)
#   setup_models.sh --model <path> [<path>...]      # specific model files, e.g. as printed by a launch file
#   setup_models.sh --list                          # models of all samples and whether they are present
#   setup_models.sh --size s --precision INT8       # YOLO26 size and precision of the core set or --all
#
# Present models are skipped. It runs download_models.sh, export_yolo26.sh,
# export_ssdlite_mobilenet_v2.sh, export_deeplabv3.sh and export_maskrcnn.sh, which can also be used directly.
# Target directory: $OPENVINO_TOOLKIT_DIR, else $XDG_DATA_HOME/openvino_toolkit
# (default ~/.local/share/openvino_toolkit).
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOLKIT_DIR="${OPENVINO_TOOLKIT_DIR:-${XDG_DATA_HOME:-$HOME/.local/share}/openvino_toolkit}"

usage() {
  sed -n '2,/^set -uo pipefail$/p' "$0" | sed '$d; s/^# \{0,1\}//'
}

# Installed: <prefix>/lib/openvino_node -> <prefix>/share/openvino_node. Source tree: script/ -> sample/.
share_dir() {
  local dir
  for dir in "$SCRIPT_DIR/../../share/openvino_node" "$SCRIPT_DIR/../sample"; do
    if [[ -d "$dir/param" ]]; then
      echo "$dir"
      return 0
    fi
  done
  echo "Cannot find the param directory of openvino_node next to $SCRIPT_DIR" >&2
  return 1
}

# Model paths of a YAML file, with <OPENVINO_NODE_SHARE> resolved.
yaml_models() {
  sed -n 's/^[[:space:]]*model:[[:space:]]*\([^[:space:]#]*\).*/\1/p' "$1" |
    sed "s|<OPENVINO_NODE_SHARE>|$TOOLKIT_DIR|"
}

# YAML file of a sample given as YAML or launch file name, with or without extension.
sample_yaml() {
  local share name launch yaml
  share="$(share_dir)" || return 1
  name="$(basename "$1")"
  name="${name%.yaml}"
  name="${name%.launch.py}"
  if [[ -f "$share/param/$name.yaml" ]]; then
    echo "$share/param/$name.yaml"
    return 0
  fi
  launch="$share/launch/$name.launch.py"
  if [[ -f "$launch" ]]; then
    yaml="$(grep -o "[A-Za-z0-9_]*\.yaml" "$launch" | head -n 1)"
    if [[ -n "$yaml" && -f "$share/param/$yaml" ]]; then
      echo "$share/param/$yaml"
      return 0
    fi
  fi
  echo "Unknown sample: $1 (no $name.yaml or $name.launch.py in $share)" >&2
  return 1
}

# Sets KIND (intel, yolo26, ssdlite, deeplabv3, manual) and ARG for a model path.
classify() {
  local path="$1"
  KIND=manual
  ARG=""
  if [[ "$path" =~ /models/yolo26/yolo26([nslmx])([a-z-]*)/([^/]+)/ ]]; then
    local task=""
    case "${BASH_REMATCH[2]}" in
      "") task=detect ;;
      -seg) task=segment ;;
      -pose) task=pose ;;
      -obb) task=obb ;;
      -cls) task=classify ;;
      -sem) task=semantic ;;
      -depth) task=depth ;;
    esac
    if [[ -n "$task" ]]; then
      KIND=yolo26
      ARG="${BASH_REMATCH[1]} ${BASH_REMATCH[3]} $task"
    fi
  elif [[ "$path" == */models/convert/public/ssdlite_mobilenet_v2/* ]]; then
    KIND=ssdlite
  elif [[ "$path" == */models/convert/public/deeplabv3/* ]]; then
    KIND=deeplabv3
  elif [[ "$path" == */models/convert/public/mask_rcnn_inception_resnet_v2_atrous_coco/* ]]; then
    KIND=maskrcnn
  elif [[ "$path" =~ /models/intel/([^/]+)/([^/]+)/ ]]; then
    KIND=intel
    ARG="${BASH_REMATCH[1]}/${BASH_REMATCH[2]}"
  fi
}

intel=()
declare -A yolo=()
ssdlite=0
deeplabv3=0
maskrcnn=0
manual=()

plan_model() {
  local size precision task key
  [[ -s "$1" ]] && return 0
  classify "$1"
  case "$KIND" in
    intel) [[ " ${intel[*]} " == *" $ARG "* ]] || intel+=("$ARG") ;;
    yolo26)
      read -r size precision task <<< "$ARG"
      key="$size $precision"
      [[ ",${yolo[$key]:-}," == *",$task,"* ]] || yolo[$key]="${yolo[$key]:+${yolo[$key]},}$task"
      ;;
    ssdlite) ssdlite=1 ;;
    deeplabv3) deeplabv3=1 ;;
    maskrcnn) maskrcnn=1 ;;
    *) manual+=("$1") ;;
  esac
}

ok=()
failed=()

run_step() {
  local label="$1"
  shift
  echo
  echo "==> $label"
  if bash "$@"; then
    ok+=("$label")
  else
    failed+=("$label")
  fi
}

run_plan() {
  local key size precision
  if [[ ${#intel[@]} -gt 0 ]]; then
    run_step "Intel models: ${intel[*]}" "$SCRIPT_DIR/download_models.sh" "${intel[@]}"
  fi
  for key in "${!yolo[@]}"; do
    read -r size precision <<< "$key"
    run_step "YOLO26 $size $precision: ${yolo[$key]}" \
      "$SCRIPT_DIR/export_yolo26.sh" --tasks "${yolo[$key]}" --size "$size" --precision "$precision"
  done
  if [[ "$ssdlite" -eq 1 ]]; then
    run_step "ssdlite_mobilenet_v2" "$SCRIPT_DIR/export_ssdlite_mobilenet_v2.sh"
  fi
  if [[ "$deeplabv3" -eq 1 ]]; then
    run_step "deeplabv3" "$SCRIPT_DIR/export_deeplabv3.sh"
  fi
  if [[ "$maskrcnn" -eq 1 ]]; then
    run_step "mask_rcnn_inception_resnet_v2_atrous_coco" "$SCRIPT_DIR/export_maskrcnn.sh"
  fi
}

list_models() {
  local share path status
  share="$(share_dir)" || exit 1
  printf '%-8s %-10s %s\n' STATUS SOURCE MODEL
  while read -r path; do
    status=missing
    [[ -s "$path" ]] && status=present
    classify "$path"
    printf '%-8s %-10s %s\n' "$status" "$KIND" "${path#"$TOOLKIT_DIR"/}"
  done < <(for yaml in "$share"/param/*.yaml; do yaml_models "$yaml"; done | sort -u)
  echo
  echo "Models directory: $TOOLKIT_DIR/models"
}

mode=core
samples=()
models=()
yolo_opts=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --all)
      mode=all
      shift
      ;;
    --list)
      mode=list
      shift
      ;;
    --sample)
      mode=selected
      samples+=("${2:?--sample needs a name}")
      shift 2
      ;;
    --model)
      mode=selected
      shift
      while [[ $# -gt 0 && "$1" != --* ]]; do
        models+=("$1")
        shift
      done
      ;;
    --size | --precision)
      yolo_opts+=("$1" "${2:?$1 needs a value}")
      shift 2
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1 (see --help)" >&2
      exit 2
      ;;
  esac
done

case "$mode" in
  list)
    list_models
    exit 0
    ;;
  core)
    run_step "Intel models" "$SCRIPT_DIR/download_models.sh" --all
    run_step "ssdlite_mobilenet_v2" "$SCRIPT_DIR/export_ssdlite_mobilenet_v2.sh"
    run_step "deeplabv3" "$SCRIPT_DIR/export_deeplabv3.sh"
    run_step "YOLO26 detection" "$SCRIPT_DIR/export_yolo26.sh" "${yolo_opts[@]}"
    ;;
  all)
    run_step "Intel models (all)" "$SCRIPT_DIR/download_models.sh" --all
    run_step "ssdlite_mobilenet_v2" "$SCRIPT_DIR/export_ssdlite_mobilenet_v2.sh"
    run_step "deeplabv3" "$SCRIPT_DIR/export_deeplabv3.sh"
    run_step "YOLO26 (all tasks)" "$SCRIPT_DIR/export_yolo26.sh" --tasks all "${yolo_opts[@]}"
    run_step "mask_rcnn_inception_resnet_v2_atrous_coco" "$SCRIPT_DIR/export_maskrcnn.sh"
    ;;
  selected)
    for sample in "${samples[@]}"; do
      yaml="$(sample_yaml "$sample")" || exit 2
      while read -r path; do
        plan_model "$path"
      done < <(yaml_models "$yaml")
    done
    for path in "${models[@]}"; do
      plan_model "$path"
    done
    run_plan
    ;;
esac

echo
echo "Summary (models directory: $TOOLKIT_DIR/models)"
if [[ ${#ok[@]} -eq 0 && ${#failed[@]} -eq 0 && ${#manual[@]} -eq 0 ]]; then
  echo "  All requested models are present."
fi
for step in "${ok[@]}"; do
  echo "  OK      $step"
done
for step in "${failed[@]}"; do
  echo "  FAILED  $step"
done
for path in "${manual[@]}"; do
  echo "  MANUAL  $path (not provided by the scripts, convert it yourself)"
done
[[ ${#failed[@]} -eq 0 ]]

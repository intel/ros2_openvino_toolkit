#!/usr/bin/env bash
# Export Ultralytics YOLO26 models to OpenVINO IR for the YOLO26 pipelines.
#
# Usage:
#   export_yolo26.sh                            # detection model yolo26n, FP16
#   export_yolo26.sh --tasks detect,segment     # detect segment pose obb classify semantic depth, or all
#   export_yolo26.sh --size s --precision INT8  # size: n s m l x, precision: FP32 FP16 INT8
#
# The export runs in a Python virtual environment with the ultralytics package,
# which is created on first use (about 2 GB). Target directory as for
# download_models.sh: $OPENVINO_TOOLKIT_DIR, else $XDG_DATA_HOME/openvino_toolkit
# (default ~/.local/share/openvino_toolkit). Models land in
# <target>/models/yolo26/<name>/<precision>/ together with <name>.labels.
#
# The YOLO26 code and weights are provided by Ultralytics under AGPL-3.0 or an
# Ultralytics Enterprise license. This script downloads them; they are not part
# of this repository.
set -euo pipefail

ULTRALYTICS_VERSION="${ULTRALYTICS_VERSION:-8.4.216}"
# Same release as OPENVINO_VERSION in the Makefile, so that the runtime can read the exported IR.
OPENVINO_VERSION="${OPENVINO_VERSION:-2025.4.0}"
TOOLKIT_DIR="${OPENVINO_TOOLKIT_DIR:-${XDG_DATA_HOME:-$HOME/.local/share}/openvino_toolkit}"
DEST="$TOOLKIT_DIR/models/yolo26"
VENV_DIR="$TOOLKIT_DIR/venv/yolo26"
WORK_DIR="$TOOLKIT_DIR/cache/yolo26"

declare -A SUFFIX=([detect]="" [segment]="-seg" [pose]="-pose" [obb]="-obb"
  [classify]="-cls" [semantic]="-sem" [depth]="-depth")
ALL_TASKS=(detect segment pose obb classify semantic depth)
declare -A QUANTIZE=([FP32]=32 [FP16]=16 [INT8]=8)

tasks=(detect)
size=n
precision=FP16

while [[ $# -gt 0 ]]; do
  case "$1" in
    --tasks)
      IFS=',' read -r -a tasks <<< "${2:?--tasks needs a value}"
      shift 2
      ;;
    --size)
      size="${2:?--size needs a value}"
      shift 2
      ;;
    --precision)
      precision="${2:?--precision needs a value}"
      shift 2
      ;;
    -h | --help)
      sed -n '2,/^set -euo pipefail$/p' "$0" | sed '$d; s/^# \{0,1\}//'
      exit 0
      ;;
    *)
      echo "Unknown argument: $1 (see --help)" >&2
      exit 2
      ;;
  esac
done

if [[ "${tasks[*]}" == "all" ]]; then
  tasks=("${ALL_TASKS[@]}")
fi
for task in "${tasks[@]}"; do
  if [[ ! -v "SUFFIX[$task]" ]]; then
    echo "Unknown task: $task (valid: ${ALL_TASKS[*]}, all)" >&2
    exit 2
  fi
done
if [[ ! "$size" =~ ^[nsmlx]$ ]]; then
  echo "Unknown size: $size (valid: n s m l x)" >&2
  exit 2
fi
if [[ ! -v "QUANTIZE[$precision]" ]]; then
  echo "Unknown precision: $precision (valid: FP32 FP16 INT8)" >&2
  exit 2
fi

if [[ ! -x "$VENV_DIR/bin/yolo" ]]; then
  echo "Creating the Python environment in $VENV_DIR"
  python3 -m venv "$VENV_DIR"
  "$VENV_DIR/bin/pip" install --quiet --upgrade pip
  # CPU-only PyTorch: the export does not use a GPU.
  "$VENV_DIR/bin/pip" install --quiet torch torchvision --index-url https://download.pytorch.org/whl/cpu
  "$VENV_DIR/bin/pip" install --quiet "ultralytics[export-openvino]==$ULTRALYTICS_VERSION" \
    "openvino==$OPENVINO_VERSION"
fi

# Keep the pinned packages: ultralytics must not install or update anything by itself.
export YOLO_AUTOINSTALL=false
mkdir -p "$WORK_DIR"

if [[ "$precision" == "INT8" ]]; then
  echo "INT8 models are calibrated on the small default dataset of Ultralytics: check the accuracy before use."
fi

failed=0
for task in "${tasks[@]}"; do
  name="yolo26${size}${SUFFIX[$task]}"
  out="$DEST/$name/$precision"
  if [[ -s "$out/$name.xml" && -s "$out/$name.bin" && -e "$out/$name.labels" ]]; then
    echo "$name/$precision is already exported"
    continue
  fi

  args=("model=$name.pt" format=openvino "quantize=${QUANTIZE[$precision]}")
  # End-to-end head, the pipelines do not run NMS.
  case "$task" in
    detect | segment | pose | obb) args+=(nms=False) ;;
  esac

  echo "Exporting $name ($precision)"
  if ! (cd "$WORK_DIR" && "$VENV_DIR/bin/yolo" export "${args[@]}"); then
    echo "  FAILED: $name" >&2
    failed=1
    continue
  fi

  export_dir="$WORK_DIR/${name}_openvino_model"
  if [[ "$precision" == "INT8" ]]; then
    export_dir="$WORK_DIR/${name}_int8_openvino_model"
  fi
  mkdir -p "$out"
  cp "$export_dir/$name.xml" "$export_dir/$name.bin" "$export_dir/metadata.yaml" "$out/"
  "$VENV_DIR/bin/python" - "$out/metadata.yaml" "$out/$name.labels" << 'EOF'
import sys

from ultralytics.utils import YAML

names = YAML.load(sys.argv[1]).get("names") or {}
with open(sys.argv[2], "w", encoding="utf-8") as labels:
    labels.writelines(f"{names[class_id]}\n" for class_id in sorted(names))
EOF
done

echo "Models directory: $DEST"
echo "YOLO26 models are licensed by Ultralytics under AGPL-3.0: https://www.ultralytics.com/license"
exit "$failed"

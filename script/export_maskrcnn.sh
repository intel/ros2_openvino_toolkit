#!/usr/bin/env bash
# Convert the TensorFlow Mask R-CNN Inception ResNet v2 (COCO) model to OpenVINO IR (FP16).
#
# Usage:
#   export_maskrcnn.sh
#
# Output: <target>/models/convert/public/mask_rcnn_inception_resnet_v2_atrous_coco/FP16/
# Target directory as for download_models.sh: $OPENVINO_TOOLKIT_DIR, else
# $XDG_DATA_HOME/openvino_toolkit (default ~/.local/share/openvino_toolkit).
#
# The pipeline needs the IR layout of the legacy Model Optimizer (inputs image_tensor and
# image_info, outputs masks and reshape_do_2d), which ovc cannot produce. The conversion
# therefore uses openvino-dev 2024.6 with TensorFlow (about 2 GB, created on first use).
# openvino-dev 2024.6 needs Python 3.9-3.12: set MASKRCNN_PYTHON to such an interpreter
# if python3 is newer, e.g. after "uv python install 3.12".
set -euo pipefail

if [[ "$EUID" -eq 0 && -z "${OPENVINO_TOOLKIT_DIR:-}" ]]; then
  echo "Run as your user, not root: root would write to /root/.local (or set OPENVINO_TOOLKIT_DIR)" >&2
  exit 2
fi

MO_VERSION="${MO_VERSION:-2024.6.0}"
TOOLKIT_DIR="${OPENVINO_TOOLKIT_DIR:-${XDG_DATA_HOME:-$HOME/.local/share}/openvino_toolkit}"
VENV_DIR="$TOOLKIT_DIR/venv/maskrcnn"
WORK_DIR="$TOOLKIT_DIR/cache/maskrcnn"
NAME="mask_rcnn_inception_resnet_v2_atrous_coco"
OUT_DIR="$TOOLKIT_DIR/models/convert/public/$NAME/FP16"

MODEL="${NAME}_2018_01_28"
ARCHIVE="${MODEL}.tar.gz"
URL="http://download.tensorflow.org/models/object_detection/${ARCHIVE}"
ARCHIVE_SHA256="816afc8987c2152cbd0e24af3a6fb82086cea0645a38778884bd166c0d016bb5"

if [[ -s "$OUT_DIR/$NAME.xml" && -s "$OUT_DIR/$NAME.bin" ]]; then
  echo "$NAME/FP16 is already converted"
  exit 0
fi

if [[ ! -x "$VENV_DIR/bin/mo" ]]; then
  python=""
  for candidate in "${MASKRCNN_PYTHON:-}" python3.12 python3.11 python3.10 python3; do
    if [[ -n "$candidate" ]] && command -v "$candidate" > /dev/null &&
      "$candidate" -c 'import sys; sys.exit(not (3, 9) <= sys.version_info[:2] <= (3, 12))'; then
      python="$candidate"
      break
    fi
  done
  if [[ -z "$python" ]]; then
    echo "openvino-dev $MO_VERSION needs Python 3.9-3.12, but $(python3 --version) was found." >&2
    echo "Install one (e.g. 'uv python install 3.12') and set MASKRCNN_PYTHON to its path." >&2
    exit 2
  fi
  echo "Creating the Python environment in $VENV_DIR with $($python --version)"
  "$python" -m venv "$VENV_DIR"
  "$VENV_DIR/bin/pip" install --quiet --upgrade pip
  "$VENV_DIR/bin/pip" install --quiet "openvino-dev[tensorflow2]==$MO_VERSION"
fi

mkdir -p "$WORK_DIR" "$OUT_DIR"
cd "$WORK_DIR"

echo "[1/3] Downloading TensorFlow model (about 700 MB)..."
# curl, not wget: curl is a dependency of the openvino_node package and of the Docker image.
if [[ ! -s "$ARCHIVE" ]]; then
  curl -fSL --retry 3 -o "$ARCHIVE.part" "$URL"
  mv "$ARCHIVE.part" "$ARCHIVE"
fi
if ! echo "$ARCHIVE_SHA256  $ARCHIVE" | sha256sum -c --status; then
  echo "Checksum mismatch for $WORK_DIR/$ARCHIVE, delete it and retry" >&2
  exit 1
fi

echo "[2/3] Extracting model..."
tar -xzf "$ARCHIVE"

echo "[3/3] Converting to OpenVINO IR..."
MO_CONFIG="$("$VENV_DIR/bin/python" -c \
  'import os, openvino.tools.mo as mo; print(os.path.join(os.path.dirname(mo.__file__), "front/tf/mask_rcnn_support.json"))')"
"$VENV_DIR/bin/mo" \
  --input_model "$MODEL/frozen_inference_graph.pb" \
  --transformations_config "$MO_CONFIG" \
  --tensorflow_object_detection_api_pipeline_config "$MODEL/pipeline.config" \
  --input image_tensor --input_shape "[1,800,1365,3]" \
  --reverse_input_channels --compress_to_fp16 \
  --output_dir "$OUT_DIR" --model_name "$NAME"

echo "Conversion complete!"
echo "OpenVINO IR: $OUT_DIR/$NAME.xml"
echo "Weights:     $OUT_DIR/$NAME.bin"

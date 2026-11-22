#!/usr/bin/env bash
# Convert the TensorFlow ssdlite_mobilenet_v2 COCO model to OpenVINO IR (FP16).
#
# Usage:
#   export_ssdlite_mobilenet_v2.sh
#
# Output: <target>/models/convert/public/ssdlite_mobilenet_v2/FP16/ssdlite_mobilenet_v2.{xml,bin}
# Target directory as for download_models.sh: $OPENVINO_TOOLKIT_DIR, else
# $XDG_DATA_HOME/openvino_toolkit (default ~/.local/share/openvino_toolkit).
set -euo pipefail

if [[ "$EUID" -eq 0 ]]; then
  echo "Run as your user, not root: root would write to /root/.local" >&2
  exit 2
fi

OPENVINO_VERSION="${OPENVINO_VERSION:-2025.4.0}"
TOOLKIT_DIR="${OPENVINO_TOOLKIT_DIR:-${XDG_DATA_HOME:-$HOME/.local/share}/openvino_toolkit}"
VENV_DIR="$TOOLKIT_DIR/venv/ssdlite"
WORK_DIR="$TOOLKIT_DIR/cache/ssdlite_mobilenet_v2"
OUT_DIR="$TOOLKIT_DIR/models/convert/public/ssdlite_mobilenet_v2/FP16"

MODEL="ssdlite_mobilenet_v2_coco_2018_05_09"
ARCHIVE="${MODEL}.tar.gz"
URL="http://download.tensorflow.org/models/object_detection/${ARCHIVE}"
ARCHIVE_SHA256="542445cce834dbfbb7df1991425d475e85a2d7ec68c60a4f262bb18aac10c8b2"

if [[ ! -x "$VENV_DIR/bin/ovc" ]]; then
  echo "Creating the Python environment in $VENV_DIR"
  python3 -m venv "$VENV_DIR"
  "$VENV_DIR/bin/pip" install --quiet --upgrade pip
  "$VENV_DIR/bin/pip" install --quiet "openvino==$OPENVINO_VERSION"
fi

mkdir -p "$WORK_DIR" "$OUT_DIR"
cd "$WORK_DIR"

echo "[1/3] Downloading TensorFlow model..."
# curl, not wget: curl is a dependency of the openvino_node package and of the Docker image.
if [[ ! -s "$ARCHIVE" ]]; then
  curl -fsSL --retry 3 -o "$ARCHIVE.part" "$URL"
  mv "$ARCHIVE.part" "$ARCHIVE"
fi
if ! echo "$ARCHIVE_SHA256  $ARCHIVE" | sha256sum -c --status; then
  echo "Checksum mismatch for $WORK_DIR/$ARCHIVE, delete it and retry" >&2
  exit 1
fi

echo "[2/3] Extracting model..."
tar -xzf "$ARCHIVE"

echo "[3/3] Converting to OpenVINO IR..."
"$VENV_DIR/bin/ovc" "${MODEL}/frozen_inference_graph.pb" \
  --output_model "$OUT_DIR/ssdlite_mobilenet_v2.xml" \
  --compress_to_fp16

echo "Conversion complete!"
echo "OpenVINO IR: $OUT_DIR/ssdlite_mobilenet_v2.xml"
echo "Weights:     $OUT_DIR/ssdlite_mobilenet_v2.bin"

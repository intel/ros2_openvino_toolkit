#!/usr/bin/env bash
# Convert the TensorFlow DeepLabv3 (MobileNetV2, Pascal VOC) model to OpenVINO IR (FP16).
#
# Usage:
#   export_deeplabv3.sh
#
# Output: <target>/models/convert/public/deeplabv3/FP16/deeplabv3.{xml,bin}
# Target directory as for download_models.sh: $OPENVINO_TOOLKIT_DIR, else
# $XDG_DATA_HOME/openvino_toolkit (default ~/.local/share/openvino_toolkit).
set -euo pipefail

if [[ "$EUID" -eq 0 && -z "${OPENVINO_TOOLKIT_DIR:-}" ]]; then
  echo "Run as your user, not root: root would write to /root/.local (or set OPENVINO_TOOLKIT_DIR)" >&2
  exit 2
fi

OPENVINO_VERSION="${OPENVINO_VERSION:-2025.4.0}"
TOOLKIT_DIR="${OPENVINO_TOOLKIT_DIR:-${XDG_DATA_HOME:-$HOME/.local/share}/openvino_toolkit}"
VENV_DIR="$TOOLKIT_DIR/venv/deeplabv3"
WORK_DIR="$TOOLKIT_DIR/cache/deeplabv3"
OUT_DIR="$TOOLKIT_DIR/models/convert/public/deeplabv3/FP16"

MODEL="deeplabv3_mnv2_pascal_train_aug"
ARCHIVE="${MODEL}_2018_01_29.tar.gz"
URL="http://download.tensorflow.org/models/${ARCHIVE}"
ARCHIVE_SHA256="cd506941e4f88fd053903913761df2e76ea00c01079cecfd35855304a4a5fb48"

if [[ -s "$OUT_DIR/deeplabv3.xml" && -s "$OUT_DIR/deeplabv3.bin" ]]; then
  echo "deeplabv3/FP16 is already converted"
  exit 0
fi

if [[ ! -x "$VENV_DIR/bin/python" ]] || ! "$VENV_DIR/bin/python" -c "import openvino" 2>/dev/null; then
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
# Cut at ArgMax for a static [1,513,513] output (SemanticPredictions is dynamic), and
# take BGR input like the pipeline frames (the TensorFlow model expects RGB).
"$VENV_DIR/bin/python" - "${MODEL}/frozen_inference_graph.pb" "$OUT_DIR/deeplabv3.xml" <<'EOF'
import sys
import openvino as ov
from openvino.preprocess import PrePostProcessor

model = ov.convert_model(sys.argv[1], input=[1, 513, 513, 3])
argmax = next(op for op in model.get_ordered_ops() if op.get_friendly_name() == 'ArgMax')
model = ov.Model([argmax.output(0)], model.get_parameters(), 'deeplabv3')
ppp = PrePostProcessor(model)
ppp.input().tensor().set_layout(ov.Layout('NHWC'))
ppp.input().preprocess().reverse_channels()
ov.save_model(ppp.build(), sys.argv[2], compress_to_fp16=True)
EOF

echo "Conversion complete!"
echo "OpenVINO IR: $OUT_DIR/deeplabv3.xml"
echo "Weights:     $OUT_DIR/deeplabv3.bin"

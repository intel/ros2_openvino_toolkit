#!/usr/bin/env bash
# Download pre-converted OpenVINO IR models (Open Model Zoo "intel" models).
#
# Usage:
#   download_models.sh                 # models for the image/people pipelines
#   download_models.sh --all           # all supported intel models
#   download_models.sh name/PRECISION  # specific models, e.g. face-detection-adas-0001/FP16
#
# Target directory: $OPENVINO_TOOLKIT_DIR, else $XDG_DATA_HOME/openvino_toolkit
# (default ~/.local/share/openvino_toolkit). Models land in <target>/models/intel/.
set -euo pipefail

BASE_URL="${OPENVINO_MODEL_BASE_URL:-https://storage.openvinotoolkit.org/repositories/open_model_zoo/2023.0/models_bin/1}"
TOOLKIT_DIR="${OPENVINO_TOOLKIT_DIR:-${XDG_DATA_HOME:-$HOME/.local/share}/openvino_toolkit}"
DEST="$TOOLKIT_DIR/models/intel"

DEFAULT_MODELS=(
  face-detection-adas-0001/FP16
  age-gender-recognition-retail-0013/FP32
  emotions-recognition-retail-0003/FP32
  head-pose-estimation-adas-0001/FP32
)
ALL_MODELS=(
  "${DEFAULT_MODELS[@]}"
  face-detection-adas-0001/FP32
  landmarks-regression-retail-0009/FP32
  face-reidentification-retail-0095/FP32
  person-detection-retail-0013/FP32
  person-reidentification-retail-0277/FP32
  person-attributes-recognition-crossroad-0230/FP32
  vehicle-license-plate-detection-barrier-0106/FP32
  vehicle-attributes-recognition-barrier-0039/FP32
  license-plate-recognition-barrier-0001/FP32
  semantic-segmentation-adas-0001/FP16
)

if [[ "${1:-}" == "--all" ]]; then
  MODELS=("${ALL_MODELS[@]}")
elif [[ $# -gt 0 ]]; then
  MODELS=("$@")
else
  MODELS=("${DEFAULT_MODELS[@]}")
fi

failed=0
for entry in "${MODELS[@]}"; do
  name="${entry%%/*}"
  precision="${entry#*/}"
  out="$DEST/$name/$precision"
  mkdir -p "$out"
  for ext in xml bin; do
    file="$out/$name.$ext"
    if [[ -s "$file" ]]; then
      continue
    fi
    echo "Downloading $name/$precision/$name.$ext"
    if ! curl -fsSL --retry 3 -o "$file.part" "$BASE_URL/$name/$precision/$name.$ext"; then
      echo "  FAILED: $name/$precision/$name.$ext" >&2
      rm -f "$file.part"
      failed=1
      continue
    fi
    mv "$file.part" "$file"
  done
done

echo "Models directory: $TOOLKIT_DIR"
echo "Export OPENVINO_TOOLKIT_DIR=$TOOLKIT_DIR if you use a non-default location."
exit "$failed"

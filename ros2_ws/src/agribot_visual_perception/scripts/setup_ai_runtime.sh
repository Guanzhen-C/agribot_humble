#!/usr/bin/env bash

set -Eeuo pipefail

RUNTIME_ROOT="${AGRIBOT_VISION_RUNTIME_ROOT:-$HOME/.local/share/agribot}"
VENV_DIR="$RUNTIME_ROOT/vision_venv"
MODEL_DIR="$RUNTIME_ROOT/vision_models"
PACKAGE_VERSION="${AGRIBOT_VISION_PACKAGE_VERSION:-8.4.156}"

mkdir -p "$RUNTIME_ROOT" "$MODEL_DIR"
if [[ ! -x "$VENV_DIR/bin/python3" ]]; then
  python3 -m venv --system-site-packages "$VENV_DIR"
fi

"$VENV_DIR/bin/python3" -m pip install --upgrade pip
"$VENV_DIR/bin/python3" -m pip install \
  "numpy==1.26.4" \
  "opencv-python==4.10.0.84"
"$VENV_DIR/bin/python3" -m pip install \
  "torch==2.5.1" \
  "torchvision==0.20.1" \
  --index-url https://download.pytorch.org/whl/cu121
"$VENV_DIR/bin/python3" -m pip install \
  "cloudpickle>=3.1.1" \
  "nvidia-ml-py>=12.0.0" \
  "polars==1.44.2" \
  "ultralytics-thop>=2.1.6"
"$VENV_DIR/bin/python3" -m pip install \
  "ultralytics==$PACKAGE_VERSION" \
  --no-deps

MODEL_DIR="$MODEL_DIR" "$VENV_DIR/bin/python3" - <<'PY'
import os
from pathlib import Path

from ultralytics import YOLO

model_dir = Path(os.environ["MODEL_DIR"]).expanduser().resolve()
model_dir.mkdir(parents=True, exist_ok=True)
models = ("yolo26n.pt", "yolo26n-seg.pt", "yolo26n-pose.pt")
old_cwd = Path.cwd()
try:
    os.chdir(model_dir)
    for filename in models:
        path = model_dir / filename
        if not path.is_file():
            YOLO(filename)
        if not path.is_file():
            raise RuntimeError(f"model download did not create {path}")
        print(f"MODEL_READY={path}")
finally:
    os.chdir(old_cwd)
PY

printf 'VISION_PYTHON=%s\n' "$VENV_DIR/bin/python3"
printf 'VISION_MODEL_DIR=%s\n' "$MODEL_DIR"

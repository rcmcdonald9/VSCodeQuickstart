#!/usr/bin/env bash
# Downloads YOLO11n and converts it to NCNN (about 2-3x faster than PyTorch on a Pi 5 CPU).
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
cd "$HERE"
PY="$HERE/.venv/bin/python"; [ -x "$PY" ] || PY=python3
"$PY" - <<'PYEOF'
from ultralytics import YOLO
YOLO("yolo11n.pt").export(format="ncnn", imgsz=480)
PYEOF
echo "model at $HERE/yolo11n_ncnn_model (config.yaml: vision.yolo_model)"

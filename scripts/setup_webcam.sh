#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m venv --system-site-packages "$workspace_dir/.venv"
"$workspace_dir/.venv/bin/python3" -m pip install -r "$workspace_dir/src/g1_motion_demo/requirements-webcam.txt"
mkdir -p "$workspace_dir/models"
"$workspace_dir/.venv/bin/python3" - "$workspace_dir/models/pose_landmarker_lite.task" <<'PY'
from pathlib import Path
import sys
import urllib.request

destination = Path(sys.argv[1])
if not destination.exists():
    url = ('https://storage.googleapis.com/mediapipe-models/pose_landmarker/'
           'pose_landmarker_lite/float16/1/pose_landmarker_lite.task')
    temporary = destination.with_suffix('.download')
    urllib.request.urlretrieve(url, temporary)
    temporary.replace(destination)
print(f'Webcam model: {destination}')
PY

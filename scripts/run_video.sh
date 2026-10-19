#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VIDEO="${1:-${ROOT}/data/original.mp4}"
RUN="${2:-${ROOT}/outputs/exhibition}"
PYTHON="${ROOT}/.venv/bin/python"
if [[ ! -x "${PYTHON}" || ! -x "${ROOT}/build/mono_video" ]]; then
  echo "Run bash scripts/bootstrap_autodl.sh first." >&2
  exit 2
fi
export LD_LIBRARY_PATH="${ROOT}/third_party/install/lib:${ROOT}/third_party/install/lib64:${ROOT}/third_party/ORB_SLAM3/lib:${LD_LIBRARY_PATH:-}"
export QT_QPA_PLATFORM=offscreen
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
"${PYTHON}" "${ROOT}/prepare_video.py" --video "${VIDEO}" --output "${RUN}/dataset"
mkdir -p "${RUN}/raw"
if [[ -f "${RUN}/raw/tracking.csv" ]]; then
  echo "Results already exist at ${RUN}; choose another output directory." >&2
  exit 2
fi
"${ROOT}/build/mono_video" "${ROOT}/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
  "${RUN}/dataset/camera.yaml" "${RUN}/dataset" "${RUN}/raw" 1 \
  2>&1 | tee "${RUN}/orb_slam3.log"
"${PYTHON}" "${ROOT}/export_map.py" --dataset "${RUN}/dataset" --raw "${RUN}/raw" --output "${RUN}/replay"
"${PYTHON}" "${ROOT}/render_video.py" --dataset "${RUN}/dataset" --replay "${RUN}/replay" --output "${RUN}/slam_demo.mp4"
echo "Done: ${RUN}/slam_demo.mp4"

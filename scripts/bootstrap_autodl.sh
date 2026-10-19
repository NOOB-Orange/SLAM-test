#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JOBS="${JOBS:-4}"
source /etc/os-release
if [[ "${ID}" != "ubuntu" || "${VERSION_ID}" != "22.04" ]]; then
  echo "This setup targets Ubuntu 22.04. Select an Ubuntu 22.04 AutoDL image. Detected: ${PRETTY_NAME}" >&2
  exit 2
fi
if [[ "${EUID}" -eq 0 ]]; then SUDO=(); else SUDO=(sudo); fi
"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y --no-install-recommends \
  build-essential cmake git pkg-config ca-certificates python3-dev python3-venv \
  libeigen3-dev libopencv-dev libboost-serialization-dev libssl-dev \
  libglew-dev libgl1-mesa-dev libegl1-mesa-dev libx11-dev libepoxy-dev \
  libjpeg-dev libpng-dev libavcodec-dev libavformat-dev libavutil-dev libswscale-dev
cd "${ROOT}"
/usr/bin/python3 -m venv .venv
PYTHON="${ROOT}/.venv/bin/python"
"${PYTHON}" -m pip install --upgrade pip
"${PYTHON}" -m pip install -r requirements.txt
"${PYTHON}" scripts/fetch_dependencies.py all
"${PYTHON}" scripts/patch_upstream.py
CMAKE_BIN="${CMAKE_BIN:-/usr/bin/cmake}"
PANGO="${ROOT}/third_party/Pangolin"
PREFIX="${ROOT}/third_party/install"
"${CMAKE_BIN}" -S "${PANGO}" -B "${PANGO}/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DCMAKE_CXX_STANDARD=17 -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF \
  -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF \
  -DBUILD_PANGOLIN_FFMPEG=OFF -DBUILD_PANGOLIN_LIBOPENEXR=OFF
"${CMAKE_BIN}" --build "${PANGO}/build" --parallel "${JOBS}"
"${CMAKE_BIN}" --install "${PANGO}/build"
ORB="${ROOT}/third_party/ORB_SLAM3"
for dependency in DBoW2 g2o; do
  "${CMAKE_BIN}" -S "${ORB}/Thirdparty/${dependency}" -B "${ORB}/Thirdparty/${dependency}/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17
  "${CMAKE_BIN}" --build "${ORB}/Thirdparty/${dependency}/build" --parallel "${JOBS}"
done
# Sophus is header-only for ORB-SLAM3; do not build its unrelated tests/examples.
if [[ ! -s "${ORB}/Vocabulary/ORBvoc.txt" ]]; then
  tar -xzf "${ORB}/Vocabulary/ORBvoc.txt.tar.gz" -C "${ORB}/Vocabulary"
fi
"${CMAKE_BIN}" -S "${ORB}" -B "${ORB}/build" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 -DCMAKE_PREFIX_PATH="${PREFIX}"
"${CMAKE_BIN}" --build "${ORB}/build" --target ORB_SLAM3 --parallel "${JOBS}"
"${CMAKE_BIN}" -S "${ROOT}/cpp" -B "${ROOT}/build" -DCMAKE_BUILD_TYPE=Release \
  -DORB_SLAM3_ROOT="${ORB}" -DCMAKE_PREFIX_PATH="${PREFIX}"
"${CMAKE_BIN}" --build "${ROOT}/build" --parallel "${JOBS}"
"${PYTHON}" -m unittest discover -s tests -v
echo "Build complete. Next: bash scripts/run_video.sh"

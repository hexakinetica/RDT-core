#!/usr/bin/env bash
# install_prereqs.sh
# 3rd-party deps for FULL OCCT + toolchain + Qt/KDL/tinyxml2
set -euo pipefail

echo "== [Prereqs] Updating apt index =="
sudo apt-get update

echo "== [Prereqs] Toolchain & helpers =="
sudo apt-get install -y \
  build-essential cmake ninja-build pkg-config git curl unzip ca-certificates \
  gcc g++ make software-properties-common

echo "== [Prereqs] X11/OpenGL stack (Visualization) =="
sudo apt-get install -y \
  libx11-dev libxext-dev libxmu-dev libxi-dev libxrender-dev \
  libxrandr-dev libxau-dev libxdmcp-dev \
  libgl1-mesa-dev libglu1-mesa-dev

echo "== [Prereqs] Image/Font/TBB/Vector export (DataExchange/Vis extras) =="
sudo apt-get install -y \
  libfreetype6-dev libtbb-dev libfreeimage-dev libgl2ps-dev \
  zlib1g-dev libexpat1-dev

echo "== [Prereqs] glTF support (RapidJSON + Draco) =="
sudo apt-get install -y rapidjson-dev libdraco-dev

echo "== [Prereqs] OCCT DRAW (dev console) =="
sudo apt-get install -y tcl-dev tk-dev

echo "== [Prereqs] Optional: OpenCL (enable if GPU ICD present) =="
sudo apt-get install -y ocl-icd-opencl-dev || true

echo "== [Prereqs] Qt6 headers via apt (runtime deps подтянутся автоматически) =="
sudo apt-get install -y qt6-base-dev qt6-base-dev-tools libqt6opengl6-dev

echo "== [Prereqs] Versions =="
cmake --version
g++ --version
pkg-config --version

echo "== [Prereqs] OK =="


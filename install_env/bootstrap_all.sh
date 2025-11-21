#!/usr/bin/env bash
set -euo pipefail

# --- ВАЖНОЕ ИЗМЕНЕНИЕ ---
# Указываем путь к текущей директории и загружаем переменные
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT_DIR/versions.env"
# -------------------------

echo "== RDT environment bootstrap (Linux) =="
echo "Install prefix: ${PREFIX}"
echo "Eigen: ${EIGEN_VERSION}, KDL: ${KDL_VERSION}, tinyxml2: ${TINYXML2_VERSION}, OCCT: ${OCCT_VERSION}"
echo

# 1) 3rd-party deps
./install_prereqs.sh

# 2) Eigen
./install_eigen.sh

# 3) Orocos-KDL
./install_kdl.sh

# 4) tinyxml2
./install_tinyxml2.sh

# 5) Qt6 (apt-based)
./install_qt6.sh

# 6) OpenCASCADE
./install_occt.sh

# 7)
./verify_all.sh

echo
echo "== ALL DONE ✅ =="
echo "You can do"
echo "  cmake -S . -B build -DCMAKE_PREFIX_PATH="$PREFIX""

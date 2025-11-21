#!/usr/bin/env bash
# install_kdl.sh
set -euo pipefail

# Если переменные не установлены, пытаемся загрузить их из versions.env
if [[ -z "${KDL_VERSION:-}" ]]; then
  echo "WARN: KDL_VERSION is not set. Attempting to source from versions.env"
  source "$(dirname "${BASH_SOURCE[0]}")/versions.env"
fi

echo "== [KDL] Installing v${KDL_VERSION} to ${PREFIX} =="

TMP=/tmp/kdl && rm -rf "$TMP" && mkdir -p "$TMP"
git clone --depth 1 --branch "v${KDL_VERSION}" https://github.com/orocos/orocos_kinematics_dynamics.git "$TMP/src"

cmake -S "$TMP/src/orocos_kdl" -B "$TMP/build" \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DCMAKE_PREFIX_PATH="$PREFIX" \
  -DBUILD_TESTING=OFF

cmake --build "$TMP/build" -j
cmake --install "$TMP/build"
sudo ldconfig

# --- НАЧАЛО ИСПРАВЛЕННОГО БЛОКА ПРОВЕРКИ ---
echo "== [KDL] Verifying installation =="
CHECK=/tmp/kdl_check && rm -rf "$CHECK" && mkdir -p "$CHECK"
# Убираем кавычки у CMAKE, чтобы ${PREFIX} подставился
cat > "$CHECK/CMakeLists.txt" <<CMAKE
cmake_minimum_required(VERSION 3.16)
project(kdl_probe LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

# Эта строка теперь будет содержать правильный путь
set(CMAKE_PREFIX_PATH "${PREFIX}")

find_package(orocos_kdl REQUIRED)

add_executable(kdl_probe main.cpp)

# Экранируем $, чтобы bash не трогал эти переменные CMake
target_include_directories(kdl_probe PRIVATE \${orocos_kdl_INCLUDE_DIRS})
target_link_libraries(kdl_probe PRIVATE \${orocos_kdl_LIBRARIES})
CMAKE
cat > "$CHECK/main.cpp" <<'CPP'
#include <kdl/frames.hpp>
#include <iostream>
int main(){
    KDL::Vector v(1,2,3);
    std::cout << "KDL vector norm: " << v.Norm() << std::endl;
    if (v.Norm() > 0) {
        return 0; // Success
    }
    return 1; // Failure
}
CPP

echo "--- [KDL] Running check build ---"
# Передаем PREFIX и здесь, для максимальной надежности
cmake -S "$CHECK" -B "$CHECK/build" -DCMAKE_PREFIX_PATH="$PREFIX"
cmake --build "$CHECK/build" -j

echo "--- [KDL] Executing check ---"
export LD_LIBRARY_PATH="${PREFIX}/lib:\${LD_LIBRARY_PATH:-}"
"$CHECK/build/kdl_probe"

echo "== Orocos-KDL ${KDL_VERSION} OK =="
# --- КОНЕЦ ИСПРАВЛЕННОГО БЛОКА ПРОВЕРКИ ---

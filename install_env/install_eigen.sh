#!/usr/bin/env bash
# install_eigen.sh
set -euo pipefail

echo "== [Eigen] Installing v${EIGEN_VERSION} to ${PREFIX} =="

TMP=/tmp/eigen && rm -rf "$TMP" && mkdir -p "$TMP"
curl -L -o "$TMP/eigen.tar.gz" "https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz"
tar -xf "$TMP/eigen.tar.gz" -C "$TMP"

cmake -S "$TMP/eigen-${EIGEN_VERSION}" -B "$TMP/build" \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DBUILD_TESTING=OFF

cmake --build "$TMP/build" -j
# sudo не нужен для установки в пользовательскую директорию
cmake --install "$TMP/build"

# Проверка find_package(Eigen3 ...)
CHECK=/tmp/eigen_check && rm -rf "$CHECK" && mkdir -p "$CHECK"
cat > "$CHECK/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.16)
project(eigen_probe LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_PREFIX_PATH "${PREFIX}")
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
add_executable(eigen_probe main.cpp)
target_link_libraries(eigen_probe PRIVATE Eigen3::Eigen)
CMAKE
cat > "$CHECK/main.cpp" <<'CPP'
#include <Eigen/Dense>
#include <iostream>
int main(){ Eigen::Vector3d v(1,2,3); std::cout << "Norm: " << v.norm() << std::endl; return 0; }
CPP
# Передаем PREFIX в CMAKE_PREFIX_PATH для корректного поиска
cmake -S "$CHECK" -B "$CHECK/build" -DCMAKE_PREFIX_PATH="$PREFIX"
cmake --build "$CHECK/build" -j
"$CHECK/build/eigen_probe"
echo "== Eigen ${EIGEN_VERSION} OK =="

#!/usr/bin/env bash
set -euo pipefail

# Загружаем переменную PREFIX
source "$(dirname "${BASH_SOURCE[0]}")/versions.env"

echo "== [Verify] Unified CMake probe for Qt6/Eigen/KDL/tinyxml2/OCCT =="

CHECK=/tmp/rdt_full_probe && rm -rf "$CHECK" && mkdir -p "$CHECK"
cat > "$CHECK/CMakeLists.txt" <<CMAKE
cmake_minimum_required(VERSION 3.20)
project(rdt_full_probe LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_PREFIX_PATH "${PREFIX}")

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGL)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(orocos_kdl REQUIRED)
find_package(tinyxml2 CONFIG REQUIRED)
find_package(OpenCASCADE REQUIRED)

add_executable(rdt_full_probe main.cpp)

# --- ФИНАЛЬНОЕ ИСПРАВЛЕНИЕ ЗДЕСЬ ---
# Явно добавляем пути к заголовочным файлам OCCT
target_include_directories(rdt_full_probe PRIVATE \${OpenCASCADE_INCLUDE_DIRS})

# Линкуемся со всеми остальными библиотеками как и раньше,
# но для OCCT используем "старую" переменную \${OpenCASCADE_LIBRARIES},
# которая содержит список всех нужных .so файлов.
target_link_libraries(rdt_full_probe PRIVATE
  Qt6::Core Qt6::Gui Qt6::Widgets Qt6::OpenGL
  Eigen3::Eigen
  orocos-kdl
  tinyxml2::tinyxml2
  \${OpenCASCADE_LIBRARIES}
)
CMAKE
cat > "$CHECK/main.cpp" <<'CPP'
#include <QApplication>
#include <QWidget>
#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <tinyxml2.h>
#include <Standard_Version.hxx>
#include <iostream>

int main(int argc, char** argv){
  std::cout << "--- Probing Libraries ---" << std::endl;
  std::cout << "Eigen...";
  Eigen::Vector3d v(1,2,3);
  if(v.norm() <= 0) return 1;
  std::cout << " OK" << std::endl;

  std::cout << "KDL...";
  KDL::Vector vk(1,2,3);
  if(vk.Norm() <= 0) return 2;
  std::cout << " OK" << std::endl;

  std::cout << "tinyxml2...";
  tinyxml2::XMLDocument d;
  d.Parse("<ok/>");
  if (d.Error()) return 3;
  std::cout << " OK" << std::endl;

  std::cout << "OpenCASCADE version: " << OCC_VERSION_STRING_EXT << std::endl;
  std::cout << "Qt...";
  QApplication app(argc,argv);
  std::cout << " OK" << std::endl;
  std::cout << "--- All Probes Passed ---" << std::endl;
  return 0;
}
CPP

echo "--- [Verify] Building the final probe ---"
cmake -S "$CHECK" -B "$CHECK/build" -DCMAKE_PREFIX_PATH="${PREFIX}"
cmake --build "$CHECK/build" -j

echo "--- [Verify] Executing the final probe ---"
export LD_LIBRARY_PATH="${PREFIX}/lib:\${LD_LIBRARY_PATH:-}"
"$CHECK/build/rdt_full_probe"

echo "== [Verify] DONE =="

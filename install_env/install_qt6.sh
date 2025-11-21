#!/usr/bin/env bash
# install_qt6.sh
set -euo pipefail

sudo apt-get update
sudo apt-get install -y qt6-base-dev qt6-base-dev-tools libqt6opengl6-dev

# Проверка find_package(Qt6 ...)
TMP=/tmp/qt6_check && rm -rf "$TMP" && mkdir -p "$TMP"
cat > "$TMP/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.20)
project(qt6_probe LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets)
add_executable(qt6_probe main.cpp)
target_link_libraries(qt6_probe PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets)
CMAKE
cat > "$TMP/main.cpp" <<'CPP'
#include <QApplication>
#include <QWidget>
int main(int argc, char** argv){ QApplication app(argc, argv); QWidget w; return 0; }
CPP
cmake -S "$TMP" -B "$TMP/build"
cmake --build "$TMP/build" -j
echo "== Qt6 OK =="


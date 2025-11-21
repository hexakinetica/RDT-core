#!/usr/bin/env bash
# install_tinyxml2.sh
set -euo pipefail

# Защита от запуска без sourcing'а versions.env
if [[ -z "${TINYXML2_VERSION:-}" ]]; then
  echo "WARN: TINYXML2_VERSION is not set. Attempting to source from versions.env"
  source "$(dirname "${BASH_SOURCE[0]}")/versions.env"
fi

echo "== [tinyxml2] Installing v${TINYXML2_VERSION} to ${PREFIX} =="

TMP=/tmp/tinyxml2 && rm -rf "$TMP" && mkdir -p "$TMP"
git clone --depth 1 --branch "${TINYXML2_VERSION}" https://github.com/leethomason/tinyxml2.git "$TMP/src"

cmake -S "$TMP/src" -B "$TMP/build" \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DBUILD_TESTS=OFF \
    -DBUILD_SHARED_LIBS=ON

cmake --build "$TMP/build" -j
# sudo не нужен для установки в домашнюю директорию
cmake --install "$TMP/build"
sudo ldconfig

# --- ОБНОВЛЕННЫЙ БЛОК ПРОВЕРКИ ---
echo "== [tinyxml2] Verifying installation =="
CHECK=/tmp/txml_check && rm -rf "$CHECK" && mkdir -p "$CHECK"
# Убираем кавычки у CMAKE, чтобы ${PREFIX} подставился
cat > "$CHECK/CMakeLists.txt" <<CMAKE
cmake_minimum_required(VERSION 3.16)
project(txml_probe LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_PREFIX_PATH "${PREFIX}")
find_package(tinyxml2 CONFIG REQUIRED)
add_executable(txml_probe main.cpp)
target_link_libraries(txml_probe PRIVATE tinyxml2::tinyxml2)
CMAKE
cat > "$CHECK/main.cpp" <<'CPP'
#include <tinyxml2.h>
#include <iostream>
int main(){
    tinyxml2::XMLDocument doc;
    if (doc.Parse("<ok/>") == tinyxml2::XML_SUCCESS) {
        std::cout << "tinyxml2 parsed string successfully." << std::endl;
        return 0;
    }
    return 1;
}
CPP

echo "--- [tinyxml2] Running check build ---"
cmake -S "$CHECK" -B "$CHECK/build" -DCMAKE_PREFIX_PATH="${PREFIX}"
cmake --build "$CHECK/build" -j

echo "--- [tinyxml2] Executing check ---"
export LD_LIBRARY_PATH="${PREFIX}/lib:\${LD_LIBRARY_PATH:-}"
"$CHECK/build/txml_probe"

echo "== tinyxml2 ${TINYXML2_VERSION} OK =="

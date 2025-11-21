#!/usr/bin/env bash
# install_occt.sh
set -euo pipefail

if [[ -z "${OCCT_VERSION:-}" ]]; then
  echo "WARN: OCCT_VERSION is not set. Attempting to source from versions.env"
  source "$(dirname "${BASH_SOURCE[0]}")/versions.env"
fi

echo "== [OCCT] Full install V${OCCT_VERSION} to ${PREFIX} =="

TMP=/tmp/occt && rm -rf "$TMP" && mkdir -p "$TMP"
TAG="V${OCCT_VERSION//./_}"
curl -L -o "$TMP/occt.tar.gz" "https://github.com/Open-Cascade-SAS/OCCT/archive/refs/tags/${TAG}.tar.gz"
tar -xf "$TMP/occt.tar.gz" -C "$TMP"

OCCT_SOURCE_DIR=$(find "$TMP" -mindepth 1 -maxdepth 1 -type d)
if [ ! -d "$OCCT_SOURCE_DIR" ]; then
    echo "FATAL: Could not find the extracted OCCT source directory in $TMP"
    exit 1
fi
echo "Found OCCT source directory: ${OCCT_SOURCE_DIR}"

USE_OPENCL="OFF"
if pkg-config --exists OpenCL 2>/dev/null || [[ -f /usr/include/CL/opencl.h || -f /usr/include/CL/cl.h ]]; then
  USE_OPENCL="ON"
  echo "Found OpenCL, enabling it for OCCT."
fi

# Конфигурация CMake остается прежней
cmake -S "${OCCT_SOURCE_DIR}" -B "$TMP/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DINSTALL_DIR="$PREFIX" \
  -DUSE_TBB=ON \
  -DTBB_INCLUDE_DIR="/usr/include" \
  -DTBB_LIBRARY_DIR="/usr/lib/x86_64-linux-gnu" \
  -DBUILD_MODULE_FoundationClasses=ON \
  -DBUILD_MODULE_ModelingData=ON \
  -DBUILD_MODULE_ModelingAlgorithms=ON \
  -DBUILD_MODULE_ApplicationFramework=ON \
  -DBUILD_MODULE_Visualization=ON \
  -DBUILD_MODULE_DataExchange=ON \
  -DBUILD_MODULE_Draw=ON \
  -DUSE_OPENGL=ON \
  -DUSE_XLIB=ON \
  -DUSE_FREETYPE=ON \
  -DUSE_FREEIMAGE=ON \
  -DUSE_GL2PS=ON \
  -DUSE_OPENCL=${USE_OPENCL} \
  -DUSE_RAPIDJSON=ON \
  -DUSE_DRACO=ON \
  -DUSE_VTK=OFF \
  -DBUILD_Inspector=OFF \
  -DBUILD_DOC_Overview=OFF \
  -DBUILD_LIBRARY_TYPE=Shared

# --- ГЛАВНОЕ ИЗМЕНЕНИЕ: ДИАГНОСТИЧЕСКИЙ РЕЖИМ СБОРКИ ---
LOG_FILE="$TMP/build_log.txt"
echo "Starting OCCT build... All output will be logged to ${LOG_FILE}"

# Запускаем сборку и перенаправляем ВЕСЬ вывод в лог-файл.
# Если команда завершится с ошибкой (||), то выполнится блок диагностики.
cmake --build "$TMP/build" -j8 > "${LOG_FILE}" 2>&1 || {
    echo ""
    echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    echo "!! СБОРКА OCCT ПРОВАЛИЛАСЬ."
    echo "!! Ниже приведены ПЕРВЫЕ 100 строк из лога сборки."
    echo "!! НАСТОЯЩАЯ ПРИЧИНА ОШИБКИ НАХОДИТСЯ ЗДЕСЬ."
    echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    echo ""
    head -n 100 "${LOG_FILE}"
    echo ""
    echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    echo "!! Полный лог находится в файле: ${LOG_FILE}"
    echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    exit 1
}

echo "OCCT build completed successfully."

# Если сборка прошла успешно, продолжаем как обычно.
cmake --install "$TMP/build"
sudo ldconfig

# --- НАЧАЛО НОВОГО, НАДЁЖНОГО БЛОКА ПРОВЕРКИ ---
echo "== [OCCT] Verifying installation =="
 
 
# --- КОНЕЦ БЛОКА ПРОВЕРКИ ---

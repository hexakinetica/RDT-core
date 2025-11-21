#!/usr/bin/env bash
set -euo pipefail

# Загружаем переменную PREFIX, чтобы знать, где всё установлено
source "$(dirname "${BASH_SOURCE[0]}")/versions.env"

echo "== [OCCT Visualization Test - The Stable Way] =="
CHECK=/tmp/occt_stable_visu_check && rm -rf "$CHECK" && mkdir -p "$CHECK"

# --- ШАГ 1: CMakeLists.txt ---
# Явно находим и линкуемся с X11
cat > "$CHECK/CMakeLists.txt" <<CMAKE
cmake_minimum_required(VERSION 3.20)
project(occt_stable_visu_probe LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(CMAKE_PREFIX_PATH "${PREFIX}")

find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGLWidgets)
find_package(OpenCASCADE REQUIRED HINTS "\${CMAKE_PREFIX_PATH}" NO_CMAKE_PACKAGE_REGISTRY)
find_package(X11 REQUIRED)

add_executable(occt_stable_visu_probe main.cpp)

target_include_directories(occt_stable_visu_probe PRIVATE
    \${OpenCASCADE_INCLUDE_DIRS}
    \${X11_INCLUDE_DIR}
)
target_link_libraries(occt_stable_visu_probe PRIVATE
    Qt6::Widgets
    Qt6::OpenGLWidgets
    \${OpenCASCADE_LIBRARIES}
    \${X11_LIBRARIES}
)
CMAKE

# --- ШАГ 2: Исходный код C++ (надежный метод с Xw_Window и защитой от макросов) ---
cat > "$CHECK/main.cpp" <<'CPP'
#include <iostream>

// ПРАВИЛЬНЫЙ ПОРЯДОК: Сначала Qt, потом всё остальное.
#include <QApplication>
#include <QWidget>

// Подключаем "грязные" заголовки X11
#include <X11/Xlib.h>
#include <X11/Xutil.h>

// --- ГЛАВНОЕ ИСПРАВЛЕНИЕ ---
// Немедленно "обезвреживаем" вредоносные макросы из X11.
#undef None
#undef Success

// Теперь можно безопасно подключать OpenCASCADE
#include <Aspect_DisplayConnection.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <Xw_Window.hxx>
#include <V3d_Viewer.hxx>
#include <V3d_View.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>

int main(int argc, char** argv) {
    std::cout << "Initializing Visualization Test..." << std::endl;

    QApplication app(argc, argv);
    QWidget window;
    window.setWindowTitle("OCCT Visualization - Success!");
    window.resize(800, 600);
    window.show(); // Показываем окно сразу

    Handle(Aspect_DisplayConnection) displayConnection = new Aspect_DisplayConnection();
    Handle(OpenGl_GraphicDriver) graphicDriver = new OpenGl_GraphicDriver(displayConnection);
    
    Handle(V3d_Viewer) viewer = new V3d_Viewer(graphicDriver);
    viewer->SetDefaultLights();
    viewer->SetLightOn();

    Handle(V3d_View) view = viewer->CreateView();

    // Используем winId() для прямой интеграции с окном X11
    Handle(Xw_Window) xwWindow = new Xw_Window(displayConnection, (Window) window.winId());
    view->SetWindow(xwWindow);
    if (!xwWindow->IsMapped()) {
        xwWindow->Map();
    }
    
    Handle(AIS_InteractiveContext) context = new AIS_InteractiveContext(viewer);

    TopoDS_Shape boxShape = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();
    Handle(AIS_Shape) aisBox = new AIS_Shape(boxShape);
    context->Display(aisBox, Standard_True);
    view->FitAll();
    view->ZFitAll();

    std::cout << "Window is showing. Close it to exit the test." << std::endl;
    return app.exec();
}
CPP

# --- ШАГ 3: Сборка и запуск ---
echo "--- [Stable Visu Test] Configuring... ---"
cmake -S "$CHECK" -B "$CHECK/build"
echo "--- [Stable Visu Test] Building... ---"
cmake --build "$CHECK/build" -j
echo "--- [Stable Visu Test] Launching... ---"
echo "!!! A 3D WINDOW SHOULD APPEAR. PLEASE CLOSE IT TO CONTINUE. !!!"
export LD_LIBRARY_PATH="${PREFIX}/lib:${LD_LIBRARY_PATH:-}"
"$CHECK/build/occt_stable_visu_probe"
echo "== [Stable Visu Test] Application closed. Success! =="

#!/usr/bin/env bash
set -euo pipefail

# Загружаем переменную PREFIX, чтобы знать, где всё установлено
source "$(dirname "${BASH_SOURCE[0]}")/versions.env"

echo "== [OCCT Visualization Test] Verifying installation in ${PREFIX} =="
CHECK=/tmp/occt_visu_check && rm -rf "$CHECK" && mkdir -p "$CHECK"

# --- ШАГ 1: Создание CMakeLists.txt ---
# Он уже был правильным, но на всякий случай добавим явную линковку с X11,
# чтобы сделать его еще более надежным.
cat > "$CHECK/CMakeLists.txt" <<CMAKE
cmake_minimum_required(VERSION 3.20)
project(occt_visu_probe LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(CMAKE_PREFIX_PATH "${PREFIX}")

find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGLWidgets)

find_package(OpenCASCADE REQUIRED
    HINTS "\${CMAKE_PREFIX_PATH}"
    NO_CMAKE_PACKAGE_REGISTRY
)

# Находим библиотеку X11
find_package(X11 REQUIRED)

add_executable(occt_visu_probe main.cpp)

# Подключаем пути к заголовочным файлам OCCT и X11
target_include_directories(occt_visu_probe PRIVATE
    \${OpenCASCADE_INCLUDE_DIRS}
    \${X11_INCLUDE_DIR}
)

# Линкуем наше приложение со всеми необходимыми библиотеками
target_link_libraries(occt_visu_probe PRIVATE
    Qt6::Widgets
    Qt6::OpenGLWidgets
    \${OpenCASCADE_LIBRARIES}
    \${X11_LIBRARIES}
)
CMAKE

# --- ШАГ 2: Создание исходного кода C++ (с финальным исправлением) ---
cat > "$CHECK/main.cpp" <<'CPP'
#include <iostream>

// ПРАВИЛЬНЫЙ ПОРЯДОК: Сначала все заголовки Qt, потом все остальные.

// Qt Headers
#include <QApplication>
#include <QWidget>

// X11 Headers
// Подключаем ОБА необходимых файла для полной совместимости.
#include <X11/Xlib.h>
#include <X11/Xutil.h>

// OpenCASCADE Headers
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

    // 1. Инициализация Qt приложения
    QApplication app(argc, argv);
    QWidget window;
    window.setWindowTitle("OpenCASCADE Visualization Test");
    window.resize(800, 600);

    // 2. Инициализация графического драйвера OpenCASCADE
    Handle(Aspect_DisplayConnection) displayConnection = new Aspect_DisplayConnection();
    Handle(OpenGl_GraphicDriver) graphicDriver = new OpenGl_GraphicDriver(displayConnection);

    // 3. Создание 3D-вьювера
    Handle(V3d_Viewer) viewer = new V3d_Viewer(graphicDriver);
    viewer->SetDefaultLights();
    viewer->SetLightOn();

    // 4. Создание 3D-сцены (вида)
    Handle(V3d_View) view = viewer->CreateView();

    // 5. Интеграция OCCT с окном Qt.
    Handle(Xw_Window) xwWindow = new Xw_Window(displayConnection, (Window) window.winId());
    view->SetWindow(xwWindow);
    if (!xwWindow->IsMapped()) {
        xwWindow->Map();
    }
    
    // 6. Создание интерактивного контекста
    Handle(AIS_InteractiveContext) context = new AIS_InteractiveContext(viewer);

    // 7. Создание 3D-объекта (куба)
    TopoDS_Shape boxShape = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();
    Handle(AIS_Shape) aisBox = new AIS_Shape(boxShape);

    // 8. Отображение объекта и настройка камеры
    context->Display(aisBox, Standard_True);
    view->FitAll();
    view->ZFitAll();

    // 9. Отображение окна Qt и запуск приложения
    window.show();
    std::cout << "Window is showing. Close it to exit the test." << std::endl;

    return app.exec();
}
CPP

# --- ШАГ 3: Сборка и запуск теста ---
echo "--- [Visu Test] Configuring the project with CMake... ---"
cmake -S "$CHECK" -B "$CHECK/build"

echo "--- [Visu Test] Building the executable... ---"
cmake --build "$CHECK/build" -j

echo "--- [Visu Test] Launching the application... ---"
echo "!!! A 3D WINDOW SHOULD APPEAR. PLEASE CLOSE IT TO CONTINUE. !!!"
export LD_LIBRARY_PATH="${PREFIX}/lib:${LD_LIBRARY_PATH:-}"
"$CHECK/build/occt_visu_probe"

echo "== [Visu Test] Application closed. Test successful! =="

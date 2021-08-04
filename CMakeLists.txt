cmake_minimum_required(VERSION 3.5)

project(HNU_RMUT_Vision LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_PREFIX_PATH "/opt/Qt5.12.9/5.12.9/gcc_64")

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check http://doc.qt.io/qt-5/deployment-android.html for more information.
# They need to be set before the find_package(Qt5 ...) call.

find_package(Qt5 COMPONENTS Widgets SerialPort PrintSupport Concurrent REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(/opt/Qt5.12.9/5.12.9/gcc_64/include/QtSerialPort)

QT5_WRAP_UI(ui_FILES  mainwindow.ui)
AUX_SOURCE_DIRECTORY(. DIR_SRCS)
add_executable(HNU_RMUT_Vision
  ${DIR_SRCS}
  ${ui_FILES}
  qcustomplot/qcustomplot.cpp
  qcustomplot/qcustomplot.h
  Daheng_inc/DxImageProc.h
  Daheng_inc/GxIAPI.h
  )

target_link_libraries(HNU_RMUT_Vision PRIVATE ${OpenCV_LIBS} Qt5::Widgets Qt5::PrintSupport Qt5::SerialPort Qt5::Concurrent Ceres::ceres gxiapi)

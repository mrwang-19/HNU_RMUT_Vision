cmake_minimum_required(VERSION 3.5)

project(RMUT_Version LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check http://doc.qt.io/qt-5/deployment-android.html for more information.
# They need to be set before the find_package(Qt5 ...) call.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

find_package(Qt5 COMPONENTS Widgets SerialPort PrintSupport Concurrent REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(/opt/Qt5.12.9/5.12.9/gcc_64/include/QtSerialPort)

if(ANDROID)
  add_library(RMUT_Version SHARED
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
  )
else()
  AUX_SOURCE_DIRECTORY(. DIR_SRCS)
  add_executable(RMUT_Version
    ${DIR_SRCS}
    qcustomplot/qcustomplot.cpp
    qcustomplot/qcustomplot.h
    Daheng_inc/DxImageProc.h
    Daheng_inc/GxIAPI.h
  )
#target_link_directories(RMUT_Version /usr/local/lib/)
endif()
#target_link_libraries(RMUT_Version  ${OpenCV_LIBS})
target_link_libraries(RMUT_Version PRIVATE ${OpenCV_LIBS} Qt5::Widgets Qt5::PrintSupport Qt5::SerialPort Qt5::Concurrent Ceres::ceres gxiapi)

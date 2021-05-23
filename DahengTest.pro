QT       += core gui serialport charts
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

qtHaveModule(printsupport): QT += printsupport

CONFIG += c++14

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += /usr/local/include/opencv4 \
               /usr/include/eigen3/

SOURCES += \
    chartpainter.cpp \
    imageprocessor.cpp \
    main.cpp \
    mainwindow.cpp \
    pid.cpp \
    qcustomplot/qcustomplot.cpp \
    transceiver.cpp

HEADERS += \
    Daheng_inc/DxImageProc.h \
    Daheng_inc/GxIAPI.h \
    chartpainter.h \
    imageprocessor.h \
    mainwindow.h \
    pid.h \
    qcustomplot/qcustomplot.h \
    transceiver.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

LIBS += -lgxiapi \
        /usr/local/lib/libopencv*.so

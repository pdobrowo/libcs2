QT += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = vis_tt
TEMPLATE = app

QMAKE_CXXFLAGS_RELEASE += -O3 -fomit-frame-pointer -march=native

freebsd*:{
    INCLUDEPATH += /usr/local/include
    LIBS += -L/usr/local/lib
}

INCLUDEPATH += ../../../inc
INCLUDEPATH += ../common

LIBS += -lGLEW
LIBS += -lGLU
LIBS += -ldl

LIBS += ../../../lib/libcs2_s.a

SOURCES += \
    mainwindow.cpp \
    ../common/main.cpp\
    ../common/gridmesh.cpp \
    ../common/renderview.cpp \
    ../common/mesh.cpp \
    ../common/renderviewarcballcamera.cpp \
    ../common/renderviewautocamera.cpp \
    ../common/renderviewflycamera.cpp \
    ../common/shader.cpp \
    ../common/linelistmesh.cpp \
    ../common/trianglelistmesh.cpp \
    ../common/meshing.cpp

HEADERS += \
    mainwindow.h \
    ../common/gridmesh.h \
    ../common/renderview.h \
    ../common/mesh.h \
    ../common/renderviewarcballcamera.h \
    ../common/renderviewautocamera.h \
    ../common/renderviewcamera.h \
    ../common/renderviewflycamera.h \
    ../common/shader.h \
    ../common/linelistmesh.h \
    ../common/trianglelistmesh.h \
    ../common/meshing.h

FORMS += \
    mainwindow.ui

OTHER_FILES += \
    ../common/perpixel.frag \
    ../common/perpixel.vert

RESOURCES += \
    ../res.qrc

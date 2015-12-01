QT += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = vis
TEMPLATE = app

LIBS += -lGLEW
LIBS += -lGLU

SOURCES += main.cpp\
        mainwindow.cpp \
    gridmesh.cpp \
    renderview.cpp \
    mesh.cpp \
    renderviewarcballcamera.cpp \
    renderviewautocamera.cpp \
    renderviewflycamera.cpp \
    shader.cpp \
    trianglelistmesh.cpp

HEADERS  += mainwindow.h \
    gridmesh.h \
    renderview.h \
    mesh.h \
    renderviewarcballcamera.h \
    renderviewautocamera.h \
    renderviewcamera.h \
    renderviewflycamera.h \
    shader.h \
    trianglelistmesh.h

FORMS += mainwindow.ui

OTHER_FILES += \
    perpixel.frag \
    perpixel.vert

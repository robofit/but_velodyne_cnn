#-------------------------------------------------
#
# Project created by QtCreator 2014-05-01T14:24:33
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ground_annotator
TEMPLATE = app


SOURCES += main.cpp\
        pclviewer.cpp \
    annotationarea.cpp \
    ringfiller.cpp

HEADERS  += pclviewer.h \
    annotationarea.h \
    ringfiller.h \
    labeling.h

FORMS    += pclviewer.ui

INCLUDEPATH += /home/ivelas/local/include/but_velodyne-0.1/\
    /usr/include/pcl-1.7/\
    /usr/include/vtk-5.8/

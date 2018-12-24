#-------------------------------------------------
#
# Project created by QtCreator 2018-12-13T10:44:55
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ElasticManipulator
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    DxlControl.cpp \
    dob.cpp

HEADERS  += mainwindow.h \
    DxlControl.h \
    dob.h

FORMS    += mainwindow.ui

unix:!macx: LIBS += -L$$PWD/lib/ -ldxl_x86_cpp

INCLUDEPATH += $$PWD/include\
                $$PWD/../dev/lib/NRMK/xenomai/include\
                $$PWD/../dev/lib/NRMK\
                $$PWD/../dev/lib/NRMK/core/3rdparty/Poco/lib/i686\
                $$PWD/../dev/lib/tp_gpio

DEPENDPATH += $$PWD/include\
                $$PWD/../dev/lib/NRMK/xenomai/include\
                $$PWD/../dev/lib/NRMK\
                $$PWD/../dev/lib/NRMK/core/3rdparty/Poco/lib/i686\

unix:!macx: LIBS += -L$$PWD/../dev/lib/NRMK/xenomai/lib/ -lnative\
                -L$$PWD/../dev/lib/NRMK/xenomai/lib/ -lxenomai\
                -L$$PWD/../dev/lib/NRMK/xenomai/lib/ -lrtdm\
                -L$$PWD/../dev/lib/NRMK/lib/ -lNRMKHelperi686\
                -L$$PWD/../dev/lib/NRMK/core/3rdparty/Poco/lib/i686/ -lPocoFoundation\
                -L$$PWD/../dev/lib/NRMK/core/3rdparty/Poco/lib/i686/ -lPocoNet

unix:!macx: PRE_TARGETDEPS += $$PWD/../dev/lib/NRMK/lib/libNRMKHelperi686.a

TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    node.cpp \
    friction.cpp \
    integrator.cpp

HEADERS += \
    node.h \
    friction.h \
    integrator.h

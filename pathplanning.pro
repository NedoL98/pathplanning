TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

win32 {
QMAKE_LFLAGS += -static -static-libgcc -static-libstdc++ -std=c++0x
}

SOURCES += pathplanning.cpp \
    tinystr.cpp \
    tinyxml.cpp \
    tinyxmlerror.cpp \
    tinyxmlparser.cpp

HEADERS += \
    tinystr.h \
    tinyxml.h

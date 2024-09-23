#-------------------------------------------------
#
# Project created by QtCreator 2016-02-09T08:58:44
#
#-------------------------------------------------

QT       -= core gui

TEMPLATE = lib
TARGET = RTKLib

DEFINES -= UNICODE TRACE

include(../RTKLib.pri)

*g++* {
    QMAKE_CFLAGS += -std=c99 -Wall -pedantic -Wno-unused-but-set-variable -g
    QMAKE_LFLAGS += -Wl,-z,undefs
}

win* {
    CONFIG += staticlib
}

macx {
    CONFIG += staticlib
}


*msvc* {
    QMAKE_CFLAGS += -D_CRT_SECURE_NO_WARNINGS
}

ROOT_DIRECTORY = $${PWD}/..

DESTDIR = $${ROOT_DIRECTORY}/lib

SOURCES += rtkcmn.c \
    trace.c \
    convkml.c \
    convrnx.c \
    convgpx.c \
    datum.c \
    download.c \
    ephemeris.c \
    geoid.c \
    gis.c \
    ionex.c \
    lambda.c \
    options.c \
    pntpos.c \
    postpos.c \
    ppp.c \
    ppp_ar.c \
    preceph.c \
    rcvraw.c \
    rinex.c \
    rtcm.c \
    rtcm2.c \
    rtcm3.c \
    rtcm3e.c \
    rtkpos.c \
    rtksvr.c \
    sbas.c \
    solution.c \
    stream.c \
    streamsvr.c \
    tides.c \
    tle.c \
    rcv/binex.c \
    rcv/crescent.c \
    rcv/javad.c \
    rcv/novatel.c \
    rcv/nvs.c \
    rcv/rt17.c \
    rcv/septentrio.c \
    rcv/skytraq.c \
    rcv/swiftnav.c \
    rcv/ublox.c \
    rcv/unicore.c 

HEADERS += rtklib.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

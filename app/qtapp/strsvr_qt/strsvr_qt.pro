QT += core gui widgets serialport

include(../qtapp.pri)

INCLUDEPATH += ../../../src/ ../appcmn_qt

TEMPLATE = app
TARGET = strsvr_qt
target.path = $$INSTALLROOT/bin
INSTALLS += target

SOURCES += \  
    convdlg.cpp \
    strsvr.cpp \
    svrmain.cpp \
    svroptdlg.cpp \
    ../appcmn_qt/aboutdlg.cpp \
    ../appcmn_qt/tcpoptdlg.cpp \
    ../appcmn_qt/serioptdlg.cpp \
    ../appcmn_qt/cmdoptdlg.cpp \
    ../appcmn_qt/console.cpp \
    ../appcmn_qt/fileoptdlg.cpp \
    ../appcmn_qt/refdlg.cpp \
    ../appcmn_qt/mntpoptdlg.cpp \
    ../appcmn_qt/keydlg.cpp \
    ../appcmn_qt/helper.cpp \
    mondlg.cpp

HEADERS  += \ 
    convdlg.h \
    svrmain.h \
    svroptdlg.h \
    ../appcmn_qt/tcpoptdlg.h \
    ../appcmn_qt/serioptdlg.h \
    ../appcmn_qt/aboutdlg.h \
    ../appcmn_qt/cmdoptdlg.h \
    ../appcmn_qt/console.h \
    ../appcmn_qt/fileoptdlg.h \
    ../appcmn_qt/refdlg.h \
    ../appcmn_qt/mntpoptdlg.h \
    ../appcmn_qt/keydlg.h \
    ../appcmn_qt/helper.h \
    mondlg.h

FORMS    += \ 
    convdlg.ui \
    svrmain.ui \
    svroptdlg.ui \
    ../appcmn_qt/tcpoptdlg.ui \
    ../appcmn_qt/serioptdlg.ui \
    ../appcmn_qt/aboutdlg.ui \
    ../appcmn_qt/cmdoptdlg.ui \
    ../appcmn_qt/console.ui \
    ../appcmn_qt/fileoptdlg.ui \
    ../appcmn_qt/refdlg.ui \
    ../appcmn_qt/mntpoptdlg.ui \
    ../appcmn_qt/keydlg.ui \
    mondlg.ui

RESOURCES += \
    ../appcmn_qt/appcmn_qt.qrc \
    ../icon/resources.qrc

RC_FILE = strsvr_qt.rc

desktop.path   = $$INSTALLROOT/share/applications/
desktop.files += strsvr_qt.desktop
INSTALLS      += desktop

icons.path   = $$INSTALLROOT/share/pixmaps/
icons.files += ../icon/strsvr.png
INSTALLS    += icons

win32 {
CONFIG(release,debug|release) {
QMAKE_POST_LINK += $$QMAKE_COPY $$shell_path($$OUT_PWD/release/$${TARGET}.exe) $$shell_path($$PKGDIR/packages/com.rtklib.$${TARGET}/data)
}
}

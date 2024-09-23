QT += widgets core gui serialport

include(../qtapp.pri)

TEMPLATE = app
TARGET = rtknavi_qt
target.path = $$INSTALLROOT/bin
INSTALLS += target

INCLUDEPATH += ../../../src/ ../appcmn_qt ../widgets_qt

SOURCES += \
    ../appcmn_qt/aboutdlg.cpp \
    ../appcmn_qt/cmdoptdlg.cpp \
    ../appcmn_qt/fileoptdlg.cpp \
    ../appcmn_qt/ftpoptdlg.cpp \
    ../appcmn_qt/keydlg.cpp \
    ../appcmn_qt/labelstretcher.cpp \
    ../appcmn_qt/maskoptdlg.cpp \
    ../appcmn_qt/refdlg.cpp \
    ../appcmn_qt/serioptdlg.cpp \
    ../appcmn_qt/mntpoptdlg.cpp \
    ../appcmn_qt/tcpoptdlg.cpp \
    ../appcmn_qt/viewer.cpp \
    ../appcmn_qt/vieweropt.cpp \
    ../appcmn_qt/freqdlg.cpp \
    ../appcmn_qt/helper.cpp \
    ../appcmn_qt/graph.cpp \
    #../appcmn_qt/pntdlg.cpp \
    ../widgets_qt/doubleunitvalidator.cpp \
    ../widgets_qt/intunitvalidator.cpp \
    ../widgets_qt/scientificspinbox.cpp \
    ../appcmn_qt/navi_post_opt.cpp \
    mondlg.cpp \
    instrdlg.cpp \
    logstrdlg.cpp \
    main.cpp \
    navimain.cpp \
    outstrdlg.cpp \
    rcvoptdlg.cpp \
    #mapdlg.cpp \
    markdlg.cpp

HEADERS  += \
    ../appcmn_qt/labelstretcher.h \
    instrdlg.h \
    rcvoptdlg.h \
    logstrdlg.h \
    navimain.h \
    outstrdlg.h \
    mondlg.h \
    #mapdlg.h \
    markdlg.h \
    ../appcmn_qt/navi_post_opt.h \
    ../appcmn_qt/aboutdlg.h \
    ../appcmn_qt/cmdoptdlg.h \
    ../appcmn_qt/fileoptdlg.h \
    ../appcmn_qt/ftpoptdlg.h \
    ../appcmn_qt/keydlg.h \
    ../appcmn_qt/maskoptdlg.h \
    ../appcmn_qt/refdlg.h \
    ../appcmn_qt/serioptdlg.h \
    ../appcmn_qt/tcpoptdlg.h \
    ../appcmn_qt/mntpoptdlg.h \
    ../appcmn_qt/viewer.h \
    ../appcmn_qt/vieweropt.h \
    ../appcmn_qt/freqdlg.h \
    ../appcmn_qt/helper.h \
    ../appcmn_qt/graph.h \
    #../appcmn_qt/pntdlg.h \
    ../widgets_qt/intunitvalidator.h \
    ../widgets_qt/doubleunitvalidator.h \
    ../widgets_qt/scientificspinbox.h

FORMS    += \
    instrdlg.ui \
    logstrdlg.ui \
    navimain.ui \
    outstrdlg.ui \
    rcvoptdlg.ui \
    mondlg.ui \
    #mapdlg.ui \
    markdlg.ui \
    ../appcmn_qt/navi_post_opt.ui \
    ../appcmn_qt/aboutdlg.ui \
    ../appcmn_qt/cmdoptdlg.ui \
    ../appcmn_qt/fileoptdlg.ui \
    ../appcmn_qt/ftpoptdlg.ui \
    ../appcmn_qt/keydlg.ui \
    ../appcmn_qt/maskoptdlg.ui \
    ../appcmn_qt/refdlg.ui \
    ../appcmn_qt/serioptdlg.ui \
    ../appcmn_qt/viewer.ui \
    ../appcmn_qt/vieweropt.ui \
    ../appcmn_qt/tcpoptdlg.ui \
    ../appcmn_qt/mntpoptdlg.ui \
    #../appcmn_qt/pntdlg.ui \
    ../appcmn_qt/freqdlg.ui

RESOURCES += \
    ../appcmn_qt/appcmn_qt.qrc \
    ../icon/resources.qrc

RC_FILE = rtknavi_qt.rc

desktop.path   = $$INSTALLROOT/share/applications/
desktop.files += rtknavi_qt.desktop
INSTALLS      += desktop

icons.path   = $$INSTALLROOT/share/pixmaps/
icons.files += ../icon/rtknavi.png
INSTALLS    += icons

win32 {
CONFIG(release,debug|release) {
QMAKE_POST_LINK += $$QMAKE_COPY $$shell_path($$OUT_PWD/release/$${TARGET}.exe) $$shell_path($$PKGDIR/packages/com.rtklib.$${TARGET}/data)
}
}

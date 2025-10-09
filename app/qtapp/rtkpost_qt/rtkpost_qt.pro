QT += widgets core gui

include(../qtapp.pri)

TEMPLATE = app
TARGET = rtkpost_qt
target.path = $$INSTALLROOT/bin
INSTALLS += target

INCLUDEPATH += ../../../src/ ../appcmn_qt ../widgets_qt

SOURCES += \
    kmzconv.cpp \
    postmain.cpp \
    rtkpost.cpp \
    ../widgets_qt/doubleunitvalidator.cpp \
    ../widgets_qt/scientificspinbox.cpp \
    ../widgets_qt/excludedsatellitevalidator.cpp \
    ../appcmn_qt/navi_post_opt.cpp \
    ../appcmn_qt/aboutdlg.cpp \
    ../appcmn_qt/freqdlg.cpp \
    ../appcmn_qt/keydlg.cpp \
    ../appcmn_qt/maskoptdlg.cpp \
    ../appcmn_qt/refdlg.cpp \
    ../appcmn_qt/viewer.cpp \
    ../appcmn_qt/vieweropt.cpp \
    ../appcmn_qt/helper.cpp \
    ../appcmn_qt/timedlg.cpp

HEADERS  += \
    kmzconv.h \
    postmain.h \
    ../widgets_qt/doubleunitvalidator.h \
    ../widgets_qt/scientificspinbox.h \
    ../appcmn_qt/navi_post_opt.h \
    ../appcmn_qt/keydlg.h \
    ../appcmn_qt/freqdlg.h \
    ../appcmn_qt/maskoptdlg.h \
    ../appcmn_qt/refdlg.h \
    ../appcmn_qt/viewer.h \
    ../appcmn_qt/vieweropt.h \
    ../appcmn_qt/helper.h \
    ../appcmn_qt/aboutdlg.h \
    ../appcmn_qt/timedlg.h

FORMS    += \ 
    kmzconv.ui \
    postmain.ui \
    ../appcmn_qt/navi_post_opt.ui \
    ../appcmn_qt/keydlg.ui \
    ../appcmn_qt/freqdlg.ui \
    ../appcmn_qt/maskoptdlg.ui \
    ../appcmn_qt/refdlg.ui \
    ../appcmn_qt/viewer.ui \
    ../appcmn_qt/vieweropt.ui \
    ../appcmn_qt/aboutdlg.ui \
    ../appcmn_qt/timedlg.ui

RESOURCES += \
    ../appcmn_qt/appcmn_qt.qrc \
    ../icon/resources.qrc

RC_FILE = rtkpost_qt.rc

desktop.path   = $$INSTALLROOT/share/applications/
desktop.files += rtkpost_qt.desktop
INSTALLS      += desktop

icons.path   = $$INSTALLROOT/share/pixmaps/
icons.files += ../icon/rtkpost.png
INSTALLS    += icons

win32 {
CONFIG(release,debug|release) {
QMAKE_POST_LINK += $$QMAKE_COPY $$shell_path($$OUT_PWD/release/$${TARGET}.exe) $$shell_path($$PKGDIR/packages/com.rtklib.$${TARGET}/data)
}
}

add_executable(strsvr_qt
    convdlg.cpp
    strsvr.cpp
    svrmain.cpp
    svroptdlg.cpp
    mondlg.cpp

    convdlg.ui
    svrmain.ui
    svroptdlg.ui
    mondlg.ui

    ../appcmn_qt/aboutdlg.cpp
    ../appcmn_qt/tcpoptdlg.cpp
    ../appcmn_qt/serioptdlg.cpp
    ../appcmn_qt/cmdoptdlg.cpp
    ../appcmn_qt/console.cpp
    ../appcmn_qt/fileoptdlg.cpp
    ../appcmn_qt/refdlg.cpp
    ../appcmn_qt/mntpoptdlg.cpp
    ../appcmn_qt/keydlg.cpp
    ../appcmn_qt/helper.cpp

    ../appcmn_qt/tcpoptdlg.ui
    ../appcmn_qt/serioptdlg.ui
    ../appcmn_qt/aboutdlg.ui
    ../appcmn_qt/cmdoptdlg.ui
    ../appcmn_qt/console.ui
    ../appcmn_qt/fileoptdlg.ui
    ../appcmn_qt/refdlg.ui
    ../appcmn_qt/mntpoptdlg.ui
    ../appcmn_qt/keydlg.ui

    ../appcmn_qt/appcmn_qt.qrc
    ../icon/resources.qrc

)
target_link_libraries(strsvr_qt PRIVATE
        rtklib
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::Widgets
	Qt${QT_VERSION_MAJOR}::SerialPort
)

set_target_properties(strsvr_qt PROPERTIES
    WIN32_EXECUTABLE ON
    MACOSX_BUNDLE ON
)


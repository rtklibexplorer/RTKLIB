add_executable(rtknavi_qt
    main.cpp
    mondlg.cpp
    instrdlg.cpp
    logstrdlg.cpp
    navimain.cpp
    outstrdlg.cpp
    rcvoptdlg.cpp
    #mapdlg.cpp
    markdlg.cpp

    instrdlg.ui
    logstrdlg.ui
    navimain.ui
    outstrdlg.ui
    rcvoptdlg.ui
    mondlg.ui
    #mapdlg.ui
    markdlg.ui

    ../appcmn_qt/aboutdlg.cpp
    ../appcmn_qt/cmdoptdlg.cpp
    ../appcmn_qt/fileoptdlg.cpp
    ../appcmn_qt/ftpoptdlg.cpp
    ../appcmn_qt/keydlg.cpp
    ../appcmn_qt/labelstretcher.cpp
    ../appcmn_qt/maskoptdlg.cpp
    ../appcmn_qt/refdlg.cpp
    ../appcmn_qt/serioptdlg.cpp
    ../appcmn_qt/mntpoptdlg.cpp
    ../appcmn_qt/tcpoptdlg.cpp
    ../appcmn_qt/viewer.cpp
    ../appcmn_qt/vieweropt.cpp
    ../appcmn_qt/freqdlg.cpp
    ../appcmn_qt/helper.cpp
    ../appcmn_qt/graph.cpp
    #../appcmn_qt/pntdlg.cpp
    ../appcmn_qt/navi_post_opt.cpp

    ../widgets_qt/doubleunitvalidator.cpp
    ../widgets_qt/intunitvalidator.cpp
    ../widgets_qt/scientificspinbox.cpp

    ../appcmn_qt/navi_post_opt.ui
    ../appcmn_qt/aboutdlg.ui
    ../appcmn_qt/cmdoptdlg.ui
    ../appcmn_qt/fileoptdlg.ui
    ../appcmn_qt/ftpoptdlg.ui
    ../appcmn_qt/keydlg.ui
    ../appcmn_qt/maskoptdlg.ui
    ../appcmn_qt/refdlg.ui
    ../appcmn_qt/serioptdlg.ui
    ../appcmn_qt/viewer.ui
    ../appcmn_qt/vieweropt.ui
    ../appcmn_qt/tcpoptdlg.ui
    ../appcmn_qt/mntpoptdlg.ui
    #../appcmn_qt/pntdlg.ui
    ../appcmn_qt/freqdlg.ui


    ../appcmn_qt/appcmn_qt.qrc
    ../icon/resources.qrc

)
if(QT_VERSION GREATER_EQUAL 6.5)
        qt_add_translations(rtknavi_qt
		SOURCE_TARGETS rtknavi_qt
		INCLUDE_DIRECTORIES . ..appcmn_qt
        )
endif()
target_link_libraries(rtknavi_qt PRIVATE
        rtklib
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::Widgets
        Qt${QT_VERSION_MAJOR}::SerialPort
)

set_target_properties(rtknavi_qt PROPERTIES
    WIN32_EXECUTABLE ON
    MACOSX_BUNDLE ON
)
install(TARGETS rtknavi_qt RUNTIME DESTINATION bin)
if (UNIX)
   install(FILES rtknavi_qt.desktop DESTINATION ${XDG_APPS_INSTALL_DIR})
   install(FILES ../icon/rtknavi.png DESTINATION ${XDG_APPS_PIXMAPS_DIR})
endif()

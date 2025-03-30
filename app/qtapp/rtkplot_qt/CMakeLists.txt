add_executable(rtkplot_qt
    conndlg.cpp
    mapoptdlg.cpp
    plotcmn.cpp
    plotdata.cpp
    plotdraw.cpp
    plotinfo.cpp
    plotmain.cpp
    plotopt.cpp
    pntdlg.cpp
    rtkplot.cpp
    skydlg.cpp
    fileseldlg.cpp
    vmapdlg.cpp

    conndlg.ui
    mapoptdlg.ui
    plotmain.ui
    plotopt.ui
    pntdlg.ui
    skydlg.ui
    fileseldlg.ui
    vmapdlg.ui

    ../appcmn_qt/aboutdlg.cpp
    ../appcmn_qt/refdlg.cpp
    ../appcmn_qt/freqdlg.cpp
    ../appcmn_qt/viewer.cpp
    ../appcmn_qt/vieweropt.cpp
    ../appcmn_qt/cmdoptdlg.cpp
    ../appcmn_qt/fileoptdlg.cpp
    ../appcmn_qt/serioptdlg.cpp
    ../appcmn_qt/tcpoptdlg.cpp
    ../appcmn_qt/mntpoptdlg.cpp
    ../appcmn_qt/keydlg.cpp
    ../appcmn_qt/graph.cpp
    ../appcmn_qt/console.cpp
    ../appcmn_qt/tspandlg.cpp
    ../appcmn_qt/timedlg.cpp
    ../appcmn_qt/helper.cpp
    ../appcmn_qt/mapview.cpp
    ../appcmn_qt/mapviewopt.cpp

    ../appcmn_qt/aboutdlg.ui
    ../appcmn_qt/refdlg.ui
    ../appcmn_qt/freqdlg.ui
    ../appcmn_qt/viewer.ui
    ../appcmn_qt/vieweropt.ui
    ../appcmn_qt/cmdoptdlg.ui
    ../appcmn_qt/fileoptdlg.ui
    ../appcmn_qt/serioptdlg.ui
    ../appcmn_qt/tcpoptdlg.ui
    ../appcmn_qt/mntpoptdlg.ui
    ../appcmn_qt/keydlg.ui
    ../appcmn_qt/console.ui
    ../appcmn_qt/tspandlg.ui
    ../appcmn_qt/timedlg.ui
    ../appcmn_qt/mapview.ui
    ../appcmn_qt/mapviewopt.ui

    ../appcmn_qt/appcmn_qt.qrc
    ../icon/resources.qrc

)
if(QT_VERSION GREATER_EQUAL 6.5)
	qt_add_translations(rtkplot_qt 
		SOURCE_TARGETS rtkplot_qt
		INCLUDE_DIRECTORIES . ..appcmn_qt
	)
endif()
target_link_libraries(rtkplot_qt PRIVATE
        rtklib
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::Widgets
	Qt${QT_VERSION_MAJOR}::Xml
	Qt${QT_VERSION_MAJOR}::SerialPort
)

if(${QtWebEngineWidgets_FOUND})
   target_link_libraries(rtkplot_qt PRIVATE Qt${QT_VERSION_MAJOR}::WebEngineWidgets)
else()
  if(${QtWebKitWidgets_FOUND})
     target_link_libraries(rtkplot_qt PRIVATE Qt${QT_VERSION_MAJOR}::WebKitWidgets)
  endif()
endif()


set_target_properties(rtkplot_qt PROPERTIES
    WIN32_EXECUTABLE ON
    MACOSX_BUNDLE ON
)
install(TARGETS rtkplot_qt RUNTIME DESTINATION bin)
if (UNIX)
   install(FILES rtkplot_qt.desktop DESTINATION ${XDG_APPS_INSTALL_DIR})
   install(FILES ../icon/rtkplot.png DESTINATION ${XDG_APPS_PIXMAPS_DIR})
endif()

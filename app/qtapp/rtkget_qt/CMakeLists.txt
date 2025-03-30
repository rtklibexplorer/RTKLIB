add_executable(rtkget_qt
    main.cpp
    getmain.cpp
    getoptdlg.cpp

    getmain.ui
    getoptdlg.ui

    ../appcmn_qt/staoptdlg.cpp
    ../appcmn_qt/aboutdlg.cpp
    ../appcmn_qt/keydlg.cpp
    ../appcmn_qt/viewer.cpp
    ../appcmn_qt/vieweropt.cpp
    ../appcmn_qt/helper.cpp
    ../appcmn_qt/timedlg.cpp

    ../appcmn_qt/staoptdlg.ui
    ../appcmn_qt/aboutdlg.ui
    ../appcmn_qt/keydlg.ui
    ../appcmn_qt/viewer.ui
    ../appcmn_qt/vieweropt.ui
    ../appcmn_qt/timedlg.ui

    ../appcmn_qt/appcmn_qt.qrc
    ../icon/resources.qrc

)
if(QT_VERSION GREATER_EQUAL 6.5)
	qt_add_translations(rtkget_qt
		            SOURCE_TARGETS rtkget_qt
			    INCLUDE_DIRECTORIES . ..appcmn_qt
        )		
endif()

target_link_libraries(rtkget_qt PRIVATE
        rtklib
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::Widgets)

set_target_properties(rtkget_qt PROPERTIES
    WIN32_EXECUTABLE ON
    MACOSX_BUNDLE ON
)

install(TARGETS rtkget_qt RUNTIME DESTINATION bin)
if (UNIX)
   install(FILES rtkget_qt.desktop DESTINATION ${XDG_APPS_INSTALL_DIR})
   install(FILES ../icon/rtkget.png DESTINATION ${XDG_APPS_PIXMAPS_DIR})
endif()

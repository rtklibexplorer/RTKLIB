add_executable(srctblbrows_qt
    browsmain.cpp
    srctblbrows.cpp

    browsmain.ui

    ../appcmn_qt/staoptdlg.cpp
    ../appcmn_qt/mapview.cpp
    ../appcmn_qt/mapviewopt.cpp
    ../appcmn_qt/aboutdlg.cpp
    ../appcmn_qt/helper.cpp
    ../appcmn_qt/mntpoptdlg.cpp
    ../appcmn_qt/viewer.cpp
    ../appcmn_qt/vieweropt.cpp

    ../appcmn_qt/staoptdlg.ui
    ../appcmn_qt/mapview.ui
    ../appcmn_qt/mapviewopt.ui
    ../appcmn_qt/aboutdlg.ui
    ../appcmn_qt/mntpoptdlg.ui
    ../appcmn_qt/viewer.ui
    ../appcmn_qt/vieweropt.ui

    ../appcmn_qt/appcmn_qt.qrc
    ../icon/resources.qrc

)
if(QT_VERSION GREATER_EQUAL 6.5)
        qt_add_translations(TARGETS srctblbrows_qt
		SOURCE_TARGETS srctblbrows_qt
		INCLUDE_DIRECTORIES . ..appcmn_qt
        )
endif()
target_link_libraries(srctblbrows_qt PRIVATE
        rtklib
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::Widgets
	Qt${QT_VERSION_MAJOR}::Concurrent
)

set_target_properties(srctblbrows_qt PROPERTIES
    WIN32_EXECUTABLE ON
    MACOSX_BUNDLE ON
)
install(TARGETS srctblbrows_qt RUNTIME DESTINATION bin)
if (UNIX)
   install(FILES srctblbrows_qt.desktop DESTINATION ${XDG_APPS_INSTALL_DIR})
   install(FILES ../icon/srctblbrows.png DESTINATION ${XDG_APPS_PIXMAPS_DIR})
endif()

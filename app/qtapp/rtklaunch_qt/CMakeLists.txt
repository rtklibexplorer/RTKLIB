add_executable(rtklaunch_qt
    launchmain.cpp
    main.cpp
    launchoptdlg.cpp

    launchmain.ui
    launchoptdlg.ui

    ../appcmn_qt/appcmn_qt.qrc
    ../icon/resources.qrc

)
if(QT_VERSION GREATER_EQUAL 6.5)
        qt_add_translations(rtklaunch_qt
		SOURCE_TARGETS rtklaunch_qt
		INCLUDE_DIRECTORIES . ..appcmn_qt
        )
endif()
target_link_libraries(rtklaunch_qt PRIVATE
        rtklib
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::Widgets)

set_target_properties(rtklaunch_qt PROPERTIES
    WIN32_EXECUTABLE ON
    MACOSX_BUNDLE ON
)
install(TARGETS rtklaunch_qt RUNTIME DESTINATION bin)
if (UNIX)
   install(FILES rtklaunch_qt.desktop DESTINATION ${XDG_APPS_INSTALL_DIR})
   install(FILES ../icon/rtklaunch.png DESTINATION ${XDG_APPS_PIXMAPS_DIR})
endif()

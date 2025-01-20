add_executable(rtklaunch_qt
    launchmain.cpp
    main.cpp
    launchoptdlg.cpp

    launchmain.ui
    launchoptdlg.ui

    ../appcmn_qt/appcmn_qt.qrc
    ../icon/resources.qrc

)
target_link_libraries(rtklaunch_qt PRIVATE
        rtklib
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::Widgets)

set_target_properties(rtklaunch_qt PROPERTIES
    WIN32_EXECUTABLE ON
    MACOSX_BUNDLE ON
)

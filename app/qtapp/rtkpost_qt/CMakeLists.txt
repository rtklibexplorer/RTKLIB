add_executable(rtkpost_qt
    kmzconv.cpp
    postmain.cpp
    rtkpost.cpp

    kmzconv.ui
    postmain.ui

    ../appcmn_qt/navi_post_opt.cpp
    ../appcmn_qt/aboutdlg.cpp
    ../appcmn_qt/freqdlg.cpp
    ../appcmn_qt/keydlg.cpp
    ../appcmn_qt/maskoptdlg.cpp
    ../appcmn_qt/refdlg.cpp
    ../appcmn_qt/viewer.cpp
    ../appcmn_qt/vieweropt.cpp
    ../appcmn_qt/helper.cpp
    ../appcmn_qt/timedlg.cpp

    ../widgets_qt/doubleunitvalidator.cpp
    ../widgets_qt/scientificspinbox.cpp

    ../appcmn_qt/navi_post_opt.ui
    ../appcmn_qt/keydlg.ui
    ../appcmn_qt/freqdlg.ui
    ../appcmn_qt/maskoptdlg.ui
    ../appcmn_qt/refdlg.ui
    ../appcmn_qt/viewer.ui
    ../appcmn_qt/vieweropt.ui
    ../appcmn_qt/aboutdlg.ui
    ../appcmn_qt/timedlg.ui

    ../appcmn_qt/appcmn_qt.qrc
    ../icon/resources.qrc

)
target_link_libraries(rtkpost_qt PRIVATE
        rtklib
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::Widgets)

set_target_properties(rtkpost_qt PROPERTIES
    WIN32_EXECUTABLE ON
    MACOSX_BUNDLE ON
)


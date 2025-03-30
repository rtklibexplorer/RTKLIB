# CMake file for the QT GUI apps
# set output dir
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(XDG_APPS_INSTALL_DIR "${CMAKE_INSTALL_DATAROOTDIR}/applications")
set(XDG_APPS_PIXMAPS_DIR "${CMAKE_INSTALL_DATAROOTDIR}/icons/")

include_directories(
	${PROJECT_SOURCE_DIR}/src
	appcmn_qt
	widgets_qt
)

#
# Finding Qt
#
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core)

message(STATUS "Using Qt Version ${QT_VERSION_MAJOR}")
find_package(Qt${QT_VERSION_MAJOR} NO_MODULE REQUIRED COMPONENTS Gui Widgets Concurrent Xml SerialPort)

find_package(
    Qt${QT_VERSION_MAJOR} NO_MODULE QUIET
    OPTIONAL_COMPONENTS
    WebEngineWidgets
    WebKitWidgets
)
set(CMAKE_AUTOMOC TRUE)
set(CMAKE_AUTORCC TRUE)
set(CMAKE_AUTOUIC TRUE)
if(${QT_VERSION_MAJOR} EQUAL 6)
   # add translation support
   find_package(Qt6 REQUIRED COMPONENTS LinguistTools)

   qt_standard_project_setup(I18N_TRANSLATED_LANGUAGES de)
endif()

if(${QtWebEngineWidgets_FOUND})
   add_definitions(-DQWEBENGINE)
endif()

if(${QtWebKitWidgets_FOUND})
	add_definitions(-DQWEBKIT)
endif()

if(QT_VERSION_MAJOR GREATER_EQUAL 6)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    if(MSVC)
        add_compile_options(/Zc:__cplusplus)
    endif()
    if(MINGW)
        # prevent 'file too big' and 'too many sections' errors with Qt6
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wa,-mbig-obj")
    endif()
endif()

# try to set up the menu system
find_program(XDG-MIME_EXECUTABLE xdg-mime)
find_program(XDG-DESKTOP-MENU_EXECUTABLE xdg-desktop-menu)

if(WIN32)
	add_definitions(-DWIN32)
endif()

add_subdirectory(rtkconv_qt)
add_subdirectory(rtkget_qt)
add_subdirectory(rtklaunch_qt)
add_subdirectory(rtknavi_qt)
add_subdirectory(rtkplot_qt)
add_subdirectory(rtkpost_qt)
add_subdirectory(srctblbrows_qt)
add_subdirectory(strsvr_qt)

if(IERS_MODEL)
   target_link_libraries(-liers -lgfortran)
endif()

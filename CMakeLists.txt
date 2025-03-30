cmake_minimum_required(VERSION 3.16)
# read and set rtklib version
file(READ "src/rtklib.h" ver)

string(REGEX MATCH "VER_RTKLIB[ ]+\"([a-zA-Z0-9]*)\"" _ ${ver})
set(VERSION ${CMAKE_MATCH_1})
string(REGEX MATCH "PATCH_LEVEL[ ]+\"([a-zA-Z0-9]*)\"" _ ${ver})
set(PATCH_LEVEL ${CMAKE_MATCH_1})

message("RTKLIB version: ${VERSION}-${PATCH_LEVEL}")

project(rtklib LANGUAGES C CXX VERSION 2.4.3)

option(IERS_MODEL "Use Earth models from IERS" OFF)

# configure rtklib
add_definitions(-DENAGLO -DENAQZS -DENACMP -DENAGAL -DENAIRN -DNFREQ=3 -DNEXOBS=3 -DTRACE -DSVR_REUSEADDR)


# rtklib library
add_subdirectory(src)

# console apps
message(STATUS "Building console apps")
add_subdirectory(app/consapp)

# windows app
if(CMAKE_CXX_COMPILER_ID MATCHES "Embarcadero")
    # windows gui apps
    message(STATUS "Found Embarcadero Compile. Building Windows apps.")
    add_subdirectory(app/winapp)
endif()

# check for Qt to build Qt Gui
find_package(QT NAMES Qt6 Qt5 COMPONENTS Core)
if(QT_FOUND)
    # qt gui apps
    message(STATUS "Found Qt. Building Qt apps")
    add_subdirectory(app/qtapp)
endif()

#CPACK: General Settings
set(CPACK_GENERATOR "TBZ2" "RPM" "DEB")
set(CPACK_PACKAGE_NAME "rtklib")
set(CPACK_PACKAGE_DESCRIPTION "Rtklib is a library and GUI frontend for GNSS (Global Navigation Satellite Systems) raw data processing, visualisation and analysis. It support both, real-time kinematik (RTK) and precise point positioning (PPP) processing.")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Libraries and Tools to process GNSS data.")
set(CPACK_PACKAGE_VENDOR "")
set(CPACK_PACKAGE_CONTACT "Jens Reimann")
set(CPACK_PACKAGE_VERSION ${CMAKE_PROJECT_VERSION}-${VERSION})

#CPACK: RPM Specific Settings
set(CPACK_RPM_PACKAGE_LICENSE "GPLv2+")
set(CPACK_RPM_PACKAGE_GROUP "Development/Tools")
set(CPACK_RPM_PACKAGE_VERSION ${VERSION}.${PATCH_LEVEL})

#CPACK: DEB Specific Settings
set(CPACK_DEBIAN_PACKAGE_SECTION "Development")

include(CPack)

cmake_minimum_required(VERSION 3.16)
project(rtklib LANGUAGES C CXX)

option(IERS_MODEL "Use Earth models from IERS" OFF)

# configure rtklib
add_definitions(-DENAGLO -DENAQZS -DENACMP -DENAGAL -DENAIRN -DNFREQ=3 -DNEXOBS=3 -DTRACE -DSVR_REUSEADDR)

# rtklib library
add_subdirectory(src)

# console apps
add_subdirectory(app/consapp)

if(CMAKE_CXX_COMPILER_ID MATCHES "Embarcadero")
    # windows gui apps
    add_subdirectory(app/winapp)
else()
    # qt gui apps
    add_subdirectory(app/qtapp)
endif()

#CPACK: General Settings
set(CPACK_GENERATOR "TBZ2")
set(CPACK_PACKAGE_NAME "rtklib")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Libraries and Tools to process GNSS data.")
#TODO: shorten lines in the Readme.txt to make rpmlint happy
#set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_SOURCE_DIR}/${README_FILE}")
set(CPACK_PACKAGE_VENDOR "")
set(CPACK_PACKAGE_CONTACT "")
set(CPACK_PACKAGE_VERSION_MAJOR "${RTKLIB_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${RTKLIB_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${RTKLIB_VERSION_PATCH}")
set(CPACK_PACKAGE_VERSION "${RTKLIB_VERSION}")

#CPACK: RPM Specific Settings
set(CPACK_RPM_PACKAGE_LICENSE "GPLv2+")
set(CPACK_RPM_PACKAGE_GROUP "Development/Tools")

#CPACK: DEB Specific Settings
set(CPACK_DEBIAN_PACKAGE_SECTION "Development")

include(CPack)

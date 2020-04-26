cmake_minimum_required(VERSION 2.8.9)
project(utest)

configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/googletest-download"
)
execute_process(COMMAND "${CMAKE_COMMAND}" --build .
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/googletest-download"
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

add_subdirectory("${CMAKE_BINARY_DIR}/googletest-src"
                 "${CMAKE_BINARY_DIR}/googletest-build"
)

if(CMAKE_VERSION VERSION_LESS 2.8.11)
    include_directories("${gtest_SOURCE_DIR}/include"
                        "${gmock_SOURCE_DIR}/include"
    )
endif()

set(RTKLIB_SRC_DIR ${CMAKE_SOURCE_DIR}/../../src)
set(RTKLIB_RCV_DIR ${CMAKE_SOURCE_DIR}/../../src/rcv)

include_directories(include ${RTKLIB_SRC_DIR} ${CMAKE_BINARY_DIR})

set(RCVSRC
${RTKLIB_RCV_DIR}/binex.c
${RTKLIB_RCV_DIR}/cmr.c
${RTKLIB_RCV_DIR}/comnav.c
${RTKLIB_RCV_DIR}/crescent.c
${RTKLIB_RCV_DIR}/gw10.c
${RTKLIB_RCV_DIR}/javad.c
${RTKLIB_RCV_DIR}/novatel.c
${RTKLIB_RCV_DIR}/nvs.c
${RTKLIB_RCV_DIR}/rcvlex.c
${RTKLIB_RCV_DIR}/rt17.c
${RTKLIB_RCV_DIR}/septentrio.c
${RTKLIB_RCV_DIR}/skytraq.c
${RTKLIB_RCV_DIR}/swiftnav.c
${RTKLIB_RCV_DIR}/tersus.c
${RTKLIB_RCV_DIR}/ublox.c
)
set(rtklibSrc
${RTKLIB_SRC_DIR}/convgpx.c
${RTKLIB_SRC_DIR}/convkml.c
${RTKLIB_SRC_DIR}/convrnx.c
${RTKLIB_SRC_DIR}/datum.c
${RTKLIB_SRC_DIR}/download.c
${RTKLIB_SRC_DIR}/ephemeris.c
${RTKLIB_SRC_DIR}/geoid.c
${RTKLIB_SRC_DIR}/gis.c
${RTKLIB_SRC_DIR}/ionex.c
${RTKLIB_SRC_DIR}/lambda.c
${RTKLIB_SRC_DIR}/options.c
${RTKLIB_SRC_DIR}/pntpos.c
${RTKLIB_SRC_DIR}/postpos.c
${RTKLIB_SRC_DIR}/ppp.c
${RTKLIB_SRC_DIR}/ppp_ar.c
${RTKLIB_SRC_DIR}/ppp_corr.c
${RTKLIB_SRC_DIR}/preceph.c
${RTKLIB_SRC_DIR}/qzslex.c
${RTKLIB_SRC_DIR}/rcvraw.c
${RTKLIB_SRC_DIR}/rinex.c
${RTKLIB_SRC_DIR}/rtcm.c
${RTKLIB_SRC_DIR}/rtcm2.c
${RTKLIB_SRC_DIR}/rtcm3.c
${RTKLIB_SRC_DIR}/rtcm3e.c
${RTKLIB_SRC_DIR}/rtkcmn.c
${RTKLIB_SRC_DIR}/rtklib.h
${RTKLIB_SRC_DIR}/rtkpos.c
${RTKLIB_SRC_DIR}/rtksvr.c
${RTKLIB_SRC_DIR}/sbas.c
${RTKLIB_SRC_DIR}/solution.c
${RTKLIB_SRC_DIR}/src.pro
${RTKLIB_SRC_DIR}/stream.c
${RTKLIB_SRC_DIR}/streamsvr.c
${RTKLIB_SRC_DIR}/tides.c
${RTKLIB_SRC_DIR}/tle.c
)

set (SRC_PATH ${CMAKE_CURRENT_SOURCE_DIR})

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/datadir.h.ini
  ${CMAKE_BINARY_DIR}/datadir.h
  @ONLY
  )

  set(SOURCES
test/imp.c
test/t_atmos.cpp
test/t_coord.cpp
test/t_gloeph.cpp
test/t_ionex.cpp
test/t_lambda.cpp
test/t_matrix.cpp
test/t_misc.cpp
test/t_ppp.cpp
test/t_preceph.cpp
test/t_rinex.cpp
test/t_time.cpp
test/t_tle.cpp
${CMAKE_BINARY_DIR}/datadir.h
)

source_group(main FILES ${SOURCES} datadir.h)
source_group(rtklib FILES ${rtklibSrc})
source_group(rtklib\\rcv FILES ${RCVSRC})

add_executable(utest ${SOURCES} ${RCVSRC} ${rtklibSrc})
target_compile_definitions(utest PUBLIC ENAGLO)

target_link_libraries(utest gtest gtest_main)
IF (WIN32)
   target_link_libraries(utest ws2_32 winmm)
ELSE()
ENDIF()
enable_testing()
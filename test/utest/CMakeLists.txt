
set(RTKLBI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

find_package(BLAS   REQUIRED)
find_package(LAPACK REQUIRED)
set(BASE_LIBRARIES ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/test/utest)

# The deterministic command/thread test doubles use POSIX APIs.
if(UNIX)
    add_executable(t_ftp_lifecycle t_ftp_lifecycle.c)
    target_link_libraries(t_ftp_lifecycle rtklib)
    foreach(protocol ftp http)
        foreach(scenario inactive cached missing-directory create-error close-active success error)
            add_test(NAME ftp_${protocol}_${scenario}
                     COMMAND t_ftp_lifecycle ${scenario} ${protocol})
            set_tests_properties(ftp_${protocol}_${scenario} PROPERTIES TIMEOUT 15)
        endforeach()
    endforeach()
endif()

add_executable(t_matrix t_matrix.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/preceph.c)
target_link_libraries(t_matrix m ${BASE_LIBRARIES})

add_executable(t_time t_time.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/preceph.c)
target_link_libraries(t_time m ${BASE_LIBRARIES})

add_executable(t_coord t_coord.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/geoid.c ${RTKLBI_DIR}/preceph.c)
target_link_libraries(t_coord m ${BASE_LIBRARIES})

add_executable(t_rinex t_rinex.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/rinex.c ${RTKLBI_DIR}/preceph.c)
target_link_libraries(t_rinex m ${BASE_LIBRARIES})

add_executable(t_lambda t_lambda.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/lambda.c ${RTKLBI_DIR}/preceph.c)
target_link_libraries(t_lambda m ${BASE_LIBRARIES})

add_executable(t_atmos t_atmos.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/preceph.c)
target_link_libraries(t_atmos m ${BASE_LIBRARIES})

add_executable(t_misc t_misc.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/preceph.c)
target_link_libraries(t_misc m ${BASE_LIBRARIES})

add_executable(t_preceph t_preceph.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/preceph.c ${RTKLBI_DIR}/rinex.c ${RTKLBI_DIR}/ephemeris.c ${RTKLBI_DIR}/sbas.c)
target_link_libraries(t_preceph m ${BASE_LIBRARIES})

add_executable(t_gloeph t_gloeph.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/rinex.c ${RTKLBI_DIR}/ephemeris.c ${RTKLBI_DIR}/sbas.c ${RTKLBI_DIR}/preceph.c)
target_link_libraries(t_gloeph m ${BASE_LIBRARIES})

add_executable(t_geoid t_geoid.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/preceph.c ${RTKLBI_DIR}/geoid.c)
target_link_libraries(t_geoid m ${BASE_LIBRARIES})

add_executable(t_ppp t_ppp.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/ephemeris.c ${RTKLBI_DIR}/preceph.c ${RTKLBI_DIR}/sbas.c ${RTKLBI_DIR}/ionex.c ${RTKLBI_DIR}/pntpos.c ${RTKLBI_DIR}/ppp.c ${RTKLBI_DIR}/ppp_ar.c ${RTKLBI_DIR}/lambda.c ${RTKLBI_DIR}/tides.c)
target_link_libraries(t_ppp m ${BASE_LIBRARIES})

add_executable(t_ionex t_ionex.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/preceph.c ${RTKLBI_DIR}/ionex.c)
target_link_libraries(t_ionex m ${BASE_LIBRARIES})

add_executable(t_tle t_tle.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/rinex.c ${RTKLBI_DIR}/ephemeris.c ${RTKLBI_DIR}/sbas.c ${RTKLBI_DIR}/preceph.c ${RTKLBI_DIR}/tle.c)
target_link_libraries(t_tle m ${BASE_LIBRARIES})


add_test(NAME matrix_test COMMAND t_matrix WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME time_test COMMAND t_time WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME coord_test COMMAND t_coord WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME rinex_test COMMAND t_rinex WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME lambda_test COMMAND t_lambda WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME atmos_test COMMAND t_atmos WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME misc_test COMMAND t_misc WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME preceph_test COMMAND t_preceph WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME gloeph_test COMMAND t_gloeph WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME geoid_test COMMAND t_geoid WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME ppp_test COMMAND t_ppp WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME ionex_test COMMAND t_ionex WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME tlr_test COMMAND t_tle WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

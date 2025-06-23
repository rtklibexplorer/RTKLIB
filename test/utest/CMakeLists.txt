
set(RTKLBI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

add_executable(t_matrix t_matrix.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/preceph.c)
target_link_libraries(t_matrix m lapack blas)

add_executable(t_time t_time.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/preceph.c)
target_link_libraries(t_time m lapack blas)

add_executable(t_coord t_coord.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/geoid.c ${RTKLBI_DIR}/preceph.c)
target_link_libraries(t_coord m lapack blas)

add_executable(t_rinex t_rinex.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/rinex.c ${RTKLBI_DIR}/preceph.c)
target_link_libraries(t_rinex m lapack blas)

add_executable(t_lambda t_lambda.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/lambda.c ${RTKLBI_DIR}/preceph.c)
target_link_libraries(t_lambda m lapack blas)

add_executable(t_atmos t_atmos.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/preceph.c)
target_link_libraries(t_atmos m lapack blas)

add_executable(t_misc t_misc.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/preceph.c)
target_link_libraries(t_misc m lapack blas)

add_executable(t_preceph t_preceph.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/preceph.c ${RTKLBI_DIR}/rinex.c ${RTKLBI_DIR}/ephemeris.c ${RTKLBI_DIR}/sbas.c)
target_link_libraries(t_preceph m lapack blas)

add_executable(t_gloeph t_gloeph.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/rinex.c ${RTKLBI_DIR}/ephemeris.c ${RTKLBI_DIR}/sbas.c ${RTKLBI_DIR}/preceph.c)
target_link_libraries(t_gloeph m lapack blas)

add_executable(t_geoid t_geoid.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/preceph.c ${RTKLBI_DIR}/geoid.c)
target_link_libraries(t_geoid m lapack blas)

add_executable(t_ppp t_ppp.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/ephemeris.c ${RTKLBI_DIR}/preceph.c ${RTKLBI_DIR}/sbas.c ${RTKLBI_DIR}/ionex.c ${RTKLBI_DIR}/pntpos.c ${RTKLBI_DIR}/ppp.c ${RTKLBI_DIR}/ppp_ar.c ${RTKLBI_DIR}/lambda.c ${RTKLBI_DIR}/tides.c)
target_link_libraries(t_ppp m lapack blas)

add_executable(t_ionex t_ionex.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/preceph.c ${RTKLBI_DIR}/ionex.c)
target_link_libraries(t_ionex m lapack blas)

add_executable(t_tle t_tle.c ${RTKLBI_DIR}/rtkcmn.c ${RTKLBI_DIR}/trace.c ${RTKLBI_DIR}/rinex.c ${RTKLBI_DIR}/ephemeris.c ${RTKLBI_DIR}/sbas.c ${RTKLBI_DIR}/preceph.c ${RTKLBI_DIR}/tle.c)
target_link_libraries(t_tle m lapack blas)


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

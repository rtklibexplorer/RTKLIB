include_directories(${PROJECT_SOURCE_DIR}/src)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(rnx2rtkp rnx2rtkp/rnx2rtkp.c)
target_link_libraries(rnx2rtkp rtklib)
install(TARGETS rnx2rtkp RUNTIME DESTINATION bin)

add_executable(convbin convbin/convbin.c)
target_link_libraries(convbin rtklib)
install(TARGETS convbin RUNTIME DESTINATION bin)

add_executable(pos2kml pos2kml/pos2kml.c)
target_link_libraries(pos2kml rtklib)
install(TARGETS pos2kml RUNTIME DESTINATION bin)

# add executables only supported on unix-like systems
if (UNIX)
  add_executable(rtkrcv rtkrcv/rtkrcv.c rtkrcv/vt.c)
  target_link_libraries(rtkrcv rtklib)
  install(TARGETS rtkrcv RUNTIME DESTINATION bin)

  add_executable(str2str str2str/str2str.c)
  target_link_libraries(str2str rtklib)
  install(TARGETS str2str RUNTIME DESTINATION bin)
endif()

# add additional libraries to compile on windows
if(WIN32)
  target_link_libraries(rnx2rtkp wsock32 ws2_32 winmm)
  target_link_libraries(convbin wsock32 ws2_32 winmm)
  target_link_libraries(pos2kml wsock32 ws2_32 winmm)
endif()


### define tests
# convbin
add_test(NAME convbin_test1 COMMAND convbin -r nov ${TEST_DATA_DIR}/rcvraw/oemv_200911218.gps -ti 10 -d . -os)
add_test(NAME convbin_test2 COMMAND convbin -r hemis ${TEST_DATA_DIR}/rcvraw/cres_20080526.bin -ti 10 -d . -f 1 -od -os)
add_test(NAME convbin_test3 COMMAND convbin ${TEST_DATA_DIR}/rcvraw/ubx_20080526.ubx -o ubx_test.obs -d . -f 1 -ts 2008/5/26 6:00 -te 2008/5/26 6:10)
add_test(NAME convbin_test4 COMMAND convbin ${TEST_DATA_DIR}/rcvraw/ubx_20080526.ubx -n ubx_test.nav -d .)
add_test(NAME convbin_test5 COMMAND convbin ${TEST_DATA_DIR}/rcvraw/ubx_20080526.ubx -h ubx_test.hnav -s ubx_test.sbs -d . -x 129)
add_test(NAME convbin_test7 COMMAND convbin ${TEST_DATA_DIR}/rcvraw/testglo.rtcm2 -tr 2009/12/18 23:20 -d .)
add_test(NAME convbin_test8 COMMAND convbin ${TEST_DATA_DIR}/rcvraw/testglo.rtcm3 -os -tr 2009/12/18 23:20 -d .)
add_test(NAME convbin_test9 COMMAND convbin -v 3 -f 6 -r nov ${TEST_DATA_DIR}/rcvraw/oemv_200911218.gps -od -os -o rnx3_test.obs -n rnx3_test.nav -d .)
add_test(NAME convbin_test10 COMMAND convbin ${TEST_DATA_DIR}/rcvraw/testglo.rtcm3 -os -tr 2009/12/18 23:20 -d .)
add_test(NAME convbin_test11 COMMAND convbin ${TEST_DATA_DIR}/rcvraw/javad_20110115.jps -d out -c JAV1)
add_test(NAME convbin_test12 COMMAND convbin ${TEST_DATA_DIR}/rcvraw/javad_20110115.jps -d out -v 3.00 -f 3 -od -os)
add_test(NAME convbin_test13 COMMAND convbin ${TEST_DATA_DIR}/rcvraw/javad_20110115.jps -d out -o test13.obs -v 3 -hc test1 -hc test2 -hm MARKER -hn MARKERNO -ht MARKKERTYPE -ho OBSERVER/AGENCY -hr 1234/RECEIVER/V.0.1.2 -ha ANTNO/ANTENNA -hp 1234.567/8901.234/5678.901 -hd 0.123/0.234/0.567)
add_test(NAME convbin_test14 COMMAND convbin ${TEST_DATA_DIR}/rcvraw/javad_20110115.jps -d out -o test14.obs -v 3 -y S -y J -x 2 -x R19 -x R21)
add_test(NAME convbin_test15 COMMAND convbin ${TEST_DATA_DIR}/rcvraw/javad_20110115.jps -d out -o test15.obs -v 3 -ro "-GL1P -GL2C")
add_test(NAME convbin_test16 COMMAND convbin ${TEST_DATA_DIR}/rcvraw/javad_20110115.jps -d out -o test15.obs -v 3 -ro "-GL1P -GL2C")
add_test(NAME convbin_test17 COMMAND convbin ${TEST_DATA_DIR}/rcvraw/GMSD7_20121014.rtcm3 -tr 2012/10/14 0:00:00)
add_test(NAME convbin_test18 COMMAND convbin ${TEST_DATA_DIR}/rcvraw/GMSD7_20121014.rtcm3 -scan -v 3.01 -f 6 -od -os -tr 2012/10/14 0:00:00)
#add_test(NAME convbin_test21 COMMAND stty raw < /dev/ttyACM0 && convbin -r ubx -o ubx.obs -n ubx.nav -s ubx.sbs -h ubx.hnav /dev/ttyACM0)

# rnx2rtkp
set(RNX2RTKP_TEST_INPUT11 ${TEST_DATA_DIR}/rinex/07590920.05o ${TEST_DATA_DIR}/rinex/30400920.05n)
set(RNX2RTKP_TEST_INPUT12 ${TEST_DATA_DIR}/rinex/30400920.05o)
set(RNX2RTKP_TEST_OPTS1 -r -3978241.958 3382840.234 3649900.853)
set(RNX2RTKP_TEST_OPTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rnx2rtkp/test/)

add_test(NAME rnx2rtkp_test1 COMMAND rnx2rtkp ${RNX2RTKP_TEST_INPUT11} -x 5 -o test1.pos)
add_test(NAME rnx2rtkp_test2 COMMAND rnx2rtkp -t -e ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} > test2.pos)
add_test(NAME rnx2rtkp_test3 COMMAND rnx2rtkp -t -p 1 -e ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} ${RNX2RTKP_TEST_INPUT12} > test3.pos)
add_test(NAME rnx2rtkp_test4 COMMAND rnx2rtkp -t -p 3 -e ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} ${RNX2RTKP_TEST_INPUT12} > test4.pos)
add_test(NAME rnx2rtkp_test5 COMMAND rnx2rtkp -t -m 15 -e ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} ${RNX2RTKP_TEST_INPUT12} > test5.pos)
add_test(NAME rnx2rtkp_test6 COMMAND rnx2rtkp -t -f 1 -e ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} ${RNX2RTKP_TEST_INPUT12} > test6.pos)
add_test(NAME rnx2rtkp_test7 COMMAND rnx2rtkp -t -v 5 -e ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} ${RNX2RTKP_TEST_INPUT12} > test7.pos)
add_test(NAME rnx2rtkp_test8 COMMAND rnx2rtkp -t -i -e ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} ${RNX2RTKP_TEST_INPUT12} > test8.pos)
add_test(NAME rnx2rtkp_test9 COMMAND rnx2rtkp -t -p 0 ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} > test9.pos)
add_test(NAME rnx2rtkp_test10 COMMAND rnx2rtkp -t -p 0 ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} -o test10.pos)
add_test(NAME rnx2rtkp_test11 COMMAND rnx2rtkp -t -p 0 -n ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} > test11.pos)
add_test(NAME rnx2rtkp_test12 COMMAND rnx2rtkp -t -p 0 -g ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} > test12.pos)
add_test(NAME rnx2rtkp_test13 COMMAND rnx2rtkp -t -p 0 ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} > test13.pos)
add_test(NAME rnx2rtkp_test14 COMMAND rnx2rtkp -t -p 0 -u ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} > test14.pos)
add_test(NAME rnx2rtkp_test15 COMMAND rnx2rtkp -t -p 0 -d 9 ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} > test15.pos)
add_test(NAME rnx2rtkp_test16 COMMAND rnx2rtkp -t -p 0 -s , ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} > test16.pos)
add_test(NAME rnx2rtkp_test17 COMMAND rnx2rtkp -t -b -e ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} ${RNX2RTKP_TEST_INPUT12} > test17.pos)
add_test(NAME rnx2rtkp_test18 COMMAND rnx2rtkp -t -c -e ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} ${RNX2RTKP_TEST_INPUT12} > test18.pos)
add_test(NAME rnx2rtkp_test19 COMMAND rnx2rtkp -t -h -e ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} ${RNX2RTKP_TEST_INPUT12} > test19.pos)
add_test(NAME rnx2rtkp_test20 COMMAND rnx2rtkp -t -p 4 -a ${RNX2RTKP_TEST_OPTS1} ${RNX2RTKP_TEST_INPUT11} ${RNX2RTKP_TEST_INPUT12} > test20.pos)
add_test(NAME rnx2rtkp_test21 COMMAND rnx2rtkp ${RNX2RTKP_TEST_INPUT11} ${RNX2RTKP_TEST_INPUT12} > test21.pos)
add_test(NAME rnx2rtkp_test22 COMMAND rnx2rtkp -k ${RNX2RTKP_TEST_OPTS_DIR}opts1.conf ${RNX2RTKP_TEST_INPUT11} ${RNX2RTKP_TEST_INPUT12} > test22.pos WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rnx2rtkp/gcc)
add_test(NAME rnx2rtkp_test23 COMMAND rnx2rtkp -k ${RNX2RTKP_TEST_OPTS_DIR}opts2.conf ${RNX2RTKP_TEST_INPUT11} ${RNX2RTKP_TEST_INPUT12} > test23.pos WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rnx2rtkp/gcc)
add_test(NAME rnx2rtkp_test24 COMMAND rnx2rtkp -k ${RNX2RTKP_TEST_OPTS_DIR}opts3.conf ${RNX2RTKP_TEST_INPUT11} ${RNX2RTKP_TEST_INPUT12} -y 2 -o test24.pos WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rnx2rtkp/gcc)
add_test(NAME rnx2rtkp_test25 COMMAND rnx2rtkp -k ${RNX2RTKP_TEST_OPTS_DIR}opts4.conf ${RNX2RTKP_TEST_INPUT11} ${RNX2RTKP_TEST_INPUT12} -y 2 -o test25.pos WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rnx2rtkp/gcc)

# rtkrcv
set(RTKRCV_TEST_CONF_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rtkrcv/conf/)
#add_test(NAME rtkrcv_test1 COMMAND rtkrcv -t 4 -m 52001 -t 4)
#add_test(NAME rtkrcv_test2 COMMAND rtkrcv -p 2105 -m 52001)
#add_test(NAME rtkrcv_test3 COMMAND rtkrcv -o ${RTKRCV_TEST_CONF_DIR}rtk.conf)


# str2str
set(STR2STR_DEVICE ttyS1:115200:8:n:1:off)

#add_test(NAME str2str_test1 COMMAND str2str -in serial://${STR2STR_DEVICE} -out file://test1.out -t 2)
#add_test(NAME str2str_test2 COMMAND str2str -in serial://${STR2STR_DEVICE} -out tcpsvr://:2102 -c ../../../data/oem4_raw_10hz.cmd -t 5)
#add_test(NAME str2str_test3 COMMAND str2str -in serial://${STR2STR_DEVICE} -out ntrips://:sesam@localhost:8000/BUCU0 -c ../../../data/oem4_raw_10hz.cmd -t 5)
#add_test(NAME str2str_test4 COMMAND str2str -in ../oem6_20121107.gps nov -out oem6_20121107.rtcm3 rtcm3 -sta 123 -msg "1077(10),1087(10),1097(10),1107(10),1117(10),1019,1020")
#add_test(NAME str2str_test5 COMMAND str2str -in ../oem6_20121107.gps nov -out oem6_20121107_gal.rtcm3 rtcm3 -msg "1097(10)")
#add_test(NAME str2str_test6 COMMAND str2str -in serial://ttyS54:115200 nov -out tcpsvr://:2103 rtcm3 -c ../../../data/oem4_raw_10hz.cmd)


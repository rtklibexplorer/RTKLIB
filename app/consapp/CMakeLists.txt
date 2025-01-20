include_directories(${PROJECT_SOURCE_DIR}/src)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(rnx2rtkp rnx2rtkp/rnx2rtkp.c)
target_link_libraries(rnx2rtkp rtklib)

add_executable(convbin convbin/convbin.c)
target_link_libraries(convbin rtklib)

add_executable(pos2kml pos2kml/pos2kml.c)
target_link_libraries(pos2kml rtklib)

# add executables only supported on unix-like systems
if (UNIX)
  add_executable(rtkrcv rtkrcv/rtkrcv.c rtkrcv/vt.c)
  target_link_libraries(rtkrcv rtklib)

  add_executable(str2str str2str/str2str.c)
  target_link_libraries(str2str rtklib)
endif()

if(WIN32)
  target_link_libraries(rnx2rtkp wsock32 ws2_32 winmm)
  target_link_libraries(convbin wsock32 ws2_32 winmm)
  target_link_libraries(pos2kml wsock32 ws2_32 winmm)
endif()

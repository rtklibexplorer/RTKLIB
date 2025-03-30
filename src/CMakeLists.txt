include_directories(.)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

if(WIN32)
  add_definitions(-DWIN_DLL)
else()
  add_definitions(-DDLL)
endif()

if (GCC)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wall -O3 -ansi -pedantic")
    set(CMAKE_C_FLAGS "-Wno-unused-but-set-variable -Wno-format-overflow -Wno-unused-result -Wpointer-to-int-cast")
endif()

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    if (GCC)
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
    endif()
endif()

if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

aux_source_directory(. DIR_SRCS_RTKLIB)
set(DIR_SRCS_RTKLIB_RCV
  ${DIR_SRCS_RTKLIB_RCV}
  rcv/binex.c
  rcv/crescent.c
  rcv/javad.c
  rcv/novatel.c
  rcv/nvs.c
  rcv/rt17.c
  rcv/septentrio.c
  rcv/skytraq.c
  rcv/swiftnav.c
  rcv/ublox.c
  rcv/unicore.c
)
list(APPEND DIR_SRCS ${DIR_SRCS_RTKLIB} ${DIR_SRCS_RTKLIB_RCV})

add_library(rtklib SHARED ${DIR_SRCS} rtklib.h)

target_include_directories(rtklib
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

if(WIN32)
  target_link_libraries(rtklib wsock32 ws2_32 winmm)
  add_definitions(-DWIN_DLL)
else()
  target_link_libraries(rtklib m)
  add_definitions(-DDLL)
endif()
install(TARGETS rtklib RUNTIME DESTINATION lib)

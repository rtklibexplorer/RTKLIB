add_executable(srctblbrows WIN32
    browsmain.cpp  
    srctblbrows.cpp  
    staoptdlg.cpp

    ../appcmn/aboutdlg.cpp
    ../appcmn/gmview.cpp
)

set_embt_target(srctblbrows VCL)

target_include_directories(srctblbrows PRIVATE .)
target_link_libraries(srctblbrows PRIVATE srctblbrows.res)

install(TARGETS srctblbrows
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)


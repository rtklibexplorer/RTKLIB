add_executable(rtkget WIN32
    getmain.cpp  
    getoptdlg.cpp  
    rtkget.cpp  
    staoptdlg.cpp

    ../appcmn/aboutdlg.cpp
    ../appcmn/keydlg.cpp
    ../appcmn/timedlg.cpp
    ../appcmn/viewer.cpp
    ../appcmn/vieweropt.cpp
)

set_embt_target(rtkget VCL)

target_include_directories(rtkget PRIVATE .)
target_link_libraries(rtkget PRIVATE rtkget.res)

install(TARGETS rtkget
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)


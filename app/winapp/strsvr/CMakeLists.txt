add_executable(strsvr WIN32
    convdlg.cpp  
    mondlg.cpp  
    strsvr.cpp  
    svrmain.cpp  
    svroptdlg.cpp

    ../appcmn/aboutdlg.cpp
    ../appcmn/cmdoptdlg.cpp
    ../appcmn/confdlg.cpp
    ../appcmn/fileoptdlg.cpp
    ../appcmn/ftpoptdlg.cpp
    ../appcmn/keydlg.cpp
    ../appcmn/mntpoptdlg.cpp
    ../appcmn/refdlg.cpp
    ../appcmn/serioptdlg.cpp
    ../appcmn/tcpoptdlg.cpp
    ../appcmn/viewer.cpp
    ../appcmn/vieweropt.cpp
)

set_embt_target(strsvr VCL)

target_include_directories(strsvr PRIVATE .)
target_link_libraries(strsvr PRIVATE strsvr.res)

install(TARGETS strsvr
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)


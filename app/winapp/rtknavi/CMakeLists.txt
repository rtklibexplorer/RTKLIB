add_executable(rtknavi WIN32
     instrdlg.cpp  
     logstrdlg.cpp  
     mapdlg.cpp  
     markdlg.cpp  
     mondlg.cpp  
     navimain.cpp  
     naviopt.cpp  
     outstrdlg.cpp  
     rcvoptdlg.cpp  
     rtknavi.cpp

     ../appcmn/aboutdlg.cpp
     ../appcmn/cmdoptdlg.cpp
     ../appcmn/confdlg.cpp
     ../appcmn/fileoptdlg.cpp
     ../appcmn/freqdlg.cpp
     ../appcmn/ftpoptdlg.cpp
     ../appcmn/graph.cpp
     ../appcmn/keydlg.cpp
     ../appcmn/mntpoptdlg.cpp
     ../appcmn/refdlg.cpp
     ../appcmn/serioptdlg.cpp
     ../appcmn/tcpoptdlg.cpp
     ../appcmn/viewer.cpp
     ../appcmn/vieweropt.cpp
)

set_embt_target(rtknavi VCL)

target_include_directories(rtknavi PRIVATE .)
target_link_libraries(rtknavi PRIVATE rtknavi.res)

install(TARGETS rtknavi
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

add_executable(rtkplot WIN32
     conndlg.cpp     
     mapoptdlg.cpp  
     mapviewopt.cpp  
     plotdata.cpp  
     plotinfo.cpp  
     plotopt.cpp  
     rtkplot.cpp  
     vmapdlg.cpp
     fileseldlg.cpp  
     mapview.cpp    
     plotcmn.cpp     
     plotdraw.cpp  
     plotmain.cpp  
     pntdlg.cpp   
     skydlg.cpp

     ../appcmn/aboutdlg.cpp
     ../appcmn/cmdoptdlg.cpp
     ../appcmn/confdlg.cpp
     ../appcmn/console.cpp
     ../appcmn/fileoptdlg.cpp
     ../appcmn/freqdlg.cpp
     ../appcmn/ftpoptdlg.cpp
     ../appcmn/graph.cpp
     ../appcmn/keydlg.cpp
     ../appcmn/mntpoptdlg.cpp
     ../appcmn/refdlg.cpp
     ../appcmn/serioptdlg.cpp
     ../appcmn/tcpoptdlg.cpp
     ../appcmn/timedlg.cpp
     ../appcmn/tspandlg.cpp
     ../appcmn/viewer.cpp
     ../appcmn/vieweropt.cpp
)

set_embt_target(rtkplot VCL)

target_include_directories(rtkplot PRIVATE .)
target_link_libraries(rtkplot PRIVATE rtkplot.res)

install(TARGETS rtkplot
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)


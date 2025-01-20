add_executable(rtkpost WIN32
     kmzconv.cpp  
     postmain.cpp  
     postopt.cpp  
     rtkpost.cpp

     ../appcmn/aboutdlg.cpp
     ../appcmn/confdlg.cpp
     ../appcmn/freqdlg.cpp
     ../appcmn/keydlg.cpp
     ../appcmn/maskoptdlg.cpp
     ../appcmn/refdlg.cpp
     ../appcmn/timedlg.cpp
     ../appcmn/viewer.cpp
     ../appcmn/vieweropt.cpp
)

set_embt_target(rtkpost VCL)

target_include_directories(rtkpost PRIVATE .)
target_link_libraries(rtkpost PRIVATE rtkpost.res)

install(TARGETS rtkpost
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

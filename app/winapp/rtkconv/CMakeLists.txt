add_executable(rtkconv WIN32
    codeopt.cpp  
    convmain.cpp  
    convopt.cpp  
    rtkconv.cpp  
    startdlg.cpp

    ../appcmn/aboutdlg.cpp
    ../appcmn/confdlg.cpp
    ../appcmn/freqdlg.cpp
    ../appcmn/glofcndlg.cpp
    ../appcmn/keydlg.cpp
    ../appcmn/timedlg.cpp
    ../appcmn/viewer.cpp
    ../appcmn/vieweropt.cpp
)

set_embt_target(rtkconv VCL)

target_include_directories(rtkconv PRIVATE .)
target_link_libraries(rtkconv PRIVATE rtkconv.res)

install(TARGETS rtkconv
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

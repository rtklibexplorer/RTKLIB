add_executable(rtklaunch WIN32
   launchmain.cpp  
   launchoptdlg.cpp  
   rtklaunch.cpp
)

set_embt_target(rtklaunch VCL)

target_include_directories(rtklaunch PRIVATE .)
target_link_libraries(rtklaunch PRIVATE rtklaunch.res)

install(TARGETS rtklaunch
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)


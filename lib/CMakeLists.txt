if(IERS_MODEL)
  enable_language(Fortran)
  get_filename_component (Fortran_COMPILER_NAME ${CMAKE_Fortran_COMPILER} NAME)
  set(CMAKE_Fortran_SOURCE_FILE_EXTENSIONS f)

  set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -O3 -ffixed-line-length-132")
  file( GLOB DIR_SRCS_IERS ${CMAKE_CURRENT_SOURCE_DIR}/iers/src/*.f )
  #aux_source_directory(iers/src DIR_SRCS_IERS)
  add_library(iers SHARED ${DIR_SRCS_IERS})

  if(IERS_MODEL)
   target_link_libraries(iers gfortran)
  endif()
endif()

cmake_minimum_required(VERSION 2.6)

#find_package(Lapack REQUIRED)

include_directories(${LAPACK_INCLUDE_DIRS})
link_directories(${LAPACK_LIBRARY_DIRS})

if(NOT CMAKE_BUILD_TYPE)
  set(
    CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

set(right_shared
  ikfast.HIRONX_RARM.${CMAKE_SYSTEM_PROCESSOR} 
  )
  
set(left_shared
  ikfast.HIRONX_LARM.${CMAKE_SYSTEM_PROCESSOR} 
  )

set(target 
  right_shared left_shared
  )

set(right_source
  ikfast41.HIRONX.Transform6D_right.cpp
  )

set(left_source
  ikfast41.HIRONX.Transform6D_left.cpp
  )

add_definitions(-DIKFAST_NO_MAIN -DIKFAST_CLIBRARY)

add_library(${left_shared} SHARED ${left_source})
set_target_properties(${left_shared} PROPERTIES PREFIX "")
#target_link_libraries(${left_shared} ${LAPACK_LIBRARIES})
target_link_libraries(${left_shared} lapack)

add_library(${right_shared} SHARED ${right_source})
set_target_properties(${right_shared} PROPERTIES PREFIX "")
#target_link_libraries(${right_shared} ${LAPACK_LIBRARIES})
target_link_libraries(${right_shared} lapack)



install(TARGETS ${left_shared} ${right_shared}
  LIBRARY DESTINATION ${ROBOT}/lib CONFIGURATIONS Release Debug
)

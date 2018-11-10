#   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)

if(UNIX)
  if(CMAKE_MAJOR_VERSION EQUAL 2 AND CMAKE_MINOR_VERSION LESS 5)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR} )
  else()
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )
  endif()
endif()



set(libname GrasplotPA10)

set(sources
GrasplotPluginPA10.cpp
)

set(headers
GrasplotPluginPA10.h
)

if(MSVC)
  set(sources ${sources} ../../../Grasp/Arm.cpp)
endif()

include_directories(${Choreonoid_SOURCE_DIR})

add_library(${libname} SHARED ${sources} ${headers} )

set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)

if(MSVC)
 set_target_properties(${libname} PROPERTIES DEBUG_POSTFIX d )
 add_definitions(-DEXCADE_GRASPLOT_MAKE_DLL)
endif()

#target_link_libraries(${libname} Excade${EXCADE_VERSION_SUFFIX} ExcadeGrasplot${EXCADE_VERSION_SUFFIX})
target_link_libraries(${libname} CnoidGraspPlugin)

if(MSVC)
install(TARGETS ${libname}
    RUNTIME DESTINATION bin/PA10 CONFIGURATIONS Release Debug
    LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR} CONFIGURATIONS Release Debug
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug)
else()
  install(TARGETS ${libname}
    RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
    LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR} CONFIGURATIONS Release Debug
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug)
endif()

if(INSTALL_SDK)
  install(FILES ${headers} DESTINATION ${CNOID_HEADER_SUBDIR}/extplugin/graspPlugin/Grasp)
endif()


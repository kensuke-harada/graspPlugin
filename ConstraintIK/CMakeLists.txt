set(libname CnoidConstraintIKPlugin)

set(sources
  ConstraintIKPlugin.cpp
  ConstraintIKBar.cpp
  ConstraintIK.cpp
  ObstacleShape.cpp
  )

set(headers
  ConstraintIKBar.h
  ConstraintIK.h
  ObstacleShape.h
  eiquadprog.hpp
  )

if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
  add_library(${libname} SHARED ${sources} ${headers})
else()
  add_cnoid_plugin(${libname} SHARED ${sources} ${headers})
endif()

set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)
#set_target_properties(${libname} PROPERTIES DEFINE_SYMBOL "EXCADE_SAMPLE1_MAKE_DLL")

if(MSVC)
  add_definitions(-DEXCADE_CONSTRAINTIK_MAKE_DLL)
endif()

target_link_libraries(${libname} CnoidBodyPlugin CnoidGraspPlugin CnoidGeometryHandlerPlugin)

apply_common_setting_for_plugin(${libname})

#add_subdirectory(project)

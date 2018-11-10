
# @author Shin'ichiro Nakaoka


set(libname CnoidPlacePlanPlugin)

set(sources
  PlacePlanPlugin.cpp
  PlacePlanner.cpp
  PlaceBar.cpp
  CollisionPair.cpp
  ZmapInterface.cpp
)

set(headers
  PlacePlanner.h
  PlaceBar.h
  CollisionPair.h
  ZmapInterface.h
)

if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
 add_library(${libname} SHARED ${sources} ${headers})
else()
 add_cnoid_plugin(${libname} SHARED ${sources} ${headers})
endif()

#add_definitions(-Wall -fPIC -O2 -I/usr/include -I/usr/include/rtm/idl -I.)
#include_directories(${rtc-dir})

set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)

if(MSVC)
  add_definitions(-DEXCADE_OBJECTPLACEPLANNER_MAKE_DLL)
endif()

target_link_libraries(${libname} CnoidBodyPlugin CnoidGraspPlugin CnoidGeometryHandlerPlugin )

apply_common_setting_for_plugin(${libname})
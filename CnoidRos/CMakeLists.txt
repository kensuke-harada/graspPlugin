set(target CnoidRosPlugin)

set(sources CnoidRosPlugin.cpp)

if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
  add_library(${target} SHARED ${sources} ${headers})
else()
  add_cnoid_plugin(${target} SHARED ${sources} ${headers})
endif()

if(MSVC)
  add_definitions(-DEXCADE_DUALARMGRASP_MAKE_DLL)
endif()

target_link_libraries(${target} CnoidBodyPlugin)

apply_common_setting_for_plugin(${target})

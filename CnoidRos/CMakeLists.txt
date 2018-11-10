set(libname CnoidRosPlugin)

set(sources CnoidRosPlugin.cpp)

if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
 add_library(${libname} SHARED ${sources} ${headers})
else()
 add_cnoid_plugin(${libname} SHARED ${sources} ${headers})
endif()

if(MSVC)
	add_definitions(-DEXCADE_DUALARMGRASP_MAKE_DLL)
endif()

target_link_libraries(${libname} CnoidBodyPlugin)

apply_common_setting_for_plugin(${libname})


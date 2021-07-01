
# @author Tokuo Tsuji, Shin'ichiro Nakaoka (AIST)



set(target CnoidGraspRtcPlugin)

set(rtcdir ${CMAKE_CURRENT_SOURCE_DIR}/rtc)

set(sources
  GraspRtcPlugin.cpp
  GraspRtcController.cpp
  GraspRtcBar.cpp
  ${rtcdir}/GraspControllerSVC_impl.cpp
  ${rtcdir}/GraspControllerSkel.cpp
  ${rtcdir}/GraspConsumer.cpp
  )

set(headers
  GraspRtcBar.h
  GraspRtcController.h
  )

generate_idl_cpp(
  IDLFILE ${rtcdir}/GraspController.idl
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/rtc
  GENERATED_FILES_VARIABLE idl_cpp_stubs
  )

add_custom_target(GraspController_idl_cpp DEPENDS ${idl_cpp_stubs})

include_directories(${OPENRTM_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/rtc)
link_directories(${OPENRTM_LIBRARY_DIRS})

if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
  add_library(${target} SHARED ${sources} ${headers})
else()
  add_cnoid_plugin(${target} SHARED ${sources} ${headers})
endif()

add_dependencies(${target} GraspController_idl_cpp)

set_target_properties(${target} PROPERTIES VERSION 0.0.0 SOVERSION 0)

target_link_libraries(${target} CnoidBodyPlugin CnoidGraspPlugin uuid dl pthread omniORB4 omnithread omniDynamic4 RTC coil  )

apply_common_setting_for_plugin(${target})

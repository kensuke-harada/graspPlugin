
# @author Tokuo Tsuji, Shin'ichiro Nakaoka (AIST)



set(target CnoidGraspRtcPlugin)

set(rtcdir ${CMAKE_CURRENT_SOURCE_DIR}/rtc)

set(sources
  GraspRtcPlugin.cpp
  GraspRtcController.cpp
  GraspRtcBar.cpp
  GraspControllerSVC_impl.cpp
  ${rtcdir}/GraspControllerSkel.cpp
  ${rtcdir}/GraspConsumer.cpp
  )

set(headers
  GraspConsumer.h
  GraspRtcBar.h
  GraspRtcController.h
  )


include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})

if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
  add_library(${target} SHARED ${sources} ${headers})
else()
  add_cnoid_plugin(${target} SHARED ${sources} ${headers})
endif()

set_target_properties(${target} PROPERTIES VERSION 0.0.0 SOVERSION 0)

target_link_libraries(${target} CnoidBodyPlugin CnoidGraspPlugin uuid dl pthread omniORB4 omnithread omniDynamic4 RTC coil  )

apply_common_setting_for_plugin(${target})

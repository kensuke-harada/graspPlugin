
# @author Tokuo Tsuji, Shin'ichiro Nakaoka (AIST)



set(libname CnoidGraspRtcPlugin)

set(rtc-dir ${CMAKE_CURRENT_SOURCE_DIR}/rtc/)

set(sources
GraspRtcPlugin.cpp
GraspRtcController.cpp
GraspRtcBar.cpp
GraspControllerSVC_impl.cpp
${rtc-dir}GraspControllerSkel.cpp
${rtc-dir}GraspConsumer.cpp
)

set(headers
GraspConsumer.h
GraspRtcBar.h
GraspRtcController.h
  )


include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})

if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
 add_library(${libname} SHARED ${sources} ${headers})
else()
 add_cnoid_plugin(${libname} SHARED ${sources} ${headers})
endif()

set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)

target_link_libraries(${libname} CnoidBodyPlugin CnoidGraspPlugin uuid dl pthread omniORB4 omnithread omniDynamic4 RTC coil  )

apply_common_setting_for_plugin(${libname})


# c) Tokuo Tsuji (Kyushu univ./AIST)


set(target CnoidVisionTriggerPlugin)

set(rtcdir ${CMAKE_CURRENT_SOURCE_DIR}/rtc)

set(sources
  VisionTriggerPlugin.cpp
  VisionTriggerRtcController.cpp
  VisionTriggerBar.cpp
  ObjectRecognitionResultManipulator.cpp
  ${rtcdir}/RecognitionServiceSkel.cpp
  # ${rtcdir}/RecognitionServiceSVC_impl.cpp
  ${rtcdir}/RecognitionServiceExtSkel.cpp
  ${rtcdir}/RecognitionServiceExtSVC_impl.cpp
  ${rtcdir}/VisionRecognitionTrigger.cpp
  )

set(headers
  VisionTriggerRtcBar.h
  VisionTriggerRtcController.h
  ObjectRecognitionResultManipulator.h
  )

generate_idl_cpp(
  IDLFILE ${rtcdir}/RecognitionService.idl
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/rtc
  GENERATED_FILES_VARIABLE idl_cpp_stubs
  )

add_custom_target(RecognitionService_idl_cpp DEPENDS ${idl_cpp_stubs})

generate_idl_cpp(
  IDLFILE ${rtcdir}/RecognitionServiceExt.idl
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/rtc
  GENERATED_FILES_VARIABLE idl_cpp_ext_stubs
  )

add_custom_target(RecognitionServiceExt_idl_cpp DEPENDS ${idl_cpp_ext_stubs})

include_directories(${OPENRTM_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/rtc)
link_directories(${OPENRTM_LIBRARY_DIRS})

if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
  add_library(${target} SHARED ${sources} ${headers})
else()
  add_cnoid_plugin(${target} SHARED ${sources} ${headers})
endif()

add_dependencies(${target} RecognitionService_idl_cpp)
add_dependencies(${target} RecognitionServiceExt_idl_cpp)

#set_target_properties(${target} PROPERTIES VERSION 0.0.0 SOVERSION 0 LINK_FLAGS -export-dynamic -Wl,-z,defs -L/usr/lib)
set_target_properties(${target} PROPERTIES VERSION 0.0.0 SOVERSION 0)

#add_definitions(-export-dynamic -Wl,-z,defs -luuid -ldl -L/usr/lib -lpthread -lomniORB4 -lomnithread -lomniDynamic4 -lRTC -lcoil)

#add_custom_command(
#	OUTPUT ${target}
#	COMMAND g++ ${target}   `rtm-config --libs`
#	DEPENDS ${sources}
#	)

target_link_libraries(${target}  CnoidBodyPlugin CnoidGraspPlugin  uuid dl pthread omniORB4 omnithread omniDynamic4 RTC coil  )
#target_link_libraries(${target} ${OPENRTM_LIBRARIES} Excade${EXCADE_VERSION_SUFFIX} ExcadeGrasplot${EXCADE_VERSION_SUFFIX}   )

apply_common_setting_for_plugin(${target})

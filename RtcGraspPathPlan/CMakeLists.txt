
# @author Tokuo Tsuji, Shin'ichiro Nakaoka (AIST)



set(target CnoidRtcGraspPathPlugin)

set(rtcdir ${CMAKE_CURRENT_SOURCE_DIR}/rtc)

set(sources
  GraspPathPlugin.cpp
  GraspPathController.cpp
  GraspPathBar.cpp
  ${rtcdir}/GraspPathControllerSVC_impl.cpp
  ${rtcdir}/GraspPathControllerSkel.cpp
  ${rtcdir}/GraspPathPlanner.cpp
  )

set(headers
  GraspPathBar.h
  GraspPathController.h
  )

if(MSVC)
  set(OPENRTM_DIR ${OPENRTM_DIR} CACHE PATH "set the directory of OpenRTM-aist")
  if(NOT OPENRTM_DIR)
    message(FATAL_ERROR "Please specify the OpenRTM-aist top directory to OPENRTM_DIR.")
  endif()
  link_directories(${OPENRTM_DIR}/bin)
endif()

if(UNIX)
  generate_idl_cpp(
    IDLFILE ${rtcdir}/GraspPathController.idl
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/rtc
    GENERATED_FILES_VARIABLE idl_cpp_stubs
    )

  add_custom_target(GraspPathController_idl_cpp DEPENDS ${idl_cpp_stubs})

  include_directories(${OPENRTM_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/rtc)
  link_directories(${OPENRTM_LIBRARY_DIRS})
elseif(MSVC)
  add_definitions(-DEXCADE_GRASPLOT_MAKE_DLL -DRTC_CORBA_CXXMAPPING11 -D_WIN32_WINNT=0x0500)
  include_directories(${OPENRTM_DIR})
  include_directories(${OPENRTM_DIR}/rtm/idl)
  include_directories(.)
  include_directories(./rtc)
endif(UNIX)
if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
  add_library(${target} SHARED ${sources} ${headers})
else()
  add_cnoid_plugin(${target} SHARED ${sources} ${headers})
endif()

add_dependencies(${target} GraspPathController_idl_cpp)

set_target_properties(${target} PROPERTIES VERSION 0.0.0 SOVERSION 0)

if(UNIX)
  target_link_libraries(${target} CnoidBodyPlugin CnoidGraspPlugin CnoidPRMPlugin uuid dl pthread omniORB4 omnithread omniDynamic4 RTC coil  )
elseif(MSVC)
  target_link_libraries(${target}
    general CnoidBodyPlugin CnoidGraspPlugin CnoidPRMPlugin uuid ws2_32 ${OMNIORB_LIBRARIES}
    optimized RTC110 optimized coil110
    debug  RTC110d debug coil110d
    )
endif(UNIX)

apply_common_setting_for_plugin(${target})

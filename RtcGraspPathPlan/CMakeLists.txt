
# @author Tokuo Tsuji, Shin'ichiro Nakaoka (AIST)



set(libname CnoidRtcGraspPathPlugin)

set(rtc-dir ${CMAKE_CURRENT_SOURCE_DIR}/rtc/)

set(sources
GraspPathPlugin.cpp
GraspPathController.cpp
GraspPathBar.cpp
GraspPathControllerSVC_impl.cpp
${rtc-dir}GraspPathControllerSkel.cpp
${rtc-dir}GraspPathPlanner.cpp
)

set(headers
GraspPathController.h
)

if(MSVC)
	set(OPENRTM_DIR ${OPENRTM_DIR} CACHE PATH "set the directory of OpenRTM-aist")
	if(NOT OPENRTM_DIR)
		message(FATAL_ERROR "Please specify the OpenRTM-aist top directory to OPENRTM_DIR.")
	endif()
	link_directories(${OPENRTM_DIR}/bin)
endif()

if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
 add_library(${libname} SHARED ${sources} ${headers})
else()
 add_cnoid_plugin(${libname} SHARED ${sources} ${headers})
endif()
if(UNIX)
#    find_package(OpenRTM)
#    add_definitions(-Wall -fPIC -O2 -I${OPENRTM_INCLUDE_DIR} -I${OPENRTM_INCLUDE_DIR}/rtm/idl -I. -I${COIL_INCLUDE_DIR})
    add_definitions(-Wall -fPIC -O2 -I/usr/include -I/usr/include/rtm/idl -I/usr/include/openrtm-1.1 -I/usr/include/coil-1.1 -I/usr/include/openrtm-1.1/rtm/idl -I.)
	include_directories(${rtc-dir})
elseif(MSVC)
	add_definitions(-DEXCADE_GRASPLOT_MAKE_DLL -DRTC_CORBA_CXXMAPPING11 -D_WIN32_WINNT=0x0500)
	include_directories(${OPENRTM_DIR})
	include_directories(${OPENRTM_DIR}/rtm/idl)
	include_directories(.)
	include_directories(./rtc)
endif(UNIX)

set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)

if(UNIX)
target_link_libraries(${libname} CnoidBodyPlugin CnoidGraspPlugin CnoidPRMPlugin uuid dl pthread omniORB4 omnithread omniDynamic4 RTC coil  )
elseif(MSVC)
target_link_libraries(${libname}
 general CnoidBodyPlugin CnoidGraspPlugin CnoidPRMPlugin uuid ws2_32 ${OMNIORB_LIBRARIES}
  optimized RTC110 optimized coil110
  debug  RTC110d debug coil110d
)
endif(UNIX)

apply_common_setting_for_plugin(${libname})

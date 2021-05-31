
# @author Shin'ichiro Nakaoka (AIST)

if(UNIX)
  set(rtc-dir ${CMAKE_CURRENT_SOURCE_DIR}/ArmControllerRtc)
else()
   find_package(OpenRTM)
   if(OPENRTM_VERSION VERSION_EQUAL 1.1.2)
     set(rtc-dir ${CMAKE_CURRENT_SOURCE_DIR}/ArmControllerRtc1_1_2)
	 add_definitions(-DOPENRTM112)
   else()
     set(rtc-dir ${CMAKE_CURRENT_SOURCE_DIR}/ArmControllerRtc)
   endif()
endif()

set(libname CnoidRobotInterfacePlugin)

set(sources
	RobotInterfacePlugin.cpp
	RobotInterface.cpp
	RobotInterfaceBar.cpp
	ArmControllerRtc.cpp
	HistoricMotionState.cpp

    ${rtc-dir}/ArmController.cpp
    ${rtc-dir}/HIROControllerStub.cpp
    ${rtc-dir}/HiroNXStub.cpp
    ${rtc-dir}/ManipulatorCommonInterface_CommonStub.cpp
    ${rtc-dir}/ManipulatorCommonInterface_MiddleLevelStub.cpp
    ${rtc-dir}/ManipulatorCommonInterface_DataTypesSK.cc

	../PickAndPlacePlanner/RobotLocalFunctions.cpp
)

set(headers
	ArmControllerRtc.h
	RobotInterface.h
	RobotInterfaceBar.h
	HistoricMotionState.h

    ${rtc-dir}/ArmController.h
    ${rtc-dir}/HIROController.hh
    ${rtc-dir}/HIROControllerStub.h
    ${rtc-dir}/HiroNX.hh
    ${rtc-dir}/HiroNXStub.h
    ${rtc-dir}/ManipulatorCommonInterface_Common.hh
    ${rtc-dir}/ManipulatorCommonInterface_CommonStub.h
    ${rtc-dir}/ManipulatorCommonInterface_MiddleLevel.hh
    ${rtc-dir}/ManipulatorCommonInterface_MiddleLevelStub.h
    ${rtc-dir}/ManipulatorCommonInterface_DataTypes.hh
	../PickAndPlacePlanner/RobotLocalFunctions.h
  )

find_package(PythonLibs REQUIRED)

if(UNIX)
  include_directories(${OPENRTM_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIR})
  link_directories(${OPENRTM_LIBRARY_DIRS})
else()
  include_directories(${PYTHON_INCLUDE_PATH} ${rtc-dir} . ${OPENRTM_INCLUDE_DIRS} ${OMNIORB_INCLUDE_DIRS})
  string(REPLACE ";" " " openrtm_flag "${OPENRTM_CFLAGS}")
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLABS} "${openrtm_flag}")
  link_directories(${OPENRTM_LIBRARY_DIRS})
endif()

if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
 add_library(${libname} SHARED ${sources} ${headers})
else()
 add_cnoid_plugin(${libname} SHARED ${sources} ${headers})
endif()

set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)

if(UNIX)

  target_link_libraries(${libname}
  	CnoidBodyPlugin CnoidGraspPlugin
  	${PYTHON_LIBRARIES} uuid dl pthread
  	omniORB4 omnithread omniDynamic4 RTC coil)

  apply_common_setting_for_plugin(${libname})

elseif(MSVC)
  add_definitions(-DEXCADE_ROBOTINTERFACE_MAKE_DLL)
  source_group("Header Files" FILES ${headers})
  target_link_libraries(${libname} #Excade${EXCADE_VERSION_SUFFIX}
  CnoidBodyPlugin CnoidGraspPlugin
  ${PYTHON_LIBRARIES}
  ${OPENRTM_LIBRARIES}
  )

  set_target_properties(${libname} PROPERTIES COMPILE_FLAGS "/wd4250 /wd4275")

if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
  install(TARGETS ${libname}
    RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
    LIBRARY DESTINATION lib CONFIGURATIONS Release Debug)
elseif(CNOID_VERSION EQUAL 1.5)
  apply_common_setting_for_plugin(${libname})
else()
  set_target_properties(${libname} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR})
endif()

endif(UNIX)

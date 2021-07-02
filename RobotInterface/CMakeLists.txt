
# @author Shin'ichiro Nakaoka (AIST)

if(UNIX)
  set(rtcdir ${CMAKE_CURRENT_SOURCE_DIR}/ArmControllerRtc)
else()
  find_package(OpenRTM)
  if(OPENRTM_VERSION VERSION_EQUAL 1.1.2)
    set(rtcdir ${CMAKE_CURRENT_SOURCE_DIR}/ArmControllerRtc1_1_2)
    add_definitions(-DOPENRTM112)
  else()
    set(rtcdir ${CMAKE_CURRENT_SOURCE_DIR}/ArmControllerRtc)
  endif()
endif()

set(target CnoidRobotInterfacePlugin)

set(sources
  RobotInterfacePlugin.cpp
  RobotInterface.cpp
  RobotInterfaceBar.cpp
  ArmControllerRtc.cpp
  HistoricMotionState.cpp

  ${rtcdir}/ArmController.cpp
  ${rtcdir}/HIROControllerStub.cpp
  ${rtcdir}/HiroNXStub.cpp
  ${rtcdir}/ManipulatorCommonInterface_CommonStub.cpp
  ${rtcdir}/ManipulatorCommonInterface_MiddleLevelStub.cpp

  ../PickAndPlacePlanner/RobotLocalFunctions.cpp
  )

set(headers
  ArmControllerRtc.h
  RobotInterface.h
  RobotInterfaceBar.h
  HistoricMotionState.h

  ${rtcdir}/ArmController.h
  ${rtcdir}/HIROControllerStub.h
  ${rtcdir}/HiroNXStub.h
  ${rtcdir}/ManipulatorCommonInterface_CommonStub.h
  ${rtcdir}/ManipulatorCommonInterface_MiddleLevelStub.h
  ../PickAndPlacePlanner/RobotLocalFunctions.h
  )

find_package(PythonLibs REQUIRED)

if(UNIX)
  generate_idl_cpp(
    IDLFILE ${rtcdir}/BasicDataType.idl
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/ArmControllerRtc
    GENERATED_FILES_VARIABLE BasicDataType_idl_cpp_stubs
    DISABLE_Wba
    )
  add_custom_target(BasicDataType_idl_cpp DEPENDS ${BasicDataType_idl_cpp_stubs})

  generate_idl_cpp(
    IDLFILE ${rtcdir}/HIROController.idl
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/ArmControllerRtc
    GENERATED_FILES_VARIABLE HIROController_idl_cpp_stubs
    )
  add_custom_target(HIROController_idl_cpp DEPENDS ${HIROController_idl_cpp_stubs})

  generate_idl_cpp(
    IDLFILE ${rtcdir}/HiroNX.idl
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/ArmControllerRtc
    GENERATED_FILES_VARIABLE HiroNX_idl_cpp_stubs
    )
  add_custom_target(HiroNX_idl_cpp DEPENDS ${HiroNX_idl_cpp_stubs})

  generate_idl_cpp(
    IDLFILE ${rtcdir}/ManipulatorCommonInterface_Common.idl
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/ArmControllerRtc
    GENERATED_FILES_VARIABLE ManipulatorCommonInterface_Common_idl_cpp_stubs
    )
  add_custom_target(ManipulatorCommonInterface_Common_idl_cpp
    DEPENDS ${ManipulatorCommonInterface_Common_idl_cpp_stubs})

  generate_idl_cpp(
    IDLFILE ${rtcdir}/ManipulatorCommonInterface_DataTypes.idl
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/ArmControllerRtc
    GENERATED_FILES_VARIABLE ManipulatorCommonInterface_DataTypes_idl_cpp_stubs
    DISABLE_Wba
    )
  add_custom_target(ManipulatorCommonInterface_DataTypes_idl_cpp
    DEPENDS ${ManipulatorCommonInterface_DataTypes_idl_cpp_stubs})

  generate_idl_cpp(
    IDLFILE ${rtcdir}/ManipulatorCommonInterface_MiddleLevel.idl
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/ArmControllerRtc
    GENERATED_FILES_VARIABLE ManipulatorCommonInterface_MiddleLevel_idl_cpp_stubs
    )
  add_custom_target(ManipulatorCommonInterface_MiddleLevel_idl_cpp
    DEPENDS ${ManipulatorCommonInterface_MiddleLevel_idl_cpp_stubs})

  include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}/ArmControllerRtc
    ${OPENRTM_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIR}
    )
  link_directories(${OPENRTM_LIBRARY_DIRS})
else()
  include_directories(${PYTHON_INCLUDE_PATH} ${rtcdir} . ${OPENRTM_INCLUDE_DIRS} ${OMNIORB_INCLUDE_DIRS})
  string(REPLACE ";" " " openrtm_flag "${OPENRTM_CFLAGS}")
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLABS} "${openrtm_flag}")
  link_directories(${OPENRTM_LIBRARY_DIRS})
endif()

if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
  add_library(${target} SHARED ${sources} ${headers})
else()
  add_cnoid_plugin(${target} SHARED ${sources} ${headers})
endif()

add_dependencies(${target} BasicDataType_idl_cpp)
add_dependencies(${target} HIROController_idl_cpp)
add_dependencies(${target} HiroNX_idl_cpp)
add_dependencies(${target} ManipulatorCommonInterface_Common_idl_cpp)
add_dependencies(${target} ManipulatorCommonInterface_DataTypes_idl_cpp)
add_dependencies(${target} ManipulatorCommonInterface_MiddleLevel_idl_cpp)

set_target_properties(${target} PROPERTIES VERSION 0.0.0 SOVERSION 0)

if(UNIX)

  target_link_libraries(${target}
    CnoidBodyPlugin CnoidGraspPlugin
    ${PYTHON_LIBRARIES} uuid dl pthread
    omniORB4 omnithread omniDynamic4 RTC coil)

  apply_common_setting_for_plugin(${target})

elseif(MSVC)
  add_definitions(-DEXCADE_ROBOTINTERFACE_MAKE_DLL)
  source_group("Header Files" FILES ${headers})
  target_link_libraries(${target} #Excade${EXCADE_VERSION_SUFFIX}
    CnoidBodyPlugin CnoidGraspPlugin
    ${PYTHON_LIBRARIES}
    ${OPENRTM_LIBRARIES}
    )

  set_target_properties(${target} PROPERTIES COMPILE_FLAGS "/wd4250 /wd4275")

  if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
    install(TARGETS ${target}
      RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
      LIBRARY DESTINATION lib CONFIGURATIONS Release Debug)
  elseif(CNOID_VERSION EQUAL 1.5)
    apply_common_setting_for_plugin(${target})
  else()
    set_target_properties(${target} PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}
      ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}
      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR})
  endif()

endif(UNIX)

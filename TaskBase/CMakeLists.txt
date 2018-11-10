# c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)

set(libname CnoidTaskBasePlugin)

set(sources
TaskBasePlugin.cpp
TaskBaseBar.cpp
Action.cpp
ActionSequence.cpp
RobotMotionGenerator.cpp
Task.cpp
GraspPoints.cpp
ExportAssemblyGraphDialog.cpp
AssemblyGraphExporter.cpp
)

set(headers
TaskBaseBar.h
Action.h
ActionSequence.h
RobotMotionGenerator.h
Task.h
GraspPoints.h
ExportAssemblyGraphDialog.h
AssemblyGraphExporter.h
)




#add_definitions(-DDO_TOLERANCE_TEST)
#add_definitions(-DTOLERANCE_SMOOTHER)

if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
 add_library(${libname} SHARED ${sources} ${headers})
else()
 add_cnoid_plugin(${libname} SHARED ${sources} ${headers})
endif()

set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)

if(UNIX)

  target_link_libraries(${libname} CnoidBodyPlugin CnoidPoseSeqPlugin CnoidGraspPlugin CnoidPRMPlugin)
  apply_common_setting_for_plugin(${libname})

elseif(MSVC)
  add_definitions(-DEXCADE_OPENHRP_MAKE_DLL)
  target_link_libraries(${libname} CnoidBodyPlugin CnoidPoseSeqPlugin CnoidGraspPlugin CnoidPRMPlugin)

  source_group("Header Files" FILES ${headers})

  set_target_properties(${libname} PROPERTIES LINK_FLAGS /NODEFAULTLIB:LIBCMT )
  set_target_properties(${libname} PROPERTIES DEBUG_POSTFIX d )
  set_target_properties(${libname} PROPERTIES COMPILE_FLAGS "/wd4250 /wd4275")

if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
  install(TARGETS ${libname}
    RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
    LIBRARY DESTINATION lib CONFIGURATIONS Release Debug)
else()    
  set_target_properties(${libname} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR})
endif()
    
    apply_common_setting_for_plugin(${libname})
endif(UNIX)

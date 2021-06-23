set(target CnoidGraspDataGenPlugin)

set(sources
  GraspDataGenPlugin.cpp
  GraspDataGenBar.cpp
  GraspDataGenerator.cpp
  WaistPositionSearcher.cpp
  ConvexHull.cpp
  ParamDialog.cpp
  PrehensionParamHandler.cpp
  GraspPatternDialog.cpp
  BoxInfo.cpp
  GraspableRegion.cpp
  WaistPosSearchDialog.cpp
  PathSearcher.cpp
  WaistPosSearchMotion.cpp
  WaistPosSearchResultViewDialog.cpp
  ../PickAndPlacePlanner/GraspDatabaseManipulator.cpp
  )

set(headers
  GraspDataGenBar.h
  GraspDataGenerator.h
  WaistPositionSearcher.h
  ConvexHull.h
  ParamDialog.h
  PrehensionParamHandler.h
  GraspPatternDialog.h
  BoxInfo.h
  GraspableRegion.h
  WaistPosSearchDialog.h
  PathSearcher.h
  WaistPosSearchMotion.h
  WaistPosSearchResultViewDialog.h
  ../PickAndPlacePlanner/GraspDatabaseManipulator.h
  )

#add_definitions(-DTHREAD)

if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
  add_library(${target} SHARED ${sources} ${headers})
else()
  add_cnoid_plugin(${target} SHARED ${sources} ${headers})
endif()

set_target_properties(${target} PROPERTIES VERSION 0.0.0 SOVERSION 0)
#set_target_properties(${target} PROPERTIES DEFINE_SYMBOL "EXCADE_SAMPLE1_MAKE_DLL")

include(FindPythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

if(MSVC)
  add_definitions(-DEXCADE_GRASPDATAGEN_MAKE_DLL)
endif()

target_link_libraries(${target} CnoidBodyPlugin CnoidPoseSeqPlugin CnoidGraspPlugin CnoidGeometryHandlerPlugin CnoidSoftFingerStabilityPlugin CnoidConstraintIKPlugin CnoidGraspPCLPlugin)

apply_common_setting_for_plugin(${target})

#add_subdirectory(project)

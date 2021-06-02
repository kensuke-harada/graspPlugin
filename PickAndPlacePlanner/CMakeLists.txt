# c) Kensuke Harada (AIST)

set(libname CnoidPickAndPlacePlannerPlugin)

set(grasp-dir ../grasp)
include_directories(${grasp-dir})

set(sources
  ManipPlugin.cpp
  ManipController.cpp
  RobotLocalFunctions.cpp
  ManipBar.cpp
  DualArmManipulation.cpp
  ManipFailureAnalyzer.cpp
  GraspDatabaseManipulator.cpp
  ObjectPosReader.cpp
  SweptVolume.cpp
  SweptVolumeDialog.cpp
  PenaltyFunction.cpp
  LearningDialog.cpp
  AbstractClassifier.cpp
  RandomForestClassifier.cpp
  SVMClassifier.cpp
  libsvm/svm.cpp
  LearningDataManipulator.cpp
  )

set(headers
  ManipBar.h
  ManipController.h
  RobotLocalFunctions.h
  DataConv.h
  DualArmManipulation.h
  ManipFailureAnalyzer.h
  GraspDatabaseManipulator.h
  ObjectPosReader.h
  SweptVolume.h
  SweptVolumeDialog.h
  PenaltyFunction.h
  LearningDialog.h
  AbstractClassifier.h
  RandomForestClassifier.h
  SVMClassifier.h
  ClassifierFactory.h
  libsvm/svm.h
  LearningDataManipulator.h
  )

#add_subdirectory(utils)

if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
  add_library(${libname} SHARED ${sources} ${headers})
else()
  add_cnoid_plugin(${libname} SHARED ${sources} ${headers})
endif()

set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)
#set_target_properties(${libname} PROPERTIES DEFINE_SYMBOL "EXCADE_SAMPLE1_MAKE_DLL")

if(UNIX)
  add_definitions(-g -fPIC -O2 -I/usr/include -I.)
endif()

option(USE_DB_IN_PICKANDPLACE "Use DB in pick and place" OFF)

if(USE_DB_IN_PICKANDPLACE)
  add_definitions(-DGRASPPOSTURE_FROM_DB)
endif()

if(UNIX)
  target_link_libraries(${libname} CnoidBodyPlugin CnoidGraspPlugin CnoidGeometryHandlerPlugin CnoidPRMPlugin CnoidPlacePlanPlugin
    uuid dl pthread )
elseif(MSVC)
  add_definitions(-DEXCADE_PICKANDPLACEPLANNER_MAKE_DLL)
  target_link_libraries(${libname} CnoidBodyPlugin CnoidGraspPlugin CnoidGeometryHandlerPlugin CnoidPRMPlugin CnoidPlacePlanPlugin)
  install(TARGETS ${libname}
    RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
    LIBRARY DESTINATION lib CONFIGURATIONS Release Debug)

endif(UNIX)

apply_common_setting_for_plugin(${libname})

if(ENABLE_PYTHON)
  add_subdirectory(python)
endif()

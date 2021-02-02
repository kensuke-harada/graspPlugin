# c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)

set(libname CnoidGraspPlugin)

set(sources
  GraspPlugin.cpp
  PlanBase.cpp
  PlanInterface.cpp
  GraspController.cpp
  PlaceController.cpp
  Finger.cpp
  Arm.cpp
  readtext.cpp
  ConvexAnalysis.cpp 
  VectorMath.cpp
  ForceClosureTest.cpp
  GraspBar.cpp
  GraspPluginManager.cpp
  InterObject.cpp
  ExtCollisionPairInserter.cpp
  ExtTriOverlap.cpp
  PrehensionFileMenu.cpp
  PrehensionParameter.cpp
	RobotBodyColChecker.cpp
  Parallelizer.cpp
	TransformationMatrixReader.cpp
	Camera.cpp
	CameraSelectDialog.cpp
	RobotHand.cpp
	RobotBody.cpp
	AssemblyObject.cpp
	MultiHandObjectColChecker.cpp
	HandSelectDialog.cpp
	ColdetModelGetter.cpp
	ObjectBase.cpp
	ObjectManager.cpp
	Connection.cpp
	ObjectPositionState.cpp
	PointCloudCollisionChecker.cpp
	ObjectTraversal.cpp
	ObjectPoseEstimationSolution.cpp
	ModelShapeWrapper.cpp
	IniFileParameterReader.cpp
)

set(headers
	Arm.h
	Finger.h
	ConvexAnalysis.h
	ForceClosureTest.h
	GraspBar.h
	PlanBase.h
	PlanInterface.h
	GraspController.h
	PlaceController.h
	GraspSceneBody.h
	VectorMath.h
	readtext.h
	GraspPluginManager.h
	exportdef.h
	DrawUtility.h
	InterObject.h
	ExtCollisionPairInserter.h
	PrehensionFileMenu.h
	PrehensionParameter.h
	RobotBodyColChecker.h
	Parallelizer.h
	ClearanceBoundingVolume.h
	TransformationMatrixReader.h
	Camera.h
	CameraSelectDialog.h
	PointCloudEnv.h
	RobotHand.h
	RobotBody.h
	AssemblyObject.h
	MultiHandObjectColChecker.h
	HandSelectDialog.h
	ColdetModelGetter.h
	ObjectBase.h
	ObjectManager.h
	Connection.h
	ObjectPositionState.h
	PointCloudCollisionChecker.h
	ObjectTraversal.h
	ObjectPoseEstimationSolution.h
	ModelShapeWrapper.h
	IniFileParameterReader.h
	OccupiedVoxel.h
)

if((CNOID_VERSION VERSION_EQUAL 1.5) OR (CNOID_VERSION VERSION_GREATER 1.5))
	set(sources ${sources}
		SceneWidgetEventEaterManager.cpp
		GraspEventEater.cpp
		DrawFunctions.cpp
		)
	set(headers ${headers}
		SceneWidgetEventEaterManager.h
		EventEater.h
		GraspEventEater.h
		DrawFunctions.h
		)
endif()

if((CNOID_VERSION VERSION_EQUAL 1.6) OR (CNOID_VERSION VERSION_GREATER 1.6))
	set(sources ${sources}
		SgStringRenderer.cpp
		)
endif()

if(MSVC)
  if((CNOID_VERSION VERSION_EQUAL 1.5) OR (CNOID_VERSION VERSION_GREATER 1.5))
    set(sources ${sources}   ../../../src/AISTCollisionDetector/TriOverlap.cpp)
  else()
    set(sources ${sources}   ../../../src/Collision/TriOverlap.cpp)
  endif()
endif()

 include(FindPythonLibs)

option(ADD_TEST_FUNCTION "Addd test funtion" OFF)
if(ADD_TEST_FUNCTION)
  add_definitions(-DTEST_FUNCTION)
 endif()

if(ENABLE_OSG)
  add_cnoid_plugin(${libname} SHARED ${sources} DrawUtility.cpp ${headers})
elseif((CNOID_VERSION VERSION_EQUAL 1.5) OR (CNOID_VERSION VERSION_GREATER 1.5))
  add_cnoid_plugin(${libname} SHARED ${sources} DrawUtility15.cpp ${headers})
else()
  add_cnoid_plugin(${libname} SHARED ${sources} ${headers})
endif(ENABLE_OSG)

if(UNIX)
  add_definitions(-g)
endif()
set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)
#set_target_properties(${libname} PROPERTIES DEFINE_SYMBOL "EXCADE_OPENHRP_MAKE_DLL")
include_directories(${PYTHON_INCLUDE_PATH})

if(UNIX)


	target_link_libraries(${libname} CnoidBodyPlugin qhull ${PYTHON_LIBRARIES} ${Boost_REGEX_LIBRARY} ${Boost_PYTHON_LIBRARY} ${Boost_THREAD_LIBRARY} glut)
    apply_common_setting_for_plugin(${libname})

elseif(MSVC)  
  source_group("Header Files" FILES ${headers})

  add_definitions(-DEXCADE_GRASPLOT_MAKE_DLL -Dqh_QHpointer_dllimport -Dqh_QHpointer)

  target_link_libraries(${libname}
  general CnoidBodyPlugin
#    Excade${EXCADE_VERSION_SUFFIX}
#    optimized hrpUtil-${OPENHRP_VERSION} debug hrpUtil-${OPENHRP_VERSION}d
#    optimized hrpModel-${OPENHRP_VERSION} debug hrpModel-${OPENHRP_VERSION}d
#    optimized hrpCollision-${OPENHRP_VERSION} debug hrpCollision-${OPENHRP_VERSION}d
#    optimized hrpCorbaStubSkel-${OPENHRP_VERSION} debug hrpCorbaStubSkel-${OPENHRP_VERSION}d
	${PYTHON_LIBRARIES}
	${FREEGLUT_LIBRARY}
	${QHULL__LIBRARY}
#	ExcadeRobotics${EXCADE_VERSION_SUFFIX}
  )

  set_target_properties(${libname} PROPERTIES LINK_FLAGS /NODEFAULTLIB:LIBCMT )
  set_target_properties(${libname} PROPERTIES DEBUG_POSTFIX d )
  set_target_properties(${libname} PROPERTIES COMPILE_FLAGS "/wd4250 /wd4275")

  apply_common_setting_for_plugin(${libname})

  if(INSTALL_SDK)
    install(TARGETS ${libname}
      RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
      LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
      ARCHIVE DESTINATION lib CONFIGURATIONS Release Debug)
  else()
    #install(TARGETS ${libname}
    #  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
	#  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug)
  endif()
    
  find_package(Boost 1.36.0 REQUIRED COMPONENTS python)
  install_external_libraries(${Boost_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS}
  ${Boost_REGEX_LIBRARY}   ${Boost_PYTHON_LIBRARY})
 endif(UNIX)


if(ENABLE_INSTALL_RPATH)
  SET(CMAKE_INSTALL_RPATH "$ORIGIN/..")
endif()

if(UNIX)
  if(CMAKE_MAJOR_VERSION EQUAL 2 AND CMAKE_MINOR_VERSION LESS 5)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/${RELATIVE_PLUGINS_INSTALL_PATH})
  else()
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${RELATIVE_PLUGINS_INSTALL_PATH})
  endif()
endif()

# check qhull
#find_package(qhull REQUIRED)

#include_directories(${QHULL_INCLUDE_DIRS})
#link_directories(${QHULL_LIBRARY_DIRS})

if(INSTALL_SDK)
  install(FILES ${headers} DESTINATION ${CNOID_HEADER_SUBDIR}/extplugin/graspPlugin/Grasp)
endif()

if(ENABLE_PYTHON)
  add_subdirectory(python)
endif()


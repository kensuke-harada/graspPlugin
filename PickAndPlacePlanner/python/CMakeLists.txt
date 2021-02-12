# @author Akira Ohchi

set(target PyPickAndPlacePlannerPlugin)

if(ENABLE_INSTALL_RPATH)
  SET(CMAKE_INSTALL_RPATH ".;$ORIGIN/../..")
endif()

add_cnoid_python_module(${target}
	PyPickAndPlacePlannerPluginModule.cpp
	PyLearningDataManipulator.cpp
	)

target_link_libraries(${target} CnoidPickAndPlacePlannerPlugin CnoidPyBase)

# @author Akira Ohchi

set(target PyPickAndPlacePlannerPlugin)

add_cnoid_python_module(${target}
	PyPickAndPlacePlannerPluginModule.cpp
	PyLearningDataManipulator.cpp
	)

target_link_libraries(${target} CnoidPickAndPlacePlannerPlugin CnoidPyBase)

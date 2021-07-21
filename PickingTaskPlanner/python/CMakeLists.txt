# @author Akira Ohchi

set(target PyPickingTaskPlannerPlugin)

add_cnoid_python_module(${target}
	PyPickingTaskPlannerPluginModule.cpp
	PyPickingTaskResultDrawer.cpp
	PySceneCapture.cpp
	PyResultImporter.cpp
	)

target_link_libraries(${target} CnoidPickingTaskPlannerPlugin CnoidPyBase)

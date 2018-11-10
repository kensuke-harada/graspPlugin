# @author Akira Ohchi

set(target PyGraspPlugin)

add_cnoid_python_module(${target}
	PyGraspPluginModule.cpp
	PyPlanBase.cpp
	PyGrasp.cpp
	PyAssemblyObject.cpp
	)

target_link_libraries(${target} CnoidGraspPlugin CnoidPyBase PyBody PyBodyPlugin)

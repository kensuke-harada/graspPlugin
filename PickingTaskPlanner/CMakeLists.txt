# @author Akira Ohchi

set(target CnoidPickingTaskPlannerPlugin)

set(sources
  PickingTaskPlannerPlugin.cpp
  PickingTaskPlannerBar.cpp
  PickingTaskPlanner.cpp
	JointSeqDivider.cpp
	ConfirmDialog.cpp
	PickingTaskResultDrawer.cpp
	ResultDataManipulator.cpp
)

set(headers
  PickingTaskPlannerBar.h
  PickingTaskPlanner.h
	Config.h
	JointSeqDivider.h
	ConfirmDialog.h
	PickingTaskResultDrawer.h
	ResultDataManipulator.h
)

add_cnoid_plugin(${target} SHARED ${sources} ${headers})

## for debug
add_definitions(-g)

target_link_libraries(${target}
	CnoidBodyPlugin
	CnoidPoseSeqPlugin
	CnoidGraspPlugin
	CnoidPRMPlugin
	CnoidPickAndPlacePlannerPlugin
	CnoidRobotInterfacePlugin;
	CnoidGraspDataGenPlugin
	CnoidGraspPCLPlugin	
	CnoidViewPlannerPlugin
	CnoidVisionTriggerPlugin
)

apply_common_setting_for_plugin(${target})

if(ENABLE_PYTHON)
	add_subdirectory(python)
endif()

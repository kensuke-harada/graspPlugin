/**
 * @file   PickingTaskPlannerPlugin.cpp
 * @author Akira Ohchi
 */

#include <cnoid/Plugin>

#include "PickingTaskPlannerBar.h"

class PickingTaskPlannerPlugin :
	public cnoid::Plugin {
public:
	PickingTaskPlannerPlugin() :
		Plugin("PickingTaskPlanner") {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		depend("Grasp");
		depend("PickAndPlacePlanner");
		depend("GraspPCL");
		depend("GraspDataGen");
		depend("RobotInterface");
		depend("ViewPlanner");
		depend("VisionTrigger");
#else
		require("PoseSeq");
		require("Grasp");
		require("Trajectory");
		require("PickAndPlacePlanner");
		require("GraspPCL");
		require("GraspDataGen");
		require("RobotInterface");
		require("ViewPlanner");
		require("VisionTrigger");
#endif
	}

	virtual bool initialize() {
		addToolBar(grasp::PickingTaskPlannerBar::instance());

		return true;
	}
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(PickingTaskPlannerPlugin);

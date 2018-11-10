#include "../ColdetPairData.h"
#include "../PlanBase.h"

#include <cnoid/PyBase>
#include <cnoid/BodyItem>

using namespace boost::python;
using namespace grasp;

namespace {
	PlanBase* PlanBase_Instance(PlanBase* pb = NULL) {return grasp::PlanBase::instance(pb);}

	BOOST_PYTHON_FUNCTION_OVERLOADS(PlanBase_Instance_overloads, PlanBase_Instance, 0, 1)

	bool Set_grasping_robot(PlanBase& self, cnoid::ItemPtr item, int arm_id = 0) {
		cnoid::BodyItemPtr bodyitem = cnoid::dynamic_pointer_cast<cnoid::BodyItem, cnoid::Item>(item);
		bool ret = true;
		if (!(self.targetArmFinger) || (self.bodyItemRobot()->body() != bodyitem->body())) {
			ret = self.SetGraspingRobot(bodyitem);
			if (ret) self.targetArmFinger = self.armsList[arm_id];
		}
		return ret;
	}

	BOOST_PYTHON_FUNCTION_OVERLOADS(Set_grasping_robot_overloads, Set_grasping_robot, 2, 3)

	void Select_arm(PlanBase& self, int arm_id) {
		self.targetArmFinger = self.armsList[arm_id];
	}

	void Set_grasped_object(PlanBase& self, cnoid::ItemPtr item) {
		cnoid::BodyItemPtr bodyitem = cnoid::dynamic_pointer_cast<cnoid::BodyItem, cnoid::Item>(item);
		self.SetGraspedObject(bodyitem);
	}

	bool Append_assemble_object(PlanBase& self, cnoid::ItemPtr item) {
		cnoid::BodyItemPtr bodyitem = cnoid::dynamic_pointer_cast<cnoid::BodyItem, cnoid::Item>(item);
		return self.AppendAssembleObject(bodyitem);
	}
}

void exportPlanBase() {
	class_<PlanBase>("PlanBase", no_init)
		.def("instance", PlanBase_Instance, PlanBase_Instance_overloads()[return_value_policy<reference_existing_object>()]).staticmethod("instance")
		.def("setRobot", Set_grasping_robot, Set_grasping_robot_overloads())
		.def("selectArm", Select_arm)
		.def("setObject", Set_grasped_object)
		.def("appendAssembleObject", Append_assemble_object)
		;
}

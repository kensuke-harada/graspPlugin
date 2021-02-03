#include <iostream>

#include <cnoid/PyBase>
#include <cnoid/BodyItem>
#include <cnoid/MessageView>
#include <boost/python.hpp>

#include "../PlanBase.h"
#include "../PlanInterface.h"

using namespace boost::python;
using namespace grasp;

namespace {
	void Grasp() {
		bool init = PlanBase::instance()->initial();

		if (!init) {
			cnoid::MessageView::instance()->cout() << "Failed: Grasp Planning Initial" << std::endl;
			return;
		}
		try {
			PlanInterface::instance()->doGraspPlanning();
    }
    catch(int number) {
			PlanBase::instance()->stopFlag = false;
			cnoid::MessageView::instance()->cout() << "Grasp Planning is stopped" << std::endl;
    }
    PlanBase::instance()->flush();
	}

	bool Set_grasping_robot(cnoid::ItemPtr item, int arm_id = 0) {
		cnoid::BodyItemPtr bodyitem = cnoid::dynamic_pointer_cast<cnoid::BodyItem, cnoid::Item>(item);
		bool ret = true;
		if (!(PlanBase::instance()->targetArmFinger) || (PlanBase::instance()->bodyItemRobot()->body() != bodyitem->body())) {
			ret = PlanBase::instance()->SetGraspingRobot(bodyitem);
			if (ret) PlanBase::instance()->targetArmFinger = PlanBase::instance()->armsList[arm_id];
		}
		return ret;
	}

	BOOST_PYTHON_FUNCTION_OVERLOADS(Set_grasping_robot_overloads, Set_grasping_robot, 1, 2)

	void Set_grasped_object(cnoid::ItemPtr item) {
		cnoid::BodyItemPtr bodyitem = cnoid::dynamic_pointer_cast<cnoid::BodyItem, cnoid::Item>(item);
		PlanBase::instance()->SetGraspedObject(bodyitem);
	}
}

void exportGrasp() {
	def("grasp", Grasp);
	def("set_robot", Set_grasping_robot, Set_grasping_robot_overloads());
	def("set_object", Set_grasped_object);
}

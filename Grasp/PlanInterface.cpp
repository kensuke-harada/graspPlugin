/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include "PlanInterface.h"
#include "PlanBase.h"
#include "GraspController.h"
#include "PlaceController.h"
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  


using namespace std;
using namespace cnoid;
using namespace grasp;

PlanInterface* PlanInterface::instance(PlanInterface *gc) {
	static PlanInterface* instance = (gc) ? gc : new PlanInterface();
	if(gc) instance = gc;
	return instance;
}


PlanInterface::PlanInterface()  : 	os (MessageView::mainInstance()->cout() ) {
		
}

void PlanInterface::doGraspPlanning() {

	if ( !PlanBase::instance()->targetObject || !PlanBase::instance()->targetArmFinger) {
		os << "Please select Grasped Object and Grasping Robot" << endl;
		return;
	}
	GraspController::instance()->tc = PlanBase::instance();
//	GraspController::instance()->targetArmFinger = targetArmFinger; 
//	GraspController::instance()->targetObject = targetObject; 
	GraspController::instance()->initial(PlanBase::instance()->targetObject,  PlanBase::instance()->targetArmFinger);
	GraspController::instance()->doGraspPlanning();
	
	PlanBase::instance()->graspMotionState = PlanBase::instance()->getMotionState();
	
	return;
	
	
}

bool PlanInterface::doPlacePlanning() {

	if ( !PlanBase::instance()->targetObject || !PlanBase::instance()->targetArmFinger) {
		os << "Please select Grasped Object and Grasping Robot" << endl;
		return false;
	}
	bool res = PlaceController::instance()->findFinalPose();
//	GraspController::instance()->tc = this;
//	GraspController::instance()->initial(targetObject,  targetArmFinger);
//	GraspController::instance()->doGraspPlanning();
	return res;
}

void PlanInterface::doPickAndPlacePlanning(){
	bool isSuccess;
	
	if ( !PlanBase::instance()->targetObject || !PlanBase::instance()->targetArmFinger) {
		os << "Please select Grasped Object and Grasping Robot" << endl;
		return;
	}
	
	GraspController::instance()->tc = PlanBase::instance();
	GraspController::instance()->initial(PlanBase::instance()->targetObject,  PlanBase::instance()->targetArmFinger);
	isSuccess = GraspController::instance()->doGraspPlanning();
	PlanBase::instance()->graspMotionState = PlanBase::instance()->getMotionState();
	
	PlaceController::instance()->findFinalPose();
	PlanBase::instance()->placeMotionState = PlanBase::instance()->getMotionState();
	
	pickAndPlaceMotionPlanning();
	
	return ;
}

bool PlanInterface::pickAndPlaceMotionPlanning(){
//	PlanInterface* tc = PlanInterface::instance();
//	vector<MotionState>& graspMotionSeq(graspMotionSeq);
	
	return false;
	
/*	
	Vector3 Pp_(palm()->p);
	Matrix3 Rp_(palm()->R);
	
	MotionState armJoint_init, armJoint_grasp, armJoint_lift;

	//initial setting
	armJoint_grasp = getMotionState();
	armJoint_grasp.graspingState = PlanInterface::GRASPING; 
	armJoint_init = graspMotionSeq[0];
	graspMotionSeq.clear();
	setGraspingState(PlanInterface::NOT_GRASPING);
	
	//===== Initial Point (already included in armJointSeq)
	graspMotionSeq.push_back( armJoint_init );

	//==== Approach Point
	arm()->IK_arm(Vector3(Rp_*Vector3(0,0,-0.1)+Pp_), Rp_);
	setGraspingState(PlanInterface::UNDER_GRASPING);	
	graspMotionSeq.push_back( getMotionState() );

	//==== Grasping Point
	setGraspingState(PlanInterface::GRASPING);	
	graspMotionSeq.push_back( armJoint_grasp );

	//==== LiftUp Point
	arm()->IK_arm(Vector3(Vector3(0,0,0.1)+Pp_), Rp_);
	armJoint_lift = getMotionState();
	graspMotionSeq.push_back( armJoint_lift );

	//=== Top of release point


	//== Release point


	//==Just for Graphics
	//setMotionState(armJoint_grasp);
	//object()->p = tc->objVisPos();
	//object()->R = tc->objVisRot();
	//setGraspingState(PlanInterface::NOT_GRASPING);

	return true;
*/
}

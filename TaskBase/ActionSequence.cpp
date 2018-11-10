#include "ActionSequence.h"

using namespace grasp;

ActionContent::ActionContent() {
	sub_obj_prehension_.type = ActionPrehension::NONE;
}

ActionContent::~ActionContent() {
}

double& ActionContent::time() {
	return time_;
}

const double& ActionContent::time() const {
	return time_;
}

ActionPose& ActionContent::ref_pose() {
	return ref_pose_;
}

const ActionPose& ActionContent::ref_pose() const {
	return ref_pose_;
}

ActionPrehension& ActionContent::subPrehension() {
	return sub_obj_prehension_;
}

const ActionPrehension& ActionContent::subPrehension() const {
	return sub_obj_prehension_;
}

cnoid::PoseSeq::iterator& ActionContent::refPoseIte() {
	return ref_pose_ite_;
}

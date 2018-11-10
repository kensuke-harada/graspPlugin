#include "ObjectPoseEstimationSolution.h"

#include "PlanBase.h"

using namespace grasp;

std::string ObjPoseEstimateSol::datafile_id = "";

ObjPoseEstimateSol::ObjPoseEstimateSol() :
	targetObject(NULL),
	env_point_cloud(NULL),
	is_target(false),
	is_feasible(true) {
}

ObjPoseEstimateSol::~ObjPoseEstimateSol() {
	if (targetObject != NULL) {
		delete targetObject;
	}
	targetObject = NULL;
}

ObjPoseEstimateSolHolder::ObjPoseEstimateSolHolder() {
}

ObjPoseEstimateSolHolder::~ObjPoseEstimateSolHolder() {
}

ObjPoseEstimateSolHolder* ObjPoseEstimateSolHolder::instance() {
	static ObjPoseEstimateSolHolder* instance = new ObjPoseEstimateSolHolder();
	return instance;
}

bool ObjPoseEstimateSolHolder::empty() const {
	return pose_estimation_sols_.empty();
}

int ObjPoseEstimateSolHolder::size() const {
	return pose_estimation_sols_.size();
}

void ObjPoseEstimateSolHolder::resize(int i) {
	pose_estimation_sols_.resize(i);
}

ObjPoseEstimateSol& ObjPoseEstimateSolHolder::at(int i) {
	return pose_estimation_sols_.at(i);
}

const ObjPoseEstimateSol& ObjPoseEstimateSolHolder::at(int i) const {
	return pose_estimation_sols_.at(i);
}

ObjPoseEstimateSol& ObjPoseEstimateSolHolder::back() {
	return pose_estimation_sols_.back();
}

const ObjPoseEstimateSol& ObjPoseEstimateSolHolder::back() const {
	return pose_estimation_sols_.back();
}

void ObjPoseEstimateSolHolder::push_back(ObjPoseEstimateSol& sol) {
	pose_estimation_sols_.push_back(sol);
}

void ObjPoseEstimateSolHolder::clear() {
	pose_estimation_sols_.clear();
}

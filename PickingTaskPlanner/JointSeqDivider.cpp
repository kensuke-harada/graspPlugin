/**
 * @file   JointSeqDivider.cpp
 * @author Akira Ohchi
 */

#include "JointSeqDivider.h"

#include "../Grasp/PlanBase.h"

namespace grasp {
	class JointSeqData {
	public:
		std::vector<cnoid::VectorXd> jointSeq;
		std::vector<int> graspingStateSeq;
		std::vector<int> graspingStateSeq2;
		std::vector<int> objectContactStateSeq;
		std::vector<std::vector<int> > pathPlanDOFSeq;
		std::vector<double> motionTimeSeq;
		std::vector<cnoid::Vector3> objectPalmPosSeq;
		std::vector<cnoid::Matrix3> objectPalmRotSeq;
	};

	class MotionSeqData {
	public:
		std::vector<grasp::MotionState> graspMotionSeq;
	};
}

using namespace grasp;

JointSeqDivider::JointSeqDivider() {
	is_divide_pose_.resize(10, false);
	original_data_ = NULL;
}

JointSeqDivider::~JointSeqDivider() {
	clear();
}

void JointSeqDivider::setDividingKeyPose(KeyPose keypose) {
	is_divide_pose_[static_cast<int>(keypose)] = true;
}

void JointSeqDivider::clearDividingKeyPose() {
	for (size_t i = 0; i < is_divide_pose_.size(); i++) {
		is_divide_pose_[i] = false;
	}
}

void JointSeqDivider::divide() {
	clear();

	grasp::PlanBase* pb = grasp::PlanBase::instance();
	original_data_ = new JointSeqData();
	original_data_->jointSeq = pb->jointSeq;
	original_data_->graspingStateSeq = pb->graspingStateSeq;
	original_data_->graspingStateSeq2 = pb->graspingStateSeq2;
	original_data_->objectContactStateSeq = pb->objectContactStateSeq;
	original_data_->pathPlanDOFSeq = pb->pathPlanDOFSeq;
	original_data_->motionTimeSeq = pb->motionTimeSeq;
	original_data_->objectPalmPosSeq = pb->objectPalmPosSeq;
	original_data_->objectPalmRotSeq = pb->objectPalmRotSeq;

	JointSeqData* target_data = new JointSeqData();
	for (size_t i = 0; i < original_data_->jointSeq.size(); i++) {
		target_data->jointSeq.push_back(pb->jointSeq[i]);
		target_data->graspingStateSeq.push_back(pb->graspingStateSeq[i]);
		target_data->graspingStateSeq2.push_back(pb->graspingStateSeq2[i]);
		if (pb->objectContactStateSeq.size() > 0) {
			target_data->objectContactStateSeq.push_back(pb->objectContactStateSeq[i]);
		}
		if (pb->pathPlanDOFSeq.size() > 0) {
			target_data->pathPlanDOFSeq.push_back(pb->pathPlanDOFSeq[i]);
		}
		target_data->motionTimeSeq.push_back(pb->motionTimeSeq[i]);
		target_data->objectPalmPosSeq.push_back(pb->objectPalmPosSeq[i]);
		target_data->objectPalmRotSeq.push_back(pb->objectPalmRotSeq[i]);

		if (is_divide_pose_[i]) {
			divided_data_.push_back(target_data);
			target_data = new JointSeqData();
			target_data->jointSeq.clear();
			target_data->graspingStateSeq.clear();
			target_data->graspingStateSeq2.clear();
			target_data->objectContactStateSeq.clear();
			target_data->pathPlanDOFSeq.clear();
			target_data->motionTimeSeq.clear();
			target_data->objectPalmPosSeq.clear();
			target_data->objectPalmRotSeq.clear();

			target_data->jointSeq.push_back(pb->jointSeq[i]);
			target_data->graspingStateSeq.push_back(pb->graspingStateSeq[i]);
			target_data->graspingStateSeq2.push_back(pb->graspingStateSeq2[i]);
			if (pb->objectContactStateSeq.size() > 0) {
				target_data->objectContactStateSeq.push_back(pb->objectContactStateSeq[i]);
			}
			if (pb->pathPlanDOFSeq.size() > 0) {
				target_data->pathPlanDOFSeq.push_back(pb->pathPlanDOFSeq[i]);
			}
			target_data->motionTimeSeq.push_back(pb->motionTimeSeq[i]);
			target_data->objectPalmPosSeq.push_back(pb->objectPalmPosSeq[i]);
			target_data->objectPalmRotSeq.push_back(pb->objectPalmRotSeq[i]);
		}
	}
	divided_data_.push_back(target_data);
	motion_data_.resize(divided_data_.size(), NULL);
}

int JointSeqDivider::size() const {
	return divided_data_.size();
}

void JointSeqDivider::setJointSeq(int num) {
	grasp::PlanBase* pb = grasp::PlanBase::instance();
	pb->jointSeq = divided_data_[num]->jointSeq;
	pb->graspingStateSeq = divided_data_[num]->graspingStateSeq;
	pb->graspingStateSeq2 = divided_data_[num]->graspingStateSeq2;
	pb->objectContactStateSeq = divided_data_[num]->objectContactStateSeq;
	pb->pathPlanDOFSeq = divided_data_[num]->pathPlanDOFSeq;
	pb->motionTimeSeq = divided_data_[num]->motionTimeSeq;
	pb->objectPalmPosSeq = divided_data_[num]->objectPalmPosSeq;
	pb->objectPalmRotSeq = divided_data_[num]->objectPalmRotSeq;
}

void JointSeqDivider::revertJointSeq() {
	if (!original_data_) return;
	grasp::PlanBase* pb = grasp::PlanBase::instance();
	pb->jointSeq = original_data_->jointSeq;
	pb->graspingStateSeq = original_data_->graspingStateSeq;
	pb->graspingStateSeq2 = original_data_->graspingStateSeq2;
	pb->objectContactStateSeq = original_data_->objectContactStateSeq;
	pb->pathPlanDOFSeq = original_data_->pathPlanDOFSeq;
	pb->motionTimeSeq = original_data_->motionTimeSeq;
	pb->objectPalmPosSeq = original_data_->objectPalmPosSeq;
	pb->objectPalmRotSeq = original_data_->objectPalmRotSeq;
}

void JointSeqDivider::registerMotionSeq(int num) {
	if (motion_data_[num] != NULL) delete motion_data_[num];
	MotionSeqData* motion = new MotionSeqData;
	motion->graspMotionSeq = grasp::PlanBase::instance()->graspMotionSeq;
	motion_data_[num] = motion;
}

void JointSeqDivider::setMotionSeq(int num) {
	if (motion_data_[num] == NULL) return;
	grasp::PlanBase::instance()->graspMotionSeq = motion_data_[num]->graspMotionSeq;
}


void JointSeqDivider::clear() {
	if (original_data_ != NULL) delete original_data_;
	for (size_t i = 0; i < divided_data_.size(); i++) {
		if (divided_data_[i] != NULL) delete divided_data_[i];
	}
	divided_data_.clear();
	for (size_t i = 0; i < motion_data_.size(); i++) {
		if (motion_data_[i] != NULL) delete motion_data_[i];
	}
	motion_data_.clear();
}

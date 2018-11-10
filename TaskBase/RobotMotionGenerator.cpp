#include "RobotMotionGenerator.h"

#include <../src/PoseSeqPlugin/PoseSeqItem.h>

#include "../Grasp/PlanBase.h"
#include "../Grasp/ObjectManager.h"
#include "../Grasp/AssemblyObject.h"
#include "../PRM/TrajectoryPlanner.h"

using namespace grasp;

RobotMotionGenerator::RobotMotionGenerator() :
	os(cnoid::MessageView::instance()->cout()) {
}

RobotMotionGenerator::~RobotMotionGenerator() {
}

bool RobotMotionGenerator::plan(const TaskItemPtr& task) const {
	bool ret = true;

	grasp::PlanBase* pb = grasp::PlanBase::instance();
	if (pb->targetArmFinger == NULL) {
		os << "Please select Grasping Robot" << std::endl;
		return false;
	}

	os << "Start planning: task " << task->name() << std::endl;

	// register target objects to PlanBase
	registerAssemblyObjects(task);
	// move target objects to initial position
	moveObjectsInitial(task);

	ActionGraspMotionConverter converter;

	pb->graspMotionSeq.clear();

	TaskItem::actionSeqConstIterator ite = task->cbegin();
	for (; ite != task->cend(); ++ite) {
		std::vector<MotionState> motion_seq;
		ret &= converter.convert((*ite).action, motion_seq);
		pb->graspMotionSeq.insert(pb->graspMotionSeq.end(), motion_seq.begin(), motion_seq.end());
	}

	if (!ret) {
		os << "Failed generating robot arm key poses" << std::endl;
		return false;
	}

	if (!doTrajectoryAndRenamePoseSeqs(task->name())) {
		os << "Failed trajectory planning" << std::endl;
		return false;
	}

	os << "Finish planning" << std::endl;

	return ret;
}

bool RobotMotionGenerator::plan(const ActionItemPtr& action) const {
	bool ret = true;

	grasp::PlanBase* pb = grasp::PlanBase::instance();
	if (pb->targetArmFinger == NULL) {
		os << "Please select Grasping Robot" << std::endl;
		return false;
	}

	os << "Start planning: action " << action->name() << std::endl;

	// register target objects to PlanBase
	registerAssemblyObjects(action);
	// move target objects to initial position
	moveObjectsInitial(action);

	pb->graspMotionSeq.clear();

	std::vector<MotionState> motion_seq;
	ActionGraspMotionConverter converter;
	ret = converter.convert(action, motion_seq);
	pb->graspMotionSeq.insert(pb->graspMotionSeq.end(), motion_seq.begin(), motion_seq.end());

	if (!ret) {
		os << "Failed generating robot arm key poses" << std::endl;
		return false;
	}

	if (!doTrajectoryAndRenamePoseSeqs(action->name())) {
		os << "Failed trajectory planning" << std::endl;
		return false;
	}

	os << "Finish planning" << std::endl;

	return ret;
}


void RobotMotionGenerator::registerAssemblyObjects(const TaskItemPtr& task) const {
	PlanBase* pb = PlanBase::instance();
	ObjectManager* om = pb->getObjectManager();

	// clear objects
	om->clearObjects();
	pb->robotBody()->clearConnections();

	// register objects
	TaskItem::actionSeqConstIterator ite = task->cbegin();
	for (; ite != task->cend(); ++ite) {
		const ActionItemPtr& action_item = (*ite).action;
		om->getOrCreateObject(action_item->getMainItem());
		om->getOrCreateObject(action_item->getSubItem());
	}
}

void RobotMotionGenerator::registerAssemblyObjects(const ActionItemPtr& action) const {
	PlanBase* pb = PlanBase::instance();
	ObjectManager* om = pb->getObjectManager();

	// clear objects
	om->clearObjects();
	pb->robotBody()->clearConnections();

	// register objects
	om->getOrCreateObject(action->getMainItem());
	om->getOrCreateObject(action->getSubItem());
}

void RobotMotionGenerator::moveObjectsInitial(const TaskItemPtr& task) const {
	std::map<cnoid::PoseSeqItem*, double> start_time;

	TaskItem::actionSeqConstIterator ite = task->cbegin();
	for (; ite != task->cend(); ++ite) {
		const ActionItemPtr& action_item = (*ite).action;
		cnoid::PoseSeqItemPtr& pose = action_item->absPoseSeqItem();

		bool is_first_pose = false;
		double beginningtime = pose->poseSeq()->beginningTime();
		if (start_time.count(pose.get()) < 1) {
			is_first_pose = true;
		} else if (start_time[pose.get()] > beginningtime) {
			is_first_pose = true;
		}

		if (is_first_pose) {
			start_time[pose.get()] = beginningtime;
			cnoid::PoseSeq::iterator seq_ite = pose->poseSeq()->begin();

			action_item->getSubItem()->body()->rootLink()->p() = (*seq_ite).get<cnoid::Pose>()->baseLinkInfo()->p;
			action_item->getSubItem()->body()->rootLink()->R() = (*seq_ite).get<cnoid::Pose>()->baseLinkInfo()->R;
		}
	}
}

void RobotMotionGenerator::moveObjectsInitial(const ActionItemPtr& action) const {
	cnoid::PoseSeqItemPtr& pose = action->absPoseSeqItem();

	cnoid::PoseSeq::iterator seq_ite = pose->poseSeq()->begin();
	action->getSubItem()->body()->rootLink()->p() = (*seq_ite).get<cnoid::Pose>()->baseLinkInfo()->p;
	action->getSubItem()->body()->rootLink()->R() = (*seq_ite).get<cnoid::Pose>()->baseLinkInfo()->R;
}

bool RobotMotionGenerator::doTrajectoryAndRenamePoseSeqs(const std::string& name) const {
	TrajectoryPlanner tp;
	bool ret = tp.doTrajectoryPlanning();

	// rename poseseq
	tp.poseSeqItemRobot->setName(name);
	const std::vector<cnoid::PoseSeqItemPtr>& pose_seq_items = tp.poseSeqItemAccompanyings();
	for (size_t i = 0; i < pose_seq_items.size(); i++) {
		pose_seq_items[i]->setName(name);
	}
	return ret;
}

ActionGraspMotionConverter::ActionGraspMotionConverter() :
	os(cnoid::MessageView::instance()->cout()) {
	app_vec_ = cnoid::Vector3(0.05, 0, 0);
}

ActionGraspMotionConverter::~ActionGraspMotionConverter() {
}

bool ActionGraspMotionConverter::convert(const ActionItemPtr& action, std::vector<MotionState>& motion_seq) const {
	bool ret = true;
	PlanBase* pb = PlanBase::instance();
	pb->setTrajectoryPlanDOF();

	// taget arm id
	int arm_id = pb->getArmID(pb->arm());
	// target object
	ObjectManager* om = pb->getObjectManager();
	AssemblyObjectPtr target_obj = om->getOrCreateObject(action->getSubItem());
	AssemblyObjectPtr base_obj = om->getOrCreateObject(action->getMainItem());

	cnoid::PoseSeqPtr obj_pose_seq = action->absPoseSeqItem()->poseSeq();
	const ActionSequence& seq = action->actionSeq();

	if (obj_pose_seq->size() != seq.size()) {
		os << "Error: the size of PoseSeq is not equal to the size of action sequence" << std::endl;
		return false;
	}
	cnoid::PoseSeq::iterator pseq_ite = obj_pose_seq->begin();
	ActionSequence::const_iterator aseq_ite = seq.begin();

	bool is_current_free = true;
	for (; pseq_ite != obj_pose_seq->end(); ++pseq_ite, ++aseq_ite) {
		if ((*aseq_ite).subPrehension().type != ActionPrehension::PREHENSION) {
			continue;
		}
		// object position
		const cnoid::Vector3& obj_p = (*pseq_ite).get<cnoid::Pose>()->baseLinkInfo()->p;
		const cnoid::Matrix3& obj_R = (*pseq_ite).get<cnoid::Pose>()->baseLinkInfo()->R;

		// handfram offset
		const cnoid::Vector3& handframe_p = (*aseq_ite).subPrehension().hand_frame.p;
		const cnoid::Matrix3& handframe_R = (*aseq_ite).subPrehension().hand_frame.R;

		// calc hand frame position
		cnoid::Vector3 des_p = obj_p + obj_R * handframe_p;
		cnoid::Matrix3 des_R = obj_R * handframe_R;

		bool is_grasp_motion = (is_current_free && ((*aseq_ite).subPrehension().type == ActionPrehension::PREHENSION));
		// // insert pre assemble pose
		// if (is_grasp_motion) {
		// 	is_current_free = false;
		// 	if (!addAppPose(des_p, des_R, motion_seq)) {
		// 		os << "[IK failed] approach action : " << action->name() << " time : " << (*pseq_ite).time() << std::endl;
		// 		ret = false;
		// 	}
		// }

		// solve IK
		if (!pb->targetArmFinger->arm->IK_arm(des_p, des_R)) {
			os << "[IK failed] action : " << action->name() << " time : " << (*pseq_ite).time() << std::endl;
			ret = false;
		}
		pb->calcForwardKinematics();

		// change gasping state
		if (is_grasp_motion) {
			is_current_free = false;
			pb->arm(arm_id)->target_grasp_objid = target_obj->getID();
			pb->setGraspingState(arm_id, PlanBase::GRASPING);
			target_obj->connect(base_obj.get(), base_obj->bodyItem()->body()->rootLink(), Connection::STATE_UNDERCONNECT);
			base_obj->connect(target_obj.get(), target_obj->bodyItem()->body()->rootLink(), Connection::STATE_UNDERCONNECT);
		}

		// check ungrasping pose
		cnoid::PoseSeq::iterator next_pite = pseq_ite;
		next_pite++;
		ActionSequence::const_iterator next_aite = aseq_ite;
		next_aite++;
		bool is_ungrasp_motion = ((!is_current_free) &&
															((next_pite == obj_pose_seq->end()) ||
															 ((*next_aite).subPrehension().type == ActionPrehension::FREE) ||
															 ((*next_aite).subPrehension().type == ActionPrehension::NONE)));

		// change grasping state to NOT_GRASPING
		if (is_ungrasp_motion) {
			is_current_free = true;
			pb->setGraspingState(arm_id, PlanBase::UNDER_GRASPING);
		}

		MotionState state = pb->getMotionState();
		motion_seq.push_back(state);
		// // insert post assemble pose
		// if (is_ungrasp_motion) {
		// 	if (!addAppPose(des_p, des_R, motion_seq)) {
		// 		os << "[IK failed] approach action : " << action->name() << " time : " << (*pseq_ite).time() << std::endl;
		// 		ret = false;
		// 	}
		// }
	}

	return ret;
}

void ActionGraspMotionConverter::setAppVec(const cnoid::Vector3& vec) {
	app_vec_ = vec;
}

bool ActionGraspMotionConverter::addAppPose(const cnoid::Vector3& grasp_p, const cnoid::Matrix3& grasp_R,
																						std::vector<MotionState>& motion_seq) const {
	PlanBase* pb = PlanBase::instance();
	cnoid::Vector3 app_p = grasp_p + grasp_R * app_vec_;
	bool ret = pb->targetArmFinger->arm->IK_arm(app_p, grasp_R);
	pb->calcForwardKinematics();
	MotionState state = pb->getMotionState();
	motion_seq.push_back(state);

	return ret;
}

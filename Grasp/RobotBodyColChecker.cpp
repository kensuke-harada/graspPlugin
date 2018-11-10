#include "RobotBodyColChecker.h"

#include <list>
#include <algorithm>
#include "PlanBase.h"
#include "RobotHand.h"
#include "MultiHandObjectColChecker.h"
#include "AssemblyObject.h"

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include "ColdetConverter.h"
#endif

using namespace std;
using namespace grasp;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
using namespace cnoid;
#endif

RobotBodyColChecker::RobotBodyColChecker() {
	PlanBase* pb = PlanBase::instance();

	// copy state
	objectContactState = pb->getObjectContactState();
	useRobotSafeBoundingBox = pb->useRobotSafeBoundingBox;
	doCheckCollisionPointCloudFinger = pb->doCheckCollisionPointCloudFinger;
	tolerance = pb->tolerance;

	// copy target object
	targetObject = NULL;
	if (pb->targetObject) {
		cnoid::BodyItemPtr object_copy = new cnoid::BodyItem(*pb->targetObject->bodyItemObject);
		targetObject = new TargetObject(object_copy);
		obj_body = targetObject->bodyItemObject->body();
		object = obj_body->link(pb->object()->name());
	} else {
		object = NULL;
	}
	bodyItemEnv = pb->bodyItemEnv;

	obj_manager_ = pb->getObjectManager()->clone();

	body_ = boost::dynamic_pointer_cast<RobotBody>(obj_manager_->getObject((*(obj_manager_->robot_begin()))->descriptor()));

	hand_obj_col_checker_ = new MultiHandObjectColChecker();
	hand_obj_col_checker_->setObjectManager(obj_manager_);
	hand_obj_col_checker_->setTargetObject(targetObject);
	
	copyTestPairs();
	copyInterObjectList();
	copyInterLinkList();
	copyTargetArmFinger();
}

RobotBodyColChecker::~RobotBodyColChecker() {
	delete hand_obj_col_checker_;
	delete obj_manager_;
	if (targetObject != NULL) delete targetObject;
}

bool RobotBodyColChecker::isColliding() {
	if (isCollidingTest(robotSelfPairs, "self collide")) {
		return true;
	}

	if (useRobotSafeBoundingBox) {
		if (isCollidingTest(safeRobotEnvPairs, "robot env collide")) {
			return true;
		}
	} else {
		if (isCollidingTest(robotEnvPairs, "robot env collide")) {
			return true;
		}
	}

	if (isCollidingTestPointCloud(robotExcludingFingPointCloudPairs, "robot pointcloud collide")) {
		return true;
	}

	if (doCheckCollisionPointCloudFinger) {
		if (isCollidingTestPointCloud(fingPointCloudPairs, "robot(finger) pointcloud collide")) {
			return true;
		}
	}

	if (object != NULL) {
		if (isCollidingTest(robotObjPairWithoutHand, "robot obj collide")) {
			return true;
		}

		for (size_t i = 0; i < handObjPair.size(); i++) {
			if (checkGraspingState(i) == PlanBase::NOT_GRASPING) {
				if (isCollidingTest(handObjPair[i], "robot obj collide")) {
					return true;
				}
			}
		}

		if (getObjectContactState() == PlanBase::OFF_ENVIRONMENT) {
			if (isCollidingTest(objEnvPairs, "obj env collide")) {
				return true;
			}

			if (isCollidingTestPointCloud(objPointCloudPairs, "obj pointcloud collide")) {
				return true;
			}
		}
	}

	for (size_t i = 0; i < interObjectList.size(); i++) {
		if (interObjectList[i].isColliding()) return true;
	}

	if (hand_obj_col_checker_->isColliding()) return true;

	return false;
}

double RobotBodyColChecker::clearance() {
	double min_sep = 1.e10;

	if (isCollidingTest(robotSelfPairs, "self collide")) {
		return 0;
	}

	if (isCollidingTest(robotObjPairWithoutHand, "robot obj collide")) {
		return 0;
	}

	for (size_t i = 0; i < handObjPair.size(); i++) {
		if (checkGraspingState(i) == PlanBase::NOT_GRASPING) {
			if (isCollidingTest(handObjPair[i], "robot obj collide")) {
				return 0;
			}
		}
	}

	if (isCollidingTestPointCloud(robotExcludingFingPointCloudPairs, "robot pointcloud collide")) {
		return 0;
	}

	if (doCheckCollisionPointCloudFinger) {
		if (isCollidingTestPointCloud(fingPointCloudPairs, "robot(finger) pointcloud collide")) {
			return 0;
		}
	}

	for (size_t i = 0; i < robotEnvPairs.size(); i++) {
		ColdetLinkPairPtr testPair = robotEnvPairs[i];
		testPair->updatePositions();

		testPair->setTolerance(tolerance);

		if (testPair->detectIntersection()) {
#ifdef DEBUG_MODE
			std::cout << "rob-env tolerance collide " << testPair->model(0)->name() << " " << testPair->model(1)->name() << std::endl;
#endif
			return 0;
		}
	}

	if (getObjectContactState() == PlanBase::OFF_ENVIRONMENT) {
		for (size_t i = 0; i < objEnvPairs.size(); i++) {
			ColdetLinkPairPtr testPair = objEnvPairs[i];
			testPair->updatePositions();

			testPair->setTolerance(tolerance);
			if (testPair->detectIntersection()) {
#ifdef DEBUG_MODE
				std::cout << "obj-env tolerance collide " << testPair->model(0)->name() << " " << testPair->model(1)->name() << std::endl;
#endif
				return 0;
			}
		}

		if (isCollidingTestPointCloud(objPointCloudPairs, "obj pointcloud collide")) {
				return 0;
		}
	}

	double distance = hand_obj_col_checker_->clearance(tolerance);
	if (distance < min_sep) {
		min_sep = distance;
	}

	return min_sep;
}

void RobotBodyColChecker::calcForwardKinematics() {
	setInterLink();

	body_->calcForwardKinematics();

	body_->calcFK();

	ArmFingers* armfing0 = body_->getArmFingers(0);
	
	if (armfing0->arm->graspingState == PlanBase::GRASPING &&
			armfing0->arm->target_grasp_objid < 0) {
		if (armfing0->nFing > 0) {
			object->R() = armfing0->fingers[0]->tip->R()*(armfing0->objectPalmRot);
			object->p() = armfing0->fingers[0]->tip->p()+armfing0->fingers[0]->tip->R()*armfing0->objectPalmPos;
		} else {
			object->R() = armfing0->palm->R()*(armfing0->objectPalmRot);
			object->p() = armfing0->palm->p()+armfing0->palm->R()*armfing0->objectPalmPos;
		}
	}	else if (body_->armSize() > 1 && body_->getArmFingers(1)->arm->graspingState == PlanBase::GRASPING &&
						 body_->getArmFingers(1)->arm->target_grasp_objid < 0) {
			if(body_->getArmFingers(1)->nFing>0) {
			object->R() = body_->getArmFingers(1)->fingers[0]->tip->R()*(body_->getArmFingers(1)->objectPalmRot);
			object->p() = body_->getArmFingers(1)->fingers[0]->tip->p()+body_->getArmFingers(1)->fingers[0]->tip->R()*body_->getArmFingers(1)->objectPalmPos;
		} else {
			object->R() = body_->getArmFingers(1)->palm->R()*(body_->getArmFingers(1)->objectPalmRot);
			object->p() = body_->getArmFingers(1)->palm->p()+body_->getArmFingers(1)->palm->R()*body_->getArmFingers(1)->objectPalmPos;
		}
	}
	for (size_t i = 0; i < interObjectList.size(); i++) {
		interObjectList[i].setInterObject();
	}
}

void RobotBodyColChecker::setInterLink() {
	if (interLinkList.empty()) return;
	for (size_t i = 0; i < interLinkList.size(); i++) {
		interLinkList[i].slave->q() = interLinkList[i].master->q() *interLinkList[i].ratio;
		if( interLinkList[i].slave->q() < interLinkList[i].slave->q_lower()) interLinkList[i].slave->q() = interLinkList[i].slave->q_lower();
		if( interLinkList[i].slave->q() > interLinkList[i].slave->q_upper()) interLinkList[i].slave->q() = interLinkList[i].slave->q_upper();
	}
	for (int arm_id = 0; arm_id < body_->armSize(); arm_id++) {
		for (int hand_id = 0; hand_id < body_->handListSize(arm_id); hand_id++) {
			RobotHand* hand = body_->getRobotArmFingers(arm_id)->getRobotHand(hand_id);
			for (size_t i = 0; i < hand->interLinkList.size(); i++) {
				hand->interLinkList[i].slave->q() = hand->interLinkList[i].master->q() * hand->interLinkList[i].ratio;
				if( hand->interLinkList[i].slave->q() < hand->interLinkList[i].slave->q_lower()) hand->interLinkList[i].slave->q() = hand->interLinkList[i].slave->q_lower();
				if( hand->interLinkList[i].slave->q() > hand->interLinkList[i].slave->q_upper()) hand->interLinkList[i].slave->q() = hand->interLinkList[i].slave->q_upper();
			}
		}
	}
}

void RobotBodyColChecker::setGraspingState(int state) {
	setGraspingState(0, state);
}

void RobotBodyColChecker::setGraspingState2(int state) {
	if (body_->armSize() < 2) return;
	setGraspingState(1, state);
}

void RobotBodyColChecker::setGraspingState(int i, int state) {
	if (object == NULL) return;
	ArmFingers* arm_fing = body_->getArmFingers(i);
	if (state == PlanBase::GRASPING) {
		if (arm_fing->arm->target_grasp_objid < 0) { // grasping target object
			if (arm_fing->nFing > 0) {
				arm_fing->objectPalmPos = (cnoid::Matrix3(arm_fing->fingers[0]->tip->R())).transpose() * (object->p() - arm_fing->fingers[0]->tip->p());
				arm_fing->objectPalmRot = (cnoid::Matrix3(arm_fing->fingers[0]->tip->R())).transpose() * object->R();
			} else {
				arm_fing->objectPalmPos = (cnoid::Matrix3(arm_fing->palm->R())).transpose() * (object->p() - arm_fing->palm->p());
				arm_fing->objectPalmRot = (cnoid::Matrix3(arm_fing->palm->R())).transpose() * object->R();
			}
		}
	}
	body_->getRobotArmFingers(i)->setGraspingState(state);
	arm_fing->arm->graspingState = state;
}

int RobotBodyColChecker::checkGraspingState(int arm_id) const {
	return body_->getArmFingers(arm_id)->arm->graspingState;
}

void RobotBodyColChecker::SetEnvironment(cnoid::BodyItemPtr& bodyitem) {
	bodyItemEnv.push_back(bodyitem);
	bodyItemEnv.sort();
	bodyItemEnv.unique();
	copyTestPairs();
}

void RobotBodyColChecker::RemoveEnvironment(cnoid::BodyItemPtr& bodyitem) {
	bodyItemEnv.remove(bodyitem);
	bodyItemEnv.sort();
	bodyItemEnv.unique();
	copyTestPairs();
}

bool RobotBodyColChecker::isCollidingTest(const ColdetLinkPairVector& target_pairs, const std::string& debug_msg) {
	for (size_t i = 0; i < target_pairs.size(); i++) {
		ColdetLinkPairPtr testPair = target_pairs[i];
		testPair->updatePositions();
		bool coll = testPair->checkCollision();
		if (coll) {
			colPairName[0] = testPair->model(0)->name();
			colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
			std::cout << debug_msg << " " << testPair->model(0)->name() << " " << testPair->model(1)->name() << std::endl;
#endif
			return true;
		}
	}
	return false;
}

bool RobotBodyColChecker::isCollidingTestPointCloud(const std::vector<std::pair<cnoid::Link*, grasp::PointCloudEnv*> >& target_pairs, const std::string& debug_msg) {
	for (size_t i = 0; i < target_pairs.size(); i++) {
		bool coll = grasp::PlanBase::instance()->isCollidingPointCloudSub(target_pairs[i].second, target_pairs[i].first);
		if (coll) {
#ifdef DEBUG_MODE
			std::cout << debug_msg << std::endl;
#endif
			return true;
		}
	}
	return false;
 }

void RobotBodyColChecker::copyTestPairs() {
	PlanBase* pb = PlanBase::instance();

	robotSelfPairs.clear();
	robotEnvPairs.clear();
	robotObjPairs.clear();
	robotObjPairWithoutHand.clear();
	handObjPair.clear();
	objEnvPairs.clear();
	safeRobotEnvPairs.clear();
	safeRobotObjPairs.clear();
	robotExcludingFingPointCloudPairs.clear();
	fingPointCloudPairs.clear();
	objPointCloudPairs.clear();

	cnoid::BodyPtr body = body_->getRootBodyItem()->body();
	for (size_t i = 0; i < pb->robotSelfPairs.size(); i++) {
		robotSelfPairs.push_back(boost::make_shared<ColdetLinkPair>(body, body->link(pb->robotSelfPairs[i]->link(0)->name()),
																																			 body, body->link(pb->robotSelfPairs[i]->link(1)->name())));
	}

	std::list<cnoid::Link*> target_links;
	for (size_t i = 0; i < pb->robotEnvPairs.size(); i++) {
		target_links.push_back(pb->robotEnvPairs[i]->link(0));
	}
	target_links.sort();
	target_links.unique();
	for (std::list<cnoid::BodyItemPtr>::const_iterator env_ite = bodyItemEnv.begin(); env_ite != bodyItemEnv.end(); ++env_ite) {
		for (size_t i = 0; i < (*env_ite)->body()->numLinks(); i++) {
			for (std::list<cnoid::Link*>::const_iterator link_ite = target_links.begin(); link_ite != target_links.end(); ++link_ite) {
				robotEnvPairs.push_back(boost::make_shared<ColdetLinkPair>(body, body->link((*link_ite)->name()),
																																					(*env_ite)->body(), (*env_ite)->body()->link(i)));
			}
		}
	}

	for (size_t i = 0; i < pb->robotObjPairs.size(); i++) {
		robotObjPairs.push_back(boost::make_shared<ColdetLinkPair>(body, body->link(pb->robotObjPairs[i]->link(0)->name()),
																																			obj_body, obj_body->link(pb->robotObjPairs[i]->link(1)->name())));
	}

	for (size_t i = 0; i < pb->robotObjPairWithoutHand.size(); i++) {
		robotObjPairWithoutHand.push_back(boost::make_shared<ColdetLinkPair>(body, body->link(pb->robotObjPairWithoutHand[i]->link(0)->name()),
																																								obj_body, obj_body->link(pb->robotObjPairWithoutHand[i]->link(1)->name())));
	}

	for (size_t j = 0; j < pb->handObjPair.size(); j++) {
		handObjPair.push_back(std::vector<ColdetLinkPairPtr>());
		for (size_t i = 0; i < pb->handObjPair[j].size(); i++) {
			handObjPair[j].push_back(boost::make_shared<ColdetLinkPair>(body, body->link(pb->handObjPair[j][i]->link(0)->name()),
																																				 obj_body, obj_body->link(pb->handObjPair[j][i]->link(1)->name())));
		}
	}

	target_links.clear();
	for (size_t i = 0; i < pb->objEnvPairs.size(); i++) {
		target_links.push_back(pb->objEnvPairs[i]->link(0));
	}
	target_links.sort();
	target_links.unique();
	for (std::list<cnoid::BodyItemPtr>::const_iterator env_ite = bodyItemEnv.begin(); env_ite != bodyItemEnv.end(); ++env_ite) {
		for (int i = 0; i < (*env_ite)->body()->numLinks(); i++) {
			for (std::list<cnoid::Link*>::const_iterator link_ite = target_links.begin(); link_ite != target_links.end(); ++link_ite) {
				objEnvPairs.push_back(boost::make_shared<ColdetLinkPair>(obj_body, obj_body->link((*link_ite)->name()),
																																				(*env_ite)->body(), (*env_ite)->body()->link(i)));
			}
		}
	}

	std::map<cnoid::Link*, cnoid::ColdetModelPtr> link_coldet_map;
	for (size_t i = 0; i < pb->safeRobotEnvPairs.size(); i++) {
		link_coldet_map[pb->safeRobotEnvPairs[i]->link(0)] = pb->safeRobotEnvPairs[i]->model(0);
	}
	for (std::list<cnoid::BodyItemPtr>::const_iterator env_ite = bodyItemEnv.begin(); env_ite != bodyItemEnv.end(); ++env_ite) {
		for (int i = 0; i < (*env_ite)->body()->numLinks(); i++) {
			for (std::map<cnoid::Link*, cnoid::ColdetModelPtr>::const_iterator link_ite = link_coldet_map.begin(); link_ite != link_coldet_map.end(); ++link_ite) {
				cnoid::Link* target_link = (*link_ite).first;
				cnoid::ColdetModelPtr target_model = (*link_ite).second;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
				cnoid::ColdetModelPtr backup = body->link(target_link->name())->coldetModel();
				body->link(target_link->name())->setColdetModel(target_model->clone());
#else
				cnoid::SgNodePtr backup = body->link(target_link->name())->collisionShape();
				body->link(target_link->name())->setCollisionShape(
				ColdetConverter::ConvertTo(target_model->clone()));
#endif
				safeRobotEnvPairs.push_back(boost::make_shared<ColdetLinkPair>(body, body->link(target_link->name()),
																																							(*env_ite)->body(), (*env_ite)->body()->link(i)));
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
				body->link(target_link->name())->setColdetModel(backup);
#else
				body->link(target_link->name())->setCollisionShape(backup);
#endif
			}
		}
	}

	for (size_t i = 0; i < pb->safeRobotObjPairs.size(); i++) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		cnoid::ColdetModelPtr backup = body->link(pb->safeRobotObjPairs[i]->link(0)->name())->coldetModel();
		body->link(pb->safeRobotObjPairs[i]->link(0)->name())->setColdetModel(pb->safeRobotObjPairs[i]->model(0)->clone());
#else
		cnoid::SgNodePtr backup = body->link(pb->safeRobotObjPairs[i]->link(0)->name())->collisionShape();
		body->link(pb->safeRobotObjPairs[i]->link(0)->name())->setCollisionShape(
		ColdetConverter::ConvertTo(pb->safeRobotObjPairs[i]->model(0)->clone()));
#endif
		safeRobotObjPairs.push_back(boost::make_shared<ColdetLinkPair>(body, body->link(pb->safeRobotObjPairs[i]->link(0)->name()),
																																	 obj_body, obj_body->link(pb->safeRobotObjPairs[i]->link(1)->name())));
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		body->link(pb->safeRobotObjPairs[i]->link(0)->name())->setColdetModel(backup);
#else
		body->link(pb->safeRobotObjPairs[i]->link(0)->name())->setCollisionShape(backup);
#endif
	}

	for (size_t i = 0; i < pb->robotExcludingFingPointCloudPairs.size(); i++) {
		robotExcludingFingPointCloudPairs.push_back(make_pair(body->link(pb->robotExcludingFingPointCloudPairs[i].first->name()), pb->robotExcludingFingPointCloudPairs[i].second));
	}

	for (size_t i = 0; i < pb->fingPointCloudPairs.size(); i++) {
		fingPointCloudPairs.push_back(make_pair(body->link(pb->fingPointCloudPairs[i].first->name()), pb->fingPointCloudPairs[i].second));
	}

	for (size_t i = 0; i < pb->objPointCloudPairs.size(); i++) {
		objPointCloudPairs.push_back(make_pair(obj_body->link(pb->objPointCloudPairs[i].first->name()), pb->objPointCloudPairs[i].second));
	}

	hand_obj_col_checker_->initialCollision(bodyItemEnv, pb->pointCloudEnv);
}

void RobotBodyColChecker::copyInterObjectList() {
	PlanBase* pb = PlanBase::instance();
	cnoid::BodyPtr body = body_->getRootBodyItem()->body();
	interObjectList.clear();
	interObjectList.resize(pb->interObjectList.size());
	for (size_t i = 0; i < pb->interObjectList.size(); i++) {
		interObjectList[i].slaveItem = new cnoid::BodyItem(*(pb->interObjectList[i].slaveItem));
		interObjectList[i].master = body->link(pb->interObjectList[i].master->name());
		interObjectList[i].relativePos = pb->interObjectList[i].relativePos;
		interObjectList[i].relativeRot = pb->interObjectList[i].relativeRot;
		for (size_t j = 0; j < pb->interObjectList[i].slaveEnvPairs.size(); i++) {
			interObjectList[i].slaveEnvPairs.push_back(boost::make_shared<ColdetLinkPair>(interObjectList[i].slaveItem->body(), body->link(interObjectList[i].slaveEnvPairs[j]->link(0)->name()),
																																													 pb->interObjectList[i].slaveEnvPairs[j]->body(1), pb->interObjectList[i].slaveEnvPairs[j]->link(1)));
		}
	}
}

void RobotBodyColChecker::copyInterLinkList() {
	PlanBase* pb = PlanBase::instance();
	interLinkList.clear();
	interLinkList.resize(pb->interLinkList.size());
	cnoid::BodyPtr root_body = body_->getRootBodyItem()->body();
	for (size_t i = 0; i < pb->interLinkList.size(); i++) {
		interLinkList[i] = pb->interLinkList[i];
		interLinkList[i].slave = root_body->link(pb->interLinkList[i].slave->name());
		interLinkList[i].master = root_body->link(pb->interLinkList[i].master->name());
	}
}

void RobotBodyColChecker::copyTargetArmFinger() {
	PlanBase* pb = PlanBase::instance();
	for (size_t i = 0; i < pb->armsList.size(); i++) {
		if (pb->armsList[i] == pb->targetArmFinger) {
			targetArmFinger = body_->getArmFingers(i);
		}
	}
}

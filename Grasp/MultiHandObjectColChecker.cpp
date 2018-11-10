#include "MultiHandObjectColChecker.h"

#include "ColdetPairData.h"
#include "RobotBody.h"
#include "AssemblyObject.h"
#include "PointCloudCollisionChecker.h"

using namespace grasp;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
using namespace cnoid;
#endif

MultiHandObjectColChecker::MultiHandObjectColChecker() :
	obj_manager_(NULL),
	object_(NULL) {
}

MultiHandObjectColChecker::~MultiHandObjectColChecker() {
	clearAll();
}

void MultiHandObjectColChecker::setObjectManager(ObjectManager* obj_manager) {
	obj_manager_ = obj_manager;
}

void MultiHandObjectColChecker::setTargetObject(TargetObject* object) {
	object_ = object;
}

void MultiHandObjectColChecker::initialCollisionSelf() {
	self_checkers_.clear();
	if (obj_manager_ == NULL) return;
	for (ObjectManager::iterator it = obj_manager_->begin(); it != obj_manager_->end(); ++it) {
		initialCollisionSelf((*it));
	}
}

void MultiHandObjectColChecker::initialCollisionSelf(ObjectBase* object) {
	object->initialCollisionSelf();
	ColCheckPair* target_pair = findColCheckPair(object->bodyItem(), object->bodyItem());
	if (target_pair == NULL) {
		target_pair = new SelfColCheckPair(object);
		checkers_cache_.push_back(target_pair);
	}
	self_checkers_.push_back(target_pair);
}

void MultiHandObjectColChecker::initialCollision(std::list<cnoid::BodyItemPtr>& env_body_items,
																								 std::list<PointCloudEnv*>& env_pointcloud) {
	checkers_.clear();
	point_cloud_checkers_.clear();
	
	if (obj_manager_ == NULL) return;
	for (ObjectManager::iterator it = obj_manager_->begin(); it != obj_manager_->end(); ++it) {
		// collision check pair between two objects
		for (ObjectManager::iterator it2 = it + 1; it2 != obj_manager_->end(); ++it2) {
			ColCheckPair* target_pair = findColCheckPair((*it)->bodyItem(), (*it2)->bodyItem());
			if (target_pair == NULL) {
				target_pair = createColCheckPair((*it), (*it2));
				checkers_cache_.push_back(target_pair);
			}
			checkers_.push_back(target_pair);
		}

		// collision check pair between object and env
		for (std::list<cnoid::BodyItemPtr>::iterator eit = env_body_items.begin();
				 eit != env_body_items.end(); ++eit) {
			ColCheckPair* target_pair = findColCheckPair((*it)->bodyItem(), (*eit));
			if (target_pair == NULL) {
				target_pair = createColCheckPairEnv((*it), (*eit));
				checkers_cache_.push_back(target_pair);
			}
			checkers_.push_back(target_pair);
		}

		// collision check pair between object and point cloud
		for (std::list<PointCloudEnv*>::iterator pit = env_pointcloud.begin();
				 pit != env_pointcloud.end(); ++pit) {
			point_cloud_checkers_.push_back(createColCheckPairPointCloud((*it), (*pit)));
		}
	}

	if (object_ != NULL) {
		for (ObjectManager::hand_iterator it = obj_manager_->hand_begin();
				 it != obj_manager_->hand_end(); ++it) {
			ColCheckPair* target_pair = findColCheckPair(object_->bodyItemObject, (*it)->bodyItem());
			if (target_pair == NULL) {
				target_pair = new HandTargetObjColCheckPair(object_, (*it));
				checkers_cache_.push_back(target_pair);
			}
			checkers_.push_back(target_pair);
		}
	}
}

void MultiHandObjectColChecker::initialCollisionAlwaysRemake(std::list<cnoid::BodyItemPtr>& env_body_items,
																														 std::list<PointCloudEnv*>& env_pointcloud) {
	self_checkers_.clear();
	checkers_.clear();
	point_cloud_checkers_.clear();

	if (obj_manager_ == NULL) return;
	for (ObjectManager::iterator it = obj_manager_->begin(); it != obj_manager_->end(); ++it) {
		initialCollisionSelf((*it));
	}

	for (ObjectManager::iterator it = obj_manager_->begin(); it != obj_manager_->end(); ++it) {
		// collision check pair between two objects
		for (ObjectManager::iterator it2 = it + 1; it2 != obj_manager_->end(); ++it2) {
			checkers_.push_back(createColCheckPair((*it), (*it2)));
		}

		// collision check pair between object and env
		for (std::list<cnoid::BodyItemPtr>::iterator eit = env_body_items.begin();
				 eit != env_body_items.end(); ++eit) {
			checkers_.push_back(createColCheckPairEnv((*it), (*eit)));
		}

		// collision check pair between object and point cloud
		for (std::list<PointCloudEnv*>::iterator pit = env_pointcloud.begin();
				 pit != env_pointcloud.end(); ++pit) {
			point_cloud_checkers_.push_back(createColCheckPairPointCloud((*it), (*pit)));
		}
	}

	if (object_ != NULL) {
		for (ObjectManager::hand_iterator it = obj_manager_->hand_begin();
				 it != obj_manager_->hand_end(); ++it) {
			checkers_.push_back(new HandTargetObjColCheckPair(object_, (*it)));
		}
	}
}

bool MultiHandObjectColChecker::isColliding() const {
	for (size_t i = 0; i < self_checkers_.size(); i++) {
		if (self_checkers_[i]->isColliding()) return true;
	}
	for (size_t i = 0; i < checkers_.size(); i++) {
		if (checkers_[i]->isColliding()) return true;
	}
	for (size_t i = 0; i < point_cloud_checkers_.size(); i++) {
		if (point_cloud_checkers_[i]->isColliding()) return true;
	}
	return false;
}

double MultiHandObjectColChecker::clearance(double tolerance) const {
	double min_sep = 1.e10;
	for (size_t i = 0; i < self_checkers_.size(); i++) {
		double tmp  = self_checkers_[i]->clearance(tolerance);
		if (tmp < min_sep) {
			min_sep = tmp;
		}
		if (min_sep == 0) return 0;
	}	
	for (size_t i = 0; i < checkers_.size(); i++) {
		double tmp  = checkers_[i]->clearance(tolerance);
		if (tmp < min_sep) {
			min_sep = tmp;
		}
		if (min_sep == 0) return 0;
	}
	for (size_t i = 0; i < point_cloud_checkers_.size(); i++) {
		double tmp  = point_cloud_checkers_[i]->clearance(tolerance);
		if (tmp < min_sep) {
			min_sep = tmp;
		}
		if (min_sep == 0) return 0;
	}
	return min_sep;
}

void MultiHandObjectColChecker::clearAll() {
	for (size_t i = 0; i < checkers_cache_.size(); i++) {
		if (checkers_cache_[i] != NULL) delete checkers_cache_[i];
		checkers_cache_[i] = NULL;
	}
	checkers_.clear();
	self_checkers_.clear();
	point_cloud_checkers_.clear();
	checkers_cache_.clear();
}

ColCheckPair* MultiHandObjectColChecker::findColCheckPair(const cnoid::BodyItemPtr& bodyitem1, const cnoid::BodyItemPtr& bodyitem2) const {
	for (size_t i = 0; i < checkers_cache_.size(); i++) {
		if (checkers_cache_[i]->isSame(bodyitem1, bodyitem2)) return checkers_cache_[i];
	}
	return NULL;
}

ColCheckPair* MultiHandObjectColChecker::createColCheckPairEnv(ObjectBase* obj, cnoid::BodyItemPtr& env) const {
	if (obj->isRobot()) {
		return new RobotEnvColCheckPair(obj, env);
	} else if (obj->isHand()) {
		return new HandEnvColCheckPair(obj, env);
	} else if (obj->isObject()) {
		return new ObjEnvColCheckPair(obj, env);
	}
	return NULL;
}

ColCheckPair* MultiHandObjectColChecker::createColCheckPair(ObjectBase* obj1, ObjectBase* obj2) const {
	if (obj1->isRobot()) {
		if (obj2->isRobot()) {
			return NULL;
		} else if (obj2->isHand()) {
			return new HandRobotColCheckPair(obj2, obj1);
		} else if (obj2->isObject()) {
			return new ObjRobotColCheckPair(obj2, obj1);
		}
	} else if (obj1->isHand()) {
		if (obj2->isRobot()) {
			return new HandRobotColCheckPair(obj1, obj2);
		} else if (obj2->isHand()) {
			return new HandHandColCheckPair(obj1, obj2);
		} else if (obj2->isObject()) {
			return new HandObjColCheckPair(obj2, obj1);
		}
	} else if (obj1->isObject()) {
		if (obj2->isRobot()) {
			return new ObjRobotColCheckPair(obj1, obj2);
		} else if (obj2->isHand()) {
			return new HandObjColCheckPair(obj1, obj2);
		} else if (obj2->isObject()) {
			return new ObjObjColCheckPair(obj1, obj2);
		}
	}
	return NULL;
}

ColCheckPairPointCloud* MultiHandObjectColChecker::createColCheckPairPointCloud(ObjectBase* obj, PointCloudEnv* pointcloud) const {
	if (obj->isRobot()) {
		return new RobotPointCloudColCheckPair(obj, pointcloud);
	} else if (obj->isHand()) {
		return new HandPointCloudColCheckPair(obj, pointcloud);
	} else if (obj->isObject()) {
		return new ObjPointCloudColCheckPair(obj, pointcloud);
	}
	return NULL;
}

ColCheckPair::ColCheckPair() :
	min_sep_(1.e10),
	is_env_pair_(false) {
}

ColCheckPair::~ColCheckPair() {
}

bool ColCheckPair::isSame(const cnoid::BodyItemPtr& bodyitem1, const cnoid::BodyItemPtr& bodyitem2) const {
	return ((bodyitem1 == bodyitem1_ && bodyitem2 == bodyitem2_) || (bodyitem1 == bodyitem2_ && bodyitem2 == bodyitem1_));
}

bool ColCheckPair::isColliding(std::ostream& out) const {
	for (size_t i = 0; i < col_link_pairs_vec_.size(); i++) {
		if (!col_test_check_func_vec_[i]()) continue;
		for (size_t j = 0; j < col_link_pairs_vec_[i].size(); j++) {
			const ColdetLinkPairPtr& testPair = col_link_pairs_vec_[i][j];
			testPair->updatePositions();
			bool coll = testPair->checkCollision();
			if (coll) {
#ifdef DEBUG_MODE
				out << debug_messages_[i] << " " << testPair->model(0)->name() << " " << testPair->model(1)->name() << std::endl;
#endif
				return true;
			}
		}
	}
	return false;
}

double ColCheckPair::clearance(double tolerance) const {
	for (size_t i = 0; i < col_link_pairs_vec_.size(); i++) {
		if (!col_test_check_func_vec_[i]()) continue;
		for (size_t j = 0; j < col_link_pairs_vec_[i].size(); j++) {
			const ColdetLinkPairPtr& testPair = col_link_pairs_vec_[i][j];
			testPair->updatePositions();
			bool coll;
			if (is_env_pair_) {
				testPair->setTolerance(tolerance);
				coll = testPair->detectIntersection();
			} else {
				coll = testPair->checkCollision();
			}
			if (coll) {
#ifdef DEBUG_MODE
				out << debug_messages_[i] << " " << testPair->model(0)->name() << " " << testPair->model(1)->name() << std::endl;
#endif
				return 0;
			}
		}
	}
	return min_sep_;
}

void ColCheckPair::initialCollision(const std::string& default_debug_msg) {
	addColdetLinkPairs(bodyitem1_, bodyitem2_);
	col_test_check_func_vec_.push_back(boost::bind(&ColCheckPair::alwaysCheck, this));
	debug_messages_.push_back(default_debug_msg);
}

void ColCheckPair::addColdetLinkPairs(const cnoid::BodyItemPtr& bodyitem1, const cnoid::BodyItemPtr& bodyitem2) {
	ColdetPairData coldata(bodyitem1, bodyitem2);
	col_link_pairs_vec_.push_back(ColdetLinkPairVector());
	col_link_pairs_vec_.back().insert(col_link_pairs_vec_.back().end(),
																		coldata.coldetLinkPairs.begin(),
																		coldata.coldetLinkPairs.end());
}

void ColCheckPair::addColdetLinkPairs(const cnoid::BodyItemPtr& bodyitem1, const std::vector<cnoid::Link*>& target_links,
																			const cnoid::BodyItemPtr& bodyitem2) {
	ColdetPairData coldata(bodyitem1, target_links, bodyitem2);
	col_link_pairs_vec_.push_back(ColdetLinkPairVector());
	col_link_pairs_vec_.back().insert(col_link_pairs_vec_.back().end(),
																		coldata.coldetLinkPairs.begin(),
																		coldata.coldetLinkPairs.end());
}

void ColCheckPair::addColdetLinkPairsSafe(const ObjectBase* obj, const cnoid::BodyItemPtr& env) {
	col_link_pairs_vec_.push_back(ColdetLinkPairVector());
	const cnoid::BodyPtr& obj_body = obj->bodyItem()->body();
	const cnoid::BodyPtr& env_body = env->body();
	for (int i = 0; i < obj_body->numLinks(); i++) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		SafeBoundingBox backup = obj_body->link(i)->coldetModel();
		SafeBoundingBox temp = obj->getSafeBoundingBox(i);
		if (temp != NULL) obj_body->link(i)->setColdetModel(temp);
#else
		cnoid::SgNode* backup = obj_body->link(i)->collisionShape();
		SafeBoundingBox temp = obj->getSafeBoundingBox(i);
		if (temp != NULL) obj_body->link(i)->setCollisionShape(temp);		
#endif
		for (int j = 0; j < env_body->numLinks(); j++) {
#ifdef  CNOID_10_11_12_13
			cnoid::ColdetLinkPairPtr temp= new cnoid::ColdetLinkPair(obj_body->link(i), env_body->link(j));
#elif defined(CNOID_14)
			cnoid::ColdetLinkPairPtr temp = boost::make_shared<cnoid::ColdetLinkPair>(obj_body, obj_body->link(i), env_body, env_body->link(j) );
#else
			grasp::ColdetLinkPairPtr temp = boost::make_shared<grasp::ColdetLinkPair>(obj_body, obj_body->link(i), env_body, env_body->link(j) );
#endif
			col_link_pairs_vec_.back().push_back(temp);
		}
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		obj_body->link(i)->setColdetModel(backup);
#else
		obj_body->link(i)->setCollisionShape(backup);
#endif
	}
}

SelfColCheckPair::SelfColCheckPair(ObjectBase* obj) {
	bodyitem1_ = obj->bodyItem();
	bodyitem2_ = obj->bodyItem();
	obj_ = obj;
}

SelfColCheckPair::~SelfColCheckPair() {
}

bool SelfColCheckPair::isColliding(std::ostream& out) const {
	std::string dummy;
	return obj_->isColliding(dummy, dummy);
}

RobotEnvColCheckPair::RobotEnvColCheckPair(ObjectBase* robot, cnoid::BodyItemPtr& env) {
	robot_ = static_cast<RobotBody*>(robot);
	bodyitem1_ = robot->bodyItem();
	bodyitem2_ = env;
	initialCollision("robot env collide");
	is_env_pair_ = true;
}

RobotEnvColCheckPair::~RobotEnvColCheckPair() {
}

void RobotEnvColCheckPair::initialCollision(const std::string& default_debug_msg) {
	addColdetLinkPairs(bodyitem1_, bodyitem2_);
	col_test_check_func_vec_.push_back(boost::bind(&RobotEnvColCheckPair::isNotUseSafeBoundingBox, this));
	debug_messages_.push_back(default_debug_msg);
	// safe bounding box
	addColdetLinkPairsSafe(robot_, bodyitem2_);
	col_test_check_func_vec_.push_back(boost::bind(&RobotEnvColCheckPair::isUseSafeBoundingBox, this));
	debug_messages_.push_back(default_debug_msg);
}

bool RobotEnvColCheckPair::isUseSafeBoundingBox() const {
	return PlanBase::instance()->useRobotSafeBoundingBox;
}

bool RobotEnvColCheckPair::isNotUseSafeBoundingBox() const {
	return !(PlanBase::instance()->useRobotSafeBoundingBox);
}


ObjEnvColCheckPair::ObjEnvColCheckPair(ObjectBase* obj, cnoid::BodyItemPtr& env) {
	object_ = static_cast<AssemblyObject*>(obj);
	bodyitem1_ = obj->bodyItem();
	bodyitem2_ = env;
	initialCollision("obj env collide");
	is_env_pair_ = true;
}

ObjEnvColCheckPair::~ObjEnvColCheckPair() {
}

void ObjEnvColCheckPair::initialCollision(const std::string& default_debug_msg) {
	addColdetLinkPairs(bodyitem1_, bodyitem2_);
	col_test_check_func_vec_.push_back(boost::bind(&ObjEnvColCheckPair::isNotUseSafeBoundingBox, this));
	debug_messages_.push_back(default_debug_msg);
	// safe bounding box
	addColdetLinkPairsSafe(object_, bodyitem2_);
	col_test_check_func_vec_.push_back(boost::bind(&ObjEnvColCheckPair::isUseSafeBoundingBox, this));
	debug_messages_.push_back(default_debug_msg);
}

bool ObjEnvColCheckPair::isUseSafeBoundingBox() const {
	return ((PlanBase::instance()->getObjectContactState() == PlanBase::OFF_ENVIRONMENT) &&
					PlanBase::instance()->useObjectSafeBoundingBox);
}

bool ObjEnvColCheckPair::isNotUseSafeBoundingBox() const {
	return ((PlanBase::instance()->getObjectContactState() == PlanBase::OFF_ENVIRONMENT) &&
					(!(PlanBase::instance()->useObjectSafeBoundingBox)));
}

HandEnvColCheckPair::HandEnvColCheckPair(ObjectBase* hand, cnoid::BodyItemPtr& env) {
	hand_ = static_cast<RobotHand*>(hand);
	bodyitem1_ = hand->bodyItem();
	bodyitem2_ = env;
	initialCollision("hand env collide");
	is_env_pair_ = true;
}

HandEnvColCheckPair::~HandEnvColCheckPair() {
}

void HandEnvColCheckPair::initialCollision(const std::string& default_debug_msg) {
	addColdetLinkPairs(bodyitem1_, bodyitem2_);
	col_test_check_func_vec_.push_back(boost::bind(&HandEnvColCheckPair::isNotUseSafeBoundingBox, this));
	debug_messages_.push_back(default_debug_msg);
	// safe bounding box
	addColdetLinkPairsSafe(hand_, bodyitem2_);
	col_test_check_func_vec_.push_back(boost::bind(&HandEnvColCheckPair::isUseSafeBoundingBox, this));
	debug_messages_.push_back(default_debug_msg);
}

bool HandEnvColCheckPair::isUseSafeBoundingBox() const {
	return PlanBase::instance()->useRobotSafeBoundingBox;
}

bool HandEnvColCheckPair::isNotUseSafeBoundingBox() const {
	return !(PlanBase::instance()->useRobotSafeBoundingBox);
}


ObjObjColCheckPair::ObjObjColCheckPair(ObjectBase* obj1, ObjectBase* obj2) {
	obj1_ = static_cast<AssemblyObject*>(obj1);
	obj2_ = static_cast<AssemblyObject*>(obj2);
	bodyitem1_ = obj1->bodyItem();
	bodyitem2_ = obj2->bodyItem();
	initialCollision("obj obj collide");
}

ObjObjColCheckPair::~ObjObjColCheckPair() {
}

void ObjObjColCheckPair::initialCollision(const std::string& default_debug_msg) {
	addColdetLinkPairs(bodyitem1_, bodyitem2_);
	col_test_check_func_vec_.push_back(boost::bind(&ObjObjColCheckPair::isCheckCollision, this));
	debug_messages_.push_back(default_debug_msg);
}

bool ObjObjColCheckPair::isCheckCollision() const {
	return (!(obj1_->isConnected(obj2_, true)));
}

HandHandColCheckPair::HandHandColCheckPair(ObjectBase* hand1, ObjectBase* hand2) {
	hand1_ = static_cast<RobotHand*>(hand1);
	hand2_ = static_cast<RobotHand*>(hand2);
	bodyitem1_ = hand1->bodyItem();
	bodyitem2_ = hand2->bodyItem();
	initialCollision("hand hand collide");
}

HandHandColCheckPair::~HandHandColCheckPair() {
}

HandRobotColCheckPair::HandRobotColCheckPair(ObjectBase* hand, ObjectBase* robot) {
	hand_ = static_cast<RobotHand*>(hand);
	robot_ = static_cast<RobotBody*>(robot);
	bodyitem1_ = hand->bodyItem();
	bodyitem2_ = robot->bodyItem();
	initialCollision("hand robot collide");
}

HandRobotColCheckPair::~HandRobotColCheckPair() {
}

void HandRobotColCheckPair::initialCollision(const std::string& default_debug_msg) {
	// between hand links and robot links excluding wrist
	std::vector<cnoid::Link*> hand_robot_exclude_wrist;
	for (int i = 0; i < robot_->bodyItem()->body()->numLinks(); i++) {
		bool is_wrist = false;
		for (int j = 0; j < robot_->armSize(); j++) {
			if (robot_->bodyItem()->body()->link(i) == robot_->getArmFingers(j)->wrist) {
				is_wrist = true;
				break;
			}
		}
		if (is_wrist) continue;
		hand_robot_exclude_wrist.push_back(robot_->bodyItem()->body()->link(i));
	}
	addColdetLinkPairs(bodyitem2_, hand_robot_exclude_wrist, bodyitem1_);
	col_test_check_func_vec_.push_back(boost::bind(&HandRobotColCheckPair::alwaysCheck, this));
	debug_messages_.push_back(default_debug_msg);

	// between hand links and wrist link
	for (int i = 0; i < robot_->armSize(); i++) {
		std::vector<cnoid::Link*> wrist_link(1);
		wrist_link[0] = robot_->getArmFingers(i)->wrist;
		addColdetLinkPairs(bodyitem2_, wrist_link, bodyitem1_);
		col_test_check_func_vec_.push_back(boost::bind(&HandRobotColCheckPair::isCheckCollision, this, i));
		debug_messages_.push_back(default_debug_msg);
	}
}

bool HandRobotColCheckPair::isCheckCollision(int arm_id) const {
	Connection* con = robot_->findConnection(hand_, robot_->getArmFingers(arm_id)->wrist);
	if (con == NULL) return true;
	return (!(con->isConnected() || con->isUnderconnect()));
}

ObjRobotColCheckPair::ObjRobotColCheckPair(ObjectBase* obj, ObjectBase* robot) {
	obj_ = static_cast<AssemblyObject*>(obj);
	robot_ = static_cast<RobotBody*>(robot);
	bodyitem1_ = obj->bodyItem();
	bodyitem2_ = robot->bodyItem();
	initialCollision("object robot collide");
}

ObjRobotColCheckPair::~ObjRobotColCheckPair() {
}

void ObjRobotColCheckPair::initialCollision(const std::string& default_debug_msg) {
	// between object links and robot links excluding hand
	std::vector<cnoid::Link*> obj_robot_exclude_hand;
	std::vector<std::vector<cnoid::Link*> > hand_links;

	robot_->obtainBodyAndHandsLink(obj_robot_exclude_hand, hand_links);

	addColdetLinkPairs(bodyitem2_, obj_robot_exclude_hand, bodyitem1_);
	col_test_check_func_vec_.push_back(boost::bind(&ObjRobotColCheckPair::alwaysCheck, this));
	debug_messages_.push_back(default_debug_msg);
	
	// between hand links and wrist link
	for (int i = 0; i < robot_->armSize(); i++) {
		if (robot_->getArmFingers(i)->isSeparatedModel) continue;
		addColdetLinkPairs(bodyitem2_, hand_links[i], bodyitem1_);
		col_test_check_func_vec_.push_back(boost::bind(&ObjRobotColCheckPair::isCheckCollision, this, i));
		debug_messages_.push_back(default_debug_msg);
	}
}

bool ObjRobotColCheckPair::isCheckCollision(int arm_id) const {
	if (robot_->getArmFingers(arm_id)->isSeparatedModel) return false;
	if (robot_->getArmFingers(arm_id)->arm->graspingState == Arm::GRASPING) return false;
	return true;
}

HandObjColCheckPair::HandObjColCheckPair(ObjectBase* obj, ObjectBase* hand) {
	obj_ = static_cast<AssemblyObject*>(obj);
	hand_ = static_cast<RobotHand*>(hand);
	bodyitem1_ = obj->bodyItem();
	bodyitem2_ = hand->bodyItem();
	initialCollision("hand object coolide");
}

HandObjColCheckPair::~HandObjColCheckPair() {
}

bool HandObjColCheckPair::isCheckCollision() const {
	Connection* con = hand_->findConnection(obj_);
	if (con == NULL) return true;
	if (con->isConnected()) return false;
	return true;
}

void HandObjColCheckPair::initialCollision(const std::string& default_debug_msg) {
	addColdetLinkPairs(bodyitem1_, bodyitem2_);
	col_test_check_func_vec_.push_back(boost::bind(&HandObjColCheckPair::isCheckCollision, this));
	debug_messages_.push_back(default_debug_msg);
}

HandTargetObjColCheckPair::HandTargetObjColCheckPair(TargetObject* obj, RobotHand* hand) {
	obj_ = obj;
	hand_ = hand;
	bodyitem1_ = obj->bodyItemObject;
	bodyitem2_ = hand->bodyItem_;
	initialCollision("hand object collide");
}

HandTargetObjColCheckPair::~HandTargetObjColCheckPair() {
}

void HandTargetObjColCheckPair::initialCollision(const std::string& default_debug_msg) {
	addColdetLinkPairs(bodyitem1_, bodyitem2_);
	col_test_check_func_vec_.push_back(boost::bind(&HandTargetObjColCheckPair::isCheckCollision, this));
	debug_messages_.push_back(default_debug_msg);
}

bool HandTargetObjColCheckPair::isCheckCollision() const {
	Arm* target_arm = hand_->arm();
	if (target_arm == NULL) return true;
	if (target_arm->graspingState == Arm::GRASPING &&
			 target_arm->target_grasp_objid < 0) {
		return false;
	}
	return true;
}

ColCheckPairPointCloud::ColCheckPairPointCloud() :
min_sep_(1.e10) {
}

ColCheckPairPointCloud::~ColCheckPairPointCloud() {
}

bool ColCheckPairPointCloud::isColliding(double tolerance, std::ostream& out) const {
	for (size_t i = 0; i < col_link_pairs_vec_.size(); i++) {
		if (!col_test_check_func_vec_[i]()) continue;
		for (size_t j = 0; j < col_link_pairs_vec_[i].size(); j++) {
			bool coll = PointCloudCollisionChecker::isCollidingPointCloudSub(col_link_pairs_vec_[i][j].second, col_link_pairs_vec_[i][j].first, tolerance);
			if (coll) {
#ifdef DEBUG_MODE
				out << debug_messages_[i] << " point cloud " << col_link_pairs_vec_[i][j].first->name() << std::endl;
#endif
				return true;
			}
		}
	}
	return false;
}

double ColCheckPairPointCloud::clearance(double tolerance) const {
	if (this->isColliding(tolerance)) {
		return 0;
	}
	return min_sep_;
}

void ColCheckPairPointCloud::initialCollision(const std::string& default_debug_msg) {
	col_link_pairs_vec_.resize(1);
	col_link_pairs_vec_[0].resize(bodyitem_->body()->numLinks());
	for (int i = 0; i < bodyitem_->body()->numLinks(); i++) {
		std::pair<cnoid::Link*, PointCloudEnv*> link_pointcloud(bodyitem_->body()->link(i), pointcloud_);
		col_link_pairs_vec_[0][i] = link_pointcloud;
	}
	col_test_check_func_vec_.push_back(boost::bind(&ColCheckPairPointCloud::alwaysCheck, this));
	debug_messages_.push_back(default_debug_msg);
}

RobotPointCloudColCheckPair::RobotPointCloudColCheckPair(ObjectBase* obj, PointCloudEnv* pointcloud) {
	bodyitem_ = obj->bodyItem();
	pointcloud_ = pointcloud;
	robot_ = static_cast<RobotBody*>(obj);
	initialCollision("robot pointcloud collide");
}

RobotPointCloudColCheckPair::~RobotPointCloudColCheckPair() {
}

void RobotPointCloudColCheckPair::initialCollision(const std::string& default_debug_msg) {
	// between object links and robot links excluding hand
	std::vector<cnoid::Link*> robot_exclude_hand;
	std::vector<std::vector<cnoid::Link*> > hand_links;

	robot_->obtainBodyAndHandsLink(robot_exclude_hand, hand_links);

	col_link_pairs_vec_.clear();
	col_link_pairs_vec_.push_back(std::vector<std::pair<cnoid::Link*, PointCloudEnv*> >());
	for (int i = 0; i < robot_exclude_hand.size(); i++) {
		std::pair<cnoid::Link*, PointCloudEnv*> link_pointcloud(robot_exclude_hand[i], pointcloud_);
		col_link_pairs_vec_[0].push_back(link_pointcloud);
	}
	col_test_check_func_vec_.push_back(boost::bind(&RobotPointCloudColCheckPair::alwaysCheck, this));
	debug_messages_.push_back(default_debug_msg);
	
	// between hand links and point cloud
	for (int i = 0; i < robot_->armSize(); i++) {
		if (robot_->getArmFingers(i)->isSeparatedModel) continue;
		col_link_pairs_vec_.push_back(std::vector<std::pair<cnoid::Link*, PointCloudEnv*> >());
		for (int j = 0; j < hand_links[i].size(); j++) {
			std::pair<cnoid::Link*, PointCloudEnv*> link_pointcloud(hand_links[i][j], pointcloud_);
			col_link_pairs_vec_.back().push_back(link_pointcloud);
		}
		col_test_check_func_vec_.push_back(boost::bind(&RobotPointCloudColCheckPair::isCheckFingerPointCloudPair, this));
		debug_messages_.push_back(default_debug_msg);
	}
}

bool RobotPointCloudColCheckPair::isCheckFingerPointCloudPair() {
	return PlanBase::instance()->doCheckCollisionPointCloudFinger;
}

HandPointCloudColCheckPair::HandPointCloudColCheckPair(ObjectBase* obj, PointCloudEnv* pointcloud) {
	bodyitem_ = obj->bodyItem();
	pointcloud_ = pointcloud;
	initialCollision("robot pointcloud collide");
}

HandPointCloudColCheckPair::~HandPointCloudColCheckPair() {
}

void HandPointCloudColCheckPair::initialCollision(const std::string& default_debug_msg) {
	col_link_pairs_vec_.resize(1);
	col_link_pairs_vec_[0].resize(bodyitem_->body()->numLinks());
	for (int i = 0; i < bodyitem_->body()->numLinks(); i++) {
		std::pair<cnoid::Link*, PointCloudEnv*> link_pointcloud(bodyitem_->body()->link(i), pointcloud_);
		col_link_pairs_vec_[0][i] = link_pointcloud;
	}
	col_test_check_func_vec_.push_back(boost::bind(&HandPointCloudColCheckPair::isCheckFingerPointCloudPair, this));
	debug_messages_.push_back(default_debug_msg);
}

bool HandPointCloudColCheckPair::isCheckFingerPointCloudPair() {
	return PlanBase::instance()->doCheckCollisionPointCloudFinger;
}

ObjPointCloudColCheckPair::ObjPointCloudColCheckPair(ObjectBase* obj, PointCloudEnv* pointcloud) {
	bodyitem_ = obj->bodyItem();
	pointcloud_ = pointcloud;
	initialCollision("obj pointcloud collide");
}

ObjPointCloudColCheckPair::~ObjPointCloudColCheckPair() {
}

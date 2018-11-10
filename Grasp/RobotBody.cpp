#include "RobotBody.h"

#include "PlanBase.h"

#include "RobotHand.h"
#include "AssemblyObject.h"
#include "ObjectManager.h"

using namespace grasp;

RobotArmFingers::RobotArmFingers(RobotBody* robotbody, const cnoid::Mapping& armSetting) :
	owner_(robotbody),
	armfinger_(NULL),
	current_hand_idx_(-1),
	os(cnoid::MessageView::instance()->cout()) {
	makeArm(armSetting);
}

RobotArmFingers::RobotArmFingers(RobotBody* robotbody, const cnoid::Mapping& armSetting, const std::vector<cnoid::BodyItemPtr>& hand_bodies) :
	owner_(robotbody),
	armfinger_(NULL),
	current_hand_idx_(-1),
	os(cnoid::MessageView::instance()->cout()) {
	PlanBase* pb = PlanBase::instance();
	std::vector<InterLink> interLinkListtmp = pb->interLinkList;
	std::vector<InterObject> interObjectListtmp = pb->interObjectList;
	makeArmProc(armSetting, hand_bodies);
	pb->interLinkList = interLinkListtmp;
	pb->interObjectList = interObjectListtmp;
}

RobotArmFingers::~RobotArmFingers() {
	if (armfinger_ != NULL) {
		if (armfinger_->parentItem() != NULL) {
			armfinger_->detachFromParentItem();
		} else {
			delete armfinger_;
		}
	}
	clearHandList();
}

ArmFingers* RobotArmFingers::getArmFingers() {
	return armfinger_;
}

int RobotArmFingers::handListSize() const {
	return hand_list_.size();
}

bool RobotArmFingers::attachHand(int hand_index) {
	if (isAttached()) {
		os << "fail to attach a hand: a hand is already attached" << std::endl;
		return false;
	}
	if (hand_index >= handListSize()) return false;

	RobotHand* target_hand = hand_list_[hand_index];

	owner_->connect(target_hand, armfinger_->wrist,
									armfinger_->arm->toolWristRot, armfinger_->arm->toolWristPos);
	target_hand->setArm(armfinger_->arm);

	current_hand_idx_ = hand_index;
	armfinger_->nFing = target_hand->nFing;
	armfinger_->nHandLink = target_hand->nHandLink;
	armfinger_->palm = target_hand->palm;
	armfinger_->fingers = target_hand->fingers;
	armfinger_->handJoint = target_hand->handJoint;
	armfinger_->mu = target_hand->mu;
	armfinger_->fmax = target_hand->fmax;
	armfinger_->hmax = target_hand->hmax;
	armfinger_->handName = target_hand->handName;
	armfinger_->prehensionFilePathList = target_hand->prehensionFilePathList;
	armfinger_->prehensionList = target_hand->prehensionList;
	armfinger_->dataFilePath = target_hand->dataFilePath;

	return true;
}

void RobotArmFingers::attachHand(const cnoid::BodyItemPtr& bodyitem) {
	for (size_t i = 0; i < hand_list_.size(); i++) {
		if (hand_list_[i]->bodyItem() != bodyitem) return;
		if (i == current_hand_idx_) continue;
		attachHand(i);
	}
}

bool RobotArmFingers::detachHand() {
	if (!armfinger_->isSeparatedModel) return true;
	if (isAttached()) {
		getCurrentRobotHand()->setArm(NULL);
			
		Connection* con = owner_->findConnection(getCurrentRobotHand(), armfinger_->wrist);
		if (con != NULL) {
			con->setUnconnected();
		}
	}

	armfinger_->nFing = 0;
	armfinger_->nHandLink = 0;
	armfinger_->palm = armfinger_->wrist;
	armfinger_->fingers = NULL;
	armfinger_->handJoint = NULL;
	armfinger_->mu = 0.5;
	armfinger_->fmax = 10;
	armfinger_->hmax = 0.005;
	armfinger_->handName = "";
	armfinger_->prehensionFilePathList.clear();
	armfinger_->prehensionList.clear();
	armfinger_->dataFilePath = "";
	current_hand_idx_ = -1;

	return true;
}

int RobotArmFingers::getCurrentHandIndex() const {
	return  current_hand_idx_;
}

bool RobotArmFingers::isAttached() const {
	return (current_hand_idx_ >=0);
}

RobotHand* RobotArmFingers::getRobotHand(int hand_index) const {
	return hand_list_[hand_index];
}

RobotHand* RobotArmFingers::getCurrentRobotHand() const {
	if (isAttached()) {
		return hand_list_[current_hand_idx_];
	}
	return NULL;
}

void RobotArmFingers::addRobotHand(RobotHand* hand) {
	hand_list_.push_back(hand);
}

void RobotArmFingers::connectObject(ObjectBase* slave, cnoid::Link* master) {
	if (!isAttached()) {
		if (NULL == owner_->findConnection(slave, master)) {
			owner_->connect(slave, master);
		}
	} else {
		if (NULL == getCurrentRobotHand()->findConnection(slave, master)) {
			getCurrentRobotHand()->connect(slave, master);
		}
	}
}

void RobotArmFingers::setGraspingState(int state) {
	if (armfinger_->arm->target_grasp_objid < 0) return;
	ObjectBase* target_object = owner_->getObjectManager()->getAssemblyObject(armfinger_->arm->target_grasp_objid);
	cnoid::Link* master;
	if (armfinger_->nFing > 0) {
		master = armfinger_->fingers[0]->tip;
	} else {
		master = armfinger_->palm;
	}
	ObjectBase* master_object;
	if (!isAttached()) {
		master_object = owner_;
	} else {
		master_object = getCurrentRobotHand();
	}
	if (state == Arm::GRASPING) {
		master_object->connect(target_object, master);
	} else if (state == Arm::UNDER_GRASPING) {
		master_object->connect(target_object, master, Connection::STATE_UNDERCONNECT);
	} else if (state == Arm::NOT_GRASPING) {
		master_object->connect(target_object, master, Connection::STATE_UNCONNECTED);
	}
}

void RobotArmFingers::makeArm(const cnoid::Mapping& armSetting) {
	cnoid::ItemList<cnoid::BodyItem> bodyitemlist;
	bodyitemlist.extractChildItems(cnoid::ItemTreeView::instance()->rootItem());

	std::vector<cnoid::BodyItemPtr> bodyitems(bodyitemlist.size());
	for (size_t j = 0; j < bodyitemlist.size(); j++) {
		bodyitems[j] = bodyitemlist[j];
	}

	makeArmProc(armSetting, bodyitems);
}

void RobotArmFingers::makeArmProc(const cnoid::Mapping& armSetting, const std::vector<cnoid::BodyItemPtr>& bodyitemlist) {
	clearHandList();

	armfinger_ = new ArmFingers(owner_->getRootBodyItem(), armSetting);

	const cnoid::Listing& handname_list = *armSetting.findListing("handNameList");
	bool is_handseparatedmodel = (handname_list.isValid() && !handname_list.empty());

	armfinger_->isSeparatedModel = is_handseparatedmodel;

	if (is_handseparatedmodel) {
		std::vector<cnoid::BodyItemPtr> hand_bodyitems;

		searchHandBodyItems(handname_list, bodyitemlist, hand_bodyitems);

		for (size_t i = 0; i < hand_bodyitems.size(); i++) {
			RobotHand* hand = owner_->getObjectManager()->getOrCreateHand(hand_bodyitems[i]).get();
			hand_list_.push_back(hand);
		}

		detachHand();
	}
}

void RobotArmFingers::searchHandBodyItems(const cnoid::Listing& handname_list,
																					const std::vector<cnoid::BodyItemPtr>& bodyitemlist,
																					std::vector<cnoid::BodyItemPtr>& hand_bodyitems) const {
	for (int i = 0; i < handname_list.size(); i++) {
		std::string hand_name = handname_list[i].toString();

		bool has_hand = false;

		for (size_t j = 0; j < bodyitemlist.size(); j++) {
			cnoid::Mapping* settings = bodyitemlist[j]->body()->info()->findMapping("graspPluginHandSetting");
			if (!settings->isValid()) continue;
			cnoid::ValueNode* handname_node = settings->find("handName");
			if (handname_node->isString()) {
				if (handname_node->toString() == hand_name) {
					hand_bodyitems.push_back(bodyitemlist[j]);
					has_hand = true;
					break;
				}
			}
		}

		if (has_hand) {
			os << "hand " << hand_name << " is found" << std::endl;
		} else {
			os << "hand " << hand_name << " is NOT FOUND!!" << std::endl;
		}
	}
}

void RobotArmFingers::clearHandList() {
	hand_list_.clear();
}

RobotBody::RobotBody(cnoid::BodyItemPtr rootBodyItem, ObjectManager* obj_manager, bool load_arms) :
	owner_(obj_manager) {
	type_ = TYPE_ROBOT;
	body_item_ = rootBodyItem;
	rootBodyItem_ = rootBodyItem;
	bb_safety_size_ = cnoid::Vector3(0.03, 0.03, 0.03);
	calcSafeBoundingBox(bb_safety_size_);
	if (load_arms) {
		makeArms();
		updateJointList();
	}
}

RobotBody::RobotBody(cnoid::BodyItemPtr rootBodyItem, bool load_arms) {
	rootBodyItem_ = rootBodyItem;
	if (load_arms) {
		makeArms();
	}
	updateJointList();
}

ObjectBasePtr RobotBody::clone(ObjectManager* owner) const {
	cnoid::BodyItemPtr new_body = new cnoid::BodyItem(*(bodyItem()));
	RobotBodyPtr ret = RobotBodyPtr(new RobotBody(new_body, owner, false));
	cloneProc(ret.get());
	ret->rootBodyItem_ = new_body;
	return ret;
}

RobotBody::~RobotBody() {
	clearArms();
}

cnoid::BodyItemPtr RobotBody::getRootBodyItem() const {
	return rootBodyItem_;
}

cnoid::BodyItemPtr RobotBody::getBodyItem(int arm_id, int hand_index) const {
	return armsList_[arm_id]->getRobotHand(hand_index)->bodyItem_;
}

int RobotBody::armSize() const {
	return armsList_.size();
}

int RobotBody::handListSize(int arm_id) const {
	return armsList_[arm_id]->handListSize();
}

bool RobotBody::attachHand(int arm_id, int hand_index) {
	return armsList_[arm_id]->attachHand(hand_index);
}

bool RobotBody::detachHand(int arm_id) {
	return armsList_[arm_id]->detachHand();
}

bool RobotBody::isAttached(int arm_id) const {
	return armsList_[arm_id]->isAttached();
}

int RobotBody::getCurrentHandID(int arm_id) const {
	return armsList_[arm_id]->getCurrentHandIndex();
}

RobotHand* RobotBody::getCurrentHand(int arm_id) const {
	return armsList_[arm_id]->getCurrentRobotHand();
}

RobotHand* RobotBody::getHand(int arm_id, int hand_index) const {
	return armsList_[arm_id]->getRobotHand(hand_index);
}

RobotArmFingers* RobotBody::getRobotArmFingers(int arm_id) const {
	return armsList_[arm_id];
}

ArmFingers* RobotBody::getArmFingers(int arm_id) const {
	return armsList_[arm_id]->getArmFingers();
}

void RobotBody::obtainBodyAndHandsLink(std::vector<cnoid::Link*>& body_links,
																			 std::vector<std::vector<cnoid::Link*> >& hand_links) const {
	body_links.clear();
	std::vector<bool> hand_flag(bodyItem()->body()->numLinks(), false);
	hand_links.resize(armSize());
	for (int i = 0; i < armSize(); i++) {
		hand_links[i].clear();
		if (getArmFingers(i)->isSeparatedModel) continue;
		cnoid::LinkTraverse* handjoint = getArmFingers(i)->handJoint;
		for (std::vector<cnoid::Link*>::const_iterator it = handjoint->begin();
				 it != handjoint->end(); ++it) {
			hand_links[i].push_back((*it));
			hand_flag[(*it)->index()] = true;
		}
	}

	for (int i = 0; i < bodyItem()->body()->numLinks(); i++) {
		if (!hand_flag[i]) body_links.push_back(bodyItem()->body()->link(i));
	}
}

cnoid::Link* RobotBody::rootLink() const {
	return rootBodyItem_->body()->rootLink();
}

cnoid::Link* RobotBody::link(int index) const {
	return linkid_to_link_[index].link;
}

cnoid::Link* RobotBody::link(const std::string& name) const {
	for (int i = 0; i < numLinks(); i++) {
		if (linkid_to_link_[i].link->name() == name) {
			return linkid_to_link_[i].link;
		}
	}
	return NULL;
}

cnoid::Link* RobotBody::joint(int index) const {
	return jointid_to_link_[index].link;
}

int RobotBody::numJoints() const {
	return jointid_to_link_.size();
}

int RobotBody::numLinks() const {
	return linkid_to_link_.size();
}

const std::vector<cnoid::BodyItemPtr>& RobotBody::getHandBodyItems() const {
	return handBodyItems_;
}

RobotBody::BodyItemIterator RobotBody::handbody_begin() {
	return handBodyItems_.begin();
}

RobotBody::BodyItemIterator RobotBody::handbody_end() {
	return handBodyItems_.end();
}

int RobotBody::allHandListSize() const {
	return handBodyItems_.size();
}

void RobotBody::updateJointList() {
	jointid_to_link_.clear();
	linkid_to_link_.clear();

	int max_joint_id = 0;
	jointid_to_link_.resize(rootBodyItem_->body()->numJoints());
	for (int i = 0; i < rootBodyItem_->body()->numJoints(); ++i) {
		jointid_to_link_[i].bodyitem = rootBodyItem_;
		jointid_to_link_[i].link = rootBodyItem_->body()->joint(i);
		int joint_id = rootBodyItem_->body()->joint(i)->jointId();
		if (max_joint_id < joint_id) {
			max_joint_id = joint_id;
		}
	}

	linkid_to_link_.resize(rootBodyItem_->body()->numLinks());
	for (int i = 0; i < rootBodyItem_->body()->numLinks(); ++i) {
		linkid_to_link_[i].bodyitem = rootBodyItem_;
		linkid_to_link_[i].link = rootBodyItem_->body()->link(i);
	}

	for (BodyItemIterator it = handbody_begin(); it != handbody_end(); ++it) {
		for (int j = 0; j < (*it)->body()->numJoints(); ++j) {
			jointid_to_link_.push_back(BodyItemLink());
			jointid_to_link_.back().bodyitem = *it;
			jointid_to_link_.back().link = (*it)->body()->joint(j);
			(*it)->body()->joint(j)->setJointId(++max_joint_id);
		}
		for (int j = 0; j < (*it)->body()->numLinks(); ++j) {
			linkid_to_link_.push_back(BodyItemLink());
			linkid_to_link_.back().bodyitem = *it;
			linkid_to_link_.back().link = (*it)->body()->link(j);
		}
	}
}

void RobotBody::calcForwardKinematics() {
	PlanBase::instance()->setInterLink();
	rootBodyItem_->body()->calcForwardKinematics();
}

void RobotBody::notifyKinematicStateChange() {
	rootBodyItem_->notifyKinematicStateChange();
	for (size_t i = 0; i < armsList_.size(); i++) {
		if (!armsList_[i]->isAttached()) continue;
		armsList_[i]->getCurrentRobotHand()->bodyItem_->notifyKinematicStateChange();
	}
}

ObjectManager* RobotBody::getObjectManager() const {
	return owner_;
}

void RobotBody::restoreState(const ObjectsState::ObjectState& state) {
	if (state.extra_state_data.empty()) {
		for (size_t i = 0; i < armsList_.size(); i++) {
			armsList_[i]->getArmFingers()->arm->target_grasp_objid = -1;
		}
	} else {
		const std::vector<int>& obj_ids = boost::get<std::vector<int> >(state.extra_state_data[0]);
		for (size_t i = 0; i < obj_ids.size(); i++) {
			armsList_[i]->getArmFingers()->arm->target_grasp_objid = obj_ids[i];
		}
	}
	ObjectBase::restoreState(state);
	for (int m = 0; m < armsList_.size(); m++) {
		bool is_hand_attached = false;
		for (int i = 0; i < descendant_connections_.size(); i++) {
			if (!descendant_connections_[i]->slaveObject()->isHand()) continue;
			if ((descendant_connections_[i]->masterLink() == armsList_[m]->getArmFingers()->wrist) &&
					(descendant_connections_[i]->isConnected())) {
				if (!armsList_[m]->isAttached()) {
					armsList_[m]->attachHand(descendant_connections_[i]->slaveObject()->bodyItem());
				}
				is_hand_attached = true;
				break;
			}
		}
		if (!is_hand_attached) {
			armsList_[m]->detachHand();
		}
	}
}

void RobotBody::storeState(ObjectsState::ObjectState& state) const {
	std::vector<int> obj_ids;
	for (size_t i = 0; i < armsList_.size(); i++) {
		obj_ids.push_back(armsList_[i]->getArmFingers()->arm->target_grasp_objid);
	}
	state.extra_state_data.clear();
	state.extra_state_data.push_back(obj_ids);
	ObjectBase::storeState(state);
	for (size_t i = 0; i < armsList_.size(); i++) {
		armsList_[i]->setGraspingState(armsList_[i]->getArmFingers()->arm->graspingState);
	}
}

int& RobotBody::stateGraspObjectID(ObjectsState& state, int arm_id) {
	ObjectsState::ObjectState& s = state.objectState(this->descriptor());
	
	return boost::get<std::vector<int> >(s.extra_state_data[0])[arm_id];
}

void RobotBody::copyHandsbind(RobotBody* org) {
	std::vector<cnoid::BodyItemPtr> hand_bodies;
	for (size_t i = 0; i < org->handBodyItems_.size(); i++) {
		/// If the max jointId is not equal to the number of joints, a bodyitem generated by
		/// BodyItem's clone method has not the same number of joints as original bodyitem
		/// (Since BodyItem's clone method adds dummy joints).
		/// We temporarily change jointIds of original bodyitem in order to obtain a bodyitem with
		/// the same number of joints.
		std::vector<int> jointid_list;
		for (int j = 0; j < org->handBodyItems_[i]->body()->numJoints(); j++) {
			jointid_list.push_back(org->handBodyItems_[i]->body()->joint(j)->jointId());
			org->handBodyItems_[i]->body()->joint(j)->setJointId(j);
		}
		const cnoid::BodyItemPtr& copy_bodyitem = org->getObjectManager()->getOrCreateHand(org->handBodyItems_[i])->bodyItem();
		for (int j = 0; j < org->handBodyItems_[i]->body()->numJoints(); j++) {
			org->handBodyItems_[i]->body()->joint(j)->setJointId(jointid_list[j]);
		}
		hand_bodies.push_back(copy_bodyitem);
	}

	cnoid::ValueNode* g_setting_node = rootBodyItem_->body()->info()->find("graspPluginSetting");
	if (g_setting_node->isListing()) {  // multi arm
		const cnoid::Listing& glist = *(g_setting_node->toListing());
		for (int i = 0; i < glist.size(); i++) {
			const cnoid::Mapping& gSettings = *glist[i].toMapping();
			if (gSettings.isValid() && !gSettings.empty()) {
				RobotArmFingers* arm = new RobotArmFingers(this, gSettings);
				RobotArmFingers* org_arm = org->getRobotArmFingers(i);
				for (int j = 0; j < org_arm->handListSize(); j++) {
					RobotHand* target_hand = owner_->getHand(org_arm->getRobotHand(j)->id);
					arm->addRobotHand(target_hand);
				}
				arm->getArmFingers()->arm->target_grasp_objid = org_arm->getArmFingers()->arm->target_grasp_objid;
				arm->getArmFingers()->arm->graspingState = org_arm->getArmFingers()->arm->graspingState;
				arm->getArmFingers()->objectPalmPos = org_arm->getArmFingers()->objectPalmPos;
				arm->getArmFingers()->objectPalmRot = org_arm->getArmFingers()->objectPalmRot;
				if (org_arm->isAttached()) {
					arm->attachHand(org_arm->getCurrentHandIndex());
				}
				armsList_.push_back(arm);
			}
		}
	} else if (g_setting_node->isMapping()) {  // single arm
		const cnoid::Mapping& gSettings = *(g_setting_node->toMapping());
		if (gSettings.isValid() && !gSettings.empty()) {
			RobotArmFingers* arm = new RobotArmFingers(this, gSettings);
			RobotArmFingers* org_arm = org->getRobotArmFingers(0);
			for (int j = 0; j < org_arm->handListSize(); j++) {
				RobotHand* target_hand = owner_->getHand(org_arm->getRobotHand(j)->id);
				arm->addRobotHand(target_hand);
			}
			arm->getArmFingers()->arm->target_grasp_objid = org_arm->getArmFingers()->arm->target_grasp_objid;
			arm->getArmFingers()->arm->graspingState = org_arm->getArmFingers()->arm->graspingState;
			arm->getArmFingers()->objectPalmPos = org_arm->getArmFingers()->objectPalmPos;
			arm->getArmFingers()->objectPalmRot = org_arm->getArmFingers()->objectPalmRot;
			if (org_arm->isAttached()) {
				arm->attachHand(org_arm->getCurrentHandIndex());
			}
			armsList_.push_back(arm);
		}
	}

	updateJointList();

	for (int i = 0; i < armSize(); i++) {
		ArmFingers* arm = getArmFingers(i);
		ArmFingers* org_arm = org->getArmFingers(i);
		arm->objectPalmPos = org_arm->objectPalmPos;
		arm->objectPalmRot = org_arm->objectPalmRot;
		arm->isSeparatedModel = org_arm->isSeparatedModel;
	}

}

void RobotBody::makeArms() {
	clearArms();

	cnoid::ValueNode* g_setting_node = rootBodyItem_->body()->info()->find("graspPluginSetting");
	if (g_setting_node->isListing()) {  // multi arm
		const cnoid::Listing& glist = *(g_setting_node->toListing());
		for (int i = 0; i < glist.size(); i++) {
			const cnoid::Mapping& gSettings = *glist[i].toMapping();
			if (gSettings.isValid() && !gSettings.empty()) {
				RobotArmFingers* arm = new RobotArmFingers(this, gSettings);
				armsList_.push_back(arm);
			}
		}
	} else if (g_setting_node->isMapping()) {  // single arm
		const cnoid::Mapping& gSettings = *(g_setting_node->toMapping());
		if (gSettings.isValid() && !gSettings.empty()) {
			RobotArmFingers* arm = new RobotArmFingers(this, gSettings);
			armsList_.push_back(arm);
		}
	}

	handBodyItems_.clear();
	for (ObjectManager::hand_iterator it = owner_->hand_begin();
			 it != owner_->hand_end(); ++it) {
		handBodyItems_.push_back((*it)->bodyItem());
	}
}

void RobotBody::clearArms() {
	for (size_t i = 0; i < armsList_.size(); i++) {
		if (armsList_[i] != NULL) delete armsList_[i];
	}
	armsList_.clear();
}


bool RobotBody::isColliding(std::string& col_pair_name1, std::string& col_pair_name2) {
	return false;
}

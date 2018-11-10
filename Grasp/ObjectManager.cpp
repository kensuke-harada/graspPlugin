#include "ObjectManager.h"

#include "RobotHand.h"
#include "AssemblyObject.h"
#include "ObjectIterator.h"

using namespace grasp;

ObjectManager::ObjectManager() :
	curr_descriptor_(0),
	curr_object_id_(0),
	curr_hand_id_(0) {
}

ObjectManager* ObjectManager::clone() const {
	ObjectManager* ret = new ObjectManager();

	// copy objects
	for (ObjectContainerConstIte it = objects_.begin(); it != objects_.end(); ++it) {
		ret->objects_[(*it).first] = (*it).second->clone(ret);
	}
	// copy hand infomation
	for (robot_iterator it = ret->robot_begin(); it != ret->robot_end(); ++it) {
		(*it)->copyHandsbind(dynamic_cast<RobotBody*>(this->objects_.find((*it)->descriptor())->second.get()));
	}
	// copy connections
	for (ObjectContainerConstIte it = objects_.begin(); it != objects_.end(); ++it) {
		ObjectBase* ori_base = (*it).second.get();
		for (size_t i = 0; i , ori_base->descendant_connections_.size(); i++) {
			const Connection* ori_con = ori_base->descendant_connections_.at(i);
			Connection* new_con = new Connection();
			ret->objects_[ori_base->descriptor()]->descendant_connections_.push_back(new_con);
			new_con->setSlaveObject(ret->getObject(ori_con->slaveObject()->descriptor()).get());
			new_con->setMasterLink(ret->getObject(ori_base->descriptor())->bodyItem()->body()->link(ori_con->masterLink()->index()));
			new_con->relativeRot() = ori_con->relativeRot();
			new_con->relativePos() = ori_con->relativePos();
			new_con->setState(ori_con->state());
		}
	}

	// copy member variable
	ret->curr_descriptor_ = curr_descriptor_;
	ret->curr_object_id_ = curr_object_id_;
	ret->curr_hand_id_ = curr_hand_id_;
	ret->object_id_descriptor_map_ = object_id_descriptor_map_;
	ret->hand_id_descriptor_map_ = hand_id_descriptor_map_;

	return ret;
}

ObjectManager::~ObjectManager() {
}

RobotBodyPtr ObjectManager::getOrCreateRobot(const cnoid::BodyItemPtr& bodyitem) {
	ObjectBasePtr target = hasObject(bodyitem);
	if (target != NULL) {
		return boost::dynamic_pointer_cast<RobotBody>(target);
	}
	return createRobot(bodyitem);
}

RobotBodyPtr ObjectManager::createRobot(const cnoid::BodyItemPtr& bodyitem) {
	RobotBodyPtr robot(new RobotBody(bodyitem, this));
	robot->descriptor_ = curr_descriptor_;
	objects_[curr_descriptor_++] = robot;
	return robot;
}

RobotHandPtr ObjectManager::getOrCreateHand(const cnoid::BodyItemPtr& bodyitem) {
	ObjectBasePtr target = hasObject(bodyitem);
	if (target != NULL) {
		return boost::dynamic_pointer_cast<RobotHand>(target);
	}
	return createHand(bodyitem);
}

RobotHandPtr ObjectManager::createHand(const cnoid::BodyItemPtr& bodyitem) {
	RobotHandPtr hand(new RobotHand(bodyitem));
	hand->descriptor_ = curr_descriptor_;
	objects_[curr_descriptor_++] = hand;
	hand->id = ++curr_hand_id_;
	hand_id_descriptor_map_[hand->id] = hand->descriptor_;
	return hand;
}

AssemblyObjectPtr ObjectManager::getOrCreateObject(const cnoid::BodyItemPtr& bodyitem) {
	ObjectBasePtr target = hasObject(bodyitem);
	if (target != NULL) {
		return boost::dynamic_pointer_cast<AssemblyObject>(target);
	}
	return createObject(bodyitem);
}

AssemblyObjectPtr ObjectManager::createObject(const cnoid::BodyItemPtr& bodyitem) {
	AssemblyObjectPtr object(new AssemblyObject(bodyitem, this));
	object->descriptor_ = curr_descriptor_;
	objects_[curr_descriptor_++] = object;
	object->id_ = curr_object_id_++;
	object_id_descriptor_map_[object->id_] = object->descriptor_;
	return object;
}

bool ObjectManager::detachObject(ObjectBasePtr object) {
	for (ObjectContainerIte it = objects_.begin(); it != objects_.end(); ++it) {
		if ((*it).second == object) {
			objects_.erase(it);
			return true;
		}
	}
	return false;
}

bool ObjectManager::detachObject(const cnoid::BodyItemPtr& bodyitem) {
	for (ObjectContainerIte it = objects_.begin(); it != objects_.end(); ++it) {
		if ((*it).second->bodyItem() == bodyitem) {
			objects_.erase(it);
			return true;
		}
	}
	return false;
}

bool ObjectManager::detachObject(ObjectDescriptor descriptor) {
	ObjectContainerIte it = objects_.find(descriptor);
	if (it == objects_.end()) return false;
	objects_.erase(it);
	return true;
}

ObjectBasePtr ObjectManager::getObject(ObjectDescriptor descriptor) const {
	if (objects_.count(descriptor) < 1) return ObjectBasePtr();
	return objects_.at(descriptor);
}

ObjectBasePtr ObjectManager::hasObject(const cnoid::BodyItemPtr& bodyitem) const {
	for (ObjectContainerConstIte it = objects_.begin(); it != objects_.end(); ++it) {
		if ((*it).second->bodyItem() == bodyitem.get()) {
			return (*it).second;
		}
	}
	return ObjectBasePtr();
}

AssemblyObject* ObjectManager::getAssemblyObject(int object_id) const {
	ObjectBasePtr ret = getObject(object_id_descriptor_map_.at(object_id));
	if (ret) {
		return static_cast<AssemblyObject*>(ret.get());
	}
	return NULL;
}

RobotHand* ObjectManager::getHand(int hand_id) const {
	ObjectBase* ret = getObject(hand_id_descriptor_map_.at(hand_id)).get();
	if (ret != NULL) {
		return static_cast<RobotHand*>(ret);
	}
	return NULL;
}

void ObjectManager::clear() {
	curr_descriptor_ = 0;
	curr_object_id_ = 0;
	curr_hand_id_ = 0;
	objects_.clear();
}

void ObjectManager::clearObjects() {
	ObjectContainerIte it = objects_.begin();
	while (it != objects_.end()) {
		if ((*it).second->isObject()) {
			objects_.erase(it++);
		} else {
			++it;
		}
	}
	curr_object_id_ = 0;
	object_id_descriptor_map_.clear();
}

void ObjectManager::clearHands() {
	ObjectContainerIte it = objects_.begin();
	while (it != objects_.end()) {
		if ((*it).second->isHand()) {
			objects_.erase(it++);
		} else {
			++it;
		}
	}
	curr_hand_id_ = 0;
	hand_id_descriptor_map_.clear();
}

void ObjectManager::extractRobotHands(std::vector<RobotHandPtr>& hands) const {
	hands.clear();
	for (ObjectContainerConstIte it = objects_.begin(); it != objects_.end(); ++it) {
		if ((*it).second->isHand()) {
			hands.push_back(boost::static_pointer_cast<RobotHand>((*it).second));
		}
	}
}

void ObjectManager::extractRobotHands(std::vector<RobotHand*>& hands) const {
	hands.clear();
	for (ObjectContainerConstIte it = objects_.begin(); it != objects_.end(); ++it) {
		if ((*it).second->isHand()) {
			hands.push_back(static_cast<RobotHand*>((*it).second.get()));
		}
	}
}

void ObjectManager::extractObjects(std::vector<AssemblyObjectPtr>& objects) const {
	objects.clear();
	for (ObjectContainerConstIte it = objects_.begin(); it != objects_.end(); ++it) {
		if ((*it).second->isObject()) {
			objects.push_back(boost::static_pointer_cast<AssemblyObject>((*it).second));
		}
	}
}

void ObjectManager::extractObjects(std::vector<AssemblyObject*>& objects) const {
	objects.clear();
	for (ObjectContainerConstIte it = objects_.begin(); it != objects_.end(); ++it) {
		if ((*it).second->isObject()) {
			objects.push_back(static_cast<AssemblyObject*>((*it).second.get()));
		}
	}
}

void ObjectManager::extract(std::vector<ObjectBase*>& objects, ObjectBase::Type type_flag) const {
	objects.clear();
	for (ObjectContainerConstIte it = objects_.begin(); it != objects_.end(); ++it) {
		if ((*it).second->isType(type_flag)) {
			objects.push_back((*it).second.get());
		}
	}
}

ObjectManager::iterator ObjectManager::begin() {
	return iterator(this, &ObjectManager::alwaysTrue, true);
}

ObjectManager::iterator ObjectManager::end() {
	return iterator(this, &ObjectManager::alwaysTrue, false);
}

ObjectManager::hand_iterator ObjectManager::hand_begin() {
	return hand_iterator(this, &ObjectManager::isTypeHand, true);
}

ObjectManager::hand_iterator ObjectManager::hand_end() {
	return hand_iterator(this, &ObjectManager::isTypeHand, false);
}

ObjectManager::object_iterator ObjectManager::obj_begin() {
	return object_iterator(this, &ObjectManager::isTypeObject, true);
}

ObjectManager::object_iterator ObjectManager::obj_end() {
	return object_iterator(this, &ObjectManager::isTypeObject, false);
}

ObjectManager::robot_iterator ObjectManager::robot_begin() {
	return robot_iterator(this, &ObjectManager::isTypeRobot, true);
}

ObjectManager::robot_iterator ObjectManager::robot_end() {
	return robot_iterator(this, &ObjectManager::isTypeObject, false);
}

void ObjectManager::restoreState(const ObjectsState& state, const std::vector<ObjectBase*>& target_objects) {
	for (size_t i = 0; i < target_objects.size(); i++) {
		std::map<ObjectDescriptor, ObjectsState::ObjectState>::const_iterator s_it
			= state.states_.find(target_objects[i]->descriptor());
		if (s_it != state.states_.end()) {
			target_objects[i]->restoreState((*s_it).second);
		}
	}
}

void ObjectManager::restoreState(const ObjectsState& state, bool do_forwardkinematics) {
	restoreState(state, ObjectBase::Type(ObjectBase::TYPE_ROBOT | ObjectBase::TYPE_HAND | ObjectBase::TYPE_OBJECT),
							 do_forwardkinematics);
}

void ObjectManager::restoreState(const ObjectsState& state, ObjectBase::Type target_type_flag, bool do_forwardkinematics) {
	for (ObjectContainerIte it  = objects_.begin(); it != objects_.end(); ++it) {
		if (!(*it).second->isType(target_type_flag)) continue;
		std::map<ObjectDescriptor, ObjectsState::ObjectState>::const_iterator s_it
			= state.states_.find((*it).first);
		if (s_it != state.states_.end()) {
			(*it).second->restoreState((*s_it).second);
		}
	}

	if (do_forwardkinematics) {
		if (robot_begin() != robot_end()) (*robot_begin())->calcFK();
	}
}

void ObjectManager::storeState(ObjectsState& state, const std::vector<ObjectBase*>& target_objects) const {
	for (size_t i = 0; i < target_objects.size(); i++) {
		target_objects[i]->storeState(state.states_[target_objects[i]->descriptor()]);
	}
}

void ObjectManager::storeState(ObjectsState& state) const {
	storeState(state, ObjectBase::Type(ObjectBase::TYPE_ROBOT | ObjectBase::TYPE_HAND | ObjectBase::TYPE_OBJECT));
}

void ObjectManager::storeState(ObjectsState& state, ObjectBase::Type target_type_flag) const {
	for (ObjectContainerConstIte it = objects_.begin(); it != objects_.end(); ++it) {
		if (!(*it).second->isType(target_type_flag)) continue;
		(*it).second->storeState(state.states_[(*it).first]);
	}
}

void ObjectManager::extractConnectedComponents(std::vector<std::vector<ObjectBase*> >& components, bool include_under_connect) const {
	const int bit_size = curr_descriptor_;
	std::map<ObjectDescriptor, boost::dynamic_bitset<> > connect_state;
	boost::dynamic_bitset<> zero_bit(bit_size);
	for (ObjectContainerConstIte it = objects_.begin(); it != objects_.end(); ++it) {
		ObjectBase* target = (*it).second.get();
		connect_state[target->descriptor()] = boost::dynamic_bitset<>(bit_size);
		connect_state[target->descriptor()].set(target->descriptor());
		for (size_t i = 0; i < target->connections().size(); i++) {
			Connection* con = target->connections()[i];
			if (con->isConnected() || (include_under_connect && con->isUnderconnect())) {
				connect_state[target->descriptor()].set(con->slaveObject()->descriptor());
			}
		}
	}

	for (ObjectContainerConstIte it = objects_.begin(); it != objects_.end(); ++it) {
		for (ObjectContainerConstIte it2 = it; it2 != objects_.end(); ++it2) {
			if (it == it2) continue;
			if ((connect_state[(*it).first] & connect_state[(*it2).first]).any()) {
				connect_state[(*it2).first] |= connect_state[(*it).first];
				connect_state[(*it).first] &= zero_bit;
				break;
			}
		}
	}

	for (ObjectContainerConstIte it = objects_.begin(); it != objects_.end(); ++it) {
		if (connect_state[(*it).first].none()) continue;
		components.push_back(std::vector<ObjectBase*>());
		for (int i = 0; i < bit_size; i++) {
			if (connect_state[(*it).first].test(i)) {
				components.back().push_back(objects_.at(i).get());
			}
		}
	}
}

void ObjectManager::extractRootObjects(std::vector<ObjectBase*>& roots, bool include_under_connect) const {
	std::vector<std::vector<ObjectBase*> > components;
	extractConnectedComponents(components, include_under_connect);

	for (size_t i = 0; i < components.size(); i++) {
		int n_obj = components[i].size();
		std::vector<int> count(n_obj, 0);
		for (int j = 0; j < n_obj; j++) {
			for (size_t k= 0; k < components[i][j]->connections().size(); k++) {
				Connection* con = components[i][j]->connections()[k];
				if (con->isConnected() || (include_under_connect && con->isUnderconnect())) {
					for (int m = 0; m < n_obj; m++) {
						if (components[i][m]->descriptor() == con->slaveObject()->descriptor()) {
							count[m]++;
							break;
						}
					}
				}
			}
		}
		bool has_root = false;
		for (int j = 0; j < n_obj; j++) {
			if (count[j] == 0) {
				roots.push_back(components[i][j]);
				has_root = true;
			}
		}
		if (!has_root) {
			roots.push_back(components[i][0]);
		}
	}
}

void ObjectManager::setUnderConnectState(ObjectsState& state, const ObjectBase* parent_obj, cnoid::Link* parent_link, const ObjectBase* child_obj) {
	ObjectsState::ObjectState& s = state.states_[parent_obj->descriptor()];
	for (size_t i = 0; i < s.descendant_connections.size(); i++) {
		if ((s.descendant_connections[i].slaveObject()->descriptor() == child_obj->descriptor()) &&
				(s.descendant_connections[i].masterLink() == parent_link)) {
			s.descendant_connections[i].setUnderconnect();
			return;
		}
	}
	s.descendant_connections.push_back(Connection());
	Connection& target_con = s.descendant_connections.back();
	target_con.setMasterLink(parent_link);
	target_con.setSlaveObject(const_cast<ObjectBase*>(child_obj));
	target_con.setUnderconnect();
}

void ObjectManager::setConnectState(ObjectsState& state, const ObjectBase* parent_obj, cnoid::Link* parent_link, const ObjectBase* child_obj) {
	ObjectsState::ObjectState& s = state.states_[parent_obj->descriptor()];
	for (size_t i = 0; i < s.descendant_connections.size(); i++) {
		if ((s.descendant_connections[i].slaveObject()->descriptor() == child_obj->descriptor()) &&
				(s.descendant_connections[i].masterLink() == parent_link)) {
			s.descendant_connections[i].setConnected();
			return;
		}
	}
	s.descendant_connections.push_back(Connection());
	Connection& target_con = s.descendant_connections.back();
	target_con.setMasterLink(parent_link);
	target_con.setSlaveObject(const_cast<ObjectBase*>(child_obj));
	target_con.setConnected();
	target_con.relativeRot() = parent_link->R().transpose() * child_obj->bodyItem()->body()->rootLink()->R();
	target_con.relativePos() = parent_link->R().transpose() * (child_obj->bodyItem()->body()->rootLink()->p() - parent_link->p());
}

bool ObjectManager::isTypeRobot(const ObjectBase* object) {
	return object->isRobot();
}

bool ObjectManager::isTypeHand(const ObjectBase* object) {
	return object->isHand();
}

bool ObjectManager::isTypeObject(const ObjectBase* object) {
	return object->isObject();
}

bool ObjectManager::alwaysTrue(const ObjectBase* object) {
	return true;
}

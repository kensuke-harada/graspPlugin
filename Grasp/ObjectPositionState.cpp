#include "ObjectPositionState.h"

#include "ObjectBase.h"

using namespace grasp;

ObjectPositionState::ObjectPositionState() {
}

ObjectPositionState::~ObjectPositionState() {
}

void ObjectPositionState::storeState(const ObjectBase* object, ObjectPositionState& os) {
	const cnoid::BodyPtr& body = object->bodyItem()->body();
	os.T() = body->rootLink()->T();
	os.jointSeq().resize(body->numJoints());
	for (int i = 0; i < body->numJoints(); i++) {
		os.q(i) = body->joint(i)->q();
	}
}

void ObjectPositionState::restoreState(ObjectBase* object, const ObjectPositionState& os) {
	const cnoid::BodyPtr& body = object->bodyItem()->body();
	body->rootLink()->T() = os.T();
	for (int i = 0; i < body->numJoints(); i++) {
		body->joint(i)->q() = os.q(i);
	}
	object->bodyItem()->calcForwardKinematics();
	object->bodyItem()->notifyKinematicStateChange();
}

cnoid::VectorXd& ObjectPositionState::jointSeq() {
	return joint_seq_;
}

const cnoid::VectorXd& ObjectPositionState::jointSeq() const {
	return joint_seq_;
}

double& ObjectPositionState::q(int id) {
	return joint_seq_[id];
}

double ObjectPositionState::q(int id) const {
	return joint_seq_[id];
}

cnoid::Position& ObjectPositionState::T() {
	return root_p_;
}

const cnoid::Position& ObjectPositionState::T() const {
	return root_p_;
}

cnoid::Position::TranslationPart ObjectPositionState::p() {
	return root_p_.translation();
}

cnoid::Position::ConstTranslationPart ObjectPositionState::p() const {
	return root_p_.translation();
}

cnoid::Position::LinearPart ObjectPositionState::R() {
	return root_p_.linear();
}

cnoid::Position::ConstLinearPart ObjectPositionState::R() const {
	return root_p_.linear();
}

ObjectsState::ObjectsState() {
}

ObjectsState::~ObjectsState() {
}

ObjectsState::ObjectState& ObjectsState::objectState(ObjectDescriptor des) {
	return states_[des];
}

bool ObjectsState::hasStateData(ObjectDescriptor des) const {
	return (states_.count(des) > 0);
}

namespace {
	void extra_print(std::ostream& os, const ObjectsState::ExtraData& data) {
		if ((data.type() == typeid(int))) {
			os << " " << boost::get<int>(data) << std::endl;
		}
		if ((data.type() == typeid(double))) {
			os << " " << boost::get<double>(data) << std::endl;
		}
		if ((data.type() == typeid(std::vector<int>))) {
			const std::vector<int>& datas = boost::get<std::vector<int> >(data);
			for (size_t i = 0; i < datas.size(); i++) {
				os << " " << datas[i];
			}
			os << std::endl;
		}
		if ((data.type() == typeid(std::vector<double>))) {
			const std::vector<double>& datas = boost::get<std::vector<double> >(data);
			for (size_t i = 0; i < datas.size(); i++) {
				os << " " << datas[i];
			}
			os << std::endl;
		}
	}
}

std::ostream& grasp::operator<<(std::ostream& os, const grasp::ObjectPositionState& ops) {
	os << "[ObjectPositionState begin]" << std::endl;
	os << " joint angles:" << std::endl;
	os << " ";
	for (size_t i = 0; i < ops.jointSeq().size(); i++) {
		os << " " << ops.q(i);
	}
	os << std::endl;
	os << " position    :" << std::endl;
	os << ops.T().matrix() << std::endl;
	os << "[ObjectPositionState end]" << std::endl;
	return os;
}

std::ostream& grasp::operator<<(std::ostream& os, const grasp::ObjectsState::ObjectState& oss) {
	os << "[ObjectState begin]" << std::endl;
	os << oss.obj_pos_state;
	for (size_t i = 0; i < oss.descendant_connections.size(); i++) {
		os << oss.descendant_connections[i];
	}
	os << " extra state data:" << std::endl;
	for (size_t i = 0; i < oss.extra_state_data.size(); i++) {
		extra_print(os, oss.extra_state_data[i]);
	}
	os << "[ObjectState end]" << std::endl;
	return os;
}

std::ostream& grasp::operator<<(std::ostream& os, const grasp::ObjectsState& oss) {
	os << "[ObjectsState begin]" << std::endl;
	for (std::map<ObjectDescriptor, grasp::ObjectsState::ObjectState>::const_iterator it = oss.states_.begin();
			 it != oss.states_.end(); ++it) {
		os << " Descritpor : " << (*it).first << std::endl;
		os << (*it).second;
	}
	os << "[ObjectsState end]" << std::endl;
	return os;
}

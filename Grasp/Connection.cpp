#include "Connection.h"

#include "ObjectBase.h"

using namespace grasp;

Connection::Connection() :
	slave_object_(NULL),
	master_link_(NULL),
	state_(STATE_UNCONNECTED) {
}

Connection::~Connection() {
}

ObjectBase* const Connection::slaveObject() const {
	return slave_object_;
}

void Connection::setSlaveObject(ObjectBase* slave) {
	slave_object_ = slave;
}

cnoid::Link* const Connection::masterLink() const {
	return master_link_;
}

void Connection::setMasterLink(cnoid::Link* link) {
	master_link_ = link;
}

void Connection::setRelativeT(const cnoid::Position& T) {
	relative_pos_ = T;
}

cnoid::Position& Connection::relativeT() {
	return relative_pos_;
}

const cnoid::Position&  Connection::relativeT() const {
	return relative_pos_;
}

cnoid::Position::LinearPart Connection::relativeRot() {
	return relative_pos_.linear();
}

cnoid::Position::ConstLinearPart Connection::relativeRot() const {
	return relative_pos_.linear();
}

cnoid::Position::TranslationPart Connection::relativePos() {
	return relative_pos_.translation();
}

cnoid::Position::ConstTranslationPart Connection::relativePos() const {
	return relative_pos_.translation();
}

bool Connection::isConnected() const {
	return (state_ == STATE_CONNECTED);
}

bool Connection::isUnconnected() const {
	return (state_ == STATE_UNCONNECTED);
}

bool Connection::isUnderconnect() const {
	return (state_ == STATE_UNDERCONNECT);
}

void Connection::setConnected() {
	state_ = STATE_CONNECTED;
}

void Connection::setUnconnected() {
	state_ = STATE_UNCONNECTED;
}

void Connection::setUnderconnect() {
	state_ = STATE_UNDERCONNECT;
}

void Connection::setState(ConnectionState state) {
	state_ = state;
}

Connection::ConnectionState Connection::state() const {
	return state_;
}

std::ostream& grasp::operator <<(std::ostream& os, const grasp::Connection& con) {
	os << "[Connection begin]" << std::endl;
	os << " slave object descriptor: " << con.slaveObject()->descriptor() << std::endl;;
	os << " master link name       : " << con.masterLink()->name() << std::endl;
	os << " state                  : ";
	if (con.isConnected()) {
		os << "connect";
	} else if (con.isUnconnected()) {
		os << "unconnect";
	} else if (con.isUnderconnect()) {
		os << "under conenct";
	}
	os << std::endl;
	os << " relative position      : " << std::endl;
	os << con.relativeT().matrix() << std::endl;
	os << "[Connection end]" << std::endl;
	return os;
}

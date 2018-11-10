#include "ObjectBase.h"
#include "ColdetPairData.h"
#include "ObjectTraversal.h"

using namespace grasp;

class ObjectBase::FKObjectVisitor :
	public ObjectVisitor {
public:
	void examineObject(ObjectBase* object) {
		object->bodyItem()->body()->calcForwardKinematics();
	}

	void examineConnection(Connection* con) {
		cnoid::Link* target_link = con->slaveObject()->bodyItem()->body()->rootLink();
		target_link->R() = con->masterLink()->R() * con->relativeRot();
		target_link->p() = con->masterLink()->p() + con->masterLink()->R() * con->relativePos();
	}
};

class ConnectionCheckVisitor :
	public ObjectConstVisitor {
public:
	ConnectionCheckVisitor() {
		include_under_connect = false;
		is_connect = false;
	}

	void examineObject(const ObjectBase* object) {
		if (object->descriptor() == target_object->descriptor()) {
			is_connect = true;
		}
	}

	bool isTargetConnection(const Connection* con) {
		return ((con->isConnected() || (include_under_connect && con->isUnderconnect())));
	}

	bool finishFlag() {
		return is_connect;
	}

	const ObjectBase* target_object;
	bool include_under_connect;
	bool is_connect;
};

class DescendantsGetterVisitor :
	public ObjectConstVisitor {
public:
	void exmineConnection(const Connection* con) {
		objects->push_back(con->slaveObject());
	}

	bool isTargetConnection(const Connection* con) {
		return ((con->isConnected() || (include_under_connect && con->isUnderconnect())));
	}

	bool include_under_connect;
	std::vector<ObjectBase*>* objects;
};

ObjectBase::ObjectBase() :
	os(cnoid::MessageView::instance()->cout()),
	bb_safety_size_(cnoid::Vector3(0.005, 0.005, 0.005)),
	fk_visitor_(new FKObjectVisitor()) {
}

ObjectBase::~ObjectBase() {
	clearConnections();
}

const cnoid::BodyItem* ObjectBase::bodyItem() const {
	return body_item_.get();
}

cnoid::BodyItem* ObjectBase::bodyItem() {
	return body_item_.get();
}

bool ObjectBase::isRobot() const {
	return (type_ == TYPE_ROBOT);
}

bool ObjectBase::isHand() const {
	return (type_ == TYPE_HAND);
}

bool ObjectBase::isObject() const {
	return (type_ == TYPE_OBJECT);
}

bool ObjectBase::isType(Type type) const {
	return (type_ & type);
}

ObjectDescriptor ObjectBase::descriptor() const {
	return descriptor_;
}

void ObjectBase::restoreState(const ObjectsState::ObjectState& state) {
	ObjectPositionState::restoreState(this, state.obj_pos_state);
	clearConnections();
	descendant_connections_.resize(state.descendant_connections.size());
	for (int i = 0; i < descendant_connections_.size(); i++) {
		descendant_connections_[i] = new Connection();
		*(descendant_connections_[i]) = state.descendant_connections[i];
	}
}

void ObjectBase::storeState(ObjectsState::ObjectState& state) const {
	ObjectPositionState::storeState(this, state.obj_pos_state);
	state.descendant_connections.clear();
	state.descendant_connections.resize(descendant_connections_.size());
	for (int i = 0; i < state.descendant_connections.size(); i++) {
		 state.descendant_connections[i] = *(descendant_connections_[i]);
	}
}

void ObjectBase::clearConnections() {
	for (size_t i = 0; i < descendant_connections_.size(); i++) {
		if (descendant_connections_[i] != NULL) delete descendant_connections_[i];
	}
	descendant_connections_.clear();
}

Connection* ObjectBase::findOrCreateConnection(ObjectBase* slave, const cnoid::Link* master_link) {
	Connection* con = findConnection(slave, master_link);
	if (con == NULL) {
		con = new Connection();
		descendant_connections_.push_back(con);
	}
	return con;
}

Connection* ObjectBase::findConnection(ObjectBase* slave, const cnoid::Link* master_link) const {
	for (size_t i = 0; i < descendant_connections_.size(); i++) {
		Connection* con = descendant_connections_[i];
		if (con->slaveObject()->descriptor() != slave->descriptor()) continue;
		if ((master_link != NULL) && (con->masterLink() != master_link)) continue;
		return con;
	}
	return NULL;
}

const std::vector<Connection*>& ObjectBase::connections() const {
	return descendant_connections_;
}

void ObjectBase::connect(ObjectBase* slave, cnoid::Link* master_link, Connection::ConnectionState state_connect) {
	cnoid::Position T;
	cnoid::Link* slave_link = slave->bodyItem()->body()->rootLink();
	T.linear() = master_link->R().transpose() * slave_link->R();
	T.translation() = master_link->R().transpose() * (slave_link->p() - master_link->p());
	connect(slave, master_link, T, state_connect);
}

void ObjectBase::connect(ObjectBase* slave, cnoid::Link* master_link,
												 const cnoid::Matrix3& relR, const cnoid::Vector3& relp, Connection::ConnectionState state_connect) {
	cnoid::Position T;
	T.translation() = relp;
	T.linear() = relR;
	connect(slave, master_link, T, state_connect);
}

void ObjectBase::connect(ObjectBase* slave, cnoid::Link* master_link,
												 const cnoid::Position& relT, Connection::ConnectionState state_connect) {
	Connection* con = findOrCreateConnection(slave, master_link);
	con->setSlaveObject(slave);
	con->setMasterLink(master_link);
	con->setRelativeT(relT);
	con->setState(state_connect);
}

void ObjectBase::breadthFirstTraversal(ObjectVisitor& visitor) {
	objectBreadthFirstTraversal(this, visitor);
}

void ObjectBase::breadthFirstTraversal(ObjectConstVisitor& visitor) const {
	objectBreadthFirstTraversal(this, visitor);
}

void ObjectBase::depthFirstTraversal(ObjectVisitor& visitor) {
	objectDepthFirstTraversal(this, visitor);
}

void ObjectBase::depthFirstTraversal(ObjectConstVisitor& visitor) const {
	objectDepthFirstTraversal(this, visitor);
}

void ObjectBase::calcFK(bool calc_recursive) {
	if (!calc_recursive) {
		body_item_->body()->calcForwardKinematics();
		return;
	}

	depthFirstTraversal(*fk_visitor_);
}

bool ObjectBase::isConnected(const ObjectBase* target_obj, bool include_under_connect) const {
	ConnectionCheckVisitor visitor;
	visitor.target_object = target_obj;
	visitor.include_under_connect = include_under_connect;

	breadthFirstTraversal(visitor);

	return visitor.is_connect;
}

void ObjectBase::extractDescendants(std::vector<ObjectBase*>& objects, bool include_under_connect) const {
	DescendantsGetterVisitor visitor;
	visitor.include_under_connect = include_under_connect;
	visitor.objects = &objects;

	breadthFirstTraversal(visitor);
}

void ObjectBase::extractChildren(std::vector<ObjectBase*>& objects, bool include_under_connect) const {
	for (size_t i = 0; i < descendant_connections_.size(); i++) {
		Connection* con = descendant_connections_[i];
		if (con->isConnected() || (include_under_connect && con->isUnderconnect())) {
			objects.push_back(con->slaveObject());
		}
	}
}

void ObjectBase::initialCollisionSelf() {
	self_pairs_.clear();

	for (int i = 0; i < bodyItem()->body()->numLinks(); i++) {
		for (int j = i + 1; j < bodyItem()->body()->numLinks(); j++) {
#ifdef  CNOID_10_11_12_13
			cnoid::ColdetLinkPairPtr temp = new cnoid::ColdetLinkPair(bodyItem()->body()->link(i), bodyItem()->body()->link(j));
#else
			cnoid::ColdetLinkPairPtr temp = boost::make_shared<cnoid::ColdetLinkPair>(bodyItem()->body(), bodyItem()->body()->link(i), bodyItem()->body(), bodyItem()->body()->link(j) );
#endif
			temp->updatePositions();
			int t1, t2;
			double p1[3], p2[3];
			double distance = temp->computeDistance(t1, p1, t2, p2);
			if (distance > 1.0e-04)	self_pairs_.push_back(temp);
		}
	}
}

bool ObjectBase::isColliding(std::string& col_pair_name1, std::string& col_pair_name2) {
	cnoid::ColdetLinkPairPtr collision_pair;
	if (isCollidingTest(self_pairs_, "self collide", collision_pair)) {
		col_pair_name1 = collision_pair->model(0)->name();
		col_pair_name2 = collision_pair->model(0)->name();
		return true;
	}

	return false;
}

bool ObjectBase::isCollidingTest(const ColdetLinkPairVector& target_pairs, const std::string& debug_msg,
																 cnoid::ColdetLinkPairPtr& collision_pair) const {
	for (size_t i = 0; i < target_pairs.size(); i++) {
		target_pairs[i]->updatePositions();
		bool coll = target_pairs[i]->checkCollision();
		if (coll) {
#ifdef DEBUG_MODE
			std::cout << debug_msg << " " << target_pairs[i]->model(0)->name() << " " << target_pairs[i]->model(1)->name() << endl;
#endif
			collision_pair = target_pairs[i];
			return true;
		}
	}
	return false;
}

void ObjectBase::calcSafeBoundingBox(const cnoid::Vector3& safety_size) {
	safe_bb_.clear();
	safe_bb_.resize(bodyItem()->body()->numLinks());
	for (int i = 0; i < bodyItem()->body()->numLinks(); i++) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		safe_bb_[i] = ColdetPairData::getSafeBoundingBox(bodyItem()->body()->link(i)->coldetModel(), safety_size);
#else
		safe_bb_[i] = ColdetPairData::getSafeBoundingBox(bodyItem()->body()->link(i)->collisionShape(), safety_size);
#endif
	}
}

SafeBoundingBox ObjectBase::getSafeBoundingBox(int id) const {
	return safe_bb_[id];
}

void ObjectBase::cloneProc(ObjectBase* obj) const {
	obj->descriptor_ = descriptor_;

	const cnoid::BodyPtr& ori_body = bodyItem()->body();
	const cnoid::BodyPtr& new_body = obj->bodyItem()->body();

	for (size_t i = 0; i < self_pairs_.size(); i++) {
		obj->self_pairs_.push_back(boost::make_shared<grasp::ColdetLinkPair>(new_body, new_body->link(self_pairs_[i]->link(0)->index()),
																																				 new_body, new_body->link(self_pairs_[i]->link(1)->index())));
	}
}

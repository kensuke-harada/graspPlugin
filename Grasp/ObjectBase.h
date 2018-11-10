#ifndef _GRASP_OBJECTBASE_H_
#define _GRASP_OBJECTBASE_H_

#include <vector>
#include <map>
#include <queue>
#include <string>
#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include <cnoid/BodyItem>
#include <cnoid/EigenTypes>
#include <cnoid/MessageView>

#include "exportdef.h"

#include "ColdetLinkPair.h"

#include "Connection.h"
#include "ObjectPositionState.h"

namespace grasp {
	typedef size_t ObjectDescriptor;
	typedef int ObjectID;
	typedef int HandID;

	class RobotBody;
	typedef boost::shared_ptr<RobotBody> RobotBodyPtr;

	class RobotHand;
	typedef boost::shared_ptr<RobotHand> RobotHandPtr;

	class AssemblyObject;
	typedef boost::shared_ptr<AssemblyObject> AssemblyObjectPtr;

	class ObjectBase;
	typedef boost::shared_ptr<ObjectBase> ObjectBasePtr;

	class ObjectManager;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	typedef cnoid::ColdetModelPtr SafeBoundingBox;
#else
	typedef cnoid::SgShapePtr SafeBoundingBox;
#endif

	template<class OBJECT, class CONNECTION>
	class EXCADE_API ObjectVisitorBase {
	public:
		ObjectVisitorBase() {;}
		virtual ~ObjectVisitorBase() {;}
		virtual void examineObject(OBJECT object) {;}
		virtual void finishObject(OBJECT object) {;}
		virtual void examineConnection(CONNECTION con) {;}
		virtual bool isTargetObject(OBJECT object) {return true;}
		virtual bool isTargetConnection(CONNECTION con) {return con->isConnected();}
		virtual bool finishFlag() {return false;}
	};

	typedef ObjectVisitorBase<ObjectBase*, Connection*> ObjectVisitor;
	typedef ObjectVisitorBase<const ObjectBase*, const Connection*> ObjectConstVisitor;

	class EXCADE_API ObjectBase {
		friend class ObjectManager;
	public:
		ObjectBase();
		virtual ~ObjectBase() = 0;

		enum Type {TYPE_ROBOT = 0x1, TYPE_HAND = 0x2, TYPE_OBJECT = 0x4};

		virtual const cnoid::BodyItem* bodyItem() const;
		virtual cnoid::BodyItem* bodyItem();

		bool isRobot() const;
		bool isHand() const;
		bool isObject() const;
		bool isType(Type type) const;

		ObjectDescriptor descriptor() const;

		virtual void restoreState(const ObjectsState::ObjectState& state);
		virtual void storeState(ObjectsState::ObjectState& state) const;

		void clearConnections();
		Connection* findOrCreateConnection(ObjectBase* slave, const cnoid::Link* master_link = NULL);
		Connection* findConnection(ObjectBase* slave, const cnoid::Link* master_link = NULL) const;
		const std::vector<Connection*>& connections() const;

		void connect(ObjectBase* slave, cnoid::Link* master_link,
								 Connection::ConnectionState state_connect = Connection::STATE_CONNECTED);
		void connect(ObjectBase* slave, cnoid::Link* master_link,
								 const cnoid::Matrix3& relR, const cnoid::Vector3& relp,
								 Connection::ConnectionState state_connect = Connection::STATE_CONNECTED);
		void connect(ObjectBase* slave, cnoid::Link* master_link,
								 const cnoid::Position& relT,
								 Connection::ConnectionState state_connect = Connection::STATE_CONNECTED);

		void calcFK(bool calc_recursive = true);
		/** @remarkes this method only search descendant objects */
		bool isConnected(const ObjectBase* target_obj, bool include_under_connect = false) const;
		void extractDescendants(std::vector<ObjectBase*>& objects, bool include_under_connect = false) const;
		void extractChildren(std::vector<ObjectBase*>& objects, bool include_under_connect = false) const;

		virtual void initialCollisionSelf();
		virtual bool isColliding(std::string& col_pair_name1, std::string& col_pair_name2);

		void calcSafeBoundingBox(const cnoid::Vector3& safety_size);
		SafeBoundingBox getSafeBoundingBox(int id) const;

		void breadthFirstTraversal(ObjectVisitor& visitor);
		void breadthFirstTraversal(ObjectConstVisitor& visitor) const;
		void depthFirstTraversal(ObjectVisitor& visitor);
		void depthFirstTraversal(ObjectConstVisitor& visitor) const;

	protected:
		cnoid::BodyItemPtr body_item_;
		std::vector<Connection*> descendant_connections_;
		Type type_;
		ObjectDescriptor descriptor_;
		ColdetLinkPairVector self_pairs_;
		std::vector<SafeBoundingBox> safe_bb_;
		cnoid::Vector3 bb_safety_size_;

		std::ostream& os;

		bool isCollidingTest(const ColdetLinkPairVector& target_pairs, const std::string& debug_msg,
												 cnoid::ColdetLinkPairPtr& collision_pair) const;

		virtual ObjectBasePtr clone(ObjectManager* owner) const = 0;
		void cloneProc(ObjectBase* obj) const;

		class FKObjectVisitor;
		boost::scoped_ptr<FKObjectVisitor> fk_visitor_;
	};
}

#endif /* _GRASP_OBJECTBASE_H_ */

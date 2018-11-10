#ifndef _GRASP_ROBOTBODY_H_
#define _GRASP_ROBOTBODY_H_

#ifdef __GNUC__
#define DEPRECATED(comment) __attribute__ ((deprecated(comment)))
#else
#define DEPRECATED(comment)
#endif

#include <iostream>
#include <vector>
#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/Link>
#include <cnoid/ItemList>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/ValueTree>

#include "ObjectBase.h"
#include "exportdef.h"

namespace grasp {
	class ArmFingers;
	class RobotBody;
	class RobotHand;
	class ObjectManager;

	class EXCADE_API RobotArmFingers {
	public:
		RobotArmFingers(RobotBody* robotbody, const cnoid::Mapping& armSetting);
		RobotArmFingers(RobotBody* robotbody, const cnoid::Mapping& armSetting, const std::vector<cnoid::BodyItemPtr>& hand_bodies);
		virtual ~RobotArmFingers();

		ArmFingers* getArmFingers();

		int handListSize() const;
		bool attachHand(int hand_index);
		void attachHand(const cnoid::BodyItemPtr& bodyitem);
		bool detachHand();
		int getCurrentHandIndex() const;
		bool isAttached() const;
		RobotHand* getRobotHand(int hand_index) const;
		RobotHand* getCurrentRobotHand() const;
		void addRobotHand(RobotHand* hand);
		void connectObject(ObjectBase* slave, cnoid::Link* master);
		void setGraspingState(int state);

	protected:
		std::ostream& os;

	private:
		RobotBody* owner_;
		ArmFingers* armfinger_;
		std::vector<RobotHand*> hand_list_;
		int current_hand_idx_;

		void makeArm(const cnoid::Mapping& armSetting);
		void makeArmProc(const cnoid::Mapping& armSetting, const std::vector<cnoid::BodyItemPtr>& bodyitemlist);
		void searchHandBodyItems(const cnoid::Listing& handname_list, const std::vector<cnoid::BodyItemPtr>& bodyitemlist, std::vector<cnoid::BodyItemPtr>& hand_bodyitems) const;
		void clearHandList();
	};

	class RobotBody;
	typedef boost::shared_ptr<RobotBody> RobotBodyPtr;

	class EXCADE_API RobotBody :
		public ObjectBase {
		friend class ObjectManager;
	public:
		RobotBody(cnoid::BodyItemPtr rootBodyItem, bool load_arms = true) DEPRECATED();
		virtual ~RobotBody();

		struct BodyItemLink {
			cnoid::BodyItemPtr bodyitem;
			cnoid::Link* link;
		};

		typedef std::vector<BodyItemLink> IdToBodyItemLink;

		typedef std::vector<cnoid::BodyItemPtr>::iterator BodyItemIterator;

		cnoid::BodyItemPtr getRootBodyItem() const;
		cnoid::BodyItemPtr getBodyItem(int arm_id, int hand_index) const;

		int armSize() const;
		int handListSize(int arm_id) const;
		bool attachHand(int arm_id, int hand_index);
		bool detachHand(int arm_id);
		bool isAttached(int arm_id) const;
		int getCurrentHandID(int arm_id) const;
		RobotHand* getCurrentHand(int arm_id) const;
		RobotHand* getHand(int arm_id, int hand_index) const;
		RobotArmFingers* getRobotArmFingers(int arm_id) const;

		ArmFingers* getArmFingers(int arm_id) const;

		void obtainBodyAndHandsLink(std::vector<cnoid::Link*>& body_links, std::vector<std::vector<cnoid::Link*> >& hand_links) const;

		cnoid::Link* rootLink() const;
		cnoid::Link* link(int index) const;
		cnoid::Link* link(const std::string& name) const;
		cnoid::Link* joint(int index) const;
		int numJoints() const;
		int numLinks() const;

		const std::vector<cnoid::BodyItemPtr>& getHandBodyItems() const;
		BodyItemIterator handbody_begin();
		BodyItemIterator handbody_end();
		int allHandListSize() const;

		void updateJointList();

		void calcForwardKinematics();
		void notifyKinematicStateChange();

		ObjectManager* getObjectManager() const;

		void restoreState(const ObjectsState::ObjectState& state);
		void storeState(ObjectsState::ObjectState& state) const;
		int& stateGraspObjectID(ObjectsState& state, int arm_id);

		bool isColliding(std::string& col_pair_name1, std::string& col_pair_name2);

	private:
		RobotBody() {;}
		RobotBody(cnoid::BodyItemPtr rootBodyItem, ObjectManager* obj_manager, bool load_arms = true);
				
		ObjectManager* owner_;
		std::vector<RobotArmFingers*> armsList_;
		cnoid::BodyItemPtr rootBodyItem_;
		std::vector<cnoid::BodyItemPtr> handBodyItems_;

		IdToBodyItemLink jointid_to_link_;
		IdToBodyItemLink linkid_to_link_;

		void makeArms();
		void clearArms();

		ObjectBasePtr clone(ObjectManager* owner) const;
		void copyHandsbind(RobotBody* org);
	};
}

#endif

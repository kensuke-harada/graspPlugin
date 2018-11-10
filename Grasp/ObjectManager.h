#ifndef _GRASP_OBJECTMANAGER_H_
#define _GRASP_OBJECTMANAGER_H_

#include <map>
#include <vector>
#include <queue>

#include <boost/bind.hpp>
#include <boost/dynamic_bitset.hpp>

#include <cnoid/BodyItem>

#include "exportdef.h"
#include "ObjectBase.h"

namespace grasp {
	template <class Type>
		class ObjectIterator;

	class ObjectManager;

	class EXCADE_API ObjectManager {
		template<class> friend class ObjectIterator;
	public:
		ObjectManager();
		ObjectManager* clone() const;
		virtual ~ObjectManager();

		typedef ObjectIterator<ObjectBase> iterator;
		typedef ObjectIterator<RobotHand> hand_iterator;
		typedef ObjectIterator<AssemblyObject> object_iterator;
		typedef ObjectIterator<RobotBody> robot_iterator;
		typedef std::map<ObjectDescriptor, ObjectBasePtr> ObjectContainer;
		typedef ObjectContainer::iterator ObjectContainerIte;
		typedef ObjectContainer::const_iterator ObjectContainerConstIte;

		RobotBodyPtr getOrCreateRobot(const cnoid::BodyItemPtr& bodyitem);
		RobotBodyPtr createRobot(const cnoid::BodyItemPtr& bodyitem);
		RobotHandPtr getOrCreateHand(const cnoid::BodyItemPtr& bodyitem);
		RobotHandPtr createHand(const cnoid::BodyItemPtr& bodyitem);
		AssemblyObjectPtr getOrCreateObject(const cnoid::BodyItemPtr& bodyitem);
		AssemblyObjectPtr createObject(const cnoid::BodyItemPtr& bodyitem);
		bool detachObject(ObjectBasePtr object);
		bool detachObject(const cnoid::BodyItemPtr& bodyitem);
		bool detachObject(ObjectDescriptor descriptor);

		ObjectBasePtr getObject(ObjectDescriptor descriptor) const;
		ObjectBasePtr hasObject(const cnoid::BodyItemPtr& bodyitem) const;

		AssemblyObject* getAssemblyObject(int object_id) const;
		RobotHand* getHand(int hand_id) const;

		void clear();
		void clearObjects();
		void clearHands();

		void extractRobotHands(std::vector<RobotHandPtr>& hands) const;
		void extractRobotHands(std::vector<RobotHand*>& hands) const;
		void extractObjects(std::vector<AssemblyObjectPtr>& objects) const;
		void extractObjects(std::vector<AssemblyObject*>& objects) const;
		void extract(std::vector<ObjectBase*>& objects, ObjectBase::Type type_flag) const;

		iterator begin();
		iterator end();

		hand_iterator hand_begin();
		hand_iterator hand_end();

		object_iterator obj_begin();
		object_iterator obj_end();

		robot_iterator robot_begin();
		robot_iterator robot_end();

		void restoreState(const ObjectsState& state, const std::vector<ObjectBase*>& target_objects);
		void restoreState(const ObjectsState& state, bool do_forwardkinematics = true);
		void restoreState(const ObjectsState& state, ObjectBase::Type target_type_flag, bool do_forwardkinematics = true);
		void storeState(ObjectsState& state, const std::vector<ObjectBase*>& target_objects) const;
		void storeState(ObjectsState& state) const;
		void storeState(ObjectsState& state, ObjectBase::Type target_type_flag) const;

		void extractConnectedComponents(std::vector<std::vector<ObjectBase*> >& components, bool include_under_connect = false) const;
		void extractRootObjects(std::vector<ObjectBase*>& roots, bool include_under_connect = false) const;

		static void setUnderConnectState(ObjectsState& state, const ObjectBase* parent_obj, cnoid::Link* parent_link, const ObjectBase* child_obj);
		static void setConnectState(ObjectsState& state, const ObjectBase* pareant_obj, cnoid::Link* parent_link, const ObjectBase* child_obj);

		static bool isTypeRobot(const ObjectBase* object);
		static bool isTypeHand(const ObjectBase* object);
		static bool isTypeObject(const ObjectBase* object);
		static bool alwaysTrue(const ObjectBase* object);

	protected:
		ObjectContainer objects_;
		ObjectDescriptor curr_descriptor_;
		int curr_object_id_;
		int curr_hand_id_;
		std::map<int, ObjectDescriptor> object_id_descriptor_map_;
		std::map<int, ObjectDescriptor> hand_id_descriptor_map_;
	};
}

#include "ObjectIterator.h"

#endif /* _GRASP_OBJECTMANAGER_H_ */

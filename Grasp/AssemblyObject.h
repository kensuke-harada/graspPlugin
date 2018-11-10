#ifndef _GRASP_ASSEMBLYOBJECT_H_
#define _GRASP_ASSEMBLYOBJECT_H_

#include <cnoid/BodyItem>

#include "PlanBase.h"
#include "ObjectBase.h"
#include "exportdef.h"

namespace grasp {
	class ObjectManager;

	class EXCADE_API AssemblyObject :
		public TargetObject,
		public ObjectBase {
	public:
		friend class ObjectManager;

		~AssemblyObject();

		ObjectID getID() const;
		void assemble(AssemblyObject* slave, cnoid::Link* master_link);

		ObjectBasePtr clone(ObjectManager* owner) const;

	private:
		AssemblyObject(const cnoid::BodyItemPtr& bodyItem, ObjectManager* owner);

		ObjectID id_;

		ObjectManager* owner_;
		};
}

#endif

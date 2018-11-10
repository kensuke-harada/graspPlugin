#include "AssemblyObject.h"

#include "ObjectManager.h"

using namespace grasp;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
using namespace cnoid;
#endif

AssemblyObject::AssemblyObject(const cnoid::BodyItemPtr& bodyItem, ObjectManager* owner) :
	TargetObject(bodyItem),
	owner_(owner) {
	body_item_ = bodyItem;
	type_ = TYPE_OBJECT;
	calcSafeBoundingBox(bb_safety_size_);
}

AssemblyObject::~AssemblyObject() {
}

int AssemblyObject::getID() const {
	return id_;
}

void AssemblyObject::assemble(AssemblyObject* slave, cnoid::Link* master_link) {
	connect(slave, master_link);
	slave->connect(this, slave->bodyItem()->body()->rootLink());
}

ObjectBasePtr AssemblyObject::clone(ObjectManager* owner) const {
	cnoid::BodyItemPtr new_body = new cnoid::BodyItem(*(bodyItem()));
	AssemblyObjectPtr ret = AssemblyObjectPtr(new AssemblyObject(new_body, owner));
	cloneProc(ret.get());
	ret->id_ = id_;
	return ret;
}

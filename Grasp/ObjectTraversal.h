#ifndef _GRASP_OBJECTTRAVERSAL_H_
#define _GRASP_OBJECTTRAVERSAL_H_

#include <map>

#include "ObjectBase.h"

namespace grasp {
	void objectBreadthFirstTraversal(ObjectBase* start_object, ObjectVisitor& visitor);
	void objectBreadthFirstTraversal(const ObjectBase* start_object, ObjectConstVisitor& visitor);
	void objectDepthFirstTraversal(ObjectBase* start_object, ObjectVisitor& visitor);
	void objectDepthFirstTraversal(const ObjectBase* start_object, ObjectConstVisitor& visitor);
};

#endif /* _GRASP_OBJECTTRAVERSAL_H_ */

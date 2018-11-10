#ifndef COLDETMODELGETTER_H
#define COLDETMODELGETTER_H

#include <iostream>
#include <map>
#include <list>

#include <cnoid/ItemTreeView>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/ColdetModel>
#include <cnoid/Link>
#include <boost/bind.hpp>

#include "exportdef.h"

namespace grasp {
	namespace ColdetModelGetter {
		EXCADE_API const cnoid::ColdetModelPtr&  get(cnoid::Link* link);
		EXCADE_API void update();
	}
}

#endif

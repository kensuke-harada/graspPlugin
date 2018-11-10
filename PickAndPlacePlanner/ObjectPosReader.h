#ifndef OBJECTPOSREADER_H
#define OBJECTPOSREADER_H

#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string>
#include <fstream>
#include <iostream>

#if !(defined(WIN32) || defined(_WIN32))
#include <dirent.h>
#endif
#include <sys/types.h>
#include <sys/stat.h>
#if !(defined(WIN32) || defined(_WIN32))
#include <sys/resource.h>
#endif

#include <cnoid/BodyItem>
#include <cnoid/WorldItem>
#include <cnoid/MessageView>
#include <cnoid/ItemTreeView>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/JointPath>
#include <boost/filesystem.hpp>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/ModelNodeSet>
#include <cnoid/ColdetLinkPair>
#else
#include "../Grasp/ColdetLinkPair.h"
#endif

#include "../Grasp/VectorMath.h"
#include "../Grasp/PlanBase.h"

#include "exportdef.h"

namespace grasp{
namespace PickAndPlacePlanner{

class ObjectPosReader {
public:
	ObjectPosReader();
	virtual ~ObjectPosReader();

	bool doReadFromFile(std::string dataPath, std::string calibFile, int num=0);

private:
	void setTransMatrix(std::string calibFile);

	cnoid::Vector3 t1, t2, t3;
	cnoid::Matrix3 R1, R2, R3;
	bool isMulti;
};

}
}

#endif

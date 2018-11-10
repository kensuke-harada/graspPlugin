#ifndef _INTEROBJECT_H
#define _INTEROBJECT_H

#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <cnoid/Item>
#include <cnoid/BodyItem>

#include "exportdef.h"

#include "ColdetLinkPair.h"

class EXCADE_API InterObject{
	public:
	virtual ~InterObject() {;}

	cnoid::BodyItemPtr slaveItem;
	cnoid::Link *master;
	cnoid::Vector3 relativePos;
	cnoid::Matrix3 relativeRot;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	std::vector<cnoid::ColdetLinkPairPtr> slaveEnvPairs;
#else
	std::vector<grasp::ColdetLinkPairPtr> slaveEnvPairs;
#endif
	int state;
	int type;
	
	enum InterObjectType{ ANIMATION, COLLISION, ROBOT, GRASPED_OBJECT};
	
	virtual void setInterObject();
	virtual void outputRelativePosition();
	virtual void initialCollision();
	virtual bool isColliding();
	
};

#endif

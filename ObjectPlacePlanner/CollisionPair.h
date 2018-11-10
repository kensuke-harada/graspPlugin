// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#ifndef COLLISIONPAIR_H
#define COLLISIONPAIR_H

#include <cnoid/MessageView>
#include <cnoid/BodyItem>
#include <boost/make_shared.hpp>
#include "../Grasp/PlanBase.h"

namespace grasp{
namespace PickAndPlacePlanner{

class CollisionPair
{

public :
		CollisionPair();
		~CollisionPair();

		void setCollisionRob();
		void setCollisionSelf();
		void setCollisionObj();
		std::ostream& os;

		bool isColliding();
		void getColPoints(std::vector<cnoid::Vector3>& colpoints);
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		std::vector<cnoid::ColdetLinkPairPtr> colPairRob, colPairSelf, colPairObj;
#else
		std::vector<grasp::ColdetLinkPairPtr> colPairRob, colPairSelf, colPairObj;
#endif
		std::vector<std::pair<cnoid::BodyItemPtr, PointCloudEnv*> > colPairObjPointCloud;
protected :
};

}
}

#endif

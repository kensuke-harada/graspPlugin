// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#ifndef PLACEPLANNER_H
#define PLACEPLANNER_H

//#define EXPO_DEMO
//#define EDGE_FACE_CONTACT

#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string>
#include <fstream>
#include <iostream>

#include <algorithm>
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
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/JointPath>
#include <boost/filesystem.hpp>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/ModelNodeSet>
#include <cnoid/ColdetLinkPair>
#else
#include "../Grasp/ColdetLinkPair.h"
#include "../Grasp/ColdetConverter.h"
#endif

#include "CollisionPair.h"
#include "../Grasp/VectorMath.h"
#include "../Grasp/Arm.h"
#include "../Grasp/PlanBase.h"
#include "../Grasp/ConvexAnalysis.h"
#include "../GeometryHandler/ClusterParameter.h"
#include "../GeometryHandler/GeometryHandle.h"
#include "exportdef.h"

#include <cnoid/ExecutablePath>
#define CHOREONOID_TOP_PATH cnoid::executableTopDirectory() + string("/")

namespace grasp{
namespace PickAndPlacePlanner{



struct putPosition
{
		std::vector<int> IndexSet;
		int assigned, Index;
};

struct rotatedBBox{
		cnoid::Matrix3 R;
		double perpendicular;
		double base_area;
		static bool Greater(const rotatedBBox& l,const rotatedBBox& r){return l.base_area > r.base_area;}
};

class EXCADE_API PlacePlanner
{

public :
		static PlacePlanner* instance();
		PlacePlanner();
		~PlacePlanner();

		void calcObjPosFaceFace(const cnoid::Vector3& pos, ClusterParameter& env, ClusterParameter& obj, std::vector<cnoid::Vector3>& Po_put, std::vector<cnoid::Matrix3>& Ro_put, bool useClusterCenter=true);
		void calcObjPosEdgeFace(const cnoid::Vector3& pos, ClusterParameter& env, ClusterParameter& obj, std::vector<cnoid::Vector3>& Po_put, std::vector<cnoid::Matrix3>& Ro_put);
		void calcObjPosFaceBoundingBox(const cnoid::Vector3& pos, ClusterParameter& env,std::vector<cnoid::Vector3>& Po_put, std::vector<cnoid::Matrix3>& Ro_put);
		void searchObjPosFaceFace(ClusterParameter& env, ClusterParameter& obj, std::vector<cnoid::Vector3>& Po_put, std::vector<cnoid::Matrix3>& Ro_put, int grid_row, int max_candidate);
		void searchObjPosFaceBoundingBox(const cnoid::Vector3& pos, ClusterParameter& env, std::vector<cnoid::Vector3>& Po_put, std::vector<cnoid::Matrix3>& Ro_put, int grid_row, int max_candidate);
		bool searchPutPos(cnoid::Vector3& pos, std::vector<cnoid::Vector3>& Po_put, std::vector<cnoid::Matrix3>& Ro_put);
		bool calcPutPos(cnoid::Vector3& pressPos, const cnoid::Matrix3& pressOri, std::vector<cnoid::Vector3>& Po_put, std::vector<cnoid::Matrix3>& Ro_put, bool useClusterCenter=true);
		bool readPutPos(std::vector<cnoid::Vector3>& Po_put, std::vector<cnoid::Matrix3>& Ro_put);
		bool writePutPos(const std::vector<cnoid::Vector3>& Po_put, const std::vector<cnoid::Matrix3>& Ro_put,bool is_append = true);
		bool objEnvCollision(const cnoid::Vector3& P, const cnoid::Matrix3& R);
		void setObjectPose(const cnoid::Vector3& P, const cnoid::Matrix3& R);
		void calcIntermediatePutPos(std::vector<cnoid::Vector3>& Po_put, std::vector<cnoid::Matrix3>& Ro_put, std::vector<cnoid::Vector3>& Po_tmp, std::vector<cnoid::Matrix3>& Ro_tmp);
		bool isGravityStable(ClusterParameter& obj, const cnoid::Vector3& Po, const cnoid::Matrix3& Ro, ClusterParameter& env, const cnoid::Vector3& Pe, const cnoid::Matrix3& Re);
		bool includeClusterBoundary(ClusterParameter& c1, const cnoid::Vector3& p1, const cnoid::Matrix3& R1,ClusterParameter& c2, const cnoid::Vector3& p2, const cnoid::Matrix3& R2);
		bool includeClusterBoundary(cnoid::Vector3& p, ClusterParameter& c);
		bool include2DBBox(ClusterParameter& c1, ClusterParameter& c2);

		std::ostream& os;
		putPosition putPos;
		CollisionPair *cp;
		ParameterFileData *pf;
		bool clusterIncludeTest, stabilityTest, collisionTest;
		double colTestMargin;
protected:
		bool lineIntersectToCluster(const cnoid::Vector3& p, const cnoid::Vector3& n, cnoid::Vector3& p_out, const std::vector<cnoid::Vector3>& Pset, bool ccw);
		int calcOuterBoundary(std::vector<std::vector<cnoid::Vector3> >& boundary);
		void calcCommonArea(const std::vector<cnoid::Vector3>& s1, const std::vector<cnoid::Vector3>& s2, std::list<cnoid::Vector3>& boundary);
		void excludeHole(const std::vector<cnoid::Vector3>& hole, const std::vector<cnoid::Vector3>& outer, std::list<cnoid::Vector3>& boundary);
		void calcPlanarConvexHull(std::list<cnoid::Vector3>& boundary, std::vector<cnoid::Vector3>& convexhull);

		void obtainSpiralSearchPath(int row, std::vector<int>& x, std::vector<int>& y) const;
};

}
}

#endif

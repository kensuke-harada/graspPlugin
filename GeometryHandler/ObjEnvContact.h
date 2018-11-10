// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef _OBJENVCONTACT_H
#define _OBJENVCONTACT_H

#include <stdio.h>
#include <fstream>

#include <cnoid/ItemTreeView>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
//#include <cnoid/PoseSeqItem>

#include "GeometryHandle.h"
#include "ObjectShape.h"
#include "../Grasp/ConvexAnalysis.h"
#include "../Grasp/VectorMath.h"

namespace grasp{

	class ObjEnvContact
	{

	public :

		ObjEnvContact(){};
		~ObjEnvContact(){};

		static ObjEnvContact* instance(){
			static ObjEnvContact* instance = new ObjEnvContact();
			return instance;
		}

		void initial();
		void calcPlaneClusters(ObjectShape& object);
		int calcTargetCluster(ObjectShape& object);
		void calcTableLegCluster(ObjectShape& obj);
		void calcParentList(ObjectShape& object, std::vector<int>& o);
		void calcClusterConvexHull(ObjectShape& object);
		void calcClusterConvexity(ObjectShape& object);
		void writeResult(ObjectShape& object);

		ObjectShape object;

	protected :
		int  calcOuterBoundary(std::vector<std::vector<cnoid::Vector3> >& boundary);
		bool lineIntersectToCluster(const cnoid::Vector3& p, const cnoid::Vector3& n, cnoid::Vector3& p_out, const std::vector<cnoid::Vector3>& Pset, bool ccw);
	};

}

#endif

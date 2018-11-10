#ifndef _FINDPARALLELPLANE_H
#define _FINDPARALLELPLANE_H

#include <stdio.h>
#include <fstream>
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/RootItem>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/
//#include <cnoid/PoseSeqItem>	/* modified by qtconv.rb 0th rule*/

#include "GeometryHandle.h"
#include "ObjectShape.h"

#include "EnCalculator.h"
#include "../Grasp/GraspController.h"
#include "../Grasp/ForceClosureTest.h"
#include "../Grasp/VectorMath.h"




namespace grasp{

  class FindParallelPlane
  {

	public :

               FindParallelPlane(){
		 PlaneParam = 0.0;
		 clusterSeedSize=2;
		 counter=0;
		 counter_sub=0;
		 intention=0;
		 objPressPos=cnoid::Vector3::Zero();
	};
	~FindParallelPlane(){};

	static FindParallelPlane* instance(){
		static FindParallelPlane* instance = new FindParallelPlane();
		return instance;
	}

	void initial();
	void calcClusters(ObjectShape& object);
	void calcClusters2(ObjectShape& object);
	void precalcParallelPlane(ObjectShape& object);
	bool calcParallelPlane(ObjectShape& object, int i);
	void writeResults(ObjectShape& object);
	void writeResults2(std::vector<ClusterImpl>& clusters);
	bool findTargetTriangle( cnoid::Link *targetLink, cnoid::Vector3 &pressPos, cnoid::BodyItemPtr pBody);

	double PlaneParam;
	std::vector<int> idCluster, idCluster_sub;
	int counter, counter_sub;
	int intention;
	int clusterSeedSize;
	std::vector<int> parentList2;
	std::vector<cnoid::Vector3> clusterNormal;
	std::vector<ClusterImpl> clusters_out;
	ObjectShape* object_out;
	cnoid::Vector3 objPressPos, objPressNormal;
	std::string objPressName;

	protected :
};

}

#endif

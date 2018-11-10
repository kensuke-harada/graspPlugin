#ifndef  __MatchClusters_H__
#define  __MatchClusters_H__

#include "GeometryHandle.h"
#include "ObjectShape.h"

namespace grasp {

class MatchClusters {
	
	public:
		static MatchClusters* instance(MatchClusters *mc=NULL) {
			static MatchClusters* instance = (mc) ? mc : new MatchClusters();
			if(mc) instance = mc;
			return instance;
		}
		void quadricalMatchClusters(ObjectShape& object1, ObjectShape& object2, ObjectShape& outObject);
		void quadricalTransform(ClusterImpl& c1, ClusterImpl& c2, ObjectShape& object1, ObjectShape& object2, cnoid::Matrix3 rot1, cnoid::Matrix3 rot2);
		double matchingErrorClusters(ClusterImpl& c1, ClusterImpl& c2, cnoid::Matrix3 rot1, cnoid::Matrix3 rot2);
		void calcRotation(std::vector<ClusterImpl*> c1, std::vector<ClusterImpl*> c2, std::vector<cnoid::Matrix3>& rot1);
	
		std::vector<ObjectShape*> objectList;
};


}


#endif
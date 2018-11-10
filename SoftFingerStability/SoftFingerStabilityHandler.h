/**
c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _SOFTFINGERSTABILITY_SOFTFINGERSTABILITYHANDLER_H_
#define _SOFTFINGERSTABILITY_SOFTFINGERSTABILITYHANDLER_H_

#include <stdlib.h>

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/
#include "../Grasp/PlanBase.h"
#include "../Grasp/GraspController.h"
#include "../Grasp/DrawUtility.h"
#include "../GeometryHandler/GeometryHandle.h"
#include "../GeometryHandler/EnCalculator.h"
#include <fstream>

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include "../Grasp/ColdetLinkPair.h"
#endif

#include "exportdef.h"

namespace grasp{
	class ContactRegionForDraw : public EnCalculator {
	public:
		void calcForDraw(const cnoid::Vector3& p_obj, const cnoid::Vector3& p_fing, const cnoid::Vector3& n_fing, const std::vector<std::vector<cnoid::Vector3> >& boundary_fing, double fmax);
		void getTrianglesSplittedNoOverlap(std::vector<const Triangle*>& triangles) const;

		ContactTriangleVec contact_tlist;
	private:
		std::vector<const ObjectShape*> finger_shapes;
		
	};


	class EXCADE_API SoftFingerStabilityHandler
	{
	public :
		SoftFingerStabilityHandler();
		~SoftFingerStabilityHandler();

		static SoftFingerStabilityHandler* instance(SoftFingerStabilityHandler *sh=NULL);

		void initialize();

		double calcStability(int nContact,cnoid::Vector3 objPos[], cnoid::Vector3 objN[]);
		double calcStability(int nContact,cnoid::Vector3 objPos[], cnoid::Vector3 objN[],
			std::vector<double>& en,std::vector<double>& area,std::vector<cnoid::Vector3>& fingPos,std::vector<cnoid::Vector3>& fingN);
		
		void showContactCluster(int index,bool is_show_obj,bool is_show_fing);
		void showContactClusterObject(int& tidx) const;
		void showContactClusterFinger(int& tidx) const;
		void displayClusterData();
private:
		void getFingerNormal(const ObjectShape& finger, const cnoid::Vector3& p_fing, const cnoid::Vector3& p_obj, const cnoid::Link* link_fing, const cnoid::Link* link_obj, cnoid::Vector3& n_fing, int& tid) const;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		void calcContactPoint(cnoid::ColdetLinkPairPtr cPair, cnoid::Vector3 &Po,cnoid::Vector3 &Pf, cnoid::Vector3 &objN, cnoid::Vector3 &fingN, int &objTid, int &fingTid) const;
		ObjectShape* createObjectShape(cnoid::ColdetModelPtr c);
#else
		void calcContactPoint(grasp::ColdetLinkPairPtr cPair, cnoid::Vector3 &Po,cnoid::Vector3 &Pf, cnoid::Vector3 &objN, cnoid::Vector3 &fingN, int &objTid, int &fingTid) const;
		ObjectShape* createObjectShape(cnoid::SgNode* c);
#endif	
		PlanBase* tc;
		std::vector<ObjectShape*> finger_shape;
		ObjectShape* obj_shape;
	};
}

#endif /* _SOFTFINGERSTABILITY_SOFTFINGERSTABILITYHANDLER_H_ */

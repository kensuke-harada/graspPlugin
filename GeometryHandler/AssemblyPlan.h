#ifndef _ASSEMBLYPLAN_H
#define _ASSEMBLYPLAN_H

#include <stdio.h>
#include <fstream>
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/RootItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
//#include <cnoid/PoseSeqItem>	/* modified by qtconv.rb 0th rule*/  

//#include <cnoid/SceneView>	/* modified by qtconv.rb 0th rule*/  
//#include <cnoid/SceneBody>	/* modified by qtconv.rb 0th rule*/  

#ifdef CNOID_ENABLE_OSG
#include <cnoid/OSGSceneView>
#include <cnoid/OSGSceneBody>
#endif

#include "GeometryHandle.h"
#include "ObjectShape.h"
#include "../Grasp/GraspController.h"
#include "../Grasp/VectorMath.h"

#define NUM_OBJ 4

namespace grasp{

  class AssemblyPlan
  {
	public :
	
               AssemblyPlan(){};
	~AssemblyPlan(){};
		
	static AssemblyPlan* instance(){
		static AssemblyPlan* instance = new AssemblyPlan();
		return instance;
	}

	void initial();
	void calcPlaneClusters(ObjectShape& object);
	void showContactPointLocus();
	void readResult_camera();
	int calcClusterId(double time, double x, double y, double z, ObjectShape& object);
	double includedIn(Triangle& t, const cnoid::Vector3 &p);
	int checkId(std::vector<std::vector<int> > idListSet[],std::vector<std::vector<int> > idListNode[], int tm, int nObj);
	int edgeArrayNum(int i, int j, int N);
	//void generateGraph(std::vector<std::vector<int> > idListSet[], std::vector<double>& timeList, std::vector<ObjectShape*> &object, int nObj);
	void generateGraph();

	double DisplaySize;// = 30.0;
	std::vector<int> tme, tme2[NUM_OBJ];
	std::vector<double> px, py, pz, fx, fy, fz;
	std::vector<double> px2[NUM_OBJ], py2[NUM_OBJ], pz2[NUM_OBJ], fx2[NUM_OBJ], fy2[NUM_OBJ], fz2[NUM_OBJ];
	std::vector<cnoid::Vector3> objPosition;

	std::vector<int> idList, idList_old;
	std::vector<int> idList2[NUM_OBJ], idList2_old[NUM_OBJ];
	
	std::vector<std::vector<int> > idListSet;
	std::vector<std::vector<int> > idListSet2[NUM_OBJ];
	std::vector<double> timeList;

	std::vector<ObjectShape*> object2; //(4);
	std::vector<std::string> objectNames;

	protected :
};

}

#endif

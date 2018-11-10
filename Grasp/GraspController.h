/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _GraspController_H
#define _GraspController_H

#include <stdlib.h>
#include <time.h>

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include "PlanBase.h"
#include "VectorMath.h"
#include "ColdetLinkPair.h"

#include "exportdef.h"

namespace grasp{

//class EXCADE_API PlanBase;
	


class EXCADE_API GraspController 
{
	
public :
	GraspController();
	~GraspController();

    static GraspController* instance(GraspController *gc=NULL);

	// interface functions of planning 
	virtual bool initial(TargetObject* targetObject, ArmFingers* targetArmFinger);
	virtual bool doGraspPlanning();
	virtual void saveGraspPattern();
	virtual void saveGraspPattern(int rotDir, double angleStep, int transDir, double transLeng, bool y_rot);
	virtual bool loadAndSelectGraspPattern( const std::vector<cnoid::Vector3>offsets = std::vector<cnoid::Vector3>() );
	virtual void doDisplayGRCPosition();
	virtual void closeFingers();

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	static double calcContactPoint(cnoid::ColdetLinkPairPtr cPair, cnoid::Vector3 &Po, cnoid::Vector3 &Pf, cnoid::Vector3 &objN2);
	static double getContactArea(cnoid::ColdetLinkPairPtr cPair, cnoid::Vector3 &Po, cnoid::Vector3 &Pf, cnoid::Vector3 &objN2);
#else
	static double calcContactPoint(grasp::ColdetLinkPairPtr cPair, cnoid::Vector3 &Po, cnoid::Vector3 &Pf, cnoid::Vector3 &objN2);
	static double getContactArea(grasp::ColdetLinkPairPtr cPair, cnoid::Vector3 &Po, cnoid::Vector3 &Pf, cnoid::Vector3 &objN2);
#endif

	//==Object==
	//TargetObject* targetObject;
	
	//==Robot==
	//ArmFingers* targetArmFinger;
	
	//==Env==
	
	//bool flush();

	PlanBase* tc;
	std::vector<MotionState>graspPranningReultList;

protected :

	std::ostream& os;

	// Thease function will be deleted, Please call PlanBase function directory.
	////////////////////////////////////////////////////////////////////////////
	cnoid::Vector3 objVisPos() { return  grasp::PlanBase::instance()->objVisPos();  }
	cnoid::Matrix3 objVisRot() { return grasp::PlanBase::instance()->objVisRot();  }
	//void setObjPos(const cnoid::Vector3& P, const cnoid::Matrix3 R);
	cnoid::Link* object() { return grasp::PlanBase::instance()->object(); }
	
	cnoid::Link* palm() { return grasp::PlanBase::instance()->palm(); }
	cnoid::Link* base() { return grasp::PlanBase::instance()->base(); }
	//ArmPtr arm(){ return targetArmFinger->arm; }

	FingerPtr fingers(int i) { return grasp::PlanBase::instance()->fingers(i); }
	int nFing(){ return grasp::PlanBase::instance()->nFing(); };

	cnoid::LinkTraverse* handJoint()	{ return grasp::PlanBase::instance()->handJoint(); }
	int nHandLink() {return grasp::PlanBase::instance()->nHandLink();}

	cnoid::BodyItemPtr bodyItemRobot(){ return grasp::PlanBase::instance()->bodyItemRobot(); } 
	std::string  bodyItemRobotPath() {return  grasp::PlanBase::instance()->bodyItemRobotPath(); }
        ////////////////////////////////////////////////////////////////////////////

	//calc rotation for searching posture
	cnoid::Matrix3 calcObjRot(const cnoid::Matrix3 &a, int n);
	cnoid::Matrix3 calcHandRot(cnoid::Matrix3 &a1, int n1);
	
	// planning functions
	double returnRefMotion(bool cs, std::string refMotion_);
	double returnRefMotion(bool cs, PrehensionPtr prehension);
	double getPalmPosture();
	bool sampleFinalPos(int number=100);
	
	//==readMotionFile==
	void readMotionFile(const char argv[]);
	void parseParameters( std::istream &fp);
	void settingPrehension(const PrehensionPtr prehension);
	void settingPrehension(const PrehensionParameter& param);

	int graspingState;
	cnoid::Vector3 objectPalmPos;
	cnoid::Matrix3 objectPalmRot;

	//==HandBBFromRefMotion==
	cnoid::BodyItemPtr bodyItemGRC;
	cnoid::Vector3  dGRC_Pos_;  
	Box GRCmax, GRCdes, GRCmin; 
	double maxLoad, minLoad;
	int appVector;
	std::string refMotion;
	
	std::string *reffile,*refCsfile;
	int refSize;
	int refCsSize;
	std::vector<PrehensionPtr> prehensions;

	//==getGraspPos==
	cnoid::Vector3 palmPos;
	cnoid::Matrix3 palmRot;

	cnoid::Vector3 dif;

};


}



#endif

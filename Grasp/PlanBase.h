/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _PlanBase_H
#define _PlanBase_H

#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <float.h>
#include <cnoid/Item>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/ColdetModel>
#include "GraspPluginManager.h"
#include <boost/make_shared.hpp>

#include "ColdetLinkPair.h"
#include "ColdetPairData.h"
#include "Finger.h"
#include "Arm.h"
#include "InterObject.h"
#include "InterLink.h"
#include "PrehensionParameter.h"
#include "RobotBodyColChecker.h"
#include "PointCloudEnv.h"
#include "RobotHand.h"
#include "RobotBody.h"
#include "ObjectBase.h"

#include "exportdef.h"

#ifdef __GNUC__
#define DEPRECATED(comment) __attribute__ ((deprecated(comment)))
#else
#define DEPRECATED(comment)
#endif

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
  #ifndef CNOID_10_11
  #define YAML_SCALAR ValueNode::SCALAR
  #define YAML_SEQUENCE ValueNode::SEQUENCE
  #endif
#else
  #define YAML_SCALAR ValueNode::SCALAR
  #define YAML_SEQUENCE ValueNode::LISTING
#endif

namespace grasp{

class EXCADE_API PlanBase;

 class ObjectManager;
 class MultiHandObjectColChecker;

class Box{
public:
	cnoid::Vector3 p;
	cnoid::Matrix3 R;
	cnoid::Vector3 edge;
};

class Approach{
public:
	Approach(const cnoid::Vector3& dir, const std::vector<cnoid::Vector3>& pos, const std::vector<cnoid::Vector3>& nor)
		{this->direction = dir; this->position = pos; this->normal = nor;}
	cnoid::Vector3 direction;
	std::vector<cnoid::Vector3> position;
	std::vector<cnoid::Vector3> normal;
};

class EXCADE_API TargetObject{
public:
	friend class PlanBase;

	TargetObject(cnoid::BodyItemPtr bodyItem);
	virtual ~TargetObject() {;}
	const std::string& name() { return bodyItemObject->name(); }

	cnoid::Vector3 objCoM(){ return cnoid::Vector3( objVisRot * objCoM_   + objVisPos ); }
	cnoid::Matrix3 OCP_R(){	return cnoid::Matrix3(objVisRot * OCP.R); }
	cnoid::Vector3 OCP_p(){ return cnoid::Vector3(objVisRot* OCP.p+objVisPos);	}
	cnoid::Vector3 OCP_edge(){	return cnoid::Vector3 (OCP.edge); }

	cnoid::BodyItemPtr bodyItemObject;
	//	private:
	cnoid::Link *object;
	cnoid::Vector3 objVisPos;
	cnoid::Matrix3 objVisRot;
	bool offsetApplied;
	double objMass;
	cnoid::Vector3 objCoM_;
	Box OCP;
	std::string preplanningFileName;

	std::vector<Approach*> approach;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	cnoid::ColdetModelPtr safeBoundingBox;
#else
	cnoid::SgShapePtr safeBoundingBox;
#endif
};

class ArmFingers;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
typedef boost::intrusive_ptr<ArmFingers> ArmFingersPtr;
#else
typedef cnoid::ref_ptr<ArmFingers> ArmFingersPtr;
#endif

class EXCADE_API ArmFingers  : public cnoid::Item
{
public:

	static void initializeClass(cnoid::ExtensionManager* ext);

	cnoid::BodyItemPtr bodyItemRobot;
	std::string bodyItemRobotPath;
	std::string dataFilePath;

 ArmFingers() : os(cnoid::MessageView::instance()->cout()) {;}
	ArmFingers(cnoid::BodyItemPtr bodyItem, const cnoid::Mapping& gSettings);
	~ArmFingers() {
		if(arm!=NULL) delete arm;
		if(!isSeparatedModel) {
			for (int i = 0; i < nFing; i++)
				if (fingers[i] != NULL) delete fingers[i];
			delete [] fingers;
			if(handJoint!=NULL) delete handJoint;
		}
	}
	int nFing;
	int nHandLink;

	// Variable wrist and palm indicate arm's palm and hand's palm, respectively.
	// If robot model contains hand part, palm and wrist point to the same Link pointer.
	cnoid::Link *wrist;
	cnoid::Link *palm;
	cnoid::Link *base;

	FingerPtr *fingers;
	cnoid::LinkTraverse *handJoint;

	ArmPtr arm;
	std::string pythonInterface;

	std::string name;
	std::multimap<std::string,std::string> contactLinks;

	cnoid::Vector3 objectPalmPos;
	cnoid::Matrix3 objectPalmRot;

	double mu;
	double fmax;
	double hmax;

	bool isSeparatedModel;

	std::string handName;

	int id;

	std::vector<std::string> prehensionFilePathList;
	std::vector<PrehensionPtr> prehensionList;
	std::vector<PrehensionPtr> getTargetPrehension();

	void loadPrehension(std::string filename);


	GrasplotEntry getGrasplotFunc() const {return gPluginManager.getGrasplotFunc;}
protected:
	std::ostream& os;
	GraspPluginManager gPluginManager;

};


class EXCADE_API MotionState {
public:
	MotionState() {
		id = -1;
		objectPalmPos = cnoid::Vector3::Zero();
		objectPalmRot = cnoid::Matrix3::Identity();
	}
	MotionState(cnoid::VectorXd jointSeq, int graspingState=0,
				int graspingState2=0, int id=-1, double tolerance=-1,
				double time=0) {
		this->jointSeq = cnoid::VectorXd::Zero(jointSeq.size());
		this->jointSeq = jointSeq;
		this->graspingState = graspingState;
		this->graspingState2= graspingState2;
		this->time = time;
		this->id = id;
		this->tolerance = tolerance;
		pos.setZero();
		rpy.setZero();
		objectPalmPos = cnoid::Vector3::Zero();
		objectPalmRot = cnoid::Matrix3::Identity();
	}
	
	void linInterpol(double rate, MotionState* m0, MotionState* m1) {
		jointSeq = (1.0-rate)*m0->jointSeq + rate*m1->jointSeq;
		pos = (1.0-rate)*m0->pos + rate*m1->pos;
		rpy = (1.0-rate)*m0->rpy + rate*m1->rpy;
		time = (1.0-rate)*m0->time + rate*m1->time;
		//graspingState = m0->graspingState;
		//graspingState2=m0->graspingState;
		//objectPalmPos = m0->objectPalmPos;
		//objectPalmRot = m0->objectPalmRot;
	}


	cnoid::VectorXd jointSeq;
	cnoid::Vector3 pos;
	cnoid::Vector3 rpy;
	int graspingState, graspingState2;
	int objectContactState;
	ObjectsState handObjectState;
	double time;
	std::vector<int> pathPlanDOF;
	double motionTime, startTime, endTime;
	int id, id2;
	double tolerance;
	std::vector<int> children;
	cnoid::Vector3 objectPalmPos;
	cnoid::Matrix3 objectPalmRot;
	cnoid::Vector3 appVec, appVec2;

	//cnoid::VectorXd bodyJointSeq;
	//std::vector<int>bodyJointPtrs; //this is id for armJointSeq;

private:
};


class EXCADE_API PlanBase
{

public :
	PlanBase();
	~PlanBase();

	static PlanBase* instance(PlanBase *gc=NULL);
	void finalize(){
		for(size_t i=0;i<armsList.size();i++){
			armsList[i]->detachFromParentItem();
		}
		armsList.clear();
	}

	void SetGraspedObject(cnoid::BodyItemPtr bodyItem);
	bool SetGraspingRobot(cnoid::BodyItemPtr bodyItem);

	bool AppendAssembleObject(const cnoid::BodyItemPtr& bodyItem);
	bool RemoveAssembleObject(const cnoid::BodyItemPtr& bodyItem);

	void SetEnvironment(cnoid::BodyItemPtr bodyItem){
		bodyItemEnv.push_back(bodyItem);
		bodyItemEnv.sort();
		bodyItemEnv.unique();
		initialCollision();
	}

	void RemoveEnvironment(cnoid::BodyItemPtr bodyItem){
		bodyItemEnv.remove(bodyItem);
		bodyItemEnv.sort();
		bodyItemEnv.unique();
		initialCollision();
	}

	void SetPointCloudEnvironment(std::vector<cnoid::Vector3>& points,
								  std::vector<cnoid::Vector3>& normals) {
		PointCloudEnv* point_cloud = new PointCloudEnv();
		point_cloud->addPoints(points);
		point_cloud->addNormals(normals);
		pointCloudEnv.push_back(point_cloud);
		initialCollision();
	}

	void RemoveAllPointCloudEnvironment() {
		for(std::list<PointCloudEnv*>::iterator li=pointCloudEnv.begin();
			li!=pointCloudEnv.end();++li) {
			delete (*li);
		}
	}

	class LessAbs{
	public:
		bool operator()(const double& l, const double& r) const{
			return fabs(l)<fabs(r);
		}
	};

	// interface functions of planning
	virtual bool initial();
	virtual void initialCollisionAlwaysRemake();
	virtual void initialCollision();
	virtual void initialRoughClearance();
	virtual void initialCollisionPointCloud(bool do_make_robo_pointcloud, bool do_make_obj_pointcloud);
	//	virtual void runPythonScripts(){};
	//	virtual void closeFingers();

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	static double calcContactPoint(cnoid::ColdetLinkPairPtr cPair, cnoid::Vector3 &Po, cnoid::Vector3 &Pf, cnoid::Vector3 &objN2, cnoid::Vector3 &fingerN);
#else
	static double calcContactPoint(grasp::ColdetLinkPairPtr cPair, cnoid::Vector3 &Po, cnoid::Vector3 &Pf, cnoid::Vector3 &objN2, cnoid::Vector3 &fingerN);
#endif
	static void calcBoundingBox(cnoid::ColdetModelPtr model, cnoid::Vector3 &edge, cnoid::Vector3& center, cnoid::Vector3& com, cnoid::Matrix3& Rot);
#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
	static void calcBoundingBox(cnoid::SgNode* node, cnoid::Vector3 &edge, cnoid::Vector3& center, cnoid::Vector3& com, cnoid::Matrix3& Rot);
#endif

	//==Object==
	TargetObject* targetObject;
	std::vector<TargetObject*> multiTargetObject;

	ObjectManager* getObjectManager() const {return obj_manager_;}

	//==Robot==
	ArmFingers* targetArmFinger;
	std::vector<ArmFingers*> armsList;
	cnoid::BodyItemPtr bodyItemRobot(){ return targetArmFinger->bodyItemRobot; }
	cnoid::BodyItemPtr bodyItemRobot(int i){ return armsList[i]->bodyItemRobot; }
	RobotBodyPtr robotBody_;
	RobotBodyPtr robotBody(){ return robotBody_; }

	//==Env==
	std::list<cnoid::BodyItemPtr> bodyItemEnv;
	std::list<PointCloudEnv*> pointCloudEnv;

	void calcForwardKinematics();
	bool isColliding();
	void setTolerance(double t){tolerance = t;}
	double clearance();
	double roughClearance(bool initial=false);
//	double roughClearance();
	double sweepRoughClearance(bool initial=false);
	double sweepRoughClearanceFast(bool initial=false);
	double tolerance;
	std::vector<int> pathPlanDOF;
	std::vector<std::vector<int> > pathPlanDOFSeq;

	bool isCollidingPointCloud(const std::vector<cnoid::Vector3>& p, cnoid::BodyItemPtr item, double tol=0.001) DEPRECATED("use PointCloudCollisionChecker");
	bool isCollidingPointCloud(PointCloudEnv* pc_env, cnoid::BodyItemPtr item, double tol=0.001) DEPRECATED("use PointCloudCollisionChecker");
	bool isCollidingPointCloud(PointCloudEnv* pc_env, cnoid::BodyPtr body, double tol=0.001) DEPRECATED("use PointCloudCollisionChecker");
	bool isCollidingPointCloudSub(PointCloudEnv* pc_env, cnoid::Link* link, double tol=0.001) DEPRECATED("use PointCloudCollisionChecker");
	bool getColPointCloud(std::vector<cnoid::Vector3>& colpoints, PointCloudEnv* pc_env, cnoid::BodyItemPtr Item, double tol=0.001) DEPRECATED("use PointCloudCollisionChecker");
	bool getColPointCloudSub(std::vector<cnoid::Vector3>& colpoints, PointCloudEnv* pc_env, cnoid::Link* link, double tol=0.001) DEPRECATED("use PointCloudCollisionChecker");
	bool isCollidingPointCloudFinger(const std::vector<cnoid::Vector3>& p, double tol=0.001);
	std::vector<cnoid::Vector3> pointCloud;
	bool doCheckCollisionPointCloudFinger;

	bool isCollidingAllCheck();
	bool isCollidingPointCloudAllCheck(PointCloudEnv* pc_env, cnoid::BodyItemPtr item, std::vector<std::string>& collink, double tol=0.001) DEPRECATED("use PointCloudCollisionChecker");
	struct ColPair {
		enum ColLinkType {ROBO, OBJ, ENV, CLOUD};
		ColLinkType type[2];
		std::string link_name[2];
	};
	std::vector<ColPair> colpair;

	int getArmID(const ArmPtr& arm) const;
	void setGraspingStateMainArm(int state);
	void setGraspingStateSubArm(int state);
	// setGraspingState and setGraspingState2 set the graspingstate of the left
	// and the right arm respectively
	void setGraspingState(int state);
	void setGraspingState2(int state);
	void setGraspingState(int i, int state);
	void setObjectContactState(int state) { objectContactState = state; }

	int getGraspingStateMainArm();
	int getGraspingStateSubArm();
	// getGraspingState and getGraspingState2 return the graspingstate of the left
	// and the right arm respectively
	int getGraspingState();
	int checkAllGraspingState();
	int getGraspingState2();
	int getObjectContactState() {return objectContactState; }
	int checkGraspingState(int arm_id);
	int getGraspingState(int i) const;
	ObjectsState getHandObjectState() const;
	void setHandObjectState(const ObjectsState& state);
	void setTrajectoryPlanDOF();
	void setTrajectoryPlanDOF(int k);
	void setTrajectoryPlanMapDOF();

	void removeColdetPairData(cnoid::BodyItemPtr bodyitem);

	// The three GraspingStates are used to switch collision detections.
	// Especially, under_grasping means the object is in the palm,
	// although not grasped by the fingers
	enum GraspingStates { NOT_GRASPING = Arm::NOT_GRASPING, UNDER_GRASPING = Arm::UNDER_GRASPING, GRASPING = Arm::GRASPING };

	// Like the GraspingStates, the TargetGraspingStates are used to switch
	// the collision detection between the object and the environment.
	enum TargetGraspingStates { ON_ENVIRONMENT, OFF_ENVIRONMENT };

	// graspHand
	enum GraspHand {RIGHT, LEFT, BOTH};

	std::vector<cnoid::VectorXd> jointSeq;
	std::vector<double> motionTimeSeq;
	std::vector<int>graspingStateSeq, graspingStateSeq2, objectContactStateSeq;
	std::vector<cnoid::Vector3> objectPalmPosSeq;
	std::vector<cnoid::Matrix3> objectPalmRotSeq;
	std::vector<MotionState>graspMotionSeq;
	std::vector<ObjectsState> handObjectStateSeq;
	MotionState getMotionState(double time=0);
	void setMotionState(MotionState gm);

	// MotionStateForPathPlanner
	MotionState startMotionState;
	MotionState endMotionState;
	MotionState graspMotionState;
	MotionState placeMotionState;
	cnoid::Vector3 ulimitMap;
	cnoid::Vector3 llimitMap;

	bool stopFlag;

	bool flush();
	std::ostream& os;

	cnoid::Link* object() { return targetObject->object; }
	cnoid::Vector3 objVisPos() { return  targetObject->objVisPos;  }
	cnoid::Matrix3 objVisRot() { return targetObject->objVisRot;  }
	void setObjPos(const cnoid::Vector3& P, const cnoid::Matrix3 R);

	void setVisOffset(const cnoid::Vector3& P);
	void setVisOffsetR();
	void removeVisOffset(const cnoid::Vector3& P);

	cnoid::BodyPtr body() { return targetArmFinger->bodyItemRobot->body(); }
	cnoid::BodyPtr body(int i) { return armsList[i]->bodyItemRobot->body(); }
	cnoid::Link* wrist() { return targetArmFinger->wrist; }
	cnoid::Link* wrist(int i) { return armsList[i]->wrist; }
	cnoid::Link* palm() { return targetArmFinger->palm; }
	cnoid::Link* palm(int i) { return armsList[i]->palm; }
	cnoid::Link* base() { return targetArmFinger->base; }
	cnoid::Link* base(int i) { return armsList[i]->base; }
	ArmPtr arm(){ return targetArmFinger->arm; }
	ArmPtr arm(int i){ return armsList[i]->arm; }

	void calcWristPos(const cnoid::Vector3& palm_p, const cnoid::Matrix3& palm_R, cnoid::Vector3& wrist_p, cnoid::Matrix3& wrist_R) const {
		wrist_R = palm_R * targetArmFinger->arm->toolWristRot.transpose();
		wrist_p = palm_p - wrist_R * targetArmFinger->arm->toolWristPos;
	}

	void calcWristPos(int i, const cnoid::Vector3& palm_p, const cnoid::Matrix3& palm_R, cnoid::Vector3& wrist_p, cnoid::Matrix3& wrist_R) const {
		wrist_R = palm_R * armsList[i]->arm->toolWristRot.transpose();
		wrist_p = palm_p - wrist_R * armsList[i]->arm->toolWristPos;
	}

	FingerPtr fingers(int i) { return targetArmFinger->fingers[i]; }
	FingerPtr fingers(int i, int j) { return armsList[i]->fingers[j]; }
	int nFing() { return targetArmFinger->nFing; }
	int nFing(int i) { return armsList[i]->nFing; }

	cnoid::LinkTraverse* handJoint() { return targetArmFinger->handJoint; }
	cnoid::LinkTraverse* handJoint(int i) { return armsList[i]->handJoint; }
	int nHandLink() {return targetArmFinger->nHandLink;}
	int nHandLink(int i) {return armsList[i]->nHandLink;}

	std::string  bodyItemRobotPath() {return  targetArmFinger->bodyItemRobotPath; }
	std::string  bodyItemRobotPath(int i) {return  armsList[i]->bodyItemRobotPath; }
	std::string  dataFilePath() {return  targetArmFinger->dataFilePath; }
	std::string  dataFilePath(int i) {return  armsList[i]->dataFilePath; }
	std::string  pythonInterface() {return  targetArmFinger->pythonInterface; }
	std::string  pythonInterface(int i) {return  armsList[i]->pythonInterface; }
	std::string armName() {return targetArmFinger->name; }
	std::string armName(int i) {return armsList[i]->name; }

	ColdetLinkPairVector robotSelfPairs, robotEnvPairs, robotObjPairs, objEnvPairs, safeRobotEnvPairs, safeRobotObjPairs; // armEnvPairs;
	ColdetLinkPairVector robotObjPairWithoutHand;
	std::vector<ColdetLinkPairVector> handObjPair;

	std::vector<std::pair<cnoid::Link*, PointCloudEnv*> > robotExcludingFingPointCloudPairs, fingPointCloudPairs, objPointCloudPairs;
	std::string colPairName[2], objPressName;
	cnoid::Vector3 objPressPos;
	cnoid::BodyItemPtr pressItem;
	bool doInitialCollision;
	/**
	 * @brief coldetPairData
	 * Each coldetPairData indicates a pair of bodyitems
	 * Each bodyitem may have several links.
	 */
	std::vector<ColdetPairData*> coldetPairData, safeColdetPairData;
	std::vector<ColdetPairData*> coldetPairDataRobotObj;
	std::vector<std::vector<ColdetPairData*> > coldetPairDataHandObj;

	std::vector<LinkRoughClearance> roughClearanceRobotEnv, roughClearanceRobotHandFreeObj, roughClearanceHandObj, roughClearanceObjEnv;

	std::map <std::string,cnoid::BodyItemPtr> objTag2Item;
	std::map <std::string,ArmFingers*> robTag2Arm;

	std::vector<InterLink> interLinkList;
	void setInterLink();

	std::vector<InterObject> interObjectList;

	bool useObjectSafeBoundingBox , useRobotSafeBoundingBox;
	cnoid::Vector3 boundingBoxSafetySize;

	// for multithread
	std::vector<RobotBodyColChecker*> col_checkers;
	RobotBodyColChecker* findColChecker(const RobotBodyPtr& body);
	RobotBodyColChecker* findColChecker(const cnoid::BodyPtr& body);
	void calcForwardKinematics(const RobotBodyPtr& body);
	double clearance(const RobotBodyPtr& body);
	bool isColliding(const RobotBodyPtr& body);
	void setGraspingState(const RobotBodyPtr& body, int state);
	void setGraspingState2(const RobotBodyPtr& body, int state);
	void setObjectContactState(const RobotBodyPtr& body, int state);
	void SetEnvironment(const RobotBodyPtr& body, cnoid::BodyItemPtr bodyItem);
	void RemoveEnvironment(const RobotBodyPtr& body, cnoid::BodyItemPtr bodyItem);
	std::list<cnoid::BodyItemPtr> getBodyItemEnv(const cnoid::BodyPtr& body);
	std::string getColPairName(const RobotBodyPtr& body, int i);

protected :

	/* int graspingState, graspingState2; */
	int motionId;
	int objectContactState;
	MultiHandObjectColChecker* hand_obj_colchecker;
	ObjectManager* obj_manager_;
};




}



#endif

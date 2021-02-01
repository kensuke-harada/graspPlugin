// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <fstream>
#include <string>
#include <iostream>

#include <math.h>

#include <algorithm>
#include <time.h>
#ifndef WIN32
#include <sys/resource.h>
#endif

#include <boost/filesystem.hpp>

#include <cnoid/JointPath>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/ExecutablePath>
#include <cnoid/ItemTreeView>
#include <cnoid/YAMLReader>

//#define DEBUG_MODE
//#define SET_TOLERANCE_MODE

#include "GraspController.h"
#include "PlaceController.h"
#include "readtext.h"
#include "VectorMath.h"
#include "ForceClosureTest.h"
#include "PrehensionParameter.h"

#include "GraspPluginManager.h"

//#define BOOST_PYTHON_STATIC_LIB
//#ifndef NDEBUG
//#define BOOST_DEBUG_PYTHON
//#endif
#include <boost/python.hpp>
#include <boost/make_shared.hpp>

#define deg2Rad(x)   ((x)*(3.141596)/(180.0))
#define m_pi    (3.141596)

#include "ColdetPairData.h"
#include "MultiHandObjectColChecker.h"
#include "AssemblyObject.h"
#include "ObjectManager.h"
#include "ColdetModelGetter.h"
#include "PointCloudCollisionChecker.h"

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#define NODE_TYPE type()
#else
#define NODE_TYPE nodeType()
#endif

#ifdef WIN32
namespace boost
{
	template <>
	grasp::PlanBase const volatile * get_pointer<class grasp::PlanBase const volatile >(
		class grasp::PlanBase const volatile *c)
	{
		return c;
	}
}
#endif

namespace grasp{


void _initrand(){
#ifdef WIN32
	srand((unsigned)time(NULL));
#else
	srand48(time(0));
#endif
}

#ifdef WIN32
double getrusage_sec() {
	return clock();
}
#else
double EXCADE_API getrusage_sec() {
	struct rusage t;
	struct timeval tv;
	getrusage(RUSAGE_SELF, &t);
	tv = t.ru_utime;
	return tv.tv_sec + (double)tv.tv_usec*1e-6;
}
#endif

class PythonConverter{
public:
	static PythonConverter* instance(){
		static PythonConverter* instance = new PythonConverter;
		return instance;
	}
	static cnoid::Vector3 setVector3(double a, double b, double c){
		return cnoid::Vector3(a,b,c);
	}
};

}

BOOST_PYTHON_MODULE( grasp )
{
	using namespace boost::python;
	//   class_<grasp::GraspController, boost::noncopyable>("GraspController", no_init)
	//       .def("instance", &grasp::GraspController::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
	//   ;
	class_<grasp::PlanBase, boost::noncopyable>("PlanBase", no_init)
			.def("instance", &grasp::PlanBase::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
			.def("initial", &grasp::PlanBase::initial)
			//        .def("doGraspPlanning", &grasp::PlanBase::doGraspPlanning)
			//       .def("doPlacePlanning", &grasp::PlanBase::doPlacePlanning)
			//      .def("doPickAndPlacePlanning", &grasp::PlanBase::doPickAndPlacePlanning)
			.add_property("stopFlag", &grasp::PlanBase::stopFlag) //test
			;
	class_<cnoid::BodyItemPtr>("BodyItemPtr")
			;
	class_<cnoid::Vector3>("Vector3")
			;
	class_<grasp::PythonConverter>("PythonConverter")
			.def("Vector3", &grasp::PythonConverter::setVector3).staticmethod("Vector3")
			;
}

using namespace std;
using namespace cnoid;
using namespace grasp;
using namespace boost;

namespace {
	ColdetPairData* findColdetPairData(const cnoid::BodyItemPtr& bodyitem1, const cnoid::BodyItemPtr& bodyitem2,
									   const std::vector<ColdetPairData*>& coldet_pair_data_vec) {
		for (int i = 0; i < coldet_pair_data_vec.size(); i++) {
			if (((coldet_pair_data_vec[i]->bodyItem1 == bodyitem1) && (coldet_pair_data_vec[i]->bodyItem2 == bodyitem2)) ||
				((coldet_pair_data_vec[i]->bodyItem1 == bodyitem2) && (coldet_pair_data_vec[i]->bodyItem2 == bodyitem1))) {
				return coldet_pair_data_vec[i];
			}
		}
		return NULL;
	}

	void initialCollisionProc(cnoid::BodyItemPtr bodyitem1, const std::list<cnoid::BodyItemPtr>& bodyitemlist, std::vector<ColdetPairData*>& coldetPairDataCache, ColdetLinkPairVector& link_pair, bool doComputeDistance=false, bool useRobotSafeBoundingBox=false) {
		for (std::list<cnoid::BodyItemPtr>::const_iterator it = bodyitemlist.begin();
			 it != bodyitemlist.end(); ++it) {
			BodyItemPtr bodyitem2 = *it;
			ColdetPairData* target_pair_data = findColdetPairData(bodyitem1, bodyitem2, coldetPairDataCache);
			if (target_pair_data == NULL) {
				target_pair_data = new ColdetPairData(bodyitem1, bodyitem2, doComputeDistance, useRobotSafeBoundingBox);
				coldetPairDataCache.push_back(target_pair_data);
			}
			link_pair.insert(link_pair.end(),
							 target_pair_data->coldetLinkPairs.begin(),
							 target_pair_data->coldetLinkPairs.end());
		}
	}

	void initialCollisionProc(cnoid::BodyItemPtr bodyitem1, std::vector<cnoid::Link*>& target_link, const std::list<cnoid::BodyItemPtr>& bodyitemlist, std::vector<ColdetPairData*>& coldetPairDataCache, ColdetLinkPairVector& link_pair) {
		for (std::list<cnoid::BodyItemPtr>::const_iterator it = bodyitemlist.begin();
			 it != bodyitemlist.end(); ++it) {
			BodyItemPtr bodyitem2 = *it;
			ColdetPairData* target_pair_data = findColdetPairData(bodyitem1, bodyitem2, coldetPairDataCache);
			if (target_pair_data == NULL) {
				target_pair_data = new ColdetPairData(bodyitem1, target_link, bodyitem2);
				coldetPairDataCache.push_back(target_pair_data);
			}
			link_pair.insert(link_pair.end(),
							 target_pair_data->coldetLinkPairs.begin(),
							 target_pair_data->coldetLinkPairs.end());
		}
	}

	void removeColdetPairDataProc(std::vector<ColdetPairData*>& pairdata, const BodyItemPtr& bodyitem) {
		std::vector<ColdetPairData*>::iterator it;
		for (it = pairdata.begin(); it != pairdata.end();) {
			if ((*it)->bodyItem1 == bodyitem || (*it)->bodyItem2 == bodyitem) {
				delete (*it);
				it = pairdata.erase(it);
			} else {
				++it;
			}
		}
	}
}

MotionState PlanBase::getMotionState(double time){
	MotionState ret;

	ret.jointSeq = VectorXd(robotBody()->numJoints());
	for(int i=0;i<robotBody()->numJoints();i++)
		ret.jointSeq[i] = robotBody()->joint(i)->q();

	ret.pos = robotBody()->link(0)->p();
	ret.rpy = rpyFromRot(robotBody()->link(0)->R());

	ret.graspingState = getGraspingState();
	ret.graspingState2 = getGraspingState2();
	if( getGraspingStateMainArm()==GRASPING ){
		ret.objectPalmPos = targetArmFinger->objectPalmPos;
		ret.objectPalmRot = targetArmFinger->objectPalmRot;
	}
	ret.objectContactState = getObjectContactState();
	ret.handObjectState = getHandObjectState();
	ret.pathPlanDOF = pathPlanDOF;
	ret.tolerance = tolerance;
	ret.time = time;
	ret.id = motionId;
	return ret;
}

void PlanBase::setMotionState(MotionState gm){
	for(int i=0;i<robotBody()->numJoints();i++){
		robotBody()->joint(i)->q() = gm.jointSeq[i];
	}
	robotBody()->link(0)->p() = gm.pos;
	robotBody()->link(0)->R() = rotFromRpy(gm.rpy);

	ObjectPositionState::storeState(robotBody().get(), gm.handObjectState.objectState(robotBody()->descriptor()).obj_pos_state);
	setHandObjectState(gm.handObjectState);
	setGraspingState(gm.graspingState);
	setGraspingState2(gm.graspingState2);
	if( getGraspingStateMainArm()==GRASPING ){
		targetArmFinger->objectPalmPos = gm.objectPalmPos;
		targetArmFinger->objectPalmRot = gm.objectPalmRot;
	}
	setObjectContactState(gm.objectContactState);
	pathPlanDOF = gm.pathPlanDOF;
	setTolerance(gm.tolerance);
	calcForwardKinematics();
	motionId = gm.id;
}

PlanBase::PlanBase()  : 	os (MessageView::mainInstance()->cout() )
{
	//	bodyItemRobot = NULL;
	//	bodyItemGRC = NULL;
	targetObject = NULL;
	targetArmFinger=NULL;
	stopFlag = false;
	//	refSize = refCsSize = 0;
	//	arm = NULL;
	//	fingers = NULL;
	// graspingState = NOT_GRASPING;
	// graspingState2 = NOT_GRASPING;
	obj_manager_ = new ObjectManager();
	hand_obj_colchecker = new MultiHandObjectColChecker();
	hand_obj_colchecker->setObjectManager(obj_manager_);
	tolerance = 0.0;
	ulimitMap=Vector3(1,1,1);
	llimitMap = Vector3(-1,-1,-1);
	doInitialCollision=true;
	useObjectSafeBoundingBox = false;
	useRobotSafeBoundingBox = false;
	boundingBoxSafetySize = Vector3(0.005,0.005,0.005);
	motionId = -1;
	doCheckCollisionPointCloudFinger = true;
#ifdef CNOID_GE_17
	if ( PyImport_AppendInittab( (char *)"grasp", PyInit_grasp ) == -1 ) {
#else
	if ( PyImport_AppendInittab( (char *)"grasp", initgrasp ) == -1 ) {
#endif
		MessageView::mainInstance()->put("faild init Grasp Module");
		//        return;
	}
}

PlanBase::~PlanBase() {
	delete hand_obj_colchecker;
	delete obj_manager_;
	RemoveAllPointCloudEnvironment();
}

PlanBase* PlanBase::instance(PlanBase *gc) {
	static PlanBase* instance = (gc) ? gc : new PlanBase();
	if(gc) instance = gc;
	return instance;
}

TargetObject::TargetObject(cnoid::BodyItemPtr bodyItem){
	bodyItemObject = bodyItem;
	object = bodyItemObject->body()->link(0);
	objVisPos = object->p();
	objVisRot = object->R();
	objMass = object->m();
	offsetApplied = false;
}


void PlanBase::SetGraspedObject(cnoid::BodyItemPtr bodyItem){
	if (targetObject != NULL) delete targetObject;
	targetObject = new TargetObject(bodyItem); //shoud be chaged;

	Box& OCP = targetObject->OCP;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	calcBoundingBox(object()->coldetModel(), OCP.edge, OCP.p, targetObject->objCoM_, OCP.R);
#else
	calcBoundingBox(object()->collisionShape(), OCP.edge, OCP.p, targetObject->objCoM_, OCP.R);
#endif
	if(targetArmFinger){
		for(int i=0;i<nFing();i++) fingers(i)->coldetLinkPair(targetObject->bodyItemObject);
#ifdef  CNOID_10_11_12_13
		arm()->palmObjPair = new ColdetLinkPair(wrist(),object() );
#else
		arm()->palmObjPair = boost::make_shared<ColdetLinkPair>(body(), wrist(), bodyItem->body(), object() );
#endif
	}
	string tagId = bodyItem->name();
	if(objTag2Item.find(tagId) == objTag2Item.end()){
		objTag2Item.insert( pair <string,BodyItemPtr>(tagId, bodyItem) );
	}
	setObjectContactState(ON_ENVIRONMENT) ;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	targetObject->safeBoundingBox = ColdetPairData::getSafeBoundingBox(object()->coldetModel(), boundingBoxSafetySize);
#else
	targetObject->safeBoundingBox = ColdetPairData::getSafeBoundingBox(object()->collisionShape(), boundingBoxSafetySize);
#endif

#ifdef ROUGH_CLEARANCE
	roughClearanceObjEnv.clear();
	roughClearanceObjEnv.push_back(LinkRoughClearance(object()));
#endif
	hand_obj_colchecker->setTargetObject(targetObject);
	initialCollision();
}

ArmFingers::ArmFingers(cnoid::BodyItemPtr bodyItem, const Mapping& gSettings) :  os (MessageView::mainInstance()->cout() )
{
	bodyItemRobot = bodyItem;

#ifdef CNOID_10_11
	boost::filesystem::path robotfullpath( bodyItemRobot->modelFilePath() );
#elif defined(CNOID_10_11_12_13) || defined(CNOID_14)
	boost::filesystem::path robotfullpath( bodyItemRobot->lastAccessedFilePath() );
#else
	boost::filesystem::path robotfullpath( bodyItemRobot->filePath() );
#endif

	//cout << robotfullpath.string() << endl;
	bodyItemRobotPath = boost::filesystem::path (robotfullpath.branch_path()).string();
	dataFilePath = bodyItemRobotPath + "/data/";

	//READ YAML setting

	palm = NULL;
	wrist = NULL;
	if (gSettings.find("palm")->isString()) {
		palm = bodyItemRobot->body()->link(gSettings["palm"].toString());
	}
	if (gSettings.find("wrist")->isString()) {
		wrist = bodyItemRobot->body()->link(gSettings["wrist"].toString());
	}
	if (palm == NULL) palm = wrist;
	if (wrist == NULL) wrist = palm;
	base = bodyItemRobot->body()->link(gSettings["base"].toString());
	palm = wrist;

	const cnoid::Listing& tips = *gSettings.findListing("fingerEnds");
	if (tips.isValid()) {
		nFing = tips.size();
	} else {
		nFing = 0;
	}

	fingers = new FingerPtr[nFing];

	arm=NULL;
	for (int i = 0;i < nFing;i++) {
		fingers[i]=NULL;
	}

	if( gSettings.find("GrasplotPluginDir")->NODE_TYPE == YAML_SCALAR ){
#ifdef WIN32
		string pluginPath =  cnoid::executableTopDirectory() + string("\\bin\\") + bodyItemRobot->name() + string("\\");
#else
		string pluginPath = bodyItemRobotPath + "/" + gSettings["GrasplotPluginDir"].toString();
#endif
		os << "Grasplot Plugin Path " << pluginPath << endl;
		gPluginManager.scanPluginFiles(pluginPath);

		arm = (Arm *)gPluginManager.loadGrasplotPlugin(bodyItemRobot->body(),base,wrist, "Arm");

		for (int i = 0;i < nFing;i++) {
			if(!fingers[i]) fingers[i] = (Finger *)gPluginManager.loadGrasplotPlugin
					(bodyItemRobot->body(), wrist, bodyItemRobot->body()->link(tips[i].toString()), "Finger");
		}
	}
	if(!arm){
		arm = new Arm(bodyItemRobot->body(),base, wrist);
	}
	for (int i = 0;i < nFing;i++) {
		if(!fingers[i]) fingers[i] = new Finger(bodyItemRobot->body(), wrist, bodyItemRobot->body()->link(tips[i].toString()) );
		fingers[i]->number = i;
	}

	if( gSettings.find("dataFilePath")->NODE_TYPE == YAML_SCALAR ){
		dataFilePath = bodyItemRobotPath + "/" + gSettings["dataFilePath"].toString() +"/";
	}

	if( gSettings.find("armStandardPose")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["armStandardPose"].toListing();
		for(int i=0;i<list.size();i++){
			arm->armStandardPose.push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("armFinalPose")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["armFinalPose"].toListing();
		for(int i=0;i<list.size();i++){
			arm->armFinalPose.push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("regraspPoseRel_X")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["regraspPoseRel_X"].toListing();
		for(int i=0;i<list.size();i++){
			arm->regraspPoseRel[0].push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("regraspPoseRel_Y")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["regraspPoseRel_Y"].toListing();
		for(int i=0;i<list.size();i++){
			arm->regraspPoseRel[1].push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("regraspPoseRel_Z")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["regraspPoseRel_Z"].toListing();
		for(int i=0;i<list.size();i++){
			arm->regraspPoseRel[2].push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("regraspPoseRel_Roll")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["regraspPoseRel_Roll"].toListing();
		for(int i=0;i<list.size();i++){
			arm->regraspPoseRel[3].push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("regraspPoseRel_Pitch")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["regraspPoseRel_Pitch"].toListing();
		for(int i=0;i<list.size();i++){
			arm->regraspPoseRel[4].push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("regraspPoseRel_Yaw")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["regraspPoseRel_Yaw"].toListing();
		for(int i=0;i<list.size();i++){
			arm->regraspPoseRel[5].push_back(list[i].toDouble());
		}
	}


	if( gSettings.find("fingerOpenPose")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["fingerOpenPose"].toListing();
		int j=0;
		int k=0;
		for(int i=0;i<list.size();i++){
			if(i>=k+fingers[j]->fing_path->numJoints()){
				k += fingers[j]->fing_path->numJoints();
				j++;
			}
			fingers[j]->fingerOpenPose.push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("fingerOpenPoseOffset")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["fingerOpenPoseOffset"].toListing();
		int j=0;
		int k=0;
		for(int i=0;i<list.size();i++){
			if(i>=k+fingers[j]->fing_path->numJoints()){
				k += fingers[j]->fing_path->numJoints();
				j++;
			}
			fingers[j]->fingerOpenPoseOffset.push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("fingerCloseOffset")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["fingerCloseOffset"].toListing();
		int j=0;
		int k=0;
		for(int i=0;i<list.size();i++){
			if(i>=k+fingers[j]->fing_path->numJoints()){
				k += fingers[j]->fing_path->numJoints();
				j++;
			}
			fingers[j]->fingerCloseOffset.push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("fingerOffset")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["fingerOffset"].toListing();
		for(int j=0; j<nFing; j++)
			fingers[j]->offset = list[0].toDouble();
	}

	if( gSettings.find("pythonInterface")->NODE_TYPE == YAML_SCALAR ){
		pythonInterface = bodyItemRobotPath + "/" + gSettings["pythonInterface"].toString();
	}else{
		pythonInterface = "/NULL";
	}

	if (PlanBase::instance()->targetArmFinger == NULL) {
	vector <InterLink> & interLinkList = PlanBase::instance()->interLinkList;
	if( gSettings.find("interlink")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["interlink"].toListing();
		for(int i=0;i<list.size();i++){
			const Listing& ilist = *list[i].toListing();
			Link* master = bodyItemRobot->body()->link(ilist[0].toString());
			double baseratio = ilist[1].toDouble();
			for(int j=1;j<ilist.size()/2;j++){
				InterLink temp;
				temp.master = master;
				temp.slave = bodyItemRobot->body()->link(ilist[2*j].toString());
				temp.ratio = ilist[2*j+1].toDouble()/baseratio;
				interLinkList.push_back(temp);
			}

		}
	}
	}
	if( gSettings.find("approachOffset")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["approachOffset"].toListing();
		for(int i=0;i<list.size();i++){
			arm->approachOffset[i] = list[i].toDouble();
		}
	}

	if( gSettings.find("selfContactPair")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["selfContactPair"].toListing();
		if(list[0].NODE_TYPE == YAML_SCALAR){
			for(int i=0;i<list.size()/2;i++){
				contactLinks.insert ( make_pair ( list[i*2].toString(), list[i*2+1].toString() ) );
			}
		}
		if(list[0].NODE_TYPE == YAML_SEQUENCE){
			for(int i=0;i<list.size();i++){
				const Listing& plist = *list[i].toListing();
				for(int j=0;j<plist.size();j++){
					for(int k=j+1;k<plist.size();k++){
						contactLinks.insert ( make_pair (plist[j].toString(), plist[k].toString() )  );
					}
				}
			}


		}
	}
	if( gSettings.find("movableArea")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["movableArea"].toListing();
		const Listing& ilist = *list[0].toListing();
		PlanBase::instance()->llimitMap = Vector3 ( ilist[0].toDouble(),ilist[1].toDouble(),ilist[2].toDouble() );
		const Listing& ilist1 = *list[1].toListing();
		PlanBase::instance()->ulimitMap = Vector3 ( ilist1[0].toDouble(),ilist1[1].toDouble(),ilist1[2].toDouble() );
	}

	//cout << "llimit" << PlanBase::instance()->llimitMap.transpose() << " ";
	//cout << "ulimit" << PlanBase::instance()->ulimitMap.transpose() << endl;

	if( gSettings.find("InterObject")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["InterObject"].toListing();
		for(int i=0;i<list.size();i++){
			InterObject tempo;
			const Listing& ilist = *list[i].toListing();
			tempo.master  = bodyItemRobot->body()->link(ilist[0].toString());

			BodyItemPtr temp = new BodyItem();
			if( !temp->loadModelFile(dataFilePath +ilist[1].toString()) ){
				os << "modelLoadError: " << dataFilePath +ilist[1].toString() << endl;
				break;
			}
			temp->setName(ilist[1].toString());
			bodyItemRobot->addChildItem (temp);
			tempo.slaveItem = temp;
			const Listing& vlist = *ilist[2].toListing();
			tempo.relativePos = Vector3 ( vlist[0].toDouble(),vlist[1].toDouble(),vlist[2].toDouble() );
			const Listing& vlist2 = *ilist[3].toListing();
			tempo.relativeRot << vlist2[0].toDouble(), vlist2[1].toDouble(), vlist2[2].toDouble(), vlist2[3].toDouble(), vlist2[4].toDouble(), vlist2[5].toDouble(), vlist2[6].toDouble(), vlist2[7].toDouble(), vlist2[8].toDouble();
			tempo.setInterObject();
			tempo.type = InterObject::ROBOT;
			PlanBase::instance()->interObjectList.push_back(tempo);
		}
	}

	arm->toolWristPos = Vector3::Zero();
	if( gSettings.find("relToolPos")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["relToolPos"].toListing();
		if (list.size() != 3) {
			os << "YAML file error (relToolPos)" << endl;
		} else {
			arm->toolWristPos = Vector3(list[0].toDouble(), list[1].toDouble(), list[2].toDouble());
		}
	}

	arm->toolWristRot = Matrix3::Identity();
	if( gSettings.find("relToolRot")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["relToolRot"].toListing();
		if (list.size() != 9) {
			os << "YAML file error (relToolRot)" << endl;
		} else {
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					arm->toolWristRot(i, j) = list[3* i + j].toDouble();
				}
			}
		}
	}

	if( gSettings.find("mu")->NODE_TYPE == YAML_SCALAR){
		mu = gSettings["mu"].toDouble();
	}else{
		mu = 0.5;
	}

	if( gSettings.find("fmax")->NODE_TYPE == YAML_SCALAR){
		fmax = gSettings["fmax"].toDouble();
	}else{
		fmax = 10;
	}

	if( gSettings.find("hmax")->NODE_TYPE == YAML_SCALAR){
		hmax = gSettings["hmax"].toDouble();
	}else{
		hmax = 0.005;
	}

	if( gSettings.find("handName")->NODE_TYPE == YAML_SCALAR){
		handName = gSettings["handName"].toString();
	}else{
		handName = "";
	}

	prehensionFilePathList.clear();
	if( gSettings.find("prehensionList")->NODE_TYPE == YAML_SEQUENCE ){
		const Listing& list = *gSettings["prehensionList"].toListing();
		if (!list.empty()){
			for (int i = 0; i < list.size() ; i++ ) {
				string path = dataFilePath + list[i].toString() + ".yaml";
				prehensionFilePathList.push_back(path);
				if (boost::filesystem::exists(boost::filesystem::path(path.c_str()))){
					loadPrehension(path);
				}
			}
		}
	}

	handJoint = new LinkTraverse(palm);
	nHandLink = handJoint->numLinks();

	if (gSettings.find("name")->NODE_TYPE == YAML_SCALAR ){
		name = gSettings["name"].toString();
	}else{
		static int i=0;
		stringstream namenum;
		namenum << arm << i;
		name = namenum.str();
	}

	isSeparatedModel = false;

}

void ArmFingers::initializeClass(cnoid::ExtensionManager* ext) {
	static bool initialized = false;

	if (!initialized) {
		cnoid::ItemManager& im = ext->itemManager();
		im.registerClass<ArmFingers>("ArmFingers");
		initialized = true;
	}
}

std::vector<PrehensionPtr> ArmFingers::getTargetPrehension() {
	vector<PrehensionPtr> prehensions;
	for (int i = 0; i < prehensionList.size(); i++) {
		if (ItemTreeView::instance()->isItemChecked(prehensionList[i])) {
			prehensions.push_back(prehensionList[i]);
		}
	}
	return prehensions;
}

void ArmFingers::loadPrehension(string filename) {
	YAMLReader parser;
	Mapping* root = parser.loadDocument(filename)->toMapping();
	ValueNode* prehen_node = root->find("Prehension");
	if (prehen_node->isListing()) {
		Listing* list = prehen_node->toListing();
		for (int i = 0; i < list->size(); i++) {
			PrehensionPtr prehen = new Prehension;
			prehen->info = list->at(i)->toMapping();
			prehen->setNameDefault();
			prehensionList.push_back(prehen);
			this->addSubItem(prehen);
		}
	} else if (prehen_node->isMapping()) {
		PrehensionPtr prehen = new Prehension;
		prehen->info = prehen_node->toMapping();
		prehen->setNameDefault();
		prehensionList.push_back(prehen);
		this->addSubItem(prehen);
	}
}

bool PlanBase::SetGraspingRobot(cnoid::BodyItemPtr bodyItem_){
	//setting robot

	armsList.clear(); //Temoporary;  will delete menbers
	interLinkList.clear();
	//	if (targetArmFinger != NULL) delete targetArmFinger;
	if (targetArmFinger) {
		obj_manager_->detachObject(targetArmFinger->bodyItemRobot);
	}

	targetArmFinger = NULL;

	robotBody_ = obj_manager_->createRobot(bodyItem_);

	for (int i = 0; i < robotBody_->armSize(); i++) {
		targetArmFinger = robotBody_->getArmFingers(i);
		armsList.push_back(targetArmFinger);
		bodyItem_->addSubItem(targetArmFinger);
		targetArmFinger->setName(targetArmFinger->name);
		for(int j=0;j<targetArmFinger->prehensionList.size();j++){
			ItemTreeView::instance()->checkItem(targetArmFinger->prehensionList[j], true);
		}
		int hand_size = robotBody_->getRobotArmFingers(i)->handListSize();
		for (int j = 0; j < hand_size; j++) {
			RobotHand* target_hand = robotBody_->getRobotArmFingers(i)->getRobotHand(j);
			for (int k = 0; k < target_hand->prehensionList.size(); k++) {
				ItemTreeView::instance()->checkItem(robotBody_->getRobotArmFingers(i)->getRobotHand(j)->prehensionList[k], true);
			}
		}
	}

	if (!armsList.empty()) {
		targetArmFinger = armsList[0];
	}

	if(targetArmFinger == NULL){
		obj_manager_->detachObject(bodyItem_);
		os << "ERROR graspPluginSetting is not found in yaml" << endl;
		return false;
	}

	robTag2Arm.clear();
	for(int i=0;i<armsList.size();i++){
		armsList[i]->id = i;
		string tagId = armsList[i]->name;
		if(robTag2Arm.find(tagId) != robTag2Arm.end()){
			os << "Error: the tagId is already recorded " << tagId << endl;
			continue;
		}else{
			robTag2Arm.insert( pair <string,ArmFingers*>(tagId, armsList[i]));
		}
	}

	os  << bodyItem_->name() << " has " <<armsList.size() << " arm(s) "<< endl;

	if(targetObject){
		for(int i=0;i<nFing();i++) fingers(i)->coldetLinkPair(targetObject->bodyItemObject);
#ifdef  CNOID_10_11_12_13
		arm()->palmObjPair = new ColdetLinkPair(wrist(),object() );
#else
		arm()->palmObjPair = boost::make_shared<ColdetLinkPair>(body(), wrist(), targetObject->bodyItemObject->body(), object() );
#endif
	}

	robotBody()->calcForwardKinematics();

	graspMotionSeq.clear();
	setGraspingState(NOT_GRASPING);
	setGraspingState2(NOT_GRASPING);

	robotSelfPairs.clear();
	for(unsigned int i=0;i<bodyItemRobot()->body()->numLinks();i++){ // If initial position is not collided, it is stored as
		for(unsigned int j=i+1;j<bodyItemRobot()->body()->numLinks();j++){
			bool pass = false;
			pair<multimap<string, string>::iterator, multimap<string, string>::iterator> ppp;
			ppp = targetArmFinger->contactLinks.equal_range(bodyItemRobot()->body()->link(i)->name() );
			for (multimap<string, string>::iterator it2 = ppp.first; it2 != ppp.second; ++it2){
				if(it2->second == bodyItemRobot()->body()->link(j)->name()) pass = true;
			}
			ppp = targetArmFinger->contactLinks.equal_range(bodyItemRobot()->body()->link(j)->name() );
			for (multimap<string, string>::iterator it2 = ppp.first; it2 != ppp.second; ++it2){
				if(it2->second == bodyItemRobot()->body()->link(i)->name()) pass = true;
			}
			if(pass) continue;
#ifdef  CNOID_10_11_12_13
			ColdetLinkPairPtr temp= new ColdetLinkPair(bodyItemRobot()->body()->link(i),bodyItemRobot()->body()->link(j));
#else
			ColdetLinkPairPtr temp = boost::make_shared<ColdetLinkPair>(bodyItemRobot()->body(), bodyItemRobot()->body()->link(i), bodyItemRobot()->body(), bodyItemRobot()->body()->link(j) );
#endif
			temp->updatePositions();
			int t1,t2;
			double p1[3],p2[3];
			double distance = temp->computeDistance(t1,p1,t2,p2);
			if(distance>1.0e-04)	robotSelfPairs.push_back(temp);
#ifdef DEBUG_MODE
			else os <<"collide on initial condition at robotSelfPair"  <<distance <<" "<< temp->model(0)->name() <<" " << temp->model(1)->name()  << endl;
#endif
		}
	}
	/*	for(unsigned int i=0; i<arm()->arm_path->numJoints(); i++){
		for(unsigned int j=i+2; j<arm()->arm_path->numJoints(); j++){
			robotSelfPairs.push_back(new ColdetLinkPair(arm()->arm_path->joint(i), arm()->arm_path->joint(j)) );
		}
	}*/
#ifdef ROUGH_CLEARANCE
	roughClearanceRobotEnv.clear();
	for(unsigned int i=0;i<bodyItemRobot()->body()->numLinks();i++){
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		if (bodyItemRobot()->body()->link(i)->coldetModel()->getNumVertices()==0) continue;
#else
		if (ColdetConverter::ExtractMesh(bodyItemRobot()->body()->link(i)->collisionShape())->vertices()->size()==0) continue;
#endif
		roughClearanceRobotEnv.push_back(LinkRoughClearance(bodyItemRobot()->body()->link(i)));
	}
	for (RobotBody::BodyItemIterator it = robotBody()->handbody_begin(); it != robotBody()->handbody_end(); ++it) {
		for (size_t i = 0; i < (*it)->body()->numLinks(); i++) {
			if (ColdetConverter::ExtractMesh((*it)->body()->link(i)->collisionShape())->vertices()->size()==0) continue;
			roughClearanceRobotEnv.push_back(LinkRoughClearance((*it)->body()->link(i)));
		}
	}
#endif
	hand_obj_colchecker->initialCollisionSelf();
	initialCollision();
	return true;
}

bool PlanBase::AppendAssembleObject(const cnoid::BodyItemPtr& bodyItem) {
	ObjectBasePtr ret = obj_manager_->hasObject(bodyItem);
	if (ret) return false;

	obj_manager_->createObject(bodyItem);
	return true;
}

bool PlanBase::RemoveAssembleObject(const cnoid::BodyItemPtr& bodyItem) {
	bool ret = obj_manager_->detachObject(bodyItem);
	if (!ret) return ret;
	return ret;
}


bool PlanBase::flush(){
	static int cnt=0;
	cnt++;

	if(stopFlag){
		stopFlag=false;
		throw(cnt);
	}
	/* it will be GraspController
	if(bodyItemGRC){
		bodyItemGRC->body()->link(0)->R = palm()->R*(GRCmax.R);
		bodyItemGRC->body()->link(0)->p = palm()->p+palm()->R*GRCmax.p;
		bodyItemGRC->notifyKinematicStateChange();
	}
*/
	//	bodyItemRobot()->body()->calcForwardKinematics();
	if(targetArmFinger) robotBody()->notifyKinematicStateChange();
	if(targetObject) targetObject->bodyItemObject->notifyKinematicStateChange();
	MessageView::mainInstance()->flush();

#ifdef  DEBUG_MODE
	usleep(100000);
#endif
	return true;

}

void PlanBase::calcForwardKinematics(){

	setInterLink();

	robotBody()->calcForwardKinematics();

	robotBody()->calcFK();

	if(getGraspingState(0)==GRASPING && arm(0)->target_grasp_objid < 0) {
		if(nFing(0)>0){
			object()->R() = fingers(0,0)->tip->R()*(armsList[0]->objectPalmRot);
			object()->p() = fingers(0,0)->tip->p()+fingers(0,0)->tip->R()*armsList[0]->objectPalmPos;
		}else{
			object()->R() = palm(0)->R()*(armsList[0]->objectPalmRot);
			object()->p() = palm(0)->p()+palm(0)->R()*armsList[0]->objectPalmPos;
		}
	}
	else if(armsList.size() > 1 && getGraspingState(1)==GRASPING && arm(1)->target_grasp_objid < 0) {
		if(nFing(1)>0) {
			object()->R() = fingers(1,0)->tip->R()*(armsList[1]->objectPalmRot);
			object()->p() = fingers(1,0)->tip->p()+fingers(1,0)->tip->R()*armsList[1]->objectPalmPos;
		} else {
			object()->R() = palm(1)->R()*(armsList[1]->objectPalmRot);
			object()->p() = palm(1)->p()+palm(1)->R()*armsList[1]->objectPalmPos;
		}
	}
	for(int i=0;i<interObjectList.size();i++){
		//		if(interObjectList[i].type == InterObject::GRASPED_OBJECT){
		interObjectList[i].setInterObject();
		//		}
	}
	//cout << "pos y "<< bodyItemRobot()->body()->link(0)->p.transpose() <<  rpyFromRot(bodyItemRobot()->body()->link(0)->R)[2] << endl;
}

bool PlanBase::isColliding(){
	//	cnoid::ColdetLinkPairPtr* robotSelfPairs, robotEnvPairs, robotObjPairs, objEnvPairs;
	for(int i=0;i<robotSelfPairs.size();i++){
		ColdetLinkPairPtr testPair = robotSelfPairs[i];
		testPair->updatePositions();
		bool coll = testPair->checkCollision();
		if(coll){
			colPairName[0] = testPair->model(0)->name();
			colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
			cout <<"self collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
			return true;
		}
	}
	int sizeRobotEnv = robotEnvPairs.size();
	if(useRobotSafeBoundingBox) sizeRobotEnv = safeRobotEnvPairs.size();
	for(int i=0;i<sizeRobotEnv;i++){
		ColdetLinkPairPtr testPair;
		if(useRobotSafeBoundingBox) testPair= safeRobotEnvPairs[i];
		else testPair= robotEnvPairs[i];

		testPair->updatePositions();
		bool coll = testPair->checkCollision();
		if(coll){
			colPairName[0] = testPair->model(0)->name();
			colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
			cout <<"robot env collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
			return true;
		}
	}
	for(int i=0;i<robotExcludingFingPointCloudPairs.size();i++){
		bool coll = isCollidingPointCloudSub(robotExcludingFingPointCloudPairs[i].second,robotExcludingFingPointCloudPairs[i].first);
		if(coll){
#ifdef DEBUG_MODE
			cout << "robot pointcloud collide" << endl;
#endif
			return true;
		}
	}
	if(doCheckCollisionPointCloudFinger){
		for(int i=0;i<fingPointCloudPairs.size();i++){
			bool coll = isCollidingPointCloudSub(fingPointCloudPairs[i].second,fingPointCloudPairs[i].first);
			if(coll){
#ifdef DEBUG_MODE
				cout << "robot(finger) pointcloud collide" << endl;
#endif
				return true;
			}
		}
	}
	if(targetObject){
		for(int i=0;i<robotObjPairWithoutHand.size();i++){
			ColdetLinkPairPtr testPair = robotObjPairWithoutHand[i];
			testPair->updatePositions();
			bool coll = testPair->checkCollision();
			if(coll){
				colPairName[0] = testPair->model(0)->name();
				colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
				cout <<"robot obj collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
				return true;
			}
		}
		for (int i=0;i<armsList.size();i++) {
			if(checkGraspingState(i)==NOT_GRASPING){
				for(int j=0;j<handObjPair[i].size();j++){
					ColdetLinkPairPtr testPair = handObjPair[i][j];
					testPair->updatePositions();
					bool coll = testPair->checkCollision();
					if(coll){
						colPairName[0] = testPair->model(0)->name();
						colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
						cout <<"robot obj collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
						return true;
					}
				}
			}
		}
		if(getObjectContactState()==OFF_ENVIRONMENT){
			for(int i=0;i<objEnvPairs.size();i++){
				ColdetLinkPairPtr testPair = objEnvPairs[i];
				testPair->updatePositions();
				bool coll = testPair->checkCollision();
				if(coll){
					colPairName[0] = testPair->model(0)->name();
					colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
					cout <<"obj env collide " << i  << " "<<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
					return true;
				}
			}
			for(int i=0;i<objPointCloudPairs.size();i++){
				bool coll = isCollidingPointCloudSub(objPointCloudPairs[i].second ,objPointCloudPairs[i].first);
				if(coll){
#ifdef DEBUG_MODE
					cout << "obj pointcloud collide" << endl;
#endif
					return true;
				}
			}
		}
	}

	for(int i=0;i<interObjectList.size();i++){
		if( interObjectList[i].isColliding() ) return true;
	}

	if(targetObject){
		if (hand_obj_colchecker->isColliding()) return true;
	}

	return false;
}

bool PlanBase::isCollidingAllCheck(){
	bool ret = false;
	colpair.clear();
	for(int i=0;i<robotSelfPairs.size();i++){
		ColdetLinkPairPtr testPair = robotSelfPairs[i];
		testPair->updatePositions();
		bool coll = testPair->checkCollision();
		if(coll){
			colPairName[0] = testPair->model(0)->name();
			colPairName[1] = testPair->model(1)->name();
			ColPair col;
			col.type[0] = ColPair::ROBO;
			col.type[1] = ColPair::ROBO;
			col.link_name[0] = colPairName[0];
			col.link_name[1] = colPairName[1];
			colpair.push_back(col);
			ret = true;
		}
	}
	int sizeRobotEnv = robotEnvPairs.size();
	if(useRobotSafeBoundingBox) sizeRobotEnv = safeRobotEnvPairs.size();
	for(int i=0;i<sizeRobotEnv;i++){
		ColdetLinkPairPtr testPair;
		if(useRobotSafeBoundingBox) testPair= safeRobotEnvPairs[i];
		else testPair= robotEnvPairs[i];

		testPair->updatePositions();
		bool coll = testPair->checkCollision();
		if(coll){
			colPairName[0] = testPair->model(0)->name();
			colPairName[1] = testPair->model(1)->name();
			ColPair col;
			col.type[0] = ColPair::ROBO;
			col.type[1] = ColPair::ENV;
			col.link_name[0] = colPairName[0];
			col.link_name[1] = colPairName[1];
			colpair.push_back(col);
			ret = true;
		}
	}

	for(int i=0;i<robotExcludingFingPointCloudPairs.size();i++){
		bool coll = isCollidingPointCloudSub(robotExcludingFingPointCloudPairs[i].second,robotExcludingFingPointCloudPairs[i].first);
		if(coll){
			ColPair col;
			col.type[0] = ColPair::ROBO;
			col.type[1] = ColPair::CLOUD;
			col.link_name[0] = robotExcludingFingPointCloudPairs[i].first->name();
			col.link_name[1] = "pointcloud";
			colpair.push_back(col);
			ret = true;
		}
	}
	if (doCheckCollisionPointCloudFinger) {
		for(int i=0;i<fingPointCloudPairs.size();i++){
			bool coll = isCollidingPointCloudSub(fingPointCloudPairs[i].second,fingPointCloudPairs[i].first);
			if(coll){
				ColPair col;
				col.type[0] = ColPair::ROBO;
				col.type[1] = ColPair::CLOUD;
				col.link_name[0] = fingPointCloudPairs[i].first->name();
				col.link_name[1] = "pointcloud";
				colpair.push_back(col);
				ret = true;
			}
		}
	}
	for(int i=0;i<robotObjPairWithoutHand.size();i++){
		ColdetLinkPairPtr testPair = robotObjPairWithoutHand[i];
		testPair->updatePositions();
		bool coll = testPair->checkCollision();
		if(coll){
			colPairName[0] = testPair->model(0)->name();
			colPairName[1] = testPair->model(1)->name();
			ColPair col;
			col.type[0] = ColPair::ROBO;
			col.type[1] = ColPair::OBJ;
			col.link_name[0] = colPairName[0];
			col.link_name[1] = colPairName[1];
			colpair.push_back(col);
			ret = true;
		}
	}
	for (int i=0;i<armsList.size();i++) {
		if(checkGraspingState(i)==NOT_GRASPING){
			for(int j=0;j<handObjPair[i].size();j++){
				ColdetLinkPairPtr testPair = handObjPair[i][j];
				testPair->updatePositions();
				bool coll = testPair->checkCollision();
				if(coll){
					colPairName[0] = testPair->model(0)->name();
					colPairName[1] = testPair->model(1)->name();
					ColPair col;
					col.type[0] = ColPair::ROBO;
					col.type[1] = ColPair::OBJ;
					col.link_name[0] = colPairName[0];
					col.link_name[1] = colPairName[1];
					colpair.push_back(col);
					ret = true;
				}
			}
		}
	}
	if(getObjectContactState()==OFF_ENVIRONMENT){
		for(int i=0;i<objEnvPairs.size();i++){
			ColdetLinkPairPtr testPair = objEnvPairs[i];
			testPair->updatePositions();
			bool coll = testPair->checkCollision();
			if(coll){
				colPairName[0] = testPair->model(0)->name();
				colPairName[1] = testPair->model(1)->name();
				ColPair col;
				col.type[0] = ColPair::OBJ;
				col.type[1] = ColPair::ENV;
				col.link_name[0] = colPairName[0];
				col.link_name[1] = colPairName[1];
				colpair.push_back(col);
				ret = true;
			}
		}
		for(int i=0;i<objPointCloudPairs.size();i++){
			bool coll = isCollidingPointCloudSub(objPointCloudPairs[i].second,objPointCloudPairs[i].first);
			if(coll){
				ColPair col;
				col.type[0] = ColPair::OBJ;
				col.type[1] = ColPair::CLOUD;
				col.link_name[0] = objPointCloudPairs[i].first->name();
				col.link_name[1] = "pointcloud";
				colpair.push_back(col);
				ret = true;
			}
		}
	}

	for(int i=0;i<interObjectList.size();i++){
		if( interObjectList[i].isColliding() ) return true;
	}

	return ret;
}

bool PlanBase::isCollidingPointCloud(PointCloudEnv* pc_env, BodyItemPtr item, double tol){
	return PointCloudCollisionChecker::isCollidingPointCloud(pc_env, item, tol);
}

bool PlanBase::isCollidingPointCloud(PointCloudEnv* pc_env, BodyPtr body, double tol){
	return PointCloudCollisionChecker::isCollidingPointCloud(pc_env, body, tol);
}

bool PlanBase::isCollidingPointCloudSub(PointCloudEnv* pc_env, Link* link, double tol) {
	return PointCloudCollisionChecker::isCollidingPointCloudSub(pc_env, link, tol);
}

bool PlanBase::getColPointCloud(vector<Vector3>& colpoints, PointCloudEnv* pc_env, BodyItemPtr item, double tol){
	return PointCloudCollisionChecker::getColPointCloud(colpoints, pc_env, item, tol);
}

bool PlanBase::getColPointCloudSub(vector<Vector3>& colpoints, PointCloudEnv* pc_env, Link* link, double tol) {
	return PointCloudCollisionChecker::getColPointCloudSub(colpoints, pc_env, link, tol);
}

bool PlanBase::isCollidingPointCloudAllCheck(PointCloudEnv* pc_env, BodyItemPtr item, vector<string>& collink, double tol){
	return PointCloudCollisionChecker::isCollidingPointCloudAllCheck(pc_env, item, collink, tol);
}

bool PlanBase::isCollidingPointCloud(const vector<Vector3>& p, BodyItemPtr item, double tol){
	return PointCloudCollisionChecker::isCollidingPointCloud(p, item, tol);
}

bool PlanBase::isCollidingPointCloudFinger(const vector<Vector3>& p, double tol){

	for(int i=0; i<nFing(); i++)
		for(int j=0; j<fingers(i)->fing_path->numJoints(); j++){
			if (ColdetModelGetter::get(fingers(i)->fing_path->joint(j))->checkCollisionWithPointCloud(p, tol))
				return true;
		}
	return false;
}

double PlanBase::clearance(){

	//	double start = getrusage_sec();

	double min_sep=1.e10;

	for(int i=0;i<robotSelfPairs.size();i++){
		ColdetLinkPairPtr testPair = robotSelfPairs[i];
		testPair->updatePositions();
		bool coll = testPair->checkCollision();
		if(coll){
			colPairName[0] = testPair->model(0)->name();
			colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
			cout <<"self collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
			return 0;
		}
	}
	for(int i=0;i<robotObjPairWithoutHand.size();i++){
		ColdetLinkPairPtr testPair = robotObjPairWithoutHand[i];
		testPair->updatePositions();
		bool coll = testPair->checkCollision();
		if(coll){
			colPairName[0] = testPair->model(0)->name();
			colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
			cout <<"robot obj collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
			return 0;
		}
	}
	for (int i=0;i<armsList.size();i++) {
		if(checkGraspingState(i)==NOT_GRASPING){
			for(int j=0;j<handObjPair[i].size();j++){
				ColdetLinkPairPtr testPair = handObjPair[i][j];
				testPair->updatePositions();
				bool coll = testPair->checkCollision();
				if(coll){
					colPairName[0] = testPair->model(0)->name();
					colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
					cout <<"robot obj collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
					return 0;
				}
			}
		}
	}

	for(int i=0;i<robotExcludingFingPointCloudPairs.size();i++){
		bool coll = isCollidingPointCloudSub(robotExcludingFingPointCloudPairs[i].second,robotExcludingFingPointCloudPairs[i].first);
		if(coll){
#ifdef DEBUG_MODE
			cout << "robot pointcloud collide" << endl;
#endif
			return 0;
		}
	}
	if(doCheckCollisionPointCloudFinger){
		for(int i=0;i<fingPointCloudPairs.size();i++){
			bool coll = isCollidingPointCloudSub(fingPointCloudPairs[i].second,fingPointCloudPairs[i].first);
			if(coll){
#ifdef DEBUG_MODE
				cout << "robot(finger) pointcloud collide" << endl;
#endif
				return 0;
			}
		}
	}

	for(int i=0;i<robotEnvPairs.size();i++){
		ColdetLinkPairPtr testPair = robotEnvPairs[i];
		testPair->updatePositions();

		testPair->setTolerance(tolerance);
		if( testPair->detectIntersection() ){
#ifdef DEBUG_MODE
			os <<"rob-env tolerance collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
			return 0;
		}
		continue;

		int t1,t2;
		double p1[3],p2[3];
		double distance = testPair->computeDistance(t1,p1,t2,p2);
		if(distance <=tolerance){
#ifdef DEBUG_MODE
			os <<"rob-env tolerance collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
			return distance;
		}
		else {
			os << distance << endl;
		}
		if(distance < min_sep){
			min_sep = distance;
		}
	}


	if(getObjectContactState()==OFF_ENVIRONMENT){
		for(int i=0;i<objEnvPairs.size();i++){
			ColdetLinkPairPtr testPair = objEnvPairs[i];
			testPair->updatePositions();

			testPair->setTolerance(tolerance);
			if( testPair->detectIntersection() ){
#ifdef DEBUG_MODE
				os <<"rob-env tolerance collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
				return 0;
			}
			continue;

			int t1,t2;
			double p1[3],p2[3];
			double distance = testPair->computeDistance(t1,p1,t2,p2);
			if(distance <=tolerance){
#ifdef DEBUG_MODE
				os <<"obj-env tolerance collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
				return distance;
			}
			else {
				os << distance << endl;
			}
			if(distance < min_sep){
				min_sep = distance;
			}
		}

		for(int i=0;i<objPointCloudPairs.size();i++){
			bool coll = isCollidingPointCloudSub(objPointCloudPairs[i].second, objPointCloudPairs[i].first);
			if(coll){
#ifdef DEBUG_MODE
				cout << "obj pointcloud collide" << endl;
#endif
				return 0;
			}
		}
	}
	//	double end = getrusage_sec();
	//	cout << "time clearance" << objEnvPairs.size() << " "<< end - start << endl;


	double distance = hand_obj_colchecker->clearance(tolerance);
	if (distance < min_sep) {
		min_sep = distance;
	}

	return min_sep;
}

double PlanBase::roughClearance(bool initial){
	double rclearance = 1;

	for(int i=0;i<robotSelfPairs.size();i++){
		ColdetLinkPairPtr testPair = robotSelfPairs[i];
		testPair->updatePositions();
		bool coll = testPair->checkCollision();
		if(coll){
			colPairName[0] = testPair->model(0)->name();
			colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
			cout <<"self collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
			return 0;
		}
	}

	for(int i=0;i < roughClearanceRobotEnv.size(); i++){
		double temp = roughClearanceRobotEnv[i].roughClearance(initial);
		if(temp==0) return 0;
		rclearance *= temp*10;
	}
	if(targetObject){
		for(int i=0;i < roughClearanceRobotHandFreeObj.size(); i++){
			double temp = roughClearanceRobotHandFreeObj[i].roughClearance(initial);
			if(temp==0) return 0;
			rclearance *= temp*10;
		}
		if(getGraspingStateMainArm()==NOT_GRASPING){
			for(int i=0;i < roughClearanceHandObj.size(); i++){
				double temp = roughClearanceHandObj[i].roughClearance(initial);
				if(temp==0) return 0;
				rclearance *= temp*10;
			}
		}
		if(getObjectContactState()==OFF_ENVIRONMENT){
			double temp = roughClearanceObjEnv[0].roughClearance(initial);
			if(temp==0) return 0;
			rclearance *= temp*10;
		}
	}
	return rclearance;
}

double PlanBase::sweepRoughClearanceFast(bool initial){
	static MotionState m0, m1;
	double rclearance = 1;
	
	int id = (initial) ? 0: 1;
	for(int i=0;i < roughClearanceRobotEnv.size(); i++){
		roughClearanceRobotEnv[i].sweepRoughClearanceFast(id);
		if(targetObject){
			for(int i=0;i < roughClearanceRobotHandFreeObj.size(); i++){
				roughClearanceRobotHandFreeObj[i].sweepRoughClearanceFast(id);
			}
			if(getGraspingStateMainArm()==NOT_GRASPING){
				for(int i=0;i < roughClearanceHandObj.size(); i++){
					roughClearanceHandObj[i].sweepRoughClearanceFast(id);
				}
			}
			if(getObjectContactState()==OFF_ENVIRONMENT){
				roughClearanceObjEnv[0].sweepRoughClearanceFast(id);
			}
		}
	}
	
	if(initial){
		m0 = getMotionState();
		return roughClearance(false);
	}
	else{
		m1 = getMotionState();
		rclearance = roughClearance(false);
	}
	
	double delta=1.0;	
	for(int level=1;level<8;level++){
		for(double t=delta/2.0;t<1.0;t+=delta){
			id++;
			rclearance = 1;
			MotionState mt= m0;
			mt.linInterpol(t,&m0,&m1);
			setMotionState(mt);
			//cout <<id << " " <<t << "\t"<< endl;
			
			for(int i=0;i < roughClearanceRobotEnv.size(); i++){
				double temp = roughClearanceRobotEnv[i].sweepRoughClearanceFast(id);
				if(temp==0) return 0;
				rclearance *= temp*10;
			}
			if(targetObject){
				for(int i=0;i < roughClearanceRobotHandFreeObj.size(); i++){
					double temp = roughClearanceRobotHandFreeObj[i].sweepRoughClearanceFast(id);
					if(temp==0) return 0;
					rclearance *= temp*10;
				}
				if(getGraspingStateMainArm()==NOT_GRASPING){
					for(int i=0;i < roughClearanceHandObj.size(); i++){
						double temp = roughClearanceHandObj[i].sweepRoughClearanceFast(id);
						if(temp==0) return 0;
						rclearance *= temp*10;
					}
				}
				if(getObjectContactState()==OFF_ENVIRONMENT){
					double temp = roughClearanceObjEnv[0].sweepRoughClearanceFast(id);
					if(temp==0) return 0;
					rclearance *= temp*10;
				}
			}
		}
		delta /= 2.0;
	}
	rclearance = 1;
	for(int i=0;i < roughClearanceRobotEnv.size(); i++){
		double temp = roughClearanceRobotEnv[i].sweepRoughClearanceFastTest();
		if(temp==0) return 0;
		rclearance *= temp*10;
	}
	if(targetObject){
		for(int i=0;i < roughClearanceRobotHandFreeObj.size(); i++){
			double temp = roughClearanceRobotHandFreeObj[i].sweepRoughClearanceFastTest();
			if(temp==0) return 0;
			rclearance *= temp*10;
		}
		if(getGraspingStateMainArm()==NOT_GRASPING){
			for(int i=0;i < roughClearanceHandObj.size(); i++){
				double temp = roughClearanceHandObj[i].sweepRoughClearanceFastTest();
				if(temp==0) return 0;
				rclearance *= temp*10;
			}
		}
		if(getObjectContactState()==OFF_ENVIRONMENT){
			double temp = roughClearanceObjEnv[0].sweepRoughClearanceFastTest();
			if(temp==0) return 0;
			rclearance *= temp*10;
		}
	}
	
	
	return rclearance;
}

double PlanBase::sweepRoughClearance(bool initial){
	if(initial){
		return roughClearance(false);
	}
	
	double rclearance = 1;

	for(int i=0;i < roughClearanceRobotEnv.size(); i++){
		double temp = roughClearanceRobotEnv[i].sweepRoughClearance();
		if(temp==0) return 0;
		rclearance *= temp*10;
	}
	if(targetObject){
		for(int i=0;i < roughClearanceRobotHandFreeObj.size(); i++){
			double temp = roughClearanceRobotHandFreeObj[i].sweepRoughClearance();
			if(temp==0) return 0;
			rclearance *= temp*10;
		}
		if(getGraspingStateMainArm()==NOT_GRASPING){
			for(int i=0;i < roughClearanceHandObj.size(); i++){
				double temp = roughClearanceHandObj[i].sweepRoughClearance();
				if(temp==0) return 0;
				rclearance *= temp*10;
			}
		}
		if(getObjectContactState()==OFF_ENVIRONMENT){
			double temp = roughClearanceObjEnv[0].sweepRoughClearance();
			if(temp==0) return 0;
			rclearance *= temp*10;
		}
	}
	return rclearance;
}

int PlanBase::getArmID(const ArmPtr& arm) const {
	for (size_t i = 0; i < armsList.size(); i++) {
		if (armsList[i]->arm == arm) return i;
	}
	return -1;
}

void PlanBase::setGraspingStateMainArm(int state) {
	int arm_id = getArmID(arm());
	setGraspingState(arm_id, state);
}

void PlanBase::setGraspingStateSubArm(int state) {
	if (armsList.size() < 2)  return;
	int arm_id = (getArmID(arm()) == 0) ? 1 : 0;
	setGraspingState(arm_id, state);
}

void PlanBase::setGraspingState(int state){
	setGraspingState(0, state);
}

void PlanBase::setGraspingState2(int state){
	if (armsList.size() < 2)  return;
	setGraspingState(1, state);
}

void PlanBase::setGraspingState(int i, int state) {
	if (state == GRASPING) {
		if (armsList[i]->arm->target_grasp_objid < 0) { // grasping target object
			if (nFing(i) > 0) {
				armsList[i]->objectPalmPos = trans(Matrix3(fingers(i, 0)->tip->R())) * (object()->p() - fingers(i, 0)->tip->p());
				armsList[i]->objectPalmRot = trans(Matrix3(fingers(i, 0)->tip->R())) * object()->R();
			} else {
				armsList[i]->objectPalmPos = trans(Matrix3(palm(i)->R())) * (object()->p() - palm(i)->p());
				armsList[i]->objectPalmRot = trans(Matrix3(palm(i)->R())) * object()->R();
			}
		}
	}
	robotBody_->getRobotArmFingers(i)->setGraspingState(state);
	armsList[i]->arm->graspingState = state;
}

int PlanBase::checkGraspingState(int arm_id){
	return getGraspingState(arm_id);
	// return ((arm_id == 0) ? graspingState : graspingState2);
}

int PlanBase::getGraspingStateMainArm() {
	int arm_id = getArmID(arm());
	return getGraspingState(arm_id);
}

int PlanBase::getGraspingStateSubArm() {
	if (armsList.size() < 2)  return Arm::NOT_GRASPING;
	int arm_id = (getArmID(arm()) == 0) ? 1 : 0;
	return getGraspingState(arm_id);
}

int PlanBase::getGraspingState() {
	return getGraspingState(0);
}

int PlanBase::checkAllGraspingState() {
	int state = getGraspingState(0);
	for (size_t i = 1; i < armsList.size(); i++) {
		int tmp_state = getGraspingState(i);
		if (state > tmp_state) state = tmp_state;
	}
	return state;
}

int PlanBase::getGraspingState2() {
	if (armsList.size() < 2)  return Arm::NOT_GRASPING;
	return getGraspingState(1);
}

int PlanBase::getGraspingState(int i) const {
	return armsList[i]->arm->graspingState;
}

ObjectsState PlanBase::getHandObjectState() const {
	ObjectsState ret;
	obj_manager_->storeState(ret);
	return ret;
}

void PlanBase::setHandObjectState(const ObjectsState& state) {
	obj_manager_->restoreState(state);
}

void PlanBase::setTrajectoryPlanDOF(){

	pathPlanDOF.clear();

	for(int i=0; i<arm()->nJoints; i++)
		pathPlanDOF.push_back(arm()->arm_path->joint(i)->jointId());
	for(int i=0; i<nFing(); i++)
		for(int j=0; j<fingers(i)->nJoints; j++)
			pathPlanDOF.push_back(fingers(i)->fing_path->joint(j)->jointId());

#ifdef DEBUG_MODE
	cout << "Plan DOF= ";
	for (unsigned int i=0; i<pathPlanDOF.size(); i++)
		cout << pathPlanDOF[i] << " ";	cout << endl;
#endif
}

void PlanBase::setTrajectoryPlanDOF(int k){

	pathPlanDOF.clear();

	for(int i=0; i<arm(k)->nJoints; i++)
		pathPlanDOF.push_back(arm(k)->arm_path->joint(i)->jointId());
	for(int i=0; i<nFing(k); i++)
		for(int j=0; j<fingers(k,i)->nJoints; j++)
			pathPlanDOF.push_back(fingers(k,i)->fing_path->joint(j)->jointId());

#ifdef DEBUG_MODE
	cout << "Plan DOF= ";
	for (unsigned int i=0; i<pathPlanDOF.size(); i++)
		cout << pathPlanDOF[i] << " ";	cout << endl;
#endif
}

void PlanBase::setTrajectoryPlanMapDOF(){

	pathPlanDOF.clear();

	int top = robotBody()->numJoints();
	pathPlanDOF.push_back(top); //position x
	pathPlanDOF.push_back(top+1); //position y;
	pathPlanDOF.push_back(top+5); //yaw;

	//ulimitMap = Vector3(3.0,3.0,3.0);
	//llimitMap = Vector3(-3.0,-3.0,-3.0);

#ifdef DEBUG_MODE
	cout << "Plan Map DOF= ";
	for (unsigned int i=0; i<pathPlanDOF.size(); i++)
		cout << pathPlanDOF[i] << " ";	cout << endl;
#endif
}

void PlanBase::removeColdetPairData(BodyItemPtr bodyitem) {
	// This method should be invoked before deleting BodyItem
	removeColdetPairDataProc(coldetPairData, bodyitem);
	removeColdetPairDataProc(safeColdetPairData, bodyitem);
	removeColdetPairDataProc(coldetPairDataRobotObj, bodyitem);
	for (size_t i = 0; i < coldetPairDataHandObj.size(); i++) {
		removeColdetPairDataProc(coldetPairDataHandObj[i], bodyitem);
	}
}


double PlanBase::calcContactPoint(ColdetLinkPairPtr cPair, Vector3 &Po, Vector3 &Pf, Vector3 &objN, Vector3 &fingerN) {

	//	int Ik = t;

	double p1[3] = {0}, p2[3] = {0};
	int tid1, tid2;

	//	ColdetLinkPairPtr cPair = linkObjPair[Ik];
	//	SgNodePtr model = cPair->model(0);
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	cPair->model(0)->setPosition(cPair->link(0)->R(), cPair->link(0)->p());
	cPair->model(1)->setPosition(cPair->link(1)->R(), cPair->link(1)->p());
#else
	cPair->model(0)->setPosition(cPair->link(0)->T());
	cPair->model(1)->setPosition(cPair->link(1)->T());
#endif
	//	cout << cPair->link(0)->p << cPair->link(1)->p << endl;

	double dsn = cPair->computeDistance(tid1, &p1[0], tid2, &p2[0]);


	Link* links[2];
	links[0] = cPair->link(0);
	links[1] = cPair->link(1);

	int v[2][3];
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	links[0]->coldetModel()->getTriangle(tid1, v[0][0], v[0][1], v[0][2]);
	links[1]->coldetModel()->getTriangle(tid2, v[1][0], v[1][1], v[1][2]);
#else
	cPair->model(0)->getTriangle(tid1, v[0][0], v[0][1], v[0][2]);
	cPair->model(1)->getTriangle(tid2, v[1][0], v[1][1], v[1][2]);
#endif

	float p[3];
	Vector3 n[3];

	for (int i = 1; i < 2;i++) {
		for (int j = 0; j < 3;j++) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
			links[i]->coldetModel()->getVertex(v[i][j], p[0], p[1], p[2]);
#else
			cPair->model(i)->getVertex(v[i][j], p[0], p[1], p[2]);
#endif
			n[j] = Vector3(p[0], p[1], p[2]);
		}
	}

	Pf = Vector3(p1[0], p1[1], p1[2]);
	Po = Vector3(p2[0], p2[1], p2[2]);
	//	cout << Po << Pf << endl;

	//Po = trans(cPair->link(1)->R) * Po - cPair->link(1)->p; //bug? ochi
	Po = trans(Matrix3(cPair->link(1)->R())) * (Po - cPair->link(1)->p());
	//	alias(Pf) = trans(cPair->link(0)->R) * Pf - cPair->link(0)->p;


	Vector3 objN2 = cross(Vector3(n[1] - n[0]), Vector3(n[2] - n[0]));
	objN = objN2 / norm2(objN2);

	for (int j = 0; j < 3;j++) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		links[0]->coldetModel()->getVertex(v[0][j], p[0], p[1], p[2]);
#else
		cPair->model(0)->getVertex(v[0][j], p[0], p[1], p[2]);
#endif
		n[j] = Vector3 (p[0], p[1], p[2]);
	}

	objN2 = cross(Vector3(n[1] - n[0]), Vector3(n[2] - n[0]));
	//normal vector of finger in the object local corrdinate
	fingerN = trans(Matrix3(cPair->link(1)->R())) * (cPair->link(0)->R()) * (objN2 / norm2(objN2));

	return dsn;

}

Matrix3 makeOrthogonal(const MatrixXd A, const VectorXd b)
{
	vector<double> c;
	for(size_t i=0; i<b.size(); i++)
		c.push_back(fabs(b(i)));

	Matrix3 A_ = d2v(A);
	int j=argmax(c);
	Vector3 v0 = A_.col(j);
	Vector3 v1 = A_.col((j+1)%3);

	double r=dot(v0,v1);
	v1 = (-r*v0 + v1)/sqrt(1-r*r);
	Vector3 v2 = cross(v0,v1);

	for(int i=0; i<3; i++){
		A_(i, (j+1)%3) = v1(i);
		A_(i, (j+2)%3) = v2(i);
	}

	return A_;
}

void PlanBase::calcBoundingBox(ColdetModelPtr model, Vector3 &edge, Vector3& center, Vector3& com, Matrix3& Rot) {

	//objVis and objPos shoulde be defined in advance.
	class Triangle{
	public:
		cnoid::Vector3 ver[3];
		float area;
	};

	// convert coldetmodel to objectshape
	float out_x, out_y, out_z;
	int v0,v1,v2;

	int nVerticies = model->getNumVertices();
	int nTriangles = model->getNumTriangles();

	Vector3* verticies = new Vector3[nVerticies];
	Triangle* triangles = new Triangle[nTriangles];

	for(int i=0;i<nVerticies;i++){
		model->getVertex(i, out_x, out_y, out_z);
		verticies[i][0] = out_x;
		verticies[i][1] = out_y;
		verticies[i][2] = out_z;
	}

	for(int i=0;i<nTriangles;i++){
		model->getTriangle(i, v0,v1,v2);
		triangles[i].ver[0] = verticies[v0];
		triangles[i].ver[1] = verticies[v1];
		triangles[i].ver[2] = verticies[v2];
	}

	// calc distribution
	Vector3 pt;
	MatrixXd distribute = MatrixXd::Zero(3, 3);
	Vector3 average(0, 0, 0);

	for(int i=0;i<nTriangles;i++){
		Vector3 e1 (triangles[i].ver[1] - triangles[i].ver[0]);
		Vector3 e2 (triangles[i].ver[2] - triangles[i].ver[0]);
		triangles[i].area = norm2 ( cross(e1,e2) ) /2.0;
	}

	Matrix3 aq,aq_n_sum,aq_p_sum;
	std::vector<double>* aq_p[3][3];
	std::vector<double>* aq_n[3][3];
	Vector3 sumCenter(0,0,0);
	double sumArea=0;
	for(int i=0;i<3;i++) for(int j=0;j<3;j++) {
		aq(i,j)=0;
		aq_p_sum(i,j)=0;
		aq_n_sum(i,j)=0;
		aq_p[i][j] = new std::vector<double>();
		aq_n[i][j] = new std::vector<double>();
	}

	for(int l=0;l<nTriangles;l++){
		Triangle& t = triangles[l];
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				for(int k=0;k<3;k++){
					double tmp_aq;
					tmp_aq = t.area*(t.ver[i][j] * t.ver[i][k])/6.0;
					tmp_aq > 0 ? aq_p[j][k]->push_back(tmp_aq) : aq_n[j][k]->push_back(tmp_aq);
					tmp_aq = t.area*(t.ver[i][j] * t.ver[(i+1)%3][k])/12.0;
					tmp_aq > 0 ? aq_p[j][k]->push_back(tmp_aq) : aq_n[j][k]->push_back(tmp_aq);
					tmp_aq = t.area*(t.ver[(i+1)%3][j] * t.ver[i][k])/12.0;
					tmp_aq > 0 ? aq_p[j][k]->push_back(tmp_aq) : aq_n[j][k]->push_back(tmp_aq);
				}
			}
		}
		sumArea +=t.area;
		sumCenter  =  sumCenter + t.area/3.0* Vector3 ( t.ver[0] + t.ver[1] + t.ver[2]);
	}
	for(int i=0;i<3;i++) for(int j=0;j<3;j++) {
		std::sort(aq_p[i][j]->begin(),aq_p[i][j]->end(),LessAbs());
		for(int n=0;n<aq_p[i][j]->size();n++){
			aq_p_sum(i,j) += aq_p[i][j]->at(n);
		}
		std::sort(aq_n[i][j]->begin(),aq_n[i][j]->end(),LessAbs());
		for(int n=0;n<aq_n[i][j]->size();n++){
			aq_n_sum(i,j) += aq_n[i][j]->at(n);
		}
		delete aq_p[i][j];
		delete aq_n[i][j];
	}
	for(int i=0;i<3;i++) for(int j=0;j<3;j++) {aq(i,j) = aq_p_sum(i,j) + aq_n_sum(i,j);}
	average = com = sumCenter/sumArea;
	for(int j=0;j<3;j++){
		for(int k=0;k<3;k++){
			distribute(j,k) = aq(j,k) - com[j] * com[k] * sumArea;
		}
	}
	MatrixXd evec(3, 3);
	VectorXd eval(3);
	int info;
	Eigen::EigenSolver<MatrixXd> es(distribute);
	eval = es.eigenvalues().real();
	evec = es.eigenvectors().real();

	Rot = makeOrthogonal(evec, eval);
	//Rot =  (d2v(evec));

	Vector3 e[3];
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			e[j][i] = Rot(i, j);

	Vector3 pt_max(0, 0, 0), pt_min(0, 0, 0);

	for(int l=0;l<nVerticies;l++){
		Vector3 pt = verticies[l];
		for (int j = 0; j < 3; j++) {
			double tmp = dot(e[j], Vector3(pt - average));
			if (tmp > pt_max[j]) pt_max[j] = tmp;
			if (tmp < pt_min[j]) pt_min[j] = tmp;
		}
	}

	//Rot =  (d2v(evec));

	edge =  (pt_max - pt_min);
	center =  average + 0.5 * Rot * (pt_max + pt_min);
	//	com = average;

	//	alias(Rot)  = objVisRot() * Rot;
	//	alias(center) = objVisRot() * center + objVisPos();
	//	alias(com)    = objVisRot() * com   + objVisPos();
#ifdef DEBUG_MODE
	cout << "bouding box size"<< edge.transpose() << endl<< Rot  <<endl;
#endif

	delete []	verticies;
	delete []  triangles;


	return;
}

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
void PlanBase::calcBoundingBox(SgNode* model, Vector3 &edge, Vector3& center, Vector3& com, Matrix3& Rot) {

	//objVis and objPos shoulde be defined in advance.
	class Triangle{
	public:
		cnoid::Vector3 ver[3];
		float area;
	};

	// convert coldetmodel to objectshape
	SgMeshPtr mesh = ColdetConverter::ExtractMesh(model);
	SgVertexArrayPtr meshVertices = mesh->vertices();
	SgIndexArray indices = mesh->triangleVertices();

	int nVerticies = meshVertices->size();
	int nTriangles = mesh->numTriangles();

	Vector3* verticies = new Vector3[nVerticies];
	Triangle* triangles = new Triangle[nTriangles];

	for(int i = 0; i < nVerticies; i++){
		Vector3f vec = meshVertices->at(i);
		verticies[i][0] = vec[0];
		verticies[i][1] = vec[1];
		verticies[i][2] = vec[2];
	}

	for(int i = 0; i < nTriangles; i++){
		triangles[i].ver[0] = verticies[indices[i * 3]];
		triangles[i].ver[1] = verticies[indices[i * 3 + 1]];
		triangles[i].ver[2] = verticies[indices[i * 3 + 2]];
	}

	// calc distribution
	Vector3 pt;
	MatrixXd distribute = MatrixXd::Zero(3, 3);
	Vector3 average(0, 0, 0);

	for(int i=0;i<nTriangles;i++){
		Vector3 e1 (triangles[i].ver[1] - triangles[i].ver[0]);
		Vector3 e2 (triangles[i].ver[2] - triangles[i].ver[0]);
		triangles[i].area = norm2 ( cross(e1,e2) ) /2.0;
	}

	Matrix3 aq,aq_n_sum,aq_p_sum;
	std::vector<double>* aq_p[3][3];
	std::vector<double>* aq_n[3][3];
	Vector3 sumCenter(0,0,0);
	double sumArea=0;
	for(int i=0;i<3;i++) for(int j=0;j<3;j++) {
		aq(i,j)=0;
		aq_p_sum(i,j)=0;
		aq_n_sum(i,j)=0;
		aq_p[i][j] = new std::vector<double>();
		aq_n[i][j] = new std::vector<double>();
	}

	for(int l=0;l<nTriangles;l++){
		Triangle& t = triangles[l];
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				for(int k=0;k<3;k++){
					double tmp_aq;
					tmp_aq = t.area*(t.ver[i][j] * t.ver[i][k])/6.0;
					tmp_aq > 0 ? aq_p[j][k]->push_back(tmp_aq) : aq_n[j][k]->push_back(tmp_aq);
					tmp_aq = t.area*(t.ver[i][j] * t.ver[(i+1)%3][k])/12.0;
					tmp_aq > 0 ? aq_p[j][k]->push_back(tmp_aq) : aq_n[j][k]->push_back(tmp_aq);
					tmp_aq = t.area*(t.ver[(i+1)%3][j] * t.ver[i][k])/12.0;
					tmp_aq > 0 ? aq_p[j][k]->push_back(tmp_aq) : aq_n[j][k]->push_back(tmp_aq);
				}
			}
		}
		sumArea +=t.area;
		sumCenter  =  sumCenter + t.area/3.0* Vector3 ( t.ver[0] + t.ver[1] + t.ver[2]);
	}
	for(int i=0;i<3;i++) for(int j=0;j<3;j++) {
		std::sort(aq_p[i][j]->begin(),aq_p[i][j]->end(),LessAbs());
		for(int n=0;n<aq_p[i][j]->size();n++){
			aq_p_sum(i,j) += aq_p[i][j]->at(n);
		}
		std::sort(aq_n[i][j]->begin(),aq_n[i][j]->end(),LessAbs());
		for(int n=0;n<aq_n[i][j]->size();n++){
			aq_n_sum(i,j) += aq_n[i][j]->at(n);
		}
		delete aq_p[i][j];
		delete aq_n[i][j];
	}
	for(int i=0;i<3;i++) for(int j=0;j<3;j++) {aq(i,j) = aq_p_sum(i,j) + aq_n_sum(i,j);}
	average = com = sumCenter/sumArea;
	for(int j=0;j<3;j++){
		for(int k=0;k<3;k++){
			distribute(j,k) = aq(j,k) - com[j] * com[k] * sumArea;
		}
	}
	MatrixXd evec(3, 3);
	VectorXd eval(3);
	int info;
	Eigen::EigenSolver<MatrixXd> es(distribute);
	eval = es.eigenvalues().real();
	evec = es.eigenvectors().real();

	Rot = makeOrthogonal(evec, eval);
	//Rot =  (d2v(evec));

	Vector3 e[3];
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			e[j][i] = Rot(i, j);

	Vector3 pt_max(0, 0, 0), pt_min(0, 0, 0);

	for(int l=0;l<nVerticies;l++){
		Vector3 pt = verticies[l];
		for (int j = 0; j < 3; j++) {
			double tmp = dot(e[j], Vector3(pt - average));
			if (tmp > pt_max[j]) pt_max[j] = tmp;
			if (tmp < pt_min[j]) pt_min[j] = tmp;
		}
	}

	//Rot =  (d2v(evec));

	edge =  (pt_max - pt_min);
	center =  average + 0.5 * Rot * (pt_max + pt_min);
	//	com = average;

	//	alias(Rot)  = objVisRot() * Rot;
	//	alias(center) = objVisRot() * center + objVisPos();
	//	alias(com)    = objVisRot() * com   + objVisPos();
#ifdef DEBUG_MODE
	cout << "bouding box size"<< edge.transpose() << endl<< Rot  <<endl;
#endif

	delete []	verticies;
	delete []  triangles;


	return;
}
#endif

//Including Enveloping grasp
//bool PlanBase::sampleFinalPos2(mpkRobotCollection* robots, vector<mpkCollPair> *test_pairs, int iterate)
bool PlanBase::initial() {
	if ( !targetObject || !targetArmFinger) {
		os << "Please select Grasped Object and Grasping Robot" << endl;
		return false;
	}

	RemoveEnvironment(targetObject->bodyItemObject);

	targetObject->objVisPos = object()->p();
	targetObject->objVisRot = object()->R();

	setGraspingState(NOT_GRASPING);
	setGraspingState2(NOT_GRASPING);
	graspMotionSeq.push_back ( getMotionState() );

	initialCollision();

	_initrand();
	return true;
}

void PlanBase::initialCollisionAlwaysRemake(){

	if(!doInitialCollision) return;

	robotEnvPairs.clear();
	robotObjPairs.clear();
	robotObjPairWithoutHand.clear();
	handObjPair.clear();
	objEnvPairs.clear();

	if(targetArmFinger==NULL) {
		return;
	}
	for(unsigned int j=0;j<bodyItemRobot()->body()->numLinks();j++){
		for( list<BodyItemPtr>::iterator it = bodyItemEnv.begin(); it !=bodyItemEnv.end(); it++){
			for(unsigned int i=0;i<(*it)->body()->numLinks();i++){
#ifdef  CNOID_10_11_12_13
				ColdetLinkPairPtr temp= new ColdetLinkPair(bodyItemRobot()->body()->link(j), (*it)->body()->link(i));
#else
				ColdetLinkPairPtr temp = boost::make_shared<ColdetLinkPair>(bodyItemRobot()->body(),bodyItemRobot()->body()->link(j), (*it)->body(), (*it)->body()->link(i) );
#endif
				temp->updatePositions();
				int t1,t2;
				double p1[3],p2[3];
				double distance = temp->computeDistance(t1,p1,t2,p2);
				if(distance>1.0e-04)	robotEnvPairs.push_back(temp);
#ifdef DEBUG_MODE
				else os <<"collide on initial condition robot and env"  <<distance <<" "<< temp->model(0)->name() <<" " << (*it)->body()->name() << endl;
#endif
			}
		}
	}
	if(targetObject){
		for(unsigned int j=0;j<bodyItemRobot()->body()->numLinks();j++){
#ifdef  CNOID_10_11_12_13
			robotObjPairs.push_back(new ColdetLinkPair(bodyItemRobot()->body()->link(j), object() ));
#else
			robotObjPairs.push_back(boost::make_shared<ColdetLinkPair>(bodyItemRobot()->body(),bodyItemRobot()->body()->link(j),targetObject->bodyItemObject->body(),object()));
#endif


		}
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		ColdetModelPtr backup = object()->coldetModel();
		if(useObjectSafeBoundingBox){
			object()->setColdetModel(targetObject->safeBoundingBox);
		}
#else
		SgNodePtr backup = object()->collisionShape();
		if(useObjectSafeBoundingBox){
			object()->setCollisionShape(targetObject->safeBoundingBox);
		}
#endif
		for(list<cnoid::BodyItemPtr>::iterator it = bodyItemEnv.begin(); it !=bodyItemEnv.end();it++){
			for(unsigned int i=0;i<(*it)->body()->numLinks();i++){
#ifdef  CNOID_10_11_12_13
				objEnvPairs.push_back(new ColdetLinkPair(object(), (*it)->body()->link(i)));
#else
				objEnvPairs.push_back( boost::make_shared<ColdetLinkPair>(targetObject->bodyItemObject->body(), object(), (*it)->body(), (*it)->body()->link(i)));
#endif
			}
		}
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		object()->setColdetModel(backup);
#else
		object()->setCollisionShape(backup);
#endif
	}

	initialCollisionPointCloud(true, (targetObject != NULL));
	for(int i=0;i<interObjectList.size();i++) interObjectList[i].initialCollision();
	if(targetObject){
		hand_obj_colchecker->initialCollisionAlwaysRemake(bodyItemEnv, pointCloudEnv);
	}
}

void PlanBase::initialCollision(){

	//static vector<ColdetPairData*> coldetPairData;
	robotEnvPairs.clear();
	safeRobotEnvPairs.clear();
	robotObjPairs.clear();
	robotObjPairWithoutHand.clear();
	handObjPair.clear();
	objEnvPairs.clear();

	//add the <robot, env> collision pairs
	if(targetArmFinger) {
		//standard collision detection models
		initialCollisionProc(bodyItemRobot(), bodyItemEnv, coldetPairData, robotEnvPairs);

		//fat collision detection models
		//bodyitem1 is blowed up with 0.03m
		initialCollisionProc(bodyItemRobot(), bodyItemEnv, safeColdetPairData, safeRobotEnvPairs, false, true);
	}

	//add the <robot, object> collision pairs
	if(targetObject) {
		if(targetArmFinger) {
			std::list<BodyItemPtr> objectBodyItem;
			objectBodyItem.push_back(targetObject->bodyItemObject);
			handObjPair.resize(armsList.size());
			if (coldetPairDataHandObj.size() < armsList.size()) coldetPairDataHandObj.resize(armsList.size());
			vector<Link*> link_exclude_hand;
			vector<vector<Link*> > hand_links(armsList.size());
			robotBody()->obtainBodyAndHandsLink(link_exclude_hand, hand_links);

			initialCollisionProc(bodyItemRobot(), link_exclude_hand, objectBodyItem,
								 coldetPairDataRobotObj, robotObjPairWithoutHand);

			for (int k = 0; k < armsList.size(); k++) {
				initialCollisionProc(bodyItemRobot(), hand_links[k], objectBodyItem,
									 coldetPairDataHandObj[k], handObjPair[k]);
			}
		}

		initialCollisionProc(targetObject->bodyItemObject, bodyItemEnv, coldetPairData, objEnvPairs);
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		if(useObjectSafeBoundingBox){
			objEnvPairs.clear();
			ColdetModelPtr backup = object()->coldetModel();
			object()->setColdetModel(targetObject->safeBoundingBox);
			for(list<cnoid::BodyItemPtr>::iterator it = bodyItemEnv.begin(); it !=bodyItemEnv.end();it++){
				for(unsigned int i=0;i<(*it)->body()->numLinks();i++){
#ifdef  CNOID_10_11_12_13
					objEnvPairs.push_back(new ColdetLinkPair(object(), (*it)->body()->link(i)));
#else
					objEnvPairs.push_back( make_shared<ColdetLinkPair>(targetObject->bodyItemObject->body(), object(), (*it)->body(), (*it)->body()->link(i)));
#endif
				}
			}
			//			object()->coldetModel() = backup;
			object()->setColdetModel(backup);
		}
#endif
	}

	initialCollisionPointCloud((targetArmFinger != NULL),
							   (targetObject != NULL));

	for(int i=0; i<interObjectList.size(); i++) {
		interObjectList[i].initialCollision();
	}

	if(targetObject) hand_obj_colchecker->initialCollision(bodyItemEnv, pointCloudEnv);

#ifdef ROUGH_CLEARANCE
	initialRoughClearance();
#endif

	os << "coldetPairData size" << coldetPairData.size() << endl;
}

void PlanBase::initialRoughClearance(){
	list <BodyItemPtr> obj;
	if(targetObject) obj.push_back( targetObject->bodyItemObject );

	roughClearanceHandObj.clear();
	roughClearanceRobotHandFreeObj.clear();

	if(targetArmFinger) {
		for(int i=0;i<roughClearanceRobotEnv.size();i++){
			roughClearanceRobotEnv[i].initialRoughClearance(bodyItemEnv);
		}
	}

	if(targetObject){
		roughClearanceObjEnv[0].initialRoughClearance(bodyItemEnv);
		if(targetArmFinger) {
			for(int j=0;j<roughClearanceRobotEnv.size();j++){
				bool find = false;
				for (int i = 0;i < nHandLink();i++){
					if( handJoint()->link(i) == roughClearanceRobotEnv[j].link ) find =true;
				}
				for (ObjectManager::hand_iterator it = obj_manager_->hand_begin(); it != obj_manager_->hand_end(); ++it) {
					RobotHand* hand = (*it);
					for (int i = 0; i < hand->nHandLink; i++) {
						if (hand->handJoint->link(i) == roughClearanceRobotEnv[j].link) find = true;
						break;
					}
					if (find) break;
				}
				if(find){
					roughClearanceHandObj.push_back(roughClearanceRobotEnv[j]);
					roughClearanceHandObj.back().initialRoughClearance(obj);
				}else{
					roughClearanceRobotHandFreeObj.push_back(roughClearanceRobotEnv[j]);
					roughClearanceRobotHandFreeObj.back().initialRoughClearance(obj);
				}
			}
		}
	}
}

/**
 * @brief PlanBase::initialCollisionPointCloud
 * @param[in] do_make_robot_pointcloud
 * a boolean value that indicates whether we make the <robot, cloud> pair
 * @param[in] do_make_obj_pointcloud
 * a boolean value that indicates whether we make the <object, cloud> pair
 */
void PlanBase::initialCollisionPointCloud(bool do_make_robot_pointcloud,
										  bool do_make_obj_pointcloud) {
	robotExcludingFingPointCloudPairs.clear();
	fingPointCloudPairs.clear();
	objPointCloudPairs.clear();
	for (list<PointCloudEnv*>::iterator it = pointCloudEnv.begin();
		 it != pointCloudEnv.end(); ++it) {
		if(do_make_robot_pointcloud) {
			for(size_t j = 0; j < bodyItemRobot()->body()->numLinks(); j++) {
				Link* target_link = bodyItemRobot()->body()->link(j);
				pair<Link*, PointCloudEnv*> link_pointcloud(target_link, *it);
				bool is_fing = false;
				for(size_t k = 0; k < armsList.size(); k++) {
					if(nFing(k) == 0 && palm(k) == target_link) {
						is_fing = true;
						break;
					} else {
						for(int n = 0; n < nHandLink(k); n++) {
							if(palm(k) != bodyItemRobot()->body()->link(j) &&
									handJoint(k)->link(n) == target_link) {
								is_fing = true;
								break;
							}
						}
					}
				}
				if (is_fing) {
					fingPointCloudPairs.push_back(link_pointcloud);
				} else {
					robotExcludingFingPointCloudPairs.push_back(link_pointcloud);
				}
			}
		}
		if(do_make_obj_pointcloud) {
			pair<Link*, PointCloudEnv*> link_pointcloud(object(), *it);
			objPointCloudPairs.push_back(link_pointcloud);
		}
	}
}

void PlanBase::setObjPos(const cnoid::Vector3& P, const cnoid::Matrix3 R){

	targetObject->objVisPos = P;
	targetObject->objVisRot = R;
	object()->p()=P;
	object()->R()=R;

	targetObject->offsetApplied = false;

	return;

}

void PlanBase::setVisOffset(const cnoid::Vector3& P)
{
	if(!targetObject->offsetApplied){
		targetObject->objVisPos += P;
		object()->p() += P;
		targetObject->offsetApplied = true;
	}
}

void PlanBase::setVisOffsetR()
{
	Vector3 y = rpyFromRot(targetObject->objVisRot);
	if( fabs(y(0))<0.1 ) y(0)=0;
	else if(fabs(y(0)-1.5708)<0.1 ) y(0) =  1.5708;
	else if(fabs(y(0)+1.5708)<0.1 ) y(0) = -1.5708;
	else if(fabs(y(0)-3.1415)<0.1 ) y(0) =  3.1415;
	else if(fabs(y(0)+3.1415)<0.1 ) y(0) = -3.1415;

	if( fabs(y(1))<0.1 ) y(1)=0;
	else if(fabs(y(1)-1.5708)<0.1 ) y(1) =  1.5708;
	else if(fabs(y(1)+1.5708)<0.1 ) y(1) = -1.5708;
	else if(fabs(y(1)-3.1415)<0.1 ) y(1) =  3.1415;
	else if(fabs(y(1)+3.1415)<0.1 ) y(1) = -3.1415;

	targetObject->objVisRot = rotFromRpy(y);
	object()->R() = rotFromRpy(y);
}

void PlanBase::removeVisOffset(const cnoid::Vector3& P)
{
	if(targetObject->offsetApplied){
		targetObject->objVisPos -= P;
		object()->p() -= P;
		targetObject->offsetApplied = false;
	}
}

void PlanBase::setInterLink(){
	//if(interLinkList.empty()) return;
	for(int i=0; i<interLinkList.size();i++){
		interLinkList[i].slave->q() = interLinkList[i].master->q() *interLinkList[i].ratio;
		if( interLinkList[i].slave->q() < interLinkList[i].slave->q_lower()) interLinkList[i].slave->q() = interLinkList[i].slave->q_lower();
		if( interLinkList[i].slave->q() > interLinkList[i].slave->q_upper()) interLinkList[i].slave->q() = interLinkList[i].slave->q_upper();
	}
	if (!robotBody_) return;
	for (int arm_id = 0; arm_id < robotBody_->armSize(); arm_id++) {
		for (int hand_id = 0; hand_id < robotBody_->handListSize(arm_id); hand_id++) {
			RobotHand* hand = robotBody_->getRobotArmFingers(arm_id)->getRobotHand(hand_id);
			for (int i = 0; i < hand->interLinkList.size(); i++) {
				hand->interLinkList[i].slave->q() = hand->interLinkList[i].master->q() * hand->interLinkList[i].ratio;
				if( hand->interLinkList[i].slave->q() < hand->interLinkList[i].slave->q_lower()) hand->interLinkList[i].slave->q() = hand->interLinkList[i].slave->q_lower();
				if( hand->interLinkList[i].slave->q() > hand->interLinkList[i].slave->q_upper()) hand->interLinkList[i].slave->q() = hand->interLinkList[i].slave->q_upper();
			}
		}
	}
}

RobotBodyColChecker* PlanBase::findColChecker(const RobotBodyPtr& body) {
	for (size_t i = 0; i < col_checkers.size(); i++) {
		if (col_checkers[i]->getRobotBody() == body) {
			return col_checkers[i];
		}
	}
	return NULL;
}

RobotBodyColChecker* PlanBase::findColChecker(const cnoid::BodyPtr& body) {
	for (size_t i = 0; i < col_checkers.size(); i++) {
		if (col_checkers[i]->getRobotBody()->bodyItem()->body() == body) {
			return col_checkers[i];
		}
	}
	return NULL;
}

void PlanBase::calcForwardKinematics(const RobotBodyPtr& body) {
	RobotBodyColChecker* checker = findColChecker(body);
	if (checker != NULL) {
		checker->calcForwardKinematics();
		return;
	}
	calcForwardKinematics();
}

double PlanBase::clearance(const RobotBodyPtr& body) {
	RobotBodyColChecker* checker = findColChecker(body);
	if (checker != NULL) {
		return checker->clearance();
	}
	return clearance();
}

bool PlanBase::isColliding(const RobotBodyPtr& body) {
	RobotBodyColChecker* checker = findColChecker(body);
	if (checker != NULL) {
		return checker->isColliding();
	}
	return isColliding();
}

void PlanBase::setGraspingState(const RobotBodyPtr& body, int state) {
	RobotBodyColChecker* checker = findColChecker(body);
	if (checker != NULL) {
		checker->setGraspingState(state);
	}
	setGraspingState(state);
}

void PlanBase::setGraspingState2(const RobotBodyPtr& body, int state) {
	RobotBodyColChecker* checker = findColChecker(body);
	if (checker != NULL) {
		checker->setGraspingState2(state);
	}
	setGraspingState2(state);
}

void PlanBase::setObjectContactState(const RobotBodyPtr& body, int state) {
	RobotBodyColChecker* checker = findColChecker(body);
	if (checker != NULL) {
		checker->setObjectContactState(state);
	}
	setObjectContactState(state);
}

void PlanBase::SetEnvironment(const RobotBodyPtr& body, cnoid::BodyItemPtr bodyItem) {
	RobotBodyColChecker* checker = findColChecker(body);
	if (checker != NULL) {
		checker->SetEnvironment(bodyItem);
	}
	SetEnvironment(bodyItem);
}

void PlanBase::RemoveEnvironment(const RobotBodyPtr& body, cnoid::BodyItemPtr bodyItem) {
	RobotBodyColChecker* checker = findColChecker(body);
	if (checker != NULL) {
		checker->RemoveEnvironment(bodyItem);
	}
	RemoveEnvironment(bodyItem);
}

std::list<cnoid::BodyItemPtr> PlanBase::getBodyItemEnv(const cnoid::BodyPtr& body) {
	RobotBodyColChecker* checker = findColChecker(body);
	if (checker != NULL) {
		return checker->getBodyItemEnv();
	}
	return bodyItemEnv;
}

std::string PlanBase::getColPairName(const RobotBodyPtr& body, int i) {
	RobotBodyColChecker* checker = findColChecker(body);
	if (checker != NULL) {
		return checker->colPairName[i];
	}
	return colPairName[i];
}


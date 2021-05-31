

#include "GraspPathPlanner.h"
#include "GraspPathController.h"

#include "../Grasp/GraspController.h"
#include "../PRM/TrajectoryPlanner.h"

#include <cnoid/JointPath>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/RootItem>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/MessageView>
#include <cnoid/ExecutablePath>

using namespace grasp;
using namespace std;
using namespace cnoid;

#include <time.h>
#ifndef WIN32
#include <sys/resource.h>
#endif

#ifdef WIN32
double getrusage_sec() {
	return clock();
}
#else
double getrusage_sec() {
	struct rusage t;
	struct timeval tv;
	getrusage(RUSAGE_SELF, &t);
	tv = t.ru_utime;
	return tv.tv_sec + (double)tv.tv_usec*1e-6;
}
#endif


GraspPathController* GraspPathController::instance()
{
	static GraspPathController* instance = new GraspPathController();
	return instance;
}


GraspPathController::GraspPathController() : os (MessageView::mainInstance()->cout() )
{
#ifdef WIN32
	objectBasePath = executableTopDirectory() + string("/") + "extplugin/graspPlugin/Samples/Object/" ; //path setting will be changed for Object
#else
	objectBasePath = "extplugin/graspPlugin/Samples/Object/" ; //path setting will be changed for Object
#endif

	ifstream objListFile( (objectBasePath+"objList.txt").c_str() );
	char line[1024];
	while (objListFile){
		objListFile.getline(line,1024);
		stringstream ss;
		ss << line;
		int id;
		string gp,file;
		ss >> id;
		ss >> gp;
		ss >> file;
		objId2File.insert(pair <int, string>(id,file));
		objId2PrePlan.insert(pair <int, string>(id,gp));
	}
	if(objId2File.size()==0){
		os << "Error: cannot read objList.txt" << endl;
	}

//	pair<char, int>((char)i, i)
}

int GraspPathController::RtcStart()
{
	int argc=1;
	char *argv[] = {(char *)("GraspPathPlannerComp")};

  RTC::Manager* manager;
  manager = RTC::Manager::init(argc, argv);

  // Initialize manager
  manager->init(argc, argv);

  // Set module initialization proceduer
  // This procedure will be invoked in activateManager() function.
  manager->setModuleInitProc(MyModuleInit);

  // Activate manager and register to naming service
  manager->activateManager();

  // run the manager in blocking mode
  // runManager(false) is the default.
  //  manager->runManager();

  // If you want to run the manager in non-blocking mode, do like this
  manager->runManager(true);

  return 0;
}


void GraspPathController::MyModuleInit(RTC::Manager* manager)
{

  GraspPathPlannerInit(manager);

  // Create a component
  instance()->comp_ = (GraspPathPlanner *)manager->createComponent("GraspPathPlanner");

  if (instance()->comp_ ==NULL)
  {
    std::cerr << "Component create failed." << std::endl;
    abort();
  }

  return;
}


bool GraspPathController::graspPathPlanStart(
			int mode, std::vector<double> begin, std::vector<double> end,
			std::string robotId, std::string objectTagId, double resolution,
			std::vector<pathInfo>* trajectory, int* state)
{
	*state = 0;

	PlanBase* tc = PlanBase::instance();
	if(robTag2Arm().find(robotId) == robTag2Arm().end()){
		os << "Error no robotid " << robotId << endl;
		*state = 1;
		return false;
	}
	tc->targetArmFinger = robTag2Arm()[robotId];
	tc->setTrajectoryPlanDOF();


	if(objTag2Item().find(objectTagId) == objTag2Item().end()){
		os << "Error no objectTagId " << objectTagId << endl;
		*state = 2;
		return false;
	}
	tc->SetGraspedObject(objTag2Item()[objectTagId]);
	tc->targetObject->preplanningFileName = objTag2PrePlan[objectTagId];

	if(tc->bodyItemRobot()->body()->numJoints() != (int)begin.size() ||
		tc->bodyItemRobot()->body()->numJoints() != (int)end.size() ){
		os << "Error: the number of Joints of input shoud be "<< tc->bodyItemRobot()->body()->numJoints() << endl;
		*state = 3;
		return false;
	}
	for(int i=0;i<(int)begin.size();i++){
        tc->bodyItemRobot()->body()->joint(i)->q() = begin[i];
	}

	tc->initial();
	tc->graspMotionSeq.clear();


	tc->setGraspingStateMainArm(PlanBase::NOT_GRASPING);
	tc->setObjectContactState(PlanBase::ON_ENVIRONMENT);
	tc->graspMotionSeq.push_back( tc->getMotionState() );
	tc->graspMotionSeq.back().id = planGraspPath::APROACH;
	tc->graspMotionSeq.back().tolerance = tc->tolerance;

	bool success = GraspController::instance()->loadAndSelectGraspPattern();
	if(!success){
		os << "Error: Cannot find grasping posure" << endl;
		*state = 4;
		return false;
	}

	MotionState graspMotionState = tc->getMotionState();

    Vector3 Pp_(tc->palm()->p());
    Matrix3 Rp_(tc->palm()->R());

	//tc->setGraspingState(PlanBase::UNDER_GRASPING);

	//==== Approach Point
	tc->setMotionState( graspMotionState );
	tc->arm()->IK_arm(Rp_*tc->arm()->approachOffset+Pp_+Vector3(0,0,0.05), Rp_);
	tc->setGraspingStateMainArm(PlanBase::UNDER_GRASPING);
	for(int i=0;i<tc->nFing();i++){
		for(int j=0;j<tc->fingers(i)->fing_path->numJoints();j++){
            tc->fingers(i)->fing_path->joint(j)->q() = tc->fingers(i)->fingerOpenPose[j];
		}
	}
	tc->graspMotionSeq.push_back( tc->getMotionState() );
	tc->graspMotionSeq.back().id = planGraspPath::APROACH;
	tc->graspMotionSeq.back().tolerance = 0;

	//tc->arm()->IK_arm(Vector3(Rp_*Vector3(0,-0.05,0.1)+Pp_), Rp_);
	//tc->graspMotionSeq.push_back( tc->getMotionState() );



	//==== Grasp Point and Hand open
	tc->setMotionState( graspMotionState );
	tc->setGraspingStateMainArm(PlanBase::UNDER_GRASPING);
	for(int i=0;i<tc->nFing();i++){
		for(int j=0;j<tc->fingers(i)->fing_path->numJoints();j++){
            tc->fingers(i)->fing_path->joint(j)->q() = tc->fingers(i)->fingerOpenPose[j];
		}
	}
	tc->graspMotionSeq.push_back( tc->getMotionState() );
	tc->graspMotionSeq.back().id = planGraspPath::CLOSING_GRIPPER;
	tc->graspMotionSeq.back().tolerance = 0;

	//==== hand close
	tc->setMotionState(graspMotionState) ;
	tc->setGraspingStateMainArm( PlanBase::GRASPING );
	tc->graspMotionSeq.push_back( tc->getMotionState() );
	tc->graspMotionSeq.back().id = planGraspPath::UP_HAND;
	tc->graspMotionSeq.back().tolerance = tc->tolerance;

	//==== lift up
	tc->arm()->IK_arm(Vector3(Vector3(0,0,0.05)+Pp_), Rp_);

	tc->setObjectContactState(PlanBase::OFF_ENVIRONMENT);
	tc->graspMotionSeq.push_back( tc->getMotionState() );
	tc->graspMotionSeq.back().id = planGraspPath::BACKAWAY;
	tc->graspMotionSeq.back().tolerance = tc->tolerance;

	//==== end point
	vector<double>closefinger;
	for(int i=0;i<tc->nFing();i++){
		for(int j=0;j<tc->fingers(i)->fing_path->numJoints();j++){
            closefinger.push_back(tc->fingers(i)->fing_path->joint(j)->q());
		}
	}
	for(int i=0;i<(int)begin.size();i++){
        tc->bodyItemRobot()->body()->joint(i)->q() = end[i];
	}
	int cnt=0;
	for(int i=0;i<tc->nFing();i++){
		for(int j=0;j<tc->fingers(i)->fing_path->numJoints();j++){
            tc->fingers(i)->fing_path->joint(j)->q() = closefinger[cnt++];
		}
	}
	tc->graspMotionSeq.push_back( tc->getMotionState() );



	double start1 =  getrusage_sec();

	TrajectoryPlanner tp;
	success = tp.doTrajectoryPlanning();

	double end1 =  getrusage_sec();
	cout << end1-start1 << "sec "<<endl;

	if(!success){
		os << "Error: Cannot find motion path" << endl;
		*state = 5;
		return false;
	}

/*
	const int numFrames = tp.poseSeqItemRobot->bodyMotionItem()->motion()->getNumFrames();
	MultiValueSeq& qseqRobot = *tp.poseSeqItemRobot->bodyMotionItem()->motion()->jointPosSeq();
	for(int i=0;i < numFrames; i++){
		pathInfo temp;
		temp.state= 0;
		MultiValueSeq::View qs = qseqRobot.frame(i);
		for(int j=0; j < tc->bodyItemRobot()->body()->numJoints(); j++){
			temp.pos.push_back(qs[j]);
		}
		trajectory.push_back(temp);
	}
*/
	vector<VectorXd> outputMotionSeq;
	vector<int> outputMotionId;
	vector<double> maxAngleSeq;
	double sumMaxAngle=0;
	for(unsigned int i=0;i < tp.motionSeq.size()-1;i++){
		double maxAngle = ( tp.motionSeq[i].jointSeq - tp.motionSeq[i+1].jointSeq ).cwiseAbs().maxCoeff();
		sumMaxAngle += maxAngle;
		maxAngleSeq.push_back(maxAngle);
		//cout << ( tp.motionSeq[i].jointSeq - tp.motionSeq[i-1].jointSeq ).transpose() << endl;
		//cout << ( tp.motionSeq[i].jointSeq - tp.motionSeq[i-1].jointSeq ).cwiseAbs().transpose() << endl;
		//cout << ( tp.motionSeq[i].jointSeq - tp.motionSeq[i-1].jointSeq ).cwiseAbs().maxCoeff() << endl;
	}
//	cout << endl;

//	outputMotionSeq.push_back( tp.motionSeq[0].jointSeq);
	for(unsigned int i=0;i < tp.motionSeq.size()-1;i++){
		if(maxAngleSeq[i]==0) continue;
		int cnt= resolution*maxAngleSeq[i]/sumMaxAngle;
		if(cnt==0) cnt=1;
		for(int j=0;j<cnt;j++){
			outputMotionSeq.push_back ( (tp.motionSeq[i].jointSeq*(cnt-j) + tp.motionSeq[i+1].jointSeq*j)/cnt );
			outputMotionId.push_back(tp.motionSeq[i].id);
		}
	}
	outputMotionSeq.push_back ( tp.motionSeq.back().jointSeq );
	outputMotionId.push_back(tp.motionSeq.back().id);


	for(unsigned int i=0;i < outputMotionSeq.size(); i++){
		pathInfo temp;
		temp.state= outputMotionId[i];
		temp.leftArm=false;
		temp.rightArm=false;
		temp.leftGripper=false;
		temp.rightGripper=false;
		temp.waist=false;
		for(int j=0; j < tc->bodyItemRobot()->body()->numJoints(); j++){
			temp.pos.push_back(outputMotionSeq[i][j]);
			if( i == outputMotionSeq.size()-1) continue;
			double diff = outputMotionSeq[i][j] - outputMotionSeq[i+1][j];
			if (diff == 0) continue;
			if( (0<=j) && (j<2) ) temp.waist = true;
			if( (2<=j) && (j<9) ) temp.rightArm = true;
			if  (j==9) temp.rightGripper = true;
			if( (10<=j) && (j<17) ) temp.leftArm = true;
			if  (j==17) temp.leftGripper = true;
		}
		trajectory->push_back(temp);
	}
	os << "Output Trajectory Size " << trajectory->size() << endl;

	os << "Planning Finished" << endl;
	//==== Hand open
	//tc->handJoint()->link(1)->q = 80.0*M_PI/180.0;
	//tc->handJoint()->link(2)->q = -59.0*M_PI/180.0;
	//tc->graspMotionSeq.push_back( tc->getMotionState() );

	return true;
}

bool GraspPathController::releasePathPlanStart(
			int mode, std::vector<double> begin, std::vector<double> end,
			std::string robotId, std::string objectTagId, double resolution,
			std::vector<pathInfo>* trajectory, int* state)
{
	PlanBase* tc = PlanBase::instance();

    Vector3 placePos = tc->object()->p();
    Matrix3 placeRot = tc->object()->R();
	Vector3 objectPalmPos = tc->targetArmFinger->objectPalmPos;
	Matrix3 objectPalmRot = tc->targetArmFinger->objectPalmRot;

	os << placePos << endl;

	*state = 0;
	if(robTag2Arm().find(robotId) == robTag2Arm().end()){
		os << "Error no robotid " << robotId << endl;
		*state = 1;
		return false;
	}
	tc->targetArmFinger = robTag2Arm()[robotId];
	tc->setTrajectoryPlanDOF();

	if(objTag2Item().find(objectTagId) == objTag2Item().end()){
		os << "Error no objectTagId " << objectTagId << endl;
		*state = 2;
		return false;
	}
	tc->SetGraspedObject(objTag2Item()[objectTagId]);
	tc->targetObject->preplanningFileName = objTag2PrePlan[objectTagId];
	tc->RemoveEnvironment(objTag2Item()[objectTagId]);


	if(tc->bodyItemRobot()->body()->numJoints() != (int)begin.size() ||
		tc->bodyItemRobot()->body()->numJoints() != (int)end.size() ){
		os << "Error: the number of Joints of input shoud be "<< tc->bodyItemRobot()->body()->numJoints() << endl;
		*state = 3;
		return false;
	}

	for(int i=0;i<(int)begin.size();i++){
        tc->bodyItemRobot()->body()->joint(i)->q() = begin[i];
	}
	tc->calcForwardKinematics();
	tc->flush();

	tc->setObjectContactState(PlanBase::ON_ENVIRONMENT);
	bool success = selectReleasePattern(placePos, placeRot);
	if(!success){
		os << "Error: Cannot find grasping posure" << endl;
		*state = 4;
		return false;
	}

    tc->object()->R() = tc->palm()->R()*(tc->targetArmFinger->objectPalmRot);
    tc->object()->p() = tc->palm()->p()+tc->palm()->R()*tc->targetArmFinger->objectPalmPos;
	tc->setGraspingStateMainArm(PlanBase::GRASPING);
	tc->setObjectContactState(PlanBase::ON_ENVIRONMENT);
	MotionState graspMotionState = tc->getMotionState();

    os << placePos << tc->object()->p() <<endl;


    Vector3 Pp_(tc->palm()->p());
    Matrix3 Rp_(tc->palm()->R());

	for(int i=0;i<(int)begin.size();i++){
        tc->bodyItemRobot()->body()->joint(i)->q() = begin[i];
	}
	tc->calcForwardKinematics();
	tc->flush();

    tc->object()->R() = tc->palm()->R()*(tc->targetArmFinger->objectPalmRot);
    tc->object()->p() = tc->palm()->p()+tc->palm()->R()*tc->targetArmFinger->objectPalmPos;
	tc->setObjectContactState(PlanBase::OFF_ENVIRONMENT);
	tc->setGraspingStateMainArm(PlanBase::GRASPING);
	tc->graspMotionSeq.clear();
	tc->graspMotionSeq.push_back( tc->getMotionState() );
	tc->graspMotionSeq.back().id = planGraspPath::APROACH;
	tc->graspMotionSeq.back().tolerance = tc->tolerance;


	//==== lift up point
	tc->arm()->IK_arm(Vector3(Vector3(0,0,0.05)+Pp_), Rp_);
	tc->setObjectContactState(PlanBase::ON_ENVIRONMENT);
	tc->calcForwardKinematics();
	tc->graspMotionSeq.push_back( tc->getMotionState() );
	tc->graspMotionSeq.back().id = planGraspPath::DOWN_HAND;
	tc->graspMotionSeq.back().tolerance = 0.0;

	//==== Grasp Point and hand keep close
	tc->setMotionState(graspMotionState) ;
	tc->calcForwardKinematics();
	tc->setGraspingStateMainArm(PlanBase::UNDER_GRASPING);
	tc->graspMotionSeq.push_back( tc->getMotionState() );
	tc->graspMotionSeq.back().id = planGraspPath::OPENING_GRIPPER;
	tc->graspMotionSeq.back().tolerance = 0.0;

	//==== Grasp Point and Hand open
	tc->setMotionState( graspMotionState );
	tc->setGraspingStateMainArm(PlanBase::NOT_GRASPING);
	for(int i=0;i<tc->nFing();i++){
		for(int j=0;j<tc->fingers(i)->fing_path->numJoints();j++){
            tc->fingers(i)->fing_path->joint(j)->q() = tc->fingers(i)->fingerOpenPose[j];
		}
	}
	tc->calcForwardKinematics();
	tc->graspMotionSeq.push_back( tc->getMotionState() );
	tc->graspMotionSeq.back().id = planGraspPath::BACKAWAY;
	tc->graspMotionSeq.back().tolerance = 0.0;

	tc->setMotionState( graspMotionState );
	tc->arm()->IK_arm(Rp_*tc->arm()->approachOffset+Pp_+Vector3(0,0,0.05), Rp_);
	tc->setGraspingStateMainArm(PlanBase::UNDER_GRASPING);
	for(int i=0;i<tc->nFing();i++){
		for(int j=0;j<tc->fingers(i)->fing_path->numJoints();j++){
            tc->fingers(i)->fing_path->joint(j)->q() = tc->fingers(i)->fingerOpenPose[j];
		}
	}
	tc->graspMotionSeq.push_back( tc->getMotionState() );
	tc->graspMotionSeq.back().id = planGraspPath::APROACH;
	tc->graspMotionSeq.back().tolerance = tc->tolerance;


	//==== end point
	for(int i=0;i<(int)begin.size();i++){
        tc->bodyItemRobot()->body()->joint(i)->q() = end[i];
	}
	tc->calcForwardKinematics();
	tc->setGraspingStateMainArm(PlanBase::NOT_GRASPING);
	tc->graspMotionSeq.push_back( tc->getMotionState() );

	tc->setMotionState(tc->graspMotionSeq[0]);
	tc->calcForwardKinematics();
    tc->setObjPos(tc->palm()->p()+tc->palm()->R()*objectPalmPos, tc->palm()->R()*objectPalmRot );

	TrajectoryPlanner tp;
	success = tp.doTrajectoryPlanning();
	if(!success){
		os << "Error: Cannot find motion path" << endl;
		*state = 5;
		return false;
	}

/*
	const int numFrames = tp.poseSeqItemRobot->bodyMotionItem()->motion()->getNumFrames();
	MultiValueSeq& qseqRobot = *tp.poseSeqItemRobot->bodyMotionItem()->motion()->jointPosSeq();
	for(int i=0;i < numFrames; i++){
		pathInfo temp;
		temp.state= 0;
		MultiValueSeq::View qs = qseqRobot.frame(i);
		for(int j=0; j < tc->bodyItemRobot()->body()->numJoints(); j++){
			temp.pos.push_back(qs[j]);
		}
		trajectory.push_back(temp);
	}
*/
	vector<VectorXd> outputMotionSeq;
	vector<int> outputMotionId;
	vector<double> maxAngleSeq;
	double sumMaxAngle=0;
	for(unsigned int i=0;i < tp.motionSeq.size()-1;i++){
		double maxAngle = ( tp.motionSeq[i].jointSeq - tp.motionSeq[i+1].jointSeq ).cwiseAbs().maxCoeff();
		sumMaxAngle += maxAngle;
		maxAngleSeq.push_back(maxAngle);
		//cout << ( tp.motionSeq[i].jointSeq - tp.motionSeq[i-1].jointSeq ).transpose() << endl;
		//cout << ( tp.motionSeq[i].jointSeq - tp.motionSeq[i-1].jointSeq ).cwiseAbs().transpose() << endl;
		//cout << ( tp.motionSeq[i].jointSeq - tp.motionSeq[i-1].jointSeq ).cwiseAbs().maxCoeff() << endl;
	}
//	cout << endl;

//	outputMotionSeq.push_back( tp.motionSeq[0].jointSeq);
	for(unsigned int i=0;i < tp.motionSeq.size()-1;i++){
		if(maxAngleSeq[i]==0) continue;
		int cnt= resolution*maxAngleSeq[i]/sumMaxAngle;
		if(cnt==0) cnt=1;
		for(int j=0;j<cnt;j++){
			outputMotionSeq.push_back ( (tp.motionSeq[i].jointSeq*(cnt-j) + tp.motionSeq[i+1].jointSeq*j)/cnt );
			outputMotionId.push_back(tp.motionSeq[i].id);
		}
	}
	outputMotionSeq.push_back ( tp.motionSeq.back().jointSeq );
	outputMotionId.push_back(tp.motionSeq.back().id);


	for(unsigned int i=0;i < outputMotionSeq.size(); i++){
		pathInfo temp;
		temp.state= outputMotionId[i];
		temp.leftArm=false;
		temp.rightArm=false;
		temp.leftGripper=false;
		temp.rightGripper=false;
		temp.waist=false;
		for(int j=0; j < tc->bodyItemRobot()->body()->numJoints(); j++){
			temp.pos.push_back(outputMotionSeq[i][j]);
			if( i == outputMotionSeq.size()-1) continue;
			double diff = outputMotionSeq[i][j] - outputMotionSeq[i+1][j];
			if (diff == 0) continue;
			if( (0<=j) && (j<2) ) temp.waist = true;
			if( (2<=j) && (j<9) ) temp.rightArm = true;
			if  (j==9) temp.rightGripper = true;
			if( (10<=j) && (j<17) ) temp.leftArm = true;
			if  (j==17) temp.leftGripper = true;
		}
		trajectory->push_back(temp);
	}
	os << "Output Trajectory Size " << trajectory->size() << endl;

	os << "Planning Finished" << endl;
	//==== Hand open
	//tc->handJoint()->link(1)->q = 80.0*M_PI/180.0;
	//tc->handJoint()->link(2)->q = -59.0*M_PI/180.0;
	//tc->graspMotionSeq.push_back( tc->getMotionState() );

	return true;
}

bool GraspPathController::selectReleasePattern(Vector3 objVisPos, Matrix3 objVisRot) {
	PlanBase* pb = PlanBase::instance();

	Vector3 palmPos;
	Matrix3 palmRot;

	Matrix3 rpr ( objVisRot*pb->targetArmFinger->objectPalmRot.transpose() );
	Vector3  rpp  ( objVisPos - rpr*pb->targetArmFinger->objectPalmPos );

	double Eval = 1.e10;
	bool found = false;

	vector<double> fpos, fpos_temp, fpos_cand;

	for(int i=0;i<18;i++){

		Vector3 trpy (0,0,2.0*3.141592*i/18.0);
		Matrix3 tr = rotFromRpy(trpy);

		Matrix3 tmpRot( tr*objVisRot*pb->targetArmFinger->objectPalmRot.transpose()  );
		Vector3  tmpPos( objVisPos - tmpRot*pb->targetArmFinger->objectPalmPos);

		bool ikConv = pb->arm()->IK_arm(tmpPos, tmpRot);
		if(!ikConv) continue;

		double dist = 0.0;
		for (int i = 0; i < pb->arm()->arm_path->numJoints(); i++){
            double edist =  (pb->arm()->arm_path->joint(i)->q() - pb->arm()->armStandardPose[i]);
			dist +=  edist*edist;
		}
		double quality = dist;



		if (  (Eval > quality) && ikConv && pb->arm()->checkArmLimit() && !pb->isColliding() ) {

			found = true;

			palmPos = tmpPos;
			palmRot = tmpRot;

			Eval = quality;

			cout << ikConv << " " << quality << endl;

//			oGraspPos = rpp;
//			oGraspRot = rpr;
		}
	}

	//int dim = pb->nHandLink() - 1;
	if (found) {
		pb->arm()->IK_arm(palmPos, palmRot);
//		for (int k = 1;k < pb->nHandLink();k++) pb->handJoint()->link(k)->q = fpos[k-1];
//		string HDHpos_data   = pb->dataFilePath() + "grasp_HDH.dat";
//		ofstream fHDHpos(HDHpos_data.c_str());
//		for (int k = 0;  k < dim; k++) {
//			fHDHpos << fpos[k]*180.0 / 3.1415 << " ";
//		}
//		fHDHpos << 2.0 << endl;
//		fHDHpos.close();
		pb->calcForwardKinematics();
		pb->flush();
	} else {
		os << " No feasible grasping posture found" << endl;
	}


	return found;

}

bool GraspPathController::createRecord(int objId, std::string tagId){
	string objFileName = objectBasePath + objId2File[objId];

	if(objTag2Item().find(tagId) != objTag2Item().end() || tagId=="ALLID"){
		//os << "Error: the tagId is already recorded " << tagId << endl;
		os << "Error: the tagId is already recorded " << tagId << endl;
		return false;
	}

	BodyItemPtr temp = new BodyItem();
	if( !temp->loadModelFile(objFileName) ){
		os << "modelLoadError: " << objFileName << endl;
		return false;
	}
	temp->setName(tagId);
	//temp->preplanningFileName=objId2PrePlan[objId];
	//item->setName(item->body()->modelName());
	//Item* parentItem = ItemTreeView::mainInstance()->selectedItem<Item>();
	Item* parentItem  = RootItem::mainInstance();
	parentItem->addChildItem (temp);
	objTag2Item().insert( pair <string,BodyItemPtr>(tagId, temp) );
	objTag2PrePlan.insert( pair <string,string>(tagId, objId2PrePlan[objId]) );

	return true;
}

bool GraspPathController::deleteRecord(std::string tagId){

	if(tagId=="ALLID"){
		disappear(tagId);
		map<string,BodyItemPtr>::iterator it = objTag2Item().begin();
		while( it != objTag2Item().end() ){
			it->second->detachFromParentItem();
			it++;
		}
		objTag2Item().clear();
		return true;
	}

	if(objTag2Item().find(tagId) == objTag2Item().end()){
		os << "Error: the tagId is not recorded " << tagId << endl;
		return false;
	}
	BodyItemPtr item = objTag2Item()[tagId];
	disappear(tagId);
	objTag2Item().erase(tagId);
	item->detachFromParentItem();

	return true;
}

bool GraspPathController::appear(std::string tagId){
	if(tagId=="ALLID"){
		map<string,BodyItemPtr>::iterator it = objTag2Item().begin();
		while( it != objTag2Item().end() ){
			PlanBase::instance()->SetEnvironment(it->second);
			ItemTreeView::mainInstance()->checkItem(it->second,true);
			it++;
		}
		return true;
	}
	if(objTag2Item().find(tagId) == objTag2Item().end()){
		os << "Error: the tagId is not recorded " << tagId << endl;
		return false;
	}
	BodyItemPtr item = objTag2Item()[tagId];
	PlanBase::instance()->SetEnvironment(item);
	ItemTreeView::mainInstance()->checkItem(item,true);
	return true;
}

bool GraspPathController::disappear(std::string tagId){
	if(tagId=="ALLID"){
		map<string,BodyItemPtr>::iterator it = objTag2Item().begin();
		while( it != objTag2Item().end() ){
			PlanBase::instance()->RemoveEnvironment(it->second);
			ItemTreeView::mainInstance()->checkItem(it->second,false);
			it++;
		}
		return true;
	}
	if(objTag2Item().find(tagId) == objTag2Item().end()){
		os << "Error: the tagId is not recorded " << tagId << endl;
		return false;
	}
	BodyItemPtr item = objTag2Item()[tagId];
	PlanBase::instance()->RemoveEnvironment(item);
	ItemTreeView::mainInstance()->checkItem(item,false);
	return true;
}

bool GraspPathController::setPos(string tagId,Vector3 pos, Matrix3 ori){
	BodyItemPtr item = NULL;
	if(objTag2Item().find(tagId) != objTag2Item().end()){
		item = objTag2Item()[tagId];
	}
	if( robTag2Arm().find(tagId) != robTag2Arm().end() ){
		item = robTag2Arm()[tagId]->bodyItemRobot;
	}
	if(!item){
		os << "Error: the tagId is not recorded " << tagId << endl;
		return false;
	}
    item->body()->link(0)->p() = pos;
    item->body()->link(0)->R() = ori;
	item->calcForwardKinematics();
	item->notifyKinematicStateChange();
	return true;
}

bool GraspPathController::setTolerance(double setTolerance){
	PlanBase::instance()->tolerance = setTolerance;
	return true;
}
/*
bool GraspPathController::graspPlanResult(const GraspPlanResult::DblSequence3& GraspPos, const GraspPlanResult::DblSequence9& GraspOri, const GraspPlanResult::DblSequence3& ApproachPos, const GraspPlanResult::DblSequence9& ApproachOri, CORBA::Double angle){

	PlanBase* gc = PlanBase::instance();

	Vector3 graspPos_(GraspPos[0],GraspPos[1],GraspPos[2]);
	Matrix3 graspOri_;
	graspOri_ << GraspOri[0],GraspOri[1],GraspOri[2],GraspOri[3],GraspOri[4],GraspOri[5],GraspOri[6],GraspOri[7],GraspOri[8];

	Vector3 approachPos_(ApproachPos[0],ApproachPos[1],ApproachPos[2]);
	Matrix3 approachOri_;
	approachOri_ << ApproachOri[0],ApproachOri[1],ApproachOri[2],ApproachOri[3],ApproachOri[4],ApproachOri[5],ApproachOri[6],ApproachOri[7],ApproachOri[8];


	cout << graspPos_ << graspOri_ << basePos << baseOri <<endl;


	graspPos_ = Vector3(baseOri*graspPos_ + basePos);
	graspOri_ = Matrix3(baseOri*graspOri_);

	approachPos_ = Vector3(baseOri*approachPos_ + basePos);
	approachOri_ = Matrix3(baseOri*approachOri_);


	bool ikConv = gc->arm()->IK_arm(approachPos_, approachOri_);
	bool limitCheck = gc->arm()->checkArmLimit();
	gc->fingers(0)->fing_path->joint(0)->q = angle;
	gc->bodyItemRobot()->body()->calcForwardKinematics();
	gc->flush();

	if(ikConv && limitCheck){

	}else{
		return false;
	}

	ikConv = gc->arm()->IK_arm(graspPos_, graspOri_);
	limitCheck = gc->arm()->checkArmLimit();
	gc->fingers(0)->fing_path->joint(0)->q = angle;
	gc->bodyItemRobot()->body()->calcForwardKinematics();
	gc->flush();

	if(ikConv && limitCheck){
		return true;
	}else{
		return false;
	}

}
*/

// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
#ifdef _DEBUG
#undef _DEBUG
#include <Python.h>
#define _DEBUG
#else
#include <Python.h>
#endif
#include "RobotInterface.h"
#include "ArmControllerRtc.h"

//#define USE_PCL_PLUGIN
//#define HAND_SEPARATE
//#define WRITE_RESULTS

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

#if (defined(WIN32) || defined(_WIN32))
#include <windows.h>
namespace {
	void usleep(int t) {
		Sleep(t / 1000);
	}
}
#endif

RobotInterface::RobotInterface(){
	isMulti=false;

	phms = HistoricMotionState::instance();
	pb = PlanBase::instance();

    return;
}

bool RobotInterface::runPythonCommand(string func){

	string filename = PlanBase::instance()->pythonInterface();
	FILE* file = fopen(filename.c_str(), "r");
	if(file != NULL){
//		Py_Initialize();
        PyRun_SimpleFile(file, filename.c_str());
		fclose(file);
		if( PyRun_SimpleString(func.c_str()) ){
			cout << func << endl;
			return false;
		}
//		Py_Finalize();
	}

	return true;
}

void RobotInterface::setTransMatrix()
{
	char line[1024];
#ifdef USE_PCL_PLUGIN
	string calibFile = "extplugin/graspPlugin/PCL/calibtools/calibmat.txt";
#else
	PlanBase* tc = PlanBase::instance();
	string calibFile = "extplugin/graspPlugin/RobotInterface/data/T" + tc->targetObject->bodyItemObject->name() + ".mat";
#endif

	FILE *ifp0=NULL;
	ifp0 = fopen(calibFile.c_str(),"rb");

	t1 = t2 = t3 = Vector3::Zero();
	R1 = R2 = R3 = Matrix3::Identity();

	if(ifp0 != NULL){
			if(! fgets(line,256,ifp0) )
					printf("result mat: Broken format1 \n");

			int j=0;
			while(1){

				while(fgets(line,256,ifp0) != NULL)
						if(line[0] != '#' && line[0]!=' ' && line[0]!=0x0D && line[0]!=0x0A && line[0]!='\t') break;

				if(line[0] != '4')
						printf("result mat: Broken format1 \n");

				Vector3 P0 = Vector3::Zero();
				Matrix3 R0 = Matrix3::Identity();

				int i=0;
				while(fgets(line,256,ifp0) != NULL){
						if(i<3 && sscanf(line,"%lf%lf%lf%lf",&R0(i,0),&R0(i,1),&R0(i,2),&P0(i)) != 4){
								printf("result mat: Broken format2 %d \n",i);
								fclose(ifp0);
								return;
						}
						i++;
						if(i >= 4) break;
				}
				if(j==0){R1 = R0; t1 = P0;}
				if(j==1){R2 = R0; t2 = P0;}
				if(j==2){R3 = R0; t3 = P0;}
				if(++j == 3) break;;
			}
			fclose(ifp0);
	}
}

void RobotInterface::doReadFromFile(int num) {

		char line[1024];

		PlanBase* tc = PlanBase::instance();

		Matrix3 Rm;
		Vector3 Pm;
		FILE *ifp=NULL;

		ifp = fopen("extplugin/graspPlugin/RobotInterface/data/data_cap.mat","rb");

		if(ifp==NULL){
				printf("No data.mat\n");
				return;
		}

		if(! fgets(line,256,ifp) ){
			printf("result mat: Broken format1 \n");
		}

		int count=0;
		while(1){
				while(fgets(line,256,ifp) != NULL)
						if(line[0] != '#' && line[0]!=' ' && line[0]!=0x0D && line[0]!=0x0A && line[0]!='\t') break;

				if(line[0] != '4')
						printf("result mat: Broken format1 \n");

				int i=0;
				while(fgets(line,256,ifp) != NULL){
						if(i<3 && sscanf(line,"%lf%lf%lf%lf",&Rm(i,0),&Rm(i,1),&Rm(i,2),&Pm(i)) != 4){
								printf("result mat: Broken format2 %d \n",i);
								return;
						}
						i++;
						if(i >= 4) break;
				}

				Pm /= 1000.0; //model m -> mm

				if(++count > num || isMulti) break;
		}

		setTransMatrix();

		Vector3 vu = t3 + R3*(t2 + R2*(t1 + R1*Pm));
		Matrix3 Ru = R3*R2*R1*Rm;

		tc->setObjPos(vu, Ru);

		tc->flush();

		std::cout << "Position/orientation of object was set to:" << tc->objVisPos().transpose() << endl;cout << tc->objVisRot() << endl;

		if( !isMulti) return;

		int cnt=0;
		while(fgets(line,256,ifp) != NULL){
			cout << line << endl;
			Matrix3 Rm;
			Vector3 Pm;
			if(line[0] == '4'){
				int i=0;
				while(fgets(line,256,ifp) != NULL){
					cout << line << endl;
					stringstream sline;
					sline << line;
					sline >> Rm(i,0);
					sline >> Rm(i,1);
					sline >> Rm(i,2);
					sline >> Pm(i);
					i++;
					if(i >= 3) break;
				}
				Pm /= 1000.0; //model m -> mm

				//tc->object()->p = Pm;
				//tc->object()->R = Rm;
				//tc->flush();
				ItemPtr temp= tc->targetObject->bodyItemObject->duplicateAll();
				tc->targetObject->bodyItemObject->parentItem()->addChildItem( temp);
				BodyItem* btemp = (BodyItem*)(temp.get());
				btemp->body()->link(0)->p() = t3 + R3*(t2 + R2*(t1 + R1*Pm));
				btemp->body()->link(0)->R() = R3*R2*R1*Rm;
				ItemTreeView::mainInstance()->checkItem(btemp,true);
//				btemp->notifyKinematicStateChange();
				cnt++;
//				if(cnt > 10) break;
			}
		}


		return;
}

void RobotInterface::doRecoginitionAndRead(int num) {

//		PlanBase* tc = PlanBase::instance();
//		string recogCommand = "/home/vvv/Logistics/recog_" + tc->targetObject->bodyItemObject->name() + ".sh";
//		int error = system(recogCommand.c_str());
//		int error = system("cp /home/vvv/Logistics/data_cap.mat extplugin/graspPlugin/RobotInterface/data");

		doReadFromFile(num);
}

void RobotInterface::doCapture() {

		int error = std::system("/home/vvv/Logistics/capture.sh");
		if(error) cout << error<< endl;
}

void RobotInterface::doJntCalib() {

		if(isDualArm()) {
			// first, setup robot
			try {
				controllerRtc()->manipulator()->setupRobot();
			} catch (CORBA::NO_IMPLEMENT e) {
				cerr << "setupRobot: NO_IMPLEMENT" << endl;
				throw ;
			}
			// second, calibrate the joints
			try {
				controllerRtc()->manipulator()->calibrateJoint();
			} catch (CORBA::NO_IMPLEMENT e) {
				cerr << "calibrateJoint: NO_IMPLEMENT" << endl;
				throw ;
			}
		}
}

void RobotInterface::doSrvOn() {

		if (isDualArm()){
			try {
				controllerRtc()->manipulator()->servoOn();
			} catch (CORBA::NO_IMPLEMENT e) {
				cerr << "servoOn: NO_IMPLEMENT" << endl;
				throw ;
			}
		} else if (isSingleArm()) {
			try {
				controllerRtc()->manipulator_common()->servoON();
			} catch (CORBA::NO_IMPLEMENT e) {
				cerr << "servoON: NO_IMPLEMENT" << endl;
				throw ;
			}
		}
}

void RobotInterface::doSrvOff() {

		if (isDualArm()) {
			try {
				controllerRtc()->manipulator()->servoOff();
			} catch (CORBA::NO_IMPLEMENT e) {
				cerr << "servoOff: NO_IMPLEMENT" << endl;
				throw ;
			}
		} else if (isSingleArm()) {
			try {
				controllerRtc()->manipulator_common()->servoOFF();
			} catch (CORBA::NO_IMPLEMENT e) {
				cerr << "servoOFF: NO_IMPLEMENT" << endl;
				throw;
			}
		}
}

void RobotInterface::doHome() {

	if (isDualArm()){
		try {
			controllerRtc()->manipulator()->goInitial();
		} catch (CORBA::NO_IMPLEMENT e) {
			cerr << "goInitial: NO_IMPLEMENT" << endl;
			throw;
		}
	} else if (isSingleArm()){
		RTC::JointPos jp;
		RTC::RETURN_ID * ret;

		try {
			DoubleSeq speed;
			CORBA_SeqUtil::push_back(speed, 10);
			ret = controllerRtc()->manipulator_motion()->setMaxSpeedJoint(speed);
		} catch (CORBA::NO_IMPLEMENT e) {
			cerr << "setSpeedJoint: NO_IMPLEMENT" << endl;
			throw;
		}
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		YamlNode& poseNode = PlanBase::instance()->body()->info()->get("standardPose");
		YamlSequence * seq = poseNode.toSequence();
		for (int i = 0; i < numJoints(); i++) {
			YamlNode& node = seq->get(i);
			CORBA_SeqUtil::push_back(jp, node.toDouble());
		}
#else
		ValueNode& poseNode = PlanBase::instance()->body()->info()->get("standardPose");
		Listing* seq = poseNode.toListing();
		for (int i = 0; i < numJoints(); i++) {
			ValueNode& node = seq->get(i);
			CORBA_SeqUtil::push_back(jp, node.toDouble());
		}
#endif
		try {
			ret = controllerRtc()->manipulator_motion()->movePTPJointAbs(jp);
		} catch (CORBA::NO_IMPLEMENT e) {
			cerr << "movePTPJointAbs: NO_IMPLEMENT" << endl;
			throw;
		}
	}
	// initialize the historical motion state
	// Note: this requires the initial robot state in the osgview
	// be the same as the home state of the real robot
	// TODO: feedback encoder value from the robot?
	phms->historicMS.clear();
	MotionState initialMS = pb->getMotionState();
	phms->historicMS.push_back(initialMS);
}

void RobotInterface::doOffPose() {
	try {
		controllerRtc()->manipulator()->goOffPose();
	} catch (CORBA::NO_IMPLEMENT e) {
		cerr << "goOffPose: NO_IMPLEMENT" << endl;
		throw;
	}
	// initialize the historical motion state
	// Note: this requires the initial robot state in the osgview
	// be the same as the home state of the real robot
	// TODO: feedback encoder value from the robot?
	phms->historicMS.clear();
}

void RobotInterface::doMove() {

		if(isSingleArm()){
				int error = moveSingleArm();
				if (error == EXIT_FAILURE)
					cout << "warning: moveSingleArm returns " << error << endl;
		}
		else if(isDualArm()){
			    int error = moveDualArm();
			    if (error == EXIT_FAILURE)
			    	cout << "warning: moveDualArm returns " << error << endl;
		}

}

void RobotInterface::doMove(const std::vector<MotionState>& motion) {

		if(isSingleArm()){
				cout << "not implemented " << endl;
		}
		else if(isDualArm()){
			    int error = moveDualArm(motion);
			    if (error == EXIT_FAILURE)
			    	cout << "warning: moveDualArm returns " << error << endl;
		}
		
}

/**
 * @brief HiroInterface::doSet
 * set the robot configuration the same to the sceneview
 */
void RobotInterface::doSet()
{
    // hiro is dual arm, we dont consider single-arm cases
    // move in 2 seconds
    MotionState lastMS = phms->historicMS.back();
    lastMS.motionTime = 2;
    MotionState updateMS = pb->getMotionState(lastMS.time+2);
    updateMS.motionTime = lastMS.motionTime+2;

    pb->graspMotionSeq.clear();
    pb->graspMotionSeq.push_back(lastMS);
    pb->graspMotionSeq.push_back(updateMS);
    int size = pb->graspMotionSeq.size();

    MotionCommands::DoubleSeq mtSeq;
    MotionCommands::JointPosSeq jpSeq;
    jpSeq.length(size);
    mtSeq.length(size);

    static const int angle_size = pb->graspMotionSeq[0].jointSeq.size(); //23;

    for(int i=0; i < size; i++){
        vector<double> angles(angle_size);
        convertAngles(pb->graspMotionSeq[i].jointSeq, angles);
        try {
            checkAngles(angles);
        } catch (std::vector<double> angles) {
            cout << "checkAngles failed " << endl;
            throw angles;
        }
        MotionCommands::JointPos jp(angle_size + 1);
        jp.length(angle_size + 1);
        for (int j = 0; j < angle_size; j++) {
            jp[j] = angles[j];
        }

        jpSeq[i] = jp;
        mtSeq[i] = pb->graspMotionSeq[i].motionTime;
        cout << "motion time(" << i << "):" << mtSeq[i] << endl;
    }
#ifdef WRITE_RESULTS
    writeResults();
#endif
    cout << "movePTPJointAbsSeq" << endl;
    try {
        MotionCommands::RETURN_ID * rid = controllerRtc()->motion()->movePTPJointAbsSeq(jpSeq, mtSeq);
        cout << "ret: " << rid->id << "; " << rid->comment << endl;
    } catch (CORBA::NO_IMPLEMENT e) {
        cerr << "setSoftLimitJoint: NO_IMPLEMENT" << endl;
        throw ;
    }

    phms->historicMS.push_back(updateMS);
}

void writeAngle(const vector<double>& angle, string f)
{
	string  File = "extplugin/graspPlugin/RobotInterface/" + f;
	ofstream fout(File.c_str());

	for(size_t i=0; i<angle.size()-1; i++)
		fout << angle[i]*180/3.14 << ", ";

	fout << angle.back()*180/3.14;
}

int RobotInterface::moveSingleArm()
{
	PlanBase* tc = PlanBase::instance();

	RETURN_ID * ret;

	int numFingJoint = PlanBase::instance()->bodyItemRobot()->body()->numJoints()-numJoints();
	vector<double> tmpAngle(numFingJoint);

	int seqSize=tc->graspMotionSeq.size();

	for(int n=0; n<seqSize; n++){
		try {
			DoubleSeq speed;
			CORBA_SeqUtil::push_back(speed, 20.0);
			ret = controllerRtc()->manipulator_motion()->setMaxSpeedJoint(speed);
			if (ret->id != EXIT_SUCCESS) {
				cout << "warning: setMaxSpeedJoint returns " << ret->id << ": " << ret->comment << endl;
				break;
			}
		} catch (CORBA::NO_IMPLEMENT e) {
			cerr << "setMaxSpeedJoint: NO_IMPLEMENT" << endl;
			throw;
		}
		RTC::JointPos jp;
		cout << "jp[" << n << "]";
		for (int i = 0; i < numJoints(); i++) {

			CORBA_SeqUtil::push_back(jp, tc->graspMotionSeq[n].jointSeq[i]);
			cout << jp[i] << ", ";
		}
		cout << endl;
		try {
			ret = controllerRtc()->manipulator_motion()->movePTPJointAbs(jp);
			if (ret->id != EXIT_SUCCESS) {
				cout << "warning: movePTPJointAbs returns " << ret->id << ": " << ret->comment << endl;
				break;
			}
		} catch (CORBA::NO_IMPLEMENT e) {
			cerr << "movePTPJointAbs: NO_IMPLEMENT" << endl;
			throw;
		}
		usleep(1300000);

		for(int i=0; i<numFingJoint; i++)
			tmpAngle[i] = tc->graspMotionSeq[n].jointSeq[numJoints()+i];


		try {
			if (tc->graspMotionSeq[n].graspingState == tc->GRASPING) {

				writeAngle(tmpAngle, tc->bodyItemRobot()->body()->name()+"Provider/close.dat");
				ret = controllerRtc()->manipulator_motion()->closeGripper();
			} else {

				writeAngle(tmpAngle, tc->bodyItemRobot()->body()->name()+"Provider/open.dat");
				ret = controllerRtc()->manipulator_motion()->openGripper();
			}

			if (ret->id != EXIT_SUCCESS) {
				cout << "warning: open/close Gripper returns " << ret->id << ": " << ret->comment << endl;
				break;
			}
		} catch (CORBA::NO_IMPLEMENT e) {
			cerr << "open/close Gripper: NO_IMPLEMENT" << endl;
			throw;
		}

	}

	try {
		//ret = controllerRtc()->manipulator_motion()->openGripper();
		ret = controllerRtc()->manipulator_motion()->resume();

		if (ret->id != EXIT_SUCCESS) {
			cout << "warning: open Gripper returns " << ret->id << ": " << ret->comment << endl;
		}
	} catch (CORBA::NO_IMPLEMENT e) {
		cerr << "openGripper: NO_IMPLEMENT" << endl;
		throw;
	}

	return ret->id;
}

void RobotInterface::writeResults()
{
	PlanBase* tc = PlanBase::instance();
	string  File = "./../../harada/hiro.pos";
	ofstream fout(File.c_str());

	int size = tc->graspMotionSeq.size();

	int angle_size = tc->graspMotionSeq[0].jointSeq.size()-6;

	double cur_time = 0.0;
	for(int i=0; i < size; i++){
		cur_time += tc->graspMotionSeq[i].motionTime;

		fout << cur_time << " ";
		for(int j=0; j<angle_size; j++)
			fout << tc->graspMotionSeq[i].jointSeq[j]*180.0/3.14159 << " ";
		fout << endl;
	}
}

int RobotInterface::moveDualArm()
{

		PlanBase* tc = PlanBase::instance();
		int error = 0;
		MotionCommands::DoubleSeq mtSeq;
		int size = tc->graspMotionSeq.size();
		MotionCommands::JointPosSeq jpSeq;
		jpSeq.length(size);
		mtSeq.length(size);

		static const int angle_size = tc->graspMotionSeq[0].jointSeq.size(); //23;

		int state_old = tc->NOT_GRASPING;
		int state2_old = tc->NOT_GRASPING;

		for(int i=0; i < size; i++){
			vector<double> angles(angle_size);
			convertAngles(tc->graspMotionSeq[i].jointSeq, angles);
			try {
				checkAngles(angles);
			} catch (std::vector<double> angles) {
				cout << "checkAngles failed " << endl;
				throw angles;
			}
			MotionCommands::JointPos jp(angle_size + 1);
			jp.length(angle_size + 1);
			for (int j = 0; j < angle_size; j++)
				jp[j] = angles[j];

			jpSeq[i] = jp;
			mtSeq[i] = tc->graspMotionSeq[i].motionTime;
			cout << "motion time(" << i << "):" << mtSeq[i] << endl;
#ifdef HAND_SEPARATE

			MotionCommands::DoubleSeq mtSeq_tmp;
			MotionCommands::JointPosSeq jpSeq_tmp;
			jpSeq_tmp.length(1);
			mtSeq_tmp.length(1);
			jpSeq_tmp[0] = jp;
			mtSeq_tmp[0] = tc->graspMotionSeq[i].motionTime;

			//MotionCommands::RETURN_ID * rid1 = controllerRtc()->motion()->setSpeedJoint(tc->graspMotionSeq[i].motionTime);
			MotionCommands::RETURN_ID * rid1 = controllerRtc()->motion()->setMotionTime(tc->graspMotionSeq[i].motionTime);
			MotionCommands::RETURN_ID * rid2 = controllerRtc()->motion()->movePTPJointAbs(jp );
			//MotionCommands::RETURN_ID * rid2 = controllerRtc()->motion()->movePTPJointAbsSeq(jpSeq_tmp, mtSeq_tmp);
			cout << "ret: " << rid2->id << "; " << rid2->comment <<  " motion time(" << i << "):" << tc->graspMotionSeq[i].motionTime << " GS (" << tc->graspMotionSeq[i].graspingState << ", " << tc->graspMotionSeq[i].graspingState2 << ")" << endl;

			int gstate = tc->graspMotionSeq[i].graspingState;
			int gstate2 = tc->graspMotionSeq[i].graspingState2;

			bool is_suction = (tc->nFing(0) == 0);
			bool is_suction2 = (tc->nFing(1) == 0);

			if (gstate == tc->GRASPING)
				controllerRtc()->manipulator()->rhandClose();
			else if (gstate == tc->UNDER_GRASPING && state_old == tc->NOT_GRASPING)
				controllerRtc()->manipulator()->rhandClose();
			else if ((!is_suction) && gstate == tc->UNDER_GRASPING && state_old == tc->GRASPING)
				controllerRtc()->manipulator()->rhandOpen();
			else if (is_suction && gstate == tc->UNDER_GRASPING && state_old == tc->UNDER_GRASPING)
				controllerRtc()->manipulator()->rhandOpen();
			else if ( gstate == tc->NOT_GRASPING)
				controllerRtc()->manipulator()->rhandOpen();

			if (gstate2 == tc->GRASPING)
				controllerRtc()->manipulator()->lhandClose();
			else if (gstate2 == tc->UNDER_GRASPING && state2_old == tc->NOT_GRASPING)
				controllerRtc()->manipulator()->lhandClose();
			else if ((!is_suction2) && gstate2 == tc->UNDER_GRASPING && state2_old == tc->GRASPING)
				controllerRtc()->manipulator()->lhandOpen();
			else if (is_suction2 && gstate2 == tc->UNDER_GRASPING && state2_old == tc->UNDER_GRASPING)
				controllerRtc()->manipulator()->lhandOpen();
			else if ( gstate == tc->NOT_GRASPING)
				controllerRtc()->manipulator()->lhandOpen();

			state_old = tc->graspMotionSeq[i].graspingState;
			state2_old = tc->graspMotionSeq[i].graspingState2;
#endif

		}
#ifdef WRITE_RESULTS
		writeResults();
#endif
#ifndef HAND_SEPARATE
		cout << "movePTPJointAbsSeq" << endl;
		try {
			MotionCommands::RETURN_ID * rid = controllerRtc()->motion()->movePTPJointAbsSeq(jpSeq, mtSeq);
			cout << "ret: " << rid->id << "; " << rid->comment << endl;
		} catch (CORBA::NO_IMPLEMENT e) {
			cerr << "setSoftLimitJoint: NO_IMPLEMENT" << endl;
			throw ;
		}
#endif
		return error;
}

int RobotInterface::moveDualArm(const std::vector<MotionState>& motion)
{

		PlanBase* tc = PlanBase::instance();
		int error = 0;
		MotionCommands::DoubleSeq mtSeq;
		int size = motion.size();
		MotionCommands::JointPosSeq jpSeq;
		jpSeq.length(size);
		mtSeq.length(size);

		static const int angle_size = motion[0].jointSeq.size(); //23;

		int state_old = tc->NOT_GRASPING;
		int state2_old = tc->NOT_GRASPING;

		for(int i=0; i < size; i++){
			vector<double> angles(angle_size);
			convertAngles(motion[i].jointSeq, angles);
			try {
				checkAngles(angles);
			} catch (std::vector<double> angles) {
				cout << "checkAngles failed " << endl;
				throw angles;
			}
			MotionCommands::JointPos jp(angle_size + 1);
			jp.length(angle_size + 1);
			for (int j = 0; j < angle_size; j++)
				jp[j] = angles[j];

			jpSeq[i] = jp;
			mtSeq[i] = motion[i].motionTime;
			cout << "motion time(" << i << "):" << mtSeq[i] << endl;
#ifdef HAND_SEPARATE
			//MotionCommands::RETURN_ID * rid1 = controllerRtc()->motion()->setSpeedJoint(motion[i].motionTime);
			MotionCommands::RETURN_ID * rid1 = controllerRtc()->motion()->setMotionTime(tc->graspMotionSeq[i].motionTime);
			MotionCommands::RETURN_ID * rid2 = controllerRtc()->motion()->movePTPJointAbs(jp );
			cout << "ret: " << rid2->id << "; " << rid2->comment <<  " motion time(" << i << "):" << motion[i].motionTime << " GS (" << motion[i].graspingState << ", " << motion[i].graspingState2 << ")" << endl;

			int gstate = motion[i].graspingState;
			int gstate2 = motion[i].graspingState2;

			if (gstate == tc->GRASPING || gstate == tc->UNDER_GRASPING)
				controllerRtc()->manipulator()->rhandClose();
			else if( gstate == tc->NOT_GRASPING)
				controllerRtc()->manipulator()->rhandOpen();

			if (gstate2 == tc->GRASPING)
				controllerRtc()->manipulator()->lhandClose();
			else if (gstate2 == tc->UNDER_GRASPING && state2_old == tc->NOT_GRASPING)
				controllerRtc()->manipulator()->lhandClose();
			else if (gstate2 == tc->UNDER_GRASPING && state2_old == tc->GRASPING)
				controllerRtc()->manipulator()->lhandOpen();
			else if(gstate2 == tc->NOT_GRASPING)
				controllerRtc()->manipulator()->lhandOpen();

			state_old = motion[i].graspingState;
			state2_old = motion[i].graspingState2;
#endif

		}
// #ifdef WRITE_RESULTS
// 		writeResults();
// #endif
#ifndef HAND_SEPARATE
		cout << "movePTPJointAbsSeq" << endl;
		try {
			MotionCommands::RETURN_ID * rid = controllerRtc()->motion()->movePTPJointAbsSeq(jpSeq, mtSeq);
			cout << "ret: " << rid->id << "; " << rid->comment << endl;
		} catch (CORBA::NO_IMPLEMENT e) {
			cerr << "setSoftLimitJoint: NO_IMPLEMENT" << endl;
			throw ;
		}
#endif
		return error;
}

::ArmController* RobotInterface::controllerRtc() {
		return ArmControllerRtc::instance()->comp_;
}

void RobotInterface::convertAngles(const VectorXd & seq, vector<double> & angles) {

	const double o = 180.0/3.14159;

	const int asize = (int)angles.size();
	for (int i = 0; i < asize; i++)
		angles[i] = seq(i)*o;

}

void RobotInterface::checkAngles(vector<double> & angles) {
	const int asize = (int)angles.size();
	for (int i = 0; i < asize; i++) {
		if (angles[i] > 360.0) {
			throw angles;
		}
	}
}

/*
int RobotInterface::objName2Id(string objname, string obj_data){

		std::ifstream fin_obj(obj_data.c_str());

		if(!fin_obj){
			cout << obj_data <<  " not found" << endl;
			return -1;
		}

		while(fin_obj){
			string tmp_obj;
			stringstream li2;
			string line2;
			int qtmp;


			getline(fin_obj,line2);
			li2 << line2;

			li2 >> qtmp;
			li2 >> tmp_obj;

			if(tmp_obj.find(objname,0) != string::npos)
				return qtmp;
		}
		return -1;
}
*/

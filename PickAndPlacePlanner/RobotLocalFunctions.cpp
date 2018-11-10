// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#include <exception>
#include "RobotLocalFunctions.h"
#include <ctime>

using namespace std;
using namespace cnoid;
using namespace grasp;
using namespace grasp::PickAndPlacePlanner;

#define m_pi (3.141592)

#if (defined(WIN32) || defined(_WIN32))
#include <windows.h>
void usleep(int t){
	Sleep(t/1000);
}
#endif

RobotLocalFunctions::RobotLocalFunctions() {
}

RobotLocalFunctions::~RobotLocalFunctions() {
}

RobotLocalFunctions* RobotLocalFunctions::instance() {
		static RobotLocalFunctions* instance = new RobotLocalFunctions();
		return instance;
}

void RobotLocalFunctions::Calibration(int robot) {


		string datafile;
		double t;
		int off;

		if(robot==HIRO || robot==HRP2){
//#define KINECT
#define SENRYAKU
#if defined(KINECT)
				datafile = "extplugin/graspPlugin/PickAndPlacePlanner/PRM/calib_HIRO.dat";

				ifstream fp(datafile.c_str()) ;

				while(!fp.eof()){

						for(int i=off; i<PlanBase::instance()->arm()->arm_path->numJoints(); i++){
								fp >> t;
                                PlanBase::instance()->arm()->arm_path->joint(i)->q() = t*m_pi/180.0;
						}
						for(int j=0; j<2; j++)
								for(int i=0; i<PlanBase::instance()->fingers(j)->fing_path->numJoints(); i++){
										fp >> t;
                                        PlanBase::instance()->fingers(j)->fing_path->joint(i)->q() = t*m_pi/180.0;
								}


						PlanBase::instance()->bodyItemRobot()->body()->calcForwardKinematics();
						PlanBase::instance()->flush();

						double x = -0.0299071;
						double y = -0.0153871;
						double z =  0.0110609;
						Vector3 offset(x,y,z);
						cout << "marker pos = ";
                        cout << Vector3(PlanBase::instance()->fingers(1)->fing_path->joint(1)->p() + PlanBase::instance()->fingers(1)->fing_path->joint(1)->attitude()*Vector3(x,y,z)).transpose() << endl;
						usleep(100000);
				}
#elif defined(SENRYAKU)
				datafile = "extplugin/graspPlugin/PickAndPlacePlanner/PRM/calib_HIRO3.dat";

				ifstream fp(datafile.c_str()) ;

				Vector3 off(-0.045, -0.0048, 0.01);

				int j=0;
				while(!fp.eof()){
					for(int i=0; i<PlanBase::instance()->bodyItemRobot()->body()->numJoints(); i++){
						fp >> t;
						PlanBase::instance()->bodyItemRobot()->body()->joint(i)->q() = t*3.14159/180.0;
					}
					PlanBase::instance()->bodyItemRobot()->body()->calcForwardKinematics();

					PlanBase::instance()->flush();
					usleep(5000000);

					Vector3 P = PlanBase::instance()->palm(1)->p() + PlanBase::instance()->palm(1)->attitude()*off;
					Matrix3 R = PlanBase::instance()->palm(1)->attitude();

					cout << "# " << j << "-th posture:" << endl;
					for(int k=0; k<3; k++)
						cout << R(k,0) << " " << R(k,1) << " " << R(k,2) << " " << P(k) << endl;
					cout << "0 0 0 1" << endl;
					j++;
				}

#else

				datafile = "extplugin/graspPlugin/PickAndPlacePlanner/PRM/calib_HIRO2.dat";

				ifstream fp(datafile.c_str()) ;
				Vector3 pos, rpy;
				off=1;

				while(!fp.eof()){
						for(int i=0; i<3; i++){
								fp >> t;
								pos(i) = t;
						}
						for(int i=0; i<3; i++){
								fp >> t;
								rpy(i) = t*m_pi/180.0;
						}
						Matrix3 rot=rotFromRpy(rpy);

						if(!PlanBase::instance()->arm()->IK_arm(pos, PlanBase::instance()->arm()->palm->calcRfromAttitude(rot)))
								cout << "IK was not solved" << endl;

						if(!PlanBase::instance()->arm()->checkArmLimit())
								cout << "Joint limit error" << endl;

						double x=0,y=0,z=0;
						if(robot==HIRO){

								cout << "timeToCalib" << off << " = 5.0" << endl;
								cout << "calibPose" << off << " = [" << endl;
                                cout << "[ " << PlanBase::instance()->arm()->arm_path->joint(0)->q()*180.0/m_pi << ", " << -PlanBase::instance()->arm()->arm_path->joint(0)->q()*180.0/m_pi << ", 60], " << endl;
								cout << "[ " << PlanBase::instance()->arm()->arm_path->joint(1)->q()*180.0/m_pi << ", "
									 << PlanBase::instance()->arm()->arm_path->joint(2)->q()*180.0/m_pi << ", "
									 << PlanBase::instance()->arm()->arm_path->joint(3)->q()*180.0/m_pi << ", "
									 << PlanBase::instance()->arm()->arm_path->joint(4)->q()*180.0/m_pi << ", "
									 << PlanBase::instance()->arm()->arm_path->joint(5)->q()*180.0/m_pi << ", "
                                     << PlanBase::instance()->arm()->arm_path->joint(6)->q()*180.0/m_pi << "], " << endl;
								cout << "[ 30, 0, -143.2,  0,  0,  0]," << endl;
								cout << "[0, 0, 0, 0]," << endl;
								cout << "[ 0, 0, 0, 0] "<< endl;
								cout << "]" << endl;
								cout << endl;

								for(int j=0; j<2; j++)
										for(int i=0; i<PlanBase::instance()->fingers(j)->fing_path->numJoints(); i++)
                                                PlanBase::instance()->fingers(j)->fing_path->joint(i)->q() = 0.0;

								PlanBase::instance()->bodyItemRobot()->body()->calcForwardKinematics();

								x = -0.030; //-0.0322 + (0.0365*0.5-0.0155);
								y =  0.010; //0.0005 + 0.021*0.5;
                                cout << "marker pos = " << Vector3(PlanBase::instance()->fingers(1)->fing_path->joint(0)->p() + PlanBase::instance()->fingers(1)->fing_path->joint(0)->attitude()*Vector3(x,y,z)).transpose() << endl;
						}
						else if(robot==HRP2){

								cout << "Joint angles ";
								for(int i=0; i<PlanBase::instance()->bodyItemRobot()->body()->numJoints(); i++)
                                        cout <<  PlanBase::instance()->bodyItemRobot()->body()->joint(i)->q()*180/m_pi << " ";
								cout << endl;

								PlanBase::instance()->bodyItemRobot()->body()->calcForwardKinematics();

                                Vector3 Ph = PlanBase::instance()->bodyItemRobot()->body()->link(16)->p();
								Matrix3 Rh = PlanBase::instance()->bodyItemRobot()->body()->link(16)->attitude();
                                Vector3 Pw = PlanBase::instance()->arm()->palm->p();
								Matrix3 Rw = PlanBase::instance()->arm()->palm->attitude();

								x = 0.045;
								z = -0.14;

								cout << "marker pos " << Vector3(Rh.transpose()*(Pw - Ph + Rw*Vector3(x,y,z))).transpose() << endl;
						}

						PlanBase::instance()->flush();
						usleep(100000);

						off++;
				}
#endif
		}
		else {
				datafile = "extplugin/graspPlugin/PickAndPlacePlanner/PRM/calib_PA10.dat";
				off=0;

				ifstream fp(datafile.c_str()) ;

				while(!fp.eof()){

						for(int i=off; i<PlanBase::instance()->arm()->arm_path->numJoints(); i++){
								fp >> t;
								//cout << t << " ";
                                PlanBase::instance()->arm()->arm_path->joint(i)->q() = t*m_pi/180.0;
						}

						//cout << endl;

						PlanBase::instance()->bodyItemRobot()->body()->calcForwardKinematics();
						PlanBase::instance()->flush();

						double x = -0.03;
						double z = 0.056;
						Vector3 offset(x,0,z);
						cout << "marker pos = ";
                        cout << Vector3( PlanBase::instance()->arm()->arm_path->joint(6)->p() + PlanBase::instance()->arm()->arm_path->joint(6)->attitude()*offset ).transpose() << endl;
						usleep(100000);
						//cout << PlanBase::instance()->arm()->arm_path->joint(6)->attitude() << endl;
				}
		}
}

void RobotLocalFunctions::clearSeq() {

	PlanBase* tc = PlanBase::instance();

	tc->jointSeq.clear();
	tc->graspingStateSeq.clear();
	tc->graspingStateSeq2.clear();
	tc->pathPlanDOFSeq.clear();
	tc->objectContactStateSeq.clear();
	tc->setTrajectoryPlanDOF(1);
	tc->objectPalmPosSeq.clear();
	tc->objectPalmRotSeq.clear();
	tc->motionTimeSeq.clear();
}

void RobotLocalFunctions::setSeq(const VectorXd& jointSeq, int graspingState, int graspingState2, int contactState, double mtime){

	PlanBase* tc = PlanBase::instance();

	tc->jointSeq.push_back(jointSeq);
	tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
	tc->objectPalmPosSeq.push_back(tc->armsList[1]->objectPalmPos);
	tc->objectPalmRotSeq.push_back(tc->armsList[1]->objectPalmRot);
	tc->graspingStateSeq.push_back(graspingState);
	tc->graspingStateSeq2.push_back(graspingState2);
	tc->objectContactStateSeq.push_back(contactState);
	tc->motionTimeSeq.push_back(mtime);
}

#ifndef EXCADE_ROBOTINTERFACE_MAKE_DLL
void RobotLocalFunctions::insertPipe() {

	Vector3 offset(0,0,-0.1);
	Vector3 app(0.0, 0.05, 0);

	PlanBase* tc = PlanBase::instance();

	//====== Initialization
	for (int n = 0; n < tc->bodyItemRobot()->body()->numJoints(); n++)
			tc->bodyItemRobot()->body()->joint(n)->q() = tc->jointSeq.back()(n);
	tc->bodyItemRobot()->body()->calcForwardKinematics();

	Vector3 P_ini = tc->palm(1)->p();
	Matrix3 R_ini = tc->palm(1)->R(); //attitude();

//#define WITH_VISION
#ifdef WITH_VISION
	ObjectPosReader opr;
	opr.doReadFromFile("extplugin/graspPlugin/RobotInterface/data/data_cap_2.mat", "extplugin/graspPlugin/PCL/calibtools/calibmat_2.txt", 0);

	Vector3 Po_cur = tc->objVisPos();
	Matrix3 Ro_cur = tc->objVisRot();
#endif

	vector<Vector3> Po_put;
	vector<Matrix3> Ro_put;
	Vector3 Zr = Vector3::Zero();
	Matrix3 Id = Matrix3::Identity();

	PlacePlanner::instance()->calcPutPos(Zr, Id, Po_put, Ro_put);

#ifndef WITH_VISION
	Vector3 Po_cur = Po_put[0];
	Matrix3 Ro_cur = Ro_put[0];
#endif

	tc->setGraspingStateMainArm(PlanBase::NOT_GRASPING);
	tc->object()->R() = Ro_cur;
    tc->object()->p() = Po_cur;
	tc->setGraspingStateMainArm(PlanBase::GRASPING);

	Po_put[0] += offset;

	//===== Initial Point
	VectorXd jointSeq_ini = tc->jointSeq.back();

	clearSeq();

	setSeq(jointSeq_ini, PlanBase::GRASPING, PlanBase::NOT_GRASPING, PlanBase::OFF_ENVIRONMENT, 10);

	//==== Next target
	Vector3 P_off = trans(Ro_cur)*(P_ini-Po_cur);
	Matrix3 R_off = trans(Ro_cur)*R_ini;

	Vector3 P_next = Po_put[0] + Ro_put[0]*P_off + app;
	Matrix3 R_next = Ro_put[0] * R_off;

	tc->arm(1)->IK_arm(P_next,R_next);

	VectorXd jointSeq(tc->bodyItemRobot()->body()->numJoints());

	for (int n = 0; n < tc->bodyItemRobot()->body()->numJoints(); n++)
			jointSeq(n) = tc->bodyItemRobot()->body()->joint(n)->q();

	setSeq(jointSeq, PlanBase::GRASPING, PlanBase::NOT_GRASPING, PlanBase::OFF_ENVIRONMENT, 10);


	//==== Inserting till target is reached
	P_next = Po_put[0] + Ro_put[0]*P_off;
	R_next = Ro_put[0] * R_off;

	tc->arm(1)->IK_arm(P_next,R_next);

	for (int n = 0; n < tc->bodyItemRobot()->body()->numJoints(); n++)
			jointSeq(n) = tc->bodyItemRobot()->body()->joint(n)->q();

	setSeq(jointSeq, PlanBase::UNDER_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, 10);

	//==== Finger Open
	for(int i=0; i<tc->nFing(1); i++)
		for(int j=0; j<tc->fingers(1,i)->fing_path->numJoints(); j++)
			jointSeq(tc->fingers(1,i)->joint(j)->jointId()) = tc->fingers(1,i)->fingerOpenPose[j];

	setSeq(jointSeq, PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, 2);

	for (int n = 0; n < tc->bodyItemRobot()->body()->numJoints(); n++)
			tc->bodyItemRobot()->body()->joint(n)->q() = tc->jointSeq.front()(n);
	tc->bodyItemRobot()->body()->calcForwardKinematics();
	tc->targetObject->bodyItemObject->body()->link(0)->attitude() = Ro_cur;
    tc->targetObject->bodyItemObject->body()->link(0)->p()        = Po_cur+offset;
    tc->setObjPos(Po_cur+offset, Ro_cur);


    return;
}
#endif

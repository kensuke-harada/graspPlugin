#ifndef _ROBOTINTERFACE_H
#define _RROBOTINTERFACE_H

#include <stdio.h>
#include <fstream>
#include <iostream>
#ifndef _WIN32
#include <unistd.h>
#endif

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/ItemTreeView>
#include <cnoid/RootItem>
#include <cnoid/MessageView>
#include <cnoid/EigenUtil>
#include "../Grasp/GraspController.h"
#include "../PickAndPlacePlanner/RobotLocalFunctions.h"
#include "HistoricMotionState.h"
#include "exportdef.h"

class ArmController;

namespace grasp{

class EXCADE_API RobotInterface {

	public :

	RobotInterface();
	virtual ~RobotInterface(){};
/*
	static RobotInterface* instance(){
		static RobotInterface* instance = new RobotInterface();
		return instance;
	}
*/
	static RobotInterface* instance( RobotInterface* ri=NULL )
	{
		static RobotInterface* instance = (ri) ? ri : new RobotInterface();
		if(ri) instance = ri;
		return instance;
	};

	virtual bool runPythonCommand(std::string func);

	virtual void doReadFromFile(int num=0);
	virtual void doRecoginitionAndRead(int num=0);
	virtual void doCapture();
	virtual void doJntCalib();
	virtual void doSrvOn();
	virtual void doSrvOff();
	virtual void doHome();
	virtual void doOffPose();
	virtual void doMove();
	virtual void doSet();
	virtual void doMove(const std::vector<MotionState>& motion);

	bool isMulti;

	protected :

	//virtual int objName2Id(std::string objname, std::string obj_data);
	//bool moveUp;

	private :
		bool isSingleArm() { return PlanBase::instance()->armsList.size() == 1; }
		bool isDualArm() { return PlanBase::instance()->armsList.size() >= 2; }
		int moveDualArm();
		int moveSingleArm();
		int moveDualArm(const std::vector<MotionState>& motion);
		::ArmController * controllerRtc();
		int numJoints() { return PlanBase::instance()->arm()->arm_path->numJoints(); }
		//void writeJointSeq(char * jointlogfile, const unsigned int filelength, const std::vector<cnoid::VectorXd>& jointSeq, std::vector<double>& motionTimeSeq);
		void convertAngles(const cnoid::VectorXd & seq, std::vector<double> & angles);
		void checkAngles(std::vector<double> & angles);


		void writeResults();
		cnoid::Matrix3 R1, R2, R3;
		cnoid::Vector3 t1, t2, t3;
		void setTransMatrix();

		HistoricMotionState* phms;
		PlanBase* pb;
};



}


#endif

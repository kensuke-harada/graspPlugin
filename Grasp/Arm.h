/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _ARM_H
#define _ARM_H

#include <stdlib.h>
#include <time.h>
#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/JointPath>

#include "exportdef.h"
#include "InterLink.h"
#include "ColdetLinkPair.h"
#include "ObjectBase.h"

namespace grasp{

class Arm;
typedef Arm* ArmPtr;

class EXCADE_API Arm{
	public:
		Arm(cnoid::BodyPtr body, cnoid::Link *base, cnoid::Link *palm);
		virtual ~Arm(){};
		cnoid::JointPathPtr arm_path;
		cnoid::Link *palm;
		int nJoints;

		std::vector<double> armStandardPose, armFinalPose;
		std::vector<double> regraspPoseRel[6];
		cnoid::Vector3 base_p;
		cnoid::Matrix3 base_R;

		virtual bool IK_arm(const cnoid::Vector3 &p, const cnoid::Matrix3 &R);
		virtual bool IK_arm(const cnoid::Vector3& p, const cnoid::Matrix3& R, const cnoid::VectorXd& q_old);
		virtual bool IK_arm(const cnoid::Vector3& p, const cnoid::Matrix3& R, double phi, const cnoid::VectorXd& q_old=cnoid::VectorXd::Zero(7));
		virtual bool getPalmPos(const cnoid::Vector3& Pco1, const cnoid::Vector3& Pco2, const cnoid::Matrix3& Rp, const cnoid::Vector3& pPcr1, const cnoid::Matrix3& pRcr1, cnoid::Vector3& Pp, cnoid::VectorXd& theta, double offset=0.0);
		virtual bool getPalmPos(const cnoid::Vector3& Pco1, const cnoid::Vector3& Pco2, const cnoid::Matrix3& Rp, const cnoid::Vector3& pPcr1, const cnoid::Matrix3& pRcr1, cnoid::Vector3& Pp, cnoid::VectorXd& theta, const std::vector<double>& offset);
		virtual bool IK_arm_palm(const cnoid::Vector3 &palm_p, const cnoid::Matrix3 &palm_R);
		virtual bool IK_arm_palm(const cnoid::Vector3& p, const cnoid::Matrix3& R, const cnoid::VectorXd& q_old);
		virtual bool IK_arm_palm(const cnoid::Vector3& p, const cnoid::Matrix3& R, double phi, const cnoid::VectorXd& q_old=cnoid::VectorXd::Zero(7));

//		bool IK_arm2(const Vector3 &p, const Matrix3 &R);
		virtual double IndexFunc(double a, double b);
		virtual cnoid::VectorXd calcGradient(double a, double b);
		virtual bool checkArmLimit();
		virtual double Manipulability();
		virtual double avoidAngleLimit();
		virtual double avoidAngleLimit2();
		virtual void calcJacobian(cnoid::MatrixXd& J);

		virtual bool closeArm(int lk, int iter, cnoid::Vector3 &oPos, cnoid::Vector3 &objN);

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
        cnoid::ColdetLinkPairPtr palmObjPair;
#else
        grasp::ColdetLinkPairPtr palmObjPair;
#endif
		bool palmContact;
		cnoid::Vector3 closeDir;
		cnoid::Vector3 approachOffset;

		bool searchBasePositionMode;

		void setPairs(const std::vector<InterLink>& interLinkList); // set pairs if you use multithread
		std::vector<InterLink> interLinks; // set interLinks if you use multithread
		bool multithread_mode;
		void setInterLink();
		std::vector<InterLink> pairs;
		bool isInterLink;

		cnoid::Vector3 toolWristPos;
		cnoid::Matrix3 toolWristRot;

		// The three GraspingStates are used to switch collision detections.
		// Especially, under_grasping means the object is in the palm,
		// although not grasped by the fingers
		enum GraspingStates { NOT_GRASPING, UNDER_GRASPING, GRASPING };
		int graspingState;
		ObjectID target_grasp_objid;
};

}

#endif
